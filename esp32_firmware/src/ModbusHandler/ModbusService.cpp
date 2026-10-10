#include "ModbusService.h"

// Set logging tag
static const char *TAG = "MODBUS";


ModbusService::ModbusService(DS18B20Manager &ds,
                             GPIOManager &gpio,
                             SlotConfigManager &slotCfg)
    : _ds(ds), _gpio(gpio), _slotCfg(slotCfg) {}

void ModbusService::begin()
{
  ESP_LOGI(TAG, "initializing ModBus registers, coils and RTU");
  // Modbus RTU (over USB Serial)
  MODBUS_SERIAL.begin(MODBUS_BAUD, MODBUS_CONFIG);
  _mb.begin(&MODBUS_SERIAL);

  _mb.server(MODBUS_SLAVE_ID);

  // Declare Holding Registers
  _mb.addHreg(REG_SENSOR_COUNT,0,1);
  _mb.addHreg(REG_TEMPS_BASE, 0, REG_TEMPS_COUNT);


  // Logical Slot ROM config area
  _mb.addHreg(REG_ROM_BASE,0,DS_MAX_SLOTS * REG_ROM_STRIDE);


  // Control registers
  _mb.addHreg(REG_APPLY_MAPPING);
  _mb.addHreg(REG_CLEAR_CONFIG);

  // Coils for GPIOs
  _mb.addCoil(0, 0, NUM_GPIO_OUTPUTS);

  // Present ROM table: count + (MAX_DISC * 4 words)
  _mb.addHreg(REG_PRESENT_BASE); // PRESENT_COUNT
  _mb.addHreg(REG_PRESENT_BASE + 1, 0, DS18B20Manager::MAX_DISC * 4);

  // Seed ROM HRs with what is in SlotConfigManager
  ESP_LOGI(TAG, "load saved sensor slot/rom mapping");
  refreshRomHRs();


  // ---------------------------
  // Write callbacks (library expects cb: uint16_t(TRegister*, uint16_t))
  // ---------------------------

  // ROM area: accept writes to the 4-word blocks per slot (defer apply)
  _mb.onSetHreg(
      REG_ROM_BASE,
      [this](TRegister *reg, uint16_t val) -> uint16_t
      {
        // Just store to the local map; commit on APPLY_MAPPING
        reg->value = val;
        return val;
      },
      DS_MAX_SLOTS *REG_ROM_STRIDE);


  // APPLY_MAPPING: commit HR -> NVS, re-bind mapping
  _mb.onSetHreg(
      REG_APPLY_MAPPING,
      [this](TRegister *reg, uint16_t val) -> uint16_t
      {
        if (val == 0xA5A5)
        {
          ESP_LOGI(TAG, "applying new slot/rom mapping");
          // 1) Commit ROMs
          for (uint16_t s = 0; s < DS_MAX_SLOTS; s++) 
          {
            uint64_t rom = getSlotROM_fromHR(s);

            // Save ROM
            _slotCfg.setROM(s, rom);
            ESP_LOGD(TAG, "slot=%u rom=%016" PRIX64, s, rom);
          }

          // Commit new ROMs.
          _slotCfg.commit();

          reg->value = 0; // auto-clear
        }
        else
        {
          reg->value = val;
        }
        return val;
      },
      1);

  // CLEAR_CONFIG: wipe NVS and clear mapping
  _mb.onSetHreg(
      REG_CLEAR_CONFIG,
      [this](TRegister *reg, uint16_t val) -> uint16_t
      {
        if (val == 0xDEAD)
        {
          ESP_LOGI(TAG, "clearing stored slot/rom mappings");
          _slotCfg.clearAll();
          refreshRomHRs();
          reg->value = 0; // auto-clear
        }
        else
        {
          reg->value = val;
        }
        return val;
      },
      1);


  _mb.onSetCoil(
      0,
      [this](TRegister *reg, uint16_t val) -> uint16_t
      {
        const uint16_t idx = reg->address.address; // absolute coil index
        if (idx >= NUM_GPIO_OUTPUTS)
        {
          return Modbus::EX_ILLEGAL_ADDRESS; // 0x02
          ESP_LOGE(TAG, "coil idx (%d) out of bounds ", idx);
        }
        const bool on = (val != 0);                                // FC05 sends 0xFF00 for ON
        _gpio.setOutput(idx, on);                  // your bool-returning setter
        reg->value = on ? 1 : 0;                                   // library stores boolean (0/1)
        ESP_LOGD(TAG, "coil idx %d updated val=%d", idx, val);
        return val;
      },
      NUM_GPIO_OUTPUTS);
}

void ModbusService::startTask(UBaseType_t prio, uint32_t stack, BaseType_t core)
{
  xTaskCreatePinnedToCore(taskTrampoline, "ModbusTask", stack, this, prio, nullptr, core);
}

void ModbusService::taskTrampoline(void *arg)
{
  reinterpret_cast<ModbusService *>(arg)->taskLoop();
}

void ModbusService::taskLoop()
{
  int16_t count = 0; // count for LOGD output to not overwhelm the console

  while (true)
  {
    _mb.task();

    // Update read-only sensor data
    uint16_t pc = _ds.getPresentCount();

    _mb.Hreg(REG_SENSOR_COUNT, pc);
    for (uint16_t i = 0; i < REG_TEMPS_COUNT; i++) {
        int16_t temp = _ds.readSlotTemp_x100(i);
        _mb.Hreg(REG_TEMPS_BASE + i, (uint16_t)temp);
        if (count == 100) {
        LOGD(TAG, "temp read from slot=%d _ds=%d",i, temp);
        }
      }
    // Mirror Present ROMs (read-only discovery table)
    publishPresentROMs();

    count++;
    if (count > 100){count = 0;}
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void ModbusService::publishPresentROMs()
{
  uint16_t pc = _ds.getPresentCount();
  _mb.Hreg(REG_PRESENT_BASE, pc);


  // Write each present ROM
  for (uint16_t i = 0; i < pc; i++)
  {
    uint64_t rom;
    if (_ds.getPresentROM(i, rom))
    {
      uint16_t base = REG_PRESENT_BASE + 1 + i * 4;
      _mb.Hreg(base + 0, (uint16_t)(rom & 0xFFFF));
      _mb.Hreg(base + 1, (uint16_t)((rom >> 16) & 0xFFFF));
      _mb.Hreg(base + 2, (uint16_t)((rom >> 32) & 0xFFFF));
      _mb.Hreg(base + 3, (uint16_t)((rom >> 48) & 0xFFFF));
    }
  }
}

// ----------------------------
// Helpers
// ----------------------------
uint64_t ModbusService::getSlotROM_fromHR(uint16_t slot)
{
  uint16_t base = REG_ROM_BASE + slot * REG_ROM_STRIDE;
  uint64_t rom = 0;

  rom |= (uint64_t)_mb.Hreg(base + 0) << 0;
  rom |= (uint64_t)_mb.Hreg(base + 1) << 16;
  rom |= (uint64_t)_mb.Hreg(base + 2) << 32;
  rom |= (uint64_t)_mb.Hreg(base + 3) << 48;

  return rom;
}

void ModbusService::setSlotROM_intoHR(uint16_t slot, uint64_t rom)
{
  uint16_t base = REG_ROM_BASE + slot * REG_ROM_STRIDE;

  _mb.Hreg(base + 0, (uint16_t)(rom & 0xFFFF));
  _mb.Hreg(base + 1, (uint16_t)((rom >> 16) & 0xFFFF));
  _mb.Hreg(base + 2, (uint16_t)((rom >> 32) & 0xFFFF));
  _mb.Hreg(base + 3, (uint16_t)((rom >> 48) & 0xFFFF));
}

void ModbusService::refreshRomHRs() {
  for (uint16_t s = 0; s < DS_MAX_SLOTS; s++) {
    setSlotROM_intoHR(s, _slotCfg.getROM(s));
  }
}


