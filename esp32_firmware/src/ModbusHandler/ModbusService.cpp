#include "ModbusService.h"

// Set logging tag
static const char *TAG = "MODBUS";
static uint16_t clampLevel(uint16_t v)
{
  return v > LOG_LEVEL_MAX ? LOG_LEVEL_MAX : v;
}

ModbusService::ModbusService(DS18B20Manager &ds,
                             GPIOManager &gpio,
                             SlotConfigManager &slotCfg)
    : _ds(ds), _gpio(gpio), _slotCfg(slotCfg) {}

void ModbusService::begin()
{
  LOGI(TAG, "initializing ModBus registers, coils and RTU");
  // Modbus RTU (over USB Serial)
  MODBUS_SERIAL.begin(MODBUS_BAUD, MODBUS_CONFIG);
  _mb.begin(&MODBUS_SERIAL);

  _mb.server(MODBUS_SLAVE_ID);

  setupLogRegisters();
  
  // Declare Holding Registers
  _mb.addHreg(REG_SENSOR_COUNT,0,1);
  _mb.addHreg(REG_TEMPS_BASE, 0, REG_TEMPS_COUNT);


  // Logical Slot ROM config area
  _mb.addHreg(REG_ROM_BASE,0,DS_MAX_SLOTS * REG_ROM_STRIDE);


  // Control registers
  _mb.addHreg(REG_SLOT_CMD, SLOT_CMD_IDLE);
  _mb.addHreg(REG_SLOT_CMD_STATUS, SLOT_STATUS_IDLE);

  // Coils for GPIOs
  _mb.addCoil(0, 0, NUM_GPIO_OUTPUTS);

  // Present ROM table: count + (MAX_DISC * 4 words)
  _mb.addHreg(REG_PRESENT_BASE); // PRESENT_COUNT
  _mb.addHreg(REG_PRESENT_BASE + 1, 0, DS18B20Manager::MAX_DISC * 4);

  // Seed ROM HRs with what is in SlotConfigManager
  LOGI(TAG, "load saved sensor slot/rom mapping");
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


  // control register for SLOT_CMD
  _mb.onSetHreg(
      REG_SLOT_CMD,
      [this](TRegister *reg, uint16_t val) -> uint16_t
      {
        handleSlotCmd(val);
        reg->value = 0;
        return val;
      },1);

  _mb.onSetCoil(
      0,
      [this](TRegister *reg, uint16_t val) -> uint16_t
      {
        const uint16_t idx = reg->address.address; // absolute coil index
        if (idx >= NUM_GPIO_OUTPUTS)
        {
          return Modbus::EX_ILLEGAL_ADDRESS; // 0x02
          LOGE(TAG, "coil idx (%d) out of bounds ", idx);
        }
        const bool on = (val != 0);                                // FC05 sends 0xFF00 for ON
        _gpio.setOutput(idx, on);                  // your bool-returning setter
        reg->value = on ? 1 : 0;                                   // library stores boolean (0/1)
        LOGD(TAG, "coil idx %d updated val=%d", idx, val);
        return val;
      },
      NUM_GPIO_OUTPUTS);
}

void ModbusService::setupLogRegisters()
{
  _mb.addHreg(REG_LOG_LEVEL_ALL, LOG_LEVEL_DEFAULT);
  esp_log_level_set("*", (esp_log_level_t)LOG_LEVEL_DEFAULT);

  for (uint16_t i = 0; i < LOG_TAG_COUNT; i++)
  {
    _mb.addHreg(REG_LOG_LEVEL_BASE + i, LOG_LEVEL_DEFAULT);
    esp_log_level_set(LOG_TAGS[i], (esp_log_level_t)LOG_LEVEL_DEFAULT);
  }

  // Set all tags
  _mb.onSetHreg(
      REG_LOG_LEVEL_ALL,
      [this](TRegister *reg, uint16_t val) -> uint16_t
      {
        uint16_t lvl = clampLevel(val);
        esp_log_level_set("*", (esp_log_level_t)lvl);
        for (uint16_t i = 0; i < LOG_TAG_COUNT; i++)
        {
          esp_log_level_set(LOG_TAGS[i], (esp_log_level_t)lvl);
          _mb.Hreg(REG_LOG_LEVEL_BASE + i, lvl); // keep per-tag read-back truthful
        }
        reg->value = lvl;
        return val;
      },
      1);

  // Per tag: one callback covers the whole block
  _mb.onSetHreg(
      REG_LOG_LEVEL_BASE,
      [](TRegister *reg, uint16_t val) -> uint16_t
      {
        uint16_t idx = reg->address.address - REG_LOG_LEVEL_BASE;
        uint16_t lvl = clampLevel(val);
        if (idx < LOG_TAG_COUNT)
          esp_log_level_set(LOG_TAGS[idx], (esp_log_level_t)lvl);
        reg->value = lvl;
        return val;
      },
      LOG_TAG_COUNT);
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

    if (_mb.Hreg(REG_SLOT_CMD_STATUS) == SLOT_STATUS_BUSY && _ds.scanDone())
    {
      setSlotStatus(SLOT_STATUS_DONE);
      _ds.clearScanDone();
    }

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

void ModbusService::setSlotStatus(uint16_t status)
{
  _mb.Hreg(REG_SLOT_CMD_STATUS, status);
}

uint16_t ModbusService::handleSlotCmd(uint16_t cmd)
{
  if (cmd == SLOT_CMD_IDLE)
  {
    setSlotStatus(SLOT_STATUS_IDLE); 
    return 0;
  }

  if (_mb.Hreg(REG_SLOT_CMD_STATUS) == SLOT_STATUS_BUSY)
  {
    LOGW(TAG, "slot cmd 0x%04X ignored, busy", cmd);
    return 0;
  }

  switch (cmd)
  {
  case SLOT_CMD_APPLY_MAPPING:
    LOGI(TAG, "applying new slot/rom mapping");
    for (uint16_t s = 0; s < DS_MAX_SLOTS; s++)
      _slotCfg.setROM(s, getSlotROM_fromHR(s));
    _slotCfg.commit();
    setSlotStatus(SLOT_STATUS_DONE);
    break;

  case SLOT_CMD_CLEAR_CONFIG:
    LOGI(TAG, "clearing stored slot/rom mappings");
    _slotCfg.clearAll();
    refreshRomHRs();
    setSlotStatus(SLOT_STATUS_DONE);
    break;

  case SLOT_CMD_SCAN:
    LOGI(TAG, "request bus scan");
    if (_ds.requestScan())
      setSlotStatus(SLOT_STATUS_BUSY);
    else
      setSlotStatus(SLOT_STATUS_ERROR);

    break;

  default:
    LOGW(TAG, "unknown slot cmd 0x%04X", cmd);
    setSlotStatus(SLOT_STATUS_ERROR);
    break;
  }

  return 0; // clear CMD on accept
}