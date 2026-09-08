#include "ModbusService.h"

ModbusService::ModbusService(DS18B20Manager &ds,
                             GPIOManager &gpio,
                             ConfigStore &cfg)
    : _ds(ds), _gpio(gpio), _cfg(cfg) {}

void ModbusService::begin()
{
  //Serial.printf("[ModbusService] &_ds = %p\n", (void *)&_ds);
  // Load ROMs from NVS
  for (uint16_t i = 0; i < DS_MAX_SLOTS; i++)
  {
    uint64_t rom = 0;
    _cfg.loadROM(i, rom);
    _slotROM[i] = rom;

    // initialize probe offsets
    int16_t off = 0;
    _cfg.loadOffset(i, off); // default to 0 if not present
    _slotOffset_x100[i] = off;
    
  }
  _ds.applyMapping(_slotROM);
  _ds.applyOffsets(_slotOffset_x100);

  // Modbus RTU (over USB Serial)
  MODBUS_SERIAL.begin(MODBUS_BAUD, MODBUS_CONFIG);
  _mb.begin(&MODBUS_SERIAL);

  _mb.server(MODBUS_SLAVE_ID);

  // Declare Holding Registers
  _mb.addHreg(REG_SENSOR_COUNT,0,1);
  _mb.addHreg(REG_TEMPS_BASE, 0, REG_TEMPS_COUNT);
  _mb.addHreg(REG_TEMPS_RAW_BASE, 0, REG_TEMPS_COUNT);


  // Logical Slot ROM config area
  _mb.addHreg(REG_ROM_BASE,0,DS_MAX_SLOTS * REG_ROM_STRIDE);

  // Offset Holding Registers: one int16 per slot (°C x100)
  _mb.addHreg(REG_TEMPS_OFFSET_BASE, 0, DS_MAX_SLOTS);

  // Control registers
  _mb.addHreg(REG_APPLY_MAPPING);
  _mb.addHreg(REG_CLEAR_CONFIG);

  // Coils for GPIOs
  _mb.addCoil(0, 0, NUM_GPIO_OUTPUTS);

  // Present ROM table: count + (MAX_DISC * 4 words)
  _mb.addHreg(REG_PRESENT_BASE); // PRESENT_COUNT
  _mb.addHreg(REG_PRESENT_BASE + 1, 0, DS18B20Manager::MAX_DISC * 4);

  // Seed ROM HRs with what we loaded from NVS
  for (uint16_t s = 0; s < DS_MAX_SLOTS; s++) {
    setSlotROM_intoHR(s, _slotROM[s]);
    setSlotOffset_intoHR(s, _slotOffset_x100[s]);
  }
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

  _mb.onSetHreg(
      REG_TEMPS_OFFSET_BASE,
      [this](TRegister *reg, uint16_t val) -> uint16_t
      {
        // Accept writes; commit later on APPLY_MAPPING
        reg->value = val;
        return val;
      },
      DS_MAX_SLOTS);

  // APPLY_MAPPING: commit HR -> NVS, re-bind mapping
  _mb.onSetHreg(
      REG_APPLY_MAPPING,
      [this](TRegister *reg, uint16_t val) -> uint16_t
      {
        if (val == 0xA5A5)
        {
          // 1) Commit ROMs
          for (uint16_t s = 0; s < DS_MAX_SLOTS; s++) 
          {
            uint64_t rom = getSlotROM_fromHR(s);
            _slotROM[s] = rom;
            _cfg.saveROM(s, rom);
          }

          // 2) Commit Offsets (°C x100)
          for (uint16_t s = 0; s < DS_MAX_SLOTS; s++)
          {
            int16_t off = getSlotOffset_fromHR(s);
            _slotOffset_x100[s] = off;
            _cfg.saveOffset(s, off);
          }

          applyMappingFromConfig();
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
          _cfg.clearAll();
          for (uint16_t s = 0; s < DS_MAX_SLOTS; s++)
          {
            _slotROM[s] = 0;
            setSlotROM_intoHR(s, 0);
            setSlotOffset_intoHR(s, 0);
          }
          applyMappingFromConfig();
          reg->value = 0; // auto-clear
        }
        else
        {
          reg->value = val;
        }
        return val;
      },
      1);

  // Coils: keep hardware in sync with coil value
 // _mb.onSetCoil(
 //     0,
 //     [this](TRegister *reg, uint16_t val) -> uint16_t
 //     {
 //       // NOTE: reg->address is TAddress; numeric offset is reg->address.address
 //       // (Documented in library API; examples also print reg->address.address)
 //       uint16_t idx = reg->address.address; // offset was 0, so this equals absolute coil index
 //       bool on = (val != 0);
 //       if (idx < NUM_GPIO_OUTPUTS)
 //         _gpio.setOutput(idx, on);
 //       reg->value = on ? 0xFF00 : 0x0000;
 //       return 0;
 //     },
 //     NUM_GPIO_OUTPUTS);

  _mb.onSetCoil(
      0,
      [this](TRegister *reg, uint16_t val) -> uint16_t
      {
        const uint16_t idx = reg->address.address; // absolute coil index
        if (idx >= NUM_GPIO_OUTPUTS)
        {
          return Modbus::EX_ILLEGAL_ADDRESS; // 0x02
        }
        const bool on = (val != 0);                                // FC05 sends 0xFF00 for ON
        _gpio.setOutput(idx, on);                  // your bool-returning setter
        reg->value = on ? 1 : 0;                                   // library stores boolean (0/1)
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
  while (true)
  {
    _mb.task();

    // Update read-only sensor data
    uint16_t pc = _ds.getPresentCount();

    _mb.Hreg(REG_SENSOR_COUNT, pc);
    
    //Serial.println(_ds.getPresentCount());
    for (uint16_t i = 0; i < REG_TEMPS_COUNT; i++) {
        int16_t raw = _ds.readSlotRawTemp_x100(i);
        int16_t cor = _ds.readSlotTemp_x100(i);
        _mb.Hreg(REG_TEMPS_RAW_BASE + i, (uint16_t)raw);
        _mb.Hreg(REG_TEMPS_BASE + i, (uint16_t)cor);
    }
    //   // Mirror coil map to hardware outputs
    //   for (uint16_t i = 0; i < NUM_GPIO_OUTPUTS; i++)
    //   {
    //     bool v = _mb.Coil(i);  // read from server’s coil map (bool 0/1)
    //     _gpio.setOutput(i, v); // drive the pin
    //   }

    // Mirror Present ROMs (read-only discovery table)
    publishPresentROMs();

    // DEBUG: read back what the server stored and print it
//    uint16_t dbg0 = _mb.Hreg(REG_SENSOR_COUNT);
//    uint16_t dbg300 = _mb.Hreg(REG_PRESENT_BASE);
//    Serial.printf("[MB] HREG[0]=%u HREG[300]=%u (pc=%u)\n", dbg0, dbg300, pc);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void ModbusService::publishPresentROMs()
{
  uint16_t pc = _ds.getPresentCount();
  _mb.Hreg(REG_PRESENT_BASE, pc);

  // Clear full region (avoid stale values)
 // for (uint16_t i = 0; i < DS18B20Manager::MAX_DISC; i++)
 // {
 //   uint16_t base = REG_PRESENT_BASE + 1 + i * 4;
 //   _mb.Hreg(base + 0, 0);
 //   _mb.Hreg(base + 1, 0);
 //   _mb.Hreg(base + 2, 0);
 //   _mb.Hreg(base + 3, 0);
 // }

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

void ModbusService::applyMappingFromConfig()
{
  _ds.applyMapping(_slotROM);
  _ds.applyOffsets(_slotOffset_x100);
}

int16_t ModbusService::getSlotOffset_fromHR(uint16_t slot)
{
  const uint16_t addr = REG_TEMPS_OFFSET_BASE + slot;
  // Read back 16-bit register and interpret as signed
  return (int16_t)_mb.Hreg(addr);
}

void ModbusService::setSlotOffset_intoHR(uint16_t slot, int16_t off)
{
  const uint16_t addr = REG_TEMPS_OFFSET_BASE + slot;
  _mb.Hreg(addr, (uint16_t)off);
}
