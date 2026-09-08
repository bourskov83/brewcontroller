#pragma once
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "../config.h"

// Utilities to convert between 64-bit ROM and DallasTemperature DeviceAddress.
static inline void rom64_to_addr(uint64_t rom, DeviceAddress addr)
{
  for (int i = 0; i < 8; i++)
    addr[i] = (uint8_t)((rom >> (8 * i)) & 0xFF);
}
static inline uint64_t addr_to_rom64(const DeviceAddress addr)
{
  uint64_t rom = 0;
  for (int i = 0; i < 8; i++)
    rom |= ((uint64_t)addr[i]) << (8 * i);
  return rom;
}

class DS18B20Manager
{
public:
  DS18B20Manager();
  void begin();
  void startTask(UBaseType_t prio = 1,
                 uint32_t stack = 4096,
                 BaseType_t core = tskNO_AFFINITY);

  // Logical slots
  void applyMapping(const uint64_t slotROMs[DS_MAX_SLOTS]);

  void applyOffsets(const int16_t slotOffsets_x100[DS_MAX_SLOTS]); 

  int16_t readSlotTemp_x100(uint16_t slot) const;
  int16_t readSlotRawTemp_x100(uint16_t slot) const;

  // Present ROMs table (for Modbus discovery)
  uint16_t getPresentCount() const { return _presentCount; }
  bool getPresentROM(uint16_t idx, uint64_t &rom) const
  {
    if (idx >= _presentCount)
      return false;
    rom = _presentROMs[idx];
    return true;
  }
  static constexpr uint16_t MAX_DISC = 16;

private:
  static void taskTrampoline(void *);
  void taskLoop();

  void busScan();
  bool romPresent(uint64_t rom) const;

  OneWire _oneWire;
  DallasTemperature _sensors;

  // Discovered devices on bus
  uint64_t _presentROMs[MAX_DISC];
  uint16_t _presentCount = 0;

  // Logical ROM mapping
  uint64_t _slotROM[DS_MAX_SLOTS] = {};
  int16_t _slotOffset_x100[DS_MAX_SLOTS] = {0};
  int16_t _slotRawTemp_x100[DS_MAX_SLOTS] = {0}; 
  int16_t _slotTemp_x100[DS_MAX_SLOTS] = {0};
};