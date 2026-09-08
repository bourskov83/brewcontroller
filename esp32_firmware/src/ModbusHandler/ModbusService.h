#pragma once
#include <Arduino.h>
#include <ModbusRTU.h>
#include "../config.h"
#include "../Managers/DS18B20Manager.h"
#include "../Managers/GPIOManager.h"
#include "../storage/ConfigStore.h"

class ModbusService
{
public:
  ModbusService(DS18B20Manager &, GPIOManager &, ConfigStore &);
  void begin();
  void startTask(UBaseType_t prio = 2,
                 uint32_t stack = 4096,
                 BaseType_t core = tskNO_AFFINITY);

private:
  static void taskTrampoline(void *);
  void taskLoop();

  void applyMappingFromConfig();

  // Helpers for ROM packing
  uint64_t getSlotROM_fromHR(uint16_t slot);
  uint64_t getOffset_fromHR(uint16_t slot);

  void setSlotROM_intoHR(uint16_t slot, uint64_t rom);

  // Helpers for Present ROMs table (read-only mirror)
  void publishPresentROMs();

  uint64_t _slotROM[DS_MAX_SLOTS] = {0};
  int16_t _slotOffset_x100[DS_MAX_SLOTS] = {0};
  int16_t getSlotOffset_fromHR(uint16_t slot);
  void setSlotOffset_intoHR(uint16_t slot, int16_t off);

  DS18B20Manager &_ds;
  GPIOManager &_gpio;
  ConfigStore &_cfg;
  ModbusRTU _mb;

};