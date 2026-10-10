#pragma once
#include <Arduino.h>
#include <ModbusRTU.h>
#include "../config.h"
#include "../Managers/DS18B20Manager.h"
#include "../Managers/GPIOManager.h"
#include "../Managers/SlotConfigManager.h"
#include "esp_log.h"
#include "inttypes.h"

class ModbusService
{
public:
  ModbusService(DS18B20Manager &, GPIOManager &, SlotConfigManager &);
  void begin();
  void startTask(UBaseType_t prio = 2,
                 uint32_t stack = 4096,
                 BaseType_t core = tskNO_AFFINITY);

private:
  static void taskTrampoline(void *);
  void taskLoop();


  // Helpers for ROM packing
  uint64_t getSlotROM_fromHR(uint16_t slot);

  void setSlotROM_intoHR(uint16_t slot, uint64_t rom);
  void setupLogRegisters();
  // Helpers for Present ROMs table (read-only mirror)
  void publishPresentROMs();
  void refreshRomHRs();
  uint16_t handleSlotCmd(uint16_t cmd);
  void setSlotStatus(uint16_t status);
  DS18B20Manager &_ds;
  GPIOManager &_gpio;
  SlotConfigManager &_slotCfg;
  ModbusRTU _mb;

};