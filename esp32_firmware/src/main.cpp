#include <Arduino.h>
#include "config.h"
#include "storage/ConfigStore.h"
#include "Managers/DS18B20Manager.h"
#include "Managers/GPIOManager.h"
#include "ModbusHandler/ModbusService.h"

ConfigStore    cfg;
DS18B20Manager dsMgr;
GPIOManager    gpioMgr;
ModbusService  mbs(dsMgr, gpioMgr, cfg);

void setup() {
  Serial.begin(115200);

  delay(200);

  cfg.begin();
  gpioMgr.begin();
  dsMgr.begin();
  dsMgr.startTask(1, 4096, tskNO_AFFINITY);

  mbs.begin();
  mbs.startTask(2, 4096, tskNO_AFFINITY);
  

}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}