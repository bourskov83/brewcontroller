#include <Arduino.h>
#include "config.h"
#include "storage/ConfigStore.h"
#include "Managers/DS18B20Manager.h"
#include "Managers/GPIOManager.h"
#include "Managers/SlotConfigManager.h"
#include "ModbusHandler/ModbusService.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

ConfigStore    cfg;
DS18B20Manager dsMgr;
GPIOManager    gpioMgr;
SlotConfigManager slotCfg(cfg, dsMgr);
ModbusService  mbs(dsMgr, gpioMgr, slotCfg);

void setup() {

  Serial.begin(115200);
  delay(200); // delay to stabilize serial as a precaution
  LOGI(TAG, "serial monitor initialized");
  
  LOGI(TAG, "init ConfigStore");
  cfg.begin();

  LOGI(TAG, "init GPIOManager");
  gpioMgr.begin();


  
  LOGI(TAG, "init DS18B20Manager");
  dsMgr.begin();

  LOGI(TAG, "init SlotConfigManager");
  slotCfg.begin();

  LOGI(TAG, "DS18B20Manager: startTask");
  dsMgr.startTask(1, 4096, 0);

  LOGI(TAG, "init ModbusService");
  mbs.begin();
  mbs.startTask(2, 4096, 1);

  LOGI(TAG, "init sequence complete");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}