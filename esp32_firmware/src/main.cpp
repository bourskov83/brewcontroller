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
  esp_log_level_set("*", ESP_LOG_DEBUG);
  esp_log_level_set("DS18B20", ESP_LOG_WARN);

  Serial.begin(115200);
  delay(200); // delay to stabilize serial as a precaution
  ESP_LOGI(TAG, "serial monitor initialized");
  
  ESP_LOGI(TAG, "init ConfigStore");
  cfg.begin();

  ESP_LOGI(TAG, "init GPIOManager");
  gpioMgr.begin();


  
  ESP_LOGI(TAG, "init DS18B20Manager");
  dsMgr.begin();

  ESP_LOGI(TAG, "init SlotConfigManager");
  slotCfg.begin();

  ESP_LOGI(TAG, "DS18B20Manager: startTask");
  dsMgr.startTask(1, 4096, 0);

  ESP_LOGI(TAG, "init ModbusService");
  mbs.begin();
  mbs.startTask(2, 4096, 1);

  ESP_LOGI(TAG, "init sequence complete");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}