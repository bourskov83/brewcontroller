#include "GPIOManager.h"

// Set logging tag
static const char *TAG = "GPIO";

// Setup pin states form config.h
void GPIOManager::begin() {
  ESP_LOGI(TAG, "initializing %u outputs", (unsigned)NUM_GPIO_OUTPUTS);
  for (uint16_t i = 0; i < NUM_GPIO_OUTPUTS; i++) {
    pinMode(GPIO_OUTPUT_PINS[i], OUTPUT);
    digitalWrite(GPIO_OUTPUT_PINS[i], GPIO_ACTIVE_HIGH[i] ? LOW : HIGH);
    ESP_LOGI(TAG, "pin %u (GPIO%d, activeHigh=%d) -> OFF",
             i, GPIO_OUTPUT_PINS[i], GPIO_ACTIVE_HIGH[i]);
  }
}

// Set output state of pins
void GPIOManager::setOutput(uint16_t index, bool state) {
  if (index >= NUM_GPIO_OUTPUTS) {
    ESP_LOGE(TAG, "index %u out of range (max %u)", index, (unsigned)NUM_GPIO_OUTPUTS - 1);
    return;
  }
  bool physicalState = GPIO_ACTIVE_HIGH[index] ? state : !state;
  uint8_t level = physicalState ? HIGH : LOW;
  bool changed = (digitalRead(GPIO_OUTPUT_PINS[index]) != level);
  digitalWrite(GPIO_OUTPUT_PINS[index], physicalState ? HIGH : LOW);

  if (changed)
    LOGI(TAG, "pin %u -> %s", index, state ? "ON" : "OFF");
}

// Get current state of output pins
bool GPIOManager::getOutput(uint16_t index) const {
  if (index >= NUM_GPIO_OUTPUTS) return false;
  return digitalRead(GPIO_OUTPUT_PINS[index]);
}