#include "GPIOManager.h"

void GPIOManager::begin() {
  for (uint16_t i = 0; i < NUM_GPIO_OUTPUTS; i++) {
    pinMode(GPIO_OUTPUT_PINS[i], OUTPUT);
    digitalWrite(GPIO_OUTPUT_PINS[i], GPIO_ACTIVE_HIGH[i] ? LOW : HIGH);
  }
}

void GPIOManager::setOutput(uint16_t index, bool state) {
  if (index >= NUM_GPIO_OUTPUTS) return;
  bool physicalState = GPIO_ACTIVE_HIGH[index] ? state : !state;
  digitalWrite(GPIO_OUTPUT_PINS[index], physicalState ? HIGH : LOW);
}

bool GPIOManager::getOutput(uint16_t index) const {
  if (index >= NUM_GPIO_OUTPUTS) return false;
  return digitalRead(GPIO_OUTPUT_PINS[index]);
}