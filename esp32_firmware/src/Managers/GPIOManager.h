#pragma once
#include <Arduino.h>
#include "../config.h"

class GPIOManager {
public:
  void begin();
  void setOutput(uint16_t index, bool on);
  bool getOutput(uint16_t index) const;
};