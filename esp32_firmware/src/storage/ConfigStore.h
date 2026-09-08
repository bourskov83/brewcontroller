
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "../config.h"

// Stores 64-bit ROM per slot in NVS.
// Existing key: "r%02u" -> 8-byte blob.
// New key:      "o%02u" -> 2-byte int16 (°C x100).
class ConfigStore
{
public:
  void begin(const char *ns = "ds18cfg") { _prefs.begin(ns, false); }
  void end() { _prefs.end(); }

  bool loadROM(uint16_t slot, uint64_t &rom)
  {
    char key[8];
    snprintf(key, sizeof(key), "r%02u", slot);
    size_t got = _prefs.getBytesLength(key);
    if (got != 8)
    {
      rom = 0;
      return false;
    }
    uint64_t val = 0;
    _prefs.getBytes(key, &val, 8);
    rom = val;
    return true;
  }

  void saveROM(uint16_t slot, uint64_t rom)
  {
    char key[8];
    snprintf(key, sizeof(key), "r%02u", slot);
    _prefs.putBytes(key, &rom, 8);
  }

  // ---- New: per-slot offset (int16, °C x100) ----
  bool loadOffset(uint16_t slot, int16_t &offset_x100)
  {
    char key[8];
    snprintf(key, sizeof(key), "o%02u", slot);
    size_t got = _prefs.getBytesLength(key);
    if (got != 2)
    {
      offset_x100 = 0;
      return false;
    }
    int16_t tmp = 0;
    _prefs.getBytes(key, &tmp, 2);
    offset_x100 = tmp;
    return true;
  }

  void saveOffset(uint16_t slot, int16_t offset_x100)
  {
    char key[8];
    snprintf(key, sizeof(key), "o%02u", slot);
    _prefs.putBytes(key, &offset_x100, 2);
  }

  void clearAll()
  {
    _prefs.clear();
  }

private:
  Preferences _prefs;
};
