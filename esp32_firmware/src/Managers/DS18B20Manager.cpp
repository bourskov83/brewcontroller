#include "DS18B20Manager.h"

DS18B20Manager::DS18B20Manager()
    : _oneWire(ONEWIRE_PIN), _sensors(&_oneWire) {}

void DS18B20Manager::begin()
{
  _sensors.begin();
  _sensors.setResolution(DS_RESOLUTION_BITS);
  _sensors.setWaitForConversion(false);
  busScan();
}

void DS18B20Manager::startTask(UBaseType_t prio, uint32_t stack, BaseType_t core)
{
  xTaskCreatePinnedToCore(taskTrampoline, "DS18Task", stack, this, prio, nullptr, core);
}

void DS18B20Manager::taskTrampoline(void *arg)
{
  reinterpret_cast<DS18B20Manager *>(arg)->taskLoop();
}

void DS18B20Manager::applyMapping(const uint64_t slotROMs[DS_MAX_SLOTS])
{
  for (uint16_t i = 0; i < DS_MAX_SLOTS; i++)
    _slotROM[i] = slotROMs[i];
  busScan();
}

bool DS18B20Manager::romPresent(uint64_t rom) const
{
  if (rom == 0)
    return false;
  for (uint16_t i = 0; i < _presentCount; i++)
    if (_presentROMs[i] == rom)
      return true;
  return false;
}

void DS18B20Manager::busScan()
{
  // First attempt
  _presentCount = 0;
  _oneWire.reset_search();

  DeviceAddress addr;
  while (_oneWire.search(addr))
  {
    if (OneWire::crc8(addr, 7) != addr[7])
    {
      continue;
    }
    const uint64_t rom = addr_to_rom64(addr);
    if (_presentCount < MAX_DISC)
    {
      _presentROMs[_presentCount++] = rom;
    }
  }

  // Retry once if the very first scan races sensor power-up and finds 0
  if (_presentCount == 0)
  {
    vTaskDelay(pdMS_TO_TICKS(100));
    _presentCount = 0;
    _oneWire.reset_search();

    while (_oneWire.search(addr))
    {
      if (OneWire::crc8(addr, 7) != addr[7])
      {
        continue;
      }
      const uint64_t rom = addr_to_rom64(addr);
      if (_presentCount < MAX_DISC)
      {
        _presentROMs[_presentCount++] = rom;
      }
    }
  }

//  Serial.printf("[DS] busScan -> presentCount=%u\n", _presentCount);
}


int16_t DS18B20Manager::readSlotTemp_x100(uint16_t slot) const
{
  if (slot >= DS_MAX_SLOTS)
    return TEMP_INVALID_SENTINEL;
  return _slotTemp_x100[slot];
}

int16_t DS18B20Manager::readSlotRawTemp_x100(uint16_t slot) const
{
  if (slot >= DS_MAX_SLOTS)
    return TEMP_INVALID_SENTINEL;
  return _slotRawTemp_x100[slot]; // raw
}

  void DS18B20Manager::applyOffsets(const int16_t slotOffsets_x100[DS_MAX_SLOTS])
  {
    for (uint16_t i = 0; i < DS_MAX_SLOTS; i++)
      _slotOffset_x100[i] = slotOffsets_x100[i];
  }

  void DS18B20Manager::taskLoop()
  {
    const TickType_t convTicks = pdMS_TO_TICKS(750);
    const TickType_t period = max(convTicks, pdMS_TO_TICKS(SENSOR_UPDATE_INTERVAL_MS));
    while (true)
    {
      _sensors.requestTemperatures();
      vTaskDelay(convTicks);

      // Update all slot temps
      for (uint16_t i = 0; i < DS_MAX_SLOTS; i++)
      {
        int16_t raw_x100 = TEMP_INVALID_SENTINEL;
        int16_t cor_x100 = TEMP_INVALID_SENTINEL;
        uint64_t rom = _slotROM[i];

        if (romPresent(rom))
        {
          DeviceAddress a;
          rom64_to_addr(rom, a);
          float c = _sensors.getTempC(a);

          if (!isnan(c) && c > -127.0f)
          {
            long vRaw = lroundf(c * 100.0f);


            if (vRaw > 32767)
              vRaw = 32767;
            if (vRaw < -32768)
              vRaw = -32768;
            raw_x100 = (int16_t)vRaw;

            // Apply offset to get corrected value
            long vCor = vRaw + (long)_slotOffset_x100[i];
            if (vCor > 32767)
              vCor = 32767;
            if (vCor < -32768)
              vCor = -32768;
            cor_x100 = (int16_t)vCor;

          }
        }

        _slotRawTemp_x100[i] = raw_x100; // raw (no offset)
        _slotTemp_x100[i] = cor_x100;    // corrected (with offset)

      }

    }
  }