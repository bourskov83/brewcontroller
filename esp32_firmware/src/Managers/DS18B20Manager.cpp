#include "DS18B20Manager.h"

// Set logging tag
static const char *TAG = "DS18B20";

DS18B20Manager::DS18B20Manager()
    : _oneWire(ONEWIRE_PIN), _sensors(&_oneWire) {}

void DS18B20Manager::begin()
{
  _sensors.begin();
  _sensors.setResolution(DS_RESOLUTION_BITS);
  _sensors.setWaitForConversion(false);

  LOGI(TAG, "1-wire pin=%d resolution=%d bits", ONEWIRE_PIN, DS_RESOLUTION_BITS);
}

void DS18B20Manager::startTask(UBaseType_t prio, uint32_t stack, BaseType_t core)
{
  xTaskCreatePinnedToCore(taskTrampoline, "DS18Task", stack, this, prio, nullptr, core);
  LOGI(TAG, "task started (core=%d prio=%u stack=%u)", core, prio, stack);
}

void DS18B20Manager::taskTrampoline(void *arg)
{
  reinterpret_cast<DS18B20Manager *>(arg)->taskLoop();
}

void DS18B20Manager::applyMapping(const uint64_t slotROMs[DS_MAX_SLOTS])
{
  for (uint16_t i = 0; i < DS_MAX_SLOTS; i++)
    _slots[i].rom = slotROMs[i];
  LOGI(TAG, "%d slots updated", DS_MAX_SLOTS);
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

bool DS18B20Manager::requestScan()
{
  uint8_t expected = SCAN_IDLE;
  // only succeeds if idle, so a second request while busy is refused
  return _scanState.compare_exchange_strong(expected, SCAN_REQUESTED);
}

bool DS18B20Manager::scanDone() const { return _scanState == SCAN_DONE; }

void DS18B20Manager::clearScanDone() { _scanState = SCAN_IDLE; }

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
      LOGW(TAG, "CRC mismatch, discarding a bus reading");
      continue;
    }
    const uint64_t rom = addr_to_rom64(addr);
    if (_presentCount < MAX_DISC)
    {
      _presentROMs[_presentCount++] = rom;
    }
    else {
      LOGE(TAG, "discovered device dropped, MAX_DISC=%u reached", MAX_DISC);
    }
  }

  // Retry once if the very first scan races sensor power-up and finds 0
  if (_presentCount == 0)
  {
    LOGW(TAG, "nothing found on first pass, retrying after power-up delay");
    vTaskDelay(pdMS_TO_TICKS(100));
    _presentCount = 0;
    _oneWire.reset_search();

    while (_oneWire.search(addr))
    {
      if (OneWire::crc8(addr, 7) != addr[7])
      {
        LOGW(TAG, "CRC mismatch, discarding a bus reading");

        continue;
      }
      const uint64_t rom = addr_to_rom64(addr);
      if (_presentCount < MAX_DISC)
      {
        _presentROMs[_presentCount++] = rom;
      }
      else
      {
        LOGE(TAG, "discovered device dropped, MAX_DISC=%u reached", MAX_DISC);
      }
    }
  }
  LOGI(TAG, "busScan -> presentCount=%u", _presentCount);
}


int16_t DS18B20Manager::readSlotTemp_x100(uint16_t slot) const
{
  if (slot >= DS_MAX_SLOTS) {
    LOGE(TAG, "slot=%d out of bounds max=%d", slot, DS_MAX_SLOTS);
    return TEMP_INVALID_SENTINEL;
  }
//    LOGD(TAG, "slot[%d] val=%d", slot, _slots[slot].temp_x100);
    return _slots[slot].temp_x100;
}



  // Convert resolution bits to conversion delays in ms
  static constexpr TickType_t ds18b20ConversionTicks(uint8_t resBits)
  {
    switch (resBits)
    { 
      // Datasheet conversion time in ms for different resoultion bits
    case 9:  return pdMS_TO_TICKS(94);
    case 10: return pdMS_TO_TICKS(188);
    case 11: return pdMS_TO_TICKS(375);
    case 12: return pdMS_TO_TICKS(750);
    default: return pdMS_TO_TICKS(750); // fallback;
    }
  }

  void DS18B20Manager::updateSlot(uint16_t idx, int16_t temp_x100) {

    // Load current slot
    SlotData &slot = _slots[idx];

    const bool looksLikePowerOn =
        slot.temp_x100 == TEMP_INVALID_SENTINEL ||
        abs(slot.temp_x100 - DS_START_TEMP_VALUE) >= DS_START_TEMP_VALUE_SUSPECT_DELTA;

    if (temp_x100 == DS_START_TEMP_VALUE)
    {
      // got start-up value, check previous value
      if (looksLikePowerOn && !slot.suspect85)
      {
        // sensor booting up, skip this cycle
        slot.suspect85 = true;
        LOGW(TAG, "rejected 85.00C reading, might be startup slot=%d, lastVal=%d", idx, slot.temp_x100);
        return;
      }
    }
    slot.suspect85 = false;
    slot.temp_x100 = temp_x100;
  }
 

  void DS18B20Manager::taskLoop()
  {
    // Get conversion time for configured resolution
    const TickType_t convTicks = ds18b20ConversionTicks(DS_RESOLUTION_BITS);
    // Evaluate conversion time vs requested scan time - scan time cannot be lower than conversion time.
    const TickType_t period = max(convTicks, pdMS_TO_TICKS(SENSOR_UPDATE_INTERVAL_MS));
    LOGI(TAG, "convTicks=%u period=%u (ticks)", convTicks, period);

    while (true)
    {
      // Record start of loop tick count so we can calculate post delay to match SENSOR_UPDATE_INTERVAL_MS
      const TickType_t loopStart = xTaskGetTickCount();

      _sensors.requestTemperatures();
      vTaskDelay(convTicks);

      // Update all slot temps
      for (uint16_t i = 0; i < DS_MAX_SLOTS; i++)
      {
        int16_t temp = TEMP_INVALID_SENTINEL;

        DeviceAddress a;
        rom64_to_addr(_slots[i].rom, a);
        float c = _sensors.getTempC(a);

        if (!isnan(c) && c > -127.0f)
        {
          long vRaw = lroundf(c * 100.0f);


          if (vRaw > 32767)
            vRaw = 32767;
          if (vRaw < -32768)
            vRaw = -32768;
          temp = (int16_t)vRaw;
        }

        // validate if new reading is reasonable
        // new sensors on bus will report 85.00 before first conversion
        updateSlot(i, temp);

        LOGD(TAG, "slot %u: temp_x100=%d, suspect85=%x", i, _slots[i].temp_x100, _slots[i].suspect85);
      }

      if (_scanState == SCAN_REQUESTED) // do a bus scan
      {
        LOGD(TAG, "requested 1-wire bus scan");
        busScan();
        _scanState = SCAN_DONE;
      }

      // Wait remaining time to match SENSOR_UPDATE_INTERVAL_MS
      const TickType_t elapsed = xTaskGetTickCount() - loopStart;
      if (elapsed < period)
        vTaskDelay(period - elapsed);
    }
  }