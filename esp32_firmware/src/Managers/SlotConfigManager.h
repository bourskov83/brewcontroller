#pragma once
#include <Arduino.h>
#include "../config.h"
#include "esp_log.h"
#include "../Managers/DS18B20Manager.h"
#include "../storage/ConfigStore.h"
#include "inttypes.h"

class SlotConfigManager
{
    public:
        SlotConfigManager(ConfigStore &, DS18B20Manager &);
        void begin();
        void commit();
        void clearAll();
        uint64_t getROM(uint16_t slot) const;
        void setROM(uint16_t slot, uint64_t rom);
        

    private:

        ConfigStore &_cfg;
        DS18B20Manager &_ds;
        uint64_t _slotROMs[DS_MAX_SLOTS] = {};

        void loadAll();
        void saveROM(uint16_t slot, uint64_t rom);
        void applyROMs();
    };