#include "SlotConfigManager.h"

// Set logging tag
static const char *TAG = "SLOTCFG";

SlotConfigManager::SlotConfigManager(
    ConfigStore &cfg,
    DS18B20Manager &ds)
    : _cfg(cfg), _ds(ds) {}

// Load ROMs from NVS


void SlotConfigManager::begin() {
    loadAll();
    applyROMs();
    ESP_LOGD(TAG, "loaded and applied");
}

void SlotConfigManager::loadAll() {
    // load roms from saved config and create array of ROMs
    for (uint16_t i = 0; i < DS_MAX_SLOTS; i++)
    {
        uint64_t rom = 0;
        _cfg.loadROM(i, rom);
        _slotROMs[i] = rom;

    }
    ESP_LOGI(TAG, "loaded %d slots from NVS", DS_MAX_SLOTS);
};

void SlotConfigManager::clearAll() {
    for (uint16_t i =0 ; i < DS_MAX_SLOTS; i++) {
        _slotROMs[i] = 0;
    }
    commit();
    ESP_LOGI(TAG, "zeroing all slot roms");
}

uint64_t SlotConfigManager::getROM(uint16_t slot) const {
    if (slot >= DS_MAX_SLOTS) {
        ESP_LOGE(TAG, "slot out of bounds! slot=%d, max_slot=%d", slot, DS_MAX_SLOTS);
        return 0;
    }

    return _slotROMs[slot];
}

void SlotConfigManager::setROM(uint16_t slot, uint64_t rom) {
    if (slot >= DS_MAX_SLOTS) {
        ESP_LOGE(TAG, "slot out of bounds! slot=%d, max_slot=%d", slot, DS_MAX_SLOTS);
        return;
    }
    _slotROMs[slot] = rom;
    ESP_LOGD(TAG, "slot=%u rom=%016" PRIX64, slot, rom);
}

void SlotConfigManager::commit() {
    for (uint16_t i = 0; i < DS_MAX_SLOTS; i++) {
        saveROM(i, _slotROMs[i]);
    }
    applyROMs();
    ESP_LOGI(TAG, "saved %d slots", DS_MAX_SLOTS);
}

void SlotConfigManager::saveROM(uint16_t slot, uint64_t rom) {
    // carry over slot,rom mapping to ConfigStore
    _cfg.saveROM(slot, rom);
    ESP_LOGD(TAG, "slot=%u rom=%016" PRIX64, slot, rom);
}



void SlotConfigManager::applyROMs() {
    _ds.applyMapping(_slotROMs);
    ESP_LOGD(TAG, "apply to _ds instance");
}
