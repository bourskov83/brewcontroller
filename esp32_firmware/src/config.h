#pragma once
#include <Arduino.h>

// ---------- OneWire / DS18B20 ----------
#define ONEWIRE_PIN                32
#define DS_MAX_SLOTS               10        // amount of sensor slots allocated
#define DS_RESOLUTION_BITS         12
#define SENSOR_UPDATE_INTERVAL_MS  790

// ---------- Modbus RTU over USB Serial ----------
#define MODBUS_SERIAL              Serial2
#define MODBUS_BAUD                38400
#define MODBUS_CONFIG              SERIAL_8N1
#define MODBUS_SLAVE_ID            1

// ---------- GPIO Outputs as Coils ----------
static const int GPIO_OUTPUT_PINS[] =   {25,    26,     27,     2};
static const bool GPIO_ACTIVE_HIGH[] =  {false, false,  false,  true};
#define NUM_GPIO_OUTPUTS (sizeof(GPIO_OUTPUT_PINS)/sizeof(GPIO_OUTPUT_PINS[0]))

// ---------- Modbus Map ----------
#define REG_SENSOR_COUNT           0
#define REG_TEMPS_BASE             10
#define REG_TEMPS_COUNT            DS_MAX_SLOTS



// ROM configuration area (64-bit per slot = 4 HREGs)
constexpr uint16_t REG_ROM_BASE      = 100;
constexpr uint16_t REG_ROM_STRIDE    = 4;

// Control registers
constexpr uint16_t REG_SLOT_CMD          = 200;
constexpr uint16_t REG_SLOT_CMD_STATUS   = 201;

// Command codes for REG_SLOT_CMD
constexpr uint16_t SLOT_CMD_IDLE = 0;
constexpr uint16_t SLOT_CMD_SCAN = 1;
constexpr uint16_t SLOT_CMD_APPLY_MAPPING = 0xA5A5; 
constexpr uint16_t SLOT_CMD_CLEAR_CONFIG = 0xC1EA; 


// Status values for REG_SLOT_CMD_STATUS
constexpr uint16_t SLOT_STATUS_IDLE     = 0;
constexpr uint16_t SLOT_STATUS_BUSY     = 1;
constexpr uint16_t SLOT_STATUS_DONE     = 2;
constexpr uint16_t SLOT_STATUS_ERROR    = 3;

// Present ROMs table (read-only mirror of current bus discovery)
#define REG_PRESENT_BASE 300
// Layout:
//   HREG[300]           -> PRESENT_COUNT (R)
//   HREG[301..304]      -> ROM64 of present[0] (R)
//   HREG[305..308]      -> ROM64 of present[1] (R)
//   ...

// Sentinel for invalid temperature
constexpr int16_t TEMP_INVALID_SENTINEL             = -32768;
constexpr int16_t DS_START_TEMP_VALUE               = 8500; // x100 85.00C before first conversion is complete
constexpr int16_t DS_START_TEMP_VALUE_SUSPECT_DELTA = 800;   // x100 max. C a sensor can step change before being deemed wrong

// =====================================================================
// Compile-time sanity checks
// =====================================================================
static_assert(DS_RESOLUTION_BITS >= 9 && DS_RESOLUTION_BITS <= 12,
              "DS_RESOLUTION_BITS must be 9-12 (DS18B20 supported range)");

static_assert(DS_MAX_SLOTS > 0 && DS_MAX_SLOTS <= 99,
              "DS_MAX_SLOTS must be 1 to 99");

static_assert(SENSOR_UPDATE_INTERVAL_MS > 0,
              "SENSOR_UPDATE_INTERVAL_MS must be positive");

// =====================================================================
// Logging configuration
// =====================================================================

constexpr uint16_t REG_LOG_LEVEL_ALL = 210;  // writing sets every tag
constexpr uint16_t REG_LOG_LEVEL_BASE = 211; // 211 + index into LOG_TAGS
constexpr uint16_t LOG_LEVEL_MAX = 5;        // 0 none, 1 error, 2 warn, 3 info, 4 debug, 5 verbose
constexpr uint16_t LOG_LEVEL_DEFAULT = 3;

constexpr const char *LOG_TAGS[] = {
    "MAIN",
    "CFGSTR",
    "SLOTCFG",
    "DS18B20",
    "GPIO",
    "MODBUS",
};
constexpr uint16_t LOG_TAG_COUNT = sizeof(LOG_TAGS) / sizeof(LOG_TAGS[0]);


// workaround issues with arduino filters
#define LOGE(tag, fmt, ...) do { if (esp_log_level_get(tag) >= ESP_LOG_ERROR)   ESP_LOGE(tag, fmt, ##__VA_ARGS__); } while (0)
#define LOGW(tag, fmt, ...) do { if (esp_log_level_get(tag) >= ESP_LOG_WARN)    ESP_LOGW(tag, fmt, ##__VA_ARGS__); } while (0)
#define LOGI(tag, fmt, ...) do { if (esp_log_level_get(tag) >= ESP_LOG_INFO)    ESP_LOGI(tag, fmt, ##__VA_ARGS__); } while (0)
#define LOGD(tag, fmt, ...) do { if (esp_log_level_get(tag) >= ESP_LOG_DEBUG)   ESP_LOGD(tag, fmt, ##__VA_ARGS__); } while (0)
#define LOGV(tag, fmt, ...) do { if (esp_log_level_get(tag) >= ESP_LOG_VERBOSE) ESP_LOGV(tag, fmt, ##__VA_ARGS__); } while (0)