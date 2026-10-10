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
#define REG_ROM_BASE               100
#define REG_ROM_STRIDE             4

// Control registers
#define REG_APPLY_MAPPING          200
#define REG_CLEAR_CONFIG           201

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
// Logging workaround for using filters in Arduino framework
// =====================================================================

#define LOGE(tag, fmt, ...) do { if (esp_log_level_get(tag) >= ESP_LOG_ERROR)   ESP_LOGE(tag, fmt, ##__VA_ARGS__); } while (0)
#define LOGW(tag, fmt, ...) do { if (esp_log_level_get(tag) >= ESP_LOG_WARN)    ESP_LOGW(tag, fmt, ##__VA_ARGS__); } while (0)
#define LOGI(tag, fmt, ...) do { if (esp_log_level_get(tag) >= ESP_LOG_INFO)    ESP_LOGI(tag, fmt, ##__VA_ARGS__); } while (0)
#define LOGD(tag, fmt, ...) do { if (esp_log_level_get(tag) >= ESP_LOG_DEBUG)   ESP_LOGD(tag, fmt, ##__VA_ARGS__); } while (0)
#define LOGV(tag, fmt, ...) do { if (esp_log_level_get(tag) >= ESP_LOG_VERBOSE) ESP_LOGV(tag, fmt, ##__VA_ARGS__); } while (0)