#pragma once
#include <Arduino.h>

// ---------- OneWire / DS18B20 ----------
#define ONEWIRE_PIN                32
#define DS_MAX_SLOTS               10        // logical slots
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
#define REG_TEMPS_RAW_BASE         30
#define REG_TEMPS_OFFSET_BASE      50
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
#define TEMP_INVALID_SENTINEL      (-32768)