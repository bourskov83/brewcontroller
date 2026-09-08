# ESP32 Modbus RTU (USB Serial) + DS18B20 ROM Mapping

This project exposes up to 10 DS18B20 sensors and GPIO outputs via **Modbus RTU** over the ESP32's **USB Serial** using the Arduino framework and FreeRTOS.

- Library: [emelianov/modbus-esp8266](https://github.com/emelianov/modbus-esp8266)
- OneWire: [Paul Stoffregen/OneWire]
- Temperature: [DallasTemperature]

## Features
- **Modbus RTU Server (Slave)** over `Serial` (USB CDC)
- **Holding Registers**:
  - `HREG[0]` → Sensor count present on bus (read-only)
  - `HREG[1..10]` → Temperatures per logical slot, **°C×100** (read-only)
  - `HREG[100 + s*4 .. 103 + s*4]` → 64-bit **ROM** for slot `s` (little-endian across 4 HREGs)
  - `HREG[200]` → Write **0xA5A5** to apply mapping (rescan + bind)
  - `HREG[201]` → Write **0xDEAD** to clear config (NVS)
- **Coils**: `COIL[0..N-1]` map to GPIO outputs (write to toggle)
- **NVS persistence** of ROMs per slot

## Wiring
- DS18B20 bus on `GPIO 4` with 4.7k pull-up to 3V3.
- GPIO outputs default: `2, 13, 14` (edit in `config.h`).

## Build (PlatformIO)
```
pio run -t upload
pio device monitor -b 115200
```

## Modbus Map Details
- **Addressing is 0-based** in the firmware/library. Some master tools use 1-based offsets—configure accordingly.
- Temperature scaling example: 23.56°C → HREG = 2356. Unbound/invalid → `-32768`.

## Programming the ROM Mapping
1. Discover your probe's 64-bit ROM (e.g., with a temporary sketch or host tool). Example bytes: `28-FF-6C-A2-91-16-05-2B` (Byte0..7).
2. Compose little-endian 64-bit: `rom64 = 0x2B051691A26CFF28`.
3. For **slot 0**, write:
   - `HREG[100] = 0xFF28`
   - `HREG[101] = 0xA26C`
   - `HREG[102] = 0x1691`
   - `HREG[103] = 0x2B05`
4. Write `0xA5A5` to `HREG[200]` to apply.
5. Read temperatures from `HREG[1..10]`.

## Notes
- Library uses callbacks to process writes safely; we defer ROM persistence until APPLY to avoid partial updates.
- You can adapt the ROM endianness to your tooling by changing packing in `ModbusService`.
- If you later move to RS-485, supply `txEnablePin` to `mb.begin(&SerialX, txEN)` and use a hardware UART.

## License
MIT for this example (check third-party libs for their licenses).