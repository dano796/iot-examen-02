# IoT Exam 2 - End Device

Temperature and GPS end device on LilyGO LoRa32 with chirp, prunning and bundling stages, sending a JSON frame every 10 s to a LoRa collector.

## Hardware

| Component | Details |
|---|---|
| Board | LilyGO T-Beam (ESP32 + SX1278 LoRa radio) |
| Temperature sensor | HDC1080, I2C address `0x40` (SDA 21, SCL 22) |
| GPS | NEO-6M on `Serial1` |

## State machine

```
CHIRP  ->  PRUNNING  ->  BUNDLING  ->  TX  -+
  ^                                         |
  +-----------------------------------------+
```

| State | What it does |
|---|---|
| **CHIRP** | Takes 10 temperature samples from the HDC1080, waiting 300 ms between them so the sensor can recover. Readings that are `NaN` or outside -40 °C to 124 °C are discarded. |
| **PRUNNING** | Needs at least 3 valid samples. Computes the mean and standard deviation, drops every sample more than 2σ away from the mean and averages the rest. Also takes the GPS position if the fix is valid and less than 10 s old. |
| **BUNDLING** | Builds the JSON frame. |
| **TX** | Sends the frame over LoRa, prints a status report on the serial port and waits until the 10 s cycle is complete. |

All waits use `smartDelay()` instead of `delay()`, so the NMEA parser keeps reading the GPS UART. At 9600 baud the buffer overflows if it is not read continuously.

## Payload

```json
{"id": "186841","lat": 6.245259, "lon": -75.596510, "temperatura" : 22.4, "metadata":{"campoCifrado":"","campoIntegridad":""}}
```

- `id`, `lat`, `lon` and `temperatura` are mandatory. Latitude and longitude use 6 decimals, temperature uses 1
- `metadata` belongs to the optional bonus (Caesar cipher and MD5 integrity). Not implemented here, so both fields are sent empty

## Fault handling

- **Sensor missing:** on startup the firmware pings the I2C bus at `0x40` before using the HDC1080. Without this check an unplugged sensor reads `0xFFFF`, which the library turns into a plausible-looking 125 °C.
- **Not enough samples:** if fewer than 3 samples are valid, the last consolidated temperature is sent again (`0.0` if there has never been one).
- **No GPS fix:** if there is no recent fix, the last known position is sent again (`0, 0` if there has never been one).

## Build and flash

Requires [PlatformIO](https://platformio.org/). Libraries are declared in `platformio.ini` and installed automatically.

```bash
pio run -t upload        # build and flash
pio device monitor       # serial output at 115200 baud
```
