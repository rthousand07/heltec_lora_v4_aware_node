# Heltec LoRa V4 Aware Node Firmware

Arduino/PlatformIO firmware for a Heltec LoRa V4 node with:

- TCA9548A I2C multiplexer
- BME280 environmental sensor
- VL53L0X time-of-flight distance sensor
- AMG8833 8x8 thermal sensor
- SCD40 CO2 sensor
- ICS43434 I2S microphone
- SX1262 LoRa radio

The node continuously samples the sensors, computes an awareness score, and uses that score to gate LoRa behavior. Normal LoRa packets are ignored while the node is quiet. Alert/wake packets can still be accepted.

## Important Pin Assumptions

Heltec V4 pin maps can vary by exact model/package. The firmware keeps all board pins near the top of `src/main.cpp`.

Default assumptions in the sketch:

| Function | GPIO |
|---|---:|
| LoRa NSS | 8 |
| LoRa SCK | 9 |
| LoRa MOSI | 10 |
| LoRa MISO | 11 |
| LoRa RST | 12 |
| LoRa BUSY | 13 |
| LoRa DIO1 | 14 |
| I2C SDA | 41 |
| I2C SCL | 42 |
| I2S BCLK | 5 |
| I2S WS/LRCLK | 6 |
| I2S DATA IN | 7 |
| OLED SDA | 17 |
| OLED SCL | 18 |
| OLED reset | 21 |
| OLED/Vext power | 36, active-low |
| OLED I2C address | 0x3C |
| GNSS RX | 34 |
| GNSS TX | 33 |
| GNSS baud | 9600 |

Verify these against your exact Heltec LoRa V4 schematic before flashing.

## OLED Status Display

On boot, the onboard SSD1306 OLED shows a flashed/ready screen. During runtime it shows:

- node ID
- LoRa OK/NO state
- awareness gate state
- current awareness mode and score
- signal bars based on the last packet RSSI
- GNSS fix/no-fix and satellite count
- last packet type
- short status such as `Running`, `RX accepted`, or `RX ignored`

If the firmware runs but the screen stays blank, open the serial monitor. Boot now prints separate `Sensor I2C scan` and `OLED I2C scan` lines. The OLED bus should show `0x3C`. If it does not, adjust `PIN_OLED_SDA`, `PIN_OLED_SCL`, `PIN_OLED_RESET`, or `PIN_OLED_POWER` in `src/main.cpp` for your exact Heltec revision.

## GNSS Port

The firmware reads NMEA sentences from the Heltec GNSS module port using TinyGPS++. The default UART settings are:

```text
GNSS RX: GPIO 34, connect to GNSS TX
GNSS TX: GPIO 33, connect to GNSS RX if needed
Baud:    9600
```

The OLED shows `GPS FIX` or `GPS NO` plus satellite count. Telemetry packets include `gnss_fix`, `gnss_sats`, and, when available, `lat`, `lon`, `alt_m`, and `spd_kmph`.

If the OLED/serial monitor always shows no fix, first check the serial monitor. The firmware prints:

```text
GNSS: UART ready baud=9600 rx=34 tx=33
```

If your exact Heltec V4 uses different GNSS port pins, change `PIN_GNSS_RX` and `PIN_GNSS_TX` in `src/main.cpp`.

## TCA9548A Channels

| TCA Channel | Sensor |
|---:|---|
| 0 | BME280 |
| 1 | VL53L0X |
| 2 | AMG8833 |
| 3 | SCD40 |

All sensors should run at 3.3 V logic. The ICS43434 is not I2C; wire it directly to the configured I2S pins.

## LoRa Packet Format

Incoming packets are expected as:

```text
LP1|TYPE|SOURCE|SEQUENCE|BODY
```

Examples:

```text
LP1|PING|raspi-01|12|hello
LP1|DATA|raspi-01|13|payload=...
LP1|ALERT|raspi-01|14|priority=high
LP1|WAKE|raspi-01|15|reason=operator
```

Default behavior:

- `ALERT`, `WAKE`, and `PING` packets can be accepted even when the node is quiet.
- `DATA`, `TELEM`, and `COMMAND` packets require the awareness gate to be open.
- The node sends `TELEM` packets only when it is aware/alert, unless idle heartbeat is enabled in the config block.

This format is intentionally simple and not authenticated. Add signing/encryption before using it for trusted commands.

## Build And Flash

```bash
pio run -e heltec_lora_v4_aware_node
pio run -e heltec_lora_v4_aware_node -t upload
pio device monitor -b 115200
```

Set `LORA_FREQ_MHZ` in `src/main.cpp` for your region. The default is `915.0` MHz for US ISM operation.
