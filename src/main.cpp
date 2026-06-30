#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include <RadioLib.h>
#include <Adafruit_BME280.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_AMG88xx.h>
#include <Adafruit_SSD1306.h>
#include <SensirionI2CScd4x.h>
#include <TinyGPSPlus.h>
#include <esp_idf_version.h>
#include <driver/i2s.h>

// ---------------------------------------------------------------------------
// Board configuration: verify these against your exact Heltec LoRa V4 variant.
// ---------------------------------------------------------------------------

static constexpr const char* NODE_ID = "heltec-aware-01";

static constexpr int PIN_LORA_NSS = 8;
static constexpr int PIN_LORA_SCK = 9;
static constexpr int PIN_LORA_MOSI = 10;
static constexpr int PIN_LORA_MISO = 11;
static constexpr int PIN_LORA_RST = 12;
static constexpr int PIN_LORA_BUSY = 13;
static constexpr int PIN_LORA_DIO1 = 14;

static constexpr int PIN_I2C_SDA = 41;
static constexpr int PIN_I2C_SCL = 42;

static constexpr int PIN_I2S_BCLK = 5;
static constexpr int PIN_I2S_WS = 6;
static constexpr int PIN_I2S_DIN = 7;

static constexpr int PIN_OLED_SDA = 17;
static constexpr int PIN_OLED_SCL = 18;
static constexpr int PIN_OLED_RESET = 21;
static constexpr int PIN_OLED_POWER = 36;
static constexpr uint8_t OLED_POWER_ON_LEVEL = LOW;
static constexpr uint8_t OLED_ADDR = 0x3C;
static constexpr int OLED_WIDTH = 128;
static constexpr int OLED_HEIGHT = 64;
static constexpr uint32_t OLED_INTERVAL_MS = 500;

static constexpr bool GNSS_ENABLED = true;
static constexpr int PIN_GNSS_RX = 34;  // ESP32-S3 RX, connect to GNSS TX.
static constexpr int PIN_GNSS_TX = 33;  // ESP32-S3 TX, connect to GNSS RX if needed.
static constexpr uint32_t GNSS_BAUD = 9600;
static constexpr uint32_t GNSS_STALE_MS = 10000;

static constexpr uint8_t TCA_ADDR = 0x70;
static constexpr uint8_t TCA_CH_BME280 = 0;
static constexpr uint8_t TCA_CH_VL53L0X = 1;
static constexpr uint8_t TCA_CH_AMG8833 = 2;
static constexpr uint8_t TCA_CH_SCD40 = 3;
static constexpr uint8_t SCD40_ADDR = 0x62;

static constexpr float LORA_FREQ_MHZ = 915.0;
static constexpr float LORA_BW_KHZ = 125.0;
static constexpr uint8_t LORA_SPREADING_FACTOR = 7;
static constexpr uint8_t LORA_CODING_RATE = 5;
static constexpr uint8_t LORA_SYNC_WORD = 0x12;
static constexpr int8_t LORA_TX_POWER_DBM = 14;
static constexpr float MIN_ACCEPT_RSSI_DBM = -125.0;

static constexpr uint32_t SENSOR_INTERVAL_MS = 1000;
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 15000;
static constexpr uint32_t IDLE_HEARTBEAT_INTERVAL_MS = 300000;
static constexpr bool SEND_IDLE_HEARTBEAT = false;
static constexpr uint32_t AWARE_GATE_HOLD_MS = 30000;

// Awareness tuning. Treat these as field-calibration starting points.
static constexpr int NEAR_OBJECT_MM = 1200;
static constexpr int VERY_NEAR_OBJECT_MM = 450;
static constexpr float THERMAL_SPAN_AWARE_C = 3.0;
static constexpr float THERMAL_DELTA_AWARE_C = 1.2;
static constexpr float THERMAL_HOT_ALERT_C = 42.0;
static constexpr float AUDIO_RMS_AWARE = 0.020;
static constexpr float AUDIO_RMS_ALERT = 0.080;
static constexpr uint16_t CO2_AWARE_PPM = 1200;
static constexpr uint16_t CO2_ALERT_PPM = 1800;

#ifndef I2S_COMM_FORMAT_STAND_I2S
#define I2S_COMM_FORMAT_STAND_I2S I2S_COMM_FORMAT_I2S
#endif

// ---------------------------------------------------------------------------
// Hardware objects
// ---------------------------------------------------------------------------

SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);
Adafruit_BME280 bme280;
Adafruit_VL53L0X vl53 = Adafruit_VL53L0X();
Adafruit_AMG88xx amg8833;
SensirionI2cScd4x scd40;
TwoWire OledWire = TwoWire(1);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &OledWire, PIN_OLED_RESET);
HardwareSerial GnssSerial(1);
TinyGPSPlus gps;

static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

bool oledOk = false;
bool bmeOk = false;
bool vl53Ok = false;
bool amgOk = false;
bool scdOk = false;
bool i2sOk = false;
bool loraOk = false;
bool gnssOk = false;

volatile bool loraPacketReady = false;

// ---------------------------------------------------------------------------
// Data model
// ---------------------------------------------------------------------------

enum class AwarenessMode : uint8_t {
  Quiet,
  Watching,
  Aware,
  Alert,
};

struct SensorFrame {
  uint32_t ms = 0;

  float bmeTempC = NAN;
  float bmeHumidityPct = NAN;
  float pressureHpa = NAN;

  uint16_t co2Ppm = 0;
  float scdTempC = NAN;
  float scdHumidityPct = NAN;

  bool distanceValid = false;
  int distanceMm = -1;

  float thermalAvgC = NAN;
  float thermalMinC = NAN;
  float thermalMaxC = NAN;
  float thermalSpanC = NAN;
  float thermalDeltaC = NAN;
  uint8_t thermalHotPixels = 0;

  float audioRms = NAN;
  float audioDbfs = NAN;
};

struct GnssState {
  bool hasFix = false;
  bool locationValid = false;
  uint32_t chars = 0;
  uint32_t sentences = 0;
  uint32_t failedChecksum = 0;
  uint32_t lastUpdateMs = 0;
  double lat = 0.0;
  double lon = 0.0;
  double altitudeM = 0.0;
  double speedKmph = 0.0;
  uint32_t ageMs = 0;
  uint32_t hdop = 0;
  uint32_t sats = 0;
};

struct AwarenessState {
  AwarenessMode mode = AwarenessMode::Quiet;
  float score = 0.0f;
  bool gateOpen = false;
  uint32_t lastInterestingMs = 0;
  float audioBaseline = NAN;
  String reasons;
};

struct PacketView {
  bool valid = false;
  String type;
  String source;
  uint32_t sequence = 0;
  String body;
};

SensorFrame currentFrame;
GnssState gnss;
AwarenessState awareness;

float lastThermalPixels[64] = {0};
bool haveLastThermalPixels = false;

uint16_t lastCo2Ppm = 0;
float lastScdTempC = NAN;
float lastScdHumidityPct = NAN;
uint32_t lastScdReadMs = 0;

uint32_t lastSensorMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastOledMs = 0;
uint32_t txSequence = 1;

float lastPacketRssiDbm = NAN;
float lastPacketSnrDb = NAN;
String lastPacketType = "none";
String lastPacketSource = "none";

// ---------------------------------------------------------------------------
// Small utilities
// ---------------------------------------------------------------------------

float clampf(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float scoreBetween(float value, float low, float high, float points) {
  if (!isfinite(value) || high <= low) return 0.0f;
  return clampf((value - low) / (high - low), 0.0f, 1.0f) * points;
}

const char* modeName(AwarenessMode mode) {
  switch (mode) {
    case AwarenessMode::Quiet: return "quiet";
    case AwarenessMode::Watching: return "watching";
    case AwarenessMode::Aware: return "aware";
    case AwarenessMode::Alert: return "alert";
  }
  return "unknown";
}

void appendReason(String& reasons, const char* reason) {
  if (reasons.length() > 0) reasons += ",";
  reasons += reason;
}

bool tcaSelect(uint8_t channel) {
  if (channel > 7) return false;

  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  const uint8_t result = Wire.endTransmission();
  delayMicroseconds(300);
  return result == 0;
}

bool tcaDisableAll() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0);
  return Wire.endTransmission() == 0;
}

void scanI2cBus(TwoWire& bus, const char* label) {
  Serial.printf("%s I2C scan:", label);
  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; ++addr) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      Serial.printf(" 0x%02X", addr);
      found++;
    }
  }

  if (found == 0) {
    Serial.print(" none");
  }
  Serial.println();
}

int rssiToBars(float rssiDbm) {
  if (!isfinite(rssiDbm)) return 0;
  if (rssiDbm >= -70.0f) return 5;
  if (rssiDbm >= -85.0f) return 4;
  if (rssiDbm >= -100.0f) return 3;
  if (rssiDbm >= -112.0f) return 2;
  if (rssiDbm >= -125.0f) return 1;
  return 0;
}

void drawSignalBars(int x, int y, int bars) {
  if (!oledOk) return;

  for (int i = 0; i < 5; ++i) {
    const int barHeight = 4 + i * 4;
    const int barX = x + i * 7;
    const int barY = y + 20 - barHeight;
    if (i < bars) {
      display.fillRect(barX, barY, 5, barHeight, SSD1306_WHITE);
    } else {
      display.drawRect(barX, barY, 5, barHeight, SSD1306_WHITE);
    }
  }
}

void displaySignalOnOled(const char* statusLine) {
  if (!oledOk) return;

  const int bars = rssiToBars(lastPacketRssiDbm);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(NODE_ID);

  display.setCursor(0, 12);
  display.print("LoRa ");
  display.print(loraOk ? "OK" : "NO");
  display.print("  Gate ");
  display.print(awareness.gateOpen ? "OPEN" : "SHUT");

  display.setCursor(0, 24);
  display.print("Mode ");
  display.print(modeName(awareness.mode));
  display.print(" ");
  display.print(awareness.score, 0);

  drawSignalBars(88, 6, bars);

  display.setCursor(0, 36);
  display.print("RSSI ");
  if (isfinite(lastPacketRssiDbm)) {
    display.print(lastPacketRssiDbm, 0);
    display.print(" dBm");
  } else {
    display.print("-- dBm");
  }

  display.setCursor(0, 48);
  display.print("GPS ");
  display.print(gnss.hasFix ? "FIX" : "NO");
  display.print(" sat ");
  display.print(gnss.sats);
  display.print(" ");
  if (lastPacketType.length() > 6) {
    display.print(lastPacketType.substring(0, 6));
  } else {
    display.print(lastPacketType);
  }

  display.setCursor(0, 56);
  display.print(statusLine);

  display.display();
}

void setupOled() {
  pinMode(PIN_OLED_POWER, OUTPUT);
  digitalWrite(PIN_OLED_POWER, OLED_POWER_ON_LEVEL);
  delay(100);

  OledWire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  OledWire.setClock(400000);
  scanI2cBus(OledWire, "OLED");

  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  if (!oledOk) {
    Serial.println("OLED: not found");
    return;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Heltec aware node");
  display.setCursor(0, 12);
  display.print("Firmware flashed");
  display.setCursor(0, 26);
  display.print("Signal display ready");
  drawSignalBars(88, 24, 5);
  display.setCursor(0, 54);
  display.print(NODE_ID);
  display.display();

  Serial.println("OLED: ok");
}

void setupGnss() {
  if (!GNSS_ENABLED) {
    Serial.println("GNSS: disabled");
    return;
  }

  GnssSerial.begin(GNSS_BAUD, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
  gnssOk = true;
  Serial.printf(
    "GNSS: UART ready baud=%lu rx=%d tx=%d\n",
    static_cast<unsigned long>(GNSS_BAUD),
    PIN_GNSS_RX,
    PIN_GNSS_TX
  );
}

void updateGnss() {
  if (!gnssOk) return;

  while (GnssSerial.available() > 0) {
    gps.encode(static_cast<char>(GnssSerial.read()));
  }

  gnss.chars = gps.charsProcessed();
  gnss.sentences = gps.sentencesWithFix();
  gnss.failedChecksum = gps.failedChecksum();
  gnss.sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  gnss.hdop = gps.hdop.isValid() ? gps.hdop.value() : 0;

  if (gps.location.isUpdated()) {
    gnss.lastUpdateMs = millis();
    gnss.locationValid = gps.location.isValid();
    gnss.lat = gps.location.lat();
    gnss.lon = gps.location.lng();
    gnss.ageMs = gps.location.age();
  } else if (gps.location.isValid()) {
    gnss.locationValid = true;
    gnss.lat = gps.location.lat();
    gnss.lon = gps.location.lng();
    gnss.ageMs = gps.location.age();
  }

  if (gps.altitude.isValid()) {
    gnss.altitudeM = gps.altitude.meters();
  }
  if (gps.speed.isValid()) {
    gnss.speedKmph = gps.speed.kmph();
  }

  const bool recentlyUpdated =
    gnss.lastUpdateMs != 0 && millis() - gnss.lastUpdateMs <= GNSS_STALE_MS;
  gnss.hasFix = gnss.locationValid && gps.location.isValid() && recentlyUpdated;
}

// ---------------------------------------------------------------------------
// Sensor setup
// ---------------------------------------------------------------------------

void setupBme280() {
  if (!tcaSelect(TCA_CH_BME280)) {
    Serial.println("BME280: TCA channel unavailable");
    return;
  }

  bmeOk = bme280.begin(0x76) || bme280.begin(0x77);
  Serial.printf("BME280: %s\n", bmeOk ? "ok" : "not found");
}

void setupVl53l0x() {
  if (!tcaSelect(TCA_CH_VL53L0X)) {
    Serial.println("VL53L0X: TCA channel unavailable");
    return;
  }

  vl53Ok = vl53.begin();
  Serial.printf("VL53L0X: %s\n", vl53Ok ? "ok" : "not found");
}

void setupAmg8833() {
  if (!tcaSelect(TCA_CH_AMG8833)) {
    Serial.println("AMG8833: TCA channel unavailable");
    return;
  }

  amgOk = amg8833.begin();
  Serial.printf("AMG8833: %s\n", amgOk ? "ok" : "not found");
}

void setupScd40() {
  if (!tcaSelect(TCA_CH_SCD40)) {
    Serial.println("SCD40: TCA channel unavailable");
    return;
  }

  scd40.begin(Wire, SCD40_ADDR);
  uint16_t error = scd40.stopPeriodicMeasurement();
  delay(500);
  error = scd40.startPeriodicMeasurement();
  scdOk = error == 0;

  Serial.printf("SCD40: %s\n", scdOk ? "ok" : "not found");
}

void setupI2sMic() {
  i2s_config_t config;
  memset(&config, 0, sizeof(config));
  config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX);
  config.sample_rate = 16000;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 4;
  config.dma_buf_len = 256;
  config.use_apll = false;
  config.tx_desc_auto_clear = false;
  config.fixed_mclk = 0;

  i2s_pin_config_t pins;
  memset(&pins, 0, sizeof(pins));
#if ESP_IDF_VERSION_MAJOR >= 5
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
#endif
  pins.bck_io_num = PIN_I2S_BCLK;
  pins.ws_io_num = PIN_I2S_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = PIN_I2S_DIN;

  esp_err_t err = i2s_driver_install(I2S_PORT, &config, 0, nullptr);
  if (err == ESP_OK) {
    err = i2s_set_pin(I2S_PORT, &pins);
  }
  if (err == ESP_OK) {
    i2s_zero_dma_buffer(I2S_PORT);
  }

  i2sOk = err == ESP_OK;
  Serial.printf("ICS43434 I2S: %s\n", i2sOk ? "ok" : "failed");
}

// ---------------------------------------------------------------------------
// LoRa setup and packet IO
// ---------------------------------------------------------------------------

#if defined(ESP32)
void IRAM_ATTR onLoraDio1() {
#else
void onLoraDio1() {
#endif
  loraPacketReady = true;
}

bool startLoRaReceive() {
  if (!loraOk) return false;
  int16_t state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa startReceive failed: %d\n", state);
    return false;
  }
  return true;
}

void setupLoRa() {
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);

  int16_t state = radio.begin(LORA_FREQ_MHZ);
  if (state == RADIOLIB_ERR_NONE) {
    radio.setBandwidth(LORA_BW_KHZ);
    radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
    radio.setCodingRate(LORA_CODING_RATE);
    radio.setSyncWord(LORA_SYNC_WORD);
    radio.setOutputPower(LORA_TX_POWER_DBM);
    radio.setCRC(true);
    radio.setDio1Action(onLoraDio1);
    loraOk = startLoRaReceive();
  }

  Serial.printf("SX1262 LoRa: %s", loraOk ? "ok" : "failed");
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf(" (%d)", state);
  }
  Serial.println();
}

PacketView parsePacket(const String& raw) {
  PacketView packet;

  const int p0 = raw.indexOf('|');
  const int p1 = raw.indexOf('|', p0 + 1);
  const int p2 = raw.indexOf('|', p1 + 1);
  const int p3 = raw.indexOf('|', p2 + 1);
  if (p0 <= 0 || p1 <= p0 || p2 <= p1 || p3 <= p2) {
    return packet;
  }

  if (raw.substring(0, p0) != "LP1") {
    return packet;
  }

  packet.type = raw.substring(p0 + 1, p1);
  packet.source = raw.substring(p1 + 1, p2);
  packet.sequence = static_cast<uint32_t>(raw.substring(p2 + 1, p3).toInt());
  packet.body = raw.substring(p3 + 1);
  packet.type.toUpperCase();
  packet.valid = packet.type.length() > 0 && packet.source.length() > 0;
  return packet;
}

bool typeIsAlwaysAllowed(const String& type) {
  return type == "ALERT" || type == "WAKE" || type == "PING";
}

bool typeRequiresAwareness(const String& type) {
  return type == "DATA" || type == "TELEM" || type == "COMMAND";
}

bool shouldAcceptPacket(const PacketView& packet, float rssiDbm) {
  if (!packet.valid) return false;
  if (rssiDbm < MIN_ACCEPT_RSSI_DBM) return false;
  if (typeIsAlwaysAllowed(packet.type)) return true;
  if (typeRequiresAwareness(packet.type)) return awareness.gateOpen;
  return awareness.gateOpen && awareness.mode != AwarenessMode::Quiet;
}

String buildTelemetryPayload(const char* type) {
  String body;
  body.reserve(220);
  body += "mode=";
  body += modeName(awareness.mode);
  body += ";score=";
  body += String(awareness.score, 1);
  body += ";gate=";
  body += awareness.gateOpen ? "1" : "0";
  body += ";reasons=";
  body += awareness.reasons.length() ? awareness.reasons : "none";
  body += ";dist_mm=";
  body += currentFrame.distanceValid ? String(currentFrame.distanceMm) : "nan";
  body += ";thermal_max_c=";
  body += String(currentFrame.thermalMaxC, 1);
  body += ";thermal_delta_c=";
  body += String(currentFrame.thermalDeltaC, 1);
  body += ";audio_rms=";
  body += String(currentFrame.audioRms, 4);
  body += ";co2_ppm=";
  body += currentFrame.co2Ppm ? String(currentFrame.co2Ppm) : "0";
  body += ";temp_c=";
  body += String(currentFrame.bmeTempC, 1);
  body += ";hum_pct=";
  body += String(currentFrame.bmeHumidityPct, 1);
  body += ";gnss_fix=";
  body += gnss.hasFix ? "1" : "0";
  body += ";gnss_sats=";
  body += String(gnss.sats);
  if (gnss.locationValid) {
    body += ";lat=";
    body += String(gnss.lat, 6);
    body += ";lon=";
    body += String(gnss.lon, 6);
    body += ";alt_m=";
    body += String(gnss.altitudeM, 1);
    body += ";spd_kmph=";
    body += String(gnss.speedKmph, 1);
  }

  String packet;
  packet.reserve(body.length() + 48);
  packet += "LP1|";
  packet += type;
  packet += "|";
  packet += NODE_ID;
  packet += "|";
  packet += txSequence++;
  packet += "|";
  packet += body;
  return packet;
}

bool transmitPacket(const String& packet) {
  if (!loraOk) return false;

  String mutablePacket = packet;
  radio.clearDio1Action();
  int16_t state = radio.transmit(mutablePacket);
  radio.setDio1Action(onLoraDio1);
  startLoRaReceive();

  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("TX ok: %s\n", packet.c_str());
    return true;
  }

  Serial.printf("TX failed: %d\n", state);
  return false;
}

void handleAcceptedPacket(const PacketView& packet, float rssiDbm, float snrDb) {
  Serial.printf(
    "RX accepted type=%s source=%s seq=%lu rssi=%.1f snr=%.1f body=%s\n",
    packet.type.c_str(),
    packet.source.c_str(),
    static_cast<unsigned long>(packet.sequence),
    rssiDbm,
    snrDb,
    packet.body.c_str()
  );

  lastPacketType = packet.type;
  lastPacketSource = packet.source;
  lastPacketRssiDbm = rssiDbm;
  lastPacketSnrDb = snrDb;
  displaySignalOnOled("RX accepted");

  if (packet.type == "PING") {
    transmitPacket(buildTelemetryPayload("ACK"));
  } else if (packet.type == "WAKE") {
    awareness.lastInterestingMs = millis();
    awareness.gateOpen = true;
  }
}

void handleLoRaReceive() {
  if (!loraOk || !loraPacketReady) return;
  loraPacketReady = false;

  String raw;
  const int16_t state = radio.readData(raw);
  const float rssiDbm = radio.getRSSI();
  const float snrDb = radio.getSNR();

  if (state == RADIOLIB_ERR_NONE) {
    PacketView packet = parsePacket(raw);
    const bool accepted = shouldAcceptPacket(packet, rssiDbm);

    if (accepted) {
      handleAcceptedPacket(packet, rssiDbm, snrDb);
    } else {
      lastPacketType = packet.valid ? packet.type : "invalid";
      lastPacketSource = packet.valid ? packet.source : "unknown";
      lastPacketRssiDbm = rssiDbm;
      lastPacketSnrDb = snrDb;
      displaySignalOnOled("RX ignored");

      Serial.printf(
        "RX ignored valid=%d gate=%d mode=%s rssi=%.1f raw=%s\n",
        packet.valid ? 1 : 0,
        awareness.gateOpen ? 1 : 0,
        modeName(awareness.mode),
        rssiDbm,
        raw.c_str()
      );
    }
  } else if (state != RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.printf("RX read failed: %d\n", state);
  }

  startLoRaReceive();
}

// ---------------------------------------------------------------------------
// Sensor reads
// ---------------------------------------------------------------------------

float readAudioRms(float* dbfsOut) {
  if (dbfsOut) *dbfsOut = NAN;
  if (!i2sOk) return NAN;

  int32_t samples[256];
  size_t bytesRead = 0;
  esp_err_t err = i2s_read(
    I2S_PORT,
    samples,
    sizeof(samples),
    &bytesRead,
    pdMS_TO_TICKS(8)
  );
  if (err != ESP_OK || bytesRead == 0) return NAN;

  const size_t count = bytesRead / sizeof(samples[0]);
  double sumSquares = 0.0;

  for (size_t i = 0; i < count; ++i) {
    const int32_t sample24 = samples[i] >> 8;
    const float normalized = static_cast<float>(sample24) / 8388608.0f;
    sumSquares += static_cast<double>(normalized) * normalized;
  }

  const float rms = sqrt(sumSquares / static_cast<double>(count));
  if (dbfsOut) {
    *dbfsOut = 20.0f * log10f(rms + 1.0e-9f);
  }
  return rms;
}

void readBme280(SensorFrame& frame) {
  if (!bmeOk || !tcaSelect(TCA_CH_BME280)) return;

  frame.bmeTempC = bme280.readTemperature();
  frame.bmeHumidityPct = bme280.readHumidity();
  frame.pressureHpa = bme280.readPressure() / 100.0f;
}

void readVl53l0x(SensorFrame& frame) {
  if (!vl53Ok || !tcaSelect(TCA_CH_VL53L0X)) return;

  VL53L0X_RangingMeasurementData_t measurement;
  vl53.rangingTest(&measurement, false);

  if (measurement.RangeStatus != 4) {
    frame.distanceValid = true;
    frame.distanceMm = measurement.RangeMilliMeter;
  }
}

void readAmg8833(SensorFrame& frame) {
  if (!amgOk || !tcaSelect(TCA_CH_AMG8833)) return;

  float pixels[64];
  amg8833.readPixels(pixels);

  float minC = pixels[0];
  float maxC = pixels[0];
  float sumC = 0.0f;
  float maxDeltaC = 0.0f;

  for (uint8_t i = 0; i < 64; ++i) {
    const float value = pixels[i];
    minC = min(minC, value);
    maxC = max(maxC, value);
    sumC += value;

    if (haveLastThermalPixels) {
      maxDeltaC = max(maxDeltaC, fabsf(value - lastThermalPixels[i]));
    }
    lastThermalPixels[i] = value;
  }

  haveLastThermalPixels = true;

  const float avgC = sumC / 64.0f;
  uint8_t hotPixels = 0;
  for (uint8_t i = 0; i < 64; ++i) {
    if (pixels[i] > avgC + 3.0f) hotPixels++;
  }

  frame.thermalAvgC = avgC;
  frame.thermalMinC = minC;
  frame.thermalMaxC = maxC;
  frame.thermalSpanC = maxC - minC;
  frame.thermalDeltaC = maxDeltaC;
  frame.thermalHotPixels = hotPixels;
}

void readScd40(SensorFrame& frame) {
  if (!scdOk || !tcaSelect(TCA_CH_SCD40)) {
    frame.co2Ppm = lastCo2Ppm;
    frame.scdTempC = lastScdTempC;
    frame.scdHumidityPct = lastScdHumidityPct;
    return;
  }

  const uint32_t now = millis();
  if (now - lastScdReadMs >= 5000 || lastScdReadMs == 0) {
    uint16_t co2 = 0;
    float tempC = NAN;
    float humidityPct = NAN;
    const uint16_t error = scd40.readMeasurement(co2, tempC, humidityPct);

    if (error == 0 && co2 != 0) {
      lastCo2Ppm = co2;
      lastScdTempC = tempC;
      lastScdHumidityPct = humidityPct;
    }
    lastScdReadMs = now;
  }

  frame.co2Ppm = lastCo2Ppm;
  frame.scdTempC = lastScdTempC;
  frame.scdHumidityPct = lastScdHumidityPct;
}

SensorFrame readSensors() {
  SensorFrame frame;
  frame.ms = millis();

  readBme280(frame);
  readVl53l0x(frame);
  readAmg8833(frame);
  readScd40(frame);
  frame.audioRms = readAudioRms(&frame.audioDbfs);

  return frame;
}

// ---------------------------------------------------------------------------
// Awareness logic
// ---------------------------------------------------------------------------

void updateAwareness(const SensorFrame& frame) {
  String reasons;
  float score = 0.0f;
  uint8_t evidenceCount = 0;

  if (isfinite(frame.audioRms)) {
    if (!isfinite(awareness.audioBaseline)) {
      awareness.audioBaseline = frame.audioRms;
    }

    const float ratio = frame.audioRms / max(awareness.audioBaseline, 0.0005f);
    const bool audioEvent = frame.audioRms >= AUDIO_RMS_AWARE || ratio >= 3.0f;

    if (audioEvent) {
      score += scoreBetween(frame.audioRms, AUDIO_RMS_AWARE, AUDIO_RMS_ALERT, 22.0f);
      if (ratio >= 3.0f) score += 8.0f;
      appendReason(reasons, "audio");
      evidenceCount++;
      awareness.audioBaseline = 0.995f * awareness.audioBaseline + 0.005f * frame.audioRms;
    } else {
      awareness.audioBaseline = 0.98f * awareness.audioBaseline + 0.02f * frame.audioRms;
    }
  }

  if (frame.distanceValid) {
    if (frame.distanceMm < NEAR_OBJECT_MM) {
      score += scoreBetween(
        static_cast<float>(NEAR_OBJECT_MM - frame.distanceMm),
        0.0f,
        static_cast<float>(NEAR_OBJECT_MM - VERY_NEAR_OBJECT_MM),
        26.0f
      );
      appendReason(reasons, "near");
      evidenceCount++;
    }

    if (frame.distanceMm <= VERY_NEAR_OBJECT_MM) {
      score += 14.0f;
      appendReason(reasons, "close");
    }
  }

  if (isfinite(frame.thermalSpanC)) {
    const bool thermalShape = frame.thermalSpanC >= THERMAL_SPAN_AWARE_C || frame.thermalHotPixels >= 2;
    const bool thermalMotion = frame.thermalDeltaC >= THERMAL_DELTA_AWARE_C;

    if (thermalShape) {
      score += scoreBetween(frame.thermalSpanC, THERMAL_SPAN_AWARE_C, 8.0f, 18.0f);
      score += (frame.thermalHotPixels < 8 ? frame.thermalHotPixels : 8) * 1.5f;
      appendReason(reasons, "thermal");
      evidenceCount++;
    }

    if (thermalMotion) {
      score += scoreBetween(frame.thermalDeltaC, THERMAL_DELTA_AWARE_C, 4.0f, 16.0f);
      appendReason(reasons, "thermal_delta");
      evidenceCount++;
    }

    if (frame.thermalMaxC >= THERMAL_HOT_ALERT_C) {
      score += 30.0f;
      appendReason(reasons, "hot");
    }
  }

  if (frame.co2Ppm >= CO2_AWARE_PPM) {
    score += scoreBetween(static_cast<float>(frame.co2Ppm), CO2_AWARE_PPM, CO2_ALERT_PPM, 18.0f);
    appendReason(reasons, "co2");
    evidenceCount++;
  }

  if (isfinite(frame.bmeTempC) && frame.bmeTempC >= 35.0f) {
    score += scoreBetween(frame.bmeTempC, 35.0f, 45.0f, 12.0f);
    appendReason(reasons, "ambient_hot");
    evidenceCount++;
  }

  if (evidenceCount >= 2) score += 10.0f;
  if (evidenceCount >= 3) score += 10.0f;

  awareness.score = clampf(score, 0.0f, 100.0f);
  awareness.reasons = reasons;

  if (awareness.score >= 75.0f || frame.co2Ppm >= CO2_ALERT_PPM) {
    awareness.mode = AwarenessMode::Alert;
  } else if (awareness.score >= 45.0f) {
    awareness.mode = AwarenessMode::Aware;
  } else if (awareness.score >= 20.0f) {
    awareness.mode = AwarenessMode::Watching;
  } else {
    awareness.mode = AwarenessMode::Quiet;
  }

  if (awareness.mode == AwarenessMode::Aware || awareness.mode == AwarenessMode::Alert) {
    awareness.lastInterestingMs = frame.ms;
  }

  awareness.gateOpen =
    awareness.mode == AwarenessMode::Aware ||
    awareness.mode == AwarenessMode::Alert ||
    (awareness.lastInterestingMs != 0 && frame.ms - awareness.lastInterestingMs < AWARE_GATE_HOLD_MS);
}

void printFrameSummary(const SensorFrame& frame) {
  Serial.printf(
    "sense mode=%s score=%.1f gate=%d reasons=%s dist=%s thermal=%.1f/%.1f delta=%.1f audio=%.4f co2=%u temp=%.1f hum=%.1f gnss=%s sats=%lu lat=%.6f lon=%.6f\n",
    modeName(awareness.mode),
    awareness.score,
    awareness.gateOpen ? 1 : 0,
    awareness.reasons.length() ? awareness.reasons.c_str() : "none",
    frame.distanceValid ? String(frame.distanceMm).c_str() : "nan",
    frame.thermalAvgC,
    frame.thermalMaxC,
    frame.thermalDeltaC,
    frame.audioRms,
    frame.co2Ppm,
    frame.bmeTempC,
    frame.bmeHumidityPct,
    gnss.hasFix ? "fix" : "no",
    static_cast<unsigned long>(gnss.sats),
    gnss.lat,
    gnss.lon
  );
}

void maybeTransmitTelemetry() {
  const uint32_t now = millis();
  const bool interesting = awareness.mode == AwarenessMode::Aware || awareness.mode == AwarenessMode::Alert;

  if (interesting && now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    transmitPacket(buildTelemetryPayload(awareness.mode == AwarenessMode::Alert ? "ALERT" : "TELEM"));
    return;
  }

  if (SEND_IDLE_HEARTBEAT && !interesting && now - lastHeartbeatMs >= IDLE_HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    transmitPacket(buildTelemetryPayload("HEARTBEAT"));
  }
}

// ---------------------------------------------------------------------------
// Arduino lifecycle
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("Heltec LoRa V4 aware node booting");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  scanI2cBus(Wire, "Sensor");

  setupOled();
  setupGnss();

  if (!tcaDisableAll()) {
    Serial.println("TCA9548A: not found at 0x70");
  } else {
    Serial.println("TCA9548A: ok");
  }

  setupBme280();
  setupVl53l0x();
  setupAmg8833();
  setupScd40();
  tcaDisableAll();

  setupI2sMic();
  setupLoRa();

  Serial.println("Boot complete");
  displaySignalOnOled("Boot complete");
}

void loop() {
  updateGnss();
  handleLoRaReceive();

  const uint32_t now = millis();
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS || lastSensorMs == 0) {
    lastSensorMs = now;
    currentFrame = readSensors();
    updateAwareness(currentFrame);
    printFrameSummary(currentFrame);
    maybeTransmitTelemetry();
  }

  if (oledOk && now - lastOledMs >= OLED_INTERVAL_MS) {
    lastOledMs = now;
    displaySignalOnOled("Running");
  }

  delay(2);
}
