#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <RTClib.h>
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "Soil_Sensor.h"
#include "A7680_ThingsBoard.h"

// ================= USER CONFIG =================
static const char* APN              = "";
static const char* SIM_PIN          = nullptr;
static const char* TB_HOST          = "nguyendinhdat.io.vn";
static const uint16_t TB_PORT       = 1883;
static const char* TB_DEVICE_TOKEN  = "GMwTEv4U420UMNRapGS0";

// Cấu hình chân Module SIM A7680S (UART2)
static const int MODEM_RX_PIN = 22;
static const int MODEM_TX_PIN = 23;
static const uint32_t MODEM_BAUD = 115200;

// Cấu hình chân Module RTC DS3231 (I2C)
#define I2C_SDA 19
#define I2C_SCL 18

// ===== FreeRTOS config =====
static const BaseType_t APP_TASK_CORE           = 1;
static const UBaseType_t TASK_PRIO_SENSOR       = 2;
static const UBaseType_t TASK_PRIO_THINGSBOARD  = 4;
static const uint32_t TASK_STACK_SENSOR         = 4096;
static const uint32_t TASK_STACK_THINGSBOARD    = 12288;

// ===== Chu kỳ chạy =====
static const uint32_t SENSOR_READ_INTERVAL_MS      = 10000UL;      // 10 giây
static const uint32_t TELEMETRY_SEND_INTERVAL_MS   = 3000UL;       // 3 giây
static const uint32_t ATTR_REQUEST_INTERVAL_MS     = 60000UL;
static const uint32_t RTC_RESYNC_INTERVAL_MS       = 43200000UL;   // 12h
static const uint32_t RTC_RETRY_INTERVAL_MS        = 60000UL;      // 1 phút
static const uint32_t SIM_CHECK_INTERVAL_MS        = 3000UL;
static const uint32_t RECONNECT_INTERVAL_MS        = 5000UL;
static const uint32_t MODEM_RECOVERY_INTERVAL_MS   = 15000UL;
static const uint32_t MODEM_BOOT_WAIT_MS           = 12000UL;

// ===== Buffer size =====
static const size_t MODEM_RESP_BUF_SIZE = 256;
static const size_t TELEMETRY_BUF_SIZE  = 512;

// ===============================================

HardwareSerial ModemSerial(2);
A7680Modem modem(ModemSerial);
A7680MqttClient mqtt(modem);
A7680ThingsBoard tb(mqtt);

RTC_DS3231 rtc;

// ===== FreeRTOS objects =====
TaskHandle_t gTaskSensorHandle = nullptr;
TaskHandle_t gTaskTbHandle     = nullptr;

SemaphoreHandle_t gSensorMutex = nullptr;
SemaphoreHandle_t gRtcMutex    = nullptr;

// ===== Shared sensor snapshot =====
struct SensorSnapshot {
  bool valid;
  char sampleTime[30];
  float moisture;
  float temperature;
  float conductivity;
  float ph;
  float nitro;
  float phospho;
  float pota;
  float salinity;
  float tds;
  uint32_t sampleCounter;
};

static SensorSnapshot gLatestSensorData = {0};

// ===== Modem / network state =====
enum SimState {
  SIM_STATE_UNKNOWN = 0,
  SIM_STATE_READY,
  SIM_STATE_PIN_REQUIRED,
  SIM_STATE_NOT_INSERTED,
  SIM_STATE_NOT_READY,
  SIM_STATE_MODEM_NO_RESPONSE
};

static bool needReconnect              = true;
static bool wasMqttConnected           = false;
static bool lastKnownSimReady          = false;
static uint8_t consecutiveConnectFails = 0;

static uint32_t lastReconnectAttemptMs = 0;
static uint32_t lastSimCheckMs         = 0;
static uint32_t lastModemRecoveryMs    = 0;
static uint32_t lastRtcSyncMs          = 0;

// ============================================================
// UTILS
// ============================================================
const char* simStateToString(SimState s) {
  switch (s) {
    case SIM_STATE_READY:             return "READY";
    case SIM_STATE_PIN_REQUIRED:      return "SIM PIN REQUIRED";
    case SIM_STATE_NOT_INSERTED:      return "NOT INSERTED";
    case SIM_STATE_NOT_READY:         return "NOT READY";
    case SIM_STATE_MODEM_NO_RESPONSE: return "MODEM NO RESPONSE";
    default:                          return "UNKNOWN";
  }
}

void formatDateTime(const DateTime& dt, char* out, size_t outSize) {
  snprintf(out, outSize, "%02d:%02d:%02d %02d/%02d/%04d",
           dt.hour(), dt.minute(), dt.second(),
           dt.day(), dt.month(), dt.year());
}

bool getRtcNow(DateTime& out) {
  if (gRtcMutex == nullptr) return false;

  if (xSemaphoreTake(gRtcMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    out = rtc.now();
    xSemaphoreGive(gRtcMutex);
    return true;
  }
  return false;
}

bool adjustRtcSafe(const DateTime& dt) {
  if (gRtcMutex == nullptr) return false;

  if (xSemaphoreTake(gRtcMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
    rtc.adjust(dt);
    xSemaphoreGive(gRtcMutex);
    return true;
  }
  return false;
}

bool getLatestSensorSnapshot(SensorSnapshot& out) {
  if (gSensorMutex == nullptr) return false;

  if (xSemaphoreTake(gSensorMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    out = gLatestSensorData;
    xSemaphoreGive(gSensorMutex);
    return out.valid;
  }
  return false;
}

bool saveLatestSensorSnapshot(const SensorSnapshot& in) {
  if (gSensorMutex == nullptr) return false;

  if (xSemaphoreTake(gSensorMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    gLatestSensorData = in;
    xSemaphoreGive(gSensorMutex);
    return true;
  }
  return false;
}

// ============================================================
// MODEM I/O - CHAR BUFFER VERSION
// ============================================================
void flushModemInput(uint32_t drainMs = 50) {
  uint32_t start = millis();
  while (millis() - start < drainMs) {
    while (ModemSerial.available()) {
      ModemSerial.read();
    }
    delay(1);
  }
}

size_t readModemResponse(char* out, size_t outSize, uint32_t timeoutMs) {
  if (out == nullptr || outSize == 0) return 0;

  out[0] = '\0';
  size_t idx = 0;
  uint32_t start = millis();
  uint32_t lastRx = millis();

  while (millis() - start < timeoutMs) {
    bool gotData = false;

    while (ModemSerial.available()) {
      char c = (char)ModemSerial.read();
      gotData = true;
      lastRx = millis();

      if (idx < (outSize - 1)) {
        out[idx++] = c;
        out[idx] = '\0';
      }
    }

    if (strstr(out, "\r\nOK\r\n") != nullptr || strstr(out, "\r\nERROR\r\n") != nullptr) {
      break;
    }

    if (idx > 0 && !gotData && (millis() - lastRx > 150)) {
      break;
    }

    delay(1);
  }

  return idx;
}

bool sendAT(const char* cmd, char* resp, size_t respSize, uint32_t timeoutMs = 2000, bool verbose = true) {
  if (cmd == nullptr || resp == nullptr || respSize == 0) return false;

  flushModemInput(20);

  if (verbose) {
    Serial.print("[AT] ");
    Serial.println(cmd);
  }

  ModemSerial.println(cmd);
  readModemResponse(resp, respSize, timeoutMs);

  if (verbose) {
    Serial.println(resp);
  }

  return true;
}

bool waitForModemAlive(uint32_t timeoutMs) {
  char resp[MODEM_RESP_BUF_SIZE];
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    sendAT("AT", resp, sizeof(resp), 800, false);
    if (strstr(resp, "OK") != nullptr) {
      return true;
    }
    delay(300);
  }

  Serial.println("[MODEM] Khong phan hoi lenh AT.");
  return false;
}

void configureModemBasic() {
  char resp[MODEM_RESP_BUF_SIZE];
  sendAT("ATE0",      resp, sizeof(resp), 1000, false);
  sendAT("AT+CMEE=2", resp, sizeof(resp), 1000, false);
  sendAT("AT+CTZU=1", resp, sizeof(resp), 1000, false);
  sendAT("AT+CTZR=1", resp, sizeof(resp), 1000, false);
}

SimState getSimState() {
  char resp[MODEM_RESP_BUF_SIZE];

  if (!waitForModemAlive(1500)) {
    return SIM_STATE_MODEM_NO_RESPONSE;
  }

  sendAT("AT+CPIN?", resp, sizeof(resp), 2000, false);

  if (strstr(resp, "+CPIN: READY")      != nullptr) return SIM_STATE_READY;
  if (strstr(resp, "SIM PIN")           != nullptr) return SIM_STATE_PIN_REQUIRED;
  if (strstr(resp, "NOT INSERTED")      != nullptr) return SIM_STATE_NOT_INSERTED;
  if (strstr(resp, "NOT READY")         != nullptr) return SIM_STATE_NOT_READY;
  if (strstr(resp, "OK")                != nullptr) return SIM_STATE_UNKNOWN;

  return SIM_STATE_UNKNOWN;
}

// ============================================================
// MODEM RECOVERY
// ============================================================
bool softRecoverModemForHotSwap() {
  char resp[MODEM_RESP_BUF_SIZE];

  Serial.println("\n[RECOVERY] Thu phuc hoi modem sau hot-swap SIM...");

  sendAT("AT+CFUN=0", resp, sizeof(resp), 3000, true);
  delay(2000);
  sendAT("AT+CFUN=1", resp, sizeof(resp), 5000, true);
  delay(6000);

  if (waitForModemAlive(6000)) {
    configureModemBasic();
    SimState sim = getSimState();
    Serial.printf("[RECOVERY] SIM state sau CFUN 0/1: %s\n", simStateToString(sim));
    if (sim == SIM_STATE_READY) {
      return true;
    }
  }

  Serial.println("[RECOVERY] CFUN 0/1 chua du -> reboot modem bang AT+CFUN=1,1");
  sendAT("AT+CFUN=1,1", resp, sizeof(resp), 3000, true);
  delay(MODEM_BOOT_WAIT_MS);

  if (!waitForModemAlive(12000)) {
    Serial.println("[RECOVERY] Modem chua song lai sau CFUN=1,1");
    return false;
  }

  configureModemBasic();
  SimState sim = getSimState();
  Serial.printf("[RECOVERY] SIM state sau CFUN=1,1: %s\n", simStateToString(sim));
  return (sim == SIM_STATE_READY);
}

// ============================================================
// TIME SYNC - CHAR VERSION
// ============================================================
bool syncTimeFromSIM() {
  char response[MODEM_RESP_BUF_SIZE];

  if (!waitForModemAlive(1500)) {
    return false;
  }

  sendAT("AT+CCLK?", response, sizeof(response), 2500, false);
  Serial.printf("[CCLK RAW] %s\n", response);

  const char* tag = "+CCLK: \"";
  char* start = strstr(response, tag);
  if (start == nullptr) return false;
  start += strlen(tag);

  char* endQuote = strchr(start, '"');
  if (endQuote == nullptr) return false;

  char raw[32];
  size_t rawLen = (size_t)(endQuote - start);
  if (rawLen >= sizeof(raw)) rawLen = sizeof(raw) - 1;

  memcpy(raw, start, rawLen);
  raw[rawLen] = '\0';

  int yy = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
  int tzQuarter = 28; // mặc định UTC+7 Việt Nam

  int matched = sscanf(raw, "%2d/%2d/%2d,%2d:%2d:%2d%d",
                       &yy, &month, &day, &hour, &minute, &second, &tzQuarter);

  if (matched < 6) {
    Serial.println("[SYNC] Khong parse duoc +CCLK.");
    return false;
  }

  int year = yy + 2000;

  if (!(year >= 2024 && year <= 2050 &&
        month >= 1 && month <= 12 &&
        day >= 1 && day <= 31 &&
        hour >= 0 && hour <= 23 &&
        minute >= 0 && minute <= 59 &&
        second >= 0 && second <= 59)) {
    Serial.println("[SYNC] Du lieu thoi gian khong hop le.");
    return false;
  }

  DateTime simTime(year, month, day, hour, minute, second);

  const int VN_TZ_QUARTER = 28;
  long deltaSeconds = (VN_TZ_QUARTER - tzQuarter) * 15L * 60L;
  DateTime vnTime(simTime.unixtime() + deltaSeconds);

  if (!adjustRtcSafe(vnTime)) {
    Serial.println("[SYNC] Khong lock duoc RTC mutex.");
    return false;
  }

  Serial.printf("[SYNC] SIM raw: %s\n", raw);
  Serial.printf("[SYNC] tzQuarter: %d\n", tzQuarter);
  Serial.printf("[SYNC] RTC VN: %02d:%02d:%02d %02d/%02d/%04d\n",
                vnTime.hour(), vnTime.minute(), vnTime.second(),
                vnTime.day(), vnTime.month(), vnTime.year());

  return true;
}

// ============================================================
// THINGSBOARD CALLBACKS
// ============================================================
void onRpcRequest(uint32_t requestId, const String& payload) {
  tb.sendRpcResponse(requestId, "{\"success\":true,\"echo\":true}");
}

void onAttrUpdate(const String& payload) {
  (void)payload;
}

void onAttrResponse(uint32_t requestId, const String& payload) {
  (void)requestId;
  (void)payload;
}

// ============================================================
// THINGSBOARD SEND WRAPPER
// ============================================================
// Nếu thư viện của bạn có hàm nhận const char*,
// hãy thay nội dung hàm này bằng:
// return tb.sendTelemetryJson(payload);
bool sendTelemetryJsonCompat(const char* payload) {
  if (payload == nullptr) return false;
  return tb.sendTelemetryJson(String(payload));
}

// ============================================================
// MQTT / CLOUD
// ============================================================
void cleanupMqttSession() {
  char resp[MODEM_RESP_BUF_SIZE];

  sendAT("AT+CMQTTDISC=0,60", resp, sizeof(resp), 1500, false);
  delay(200);
  sendAT("AT+CMQTTREL=0",     resp, sizeof(resp), 1500, false);
  delay(200);
  sendAT("AT+CMQTTSTOP",      resp, sizeof(resp), 1500, false);
  delay(400);

  while (ModemSerial.available()) {
    ModemSerial.read();
  }

  mqtt.stop();
}

bool connectCellularAndThingsBoard() {
  Serial.println("\n[NET] Bat dau ket noi lai ThingsBoard...");

  if (!waitForModemAlive(3000)) {
    Serial.println("[NET] Modem khong song.");
    return false;
  }

  configureModemBasic();

  SimState sim = getSimState();
  Serial.printf("[SIM] Trang thai truoc khi connect: %s\n", simStateToString(sim));

  if (sim != SIM_STATE_READY) {
    Serial.println("[NET] SIM chua san sang, bo qua lan connect nay.");
    return false;
  }

  cleanupMqttSession();

  if (!modem.beginNetwork(APN, SIM_PIN, 90000)) {
    Serial.println("[NET] beginNetwork() that bai.");
    return false;
  }

  if (!tb.connect(TB_HOST, TB_PORT, TB_DEVICE_TOKEN, "esp32-a7680s-demo")) {
    Serial.println("[MQTT] tb.connect() that bai.");
    return false;
  }

  if (!tb.subscribeAttributes()) {
    Serial.println("[MQTT] subscribeAttributes() that bai.");
    cleanupMqttSession();
    return false;
  }

  if (!tb.subscribeRpc()) {
    Serial.println("[MQTT] subscribeRpc() that bai.");
    cleanupMqttSession();
    return false;
  }

  Serial.println("[MQTT] Da ket noi ThingsBoard thanh cong.");
  wasMqttConnected = true;
  needReconnect = false;
  consecutiveConnectFails = 0;
  lastKnownSimReady = true;
  return true;
}

void forceOfflineAndRecover(const char* reason) {
  Serial.print("\n[OFFLINE] ");
  Serial.println(reason);

  cleanupMqttSession();
  wasMqttConnected = false;
  needReconnect = true;
}

void handleRecoveryManager() {
  uint32_t now = millis();

  if (mqtt.isConnected()) {
    wasMqttConnected = true;
    needReconnect = false;
    return;
  }

  if (wasMqttConnected) {
    Serial.println("[NET] MQTT vua bi mat ket noi.");
    wasMqttConnected = false;
    needReconnect = true;
  }

  if (lastSimCheckMs == 0 || (now - lastSimCheckMs >= SIM_CHECK_INTERVAL_MS)) {
    lastSimCheckMs = now;

    SimState sim = getSimState();
    bool simReady = (sim == SIM_STATE_READY);

    if (simReady != lastKnownSimReady) {
      Serial.printf("[SIM] Trang thai thay doi -> %s\n", simStateToString(sim));
      lastKnownSimReady = simReady;
    }

    if (!simReady) {
      if (lastModemRecoveryMs == 0 || (now - lastModemRecoveryMs >= MODEM_RECOVERY_INTERVAL_MS)) {
        lastModemRecoveryMs = now;
        Serial.printf("[SIM] SIM chua READY (%s). Thu recover modem...\n", simStateToString(sim));
        softRecoverModemForHotSwap();
      }
      return;
    }
  }

  if (needReconnect && (lastReconnectAttemptMs == 0 || (now - lastReconnectAttemptMs >= RECONNECT_INTERVAL_MS))) {
    lastReconnectAttemptMs = now;

    if (connectCellularAndThingsBoard()) {
      Serial.println("[NET] Reconnect thanh cong.");
      return;
    }

    consecutiveConnectFails++;
    Serial.printf("[NET] Reconnect that bai lan %u\n", consecutiveConnectFails);

    if (consecutiveConnectFails >= 3) {
      consecutiveConnectFails = 0;
      lastModemRecoveryMs = now;
      Serial.println("[NET] Qua 3 lan connect fail -> recover modem them 1 lan.");
      softRecoverModemForHotSwap();
    }
  }
}

// ============================================================
// SENSOR TASK
// ============================================================
void TaskSensor(void* pvParameters) {
  (void)pvParameters;
  Serial.println("[TASK] Sensor task started");

  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t sampleCounter = 0;

  for (;;) {
    Read_Sensor(0x02, TOTAL_DISSOLVE_SOLIDS_TDS);

    DateTime sampleDt;
    char sampleTime[30] = "00:00:00 00/00/0000";
    if (getRtcNow(sampleDt)) {
      formatDateTime(sampleDt, sampleTime, sizeof(sampleTime));
    }

    SensorSnapshot snapshot = {};
    snapshot.valid = true;
    strncpy(snapshot.sampleTime, sampleTime, sizeof(snapshot.sampleTime) - 1);
    snapshot.sampleTime[sizeof(snapshot.sampleTime) - 1] = '\0';

    snapshot.moisture      = moisture;
    snapshot.temperature   = Temp;
    snapshot.conductivity  = conductivity;
    snapshot.ph            = ph;
    snapshot.nitro         = nitro;
    snapshot.phospho       = phospho;
    snapshot.pota          = pota;
    snapshot.salinity      = salinity;
    snapshot.tds           = TDS;
    snapshot.sampleCounter = ++sampleCounter;

    if (saveLatestSensorSnapshot(snapshot)) {
      Serial.printf("[SENSOR] Mau #%lu | %s | Temp=%.2f | Moist=%.2f | pH=%.2f\n",
                    (unsigned long)snapshot.sampleCounter,
                    snapshot.sampleTime,
                    snapshot.temperature,
                    snapshot.moisture,
                    snapshot.ph);
    } else {
      Serial.println("[SENSOR] Loi luu snapshot.");
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
  }
}

// ============================================================
// THINGSBOARD TASK
// ============================================================
void TaskThingsBoard(void* pvParameters) {
  (void)pvParameters;
  Serial.println("[TASK] ThingsBoard task started");

  uint32_t lastTelemetrySendMs = 0;
  uint32_t lastAttrReqMs       = 0;
  lastRtcSyncMs                = 0;

  connectCellularAndThingsBoard();

  for (;;) {
    tb.loop(50);

    if (!mqtt.isConnected()) {
      handleRecoveryManager();
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    uint32_t now = millis();

    if (lastRtcSyncMs == 0 || (now - lastRtcSyncMs >= RTC_RESYNC_INTERVAL_MS)) {
      if (syncTimeFromSIM()) {
        lastRtcSyncMs = now;
      } else {
        lastRtcSyncMs = now - RTC_RESYNC_INTERVAL_MS + RTC_RETRY_INTERVAL_MS;
      }
    }

    if (now - lastTelemetrySendMs >= TELEMETRY_SEND_INTERVAL_MS) {
      lastTelemetrySendMs = now;

      SensorSnapshot snap;
      if (!getLatestSensorSnapshot(snap)) {
        Serial.println("[TB] Chua co du lieu cam bien hop le, bo qua lan gui nay.");
      } else {
        char telemetry[TELEMETRY_BUF_SIZE];

        int len = snprintf(
          telemetry, sizeof(telemetry),
          "{"
            "\"ThoiGian\":\"%s\","
            "\"DoAm\":%.2f,"
            "\"NhietDo\":%.2f,"
            "\"DoDanDien\":%.2f,"
            "\"Ph\":%.2f,"
            "\"Nito\":%.2f,"
            "\"PhotPho\":%.2f,"
            "\"Kali\":%.2f,"
            "\"DoMan\":%.2f,"
            "\"TongChatRanHoaTan\":%.2f,"
            
          "}",
          snap.sampleTime,
          snap.moisture,
          snap.temperature,
          snap.conductivity,
          snap.ph,
          snap.nitro,
          snap.phospho,
          snap.pota,
          snap.salinity,
          snap.tds
         
        );

        if (len <= 0 || len >= (int)sizeof(telemetry)) {
          Serial.println("[TB] Tao JSON bi loi hoac bi tran buffer.");
        } else {
          if (!sendTelemetryJsonCompat(telemetry)) {
            forceOfflineAndRecover("sendTelemetryJson() that bai");
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
          } else {
            Serial.printf("[TB] Gui OK | Mau #%lu | %s\n",
                          (unsigned long)snap.sampleCounter, snap.sampleTime);
          }
        }
      }
    }

    if (now - lastAttrReqMs >= ATTR_REQUEST_INTERVAL_MS) {
      lastAttrReqMs = now;
      uint32_t reqId = now / 1000;
      tb.requestAttributes(reqId, "fw_version,serial_number", "targetTemp,ledState");
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== ESP32 + A7680S + ThingsBoard + FreeRTOS + CHAR BUFFER =====");

  gSensorMutex = xSemaphoreCreateMutex();
  gRtcMutex    = xSemaphoreCreateMutex();

  if (gSensorMutex == nullptr || gRtcMutex == nullptr) {
    Serial.println("[INIT] Tao mutex that bai!");
    while (1) {
      delay(1000);
    }
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  if (rtc.begin()) {
    if (rtc.lostPower()) {
      adjustRtcSafe(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println("[RTC] RTC lost power -> set build time.");
    } else {
      Serial.println("[RTC] RTC ready.");
    }
  } else {
    Serial.println("[RTC] Khong tim thay DS3231");
  }

  UART_CONFIG();
  Serial.println("[INIT] Sensor UART ready.");

  ModemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(1500);
  Serial.println("[INIT] Modem UART ready.");

  configureModemBasic();

  tb.setRpcCallback(onRpcRequest);
  tb.setAttributesCallback(onAttrUpdate);
  tb.setAttributesResponseCallback(onAttrResponse);

  xTaskCreatePinnedToCore(
    TaskSensor,
    "TaskSensor",
    TASK_STACK_SENSOR,
    nullptr,
    TASK_PRIO_SENSOR,
    &gTaskSensorHandle,
    APP_TASK_CORE
  );

  xTaskCreatePinnedToCore(
    TaskThingsBoard,
    "TaskThingsBoard",
    TASK_STACK_THINGSBOARD,
    nullptr,
    TASK_PRIO_THINGSBOARD,
    &gTaskTbHandle,
    APP_TASK_CORE
  );

  if (gTaskSensorHandle == nullptr || gTaskTbHandle == nullptr) {
    Serial.println("[INIT] Tao task that bai!");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("[INIT] All tasks created.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}




// #include <Arduino.h>
// #include <HardwareSerial.h>
// #include <Wire.h>
// #include <RTClib.h>
// #include "Soil_Sensor.h"
// #include "A7680_ThingsBoard.h"

// // ================= USER CONFIG =================
// static const char* APN              = "";
// static const char* SIM_PIN          = nullptr;
// static const char* TB_HOST          = "nguyendinhdat.io.vn";
// static const uint16_t TB_PORT       = 1883;
// static const char* TB_DEVICE_TOKEN  = "GMwTEv4U420UMNRapGS0";

// // Cấu hình chân Module SIM A7680S (UART2)
// static const int MODEM_RX_PIN = 22;
// static const int MODEM_TX_PIN = 23;
// static const uint32_t MODEM_BAUD = 115200;

// // Cấu hình chân Module RTC DS3231 (I2C)
// #define I2C_SDA 19
// #define I2C_SCL 18
// // ===============================================

// HardwareSerial ModemSerial(2); 
// A7680Modem modem(ModemSerial);
// A7680MqttClient mqtt(modem);
// A7680ThingsBoard tb(mqtt);

// RTC_DS3231 rtc;

// uint32_t lastTelemetryMs = 0;
// uint32_t lastAttrReqMs   = 0;
// uint32_t rpcCounter      = 0;

// String getESP32_UID() {
//   uint64_t chipid = ESP.getEfuseMac(); 
//   char uid_str[20];
//   snprintf(uid_str, sizeof(uid_str), "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
//   return String(uid_str);
// }

// bool syncTimeFromSIM() {
//   while (ModemSerial.available()) ModemSerial.read();

//   ModemSerial.println("AT+CCLK?");
//   unsigned long timeout = millis() + 2000;
//   String response = "";

//   while (millis() < timeout) {
//     while (ModemSerial.available()) {
//       response += (char)ModemSerial.read();
//     }
//   }

//   Serial.println("[CCLK RAW] " + response);

//   int index = response.indexOf("+CCLK: \"");
//   if (index == -1) return false;

//   int start = index + 8;
//   int endQuote = response.indexOf('"', start);
//   if (endQuote == -1) return false;

//   // Ví dụ: 26/04/07,13:48:28+32
//   String raw = response.substring(start, endQuote);

//   if (raw.length() < 17) return false;

//   int year   = raw.substring(0, 2).toInt() + 2000;
//   int month  = raw.substring(3, 5).toInt();
//   int day    = raw.substring(6, 8).toInt();
//   int hour   = raw.substring(9, 11).toInt();
//   int minute = raw.substring(12, 14).toInt();
//   int second = raw.substring(15, 17).toInt();

//   if (!(year >= 2024 && year <= 2050 &&
//         month >= 1 && month <= 12 &&
//         day >= 1 && day <= 31 &&
//         hour >= 0 && hour <= 23 &&
//         minute >= 0 && minute <= 59 &&
//         second >= 0 && second <= 59)) {
//     Serial.println("[SYNC] Du lieu thoi gian khong hop le.");
//     return false;
//   }

//   // Parse timezone quarter-hour
//   // +28 = UTC+7, +32 = UTC+8, +24 = UTC+6
//   int tzQuarter = 28; // mặc định VN nếu không parse được
//   if (raw.length() >= 20) {
//     char sign = raw.charAt(17);       // '+' hoặc '-'
//     int q = raw.substring(18).toInt(); // 28, 32, 24...
//     tzQuarter = (sign == '-') ? -q : q;
//   }

//   DateTime simTime(year, month, day, hour, minute, second);

//   // Quy đổi về múi giờ Việt Nam UTC+7 = +28 quarter-hours
//   const int VN_TZ_QUARTER = 28;
//   long deltaSeconds = (VN_TZ_QUARTER - tzQuarter) * 15L * 60L;

//   DateTime vnTime(simTime.unixtime() + deltaSeconds);

//   rtc.adjust(vnTime);

//   Serial.printf("[SYNC] SIM raw: %s\n", raw.c_str());
//   Serial.printf("[SYNC] tzQuarter: %d\n", tzQuarter);
//   Serial.printf("[SYNC] RTC VN: %02d:%02d:%02d %02d/%02d/%04d\n",
//                 vnTime.hour(), vnTime.minute(), vnTime.second(),
//                 vnTime.day(), vnTime.month(), vnTime.year());

//   return true;
// }
// // ================= CALLBACK THINGSBOARD =================
// void onRpcRequest(uint32_t requestId, const String& payload) {
//   // Trả lời ThingsBoard khi có lệnh điều khiển gửi xuống
//   tb.sendRpcResponse(requestId, "{\"success\":true,\"echo\":true}");
// }

// void onAttrUpdate(const String& payload) {
//   // Xử lý khi có cập nhật Attribute từ Web (Hiện tại để trống)
// }

// void onAttrResponse(uint32_t requestId, const String& payload) {
//   // Xử lý phản hồi Attribute (Hiện tại để trống)
// }

// // ================= HÀM KẾT NỐI VÀ DỌN DẸP =================
// void cleanupMqttSession() {
//   ModemSerial.println("AT+CMQTTDISC=0,60");
//   delay(500);
//   ModemSerial.println("AT+CMQTTREL=0");
//   delay(500);
//   ModemSerial.println("AT+CMQTTSTOP");
//   delay(1000);
//   while(ModemSerial.available()) {
//     ModemSerial.read();
//   }
//   mqtt.stop(); 
// }

// bool connectCellularAndThingsBoard() {
//   cleanupMqttSession();

//   if (!modem.beginNetwork(APN, SIM_PIN, 90000)) return false;
//   if (!tb.connect(TB_HOST, TB_PORT, TB_DEVICE_TOKEN, "esp32-a7680s-demo")) return false;
//   if (!tb.subscribeAttributes()) return false;
//   if (!tb.subscribeRpc()) return false;

//   return true;
// }

// // ================= SETUP =================
// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   // Khởi tạo I2C và kết nối RTC DS3231
//   Wire.begin(I2C_SDA, I2C_SCL);
//   if (rtc.begin()) {
//     if (rtc.lostPower()) {
//       rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
//     }
//   }

//   // Khởi tạo UART cho Cảm biến đất
//   UART_CONFIG();

//   // Khởi tạo UART cho Modem SIM
//   ModemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
//   delay(1200); 

//   // Kích hoạt tính năng tự cập nhật giờ từ trạm phát sóng di động (NITZ)
//   ModemSerial.println("AT+CTZU=1");
//   delay(500);

//   tb.setRpcCallback(onRpcRequest);
//   tb.setAttributesCallback(onAttrUpdate);
//   tb.setAttributesResponseCallback(onAttrResponse);

//   connectCellularAndThingsBoard();
// }

// // ================= LOOP =================
// void loop() {
//   tb.loop(50);

//   // Tự động kết nối lại nếu rớt mạng MQTT
//   if (!mqtt.isConnected()) {
//     delay(3000);
//     connectCellularAndThingsBoard();
//     return;
//   }

//   uint32_t now = millis();
//   static uint32_t lastRtcSyncMs = 0;

//   // Đồng bộ giờ từ SIM sang DS3231 mỗi 12 tiếng hoặc ngay khi khởi động
//   // (43200000 ms = 12 tiếng, 60000 ms = 1 phút)
//   if (now - lastRtcSyncMs >= 43200000 || lastRtcSyncMs == 0) {
//     if (syncTimeFromSIM()) {
//       lastRtcSyncMs = now; // Nếu lấy mạng thành công, chờ 12 tiếng nữa mới đồng bộ lại
//     } else {
//       lastRtcSyncMs = now - 43200000 + 60000; // Thất bại, ép thử lại sau 1 phút
//     }
//   }

//   // Đọc và gửi dữ liệu cảm biến
//   if (now - lastTelemetryMs >= 1000) { // Lưu ý: Nên tăng thời gian này lên để tránh spam server 
//     lastTelemetryMs = now;

//     // 1. ĐỌC THỜI GIAN LUÔN LUÔN TỪ MODULE RTC DS3231 (Cực kỳ an toàn)
//     DateTime now_rtc = rtc.now();
//     char timeBuffer[30];
//     snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d %02d/%02d/%04d", 
//              now_rtc.hour(), now_rtc.minute(), now_rtc.second(), 
//              now_rtc.day(), now_rtc.month(), now_rtc.year());
    
//     // 2. ĐỌC CẢM BIẾN NPK 9-IN-1
//     Read_Sensor(0x02, TOTAL_DISSOLVE_SOLIDS_TDS);

//     // 3. TẠO CHUỖI JSON 
//     String telemetry = "{";
//     telemetry += "\"ThoiGian\":\""         + String(timeBuffer) + "\","; 
//     telemetry += "\"DoAm\":"               + String(moisture, 2) + ",";
//     telemetry += "\"NhietDo\":"            + String(Temp, 2) + ",";
//     telemetry += "\"DoDanDien\":"          + String(conductivity, 2) + ",";
//     telemetry += "\"Ph\":"                 + String(ph, 2) + ",";
//     telemetry += "\"Nito\":"               + String(nitro, 2) + ",";
//     telemetry += "\"PhotPho\":"            + String(phospho, 2) + ",";
//     telemetry += "\"Kali\":"               + String(pota, 2) + ",";
//     telemetry += "\"DoMan\":"              + String(salinity, 2) + ",";
//     telemetry += "\"TongChatRanHoaTan\":"  + String(TDS, 2); 
//     telemetry += "}";

//     // 4. GỬI LÊN THINGSBOARD VÀ XỬ LÝ LỖI
//     if (!tb.sendTelemetryJson(telemetry)) {
//       cleanupMqttSession();
//       delay(1000);
//       connectCellularAndThingsBoard();
//     }
//   }

//   // Request attributes mỗi 60 giây
//   if (now - lastAttrReqMs >= 60000) {
//     lastAttrReqMs = now;
//     uint32_t reqId = now / 1000;
//     tb.requestAttributes(reqId, "fw_version,serial_number", "targetTemp,ledState");
//   }
// }