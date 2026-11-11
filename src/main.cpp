#include <Arduino.h>
#include "config.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Update.h>
#include <PZEM004Tv30.h>
#include <Preferences.h>
#include "driver/gpio.h"

extern "C" void esp_brownout_init(void);
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ========================================================
// HARDWARE OBJECTS & RE-ENTRANT HANDLERS
// ========================================================
AsyncWebServer server(80);
// HardwareSerial 1 untuk PZEM di ESP32-S3 (RX=17, TX=18)
HardwareSerial PzemSerial(1);
Preferences prefs;

// ========================================================
// PZEM RAW MODBUS ENGINE & CRC16 HELPER
// ========================================================
static uint16_t modbusCRC16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

static uint32_t s_pzemSleepMicros = 0;

static bool rawReadPzem(float &volt, float &curr, float &pwr, float &nrg, float &frq, float &pf, uint8_t targetAddr) {
    uint8_t query[8] = {targetAddr, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00};
    uint16_t crc = modbusCRC16(query, 6);
    query[6] = crc & 0xFF;
    query[7] = (crc >> 8) & 0xFF;

    while (PzemSerial.available()) PzemSerial.read(); // Flush RX buffer
    PzemSerial.write(query, 8);
    PzemSerial.flush();

    DBG_PRINTF("[PZEM RAW TX] Sent 8B to Addr 0x%02X: %02X %02X %02X %02X %02X %02X %02X %02X\n",
               targetAddr, query[0], query[1], query[2], query[3], query[4], query[5], query[6], query[7]);

    uint8_t resp[32];
    uint8_t idx = 0;
    uint32_t start = millis();
    // 150ms window: 25 byte pada 9600 baud hanya butuh 26ms. 150ms sangat cukup dan tidak bikin CPU lag
    while ((millis() - start < 150) && (idx < 25)) {
        if (PzemSerial.available()) {
            resp[idx++] = PzemSerial.read();
        } else {
            uint32_t slpStart = micros();
            vTaskDelay(pdMS_TO_TICKS(5));
            s_pzemSleepMicros += (micros() - slpStart);
        }
    }

    if (idx > 0) {
        DBG_PRINTF("[PZEM RAW RX] ✅ Got %d bytes from Addr 0x%02X: ", idx, targetAddr);
        for (uint8_t i = 0; i < idx; i++) DBG_PRINTF("%02X ", resp[i]);
        DBG_PRINTLN();
    } else {
        DBG_PRINTF("[PZEM RAW RX] ⚠️ Timeout 0 bytes from Addr 0x%02X (TX_Pin=%d, RX_Pin=%d)\n", targetAddr, PIN_PZEM_TX, PIN_PZEM_RX);
        return false;
    }

    if (idx >= 25) {
        uint16_t calcCrc = modbusCRC16(resp, 23);
        uint16_t respCrc = (uint16_t)resp[23] | ((uint16_t)resp[24] << 8);
        if (calcCrc == respCrc) {
            volt = (float)(((uint16_t)resp[3] << 8) | resp[4]) / 10.0f;
            
            // PZEM-004T v4.0 Modbus 32-bit: Low Word (Byte 5-6), High Word (Byte 7-8)
            uint32_t rawCurr = ((uint32_t)resp[7] << 24) | ((uint32_t)resp[8] << 16) |
                               ((uint32_t)resp[5] << 8)  | (uint32_t)resp[6];
            curr = (float)rawCurr / 1000.0f;

            uint32_t rawPwr = ((uint32_t)resp[11] << 24) | ((uint32_t)resp[12] << 16) |
                              ((uint32_t)resp[9] << 8)   | (uint32_t)resp[10];
            pwr = (float)rawPwr / 10.0f;

            uint32_t rawNrg = ((uint32_t)resp[15] << 24) | ((uint32_t)resp[16] << 16) |
                              ((uint32_t)resp[13] << 8)  | (uint32_t)resp[14];
            nrg = (float)rawNrg / 1000.0f;

            frq  = (float)(((uint16_t)resp[17] << 8) | resp[18]) / 10.0f;
            pf   = (float)(((uint16_t)resp[19] << 8) | resp[20]) / 100.0f;
            DBG_PRINTF("[PZEM PARSED] ✅ V=%.1fV | A=%.2fA | W=%.1fW | Wh=%.1f | Hz=%.1f | PF=%.2f\n",
                       volt, curr, pwr, nrg, frq, pf);
            return true;
        } else {
            DBG_PRINTF("[PZEM PARSED] ❌ CRC Mismatch! Calc: %04X vs Resp: %04X\n", calcCrc, respCrc);
        }
    }
    return false;
}

static bool rawResetPzemEnergy(uint8_t targetAddr) {
    uint8_t cmd[4] = {targetAddr, 0x42, 0x00, 0x00};
    uint16_t crc = modbusCRC16(cmd, 2);
    cmd[2] = crc & 0xFF;
    cmd[3] = (crc >> 8) & 0xFF;
    PzemSerial.write(cmd, 4);
    PzemSerial.flush();
    DBG_PRINTF("[PZEM RESET] Sent Reset Energy command to Addr 0x%02X\n", targetAddr);
    return true;
}

// FreeRTOS IPC Handlers
QueueHandle_t telemetryQueue;
QueueHandle_t pduCommandQueue;
SemaphoreHandle_t relayMutex;

// System Cache (Thread-Safe Access)
TelemetryData currentTelemetry = {0};
bool targetRelayStates[4] = {false, false, false, false};
uint32_t relayCycleCount[4] = {0, 0, 0, 0};

// Protection Thresholds (Loaded from Preferences)
float g_threshUvlo   = DEFAULT_UVLO_VOLT;
float g_threshSurge  = DEFAULT_SURGE_VOLT;
float g_threshOcp    = DEFAULT_OCP_CURRENT;
float g_threshFmin   = DEFAULT_FREQ_MIN;
float g_threshFmax   = DEFAULT_FREQ_MAX;
uint32_t g_seqDelayMs = SEQUENCER_DELAY_MS;
uint32_t g_btnHoldMs  = BTN_HOLD_TOGGLE_MS;
uint32_t g_softStartDelayMs = DEFAULT_SOFTSTART_DELAY_MS;

// Blynk Modular Configuration (NVS Loaded)
String g_blynkAuthToken = BLYNK_AUTH_TOKEN;
String g_blynkTemplateId = BLYNK_TEMPLATE_ID;
bool g_blynkEnabled = true;
volatile bool g_blynkNeedReconnect = false;

// Klien ESP-NOW Management
ClientNodeInfo clientList[ESPNOW_MAX_CLIENTS];
uint8_t registeredClientCount = 0;
bool isPairingWindowActive = false;
uint32_t pairingStartTime = 0;

// Sub-PDU Lt2 Cloud Sync Cache (Thread-Safe Decoupling)
volatile uint8_t g_subPduLt2RelayState = 0;
volatile bool g_blynkSyncSubPdu = false;
static uint32_t blynkPressStart[4] = {0, 0, 0, 0};
static uint32_t blynkMasterPressStart = 0;

// Wi-Fi Runtime Credentials (Live Soft-Reconnect tanpa restart)
String g_staSsid = "";
String g_staPass = "";
volatile bool g_wifiNeedReconnect = false;

// ========================================================
// ========================================================
// PSRAM 1-SECOND TELEMETRY LOGGER & BLACKBOX NVS
// ========================================================
TelemetryHistoryPoint *psramHistoryBuffer = nullptr;
uint16_t psramHistoryHead = 0;
uint16_t psramHistoryCount = 0;
portMUX_TYPE historyMux = portMUX_INITIALIZER_UNLOCKED;

// Boot Tracking & 10-Event Ring Buffer di Flash NVS
uint32_t g_bootCount = 0;
esp_reset_reason_t g_mcuResetReason = ESP_RST_UNKNOWN;
BlackboxRecord g_tripRecords[BLACKBOX_MAX_RECORDS] = {};
uint8_t g_tripCount = 0;
uint8_t g_tripHead = 0;
BlackboxRecord g_lastTripRecord = {};
bool g_hasTripRecord = false;

// ========================================================
// RTC FAST MEMORY BREADCRUMBS (Survives WDT & Panic Resets)
// ========================================================
RTC_NOINIT_ATTR static uint32_t rtcMagic;
RTC_NOINIT_ATTR static uint8_t  crumbNet;   // Posisi terakhir Core 0 (Network & Blynk)
RTC_NOINIT_ATTR static uint8_t  crumbCtl;   // Posisi terakhir Core 1 (Control & PZEM)
RTC_NOINIT_ATTR static uint32_t lastUptimeS;
RTC_NOINIT_ATTR static uint32_t rtcBootCount;

// Snapshot diagnostik boot sebelumnya (diekspos ke /api/status & UI)
uint32_t g_lastRstReason = 0;
uint8_t  g_lastCrumbNet  = 0;
uint8_t  g_lastCrumbCtl  = 0;
uint32_t g_lastUptimeS   = 0;

// Rolling Window 5 Detik (10 Sampel @ 500ms) untuk Rekam Min/Max Voltase sebelum Trip
#define VOLT_WINDOW_5S_SIZE 10
static float g_voltWindow5s[VOLT_WINDOW_5S_SIZE] = {0};
static uint8_t g_voltWindowIdx = 0;
static uint8_t g_voltWindowCount = 0;
static portMUX_TYPE voltWindowMux = portMUX_INITIALIZER_UNLOCKED;

void recordVoltSample5s(float v) {
    if (v > 0.0f) {
        portENTER_CRITICAL(&voltWindowMux);
        g_voltWindow5s[g_voltWindowIdx] = v;
        g_voltWindowIdx = (g_voltWindowIdx + 1) % VOLT_WINDOW_5S_SIZE;
        if (g_voltWindowCount < VOLT_WINDOW_5S_SIZE) g_voltWindowCount++;
        portEXIT_CRITICAL(&voltWindowMux);
    }
}

void getVoltMinMax5s(float &minV, float &maxV, float fallbackV) {
    portENTER_CRITICAL(&voltWindowMux);
    float lo = 999.0f;
    float hi = 0.0f;
    for (uint8_t i = 0; i < g_voltWindowCount; i++) {
        float sample = g_voltWindow5s[i];
        if (sample > 0.0f) {
            if (sample < lo) lo = sample;
            if (sample > hi) hi = sample;
        }
    }
    portEXIT_CRITICAL(&voltWindowMux);

    if (lo > 900.0f) lo = fallbackV;
    if (hi == 0.0f) hi = fallbackV;
    minV = lo;
    maxV = hi;
}

// Driver RGB Status Visualizer (GPIO 48) - Dibatasi Kecerahan Maksimal 40%
void updateRgbStatus(SystemState sys, SequencerState seq, float temp) {
    if (sys == STATE_UVLO || sys == STATE_SURGE || sys == STATE_OCP) {
        neopixelWrite(PIN_RGB_LED, 102, 0, 0); // Merah Terang 40% (Trip Kelistrikan)
    } else if (temp >= 70.0f) {
        neopixelWrite(PIN_RGB_LED, 102, 0, 40); // Magenta/Merah Panas 40% (Overheating Box)
    } else if (temp >= 58.0f) {
        neopixelWrite(PIN_RGB_LED, 102, 35, 0);  // Oranye 40% (Peringatan Hangat)
    } else if (seq == SEQ_POWERON_STABILIZING || seq == SEQ_STARTING_UP || seq == SEQ_SHUTTING_DOWN) {
        neopixelWrite(PIN_RGB_LED, 70, 70, 0);   // Kuning 40% (Sequencer / Stabilisasi)
    } else if (sys == STATE_FREQ_FAULT) {
        neopixelWrite(PIN_RGB_LED, 80, 30, 0);   // Oranye Peringatan Frekuensi 40%
    } else {
        neopixelWrite(PIN_RGB_LED, 0, 50, 0);    // Hijau Normal Sejuk (Lembut 40%)
    }
}

// Status Indikator CPU
volatile float g_estimatedCpuLoad = 3.0f; // Core 1 Load (Control & PZEM)
volatile float g_core0Load = 3.0f;        // Core 0 Load (Network & Web)
volatile bool g_cpuActivityPulse = false;

// Helper teks penyebab reset hardware MCU
const char* getResetReasonStr(uint8_t code) {
    switch ((esp_reset_reason_t)code) {
        case ESP_RST_POWERON:   return "Power-On Reset (Daya Masuk / Padam)";
        case ESP_RST_EXT:       return "External Pin Reset (Tombol RST Fisik)";
        case ESP_RST_SW:        return "Software Reboot (OTA / Web Command)";
        case ESP_RST_PANIC:     return "Core Panic / Exception Crash";
        case ESP_RST_INT_WDT:   return "Interrupt Watchdog Reset";
        case ESP_RST_TASK_WDT:  return "Task Watchdog Reset (Task Freeze/Hang)";
        case ESP_RST_WDT:       return "Other Watchdog Reset";
        case ESP_RST_DEEPSLEEP: return "Deep-Sleep Wakeup";
        case ESP_RST_BROWNOUT:  return "Brownout Reset (Tegangan DC Drop < 2.8V)";
        case ESP_RST_SDIO:      return "SDIO Reset";
        default:                return "Hardware Reset";
    }
}

// Helper teks pelacak posisi terakhir breadcrumb RTC
const char* getCrumbNetStr(uint8_t c) {
    switch (c) {
        case 1: return "WiFi & Blynk Housekeeping";
        case 2: return "Check WiFi STA";
        case 3: return "Blynk.connect() (Blocking Socket / DNS)";
        case 4: return "Blynk.run()";
        case 5: return "WiFi Reconnecting";
        case 6: return "Telemetry Push to Blynk";
        case 7: return "ESP-NOW & Pairing Check";
        default: return "Idle / Normal";
    }
}

const char* getCrumbCtlStr(uint8_t c) {
    switch (c) {
        case 1: return "Command Queue & Mutex";
        case 2: return "Soft Start Monitor";
        case 3: return "Scan Button Fisik";
        case 4: return "PZEM Modbus UART Read";
        case 5: return "Evaluasi Proteksi & Cutoff";
        case 6: return "Sequencer FSM";
        default: return "Idle / Normal";
    }
}

// Simpan Blackbox ke NVS Flash saat terjadi insiden (Ring Buffer 10 Kejadian + Min/Max 5s)
void saveBlackboxTrip(SystemState reason, const char* label, float v, float a, float w, float temp) {
    float minV = v, maxV = v;
    getVoltMinMax5s(minV, maxV, v);

    BlackboxRecord rec = {};
    rec.magic = 0x50445542; // "PDUB"
    rec.bootCount = g_bootCount;
    rec.timestamp = millis() / 1000;
    rec.tripReason = (uint8_t)reason;
    rec.rstReason = (uint8_t)g_mcuResetReason;
    rec.voltage = v;
    rec.current = a;
    rec.power = w;
    rec.chipTemp = temp;
    rec.minVolt5s = minV;
    rec.maxVolt5s = maxV;
    uint8_t rBits = 0;
    for (int i = 0; i < 4; i++) { if (targetRelayStates[i]) rBits |= (1 << i); }
    rec.relayStates = rBits;
    strncpy(rec.tripLabel, label, sizeof(rec.tripLabel) - 1);

    Preferences pBb;
    pBb.begin("pdu_bb", false);

    // 1. Simpan ke slot ring buffer (trip_0 .. trip_9)
    String key = "trip_" + String(g_tripHead);
    pBb.putBytes(key.c_str(), &rec, sizeof(rec));

    // 2. Simpan juga legacy key "last_trip" untuk backwards compatibility
    pBb.putBytes("last_trip", &rec, sizeof(rec));

    g_tripRecords[g_tripHead] = rec;
    g_tripHead = (g_tripHead + 1) % BLACKBOX_MAX_RECORDS;
    if (g_tripCount < BLACKBOX_MAX_RECORDS) g_tripCount++;

    pBb.putUChar("head", g_tripHead);
    pBb.putUChar("count", g_tripCount);
    pBb.end();

    g_lastTripRecord = rec;
    g_hasTripRecord = true;
    Serial.printf("[BLACKBOX NVS] 🚨 Insiden Disimpan ke Slot #%d (Total: %d): %s\n"
                  "               V_Trip: %.1fV (Min5s: %.1fV, Max5s: %.1fV) | Arus: %.2fA | Suhu: %.1fC | Boot #%u | Reset: %s\n",
                  (g_tripHead == 0 ? BLACKBOX_MAX_RECORDS - 1 : g_tripHead - 1), g_tripCount,
                  label, v, minV, maxV, a, temp, g_bootCount, getResetReasonStr(rec.rstReason));
}

// HTTP Basic Authentication Helper
static bool checkAuth(AsyncWebServerRequest *request) {
    if (!request->authenticate(WEB_ADMIN_USER, WEB_ADMIN_PASS)) {
        request->requestAuthentication();
        return false;
    }
    return true;
}

// Mapping nama ramah untuk jenis node
String getNodeTypeName(NodeType type) {
    switch (type) {
        case NODE_SUB_PDU_LT2:   return "Sub-PDU Lt2";
        case NODE_SENSOR_ESP01:  return "Sensor ESP-01";
        case NODE_RELAY_ESP01:   return "Relay ESP-01";
        default:                 return "Sub-Node";
    }
}

// Logging Helper
void logEvent(const char *msg) {
    File f = LittleFS.open("/log.txt", FILE_APPEND);
    if (f) {
        f.printf("[%lu s] %s\n", millis() / 1000, msg);
        f.close();
    }
    Serial.printf("[BLACKBOX LOG] %s\n", msg);
}

// ========================================================
// AUDIO BUZZER CONTROLLER (NON-BLOCKING)
// ========================================================
void updateBuzzer(SystemState state) {
    static uint32_t lastBuzzerToggle = 0;
    static bool buzzerPinState = false;
    uint32_t now = millis();

    switch (state) {
        case STATE_NORMAL:
            if (buzzerPinState) {
                DBG_PIN_WRITE(PIN_BUZZER, LOW, "Buzzer Alarm");
                buzzerPinState = false;
            }
            break;

        case STATE_UVLO:
            // 1000ms ON, 200ms OFF (Beep panjang, jeda pendek)
            if (buzzerPinState && (now - lastBuzzerToggle >= 1000)) {
                DBG_PIN_WRITE(PIN_BUZZER, LOW, "Buzzer UVLO");
                buzzerPinState = false;
                lastBuzzerToggle = now;
            } else if (!buzzerPinState && (now - lastBuzzerToggle >= 200)) {
                DBG_PIN_WRITE(PIN_BUZZER, HIGH, "Buzzer UVLO");
                buzzerPinState = true;
                lastBuzzerToggle = now;
            }
            break;

        case STATE_OCP:
            // 150ms ON, 150ms OFF (Beep cepat kontinu)
            if (now - lastBuzzerToggle >= 150) {
                buzzerPinState = !buzzerPinState;
                DBG_PIN_WRITE(PIN_BUZZER, buzzerPinState ? HIGH : LOW, "Buzzer OCP");
                lastBuzzerToggle = now;
            }
            break;

        case STATE_SURGE:
            // 500ms ON, 200ms OFF
            if (buzzerPinState && (now - lastBuzzerToggle >= 500)) {
                DBG_PIN_WRITE(PIN_BUZZER, LOW, "Buzzer Surge");
                buzzerPinState = false;
                lastBuzzerToggle = now;
            } else if (!buzzerPinState && (now - lastBuzzerToggle >= 200)) {
                DBG_PIN_WRITE(PIN_BUZZER, HIGH, "Buzzer Surge");
                buzzerPinState = true;
                lastBuzzerToggle = now;
            }
            break;

        case STATE_FREQ_FAULT:
            // 1 Beep panjang (600ms) lalu 2 beep pendek (150ms)
            {
                uint32_t cycleTime = now % 1500;
                bool shouldBeep = (cycleTime < 600) || 
                                  (cycleTime >= 800 && cycleTime < 950) || 
                                  (cycleTime >= 1100 && cycleTime < 1250);
                if (buzzerPinState != shouldBeep) {
                    buzzerPinState = shouldBeep;
                    DBG_PIN_WRITE(PIN_BUZZER, shouldBeep ? HIGH : LOW, "Buzzer Freq Fault");
                }
            }
            break;

        case STATE_FAILSAFE:
            if (!buzzerPinState) {
                buzzerPinState = true;
                DBG_PIN_WRITE(PIN_BUZZER, HIGH, "Buzzer FAILSAFE Continuous");
            }
            break;
    }
}

// ========================================================
// CORE 1: DETERMINISTIC POWER PROTECTION & SEQUENCER FSM
// ========================================================
void TaskControlCore1(void *pvParameters) {
    Serial.println("[CORE 1] Task Proteksi PDU Bawah Aktif di Core 1");

    // Daftarkan ke Task Watchdog (5 detik)
    esp_task_wdt_add(NULL);

    esp_reset_reason_t rstReason = esp_reset_reason();
    bool isFastRestore = (rstReason == ESP_RST_SW || rstReason == ESP_RST_TASK_WDT || 
                          rstReason == ESP_RST_PANIC || rstReason == ESP_RST_INT_WDT || rstReason == ESP_RST_WDT);
    Serial.printf("[CORE 1] MCU Reset Reason: %d (Fast Handover = %s)\n", (int)rstReason, isFastRestore ? "YES" : "NO");

    // Muat Thresholds & State dari NVS Flash
    bool savedRelayStates[4] = {false, false, false, false};
    prefs.begin("pdu_store", false);
    g_threshUvlo    = prefs.getFloat("uvlo", DEFAULT_UVLO_VOLT);
    g_threshSurge   = prefs.getFloat("surge", DEFAULT_SURGE_VOLT);
    g_threshOcp     = prefs.getFloat("ocp", DEFAULT_OCP_CURRENT);
    g_threshFmin    = prefs.getFloat("fmin", DEFAULT_FREQ_MIN);
    g_threshFmax    = prefs.getFloat("fmax", DEFAULT_FREQ_MAX);
    g_seqDelayMs    = prefs.getUInt("seq_delay", SEQUENCER_DELAY_MS);
    g_btnHoldMs     = prefs.getUInt("btn_hold", BTN_HOLD_TOGGLE_MS);
    g_softStartDelayMs = prefs.getUInt("ss_delay", DEFAULT_SOFTSTART_DELAY_MS);

    for (int i = 0; i < 4; i++) {
        relayCycleCount[i] = prefs.getUInt(("cyc_" + String(i)).c_str(), 0);
        savedRelayStates[i] = prefs.getBool(("rl_" + String(i)).c_str(), false);
        // Sync cache targetRelayStates langsung dari pin fisik yang sudah di-latch di setup()
        targetRelayStates[i] = (digitalRead(RELAY_PINS[i]) == LOW);
    }
    prefs.end();

    pinMode(PIN_BUZZER, OUTPUT);
    DBG_PIN_WRITE(PIN_BUZZER, LOW, "Init Buzzer");

    // Inisialisasi Pin Button Fisik (INPUT_PULLUP)
    for (int i = 0; i < 4; i++) {
        pinMode(BTN_PINS[i], INPUT_PULLUP);
    }
    pinMode(BTN_MASTER, INPUT_PULLUP);
    pinMode(BTN_RESET_KWH, INPUT_PULLUP);
    DBG_PRINTLN("[CORE 1] 6x Button Pins (GPIO 10,11,12,13,9,21) di-set ke INPUT_PULLUP");

    // Sequencer FSM Variables
    // Jika Fast Handover (SW/WDT/Panic), relay sudah menyala mulus, jangan masuk stabilizing!
    SequencerState seqState = SEQ_IDLE;
    if (!isFastRestore) {
        for (int i = 0; i < 4; i++) {
            if (savedRelayStates[i]) {
                seqState = SEQ_POWERON_STABILIZING;
                break;
            }
        }
    } else {
        Serial.println("[SEQUENCER] Fast Handover (SW/WDT/Panic): Bypass Power-on Stabilizing (Beban Tetap Aktif)");
    }

    uint32_t stabilizeStartTime = millis();
    int8_t seqCurrentStep = -1;
    uint8_t seqSubStep = 0;
    uint32_t seqLastStepTime = 0;
    static bool ssActive[4] = {false, false, false, false};
    static uint32_t ssStartTime[4] = {0, 0, 0, 0};

    // Multisampling Fault Counters & Grace Period
    uint8_t uvloSamples = 0;
    uint8_t surgeSamples = 0;
    uint8_t ocpSamples = 0;
    uint8_t freqSamples = 0;
    uint32_t lastPzemCheck = 0;
    bool pzemFirstValidRead = false;
    uint32_t bootTimestamp = millis();

    for (;;) {
        lastUptimeS = millis() / 1000;
        uint32_t loopStartMicros = micros();
        esp_task_wdt_reset();
        uint32_t now = millis();

        // ----------------------------------------------------
        // 1. Cek Pesan Masuk dari Core 0 / Blynk / Web
        // ----------------------------------------------------
        crumbCtl = 1; // Breadcrumb: Command Queue & Mutex
        PduCommand cmd;
        if (xQueueReceive(pduCommandQueue, &cmd, 0) == pdPASS) {
                    DBG_PRINTF("[QUEUE RX] Perintah Diterima di Core 1: Type=%d, Ch=%d, State=%d\n", cmd.type, cmd.channel, cmd.state);
            if (cmd.type == CMD_SET_RELAY && cmd.channel == 254) {
                // Force Reset Sequencer ke IDLE
                seqState = SEQ_IDLE;
                for (int i = 0; i < 4; i++) savedRelayStates[i] = false;
                Serial.println("[SEQUENCER] Forced to SEQ_IDLE by User");
            }
            else if (cmd.type == CMD_SET_RELAY && cmd.channel < 4) {
                // Jika sedang dalam masa jeda stabilisasi pasca padam, user boleh manual override untuk membatalkan stabilizing
                if (seqState == SEQ_POWERON_STABILIZING) {
                    seqState = SEQ_IDLE; // User ambil kendali manual
                }

                if (seqState != SEQ_IDLE) {
                    DBG_PRINTF("[INTERLOCK] ⚠️ Perintah Relay %d Ditolak: Sequencer Sedang Berjalan (seqState=%d)!\n", cmd.channel + 1, seqState);
                } else {
                    if (cmd.state) {
                        // Nyalakan via Soft Start (Pre-charge)
                        if (!targetRelayStates[cmd.channel] && !ssActive[cmd.channel]) {
                            ssActive[cmd.channel] = true;
                            ssStartTime[cmd.channel] = now;
                            DBG_PIN_WRITE(RELAY_SS_PINS[cmd.channel], LOW, ("SoftStart ON Ch " + String(cmd.channel + 1)).c_str());
                            DBG_PRINTF("[SOFT START] ⚡ Ch %d Soft Start Active! Menunggu %u ms sebelum Relay Utama ON...\n", 
                                       cmd.channel + 1, g_softStartDelayMs);
                        }
                    } else {
                        // Matikan segera (Keduanya: Relay Utama & Soft Start)
                        ssActive[cmd.channel] = false;
                        xSemaphoreTake(relayMutex, portMAX_DELAY);
                        targetRelayStates[cmd.channel] = false;
                        DBG_PIN_WRITE(RELAY_PINS[cmd.channel], HIGH, ("Relay Main OFF Ch " + String(cmd.channel + 1)).c_str());
                        DBG_PIN_WRITE(RELAY_SS_PINS[cmd.channel], HIGH, ("SoftStart OFF Ch " + String(cmd.channel + 1)).c_str());
                        relayCycleCount[cmd.channel]++;

                        Preferences pStore;
                        pStore.begin("pdu_store", false);
                        pStore.putBool(("rl_" + String(cmd.channel)).c_str(), false);
                        pStore.putUInt(("cyc_" + String(cmd.channel)).c_str(), relayCycleCount[cmd.channel]);
                        pStore.end();
                        xSemaphoreGive(relayMutex);
                        DBG_PRINTF("[CORE 1 CMD] Relay %d Diubah ke -> OFF (Total Siklus: %u)\n", 
                                   cmd.channel + 1, relayCycleCount[cmd.channel]);
                    }
                }
            }
            else if (cmd.type == CMD_TRIGGER_SEQUENCER) {
                // Jika sedang dalam masa stabilisasi, trigger manual langsung membatalkan stabilizing dan menjalankan sequence
                if (seqState == SEQ_POWERON_STABILIZING) {
                    seqState = SEQ_IDLE;
                }

                if (seqState != SEQ_IDLE) {
                    DBG_PRINTF("[INTERLOCK] ⚠️ Pemicu Sequencer Ditolak: Sequencer Sedang Berjalan (seqState=%d)!\n", seqState);
                } else {
                    bool anyOn = targetRelayStates[0] || targetRelayStates[1] || targetRelayStates[2] || targetRelayStates[3];
                    if (anyOn) {
                        seqState = SEQ_SHUTTING_DOWN;
                        seqCurrentStep = 3; // Dari channel 4 mundur ke 1
                    } else {
                        seqState = SEQ_STARTING_UP;
                        seqCurrentStep = 0; // Dari channel 1 maju ke 4
                        seqSubStep = 0;
                        for (int i = 0; i < 4; i++) savedRelayStates[i] = true; // Nyalakan semua pada trigger manual
                    }
                    seqLastStepTime = now - g_seqDelayMs; // Picu channel pertama langsung tanpa jeda awal tambahan!
                    Serial.printf("[SEQUENCER] Start %s sequence (Jeda step: %u ms)...\n", anyOn ? "OFF" : "ON", g_seqDelayMs);
                }
            }
            else if (cmd.type == CMD_RESET_ENERGY) {
                rawResetPzemEnergy(0xF8);
                logEvent("Energy Counter (kWh) Reset by User");
            }
            else if (cmd.type == CMD_FAILSAFE_TRIP) {
                currentTelemetry.sysState = STATE_FAILSAFE;
                // Sesuai requirement: Ketika error di MCU langsung masuk error mode, semua relay aktif
                for (int i = 0; i < 4; i++) {
                    ssActive[i] = false;
                    DBG_PIN_WRITE(RELAY_PINS[i], LOW, ("Failsafe Relay " + String(i + 1)).c_str());
                    DBG_PIN_WRITE(RELAY_SS_PINS[i], HIGH, ("Failsafe SS OFF " + String(i + 1)).c_str());
                    targetRelayStates[i] = true;
                }
                logEvent("FAILSAFE TRIGGERED: All Relays Latched ON");
            }
            else if (cmd.type == CMD_RELOAD_THRESHOLDS) {
                prefs.begin("pdu_store", false);
                g_threshUvlo       = prefs.getFloat("uvlo", DEFAULT_UVLO_VOLT);
                g_threshSurge      = prefs.getFloat("surge", DEFAULT_SURGE_VOLT);
                g_threshOcp        = prefs.getFloat("ocp", DEFAULT_OCP_CURRENT);
                g_seqDelayMs       = prefs.getUInt("seq_delay", SEQUENCER_DELAY_MS);
                g_btnHoldMs        = prefs.getUInt("btn_hold", BTN_HOLD_TOGGLE_MS);
                g_softStartDelayMs = prefs.getUInt("ss_delay", DEFAULT_SOFTSTART_DELAY_MS);
                prefs.end();
                // Reset counter sampel seketika agar tidak tertahan fault lama
                surgeSamples = 0;
                uvloSamples = 0;
                ocpSamples = 0;
                if (currentTelemetry.sysState == STATE_SURGE || currentTelemetry.sysState == STATE_UVLO || currentTelemetry.sysState == STATE_OCP) {
                    currentTelemetry.sysState = STATE_NORMAL;
                }
                Serial.printf("[CORE 1] Thresholds reloaded: UVLO=%.1fV, Surge=%.1fV, OCP=%.1fA | Status di-reset ke NORMAL\n",
                              g_threshUvlo, g_threshSurge, g_threshOcp);
            }
        }

        // ----------------------------------------------------
        // Monitor Selesainya Pre-charge Soft Start Per Channel
        // ----------------------------------------------------
        crumbCtl = 2; // Breadcrumb: Soft Start Pre-charge Monitor
        for (int i = 0; i < 4; i++) {
            if (ssActive[i] && (now - ssStartTime[i] >= g_softStartDelayMs)) {
                ssActive[i] = false;
                xSemaphoreTake(relayMutex, portMAX_DELAY);
                targetRelayStates[i] = true;
                DBG_PIN_WRITE(RELAY_PINS[i], LOW, ("Relay Main ON Ch " + String(i + 1)).c_str());
                DBG_PIN_WRITE(RELAY_SS_PINS[i], HIGH, ("SoftStart OFF Ch " + String(i + 1)).c_str());
                relayCycleCount[i]++;

                Preferences pStore;
                pStore.begin("pdu_store", false);
                pStore.putBool(("rl_" + String(i)).c_str(), true);
                pStore.putUInt(("cyc_" + String(i)).c_str(), relayCycleCount[i]);
                pStore.end();
                xSemaphoreGive(relayMutex);
                DBG_PRINTF("[SOFT START] ✅ Ch %d Main Relay ON! (Pre-charge Selesai, Resistor By-passed)\n", i + 1);
            }
        }

        // ========================================================
        // 1. SCAN TOMBOL FISIK (WAJIB TEKAN TAHAN 2 DETIK)
        // ========================================================
        crumbCtl = 3; // Breadcrumb: Scan Tombol Fisik
        static uint32_t btnPressStartTime[4] = {0, 0, 0, 0};
        static bool btnExecuted[4] = {false, false, false, false};
        static uint32_t masterPressStartTime = 0;
        static bool masterExecuted = false;

        // A. 4 Tombol Channel Relay (GPIO 10, 11, 12, 13) -> Tahan 2 Detik Toggle ON/OFF
        for (int i = 0; i < 4; i++) {
            if (seqState == SEQ_STARTING_UP || seqState == SEQ_SHUTTING_DOWN) {
                btnPressStartTime[i] = 0;
                btnExecuted[i] = false;
                continue;
            }
            int reading = digitalRead(BTN_PINS[i]);
            if (reading == LOW) { // Tombol sedang ditekan (Active LOW ke GND)
                if (btnPressStartTime[i] == 0) {
                    btnPressStartTime[i] = now;
                    btnExecuted[i] = false;
                    DBG_PRINTF("[BTN DOWN] Tombol Fisik Ch %d (GPIO %d) Mulai Ditekan...\n", i + 1, BTN_PINS[i]);
                } else if (!btnExecuted[i] && (now - btnPressStartTime[i] >= g_btnHoldMs)) {
                    // SUDAH DITEKAN TAHAN SESUAI g_btnHoldMs! Eksekusi toggle relay langsung
                    btnExecuted[i] = true;
                    if (seqState == SEQ_POWERON_STABILIZING) seqState = SEQ_IDLE; // User ambil kendali manual
                    bool isCurrentlyActive = targetRelayStates[i] || ssActive[i];
                    if (!isCurrentlyActive) {
                        // Nyalakan dengan Soft Start
                        ssActive[i] = true;
                        ssStartTime[i] = now;
                        DBG_PIN_WRITE(RELAY_SS_PINS[i], LOW, ("Btn SoftStart ON Ch " + String(i + 1)).c_str());
                        DBG_PRINTF("[BTN 2S TRIGGER] ⚡ Tombol Ch %d Sukses Ditahan! Soft Start Active (%u ms)...\n", 
                                   i + 1, g_softStartDelayMs);
                    } else {
                        // Matikan langsung
                        ssActive[i] = false;
                        xSemaphoreTake(relayMutex, portMAX_DELAY);
                        targetRelayStates[i] = false;
                        DBG_PIN_WRITE(RELAY_PINS[i], HIGH, ("Relay Main OFF Ch " + String(i + 1)).c_str());
                        DBG_PIN_WRITE(RELAY_SS_PINS[i], HIGH, ("SoftStart OFF Ch " + String(i + 1)).c_str());
                        relayCycleCount[i]++;

                        Preferences pStore;
                        pStore.begin("pdu_store", false);
                        pStore.putBool(("rl_" + String(i)).c_str(), false);
                        pStore.putUInt(("cyc_" + String(i)).c_str(), relayCycleCount[i]);
                        pStore.end();
                        xSemaphoreGive(relayMutex);
                        DBG_PRINTF("[BTN 2S TRIGGER] ✅ Tombol Ch %d Sukses Ditahan 2 Detik! Relay -> OFF\n", i + 1);
                    }
                }
            } else {
                // Tombol dilepas
                if (btnPressStartTime[i] != 0) {
                    if (!btnExecuted[i]) {
                        DBG_PRINTF("[BTN CANCEL] Tombol Ch %d dilepas sebelum 2 detik (Durasi: %lu ms)\n", 
                                   i + 1, now - btnPressStartTime[i]);
                    }
                    btnPressStartTime[i] = 0;
                    btnExecuted[i] = false;
                }
            }
        }

        // B. Tombol ke-5: Master Sequencer (GPIO 14) -> Tahan 2.5 Detik Toggle Sequencer
        int masterReading = digitalRead(BTN_MASTER);
        if (masterReading == LOW) {
            if (masterPressStartTime == 0) {
                masterPressStartTime = now;
                masterExecuted = false;
                DBG_PRINTF("[BTN MASTER DOWN] Tombol Master (GPIO %d) Mulai Ditekan...\n", BTN_MASTER);
            } else if (!masterExecuted && (now - masterPressStartTime >= g_btnHoldMs)) {
                masterExecuted = true; // Kunci eksekusi sampai tombol benar-benar dilepas!
                if (seqState == SEQ_IDLE) {
                    bool anyOn = false;
                    xSemaphoreTake(relayMutex, portMAX_DELAY);
                    for (int i = 0; i < 4; i++) {
                        if (targetRelayStates[i]) { anyOn = true; break; }
                    }
                    xSemaphoreGive(relayMutex);

                    if (anyOn) {
                        seqState = SEQ_SHUTTING_DOWN;
                        seqCurrentStep = 3;
                        seqLastStepTime = now - g_seqDelayMs; // Langsung picu step pertama tanpa delay tambahan!
                        DBG_PRINTLN("[BTN MASTER 2.5S] ✅ Master Button 2.5s -> Memicu Sequence OFF Bertahap!");
                    } else {
                        seqState = SEQ_STARTING_UP;
                        seqCurrentStep = 0;
                        seqSubStep = 0;
                        seqLastStepTime = now - g_seqDelayMs; // Langsung picu step pertama tanpa delay tambahan!
                        for (int i = 0; i < 4; i++) savedRelayStates[i] = true;
                        DBG_PRINTLN("[BTN MASTER 2.5S] ✅ Master Button 2.5s -> Memicu Sequence ON Bertahap!");
                    }
                } else {
                    DBG_PRINTLN("[BTN MASTER 2S] ⚠️ Sequencer Sedang Berjalan! Input Diabaikan.");
                }
            }
        } else {
            // HANYA reset ketika fisik tombol BENAR-BENAR DILEPAS (HIGH)!
            masterPressStartTime = 0;
            masterExecuted = false;
        }

        // C. Tombol ke-6: Reset Energy kWh (GPIO 21) -> Tahan 5 Detik Reset kWh
        static uint32_t resetKwhPressStartTime = 0;
        static bool resetKwhExecuted = false;
        int rstReading = digitalRead(BTN_RESET_KWH);
        if (rstReading == LOW) {
            if (resetKwhPressStartTime == 0) {
                resetKwhPressStartTime = now;
                resetKwhExecuted = false;
                DBG_PRINTF("[BTN RESET DOWN] Tombol Reset kWh (GPIO %d) Mulai Ditekan...\n", BTN_RESET_KWH);
            } else if (!resetKwhExecuted && (now - resetKwhPressStartTime >= 5000)) {
                resetKwhExecuted = true;
                // Kirim frame reset energy Modbus ke PZEM (0x42)
                uint8_t resetCmd[4] = {0x01, 0x42, 0x80, 0x11};
                PzemSerial.write(resetCmd, 4);
                PzemSerial.flush();
                currentTelemetry.energy = 0.0f;
                DBG_PRINTLN("[BTN RESET 5S] ✅ Energy kWh PZEM Telah Direset ke 0!");
            }
        } else {
            if (resetKwhPressStartTime != 0) {
                resetKwhPressStartTime = 0;
                resetKwhExecuted = false;
            }
        }

        // 2. Update Suhu Chip ESP32-S3, Visualizer RGB LED (GPIO 48) & Riwayat 1-Detik ke PSRAM
        static uint32_t lastHistoryTick = 0;
        if (now - lastHistoryTick >= 1000) {
            lastHistoryTick = now;
            float currentChipTemp = temperatureRead();
            currentTelemetry.chipTemp = currentChipTemp;

            // Visualizer LED RGB Onboard (GPIO 48)
            updateRgbStatus(currentTelemetry.sysState, seqState, currentChipTemp);

            if (psramHistoryBuffer != nullptr) {
                portENTER_CRITICAL(&historyMux);
                psramHistoryBuffer[psramHistoryHead].timestamp = now / 1000;
                psramHistoryBuffer[psramHistoryHead].voltage   = currentTelemetry.voltage;
                psramHistoryBuffer[psramHistoryHead].current   = currentTelemetry.current;
                psramHistoryBuffer[psramHistoryHead].power     = currentTelemetry.power;
                psramHistoryBuffer[psramHistoryHead].chipTemp  = currentChipTemp;
                psramHistoryBuffer[psramHistoryHead].sysState  = (uint8_t)currentTelemetry.sysState;
                uint8_t rBits = 0;
                for (int i = 0; i < 4; i++) { if (targetRelayStates[i]) rBits |= (1 << i); }
                psramHistoryBuffer[psramHistoryHead].relayBits = rBits;

                psramHistoryHead = (psramHistoryHead + 1) % PSRAM_HISTORY_MAX_POINTS;
                if (psramHistoryCount < PSRAM_HISTORY_MAX_POINTS) psramHistoryCount++;
                portEXIT_CRITICAL(&historyMux);
            }
        }

        // 3. Baca Sensor PZEM-004T v4.0 via Modbus RTU Engine
        crumbCtl = 4; // Breadcrumb: Baca PZEM UART Modbus
        if (now - lastPzemCheck >= 500) {
            lastPzemCheck = now;
            float v = NAN, a = NAN, w = NAN, e = NAN, f = NAN, pf = NAN;
            bool readOk = false;
            static uint8_t activePzemAddr = 0xF8;
            static uint8_t pzemConsecutiveFails = 0;
            static uint8_t blackoutSamples = 0;

            // Coba alamat aktif (0xF8 broadcast factory default)
            readOk = rawReadPzem(v, a, w, e, f, pf, activePzemAddr);

            // Jika timeout berkali-kali, hanya probe alamat alternatif setiap 10 siklus (jangan tiap 500ms agar CPU tidak tersedot)
            if (!readOk && (pzemConsecutiveFails % 10 == 3)) {
                uint8_t altAddr = (activePzemAddr == 0xF8) ? 0x01 : 0xF8;
                if (rawReadPzem(v, a, w, e, f, pf, altAddr)) {
                    activePzemAddr = altAddr;
                    readOk = true;
                    pzemConsecutiveFails = 0;
                    DBG_PRINTF("[PZEM ADDR] ✅ Terhubung pada Alamat Modbus: 0x%02X\n", activePzemAddr);
                }
            }

            // Jika masih timeout dan kabel terbalik di software, coba switch baud / invert logika jika diperlukan
            if (!readOk && (PzemSerial.available() > 0)) {
                DBG_PRINTF("[PZEM UART SPAM] Ada %d byte masuk tapi gagal frame!\n", PzemSerial.available());
            }

            if (readOk) {
                pzemConsecutiveFails = 0;
                if (v > 50.0f) pzemFirstValidRead = true;
                currentTelemetry.voltage   = v;
                currentTelemetry.current   = a;
                currentTelemetry.power     = w;
                currentTelemetry.energy    = e;
                currentTelemetry.frequency = f;
                currentTelemetry.pf        = pf;
                recordVoltSample5s(v); // Rekam ke rolling buffer 5 detik untuk analisis sag vs glitch
            } else {
                pzemConsecutiveFails++;
                // KEBAL NOISE: JANGAN langsung nolkan voltase pada 1-3 packet drop / glitch serial sesaat!
                // HANYA nolkan nilai telemetri jika komunikasi serial benar-benar putus >= 4 siklus (~2 detik).
                if (pzemFirstValidRead && pzemConsecutiveFails >= 4) {
                    currentTelemetry.voltage = 0.0f;
                    currentTelemetry.current = 0.0f;
                    currentTelemetry.power   = 0.0f;
                }
                static uint32_t lastDbgFail = 0;
                if (now - lastDbgFail >= 3000) {
                    lastDbgFail = now;
                    DBG_PRINTF("[PZEM DIAG] ⚠️ Tidak ada balasan UART (Fail Count: %d)! Cek kabel PZEM.\n", pzemConsecutiveFails);
                }
            }

            // Monitor Status Semua Pin Fisik (Relay, Button, Buzzer, PZEM)
            static uint32_t lastPinMonitor = 0;
            if (now - lastPinMonitor >= 3000) {
                lastPinMonitor = now;
                DBG_PRINTLN("\n================ [STATUS SEMUA PIN FISIK (ESP32-S3)] ================");
                DBG_PRINTF("  RELAY OUT : Ch1(G%d)=%s | Ch2(G%d)=%s | Ch3(G%d)=%s | Ch4(G%d)=%s\n",
                           RELAY_PINS[0], digitalRead(RELAY_PINS[0]) == LOW ? "ON" : "OFF",
                           RELAY_PINS[1], digitalRead(RELAY_PINS[1]) == LOW ? "ON" : "OFF",
                           RELAY_PINS[2], digitalRead(RELAY_PINS[2]) == LOW ? "ON" : "OFF",
                           RELAY_PINS[3], digitalRead(RELAY_PINS[3]) == LOW ? "ON" : "OFF");
                DBG_PRINTF("  BTN IN    : B1(G%d)=%d | B2(G%d)=%d | B3(G%d)=%d | B4(G%d)=%d | MSTR(G%d)=%d | RST(G%d)=%d\n",
                           BTN_PINS[0], digitalRead(BTN_PINS[0]),
                           BTN_PINS[1], digitalRead(BTN_PINS[1]),
                           BTN_PINS[2], digitalRead(BTN_PINS[2]),
                           BTN_PINS[3], digitalRead(BTN_PINS[3]),
                           BTN_MASTER, digitalRead(BTN_MASTER),
                           BTN_RESET_KWH, digitalRead(BTN_RESET_KWH));
                DBG_PRINTF("  PERIPHERAL: BUZZER(G%d)=%d | PZEM_RX(G%d) | PZEM_TX(G%d) | PZEM_ADDR=0x%02X\n",
                           PIN_BUZZER, digitalRead(PIN_BUZZER), PIN_PZEM_RX, PIN_PZEM_TX, activePzemAddr);
                DBG_PRINTLN("======================================================================\n");
            }

            // Evaluasi Proteksi hanya jika tegangan AC terdeteksi (>50V)
            crumbCtl = 5; // Breadcrumb: Evaluasi Proteksi Kelistrikan & Cutoff
            if (currentTelemetry.voltage > 50.0f) {
                blackoutSamples = 0; // Reset counter blackout jika tegangan normal
                // Evaluasi Counter Fault (dengan floor 50V agar tidak false UVLO saat bench test tanpa AC)
                if (currentTelemetry.voltage > 50.0f && currentTelemetry.voltage < g_threshUvlo) uvloSamples++;
                else uvloSamples = 0;

                if (currentTelemetry.voltage > g_threshSurge) surgeSamples++;
                else surgeSamples = 0;

                if (currentTelemetry.current > g_threshOcp) ocpSamples++;
                else ocpSamples = 0;

                if (currentTelemetry.frequency > 0 && (currentTelemetry.frequency < g_threshFmin || currentTelemetry.frequency > g_threshFmax)) freqSamples++;
                else freqSamples = 0;

                // Update State berdasarkan counter sampel
                if (uvloSamples >= 3) {
                    currentTelemetry.sysState = STATE_UVLO;
                } else if (surgeSamples >= 3) {
                    currentTelemetry.sysState = STATE_SURGE;
                } else if (ocpSamples >= 3) {
                    currentTelemetry.sysState = STATE_OCP;
                } else if (freqSamples >= 4) {
                    currentTelemetry.sysState = STATE_FREQ_FAULT;
                } else if (currentTelemetry.sysState != STATE_FAILSAFE) {
                    currentTelemetry.sysState = STATE_NORMAL; // TEGANGAN NORMAL (220V) -> KEMBALIKAN KE STATE_NORMAL!
                }

                // EKSEKUSI CUT-OFF OTOMATIS: Matikan beban hanya jika benar-benar ada kondisi bahaya aktif
                if (currentTelemetry.sysState == STATE_UVLO || 
                    currentTelemetry.sysState == STATE_SURGE || 
                    currentTelemetry.sysState == STATE_OCP) {
        
                    xSemaphoreTake(relayMutex, portMAX_DELAY);
                    bool anyWasOn = false;
                    for (int i = 0; i < 4; i++) {
                        ssActive[i] = false;
                        if (targetRelayStates[i]) {
                            savedRelayStates[i] = true; // Simpan state untuk auto-recovery
                            DBG_PIN_WRITE(RELAY_PINS[i], HIGH, ("TRIP OFF Relay " + String(i + 1)).c_str());
                            DBG_PIN_WRITE(RELAY_SS_PINS[i], HIGH, ("TRIP OFF SoftStart " + String(i + 1)).c_str());
                            targetRelayStates[i] = false;
                            anyWasOn = true;
                        } else {
                            DBG_PIN_WRITE(RELAY_SS_PINS[i], HIGH, ("TRIP OFF SoftStart " + String(i + 1)).c_str());
                        }
                    }
                    xSemaphoreGive(relayMutex);

                    // REKAM SELALU KE BLACKBOX FLASH (ANTI-HILANG & TETAP TERCATAT MESKI RELAY SUDAH MATI!)
                    const char* rStr = (currentTelemetry.sysState == STATE_SURGE) ? "SURGE VOLTAGE TRIP" :
                                       (currentTelemetry.sysState == STATE_OCP) ? "OVER CURRENT (OCP)" : "UNDER VOLTAGE (UVLO)";
                    saveBlackboxTrip(currentTelemetry.sysState, rStr, currentTelemetry.voltage, currentTelemetry.current, currentTelemetry.power, currentTelemetry.chipTemp);

                    if (anyWasOn) {
                        seqState = SEQ_POWERON_STABILIZING; // Tunggu listrik normal & stabil sebelum auto-restore
                        stabilizeStartTime = now;
                        DBG_PRINTF("[SAFETY TRIP] Bahaya Kelistrikan (State %d)! Seluruh Relay Diputus, Siaga Auto-Recovery!\n", currentTelemetry.sysState);
                    }
                }
            } else {
                // Tegangan AC Hilang / Blackout (< 50V)
                // Wajib debounce multi-sample: minimal 4x siklus berturut-turut (~2 detik)
                // Mencegah false blackout akibat 1 packet drop atau noise serial UART
                blackoutSamples++;
                if (blackoutSamples >= 4) {
                    // HANYA trigger STATE_UVLO jika AC Mains sebelumnya sudah terkonfirmasi aktif (pzemFirstValidRead).
                    // Mencegah false alarm merah saat uji coba di meja (bench test via USB tanpa 220V AC).
                    if (pzemFirstValidRead) {
                        if (currentTelemetry.sysState != STATE_FAILSAFE) {
                            currentTelemetry.sysState = STATE_UVLO;
                        }
                    } else {
                        if (currentTelemetry.sysState != STATE_FAILSAFE) {
                            currentTelemetry.sysState = STATE_NORMAL;
                        }
                    }
                    // Cut-off seluruh relay HANYA jika AC Mains sebelumnya sudah terkonfirmasi aktif
                    // dan sudah melewati boot grace period (5000ms). Mencegah trip palsu saat boot atau bench test.
                    if (pzemFirstValidRead && (now - bootTimestamp >= 5000)) {
                        xSemaphoreTake(relayMutex, portMAX_DELAY);
                        bool anyWasOn = false;
                        for (int i = 0; i < 4; i++) {
                            ssActive[i] = false;
                            if (targetRelayStates[i]) {
                                savedRelayStates[i] = true; // Simpan state relay aktif sebelum blackout
                                DBG_PIN_WRITE(RELAY_PINS[i], HIGH, ("BLACKOUT OFF Relay " + String(i + 1)).c_str());
                                DBG_PIN_WRITE(RELAY_SS_PINS[i], HIGH, ("BLACKOUT OFF SoftStart " + String(i + 1)).c_str());
                                targetRelayStates[i] = false;
                                anyWasOn = true;
                            } else {
                                DBG_PIN_WRITE(RELAY_SS_PINS[i], HIGH, ("BLACKOUT OFF SoftStart " + String(i + 1)).c_str());
                            }
                        }
                        xSemaphoreGive(relayMutex);

                        saveBlackboxTrip(STATE_UVLO, "GRID BLACKOUT (<50V)", currentTelemetry.voltage, currentTelemetry.current, currentTelemetry.power, currentTelemetry.chipTemp);

                        if (anyWasOn) {
                            seqState = SEQ_POWERON_STABILIZING; // Siaga untuk auto-recovery bertahap saat listrik pulih!
                            stabilizeStartTime = now;
                            DBG_PRINTLN("[BLACKOUT] ⚠️ AC Mains Drop < 50V (4x Terkonfirmasi): Seluruh Relay Diputus & Siap Auto-Recovery!");
                        }
                    }
                }
            }

            // Sync snapshot ke telemetry struct
            xSemaphoreTake(relayMutex, portMAX_DELAY);
            for (int i = 0; i < 4; i++) {
                currentTelemetry.relayStates[i] = targetRelayStates[i];
                currentTelemetry.relayCycleCount[i] = relayCycleCount[i];
            }
            xSemaphoreGive(relayMutex);

            xQueueOverwrite(telemetryQueue, &currentTelemetry);
        }

        // ----------------------------------------------------
        // 3. Sequencer Non-Blocking FSM
        // ----------------------------------------------------
        crumbCtl = 6; // Breadcrumb: Sequencer Non-Blocking FSM
        if (seqState == SEQ_POWERON_STABILIZING) {
            bool isVoltageNormal = (currentTelemetry.voltage >= g_threshUvlo && currentTelemetry.voltage <= g_threshSurge);
            
            if (!isVoltageNormal) {
                stabilizeStartTime = now; // Reset timer kestabilan jika tegangan belum normal (<175V atau >255V)
            } else if (now - stabilizeStartTime >= POWERON_STABILIZE_MS) {
                Serial.println("[POWER-ON] Grid Voltage Stabilized! Resuming Previous Relay States...");
                
                // Cek apakah ada relay yang sebelumnya aktif (baik di RAM savedRelayStates maupun di NVS Flash)
                bool hasRelayToRestore = false;
                Preferences pRead;
                pRead.begin("pdu_store", true);
                for (int i = 0; i < 4; i++) {
                    if (savedRelayStates[i] || pRead.getBool(("rl_" + String(i)).c_str(), false)) {
                        savedRelayStates[i] = true;
                        hasRelayToRestore = true;
                    }
                }
                pRead.end();

                if (hasRelayToRestore) {
                    seqState = SEQ_STARTING_UP;
                    seqCurrentStep = 0;
                    seqSubStep = 0;
                    seqLastStepTime = now - g_seqDelayMs; // Langsung picu step pertama
                    Serial.println("[POWER-ON] Relay aktif terdeteksi! Memulai SEQ_STARTING_UP bertahap...");
                } else {
                    seqState = SEQ_IDLE; // Tidak ada relay yang perlu dinyalakan
                    Serial.println("[POWER-ON] Tidak ada riwayat relay aktif. Sequencer kembali ke IDLE.");
                }
            }
        }
        else if (seqState == SEQ_STARTING_UP) {
            if (seqCurrentStep >= 0 && seqCurrentStep < 4) {
                if (!savedRelayStates[seqCurrentStep]) {
                    DBG_PRINTF("[SEQUENCER ON] Ch %d DILAWATI (Previously OFF)\n", seqCurrentStep + 1);
                    seqCurrentStep++;
                    seqSubStep = 0;
                    seqLastStepTime = now;
                } else if (seqSubStep == 0) {
                    // Tahap 1: Nyalakan Soft Start Relay (Pre-charge)
                    if (now - seqLastStepTime >= g_seqDelayMs) {
                        DBG_PIN_WRITE(RELAY_SS_PINS[seqCurrentStep], LOW, ("Seq SS ON Ch " + String(seqCurrentStep + 1)).c_str());
                        DBG_PRINTF("[SEQUENCER ON] ⚡ Ch %d Soft Start ON (Pre-charge %u ms)...\n", seqCurrentStep + 1, g_softStartDelayMs);
                        seqLastStepTime = now;
                        seqSubStep = 1;
                    }
                } else if (seqSubStep == 1) {
                    // Tahap 2: Nyalakan Relay Utama dan matikan Soft Start Relay
                    if (now - seqLastStepTime >= g_softStartDelayMs) {
                        xSemaphoreTake(relayMutex, portMAX_DELAY);
                        targetRelayStates[seqCurrentStep] = true;
                        DBG_PIN_WRITE(RELAY_PINS[seqCurrentStep], LOW, ("Seq Main ON Ch " + String(seqCurrentStep + 1)).c_str());
                        DBG_PIN_WRITE(RELAY_SS_PINS[seqCurrentStep], HIGH, ("Seq SS OFF Ch " + String(seqCurrentStep + 1)).c_str());
                        relayCycleCount[seqCurrentStep]++;
                        Preferences pStore;
                        pStore.begin("pdu_store", false);
                        pStore.putBool(("rl_" + String(seqCurrentStep)).c_str(), true);
                        pStore.putUInt(("cyc_" + String(seqCurrentStep)).c_str(), relayCycleCount[seqCurrentStep]);
                        pStore.end();
                        xSemaphoreGive(relayMutex);
                        DBG_PRINTF("[SEQUENCER ON] ✅ Ch %d MAIN NYALA (SoftStart Selesai)\n", seqCurrentStep + 1);

                        seqLastStepTime = now;
                        seqCurrentStep++;
                        seqSubStep = 0;
                    }
                }
            } else {
                seqState = SEQ_IDLE;
                seqSubStep = 0;
                // Bersihkan flag savedRelayStates setelah sequence restore selesai
                for (int i = 0; i < 4; i++) savedRelayStates[i] = false;
                Serial.println("[SEQUENCER] Startup Sequence Finished.");
            }
        }
        else if (seqState == SEQ_SHUTTING_DOWN) {
            if (now - seqLastStepTime >= g_seqDelayMs) {
                seqLastStepTime = now;
                if (seqCurrentStep >= 0 && seqCurrentStep < 4) {
                    xSemaphoreTake(relayMutex, portMAX_DELAY);
                    targetRelayStates[seqCurrentStep] = false;
                    DBG_PIN_WRITE(RELAY_PINS[seqCurrentStep], HIGH, ("Seq OFF Main Ch " + String(seqCurrentStep + 1)).c_str());
                    DBG_PIN_WRITE(RELAY_SS_PINS[seqCurrentStep], HIGH, ("Seq OFF SS Ch " + String(seqCurrentStep + 1)).c_str());
                    relayCycleCount[seqCurrentStep]++;
                    Preferences pStore;
                    pStore.begin("pdu_store", false);
                    pStore.putBool(("rl_" + String(seqCurrentStep)).c_str(), false);
                    pStore.putUInt(("cyc_" + String(seqCurrentStep)).c_str(), relayCycleCount[seqCurrentStep]);
                    pStore.end();
                    xSemaphoreGive(relayMutex);
                    DBG_PRINTF("[SEQUENCER OFF] Ch %d MATI\n", seqCurrentStep + 1);
                    seqCurrentStep--;
                } else {
                    seqState = SEQ_IDLE;
                    Serial.println("[SEQUENCER] Shutdown Sequence Finished.");
                }
            }
        }

        // 4. Update Audio Alert & LED Indikator Hardware
        updateBuzzer(currentTelemetry.sysState);

        // Selalu sinkronkan state FSM sequencer terbaru ke struct telemetri di setiap tick
        currentTelemetry.seqState = seqState;

        // Estimasi beban kerja CPU di Core 1 (PZEM, FSM, Actuator)
        static uint32_t s_workAccum = 0;
        static uint32_t s_winStart = 0;
        uint32_t loopWorkMicros = micros() - loopStartMicros;
        if (loopWorkMicros > s_pzemSleepMicros) loopWorkMicros -= s_pzemSleepMicros;
        else loopWorkMicros = 0;
        s_pzemSleepMicros = 0;
        s_workAccum += loopWorkMicros;

        if (s_winStart == 0) s_winStart = now;
        if (now - s_winStart >= 1000) {
            uint32_t elapsed = (now - s_winStart) * 1000;
            if (elapsed > 0) {
                float rawLoad = (float)s_workAccum * 100.0f / (float)elapsed;
                rawLoad += 1.5f; // Baseline overhead FreeRTOS scheduler
                g_estimatedCpuLoad = (g_estimatedCpuLoad * 0.7f) + (rawLoad * 0.3f);
                if (g_estimatedCpuLoad < 1.0f) g_estimatedCpuLoad = 1.0f;
                if (g_estimatedCpuLoad > 100.0f) g_estimatedCpuLoad = 100.0f;

                currentTelemetry.cpuLoad1 = g_estimatedCpuLoad;
                currentTelemetry.cpuLoad0 = g_core0Load;
                currentTelemetry.cpuLoad = (g_core0Load + g_estimatedCpuLoad) / 2.0f;
                currentTelemetry.cpuFreq = ESP.getCpuFreqMHz();
            }
            s_workAccum = 0;
            s_winStart = now;
        }

        crumbCtl = 0; // Breadcrumb: Core 1 Idle / Tick Selesai
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ========================================================
// ESP-NOW CALLBACKS & LOGIC
// ========================================================
void sendEspNow(const uint8_t *destMac, EspNowPayload &packet) {
    esp_now_send(destMac, (uint8_t *)&packet, sizeof(EspNowPayload));
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len != sizeof(EspNowPayload)) return;

    EspNowPayload packet;
    memcpy(&packet, incomingData, sizeof(EspNowPayload));

    uint32_t recvNow = millis();
    g_cpuActivityPulse = true; // Picu flicker HDD pada LED Hijau saat ada paket radio masuk

    // Cari index node pengirim berdasarkan MAC
    int existingIndex = -1;
    for (int i = 0; i < registeredClientCount; i++) {
        if (memcmp(clientList[i].mac, mac, 6) == 0) {
            existingIndex = i;
            clientList[i].lastSeen = recvNow;
            clientList[i].isOnline = true;
            break;
        }
    }

    Serial.printf("[ESP-NOW RX] Paket Masuk! Type=0x%02X Node=%d dari %02X:%02X:%02X:%02X:%02X:%02X (MatchIdx=%d)\n",
                  packet.msgType, packet.nodeType, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], existingIndex);

    // DISCOVERY: Pendaftaran Node Baru
    if (packet.msgType == MSG_DISCOVERY) {
        uint8_t activeCh = WiFi.channel();
        if (activeCh == 0) activeCh = 1;

        int targetIdx = existingIndex;
        if (existingIndex != -1) {
            // Node sudah ada sebelumnya: perbarui data runtime
            clientList[existingIndex].type = (NodeType)packet.nodeType;
            clientList[existingIndex].lastSeen = recvNow;
            clientList[existingIndex].isOnline = true;
        } else if (registeredClientCount < ESPNOW_MAX_CLIENTS) {
            // Node baru: Daftarkan ke slot baru dan SIMPAN PERMANEN KE FLASH
            targetIdx = registeredClientCount;
            memcpy(clientList[targetIdx].mac, mac, 6);
            clientList[targetIdx].type = (NodeType)packet.nodeType;
            clientList[targetIdx].lastSeen = recvNow;
            clientList[targetIdx].isOnline = true;
            registeredClientCount++;

            Preferences pNode;
            pNode.begin("gateway_nodes", false);
            pNode.putUChar("node_count", registeredClientCount);
            String key = "mac_" + String(targetIdx);
            pNode.putBytes(key.c_str(), mac, 6);
            pNode.putUChar(("type_" + String(targetIdx)).c_str(), packet.nodeType);
            pNode.end();
            Serial.printf("[ESP-NOW] ✅ New Node Registered & Saved to Flash! Slot=%d Type=%d\n", targetIdx, packet.nodeType);
        }

        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, mac, 6);
        peerInfo.channel = activeCh;
        peerInfo.ifidx = WIFI_IF_AP;
        peerInfo.encrypt = false;
        if (esp_now_is_peer_exist(mac)) {
            esp_now_del_peer(mac);
        }
        esp_now_add_peer(&peerInfo);

        EspNowPayload ackPacket = {};
        ackPacket.msgType = MSG_PAIR_ACK;
        ackPacket.nodeType = (NodeType)packet.nodeType;
        ackPacket.payloadValue = activeCh; // Kirimkan nomor channel aktif Gateway ke Klien!
        WiFi.macAddress(ackPacket.macSender);
        sendEspNow(mac, ackPacket);

        Serial.printf("[ESP-NOW] Pair ACK Sent to %02X:%02X:%02X:%02X:%02X:%02X on Ch %d\n",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], activeCh);
    }
    // TELEMETRI: Dari Sub-PDU Lt 2 / Sensor
    else if (packet.msgType == MSG_TELEMETRY) {
        if (packet.nodeType == NODE_SUB_PDU_LT2) {
            g_subPduLt2RelayState = packet.payloadValue;
            g_blynkSyncSubPdu = true;
        }
    }
    // PROVISION REQ: Klien minta link download binary firmware
    else if (packet.msgType == MSG_PROVISION_REQ) {
        EspNowPayload ack = {};
        ack.msgType = MSG_PROVISION_ACK;
        WiFi.macAddress(ack.macSender);
        ack.payloadValue = 80; // Port WebServer Master
        sendEspNow(mac, ack);
        Serial.println("[ESP-NOW] Provision Request Answered to Client.");
    }
}

// ========================================================
// CORE 0: NETWORK, WEB SERVER 4 HALAMAN, & BLYNK CLOUD
// ========================================================
// Core 0 Task Network
void TaskNetworkCore0(void *pvParameters) {
    Serial.println("[CORE 0] Task Network, Web & Blynk Aktif di Core 0");
    esp_task_wdt_add(NULL);

    // KUNCI RADIO: Konfigurasi SoftAP di Channel 1
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    bool apOk = WiFi.softAP("SmartPDU-Gateway", "admin123", 1, 0, 4);
    Serial.printf("[WIFI AP] SoftAP Status: %s | IP: %s\n", apOk ? "BERHASIL" : "GAGAL", WiFi.softAPIP().toString().c_str());

    // Inisialisasi ESP-NOW SETELAH WiFi Mode AP_STA terbentuk sempurna
        if (esp_now_init() == ESP_OK) {
            esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));
            Serial.println("[ESP-NOW] Subsystem Aktif di Core 0!");

            // Daftarkan ulang peer broadcast dan peer klien dari NVS Flash
            Preferences pNode;
            pNode.begin("gateway_nodes", true);
            registeredClientCount = pNode.getUChar("node_count", 0);
            for (int i = 0; i < registeredClientCount; i++) {
                        String key = "mac_" + String(i);
                        pNode.getBytes(key.c_str(), clientList[i].mac, 6);
                        clientList[i].type = (NodeType)pNode.getUChar(("type_" + String(i)).c_str(), NODE_SUB_PDU_LT2);
                        clientList[i].isOnline = true; // Langsung anggap online saat boot
                        clientList[i].lastSeen = millis();

                        esp_now_peer_info_t peer = {};
                        memcpy(peer.peer_addr, clientList[i].mac, 6);
                        peer.channel = 0; // 0 = Ikuti channel radio home/AP aktif
                        peer.ifidx = WIFI_IF_AP;
                        peer.encrypt = false;
                        if (esp_now_is_peer_exist(clientList[i].mac)) esp_now_del_peer(clientList[i].mac);
                        esp_now_add_peer(&peer);
                    }
            pNode.end();
            Serial.printf("[ESP-NOW] Loaded %d Registered Client Nodes from Flash!\n", registeredClientCount);
        } else {
            Serial.println("[ESP-NOW] GAGAL Inisialisasi!");
        }

    // Cek SSID tersimpan dari NVS
    Preferences pWifi;
    pWifi.begin("pdu_wifi", true);
    g_staSsid = pWifi.getString("ssid", "");
    g_staPass = pWifi.getString("pass", "");
    pWifi.end();

    // HANYA konek STA jika user memang sudah pernah setting SSID
    if (g_staSsid.length() > 0) {
        Serial.printf("[WIFI] Menyambungkan ke Router: \"%s\"\n", g_staSsid.c_str());
        WiFi.begin(g_staSsid.c_str(), g_staPass.c_str());
    } else {
        Serial.println("[WIFI] Belum ada WiFi router tersimpan. Beroperasi murni AP Mode.");
    }

    // Muat Konfigurasi Blynk dari NVS Flash
    Preferences pBlynk;
    pBlynk.begin("pdu_store", true);
    g_blynkAuthToken  = pBlynk.getString("blynk_tok", BLYNK_AUTH_TOKEN);
    g_blynkTemplateId = pBlynk.getString("blynk_tmpl", BLYNK_TEMPLATE_ID);
    g_blynkEnabled    = pBlynk.getBool("blynk_en", true);
    pBlynk.end();

    if (g_blynkEnabled && g_blynkAuthToken.length() > 0) {
        Blynk.config(g_blynkAuthToken.c_str());
        Serial.printf("[BLYNK] Inisialisasi: TemplateID=%s | Token=%s (Status: Aktif)\n", 
                      g_blynkTemplateId.c_str(), g_blynkAuthToken.c_str());
    } else {
        Serial.println("[BLYNK] Dinonaktifkan di Flash NVS.");
    }

    // Inisialisasi mDNS "electric-pdu.local"
    if (MDNS.begin(MDNS_HOSTNAME)) {
        MDNS.addService("http", "tcp", 80);
        Serial.printf("[MDNS] Domain aktif di: http://%s.local\n", MDNS_HOSTNAME);
    }

        uint32_t lastBlynkConnectAttempt = 0;
        uint32_t lastCloudPush = 0;
        uint32_t lastStaCheck = 0;
        TelemetryData localData = {0};

        for (;;) {
            uint32_t loopStartMicros0 = micros();
            esp_task_wdt_reset();
            uint32_t now = millis();

            // 0. Soft-Reconnect Live jika Wi-Fi baru disimpan via Web UI (Zero Downtime)
            crumbNet = 1; // Breadcrumb: WiFi & Blynk config housekeeping
            if (g_wifiNeedReconnect) {
                g_wifiNeedReconnect = false;
                Serial.printf("[WIFI LIVE] Menerapkan konfigurasi Wi-Fi baru: \"%s\" tanpa reboot MCU...\n", g_staSsid.c_str());
                WiFi.disconnect(false);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                if (g_staSsid.length() > 0) {
                    WiFi.begin(g_staSsid.c_str(), g_staPass.c_str());
                }
                lastStaCheck = now;
            }

            // 0b. Live Update Konfigurasi Blynk jika diubah via Web UI (Zero Downtime)
            if (g_blynkNeedReconnect) {
                g_blynkNeedReconnect = false;
                if (Blynk.connected()) {
                    Blynk.disconnect();
                }
                if (g_blynkEnabled && g_blynkAuthToken.length() > 0) {
                    Blynk.config(g_blynkAuthToken.c_str());
                    lastBlynkConnectAttempt = 0; // Trigger sambung segera
                    Serial.printf("[BLYNK LIVE] Konfigurasi diterapkan: Token=\"%s\", Enabled=%s\n", 
                                  g_blynkAuthToken.c_str(), g_blynkEnabled ? "YES" : "NO");
                } else {
                    Serial.println("[BLYNK LIVE] Blynk dinonaktifkan.");
                }
            }

            // 1. Non-Blocking Wi-Fi & Blynk Connection Manager
            crumbNet = 2; // Breadcrumb: Checking Wi-Fi STA & mDNS
            if (WiFi.status() == WL_CONNECTED) {
                uint8_t staCh = WiFi.channel();
                // Pastikan mDNS juga di-init / di-update saat STA connect ke router
                static bool mdnsStaStarted = false;
                if (!mdnsStaStarted) {
                    mdnsStaStarted = true;
                    MDNS.end();
                    if (MDNS.begin(MDNS_HOSTNAME)) {
                        MDNS.addService("http", "tcp", 80);
                        Serial.printf("[MDNS] Re-registered via Router STA: http://%s.local (%s)\n", 
                                      MDNS_HOSTNAME, WiFi.localIP().toString().c_str());
                    }
                }
                static uint8_t lastKnownCh = 0;
                if (staCh != 0 && staCh != lastKnownCh) {
                    lastKnownCh = staCh;
                    Serial.printf("[WIFI] Router aktif di Channel: %d. Mengunci frekuensi ESP-NOW ke Ch %d\n", staCh, staCh);
                }

                if (g_blynkEnabled && g_blynkAuthToken.length() > 0) {
                    if (!Blynk.connected()) {
                        if (now - lastBlynkConnectAttempt >= 5000) {
                            lastBlynkConnectAttempt = now;
                            Serial.println("[BLYNK] Mencoba connect ke server Blynk...");
                            crumbNet = 3; // Breadcrumb: Di dalam Blynk.connect() (Tersangka TWDT #1)
                            esp_task_wdt_reset(); // Feed WDT sebelum blokir socket/DNS!
                            uint32_t connStart = micros();
                            Blynk.connect(1500); // 1.5-second timeout non-blocking
                            esp_task_wdt_reset(); // Feed WDT seketika setelah return!
                            loopStartMicros0 += (micros() - connStart); // Exclude socket wait
                            if (Blynk.connected()) {
                                Serial.println("[BLYNK] BERHASIL ONLINE KE BLYNK CLOUD! 🟢");
                            }
                        }
                    } else {
                        crumbNet = 4; // Breadcrumb: Di dalam Blynk.run()
                        Blynk.run();
                    }
                }
            } else {
                crumbNet = 5; // Breadcrumb: Menghubungkan ulang Wi-Fi STA
                // Jika STA terputus dan ada SSID tersimpan, coba reconnect setiap 15 detik
                if (g_staSsid.length() > 0 && (now - lastStaCheck >= 15000)) {
                    lastStaCheck = now;
                    Serial.printf("[WIFI] Status STA: %d. Mencoba reconnect ke \"%s\"...\n", WiFi.status(), g_staSsid.c_str());
                    WiFi.begin(g_staSsid.c_str(), g_staPass.c_str());
                }
            }

            // 2. Telemetry Sync ke Blynk Cloud
            crumbNet = 6; // Breadcrumb: Telemetry push ke Blynk Cloud
        static bool lastRelayStateBlynk[4] = {false, false, false, false};
        static SystemState lastSysStateBlynk = STATE_NORMAL;
        static uint8_t lastClientCountBlynk = 255;

        if (xQueueReceive(telemetryQueue, &localData, 0) == pdPASS) {
            if (now - lastCloudPush >= 1000 && g_blynkEnabled && Blynk.connected()) {
                lastCloudPush = now;
                uint32_t blynkPushStart = micros();
                Blynk.virtualWrite(VPIN_VOLT, localData.voltage);
                Blynk.virtualWrite(VPIN_AMP, localData.current);
                Blynk.virtualWrite(VPIN_WATT, localData.power);
                Blynk.virtualWrite(VPIN_FREQ, localData.frequency);
                Blynk.virtualWrite(VPIN_PF, localData.pf);
                Blynk.virtualWrite(VPIN_ENERGY, localData.energy);

                for (int i = 0; i < 4; i++) {
                    if (localData.relayStates[i] != lastRelayStateBlynk[i]) {
                        lastRelayStateBlynk[i] = localData.relayStates[i];
                        Blynk.virtualWrite(VPIN_LED[i], localData.relayStates[i] ? 255 : 0);
                        if (blynkPressStart[i] == 0) {
                            Blynk.virtualWrite(VPIN_RELAY[i], localData.relayStates[i] ? 1 : 0);
                        }
                    }
                }
                if (localData.sysState != lastSysStateBlynk) {
                    lastSysStateBlynk = localData.sysState;
                    Blynk.virtualWrite(VPIN_ALARM_LED, (localData.sysState != STATE_NORMAL) ? 255 : 0);
                }
                if (registeredClientCount != lastClientCountBlynk) {
                    lastClientCountBlynk = registeredClientCount;
                    Blynk.virtualWrite(VPIN_CLIENT_COUNT, registeredClientCount);
                }
                uint32_t blynkDuration = micros() - blynkPushStart;
                if (blynkDuration > 1000) {
                    loopStartMicros0 += (blynkDuration - 1000); // Sisakan 1ms komputasi wajar
                }
            }
        }

        // 2b. Sync Telemetri Klien Sub-PDU Lt2 ke Blynk (Thread-Safe di Core 0)
        if (g_blynkSyncSubPdu && g_blynkEnabled && Blynk.connected()) {
            g_blynkSyncSubPdu = false;
            Blynk.virtualWrite(VPIN_CLIENT_UP_RL1, g_subPduLt2RelayState);
        }

        // 2c. Safety Auto-Clear jika tombol Blynk tertekan > 10s (mencegah ghost hold)
        for (int i = 0; i < 4; i++) {
            if (blynkPressStart[i] != 0 && (now - blynkPressStart[i] > 10000)) {
                blynkPressStart[i] = 0;
            }
        }

        // 3. Pairing Window Timeout (60 Detik)
        crumbNet = 7; // Breadcrumb: ESP-NOW Pairing & Node Health-Check
        if (isPairingWindowActive && (now - pairingStartTime >= 60000)) {
            isPairingWindowActive = false;
            Serial.println("[PAIRING] Pairing window closed.");
            if (Blynk.connected()) Blynk.virtualWrite(VPIN_PAIRING_MODE, 0);
        }

        // 4. Klien Health-Check (Timeout 45 detik)
                static uint32_t lastHealthCheck = 0;
                if (now - lastHealthCheck >= 5000) {
                    lastHealthCheck = now;
                    for (int i = 0; i < registeredClientCount; i++) {
                        if (clientList[i].isOnline && (now - clientList[i].lastSeen >= 45000)) {
                            clientList[i].isOnline = false;
                            Serial.printf("[ESP-NOW] Client node %d (%02X:%02X:%02X:%02X:%02X:%02X) offline (timeout)\n", 
                                          i, clientList[i].mac[0], clientList[i].mac[1], clientList[i].mac[2],
                                          clientList[i].mac[3], clientList[i].mac[4], clientList[i].mac[5]);
                        }
                    }
                }

        // Estimasi beban kerja CPU di Core 0 (Network, AsyncTCP, & Blynk)
        static uint32_t s_workAccum0 = 0;
        static uint32_t s_winStart0 = 0;
        uint32_t loopWorkMicros0 = micros() - loopStartMicros0;
        s_workAccum0 += loopWorkMicros0;

        if (s_winStart0 == 0) s_winStart0 = now;
        if (now - s_winStart0 >= 1000) {
            uint32_t elapsed0 = (now - s_winStart0) * 1000;
            if (elapsed0 > 0) {
                float rawLoad0 = (float)s_workAccum0 * 100.0f / (float)elapsed0;
                rawLoad0 += 1.5f; // Baseline background LwIP TCP/IP stack overhead
                g_core0Load = (g_core0Load * 0.7f) + (rawLoad0 * 0.3f);
                if (g_core0Load < 1.0f) g_core0Load = 1.0f;
                if (g_core0Load > 100.0f) g_core0Load = 100.0f;
            }
            s_workAccum0 = 0;
            s_winStart0 = now;
        }

        crumbNet = 0; // Breadcrumb: Core 0 Idle / Tick Selesai
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// ========================================================
// BLYNK VIRTUAL PIN HANDLERS (WAJIB PUSH TAHAN 2 DETIK)
// ========================================================
// Pelacak waktu tekan tombol Blynk (Widget Mode: Push Button)

static void handleBlynkRelayHold2s(uint8_t ch, int val, uint8_t vpin) {
    uint32_t now = millis();
    if (val == 1) { // Jari mulai menekan tombol di HP
        blynkPressStart[ch] = now;
        DBG_PRINTF("[BLYNK HOLD] Button V%d Ch %d Ditekan... Tahan 2 Detik!\n", vpin, ch + 1);
    } else { // Jari dilepas
        if (blynkPressStart[ch] != 0) {
            uint32_t duration = now - blynkPressStart[ch];
            if (duration >= g_btnHoldMs) {
                // Berhasil ditahan >= g_btnHoldMs! Toggle relay
                xSemaphoreTake(relayMutex, portMAX_DELAY);
                bool newState = !targetRelayStates[ch];
                xSemaphoreGive(relayMutex);

                PduCommand cmd = {CMD_SET_RELAY, ch, newState};
                xQueueSend(pduCommandQueue, &cmd, 0);

                DBG_PRINTF("[BLYNK HOLD TRIGGER] ✅ V%d Sukses Ditahan %lu ms! Relay Ch %d -> %s\n", 
                           vpin, duration, ch + 1, newState ? "ON" : "OFF");
            } else {
                DBG_PRINTF("[BLYNK CANCEL] V%d Dilepas Terlalu Cepat (%lu ms < %lu ms). Dibatalkan!\n", 
                           vpin, duration, g_btnHoldMs);
            }
            blynkPressStart[ch] = 0;
            // Kembalikan status visual widget ke state aktual
            xSemaphoreTake(relayMutex, portMAX_DELAY);
            bool actual = targetRelayStates[ch];
            xSemaphoreGive(relayMutex);
            Blynk.virtualWrite(vpin, actual ? 1 : 0);
        }
    }
}

BLYNK_WRITE(V5) { handleBlynkRelayHold2s(0, param.asInt(), V5); }
BLYNK_WRITE(V6) { handleBlynkRelayHold2s(1, param.asInt(), V6); }
BLYNK_WRITE(V7) { handleBlynkRelayHold2s(2, param.asInt(), V7); }
BLYNK_WRITE(V8) { handleBlynkRelayHold2s(3, param.asInt(), V8); }

BLYNK_WRITE(VPIN_MASTER) {
    int val = param.asInt();
    DBG_PRINTF("[BLYNK MASTER] Event V10 Masuk: val=%d\n", val);
    if (val == 1) {
        if (currentTelemetry.seqState == SEQ_IDLE) {
            PduCommand cmd = {CMD_TRIGGER_SEQUENCER, 255, false};
            xQueueSend(pduCommandQueue, &cmd, 0);
            DBG_PRINTLN("[BLYNK MASTER] ✅ Sequencer Triggered via Blynk V10 Tap!");
        } else {
            DBG_PRINTLN("[BLYNK MASTER] ⚠️ Sequencer Re-trigger Ditolak: Sedang Berjalan!");
        }
        Blynk.virtualWrite(VPIN_MASTER, 0);
    }
}

BLYNK_WRITE(VPIN_RESET_KWH) {
    if (param.asInt() == 1) {
        PduCommand cmd = {CMD_RESET_ENERGY, 0, false};
        xQueueSend(pduCommandQueue, &cmd, 0);
    }
}

BLYNK_WRITE(VPIN_PAIRING_MODE) {
    isPairingWindowActive = (param.asInt() == 1);
    if (isPairingWindowActive) {
        pairingStartTime = millis();
        Serial.println("[PAIRING] Window opened via Blynk (60s)");
    }
}

// Relay 1 Klien Lantai Atas
BLYNK_WRITE(VPIN_CLIENT_UP_RL1) {
    int val = param.asInt();
    for (int i = 0; i < registeredClientCount; i++) {
        if (clientList[i].type == NODE_SUB_PDU_LT2 && clientList[i].isOnline) {
            EspNowPayload cmd = {};
            cmd.msgType = MSG_COMMAND;
            cmd.targetChannel = 1; // Relay 1
            cmd.payloadValue = val;
            WiFi.macAddress(cmd.macSender);
            sendEspNow(clientList[i].mac, cmd);
            Serial.printf("[GATEWAY -> KLIEN LT2] Perintah Blynk Relay 1 = %d terkirim!\n", val);
            break;
        }
    }
}

// Relay 2 Klien Lantai Atas
BLYNK_WRITE(VPIN_CLIENT_UP_RL2) {
    int val = param.asInt();
    for (int i = 0; i < registeredClientCount; i++) {
        if (clientList[i].type == NODE_SUB_PDU_LT2 && clientList[i].isOnline) {
            EspNowPayload cmd = {};
            cmd.msgType = MSG_COMMAND;
            cmd.targetChannel = 2; // Relay 2
            cmd.payloadValue = val;
            WiFi.macAddress(cmd.macSender);
            sendEspNow(clientList[i].mac, cmd);
            Serial.printf("[GATEWAY -> KLIEN LT2] Perintah Blynk Relay 2 = %d terkirim!\n", val);
            break;
        }
    }
}

// ========================================================
// HTML TEMPLATES FOR 4 PAGES (LITTLEFS BACKED + EMBEDDED FALLBACK)
// ========================================================
static const char NAV_BAR[] PROGMEM = 
"<nav style='display:flex;gap:10px;margin-bottom:20px;'>"
"<a href='/'><button>Dashboard🗿</button></a>"
"<a href='/pairing'><button>Pairing Node</button></a>"
"<a href='/recovery'><button>Recovery</button></a>"
"<a href='/update'><button>Update OTA</button></a>"
"</nav>";

static const char HTML_HEAD[] PROGMEM = 
"<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>"
"<title>Smart PDU Gateway</title>"
"<style>"
"body{font-family:system-ui,-apple-system,sans-serif;background:#0f172a;color:#f8fafc;padding:20px;margin:0 auto;max-width:700px;}"
".card{background:#1e293b;border:1px solid #334155;border-radius:12px;padding:16px;margin-bottom:16px;}"
"button{background:#3b82f6;color:#fff;border:none;border-radius:8px;padding:10px 16px;font-weight:600;cursor:pointer;}"
"button:hover{background:#2563eb;}"
".btn-danger{background:#ef4444;}.btn-danger:hover{background:#dc2626;}"
".btn-hold{position:relative;overflow:hidden;user-select:none;-webkit-user-select:none;touch-action:none;min-width:140px;text-align:center;}"
".btn-hold .fill{position:absolute;top:0;left:0;bottom:0;width:0%;background:rgba(255,255,255,0.35);pointer-events:none;}"
".btn-hold .lbl{position:relative;z-index:2;pointer-events:none;display:inline-block;}"
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:12px;}"
".metric{background:#0f172a;padding:12px;border-radius:8px;text-align:center;}"
".val{font-size:24px;font-weight:700;color:#38bdf8;}"
".switch{display:flex;justify-content:space-between;align-items:center;padding:8px 0;border-bottom:1px solid #334155;}"
"input[type=number],input[type=text]{background:#0f172a;border:1px solid #334155;color:#f8fafc;padding:8px;border-radius:6px;width:100%;box-sizing:border-box;}"
"</style></head><body>";

// ========================================================
// SETUP UTAMA
// ========================================================
void setup() {
    setCpuFrequencyMhz(160); // Turunkan CPU clock ke 160MHz (Optimalisasi suhu dingin & power tanpa jitter)
    esp_brownout_init(); // Aktifkan hardware brownout detector untuk proteksi power supply toko 24/7

    // 0. DETEKSI & SIMPAN RESET REASON & COUNTER BOOT KE NVS FLASH PADA SETIAP BOOT
    esp_reset_reason_t rstReason = esp_reset_reason();
    g_mcuResetReason = rstReason;
    // Fast Restore: Pulihkan relay seketika (0ms) untuk SW reboot, WDT reboot, maupun Panic reboot
    // Listrik beban toko tidak perlu mati 15+ detik jika reboot hanya karena watchdog/software!
    bool isFastRestore = (rstReason == ESP_RST_SW || rstReason == ESP_RST_TASK_WDT || 
                          rstReason == ESP_RST_PANIC || rstReason == ESP_RST_INT_WDT || rstReason == ESP_RST_WDT);

    Preferences pBoot;
    pBoot.begin("pdu_boot", false);
    g_bootCount = pBoot.getUInt("count", 0) + 1;
    pBoot.putUInt("count", g_bootCount);
    pBoot.putUChar("last_rst", (uint8_t)rstReason);

    // Ambil Breadcrumb dari RTC Memory (Selamat dari WDT & Panic reboot)
    if (rtcMagic == 0xB00B1E55 && rstReason != ESP_RST_POWERON) {
        g_lastRstReason = (uint32_t)rstReason;
        g_lastCrumbNet  = crumbNet;
        g_lastCrumbCtl  = crumbCtl;
        g_lastUptimeS   = lastUptimeS;
        pBoot.putUInt("rst", g_lastRstReason);
        pBoot.putUChar("cNet", g_lastCrumbNet);
        pBoot.putUChar("cCtl", g_lastCrumbCtl);
        pBoot.putUInt("upS", g_lastUptimeS);
    } else {
        g_lastRstReason = pBoot.getUInt("rst", (uint32_t)rstReason);
        g_lastCrumbNet  = pBoot.getUChar("cNet", 0);
        g_lastCrumbCtl  = pBoot.getUChar("cCtl", 0);
        g_lastUptimeS   = pBoot.getUInt("upS", 0);
    }
    pBoot.end();

    // Reset RTC Memory untuk sesi boot berjalan sekarang
    rtcMagic = 0xB00B1E55;
    rtcBootCount = g_bootCount;
    crumbNet = 0;
    crumbCtl = 0;
    lastUptimeS = 0;

    // 0b. RESTORASI PIN RELAY SEGERA (0ms latency, sebelum serial & delay apapun)
    prefs.begin("pdu_store", true); // Mode read-only
    for (int i = 0; i < 4; i++) {
        bool lastState = prefs.getBool(("rl_" + String(i)).c_str(), false);
        targetRelayStates[i] = lastState;
        if (isFastRestore && lastState) {
            digitalWrite(RELAY_PINS[i], LOW); // Pertahankan/pulihkan ON seketika tanpa jeda
        } else {
            digitalWrite(RELAY_PINS[i], HIGH); // OFF pada fresh power-on / brownout
            targetRelayStates[i] = false;
        }
        pinMode(RELAY_PINS[i], OUTPUT);
        gpio_hold_dis((gpio_num_t)RELAY_PINS[i]);

        // Soft Start Relay selalu di-set OFF (HIGH) saat boot
        digitalWrite(RELAY_SS_PINS[i], HIGH);
        pinMode(RELAY_SS_PINS[i], OUTPUT);
        gpio_hold_dis((gpio_num_t)RELAY_SS_PINS[i]);
    }
    gpio_deep_sleep_hold_dis();
    prefs.end();

    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================================");
    Serial.println("  SMART PDU GATEWAY UTAMA (ESP32-S3 N16R8) DUAL-CORE RTOS");
    Serial.println("========================================================");

    // Deteksi Hardware Memory N16R8 (16MB Flash + 8MB Octal PSRAM)
    Serial.printf("[MEM INFO] CPU Clock   : %u MHz (Dual Core Xtensa LX7)\n", ESP.getCpuFreqMHz());
    Serial.printf("[MEM INFO] Flash Bus   : %u MB (Speed: %u MHz, Quad SPI)\n", ESP.getFlashChipSize() / (1024 * 1024), ESP.getFlashChipSpeed() / 1000000);
    Serial.printf("[MEM INFO] Internal SRAM : %u KB (Free: %u KB)\n", ESP.getHeapSize() / 1024, ESP.getFreeHeap() / 1024);
    if (psramFound()) {
        Serial.printf("[MEM INFO] ✅ Octal PSRAM Terdeteksi : %u KB (%u MB, Speed: 80 MHz OPI) | Free: %u KB\n",
                      ESP.getPsramSize() / 1024, ESP.getPsramSize() / (1024 * 1024), ESP.getFreePsram() / 1024);
        psramHistoryBuffer = (TelemetryHistoryPoint*)ps_malloc(sizeof(TelemetryHistoryPoint) * PSRAM_HISTORY_MAX_POINTS);
        if (psramHistoryBuffer) {
            memset(psramHistoryBuffer, 0, sizeof(TelemetryHistoryPoint) * PSRAM_HISTORY_MAX_POINTS);
            Serial.printf("[PSRAM LOGGER] ✅ Alokasi %d Titik Riwayat 1-Detik (%u KB) di Octal PSRAM Berhasil!\n",
                          PSRAM_HISTORY_MAX_POINTS, (unsigned)((sizeof(TelemetryHistoryPoint) * PSRAM_HISTORY_MAX_POINTS) / 1024));
        }
    } else {
        Serial.println("[MEM INFO] ⚠️ PSRAM Tidak Terdeteksi atau Belum Aktif!");
    }

    // Baca Riwayat Insiden & Ring Buffer 10 Kejadian dari Flash NVS
    Preferences pBb;
    pBb.begin("pdu_bb", true);
    g_tripCount = pBb.getUChar("count", 0);
    g_tripHead  = pBb.getUChar("head", 0);
    if (g_tripCount > BLACKBOX_MAX_RECORDS) g_tripCount = BLACKBOX_MAX_RECORDS;
    if (g_tripHead >= BLACKBOX_MAX_RECORDS) g_tripHead = 0;

    for (uint8_t i = 0; i < g_tripCount; i++) {
        String key = "trip_" + String(i);
        if (pBb.isKey(key.c_str())) {
            pBb.getBytes(key.c_str(), &g_tripRecords[i], sizeof(BlackboxRecord));
        }
    }

    if (pBb.isKey("last_trip")) {
        pBb.getBytes("last_trip", &g_lastTripRecord, sizeof(g_lastTripRecord));
        if (g_lastTripRecord.magic == 0x50445542) {
            g_hasTripRecord = true;
        }
    } else if (g_tripCount > 0) {
        uint8_t latestIdx = (g_tripHead == 0 ? g_tripCount - 1 : g_tripHead - 1);
        g_lastTripRecord = g_tripRecords[latestIdx];
        g_hasTripRecord = (g_lastTripRecord.magic == 0x50445542);
    }
    pBb.end();

    Serial.printf("[BOOT NVS] 🔄 Boot Counter: #%u | Reset Reason: %d (%s)\n", 
                  g_bootCount, (int)rstReason, getResetReasonStr((uint8_t)rstReason));
    Serial.printf("[BLACKBOX BOOT] 🔍 Memuat %d Riwayat Insiden dari Flash NVS (Head Slot: %d)\n", 
                  g_tripCount, g_tripHead);
    if (g_hasTripRecord) {
        Serial.printf("[BLACKBOX BOOT] Insiden Terakhir: %s | Uptime: %lus | Volt: %.1fV (Min5s: %.1fV, Max5s: %.1fV) | Arus: %.2fA | Boot #%u | Reset: %s\n",
                      g_lastTripRecord.tripLabel, (unsigned long)g_lastTripRecord.timestamp,
                      g_lastTripRecord.voltage, g_lastTripRecord.minVolt5s, g_lastTripRecord.maxVolt5s,
                      g_lastTripRecord.current, g_lastTripRecord.bootCount, getResetReasonStr(g_lastTripRecord.rstReason));
    }

    // 1. Inisialisasi LittleFS File System
    if (!LittleFS.begin(true)) {
        Serial.println("[LITTLEFS] Mount Gagal, memformat partisi...");
    } else {
        Serial.println("[LITTLEFS] Mount Berhasil!");
    }

    // 2. Inisialisasi FreeRTOS Queues & Mutex
    telemetryQueue  = xQueueCreate(1, sizeof(TelemetryData));
    pduCommandQueue = xQueueCreate(10, sizeof(PduCommand));
    relayMutex      = xSemaphoreCreateMutex();

    // 3. Inisialisasi Native ESP-IDF Task Watchdog Timer
    esp_task_wdt_init(15, true); // 15 Detik timeout (mencegah false panic saat DNS/Blynk blocking)

    // 4. Inisialisasi UART PZEM di HardwareSerial 1 (ESP32-S3)
    PzemSerial.setRxBufferSize(256);
    PzemSerial.begin(9600, SERIAL_8N1, PIN_PZEM_RX, PIN_PZEM_TX);
    Serial.printf("[PZEM UART] HardwareSerial 1 Aktif (RX=%d, TX=%d)\n", PIN_PZEM_RX, PIN_PZEM_TX);

    // 5. Inisialisasi WiFi Mode di setup() (Wajib di thread utama agar LwIP stabil)
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false); // Matikan 802.11 Modem Sleep agar respon HTTP Web & Ping instan tanpa drop!

    // ====================================================
    // 6. SETUP WEB SERVER 4 HALAMAN + REST API
    // ====================================================

    // HALAMAN 1: DASHBOARD UTAMA (RUNNING)
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        String html = FPSTR(HTML_HEAD);
        html += "<h2>Synctech Smart PDU Gateway</h2>";
        html += FPSTR(NAV_BAR);

            // KARTU STATUS KONEKSI (JARINGAN & CLOUD)
            html += "<div class='card'><h3>Status Jaringan & Cloud</h3><div class='grid'>";
            bool isStaConnected = (WiFi.status() == WL_CONNECTED);
            bool isBlynkConnected = Blynk.connected();
            html += "<div class='metric'><div>Wi-Fi Router</div><div class='val' style='font-size:16px;' id='wifi_stat'>" + 
                    String(isStaConnected ? "<span style='color:#4ade80'>Terhubung (" + WiFi.SSID() + ")</span>" : "<span style='color:#ef4444'>Terputus / Mencari</span>") + "</div></div>";
            html += "<div class='metric'><div>IP Address</div><div class='val' style='font-size:16px;' id='wifi_ip'>" + 
                    String(isStaConnected ? WiFi.localIP().toString() : "192.168.4.1 (AP)") + "</div></div>";
            html += "<div class='metric'><div>Blynk IoT Cloud</div><div class='val' style='font-size:16px;' id='blynk_stat'>" + 
                    String(isBlynkConnected ? "<span style='color:#4ade80'>ONLINE 🟢</span>" : "<span style='color:#ef4444'>OFFLINE 🔴</span>") + "</div></div>";
            html += "<div class='metric'><div>Sinyal Wi-Fi (RSSI)</div><div class='val' style='font-size:16px;' id='wifi_rssi'>" + 
                    String(isStaConnected ? String(WiFi.RSSI()) + " dBm" : "-") + "</div></div>";
            html += "</div></div>";

            html += "<div class='card'><h3>Metrik Listrik AC & Hardware</h3><div class='grid'>";
        html += "<div class='metric'><div>Tegangan</div><div class='val' id='v'>" + String(currentTelemetry.voltage, 1) + " V</div></div>";
        html += "<div class='metric'><div>Arus Total</div><div class='val' id='a'>" + String(currentTelemetry.current, 2) + " A</div></div>";
        html += "<div class='metric'><div>Daya Aktif</div><div class='val' id='w'>" + String(currentTelemetry.power, 0) + " W</div></div>";
        html += "<div class='metric'><div>Frekuensi</div><div class='val' id='f'>" + String(currentTelemetry.frequency, 1) + " Hz</div></div>";
        html += "<div class='metric'><div>Power Factor</div><div class='val' id='pf'>" + String(currentTelemetry.pf, 2) + "</div></div>";
        html += "<div class='metric'><div>Energi</div><div class='val' id='e'>" + String(currentTelemetry.energy, 2) + " kWh</div></div>";
        html += "<div class='metric'><div>Suhu ESP32-S3</div><div class='val' id='chip_temp'>" + String(currentTelemetry.chipTemp, 1) + " °C</div></div>";
        html += "<div class='metric'><div>Clock CPU</div><div class='val' id='cpu_speed'>" + String(ESP.getCpuFreqMHz()) + " MHz</div></div>";
        html += "<div class='metric'><div>Beban Core 0 (Net)</div><div class='val' id='cpu_load0'>" + String(currentTelemetry.cpuLoad0, 0) + " %</div></div>";
        html += "<div class='metric'><div>Beban Core 1 (Ctrl)</div><div class='val' id='cpu_load1'>" + String(currentTelemetry.cpuLoad1, 0) + " %</div></div>";
        html += "</div></div>";

        // KARTU BLACKBOX EVENT TRIP TERAKHIR & RING BUFFER 10 KEJADIAN (PERMANEN DI DASHBOARD)
        html += "<div class='card' id='card_blackbox' style='border-left:5px solid " + String(g_hasTripRecord ? "#ef4444" : "#10b981") + ";'>";
        html += "<div style='display:flex;justify-content:space-between;align-items:center;'>";
        html += "<h3 style='margin:0;' id='bb_title'>" + String(g_hasTripRecord ? "<span style='color:#f87171;'>🚨 Blackbox: Catatan Insiden Trip (" + String(g_tripCount) + "/10)</span>" : "<span style='color:#4ade80;'>🛡️ Blackbox: Status Proteksi Sistem</span>") + "</h3>";
        html += "<div><a href='/api/trips' target='_blank' style='margin-right:6px;'><button style='padding:4px 8px;font-size:11px;background:#0284c7;'>Raw JSON</button></a>";
        html += "<button onclick=\"fetch('/api/reset-blackbox',{method:'POST'}).then(()=>{location.reload();})\" style='padding:4px 8px;font-size:11px;background:#475569;'>Reset Log</button></div></div>";

        html += "<div style='font-size:12px;color:#94a3b8;margin-top:6px;'>Status MCU: Boot <b>#" + String(g_bootCount) + "</b> | Reset: <b>" + String(getResetReasonStr((uint8_t)g_mcuResetReason)) + "</b>";
        if (g_lastCrumbNet > 0 || g_lastCrumbCtl > 0) {
            html += "<br><span style='color:#38bdf8;'>🔍 <b>Breadcrumb Hang Terakhir:</b> Core 0 [<b>" + String(getCrumbNetStr(g_lastCrumbNet)) + "</b>] | Core 1 [<b>" + String(getCrumbCtlStr(g_lastCrumbCtl)) + "</b>] (Uptime: " + String(g_lastUptimeS) + "s)</span>";
        }
        html += "</div>";

        html += "<div style='font-size:13px;line-height:1.6;color:#cbd5e1;margin-top:10px;' id='bb_content'>";
        if (g_hasTripRecord) {
            html += "<p style='margin:4px 0;'><b>Insiden Terakhir:</b> <span style='color:#f87171;font-weight:bold;'>" + String(g_lastTripRecord.tripLabel) + "</span> (Boot #" + String(g_lastTripRecord.bootCount) + ")</p>";
            html += "<p style='margin:4px 0;'><b>Tegangan saat Trip:</b> <span style='color:#fbbf24;font-weight:bold;'>" + String(g_lastTripRecord.voltage, 1) + " V</span> | <b>Jendela 5s:</b> Min " + String(g_lastTripRecord.minVolt5s, 1) + " V / Max " + String(g_lastTripRecord.maxVolt5s, 1) + " V</p>";
            html += "<p style='margin:4px 0;'><b>Arus:</b> " + String(g_lastTripRecord.current, 2) + " A | <b>Daya:</b> " + String(g_lastTripRecord.power, 0) + " W | <b>Suhu:</b> " + String(g_lastTripRecord.chipTemp, 1) + " °C | <b>Uptime:</b> " + String(g_lastTripRecord.timestamp) + "s</p>";

            if (g_tripCount > 1) {
                html += "<div style='margin-top:10px;border-top:1px solid #334155;padding-top:8px;'><b>Riwayat " + String(g_tripCount) + " Kejadian Terakhir:</b>";
                html += "<div style='max-height:140px;overflow-y:auto;margin-top:6px;'><table style='width:100%;font-size:11px;border-collapse:collapse;'>";
                html += "<tr style='color:#94a3b8;text-align:left;border-bottom:1px solid #334155;'><th>#</th><th>Boot</th><th>Pemicu</th><th>V_Trip</th><th>5s (Min-Max)</th><th>Arus</th></tr>";
                for (int i = 0; i < g_tripCount; i++) {
                    int idx = (g_tripHead - 1 - i + BLACKBOX_MAX_RECORDS) % BLACKBOX_MAX_RECORDS;
                    if (idx >= g_tripCount && g_tripCount < BLACKBOX_MAX_RECORDS) continue;
                    const BlackboxRecord &r = g_tripRecords[idx];
                    if (r.magic != 0x50445542) continue;
                    html += "<tr style='border-bottom:1px solid #1e293b;'><td style='padding:3px 2px;'>" + String(i + 1) + "</td>";
                    html += "<td>#" + String(r.bootCount) + "</td>";
                    html += "<td style='color:#f87171;'>" + String(r.tripLabel) + "</td>";
                    html += "<td style='color:#fbbf24;'>" + String(r.voltage, 1) + "V</td>";
                    html += "<td>" + String(r.minVolt5s, 1) + " - " + String(r.maxVolt5s, 1) + "V</td>";
                    html += "<td>" + String(r.current, 2) + "A</td></tr>";
                }
                html += "</table></div></div>";
            }
        } else {
            html += "<p style='margin:4px 0;color:#4ade80;'>✅ <b>Kondisi Listrik Aman:</b> Belum ada catatan trip / cut-off pada sistem.</p>";
            html += "<p style='margin:4px 0;color:#94a3b8;font-size:12px;'>Sistem siap merekam snapshot tegangan (sag vs glitch 5 detik), arus, daya, dan suhu saat terjadi anomali kelistrikan.</p>";
        }
        html += "</div></div>";

        // KARTU GRAFIK RESOLUSI 1-DETIK (PSRAM STREAM)
        html += "<div class='card'><h3>📈 Grafik Riwayat Tegangan & Arus (1 Detik / PSRAM)</h3>";
        html += "<canvas id='chartCanvas' width='640' height='200' style='width:100%;height:180px;background:#090d16;border-radius:8px;'></canvas>";
        html += "<div style='display:flex;justify-content:space-between;margin-top:6px;font-size:11px;color:#94a3b8;'>";
        html += "<span>🔵 Tegangan AC (Sumbu Kiri)</span><span>🟠 Arus Beban (Sumbu Kanan)</span><span id='chart_points'>0 detik (Live)</span></div></div>";

        html += "<div class='card'><h3>Kontrol Relay Beban (Lantai 1)</h3>";
        for (int i = 0; i < 4; i++) {
            html += "<div class='switch'><span>Channel " + String(i + 1) + "</span>";
            html += "<button id='btn_relay_" + String(i) + "' class='btn-hold' data-type='relay' data-ch='" + String(i) + "' style='background:" + String(targetRelayStates[i] ? "#ef4444" : "#3b82f6") + ";'>";
            html += "<div class='fill' id='fill_relay_" + String(i) + "'></div>";
            html += "<span class='lbl' id='lbl_relay_" + String(i) + "'>" + String(targetRelayStates[i] ? "MATIKAN" : "NYALAKAN") + "</span></button></div>";
        }
        html += "<div class='switch'><span>Bypass / Paksa IDLE</span>";
        html += "<button onclick='forceIdle()' style='background:#64748b;'>Reset ke Normal (IDLE)</button></div>";
        html += "<div style='margin-top:15px;'><button id='btn_seq' class='btn-hold' data-type='seq' style='width:100%;'>";
        html += "<div class='fill' id='fill_seq'></div>";
        html += "<span class='lbl' id='lbl_seq'>🔄 Pemicu Power Sequencer (" + String((float)g_btnHoldMs / 1000.0f, 1) + "s)</span></button></div>";
        html += "</div>";

        // KARTU KONTROL RELAY LANTAI 2 (SUB-PDU VIA ESP-NOW)
        html += "<div class='card'><h3>Kontrol Relay Sub-PDU (Lantai 2 via ESP-NOW)</h3>";
        for (int i = 0; i < 4; i++) {
            html += "<div class='switch'><span>Relay Lt 2 - Ch " + String(i + 1) + "</span>";
            html += "<button onclick='toggleClientRelay(" + String(i + 1) + ")'>Toggle Relay " + String(i + 1) + "</button></div>";
        }
        html += "</div>";

        html += "<script>"
                "function toggleRelay(ch){fetch('/api/relay?ch='+ch+'&state=toggle',{method:'POST'});}"
                "function forceIdle(){fetch('/api/force-idle',{method:'POST'}).then(function(){alert('Sequencer di-reset ke IDLE!');});}"
                "function toggleClientRelay(ch){fetch('/api/client-relay?node=1&ch='+ch+'&state=toggle',{method:'POST'}).then(function(r){return r.text();}).then(function(t){alert(t);});}"
                "function triggerSeq(){fetch('/api/sequencer',{method:'POST'});}"
                "var holdTimer=null,activeBtn=null,holdStartTime=0,holdTriggered=false,HOLD_MS=" + String(g_btnHoldMs) + ";"
                "var lastRelays=[false,false,false,false];"
                "function startHold(btn){"
                "if(btn.disabled||activeBtn)return;"
                "activeBtn=btn;holdStartTime=Date.now();holdTriggered=false;"
                "var fill=btn.querySelector('.fill');var lbl=btn.querySelector('.lbl');"
                "if(fill){fill.style.width='0%';fill.style.background='rgba(255,255,255,0.35)';}"
                "clearInterval(holdTimer);"
                "holdTimer=setInterval(function(){"
                "var el=Date.now()-holdStartTime;var pct=Math.min(100,(el/HOLD_MS)*100);"
                "if(fill)fill.style.width=pct+'%';"
                "if(el>=HOLD_MS&&!holdTriggered){"
                "holdTriggered=true;clearInterval(holdTimer);"
                "if(fill)fill.style.background='rgba(74,222,128,0.7)';"
                "if(lbl)lbl.innerText='⚡ MEMICU...';"
                "if(navigator.vibrate)navigator.vibrate(60);"
                "var type=btn.getAttribute('data-type');var ch=btn.getAttribute('data-ch');"
                "if(type==='relay')toggleRelay(ch);"
                "else if(type==='seq')triggerSeq();"
                "setTimeout(function(){endHold(btn);},350);"
                "}"
                "},30);"
                "}"
                "function endHold(btn){"
                "if(!btn)btn=activeBtn;if(!btn)return;"
                "clearInterval(holdTimer);"
                "var fill=btn.querySelector('.fill');"
                "if(fill){fill.style.width='0%';fill.style.background='rgba(255,255,255,0.35)';}"
                "restoreBtn(btn);"
                "activeBtn=null;holdTriggered=false;"
                "}"
                "function restoreBtn(btn){"
                "var type=btn.getAttribute('data-type');var lbl=btn.querySelector('.lbl');"
                "if(!lbl)return;"
                "if(type==='relay'){"
                "var c=parseInt(btn.getAttribute('data-ch'));lbl.innerText=lastRelays[c]?'MATIKAN':'NYALAKAN';btn.style.backgroundColor=lastRelays[c]?'#ef4444':'#3b82f6';"
                "}else if(type==='seq'){"
                "lbl.innerText='🔄 Pemicu Power Sequencer ('+(HOLD_MS/1000).toFixed(1)+'s)';"
                "}"
                "}"
                "function bindHold(btn){"
                "btn.addEventListener('contextmenu',function(e){e.preventDefault();});"
                "btn.addEventListener('mousedown',function(e){if(e.button===0)startHold(btn);});"
                "btn.addEventListener('mouseup',function(){if(!holdTriggered)endHold(btn);});"
                "btn.addEventListener('mouseleave',function(){if(!holdTriggered)endHold(btn);});"
                "btn.addEventListener('touchstart',function(e){e.preventDefault();startHold(btn);},{passive:false});"
                "btn.addEventListener('touchend',function(e){e.preventDefault();if(!holdTriggered)endHold(btn);});"
                "btn.addEventListener('touchcancel',function(e){e.preventDefault();if(!holdTriggered)endHold(btn);});"
                "}"
                "setInterval(function(){"
                "fetch('/api/status').then(function(r){return r.json();}).then(function(d){"
                "if(d.btnHoldMs)HOLD_MS=d.btnHoldMs;"
                "document.getElementById('v').innerText=d.voltage.toFixed(1)+' V';"
                "document.getElementById('a').innerText=d.current.toFixed(2)+' A';"
                "document.getElementById('w').innerText=d.power.toFixed(0)+' W';"
                "if(document.getElementById('f'))document.getElementById('f').innerText=d.frequency.toFixed(1)+' Hz';"
                "if(document.getElementById('pf'))document.getElementById('pf').innerText=d.pf.toFixed(2);"
                "document.getElementById('e').innerText=d.energy.toFixed(2)+' kWh';"
                "var isSeqBusy=(d.seqState===2||d.seqState===3);"
                "var sBtn=document.getElementById('btn_seq');"
                "if(sBtn&&activeBtn!==sBtn){"
                "var sLbl=sBtn.querySelector('.lbl');"
                "if(isSeqBusy){if(sLbl)sLbl.innerText='⏳ Sequencer Sedang Berjalan...';sBtn.disabled=true;sBtn.style.opacity='0.6';}"
                "else if(d.seqState===1){var vOk=(d.voltage>=175.0&&d.voltage<=255.0);if(sLbl)sLbl.innerText=vOk?'⏳ Menstabilkan Tegangan (5s)...':'⚠️ Menunggu Tegangan Normal ('+d.voltage.toFixed(0)+'V)...';sBtn.disabled=false;sBtn.style.opacity='1';}"
                "else{if(sLbl)sLbl.innerText='🔄 Pemicu Power Sequencer ('+(HOLD_MS/1000).toFixed(1)+'s)';sBtn.disabled=false;sBtn.style.opacity='1';}"
                "}"
                "if(d.relays){"
                "for(var i=0;i<4;i++){"
                "lastRelays[i]=d.relays[i];var b=document.getElementById('btn_relay_'+i);"
                "if(b&&activeBtn!==b){"
                "var bLbl=b.querySelector('.lbl');if(bLbl)bLbl.innerText=d.relays[i]?'MATIKAN':'NYALAKAN';b.style.backgroundColor=d.relays[i]?'#ef4444':'#3b82f6';"
                "b.disabled=isSeqBusy;b.style.opacity=isSeqBusy?'0.5':'1';b.style.cursor=isSeqBusy?'not-allowed':'pointer';"
                "}"
                "}"
                "}"
                "if(document.getElementById('wifi_stat')){"
                "document.getElementById('wifi_stat').innerHTML=d.wifiConnected?'<span style=\"color:#4ade80\">Terhubung ('+d.ssid+')</span>':'<span style=\"color:#ef4444\">Mencari Router...</span>';"
                "document.getElementById('wifi_ip').innerText=d.ip;"
                "document.getElementById('blynk_stat').innerHTML=d.blynkConnected?'<span style=\"color:#4ade80\">ONLINE 🟢</span>':'<span style=\"color:#ef4444\">OFFLINE 🔴</span>';"
                "document.getElementById('wifi_rssi').innerText=d.wifiConnected?(d.rssi+' dBm'):'-';"
                "}"
                "if(d.chipTemp!==undefined&&document.getElementById('chip_temp')){"
                "document.getElementById('chip_temp').innerText=d.chipTemp.toFixed(1)+' °C';"
                "if(d.chipTemp>=70.0)document.getElementById('chip_temp').style.color='#ef4444';"
                "else if(d.chipTemp>=58.0)document.getElementById('chip_temp').style.color='#f97316';"
                "else document.getElementById('chip_temp').style.color='#4ade80';"
                "}"
                "if(d.cpuSpeed!==undefined&&document.getElementById('cpu_speed'))document.getElementById('cpu_speed').innerText=d.cpuSpeed+' MHz';"
                "if(d.cpuLoad0!==undefined&&document.getElementById('cpu_load0')){"
                "document.getElementById('cpu_load0').innerText=d.cpuLoad0.toFixed(0)+' %';"
                "if(d.cpuLoad0>=70.0)document.getElementById('cpu_load0').style.color='#ef4444';"
                "else if(d.cpuLoad0>=35.0)document.getElementById('cpu_load0').style.color='#f97316';"
                "else document.getElementById('cpu_load0').style.color='#38bdf8';"
                "}"
                "if(d.cpuLoad1!==undefined&&document.getElementById('cpu_load1')){"
                "document.getElementById('cpu_load1').innerText=d.cpuLoad1.toFixed(0)+' %';"
                "if(d.cpuLoad1>=70.0)document.getElementById('cpu_load1').style.color='#ef4444';"
                "else if(d.cpuLoad1>=35.0)document.getElementById('cpu_load1').style.color='#f97316';"
                "else document.getElementById('cpu_load1').style.color='#38bdf8';"
                "}"
                "if(document.getElementById('card_blackbox')){"
                "if(d.hasTrip){"
                "document.getElementById('card_blackbox').style.borderLeftColor='#ef4444';"
                "if(document.getElementById('bb_title'))document.getElementById('bb_title').innerHTML='<span style=\"color:#f87171;\">🚨 Blackbox: Catatan Insiden Trip ('+(d.tripCount||1)+'/10)</span>';"
                "var min5=(d.tripMinVolt5s!==undefined)?d.tripMinVolt5s.toFixed(1):'-';"
                "var max5=(d.tripMaxVolt5s!==undefined)?d.tripMaxVolt5s.toFixed(1):'-';"
                "var bNo=(d.tripBoot!==undefined)?d.tripBoot:'-';"
                "document.getElementById('bb_content').innerHTML='<p style=\"margin:4px 0;\"><b>Insiden Terakhir:</b> <span style=\"color:#f87171;font-weight:bold;\">'+d.tripLabel+'</span> (Boot #'+bNo+')</p>'+'<p style=\"margin:4px 0;\"><b>Tegangan saat Trip:</b> <span style=\"color:#fbbf24;font-weight:bold;\">'+d.tripVolt.toFixed(1)+' V</span> | <b>Jendela 5s:</b> Min '+min5+' V / Max '+max5+' V</p>'+'<p style=\"margin:4px 0;\"><b>Arus:</b> '+d.tripCurr.toFixed(2)+' A | <b>Suhu:</b> '+d.tripTemp.toFixed(1)+' °C | <b>Uptime:</b> '+d.tripTime+' detik</p><p style=\"margin:4px 0;font-size:11px;color:#94a3b8;\">💡 Refresh untuk tabel 10 log lengkap atau buka <a href=\"/api/trips\" target=\"_blank\" style=\"color:#38bdf8;\">/api/trips</a></p>';"
                "}else{"
                "document.getElementById('card_blackbox').style.borderLeftColor='#10b981';"
                "if(document.getElementById('bb_title'))document.getElementById('bb_title').innerHTML='<span style=\"color:#4ade80;\">🛡️ Blackbox: Status Proteksi Sistem</span>';"
                "document.getElementById('bb_content').innerHTML='<p style=\"margin:4px 0;color:#4ade80;\">✅ <b>Kondisi Listrik Aman:</b> Belum ada catatan trip / cut-off pada sistem.</p><p style=\"margin:4px 0;color:#94a3b8;font-size:12px;\">Sistem siap merekam snapshot tegangan (sag vs glitch 5 detik), arus, daya, dan suhu saat terjadi anomali kelistrikan.</p>';"
                "}"
                "}"
                "});"
                "},1000);"
                "var cvs=document.getElementById('chartCanvas');"
                "if(cvs){"
                "var ctx=cvs.getContext('2d');"
                "function renderChart(){"
                "fetch('/api/history?limit=180').then(function(r){return r.json();}).then(function(res){"
                "if(!res.data||res.data.length<2)return;"
                "var pts=res.data;"
                "if(document.getElementById('chart_points'))document.getElementById('chart_points').innerText=pts.length+' detik (Live)';"
                "var W=cvs.width,H=cvs.height;"
                "var padL=44,padR=40,padT=22,padB=22;"
                "var pW=W-padL-padR,pH=H-padT-padB,pB=H-padB;"
                "ctx.clearRect(0,0,W,H);"
                "ctx.font='10px monospace';"
                "var vTicks=[{v:260,a:'16A'},{v:240,a:'12A'},{v:220,a:'8A'},{v:200,a:'4A'},{v:180,a:'0A'}];"
                "var minV=175,maxV=265;"
                "for(var k=0;k<vTicks.length;k++){"
                "var tV=vTicks[k].v;var norm=(tV-minV)/(maxV-minV);var y=pB-(norm*pH);"
                "ctx.setLineDash([3,4]);ctx.strokeStyle='#1e293b';ctx.lineWidth=1;"
                "ctx.beginPath();ctx.moveTo(padL,y);ctx.lineTo(padL+pW,y);ctx.stroke();"
                "ctx.setLineDash([]);ctx.fillStyle='#38bdf8';ctx.textAlign='right';"
                "ctx.fillText(tV+'V',padL-6,y+3);"
                "ctx.fillStyle='#fb923c';ctx.textAlign='left';"
                "ctx.fillText(vTicks[k].a,padL+pW+6,y+3);"
                "}"
                "ctx.strokeStyle='#334155';ctx.lineWidth=1;ctx.beginPath();"
                "ctx.moveTo(padL,pB);ctx.lineTo(padL+pW,pB);ctx.stroke();"
                "var xL=[{p:0.0,t:'-180s'},{p:0.33,t:'-120s'},{p:0.66,t:'-60s'},{p:1.0,t:'Live'}];"
                "ctx.fillStyle='#64748b';"
                "for(var m=0;m<xL.length;m++){"
                "var x=padL+(xL[m].p*pW);ctx.textAlign=(m===0)?'left':(m===xL.length-1)?'right':'center';"
                "ctx.fillText(xL[m].t,x,H-6);"
                "}"
                "ctx.setLineDash([]);ctx.strokeStyle='#38bdf8';ctx.lineWidth=2;ctx.beginPath();"
                "var step=pW/(pts.length-1);"
                "for(var i=0;i<pts.length;i++){"
                "var nV=Math.max(0,Math.min(1,(pts[i].v-minV)/(maxV-minV)));"
                "var ptY=pB-(nV*pH);var ptX=padL+(i*step);"
                "if(i===0)ctx.moveTo(ptX,ptY);else ctx.lineTo(ptX,ptY);"
                "}"
                "ctx.stroke();"
                "ctx.strokeStyle='#fb923c';ctx.lineWidth=1.8;ctx.beginPath();"
                "for(var j=0;j<pts.length;j++){"
                "var nA=Math.max(0,Math.min(1,pts[j].a/16.0));"
                "var ptYA=pB-(nA*pH);var ptXA=padL+(j*step);"
                "if(j===0)ctx.moveTo(ptXA,ptYA);else ctx.lineTo(ptXA,ptYA);"
                "}"
                "ctx.stroke();"
                "var lastP=pts[pts.length-1];"
                "ctx.fillStyle='#38bdf8';ctx.textAlign='left';"
                "ctx.fillText('V: '+lastP.v.toFixed(1)+'V',padL,14);"
                "ctx.fillStyle='#fb923c';"
                "ctx.fillText('I: '+lastP.a.toFixed(2)+'A ('+lastP.w.toFixed(0)+'W)',padL+110,14);"
                "}).catch(function(){});"
                "}"
                "setInterval(renderChart,2500);renderChart();"
                "}"
                "document.querySelectorAll('.btn-hold').forEach(bindHold);"
                "</script></body></html>";
        request->send(200, "text/html; charset=utf-8", html);
    });

    // HALAMAN 2: PAIRING KLIEN ESP-NOW
    server.on("/pairing", HTTP_GET, [](AsyncWebServerRequest *request) {
                if (!checkAuth(request)) return;
                String html = FPSTR(HTML_HEAD);
                    html += "<h2>📡 Pairing & Manajemen Klien Node</h2>";
                    html += FPSTR(NAV_BAR);
                    html += "<div class='card'><h3>Status Pairing</h3>";
                    html += "<p>Mode Pairing: <b>" + String(isPairingWindowActive ? "<span style='color:#4ade80'>AKTIF (Siaga 60s)</span>" : "TIDAK AKTIF") + "</b></p>";
                    html += "<p>Channel Radio Gateway: <b>Ch " + String(WiFi.channel()) + "</b></p>";
                    html += "<button onclick=\"fetch('/api/pairing',{method:'POST'}).then(()=>location.reload())\">Buka Pairing Window 60 Detik</button>";
                    html += "</div>";

        // Halaman Pairing: Tampilkan daftar node dengan benar
                html += "<div class='card'><h3>Daftar Node Klien Terdaftar (" + String(registeredClientCount) + "/" + String(ESPNOW_MAX_CLIENTS) + ")</h3>";
                html += "<table style='width:100%;text-align:left;border-collapse:collapse;'><tr><th>No</th><th>MAC Address</th><th>Tipe Node</th><th>Status</th></tr>";
                for (int i = 0; i < registeredClientCount; i++) {
                    char macStr[18];
                    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                             clientList[i].mac[0], clientList[i].mac[1], clientList[i].mac[2],
                             clientList[i].mac[3], clientList[i].mac[4], clientList[i].mac[5]);
                    html += "<tr style='border-bottom:1px solid #334155;'><td style='padding:8px 0;'>" + String(i + 1) + "</td>";
                    html += "<td>" + String(macStr) + "</td>";
                    html += "<td>" + getNodeTypeName(clientList[i].type) + "</td>";
                    html += "<td>" + String(clientList[i].isOnline ? "<span style='color:#4ade80'>Online 🟢</span>" : "<span style='color:#94a3b8'>Offline ⚪</span>") + "</td></tr>";
                }
                html += "</table></div></body></html>";
                request->send(200, "text/html; charset=utf-8", html);
                });

                // HALAMAN 3: RECOVERY & THRESHOLD CALIBRATOR
                server.on("/recovery", HTTP_GET, [](AsyncWebServerRequest *request) {
                if (!checkAuth(request)) return;
                String html = FPSTR(HTML_HEAD);
                html += "<h2>🛠️ Mode Recovery & Proteksi Daya</h2>";
                html += FPSTR(NAV_BAR);
                html += "<div class='card'><h3>Kalibrasi Ambang Batas Proteksi (Flash NVS)</h3>";
                html += "<form method='POST' action='/api/thresholds'>";
                html += "<p>Batas Bawah UVLO (Volt AC):<br><input type='number' step='0.1' name='uvlo' value='" + String(g_threshUvlo, 1) + "'></p>";
                html += "<p>Batas Atas Surge (Volt AC):<br><input type='number' step='0.1' name='surge' value='" + String(g_threshSurge, 1) + "'></p>";
                html += "<p>Batas Arus Maks OCP (Ampere MCB):<br><input type='number' step='0.1' name='ocp' value='" + String(g_threshOcp, 1) + "'></p>";
                html += "<p>Jeda Waktu Sequencer (ms):<br><input type='number' step='100' name='seq_delay' value='" + String(g_seqDelayMs) + "'></p>";
                html += "<p>Jeda Waktu Soft Start Pre-charge (ms):<br><input type='number' step='100' min='100' max='10000' name='ss_delay' value='" + String(g_softStartDelayMs) + "'></p>";
                html += "<p>Durasi Tahan Tombol Fisik & Web (ms):<br><input type='number' step='100' min='500' max='10000' name='btn_hold' value='" + String(g_btnHoldMs) + "'></p>";
                html += "<button type='submit'>Simpan Ambang Batas ke Flash</button></form></div>";

                html += "<div class='card'><h3>Pengaturan Blynk IoT Cloud (Modular NVS)</h3>";
                html += "<p>Status Cloud: <b>" + String(g_blynkEnabled ? (Blynk.connected() ? "<span style='color:#4ade80'>ONLINE 🟢</span>" : "<span style='color:#facc15'>CONNECTING / OFFLINE 🟡</span>") : "<span style='color:#ef4444'>DINONAKTIFKAN 🔴</span>") + "</b></p>";
                html += "<p>Template ID Blynk:<br><input type='text' id='blynk_tmpl' value='" + g_blynkTemplateId + "' placeholder='TMPLxxxxxx'></p>";
                html += "<p>Auth Token Blynk:<br><input type='text' id='blynk_tok' value='" + g_blynkAuthToken + "' placeholder='Masukkan Blynk Auth Token'></p>";
                html += "<p style='margin:10px 0;'><label style='cursor:pointer;'><input type='checkbox' id='blynk_en' " + String(g_blynkEnabled ? "checked" : "") + "> Aktifkan Koneksi Blynk Cloud</label></p>";
                html += "<button onclick=\"saveBlynk()\">Simpan Pengaturan Blynk</button></div>";

                html += "<div class='card'><h3>Pengaturan Wi-Fi Router</h3>";
                        html += "<p>Hubungkan ESP32 ke Wi-Fi agar online ke Blynk Cloud:</p>";
                        html += "<p>Nama Wi-Fi (SSID):<br><input type='text' id='wifi_ssid' placeholder='Masukkan nama Wi-Fi'></p>";
                        html += "<p>Password Wi-Fi:<br><input type='password' id='wifi_pass' placeholder='Kosongkan jika tanpa password'></p>";
                        html += "<button onclick=\"saveWifi()\">Simpan & Sambungkan</button></div>";

                        html += "<div class='card'><h3>Tindakan Darurat</h3>";
                        html += "<button class='btn-danger' onclick=\"if(confirm('Reset semua memori NVS?'))fetch('/api/reset-nvs',{method:'POST'}).then(()=>alert('NVS Cleared!'))\">Hapus Memori NVS</button> ";
                        html += "<button class='btn-danger' onclick=\"fetch('/api/failsafe',{method:'POST'}).then(()=>alert('Failsafe Mode Aktif!'))\">Paksa Failsafe All-ON</button>";
                        html += "</div><script>"
                                "function saveWifi(){"
                                "var s=document.getElementById('wifi_ssid').value.trim();"
                                "var p=document.getElementById('wifi_pass').value;"
                                "if(!s){alert('Nama Wi-Fi (SSID) tidak boleh kosong!');return;}"
                                "var formData=new URLSearchParams();"
                                "formData.append('ssid',s);"
                                "formData.append('pass',p);"
                                "fetch('/api/wifi-save',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:formData})"
                                ".then(r=>r.text()).then(t=>{alert(t);location.reload();})"
                                ".catch(e=>{alert('Error: '+e);});"
                                "}"
                                "function saveBlynk(){"
                                "var tm=document.getElementById('blynk_tmpl').value.trim();"
                                "var t=document.getElementById('blynk_tok').value.trim();"
                                "var e=document.getElementById('blynk_en').checked?'1':'0';"
                                "var formData=new URLSearchParams();"
                                "formData.append('tmpl',tm);"
                                "formData.append('token',t);"
                                "formData.append('en',e);"
                                "fetch('/api/blynk-save',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:formData})"
                                ".then(r=>r.text()).then(txt=>{alert(txt);location.reload();})"
                                ".catch(err=>{alert('Error: '+err);});"
                                "}"
                                "</script></body></html>";
                request->send(200, "text/html; charset=utf-8", html);
                });

                // HALAMAN 4: UPDATE OTA FIRMWARE
                server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        String html = FPSTR(HTML_HEAD);
        html += "<h2>🚀 Update Firmware Over-The-Air</h2>";
        html += FPSTR(NAV_BAR);
        html += "<div class='card'><h3>Flash Firmware Gateway (ESP32-S3)</h3>";
        html += "<input type='file' id='binFile' accept='.bin' style='margin-bottom:10px;'><br>";
        html += "<button id='btnUpload' onclick='uploadFirmware()'>Upload & Flash Gateway .bin</button>";
        html += "<div id='prgBox' style='display:none;margin-top:15px;'>";
        html += "<div style='background:#0f172a;border-radius:8px;overflow:hidden;height:24px;border:1px solid #334155;'>";
        html += "<div id='prgBar' style='background:#3b82f6;height:100%;width:0%;transition:width 0.2s;'></div></div>";
        html += "<div id='prgTxt' style='margin-top:8px;font-size:14px;color:#94a3b8;'>Mengunggah: 0%</div></div>";
        html += "<div id='statusMsg' style='margin-top:15px;font-weight:bold;'></div></div>";

        html += "<div class='card'><h3>Simpan Firmware Klien ke LittleFS (Centralized OTA)</h3>";
        html += "<form method='POST' action='/upload-client' enctype='multipart/form-data'>";
        html += "<input type='file' name='client_bin' style='margin-bottom:10px;'><br>";
        html += "<button type='submit'>Upload Binary Klien Sub-PDU</button></form></div>";

        html += "<script>";
        html += "function uploadFirmware(){";
        html += "var f=document.getElementById('binFile');";
        html += "if(!f.files.length){alert('Pilih file firmware .bin terlebih dahulu!');return;}";
        html += "var btn=document.getElementById('btnUpload');";
        html += "var prgBox=document.getElementById('prgBox');";
        html += "var prgBar=document.getElementById('prgBar');";
        html += "var prgTxt=document.getElementById('prgTxt');";
        html += "var msg=document.getElementById('statusMsg');";
        html += "btn.disabled=true;btn.style.opacity='0.5';prgBox.style.display='block';msg.innerHTML='';";
        html += "var fd=new FormData();fd.append('update',f.files[0]);";
        html += "var xhr=new XMLHttpRequest();xhr.open('POST','/update',true);";
        html += "xhr.upload.onprogress=function(e){if(e.lengthComputable){var p=Math.round((e.loaded/e.total)*100);prgBar.style.width=p+'%';prgTxt.innerHTML='Mengunggah & Menulis Flash: '+p+'%';}};";
        html += "xhr.onload=function(){";
        html += "  if(xhr.status===200){";
        html += "    prgBar.style.background='#22c55e';prgBar.style.width='100%';prgTxt.innerHTML='100% Selesai!';";
        html += "    msg.innerHTML='<span style=\"color:#4ade80\">✅ Firmware berhasil ditulis! Menunggu ESP32 online kembali...</span>';";
        html += "    var attempts = 0;";
        html += "    setTimeout(function poll(){";
        html += "      attempts++;";
        html += "      msg.innerHTML='<span style=\"color:#38bdf8\">⏳ Menghubungkan ke Gateway... ('+attempts+'/15)</span>';";
        html += "      fetch('/api/status', {cache:'no-store'}).then(function(r){";
        html += "        if(r.ok){";
        html += "          msg.innerHTML='<span style=\"color:#4ade80\">✅ Gateway Online! Membuka Dashboard...</span>';";
        html += "          setTimeout(function(){ window.location.href='/'; }, 800);";
        html += "        } else { setTimeout(poll, 1500); }";
        html += "      }).catch(function(){";
        html += "        if(attempts >= 15){ window.location.href='/'; } else { setTimeout(poll, 1500); }";
        html += "      });";
        html += "    }, 3000);";
        html += "  } else {";
        html += "    btn.disabled=false;btn.style.opacity='1';prgBar.style.background='#ef4444';msg.innerHTML='<span style=\"color:#ef4444\">❌ Gagal update! Cek Serial Monitor.</span>';";
        html += "  }";
        html += "};";
        html += "xhr.onerror=function(){";
        html += "  msg.innerHTML='<span style=\"color:#38bdf8\">ESP32 sedang reboot. Mengalihkan ke Dashboard...</span>';";
        html += "  setTimeout(function(){ window.location.href='/'; }, 5000);";
        html += "};";
        html += "xhr.send(fd);}";
        html += "</script></body></html>";
        request->send(200, "text/html; charset=utf-8", html);
    });

            // REST API ENDPOINTS
                server.on("/api/wifi-save", HTTP_POST, [](AsyncWebServerRequest *request) {
                    if (!checkAuth(request)) return;
                    String newSsid = "";
                    String newPass = "";

                    // Cek parameter dari POST body maupun Query String (URL)
                    if (request->hasParam("ssid", true)) {
                        newSsid = request->getParam("ssid", true)->value();
                    } else if (request->hasParam("ssid", false)) {
                        newSsid = request->getParam("ssid", false)->value();
                    }

                    if (request->hasParam("pass", true)) {
                        newPass = request->getParam("pass", true)->value();
                    } else if (request->hasParam("pass", false)) {
                        newPass = request->getParam("pass", false)->value();
                    }

                    newSsid.trim(); // Bersihkan whitespace di awal/akhir jika tidak sengaja tertekan

                    if (newSsid.length() > 0) {
                        Preferences p;
                        p.begin("pdu_wifi", false);
                        p.putString("ssid", newSsid);
                        p.putString("pass", newPass); // Password boleh kosong untuk Open WiFi
                        p.end();

                        Serial.printf("[WIFI CONFIG] SSID Baru Disimpan: \"%s\" (Panjang: %d, Pass: %d karakter)\n", 
                                      newSsid.c_str(), newSsid.length(), newPass.length());

                        // Update global state & trigger reconnect di background Core 0 (Zero Downtime / Tanpa Reboot)
                        g_staSsid = newSsid;
                        g_staPass = newPass;
                        g_wifiNeedReconnect = true;

                        request->send(200, "text/plain; charset=utf-8", "OK: WiFi \"" + newSsid + "\" Tersimpan! Menghubungkan di background tanpa restart.");
                    } else {
                        request->send(400, "text/plain; charset=utf-8", "Gagal: SSID tidak boleh kosong!");
                    }
                });

                server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
                    if (!checkAuth(request)) return;

                    // Ambil snapshot atomic agar bebas data race dengan Core 1
                    TelemetryData snap;
                    bool rSnap[4];
                    if (xSemaphoreTake(relayMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                        snap = currentTelemetry;
                        for (int i = 0; i < 4; i++) rSnap[i] = targetRelayStates[i];
                        xSemaphoreGive(relayMutex);
                    } else {
                        snap = currentTelemetry;
                        for (int i = 0; i < 4; i++) rSnap[i] = (digitalRead(RELAY_PINS[i]) == LOW);
                    }

                    String json = "{";
                    json += "\"uptime\":" + String(millis() / 1000) + ",";
                    json += "\"bootCount\":" + String(g_bootCount) + ",";
                    json += "\"resetReason\":" + String((int)g_mcuResetReason) + ",";
                    json += "\"resetReasonStr\":\"" + String(getResetReasonStr((uint8_t)g_mcuResetReason)) + "\",";
                    json += "\"lastRst\":" + String(g_lastRstReason) + ",";
                    json += "\"lastRstStr\":\"" + String(getResetReasonStr((uint8_t)g_lastRstReason)) + "\",";
                    json += "\"cNet\":" + String(g_lastCrumbNet) + ",";
                    json += "\"cNetStr\":\"" + String(getCrumbNetStr(g_lastCrumbNet)) + "\",";
                    json += "\"cCtl\":" + String(g_lastCrumbCtl) + ",";
                    json += "\"cCtlStr\":\"" + String(getCrumbCtlStr(g_lastCrumbCtl)) + "\",";
                    json += "\"upS\":" + String(g_lastUptimeS) + ",";
                    json += "\"btns\":[" + String(digitalRead(BTN_PINS[0])) + "," + String(digitalRead(BTN_PINS[1])) + "," +
                            String(digitalRead(BTN_PINS[2])) + "," + String(digitalRead(BTN_PINS[3])) + "," +
                            String(digitalRead(BTN_MASTER)) + "," + String(digitalRead(BTN_RESET_KWH)) + "],";
                    json += "\"voltage\":" + String(snap.voltage, 2) + ",";
                    json += "\"current\":" + String(snap.current, 2) + ",";
                    json += "\"power\":" + String(snap.power, 1) + ",";
                    json += "\"energy\":" + String(snap.energy, 3) + ",";
                    json += "\"frequency\":" + String(snap.frequency, 1) + ",";
                    json += "\"pf\":" + String(snap.pf, 2) + ",";
                    g_cpuActivityPulse = true; // Picu flicker disk-style saat request masuk
                    json += "\"chipTemp\":" + String(snap.chipTemp, 1) + ",";
                    json += "\"cpuSpeed\":" + String(snap.cpuFreq ? snap.cpuFreq : ESP.getCpuFreqMHz()) + ",";
                    json += "\"cpuLoad0\":" + String(snap.cpuLoad0, 1) + ",";
                    json += "\"cpuLoad1\":" + String(snap.cpuLoad1, 1) + ",";
                    json += "\"cpuLoad\":" + String(snap.cpuLoad, 1) + ",";
                    json += "\"sysState\":" + String(snap.sysState) + ",";
                    json += "\"seqState\":" + String(snap.seqState) + ",";
                    json += "\"clients\":" + String(registeredClientCount) + ",";
                    json += "\"wifiConnected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
                    json += "\"blynkConnected\":" + String((g_blynkEnabled && Blynk.connected()) ? "true" : "false") + ",";
                    json += "\"blynkEnabled\":" + String(g_blynkEnabled ? "true" : "false") + ",";
                    json += "\"tripCount\":" + String(g_tripCount) + ",";
                    json += "\"hasTrip\":" + String(g_hasTripRecord ? "true" : "false") + ",";
                    if (g_hasTripRecord) {
                        json += "\"tripLabel\":\"" + String(g_lastTripRecord.tripLabel) + "\",";
                        json += "\"tripVolt\":" + String(g_lastTripRecord.voltage, 1) + ",";
                        json += "\"tripMinVolt5s\":" + String(g_lastTripRecord.minVolt5s, 1) + ",";
                        json += "\"tripMaxVolt5s\":" + String(g_lastTripRecord.maxVolt5s, 1) + ",";
                        json += "\"tripCurr\":" + String(g_lastTripRecord.current, 2) + ",";
                        json += "\"tripTemp\":" + String(g_lastTripRecord.chipTemp, 1) + ",";
                        json += "\"tripTime\":" + String(g_lastTripRecord.timestamp) + ",";
                        json += "\"tripBoot\":" + String(g_lastTripRecord.bootCount) + ",";
                        json += "\"tripRstReason\":" + String(g_lastTripRecord.rstReason) + ",";
                    }
                    json += "\"ssid\":\"" + WiFi.SSID() + "\",";
                    json += "\"ip\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString()) + "\",";
                    json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
                    json += "\"btnHoldMs\":" + String(g_btnHoldMs) + ",";
                    json += "\"relays\":[" + String(rSnap[0]) + "," + String(rSnap[1]) + "," +
                            String(rSnap[2]) + "," + String(rSnap[3]) + "]";
                    json += "}";
                    request->send(200, "application/json", json);
                });

    server.on("/api/client-relay", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        int ch = 1;
        if (request->hasParam("ch")) ch = request->getParam("ch")->value().toInt();
        static bool clientRelayStates[4] = {false, false, false, false};
        
        if (ch >= 1 && ch <= 4) {
            clientRelayStates[ch - 1] = !clientRelayStates[ch - 1];
            for (int i = 0; i < registeredClientCount; i++) {
                if (clientList[i].type == NODE_SUB_PDU_LT2 && clientList[i].isOnline) {
                    EspNowPayload cmd = {};
                    cmd.msgType = MSG_COMMAND;
                    cmd.targetChannel = (uint8_t)ch;
                    cmd.payloadValue = clientRelayStates[ch - 1] ? 1 : 0;
                    WiFi.macAddress(cmd.macSender);
                    sendEspNow(clientList[i].mac, cmd);
                    Serial.printf("[GATEWAY -> KLIEN LT2] Perintah Relay %d = %d Terkirim!\n", ch, cmd.payloadValue);
                    request->send(200, "text/plain", "Perintah Relay " + String(ch) + " (Lt 2) -> " + String(cmd.payloadValue ? "ON" : "OFF") + " Terkirim!");
                    return;
                }
            }
            request->send(200, "text/plain", "Klien Lantai 2 sedang offline atau belum terdaftar.");
        } else {
            request->send(400, "text/plain", "Invalid channel (1..4)");
        }
    });

    server.on("/api/relay", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        if (request->hasParam("ch")) {
            int ch = request->getParam("ch")->value().toInt();
            if (ch >= 0 && ch < 4) {
                bool nextState = !targetRelayStates[ch];
                if (request->hasParam("state")) {
                    String s = request->getParam("state")->value();
                    if (s == "1" || s == "true") nextState = true;
                    else if (s == "0" || s == "false") nextState = false;
                }
                PduCommand cmd = {CMD_SET_RELAY, (uint8_t)ch, nextState};
                xQueueSend(pduCommandQueue, &cmd, 0);
            }
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/api/force-idle", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        PduCommand cmd = {CMD_SET_RELAY, 254, false}; // Special code force idle
        xQueueSend(pduCommandQueue, &cmd, 0);
        request->send(200, "text/plain", "OK");
    });

    server.on("/api/sequencer", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        PduCommand cmd = {CMD_TRIGGER_SEQUENCER, 255, false};
        xQueueSend(pduCommandQueue, &cmd, 0);
        request->send(200, "text/plain", "OK");
    });

    server.on("/api/pairing", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        isPairingWindowActive = true;
        pairingStartTime = millis();
        request->send(200, "text/plain", "PAIRING_STARTED");
    });

    server.on("/api/thresholds", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        auto getP = [](AsyncWebServerRequest *req, const String& key) -> String {
            if (req->hasParam(key, true)) return req->getParam(key, true)->value();
            if (req->hasParam(key, false)) return req->getParam(key, false)->value();
            return "";
        };

        prefs.begin("pdu_store", false);
        String vUvlo = getP(request, "uvlo");
        if (vUvlo.length() > 0) {
            g_threshUvlo = vUvlo.toFloat();
            prefs.putFloat("uvlo", g_threshUvlo);
        }
        String vSurge = getP(request, "surge");
        if (vSurge.length() > 0) {
            g_threshSurge = vSurge.toFloat();
            prefs.putFloat("surge", g_threshSurge);
        }
        String vOcp = getP(request, "ocp");
        if (vOcp.length() > 0) {
            g_threshOcp = vOcp.toFloat();
            prefs.putFloat("ocp", g_threshOcp);
        }
        String vSeq = getP(request, "seq_delay");
        if (vSeq.length() > 0) {
            g_seqDelayMs = vSeq.toInt();
            prefs.putUInt("seq_delay", g_seqDelayMs);
        }
        String vSs = getP(request, "ss_delay");
        if (vSs.length() > 0) {
            g_softStartDelayMs = vSs.toInt();
            if (g_softStartDelayMs < 100) g_softStartDelayMs = 100;
            if (g_softStartDelayMs > 10000) g_softStartDelayMs = 10000;
            prefs.putUInt("ss_delay", g_softStartDelayMs);
        }
        String vHold = getP(request, "btn_hold");
        if (vHold.length() > 0) {
            g_btnHoldMs = vHold.toInt();
            if (g_btnHoldMs < 500) g_btnHoldMs = 500;
            if (g_btnHoldMs > 10000) g_btnHoldMs = 10000;
            prefs.putUInt("btn_hold", g_btnHoldMs);
        }
        prefs.end();

        PduCommand cmd = {CMD_RELOAD_THRESHOLDS, 0, false};
        xQueueSend(pduCommandQueue, &cmd, 0);
        request->redirect("/recovery");
    });

    server.on("/api/blynk-save", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        String newTok = "";
        String newTmpl = "";
        bool newEn = false;
        if (request->hasParam("token", true)) newTok = request->getParam("token", true)->value();
        else if (request->hasParam("token", false)) newTok = request->getParam("token", false)->value();
        newTok.trim();

        if (request->hasParam("tmpl", true)) newTmpl = request->getParam("tmpl", true)->value();
        else if (request->hasParam("tmpl", false)) newTmpl = request->getParam("tmpl", false)->value();
        newTmpl.trim();

        String enStr = "";
        if (request->hasParam("en", true)) enStr = request->getParam("en", true)->value();
        else if (request->hasParam("en", false)) enStr = request->getParam("en", false)->value();
        newEn = (enStr == "1" || enStr == "true" || enStr == "on");

        Preferences p;
        p.begin("pdu_store", false);
        p.putString("blynk_tok", newTok);
        p.putString("blynk_tmpl", newTmpl);
        p.putBool("blynk_en", newEn);
        p.end();

        g_blynkAuthToken = newTok;
        g_blynkTemplateId = newTmpl;
        g_blynkEnabled = newEn;
        g_blynkNeedReconnect = true;

        request->send(200, "text/plain; charset=utf-8", "OK: Pengaturan Blynk tersimpan! Diperbarui live tanpa restart.");
    });

    server.on("/api/failsafe", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        PduCommand cmd = {CMD_FAILSAFE_TRIP, 0, false};
        xQueueSend(pduCommandQueue, &cmd, 0);
        request->send(200, "text/plain", "FAILSAFE_ACTIVE");
    });

    server.on("/api/reset-nvs", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        prefs.begin("pdu_store", false);
        prefs.clear();
        prefs.end();
        prefs.begin("gateway_nodes", false);
        prefs.clear();
        prefs.end();
        request->send(200, "text/plain", "NVS_CLEARED");
    });

    // Endpoint Reset Catatan Insiden Blackbox (Membersihkan 10 Kejadian di NVS)
    server.on("/api/reset-blackbox", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        Preferences pBb;
        pBb.begin("pdu_bb", false);
        pBb.clear();
        pBb.end();
        g_hasTripRecord = false;
        g_tripCount = 0;
        g_tripHead = 0;
        memset(&g_lastTripRecord, 0, sizeof(g_lastTripRecord));
        memset(g_tripRecords, 0, sizeof(g_tripRecords));
        request->send(200, "text/plain", "BLACKBOX_CLEARED");
    });

    // Endpoint Daftar Lengkap Riwayat Insiden (Ring Buffer 10 Kejadian)
    server.on("/api/trips", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        String json = "{";
        json += "\"bootCount\":" + String(g_bootCount) + ",";
        json += "\"resetReason\":" + String((int)g_mcuResetReason) + ",";
        json += "\"resetReasonStr\":\"" + String(getResetReasonStr((uint8_t)g_mcuResetReason)) + "\",";
        json += "\"lastRst\":" + String(g_lastRstReason) + ",";
        json += "\"lastRstStr\":\"" + String(getResetReasonStr((uint8_t)g_lastRstReason)) + "\",";
        json += "\"cNet\":" + String(g_lastCrumbNet) + ",";
        json += "\"cNetStr\":\"" + String(getCrumbNetStr(g_lastCrumbNet)) + "\",";
        json += "\"cCtl\":" + String(g_lastCrumbCtl) + ",";
        json += "\"cCtlStr\":\"" + String(getCrumbCtlStr(g_lastCrumbCtl)) + "\",";
        json += "\"upS\":" + String(g_lastUptimeS) + ",";
        json += "\"tripCount\":" + String(g_tripCount) + ",";
        json += "\"trips\":[";

        bool first = true;
        for (int i = 0; i < g_tripCount; i++) {
            int idx = (g_tripHead - 1 - i + BLACKBOX_MAX_RECORDS) % BLACKBOX_MAX_RECORDS;
            if (idx >= g_tripCount && g_tripCount < BLACKBOX_MAX_RECORDS) continue;
            const BlackboxRecord &r = g_tripRecords[idx];
            if (r.magic != 0x50445542) continue;

            if (!first) json += ",";
            first = false;

            json += "{";
            json += "\"no\":" + String(i + 1) + ",";
            json += "\"slot\":" + String(idx) + ",";
            json += "\"bootCount\":" + String(r.bootCount) + ",";
            json += "\"timestamp\":" + String(r.timestamp) + ",";
            json += "\"tripReason\":" + String(r.tripReason) + ",";
            json += "\"tripLabel\":\"" + String(r.tripLabel) + "\",";
            json += "\"rstReason\":" + String(r.rstReason) + ",";
            json += "\"rstReasonStr\":\"" + String(getResetReasonStr(r.rstReason)) + "\",";
            json += "\"voltage\":" + String(r.voltage, 1) + ",";
            json += "\"minVolt5s\":" + String(r.minVolt5s, 1) + ",";
            json += "\"maxVolt5s\":" + String(r.maxVolt5s, 1) + ",";
            json += "\"current\":" + String(r.current, 2) + ",";
            json += "\"power\":" + String(r.power, 1) + ",";
            json += "\"chipTemp\":" + String(r.chipTemp, 1) + ",";
            json += "\"relayBits\":" + String(r.relayStates);
            json += "}";
        }
        json += "]}";
        request->send(200, "application/json", json);
    });

    // Endpoint Riwayat Telemetri Resolusi 1-Detik dari Octal PSRAM
    server.on("/api/history", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        if (psramHistoryBuffer == nullptr || psramHistoryCount == 0) {
            request->send(200, "application/json", "{\"count\":0,\"data\":[]}");
            return;
        }
        int limit = 300; // Default 300 detik (5 menit detail)
        if (request->hasParam("limit")) limit = request->getParam("limit")->value().toInt();
        if (limit > 1200) limit = 1200;
        int pointsToSend = (psramHistoryCount < limit) ? psramHistoryCount : limit;

        String json = "{\"count\":" + String(pointsToSend) + ",\"data\":[";
        portENTER_CRITICAL(&historyMux);
        int startIdx = (psramHistoryHead - pointsToSend + PSRAM_HISTORY_MAX_POINTS) % PSRAM_HISTORY_MAX_POINTS;
        for (int i = 0; i < pointsToSend; i++) {
            int idx = (startIdx + i) % PSRAM_HISTORY_MAX_POINTS;
            if (i > 0) json += ",";
            json += "{\"t\":" + String(psramHistoryBuffer[idx].timestamp) +
                    ",\"v\":" + String(psramHistoryBuffer[idx].voltage, 1) +
                    ",\"a\":" + String(psramHistoryBuffer[idx].current, 2) +
                    ",\"w\":" + String(psramHistoryBuffer[idx].power, 0) +
                    ",\"c\":" + String(psramHistoryBuffer[idx].chipTemp, 1) +
                    ",\"s\":" + String(psramHistoryBuffer[idx].sysState) + "}";
        }
        portEXIT_CRITICAL(&historyMux);
        json += "]}";
        request->send(200, "application/json", json);
    });

    // Endpoint Penyedia Firmware untuk Klien (Centralized OTA Injeksi)
    server.on("/firmware/client_subnode.bin", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/firmware/client_subnode.bin")) {
            request->send(LittleFS, "/firmware/client_subnode.bin", "application/octet-stream");
        } else {
            request->send(404, "text/plain", "Firmware client belum diunggah ke LittleFS");
        }
    });

    // OTA Flashing Handler
    server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        bool shouldReboot = !Update.hasError();
        if (shouldReboot) {
            String respHtml = "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='6;url=/'><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
                              "<title>Update Berhasil</title>"
                              "<style>body{background:#0f172a;color:#f8fafc;font-family:system-ui;text-align:center;padding:50px;}"
                              ".box{background:#1e293b;border:1px solid #334155;border-radius:12px;padding:30px;max-width:450px;margin:0 auto;}"
                              "h2{color:#4ade80;margin-top:0;}</style></head><body>"
                              "<div class='box'><h2>✅ Update Berhasil!</h2><p>ESP32 sedang reboot dengan firmware baru.</p>"
                              "<p style='color:#94a3b8;'>Mengalihkan ke Dashboard dalam 6 detik...</p>"
                              "<a href='/' style='color:#38bdf8;'>Klik di sini jika tidak otomatis dialihkan</a></div></body></html>";
            AsyncWebServerResponse *response = request->beginResponse(200, "text/html; charset=utf-8", respHtml);
            response->addHeader("Connection", "close");
            request->send(response);
        } else {
            AsyncWebServerResponse *response = request->beginResponse(400, "text/plain", "ERR: Update Gagal! (Error Code: " + String(Update.getError()) + ")");
            response->addHeader("Connection", "close");
            request->send(response);
        }
        if (shouldReboot) {
            xTaskCreate([](void*) {
                // Kunci level hardware pin relay agar listrik TIDAK MATI sama sekali saat reboot!
                for (int i = 0; i < 4; i++) {
                    gpio_hold_en((gpio_num_t)RELAY_PINS[i]);
                    gpio_hold_en((gpio_num_t)RELAY_SS_PINS[i]);
                }
                gpio_deep_sleep_hold_en();
                Serial.println("[OTA] ✅ GPIO Relay di-HOLD! Listrik tetap menyala selama reboot MCU.");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                ESP.restart();
            }, "rebootTask", 2048, NULL, 5, NULL);
        }
    }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
        if (!request->authenticate(WEB_ADMIN_USER, WEB_ADMIN_PASS)) return;
        if (!index) {
            Serial.printf("[OTA] Memulai Update: %s\n", filename.c_str());
            // Gunakan partisi FLASH default (U_FLASH) agar eksplisit menulis ke slot app OTA berikutnya
            if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
                Update.printError(Serial);
            }
        }
        if (!Update.hasError()) {
            if (Update.write(data, len) != len) {
                Update.printError(Serial);
            }
        }
        if (final) {
            if (Update.end(true)) {
                Serial.printf("[OTA] Update Selesai: %u bytes\n", index + len);
            } else {
                Update.printError(Serial);
            }
        }
    });

    // Endpoint Upload Binary Klien ke LittleFS (Centralized OTA)
    server.on("/upload-client", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!checkAuth(request)) return;
        request->send(200, "text/plain", "OK: Firmware client berhasil disimpan ke LittleFS!");
    }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
        if (!request->authenticate(WEB_ADMIN_USER, WEB_ADMIN_PASS)) return;
        static File clientBinFile;
        if (!index) {
            LittleFS.mkdir("/firmware");
            clientBinFile = LittleFS.open("/firmware/client_subnode.bin", FILE_WRITE);
        }
        if (clientBinFile && len > 0) {
            clientBinFile.write(data, len);
        }
        if (final && clientBinFile) {
            clientBinFile.close();
            Serial.println("[LITTLEFS] client_subnode.bin berhasil diunggah!");
        }
    });

    server.begin();
    Serial.println("[HTTP] Web Server 4 Halaman & REST API Aktif!");

    // 7. Spawn Core 1 Task (Proteksi Real-Time, Prioritas Tinggi) FIRST
    xTaskCreatePinnedToCore(TaskControlCore1, "TaskControlCore1", 8192, NULL, 3, NULL, 1);

    // 8. Spawn Core 0 Task (Network, Web Server, Blynk, & mDNS)
    xTaskCreatePinnedToCore(TaskNetworkCore0, "TaskNetworkCore0", 8192, NULL, 1, NULL, 0);
}

void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
