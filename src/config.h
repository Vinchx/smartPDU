#pragma once
#include <Arduino.h>

// ========================================================
// 1. PINOUT CONFIGURATION KHUSUS ESP32-S3 (GATEWAY BAWAH)
// ========================================================
#define DEBUG_PINS 0  // Set 0 jika sudah rilis/production (Mode Bersih & Cepat)

#if DEBUG_PINS
  #define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
  #define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DBG_PRINTF(...)
  #define DBG_PRINTLN(...)
#endif

// 4-Channel Relays (Active-LOW: 0V = ON, 3.3V = OFF)
constexpr uint8_t RELAY_PINS[4]    = {4, 5, 6, 7};
// 4-Channel Soft Start Relays (Active-LOW: 0V = ON, 3.3V = OFF)
constexpr uint8_t RELAY_SS_PINS[4] = {8, 14, 15, 16};

// 6x Tactile Push Buttons (INPUT_PULLUP, Pin to GND)
constexpr uint8_t BTN_PINS[4]   = {10, 11, 12, 13};
constexpr uint8_t BTN_MASTER    = 9;  // Pindah dari GPIO 14 (JTAG) ke GPIO 9 (Murni General GPIO)
constexpr uint8_t BTN_RESET_KWH = 21;

// Audio Buzzer (LEDC PWM / Digital Alert)
constexpr uint8_t PIN_BUZZER    = 2;
constexpr uint8_t PIN_RGB_LED   = 48; // Onboard WS2812 RGB LED (ESP32-S3 DevKitC-1)

// PZEM-004T v4.0 UART Pins (Hardware Serial 1 di ESP32-S3)
// Pin ditukar: RX=17, TX=18
constexpr uint8_t PIN_PZEM_RX   = 17; 
constexpr uint8_t PIN_PZEM_TX   = 18; 

// ========================================================
// MACRO DEBUG PER-PIN & PER-FUNGSI (SESUAI INSTRUKSI CAK MAKSUM)
// ========================================================
#define DBG_FN_ENTER(fn) DBG_PRINTF("[FN ENTER] >>> %s()\n", fn)
#define DBG_FN_EXIT(fn)  DBG_PRINTF("[FN EXIT]  <<< %s()\n", fn)

#define DBG_PIN_WRITE(pin, val, name) do { \
    digitalWrite(pin, val); \
    DBG_PRINTF("[PIN WRITE] %-14s (GPIO %2d) -> %s (Physical: %s)\n", name, pin, (val == LOW) ? "ON (Active)" : "OFF (Release)", (val == LOW) ? "LOW / 0V" : "HIGH / 3.3V"); \
} while(0)

#define DBG_PIN_READ(pin, name) ({ \
    int _v = digitalRead(pin); \
    DBG_PRINTF("[PIN READ]  %-14s (GPIO %2d) -> %s\n", name, pin, (_v == LOW) ? "PRESSED (0V/GND)" : "IDLE (HIGH/3.3V)"); \
    _v; \
})

#define DBG_PIN_PWM(pin, freq, duty, name) do { \
    ledcWriteTone(0, freq); \
    DBG_PRINTF("[PIN PWM]   %-14s (GPIO %2d) -> Freq: %d Hz | Duty: %d\n", name, pin, freq, duty); \
} while(0)

// ========================================================
// 2. BLYNK CLOUD CREDENTIALS & V-PINS (GATEWAY)
// ========================================================
#define BLYNK_TEMPLATE_ID   "TMPL6T9iOcOCc"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN    "NFe2LJ7yy81VhAiCqW27dPsJUGDhtPKw"

// Virtual Pins (Lantai Bawah) - Dipindah ke V5..V8 agar tidak bentrok dengan template Quickstart default (V1=Uptime, V2=RSSI)
constexpr uint8_t VPIN_RELAY[4]      = {5, 6, 7, 8}; // Urut V5, V6, V7, V8
constexpr uint8_t VPIN_MASTER        = 10;
constexpr uint8_t VPIN_VOLT          = 11;
constexpr uint8_t VPIN_AMP           = 12;
constexpr uint8_t VPIN_WATT          = 13;
constexpr uint8_t VPIN_FREQ          = 14;
constexpr uint8_t VPIN_PF            = 15;
constexpr uint8_t VPIN_ENERGY        = 16;
constexpr uint8_t VPIN_RESET_KWH     = 20;
constexpr uint8_t VPIN_LED[4]        = {21, 22, 23, 24};
constexpr uint8_t VPIN_ALARM_LED     = 25;

// Virtual Pins Khusus Manajemen Klien (Lt Atas & ESP-01)
constexpr uint8_t VPIN_PAIRING_MODE  = 30; // Push Button: Buka pairing mode 60s
constexpr uint8_t VPIN_CLIENT_COUNT  = 31; // Display: Jumlah klien terdaftar
constexpr uint8_t VPIN_CLIENT_UP_RL1 = 41; // Relay 1 Klien Lantai Atas
constexpr uint8_t VPIN_CLIENT_UP_RL2 = 42; // Relay 2 Klien Lantai Atas

// ========================================================
// 3. THRESHOLD DEFAULT PROTEKSI & WAKTU (DISIMPAN DI FLASH)
// ========================================================
constexpr float DEFAULT_UVLO_VOLT       = 175.0f; // Volt AC
constexpr float DEFAULT_SURGE_VOLT      = 260.0f; // Volt AC (Toleransi PLN malam hari)
constexpr float DEFAULT_OCP_CURRENT     = 16.0f;  // Ampere (Sesuai MCB Rumah)
constexpr float DEFAULT_FREQ_MIN        = 47.0f;  // Hz
constexpr float DEFAULT_FREQ_MAX        = 53.0f;  // Hz

constexpr uint32_t BTN_HOLD_TOGGLE_MS   = 2500; // Tahan 2s toggle individual
constexpr uint32_t BTN_HOLD_RESET_MS    = 5000; // Tahan 5s reset energy
constexpr uint32_t SEQUENCER_DELAY_MS   = 2500; // Jeda 2s per channel sequencer
constexpr uint32_t DEFAULT_SOFTSTART_DELAY_MS = 1000; // Jeda 1000ms pre-charge sebelum relay utama ON
constexpr uint32_t POWERON_STABILIZE_MS = 5000; // Pantau kestabilan tegangan 5s pasca jeglek

// ========================================================
// 4. JARINGAN, MDNS, & AUTHENTICATION
// ========================================================
#define MDNS_HOSTNAME       "electric-pdu"
#define WEB_ADMIN_USER      "admin"
#define WEB_ADMIN_PASS      "admin123"
#define DEFAULT_ESPNOW_CH   6

// ========================================================
// 5. DATA STRUCTURES & FREERTOS INTER-TASK IPC
// ========================================================
enum SystemState : uint8_t {
    STATE_NORMAL = 0,
    STATE_UVLO,
    STATE_SURGE,
    STATE_OCP,
    STATE_FREQ_FAULT,
    STATE_FAILSAFE
};

enum SequencerState : uint8_t {
    SEQ_IDLE = 0,
    SEQ_POWERON_STABILIZING,
    SEQ_STARTING_UP,
    SEQ_SHUTTING_DOWN
};

enum PduCommandType : uint8_t {
    CMD_NONE = 0,
    CMD_SET_RELAY,
    CMD_TRIGGER_SEQUENCER,
    CMD_RESET_ENERGY,
    CMD_FAILSAFE_TRIP,
    CMD_RELOAD_THRESHOLDS
};

struct PduCommand {
    PduCommandType type;
    uint8_t channel; // 0..3 (atau 255 utk all)
    bool state;      // true = ON, false = OFF
};

struct TelemetryData {
    float voltage;
    float current;
    float power;
    float frequency;
    float pf;
    float energy;
    float chipTemp; // Suhu internal ESP32-S3 (°C)
    float cpuLoad;  // Estimasi beban CPU rata-rata (%)
    float cpuLoad0; // Estimasi beban Core 0 (Network / Web / Blynk) (%)
    float cpuLoad1; // Estimasi beban Core 1 (Control / PZEM / Relays) (%)
    uint16_t cpuFreq; // Frekuensi CPU Clock (MHz)
    bool relayStates[4];
    SystemState sysState;
    SequencerState seqState;
    uint32_t relayCycleCount[4];
};

// ========================================================
// 5B. STRUKTUR RIWAYAT TELEMETRI PSRAM & BLACKBOX NVS
// ========================================================
#define PSRAM_HISTORY_MAX_POINTS 3600 // 1 Jam buffer resolusi 1-detik (86.4 KB di Octal PSRAM)
#define BLACKBOX_MAX_RECORDS     10   // Ring buffer 10 riwayat trip di Flash NVS

struct TelemetryHistoryPoint {
    uint32_t timestamp;  // Uptime detik
    float    voltage;    // Volt AC
    float    current;    // Ampere
    float    power;      // Watt
    float    chipTemp;   // Suhu ESP32-S3 (°C)
    uint8_t  sysState;   // SystemState
    uint8_t  relayBits;  // Bit 0..3: Relay 1..4
};

struct BlackboxRecord {
    uint32_t magic;         // 0x50445542 ("PDUB")
    uint32_t bootCount;     // Counter boot saat kejadian
    uint32_t timestamp;     // Uptime detik saat kejadian
    uint8_t  tripReason;    // SystemState (UVLO, SURGE, OCP, BLACKOUT)
    uint8_t  rstReason;     // esp_reset_reason_t
    float    voltage;       // Tegangan saat trip
    float    current;       // Arus saat trip
    float    power;         // Daya saat trip
    float    chipTemp;      // Suhu chip saat trip
    float    minVolt5s;     // Tegangan minimum dalam 5 detik sebelum trip
    float    maxVolt5s;     // Tegangan maksimum dalam 5 detik sebelum trip
    uint8_t  relayStates;   // Status relay sebelum diputus (bit 0..3)
    char     tripLabel[32]; // Nama teks penyebab
};

// ========================================================
// 6. PROTOKOL DATA ESP-NOW (MODULAR CLIENT SYSTEM)
// ========================================================
#define ESPNOW_MAX_CLIENTS 8

enum MessageType : uint8_t {
    MSG_DISCOVERY     = 0x01,  // Klien baru cari Gateway
    MSG_PAIR_ACK      = 0x02,  // Gateway konfirmasi klien
    MSG_TELEMETRY     = 0x03,  // Klien kirim data sensor ke Gateway
    MSG_COMMAND       = 0x04,  // Gateway kontrol relay klien
    MSG_PING          = 0x05,  // Heartbeat status online/offline
    MSG_PROVISION_REQ = 0x06,  // Klien minta OTA URL
    MSG_PROVISION_ACK = 0x07   // Gateway kirim info firmware .bin
};

enum NodeType : uint8_t {
    NODE_SUB_PDU_LT2 = 1,  // ESP32 Klasik PDU Lantai Atas
    NODE_SENSOR_ESP01= 2,  // ESP-01 Sensor Pintu/PIR
    NODE_RELAY_ESP01 = 3   // ESP-01 Saklar Terpisah
};

// Payload terstandarisasi (Maksimal 250 bytes untuk ESP-NOW)
struct __attribute__((packed)) EspNowPayload {
    uint8_t     msgType;         // MessageType
    uint8_t     nodeType;        // NodeType
    uint8_t     macSender[6];    // MAC Pengirim
    uint8_t     targetChannel;   // Nomor Channel Relay / ID
    uint8_t     payloadValue;    // Nilai (1/0 ON/OFF atau ADC)
    float       voltage;         // Telemetri PZEM (jika ada)
    float       current;         // Telemetri PZEM
    float       power;           // Telemetri PZEM
};

// Node Klien Terdaftar
struct ClientNodeInfo {
    uint8_t  mac[6];
    NodeType type;
    uint32_t lastSeen;
    bool     isOnline;
};
