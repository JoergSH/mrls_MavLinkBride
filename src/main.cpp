#include <Arduino.h>
//*******************************************************
// mLRS Wireless Bridge for ESP32
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Basic but effective & reliable transparent WiFi or Bluetooth <-> serial bridge.
// Minimizes wireless traffic while respecting latency by better packeting algorithm.
//*******************************************************
// Ported to PlatformIO for ESP32-C3 Super Mini
//*********************************************************/

// For ESP8266 this sadly needs to come before User Configuration section
#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#endif


//-------------------------------------------------------
// User configuration
//-------------------------------------------------------

// Module is defined in platformio.ini via build_flags:
// -D MODULE_ESP32C3_SUPER_MINI

// Serial level
// uncomment, if you need inverted serial for a supported module
//#define USE_SERIAL_INVERTED

// Wireless protocol (set via build_flags in platformio.ini)
// 0 = WiFi TCP, 1 = WiFi UDP (AP), 2 = Wifi UDPSTA (connects to AP), 3 = Bluetooth, 4 = Wifi UDPCl, 5 = BLE
// Air Unit uses 1 (UDP AP mode), Ground Unit uses 2 (UDPSTA - connects to Air Unit)
#ifndef WIRELESS_PROTOCOL
#define WIRELESS_PROTOCOL  1  // Default: UDP AP mode
#endif

// GPIO0 usage is defined in boards.h for ESP32C3_SUPER_MINI


//**********************//
//*** WiFi settings ***//

// For TCP, UDP (only for these two)
// ssid = "" results in a default name, like "mLRS-13427 AP UDP"
// password = "" makes it an open AP
String ssid = "MAVLink-Air"; // WiFi name for Air Unit AP
String password = "mavlink123"; // WiFi password (min 8 chars)

IPAddress ip(192, 168, 4, 1); // AP IP address

int port_tcp = 5760; // connect to this port per TCP // MissionPlanner default is 5760
int port_udp = 14550; // connect to this port per UDP // MissionPlanner default is 14550

// For UDP, UDPSTA (only for these two)
// comment out only if broadcast won't work for you (ip_udp declared below will then be used)
#define WIFI_USE_BROADCAST_FOR_UDP

// For UDPSTA, UDPCl (only for these two)
// Ground Unit connects to Air Unit's WiFi network
String network_ssid = "MAVLink-Air"; // Must match Air Unit's ssid!
String network_password = "mavlink123"; // Must match Air Unit's password!

IPAddress ip_udpcl(192, 168, 0, 164); // your network's IP (only for UDPCl) // MissionPlanner default is 127.0.0.1, so enter your home's IP in MP

int port_udpcl = 14550; // listens to this port per UDPCl (only for UDPCl) // MissionPlanner default is 14550

// WiFi channel (only for TCP, UDP)
// choose 1, 6, 11, 13. Channel 13 (2461-2483 MHz) has the least overlap with mLRS 2.4 GHz frequencies.
// Note: Channel 13 is generally not available in the US, where 11 is the maximum.
#define WIFI_CHANNEL  1  // Channel 1 often less congested

// WiFi power (for all TCP, UDP, UDPSTA, UDPCl)
// this sets the power level for the WiFi protocols
// Note: If GPIO0_IO is defined, this sets the power for the medium power option.
#ifndef ESP8266
// Note: In order to find the possible options, right click on WIFI_POWER_19_5dBm and choose "Go To Definiton"
#define WIFI_POWER  WIFI_POWER_19_5dBm // Max power for better range/speed
#else
// Note: WIFI_POWER can be 0 to 20.5
#define WIFI_POWER  6
#endif


//**************************//
//*** Bluetooth settings ***//

// bluetooth_device_name = "" results in a default name, like "mLRS-13427 BT"
String bluetooth_device_name = ""; // name of your Bluetooth device as it will be seen by your operating system


//**************************//
//*** BLE settings ***//

// ble_device_name = "" results in a default name, like "mLRS-13427 BLE"
String ble_device_name = "MAVLink-Bridge"; // name of your BLE device as it will be seen by your operating system


//************************//
//*** General settings ***//

// Baudrate
#define BAUD_RATE  115200


//-------------------------------------------------------
// Version
//-------------------------------------------------------

#define VERSION_STR  "v1.3.07" // to not get version salad, use what the current mLRS version is at the time


//-------------------------------------------------------
// Module details
//-------------------------------------------------------

#include "mlrs-wireless-bridge-boards.h"


//-------------------------------------------------------
// Includes
//-------------------------------------------------------

#ifdef GPIO0_IO
#define USE_AT_MODE
#endif

#ifndef ESP8266
#ifdef ARDUINO_ESP32C3_DEV
// PlatformIO uses different version checks, skip the strict version check
#else
#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    #warning Version of your ESP Arduino Core below 3.0.0 !
#endif
#endif // ARDUINO_ESP32C3_DEV

#include <WiFi.h>
#include "esp_mac.h"
// for some reason checking
// #if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
// does not work here. Also checking e.g. PLATFORM_ESP32_C3 seems not to work.
// This sucks. So we don't try to be nice but let the compiler work it out.
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 3) // for AT commands we require BT be available
#ifdef CONFIG_IDF_TARGET_ESP32 // classic BT only available on ESP32
  #define USE_WIRELESS_PROTOCOL_BLUETOOTH
  #include <BluetoothSerial.h>
#endif
#endif
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 5) // for AT commands we require BLE be available
#if defined CONFIG_IDF_TARGET_ESP32 || defined CONFIG_IDF_TARGET_ESP32C3 // BLE available on ESP32 and ESP32C3
  #define USE_WIRELESS_PROTOCOL_BLE
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
#endif
#endif
// ESP-NOW Long Range
#if WIRELESS_PROTOCOL == 6
  #define USE_WIRELESS_PROTOCOL_ESPNOW
  #define ESPNOW_USE_LR
  #include <esp_now.h>
  #include <esp_wifi.h>
#endif
// ESP-NOW Standard (no LR, for compatibility with BLE Ground Unit)
#if WIRELESS_PROTOCOL == 8
  #define USE_WIRELESS_PROTOCOL_ESPNOW
  // no ESPNOW_USE_LR - standard protocols
  #include <esp_now.h>
  #include <esp_wifi.h>
#endif
#endif // #ifndef ESP8266


//-------------------------------------------------------
// Internals
//-------------------------------------------------------

// TCP, UDP, UDPCl
IPAddress ip_gateway(0, 0, 0, 0);
IPAddress netmask(255, 255, 255, 0);
// UDP, UDPSTA (will be overwritten if WIFI_USE_BROADCAST_FOR_UDP is set)
#ifndef ESP8266
IPAddress ip_udp(ip[0], ip[1], ip[2], ip[3]+1); // usually the client/MissionPlanner gets assigned +1
#else // the ESP8266 requires different, appears to be a bug in the Arduino lib
IPAddress ip_udp(ip[0], ip[1], ip[2], ip[3]+99); // the first DHCP client/MissionPlanner gets assigned +99
#endif
WiFiUDP udp;
// TCP
WiFiServer server(port_tcp);
WiFiClient client;
// Bluetooth
#ifdef USE_WIRELESS_PROTOCOL_BLUETOOTH
BluetoothSerial SerialBT;
#endif
// BLE
#ifdef USE_WIRELESS_PROTOCOL_BLE
#define BLE_SERVICE_UUID            "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // Nordic UART service UUID
#define BLE_CHARACTERISTIC_UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHARACTERISTIC_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
BLEServer* ble_server = NULL;
BLECharacteristic* ble_tx_characteristic;
bool ble_device_connected;
bool ble_serial_started;
uint16_t ble_negotiated_mtu;
unsigned long ble_adv_tlast_ms;
extern bool is_connected; // forward declarations, needed for BLE callbacks
extern unsigned long is_connected_tlast_ms; // forward declarations, needed for BLE callbacks

class BLEServerCallbacksHandler : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        ble_device_connected = true;
        ble_negotiated_mtu = pServer->getPeerMTU(pServer->getConnId());
        DBG_PRINTLN("BLE connected");
    }
    void onDisconnect(BLEServer* pServer) {
        ble_device_connected = false;
        ble_serial_started = false;
        DBG_PRINTLN("BLE disconnected");
    }
};

class BLECharacteristicCallbacksHandler : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        if (!ble_serial_started) {
            ble_serial_started = true;
        }
#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        std::string rxValue = pCharacteristic->getValue(); // Core 2.x
#else
        String rxValue = pCharacteristic->getValue(); // Core 3.x
#endif
        uint32_t len = rxValue.length();
        if (len > 0) {
            SERIAL.write((uint8_t*)rxValue.c_str(), len);
            is_connected = true;
            is_connected_tlast_ms = millis();
        }
    }
};
#endif // USE_WIRELESS_PROTOCOL_BLE

// ESP-NOW Long Range
#ifdef USE_WIRELESS_PROTOCOL_ESPNOW
#define ESPNOW_MAX_PACKET_SIZE 250
uint8_t espnow_peer_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast initially
bool espnow_peer_found = false;
bool espnow_peer_saved = false; // true if MAC was loaded from NVS
volatile bool espnow_need_save = false; // flag to save MAC in main loop
volatile bool espnow_send_ready = true;

// Unpair button (Boot button on ESP32-C3 Super Mini)
#define UNPAIR_BUTTON_PIN 9
#define UNPAIR_HOLD_TIME 5000  // Hold for 5 seconds to unpair
unsigned long unpair_button_press_start = 0;
bool unpair_button_held = false;

// Heartbeat for discovery (sent every 500ms when no data)
unsigned long espnow_heartbeat_tlast_ms = 0;
#define ESPNOW_HEARTBEAT_INTERVAL 500
#define ESPNOW_HEARTBEAT_MAGIC 0xAA55 // Magic bytes to identify heartbeat

// Ring buffer for received ESP-NOW data
#define ESPNOW_RX_BUF_SIZE 512
volatile uint8_t espnow_rx_buf[ESPNOW_RX_BUF_SIZE];
volatile uint16_t espnow_rx_head = 0;
volatile uint16_t espnow_rx_tail = 0;

// Check if MAC is broadcast address
bool espnow_is_broadcast(const uint8_t* mac) {
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0xFF) return false;
    }
    return true;
}

// ESP-NOW receive callback
void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    // If we have a saved peer, only accept packets from that peer
    if (espnow_peer_saved && !espnow_is_broadcast(espnow_peer_mac)) {
        if (memcmp(recv_info->src_addr, espnow_peer_mac, 6) != 0) {
            return; // Ignore packets from other senders
        }
    }

    // Store sender MAC if not yet paired
    if (!espnow_peer_found) {
        memcpy(espnow_peer_mac, recv_info->src_addr, 6);
        espnow_peer_found = true;
        espnow_need_save = true; // Signal main loop to save MAC

        // Add peer
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, espnow_peer_mac, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        if (esp_now_is_peer_exist(espnow_peer_mac)) {
            esp_now_del_peer(espnow_peer_mac);
        }
        esp_now_add_peer(&peerInfo);
    }

    // Check if this is a heartbeat packet (don't forward to serial)
    if (len == 2 && data[0] == (ESPNOW_HEARTBEAT_MAGIC >> 8) && data[1] == (ESPNOW_HEARTBEAT_MAGIC & 0xFF)) {
        // Heartbeat received - just update connection status
        extern bool is_connected;
        extern unsigned long is_connected_tlast_ms;
        is_connected = true;
        is_connected_tlast_ms = millis();
        return;
    }

    // Store data in ring buffer
    for (int i = 0; i < len; i++) {
        uint16_t next_head = (espnow_rx_head + 1) % ESPNOW_RX_BUF_SIZE;
        if (next_head != espnow_rx_tail) {
            espnow_rx_buf[espnow_rx_head] = data[i];
            espnow_rx_head = next_head;
        }
    }

    extern bool is_connected;
    extern unsigned long is_connected_tlast_ms;
    is_connected = true;
    is_connected_tlast_ms = millis();
}

// ESP-NOW send callback
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    espnow_send_ready = true;
}

// Check if data available in ESP-NOW RX buffer
int espnow_available() {
    return (ESPNOW_RX_BUF_SIZE + espnow_rx_head - espnow_rx_tail) % ESPNOW_RX_BUF_SIZE;
}

// Read from ESP-NOW RX buffer
int espnow_read(uint8_t* buf, int maxlen) {
    int count = 0;
    while (espnow_rx_tail != espnow_rx_head && count < maxlen) {
        buf[count++] = espnow_rx_buf[espnow_rx_tail];
        espnow_rx_tail = (espnow_rx_tail + 1) % ESPNOW_RX_BUF_SIZE;
    }
    return count;
}
#endif // USE_WIRELESS_PROTOCOL_ESPNOW

// ESP-NOW + BLE combined
#ifdef USE_WIRELESS_PROTOCOL_ESPNOW_BLE
// BLE UUIDs - Nordic UART Service compatible
#define BLE_ESPNOW_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_ESPNOW_CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_ESPNOW_CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer* ble_espnow_server = nullptr;
BLECharacteristic* ble_espnow_tx_characteristic = nullptr;
bool ble_espnow_device_connected = false;
uint16_t ble_espnow_negotiated_mtu = 23;

// Ring buffer for BLE RX data (from tablet)
#define BLE_ESPNOW_RX_BUF_SIZE 512
volatile uint8_t ble_espnow_rx_buf[BLE_ESPNOW_RX_BUF_SIZE];
volatile uint16_t ble_espnow_rx_head = 0;
volatile uint16_t ble_espnow_rx_tail = 0;

#define ESPNOW_BLE_MAX_PACKET_SIZE 250
uint8_t espnow_ble_peer_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast initially
bool espnow_ble_peer_found = false;
volatile bool espnow_ble_send_ready = true;

// Ring buffer for received ESP-NOW data (from Air Unit)
#define ESPNOW_BLE_RX_BUF_SIZE 512
volatile uint8_t espnow_ble_rx_buf[ESPNOW_BLE_RX_BUF_SIZE];
volatile uint16_t espnow_ble_rx_head = 0;
volatile uint16_t espnow_ble_rx_tail = 0;

// BLE Server callbacks
class BLEEspnowServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
        ble_espnow_device_connected = true;
        ble_espnow_negotiated_mtu = 23;
        if (param->connect.conn_id != 0xFFFF) {
            ble_espnow_negotiated_mtu = pServer->getPeerMTU(param->connect.conn_id);
        }
    }
    void onDisconnect(BLEServer* pServer) {
        ble_espnow_device_connected = false;
        BLEDevice::startAdvertising();
    }
};

// BLE Characteristic callbacks (for RX from tablet)
class BLEEspnowCharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        String rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            // Store in RX buffer
            for (int i = 0; i < rxValue.length(); i++) {
                uint16_t next_head = (ble_espnow_rx_head + 1) % BLE_ESPNOW_RX_BUF_SIZE;
                if (next_head != ble_espnow_rx_tail) {
                    ble_espnow_rx_buf[ble_espnow_rx_head] = rxValue[i];
                    ble_espnow_rx_head = next_head;
                }
            }
            extern bool is_connected;
            extern unsigned long is_connected_tlast_ms;
            is_connected = true;
            is_connected_tlast_ms = millis();
        }
    }
};

// ESP-NOW receive callback for combined protocol (from Air Unit)
void espnow_ble_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    // Store sender MAC if not yet paired
    if (!espnow_ble_peer_found) {
        memcpy(espnow_ble_peer_mac, recv_info->src_addr, 6);
        espnow_ble_peer_found = true;
        // Add peer
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, espnow_ble_peer_mac, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        if (esp_now_is_peer_exist(espnow_ble_peer_mac)) {
            esp_now_del_peer(espnow_ble_peer_mac);
        }
        esp_now_add_peer(&peerInfo);
    }

    // Store data in ring buffer
    for (int i = 0; i < len; i++) {
        uint16_t next_head = (espnow_ble_rx_head + 1) % ESPNOW_BLE_RX_BUF_SIZE;
        if (next_head != espnow_ble_rx_tail) {
            espnow_ble_rx_buf[espnow_ble_rx_head] = data[i];
            espnow_ble_rx_head = next_head;
        }
    }

    extern bool is_connected;
    extern unsigned long is_connected_tlast_ms;
    is_connected = true;
    is_connected_tlast_ms = millis();
}

// ESP-NOW send callback for combined protocol
void espnow_ble_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    espnow_ble_send_ready = true;
}

// Check if data available in ESP-NOW RX buffer (from Air Unit)
int espnow_ble_available() {
    return (ESPNOW_BLE_RX_BUF_SIZE + espnow_ble_rx_head - espnow_ble_rx_tail) % ESPNOW_BLE_RX_BUF_SIZE;
}

// Read from ESP-NOW RX buffer
int espnow_ble_read(uint8_t* buf, int maxlen) {
    int count = 0;
    while (espnow_ble_rx_tail != espnow_ble_rx_head && count < maxlen) {
        buf[count++] = espnow_ble_rx_buf[espnow_ble_rx_tail];
        espnow_ble_rx_tail = (espnow_ble_rx_tail + 1) % ESPNOW_BLE_RX_BUF_SIZE;
    }
    return count;
}

// Check if data available in BLE RX buffer (from tablet)
int ble_espnow_available() {
    return (BLE_ESPNOW_RX_BUF_SIZE + ble_espnow_rx_head - ble_espnow_rx_tail) % BLE_ESPNOW_RX_BUF_SIZE;
}

// Read from BLE RX buffer
int ble_espnow_read(uint8_t* buf, int maxlen) {
    int count = 0;
    while (ble_espnow_rx_tail != ble_espnow_rx_head && count < maxlen) {
        buf[count++] = ble_espnow_rx_buf[ble_espnow_rx_tail];
        ble_espnow_rx_tail = (ble_espnow_rx_tail + 1) % BLE_ESPNOW_RX_BUF_SIZE;
    }
    return count;
}
#endif // USE_WIRELESS_PROTOCOL_ESPNOW_BLE

// ESP-NOW + classic Bluetooth SPP (for ESP32 DevKit)
#ifdef USE_WIRELESS_PROTOCOL_ESPNOW_BT_SPP
BluetoothSerial SerialBT_ESP;

#define ESPNOW_BT_MAX_PACKET_SIZE 250
uint8_t espnow_bt_peer_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast initially
bool espnow_bt_peer_found = false;
volatile bool espnow_bt_send_ready = true;

// Ring buffer for received ESP-NOW data (from Air Unit)
#define ESPNOW_BT_RX_BUF_SIZE 512
volatile uint8_t espnow_bt_rx_buf[ESPNOW_BT_RX_BUF_SIZE];
volatile uint16_t espnow_bt_rx_head = 0;
volatile uint16_t espnow_bt_rx_tail = 0;

// ESP-NOW receive callback for combined protocol (from Air Unit)
void espnow_bt_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    // Store sender MAC if not yet paired
    if (!espnow_bt_peer_found) {
        memcpy(espnow_bt_peer_mac, recv_info->src_addr, 6);
        espnow_bt_peer_found = true;
        // Add peer
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, espnow_bt_peer_mac, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        if (esp_now_is_peer_exist(espnow_bt_peer_mac)) {
            esp_now_del_peer(espnow_bt_peer_mac);
        }
        esp_now_add_peer(&peerInfo);
    }

    // Store data in ring buffer
    for (int i = 0; i < len; i++) {
        uint16_t next_head = (espnow_bt_rx_head + 1) % ESPNOW_BT_RX_BUF_SIZE;
        if (next_head != espnow_bt_rx_tail) {
            espnow_bt_rx_buf[espnow_bt_rx_head] = data[i];
            espnow_bt_rx_head = next_head;
        }
    }

    extern bool is_connected;
    extern unsigned long is_connected_tlast_ms;
    is_connected = true;
    is_connected_tlast_ms = millis();
}

// ESP-NOW send callback for combined protocol
void espnow_bt_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    espnow_bt_send_ready = true;
}

// Check if data available in ESP-NOW RX buffer (from Air Unit)
int espnow_bt_available() {
    return (ESPNOW_BT_RX_BUF_SIZE + espnow_bt_rx_head - espnow_bt_rx_tail) % ESPNOW_BT_RX_BUF_SIZE;
}

// Read from ESP-NOW RX buffer
int espnow_bt_read(uint8_t* buf, int maxlen) {
    int count = 0;
    while (espnow_bt_rx_tail != espnow_bt_rx_head && count < maxlen) {
        buf[count++] = espnow_bt_rx_buf[espnow_bt_rx_tail];
        espnow_bt_rx_tail = (espnow_bt_rx_tail + 1) % ESPNOW_BT_RX_BUF_SIZE;
    }
    return count;
}
#endif // USE_WIRELESS_PROTOCOL_ESPNOW_BT_SPP

typedef enum {
    WIRELESS_PROTOCOL_TCP = 0,
    WIRELESS_PROTOCOL_UDP = 1,
    WIRELESS_PROTOCOL_UDPSTA = 2,
    WIRELESS_PROTOCOL_BT = 3,
    WIRELESS_PROTOCOL_UDPCl = 4,
    WIRELESS_PROTOCOL_BLE = 5,
    WIRELESS_PROTOCOL_ESPNOW = 6,       // ESP-NOW LR mode
    WIRELESS_PROTOCOL_ESPNOW_BT = 7,    // ESP-NOW + BLE (S3 Ground Unit)
    WIRELESS_PROTOCOL_ESPNOW_STD = 8,   // ESP-NOW Standard (for BLE compatibility)
    WIRELESS_PROTOCOL_ESPNOW_BT_SPP = 9, // ESP-NOW + classic Bluetooth SPP
} WIRELESS_PROTOCOL_ENUM;

typedef enum {
    WIFIPOWER_LOW = 0,
    WIFIPOWER_MED,
    WIFIPOWER_MAX,
} WIFIPOWER_ENUM;

#define PROTOCOL_DEFAULT  WIRELESS_PROTOCOL
#define BAUDRATE_DEFAULT  BAUD_RATE
#define WIFICHANNEL_DEFAULT  WIFI_CHANNEL
#define WIFIPOWER_DEFAULT  WIFIPOWER_MED

#define G_PROTOCOL_STR  "protocol"
int g_protocol = PROTOCOL_DEFAULT;
#define G_BAUDRATE_STR  "baudrate"
int g_baudrate = BAUDRATE_DEFAULT;
#define G_WIFICHANNEL_STR  "wifichannel"
int g_wifichannel = WIFICHANNEL_DEFAULT;
#define G_WIFIPOWER_STR  "wifipower"
int g_wifipower = WIFIPOWER_DEFAULT;
#define G_BINDPHRASE_STR  "bindphrase"
String g_bindphrase = "mlrs.0";
#define G_PEERMAC_STR  "peermac"

uint16_t device_id = 0; // is going to be set by setup_device_name_and_password(), and can be queried in at mode
String device_name = "";
String device_password = "";

#ifdef USE_AT_MODE
#include <Preferences.h>
Preferences preferences;
#include "mlrs-wireless-bridge-at-mode.h"
AtMode at_mode;
#endif

bool wifi_initialized;
bool led_state;
unsigned long led_tlast_ms;
bool is_connected;
unsigned long is_connected_tlast_ms;
unsigned long serial_data_received_tfirst_ms;


void serialFlushRx(void)
{
    while (SERIAL.available() > 0) { SERIAL.read(); }
}


//-------------------------------------------------------
// setup() and loop()
//-------------------------------------------------------

void setup_device_name_and_password(void)
{
    uint8_t MAC_buf[6+2];
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/misc_system_api.html#mac-address
    // MACs are different for STA and AP, BT
#ifndef ESP8266
    esp_base_mac_addr_get(MAC_buf);
#else
    wifi_get_macaddr(STATION_IF, MAC_buf);
#endif
    for (uint8_t i = 0; i < 5; i++) device_id += MAC_buf[i] + ((uint16_t)MAC_buf[i + 1] << 8) / 39;
    device_id += MAC_buf[5];
    device_name = "mLRS-";
#ifdef DEVICE_NAME_HEAD
    device_name = String(DEVICE_NAME_HEAD) + "-mLRS-";
#endif
    if (g_protocol == WIRELESS_PROTOCOL_TCP) {
        device_name = (ssid == "") ? device_name + String(device_id) + " AP TCP" : ssid;
    } else if (g_protocol == WIRELESS_PROTOCOL_UDP) {
        device_name = (ssid == "") ? device_name + String(device_id) + " AP UDP" : ssid;
    } else if (g_protocol == WIRELESS_PROTOCOL_UDPSTA) {
        device_name = (network_ssid == "") ? device_name + String(device_id) + " STA UDP" : network_ssid;
        device_password = (network_ssid == "") ? String("mLRS-") + g_bindphrase : network_password;
    } else if (g_protocol == WIRELESS_PROTOCOL_UDPCl) {
        device_name = network_ssid;
        device_password = network_password;
    } else if (g_protocol == WIRELESS_PROTOCOL_BT) {
        device_name = (bluetooth_device_name == "") ? device_name + String(device_id) + " BT" : bluetooth_device_name;
    } else if (g_protocol == WIRELESS_PROTOCOL_BLE) {
        device_name = (ble_device_name == "") ? device_name + String(device_id) + " BLE" : ble_device_name;
    } else if (g_protocol == WIRELESS_PROTOCOL_ESPNOW_BT) {
        device_name = (ble_device_name == "") ? device_name + String(device_id) + " ESPNOW-BLE" : ble_device_name;
    } else if (g_protocol == WIRELESS_PROTOCOL_ESPNOW_BT_SPP) {
        device_name = (bluetooth_device_name == "") ? device_name + String(device_id) + " ESPNOW-BT" : bluetooth_device_name;
    }
}


void setup_wifipower()
{
#ifndef ESP8266
    switch (g_wifipower) {
        case WIFIPOWER_LOW: WiFi.setTxPower(WIFI_POWER_MINUS_1dBm); break;
#ifdef WIFI_POWER
        case WIFIPOWER_MED: WiFi.setTxPower(WIFI_POWER); break;
#else
        case WIFIPOWER_MED: WiFi.setTxPower(WIFI_POWER_5dBm); break;
#endif
        case WIFIPOWER_MAX: WiFi.setTxPower(WIFI_POWER_19_5dBm); break;
    }
#else
    switch (g_wifipower) {
        case WIFIPOWER_LOW: WiFi.setOutputPower(0); break;
#ifdef WIFI_POWER
        case WIFIPOWER_MED: WiFi.setOutputPower(WIFI_POWER); break;
#else
        case WIFIPOWER_MED: WiFi.setOutputPower(5); break;
#endif
        case WIFIPOWER_MAX: WiFi.setOutputPower(20.5); break;
    }
#endif
}


void setup_wifi()
{
if (g_protocol == WIRELESS_PROTOCOL_TCP || g_protocol == WIRELESS_PROTOCOL_UDP) {
//-- WiFi TCP, UDP

    // AP mode
    WiFi.mode(WIFI_AP); // seems not to be needed, done by WiFi.softAP()?
    WiFi.softAPConfig(ip, ip_gateway, netmask);
    WiFi.softAP(device_name.c_str(), (password.length()) ? password.c_str() : NULL, g_wifichannel); // channel = 1 is default
    DBG_PRINT("ap ip address: ");
    DBG_PRINTLN(WiFi.softAPIP()); // comes out as 192.168.4.1
    DBG_PRINT("channel: ");
    DBG_PRINTLN(WiFi.channel());

    setup_wifipower();
    if (g_protocol == WIRELESS_PROTOCOL_TCP) {
        server.begin();
        server.setNoDelay(true);
    } else
    if (g_protocol == WIRELESS_PROTOCOL_UDP) {
#ifdef WIFI_USE_BROADCAST_FOR_UDP
        //ip_udp = WiFi.broadcastIP(); // seems to not work for AP mode
        ip_udp[3] = 255; // start with broadcast, the subnet mask is 255.255.255.0 so just last octet needs to change
#endif
        udp.begin(port_udp);
    }

} else
if (g_protocol == WIRELESS_PROTOCOL_BT) {
//-- Bluetooth
// Comment: CONFIG_BT_SSP_ENABLED appears to be defined per default, so setPin() is not available
#ifdef USE_WIRELESS_PROTOCOL_BLUETOOTH

    SerialBT.begin(device_name);

#endif
} else
if (g_protocol == WIRELESS_PROTOCOL_BLE) {
//-- BLE
#ifdef USE_WIRELESS_PROTOCOL_BLE

    ble_device_connected = false;
    ble_serial_started = false;
    ble_negotiated_mtu = 23;
    ble_adv_tlast_ms = 0;

    // Create BLE Device, set MTU
    BLEDevice::init(device_name.c_str());
    BLEDevice::setMTU(512);

    // Set BLE Power
    BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_DEFAULT);

    // Create BLE server, add callbacks
    ble_server = BLEDevice::createServer();
    ble_server->setCallbacks(new BLEServerCallbacksHandler());

    // Create BLE service
    BLEService* ble_service = ble_server->createService(BLE_SERVICE_UUID);

    // Create BLE characteristics, add callback
    ble_tx_characteristic = ble_service->createCharacteristic(BLE_CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    ble_tx_characteristic->addDescriptor(new BLE2902());
    BLECharacteristic* ble_rx_characteristic = ble_service->createCharacteristic(BLE_CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    ble_rx_characteristic->setCallbacks(new BLECharacteristicCallbacksHandler());

    // Start service
    ble_service->start();

    // Configure advertising
    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    advertising->setMinPreferred(0x12);

    // Start advertising
    advertising->start();
    DBG_PRINTLN("BLE advertising started");

#endif
} else
if (g_protocol == WIRELESS_PROTOCOL_UDPSTA) {
//-- Wifi UDPSTA

    // STA mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.begin(device_name.c_str(), device_password.c_str());

    while (WiFi.status() != WL_CONNECTED) {
        led_on(true); delay(75); led_off(); delay(75);
        led_on(true); delay(75); led_off(); delay(75);
        led_on(true); delay(75); led_off(); delay(75);
        DBG_PRINTLN("connecting to WiFi network...");
    }
    DBG_PRINTLN("connected");
    DBG_PRINT("network ip address: ");
    DBG_PRINTLN(WiFi.localIP());

    setup_wifipower();
#ifdef WIFI_USE_BROADCAST_FOR_UDP
    ip_udp = WiFi.broadcastIP(); // start with broadcast
#endif
    udp.begin(port_udp);

} else
if (g_protocol == WIRELESS_PROTOCOL_UDPCl) {
//-- Wifi UDPCl

    // STA mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.config(ip_udpcl, ip_gateway, netmask);
    WiFi.begin(device_name.c_str(), device_password.c_str());

    while (WiFi.status() != WL_CONNECTED) {
        //delay(500);
        led_on(true); delay(75); led_off(); delay(75);
        led_on(true); delay(75); led_off(); delay(75);
        led_on(true); delay(75); led_off(); delay(75);
        DBG_PRINTLN("connecting to WiFi network...");
    }
    DBG_PRINTLN("connected");
    DBG_PRINT("network ip address: ");
    DBG_PRINTLN(WiFi.localIP());

    setup_wifipower();
    udp.begin(port_udpcl);

} else
if (g_protocol == WIRELESS_PROTOCOL_ESPNOW || g_protocol == WIRELESS_PROTOCOL_ESPNOW_STD) {
//-- ESP-NOW (LR or Standard depending on build flag)
#ifdef USE_WIRELESS_PROTOCOL_ESPNOW

    // Initialize WiFi in STA mode for ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

#ifdef ESPNOW_USE_LR
    // Enable Long Range mode
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
    DBG_PRINTLN("ESP-NOW LR mode enabled");
#else
    // Use standard WiFi protocols (better compatibility)
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    DBG_PRINTLN("ESP-NOW standard mode enabled");
#endif

    // Set max TX power
    esp_wifi_set_max_tx_power(84); // 21 dBm max

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        DBG_PRINTLN("ESP-NOW init failed!");
        return;
    }

    // Register callbacks
    esp_now_register_recv_cb(espnow_recv_cb);
    esp_now_register_send_cb(espnow_send_cb);

    // Try to load saved peer MAC from NVS
    #ifdef USE_AT_MODE
    uint8_t saved_mac[6];
    if (preferences.getBytes(G_PEERMAC_STR, saved_mac, 6) == 6) {
        if (!espnow_is_broadcast(saved_mac)) {
            memcpy(espnow_peer_mac, saved_mac, 6);
            espnow_peer_found = true;
            espnow_peer_saved = true;
            DBG_PRINTLN("Loaded paired MAC from NVS");
        }
    }
    #endif

    // Add peer (saved or broadcast for discovery)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, espnow_peer_mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    // Also add broadcast peer if we have a saved peer (for heartbeat discovery)
    if (espnow_peer_saved) {
        esp_now_peer_info_t broadcastPeer = {};
        memset(broadcastPeer.peer_addr, 0xFF, 6);
        broadcastPeer.channel = 0;
        broadcastPeer.encrypt = false;
        esp_now_add_peer(&broadcastPeer);
    }

    // Setup unpair button
    pinMode(UNPAIR_BUTTON_PIN, INPUT_PULLUP);

    DBG_PRINTLN("ESP-NOW initialized");
#ifdef UNIT_AIR
    DBG_PRINTLN("Mode: AIR UNIT");
#else
    DBG_PRINTLN("Mode: GROUND UNIT");
#endif
    if (espnow_peer_saved) {
        DBG_PRINT("Paired with: ");
        for (int i = 0; i < 6; i++) {
            if (i > 0) DBG_PRINT(":");
            if (espnow_peer_mac[i] < 16) DBG_PRINT("0");
            DBG_PRINT(String(espnow_peer_mac[i], HEX));
        }
        DBG_PRINTLN("");
    } else {
        DBG_PRINTLN("No paired device - waiting for first connection");
    }
    DBG_PRINTLN("Hold BOOT button for 5 sec to unpair");

#endif // USE_WIRELESS_PROTOCOL_ESPNOW
} else
if (g_protocol == WIRELESS_PROTOCOL_ESPNOW_BT) {
//-- ESP-NOW + BLE combined (Ground Unit with BLE to tablet)
#ifdef USE_WIRELESS_PROTOCOL_ESPNOW_BLE

    // IMPORTANT: Initialize WiFi/ESP-NOW FIRST, then BLE
    // This order is critical for WiFi/BLE coexistence on ESP32-S3

    // Initialize WiFi in STA mode for ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    delay(100); // Allow WiFi to stabilize

    // Use standard ESP-NOW protocol (not LR) for better BLE coexistence
    // LR mode uses proprietary modulation that can interfere with BLE
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

    // Set TX power
    esp_wifi_set_max_tx_power(84); // 21 dBm max

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        DBG_PRINTLN("ESP-NOW init failed!");
        return;
    }

    // Register callbacks
    esp_now_register_recv_cb(espnow_ble_recv_cb);
    esp_now_register_send_cb(espnow_ble_send_cb);

    // Add broadcast peer initially (for discovery)
    esp_now_peer_info_t peerInfo = {};
    memset(peerInfo.peer_addr, 0xFF, 6); // broadcast
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    DBG_PRINTLN("ESP-NOW initialized");

    delay(100); // Allow ESP-NOW to stabilize before starting BLE

    // Now initialize BLE
    BLEDevice::init(device_name.c_str());
    BLEDevice::setMTU(512);
    BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_DEFAULT);

    // Create BLE server
    ble_espnow_server = BLEDevice::createServer();
    ble_espnow_server->setCallbacks(new BLEEspnowServerCallbacks());

    // Create BLE service with enough handles (4 handles per characteristic + service)
    BLEService* ble_service = ble_espnow_server->createService(BLEUUID(BLE_ESPNOW_SERVICE_UUID), 15);

    // Create TX characteristic (for sending data TO the client/QGC)
    ble_espnow_tx_characteristic = ble_service->createCharacteristic(
        BLE_ESPNOW_CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    BLE2902* ble2902 = new BLE2902();
    ble2902->setNotifications(true);
    ble_espnow_tx_characteristic->addDescriptor(ble2902);

    // Create RX characteristic (for receiving data FROM the client/QGC)
    BLECharacteristic* ble_rx_characteristic = ble_service->createCharacteristic(
        BLE_ESPNOW_CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    ble_rx_characteristic->setCallbacks(new BLEEspnowCharacteristicCallbacks());

    DBG_PRINTLN("BLE characteristics created");

    // Start service
    ble_service->start();

    // Start advertising
    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_ESPNOW_SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);
    advertising->start();

    DBG_PRINTLN("BLE started: " + device_name);
    DBG_PRINTLN("ESP-NOW + BLE initialized");
    DBG_PRINTLN("Mode: GROUND UNIT (ESP-NOW <-> BLE)");

#endif // USE_WIRELESS_PROTOCOL_ESPNOW_BLE
} else
if (g_protocol == WIRELESS_PROTOCOL_ESPNOW_BT_SPP) {
//-- ESP-NOW + classic Bluetooth SPP (Ground Unit with Bluetooth to tablet)
#ifdef USE_WIRELESS_PROTOCOL_ESPNOW_BT_SPP

    // Initialize WiFi in STA mode for ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    delay(100);

    // Use standard WiFi protocols for ESP-NOW
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

    // Set TX power
    esp_wifi_set_max_tx_power(84); // 21 dBm max

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        DBG_PRINTLN("ESP-NOW init failed!");
        return;
    }

    // Register callbacks
    esp_now_register_recv_cb(espnow_bt_recv_cb);
    esp_now_register_send_cb(espnow_bt_send_cb);

    // Add broadcast peer initially (for discovery)
    esp_now_peer_info_t peerInfo = {};
    memset(peerInfo.peer_addr, 0xFF, 6); // broadcast
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    DBG_PRINTLN("ESP-NOW initialized");

    // Initialize classic Bluetooth SPP
    SerialBT_ESP.begin(device_name);
    DBG_PRINTLN("Bluetooth started: " + device_name);
    DBG_PRINTLN("ESP-NOW + Bluetooth SPP initialized");
    DBG_PRINTLN("Mode: GROUND UNIT (ESP-NOW <-> Bluetooth)");

#endif // USE_WIRELESS_PROTOCOL_ESPNOW_BT_SPP
}
}


void setup()
{
    led_init();
    dbg_init();
    //delay(500); // we delay for 750 ms anyway

    // Preferences
#ifdef USE_AT_MODE
    preferences.begin("setup", false);

    g_protocol = preferences.getInt(G_PROTOCOL_STR, 255); // 255 indicates not available
    if (g_protocol != WIRELESS_PROTOCOL_TCP && g_protocol != WIRELESS_PROTOCOL_UDP && g_protocol != WIRELESS_PROTOCOL_UDPSTA &&
        g_protocol != WIRELESS_PROTOCOL_UDPCl && g_protocol != WIRELESS_PROTOCOL_BT && g_protocol != WIRELESS_PROTOCOL_BLE &&
        g_protocol != WIRELESS_PROTOCOL_ESPNOW && g_protocol != WIRELESS_PROTOCOL_ESPNOW_BT &&
        g_protocol != WIRELESS_PROTOCOL_ESPNOW_STD && g_protocol != WIRELESS_PROTOCOL_ESPNOW_BT_SPP) { // not a valid value
        g_protocol = PROTOCOL_DEFAULT;
        preferences.putInt(G_PROTOCOL_STR, g_protocol);
    }

    g_baudrate = preferences.getInt(G_BAUDRATE_STR, 0); // 0 indicates not available
    if (g_baudrate != 9600 && g_baudrate != 19200 && g_baudrate != 38400 &&
        g_baudrate != 57600 && g_baudrate != 115200 && g_baudrate != 230400) { // not a valid value
        g_baudrate = BAUDRATE_DEFAULT;
        preferences.putInt(G_BAUDRATE_STR, g_baudrate);
    }

    g_wifichannel = preferences.getInt(G_WIFICHANNEL_STR, 0); // 0 indicates not available
    if (g_wifichannel != 1 && g_wifichannel != 6 && g_wifichannel != 11 && g_wifichannel != 13) { // not a valid value
        g_wifichannel = WIFICHANNEL_DEFAULT;
        preferences.putInt(G_WIFICHANNEL_STR, g_wifichannel);
    }

    g_wifipower = preferences.getInt(G_WIFIPOWER_STR, 255); // 255 indicates not available
    if (g_wifipower < WIFIPOWER_LOW || g_wifipower > WIFIPOWER_MAX) { // not a valid value
        g_wifipower = WIFIPOWER_DEFAULT;
        preferences.putInt(G_WIFIPOWER_STR, g_wifipower);
    }

    g_bindphrase = preferences.getString(G_BINDPHRASE_STR, "");
    if (g_bindphrase.length() == 0) { // not found or empty
        g_bindphrase = "mlrs.0"; // mLRS default bind phrase
        preferences.putString(G_BINDPHRASE_STR, g_bindphrase);
    }
#endif
    setup_device_name_and_password();

    // Serial
    size_t rxbufsize = SERIAL.setRxBufferSize(2*1024); // must come before uart started, retuns 0 if it fails
#ifndef ESP8266 // not implemented on ESP8266
    size_t txbufsize = SERIAL.setTxBufferSize(512); // must come before uart started, retuns 0 if it fails
#endif
#ifdef SERIAL_RXD // if SERIAL_TXD is not defined the compiler will complain, so all good
  #ifdef USE_SERIAL_INVERTED
    SERIAL.begin(g_baudrate, SERIAL_8N1, SERIAL_RXD, SERIAL_TXD, true);
  #else
    SERIAL.begin(g_baudrate, SERIAL_8N1, SERIAL_RXD, SERIAL_TXD);
  #endif
#else
    SERIAL.begin(g_baudrate);
#endif

    DBG_PRINTLN(rxbufsize);
#ifndef ESP8266
    DBG_PRINTLN(txbufsize);
#endif

    // Gpio0 handling (AT-Mode disabled for ESP-NOW - uses button for unpair)
#ifdef USE_AT_MODE
    #ifndef USE_WIRELESS_PROTOCOL_ESPNOW
    at_mode.Init(GPIO0_IO);
    #endif
#endif

    wifi_initialized = false; // setup_wifi();

    led_tlast_ms = 0;
    led_state = false;

    is_connected = false;
    is_connected_tlast_ms = 0;

    serial_data_received_tfirst_ms = 0;

    serialFlushRx();
}


void loop()
{
#ifdef USE_AT_MODE
    // Skip AT-Mode for ESP-NOW protocols (allows unpair button to work)
    #ifndef USE_WIRELESS_PROTOCOL_ESPNOW
    if (at_mode.Do()) return;
    #endif
#endif
    unsigned long tnow_ms = millis();

    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) { // nothing from GCS for 2 secs
        is_connected = false;
    }

    if (tnow_ms - led_tlast_ms > (is_connected ? 500 : 200)) {
        led_tlast_ms = tnow_ms;
        led_state = !led_state;
        if (led_state) led_on(is_connected); else led_off();
    }

    //-- here comes the core code, handle WiFi or Bluetooth connection and do the bridge

    uint8_t buf[256]; // working buffer

    if (!wifi_initialized) {
        wifi_initialized = true;
        setup_wifi();
        return;
    }

if (g_protocol == WIRELESS_PROTOCOL_TCP) {
//-- WiFi TCP

    if (server.hasClient()) {
        if (!client.connected()) {
            client.stop(); // doesn't appear to make a difference
            client = server.accept();
            DBG_PRINTLN("connection");
        } else { // is already connected, so reject, doesn't seem to ever happen
            server.accept().stop();
            DBG_PRINTLN("connection rejected");
        }
    }

    if (!client.connected()) { // nothing to do
        client.stop();
        serialFlushRx();
        is_connected = false;
        return;
    }

    while (client.available()) {
        //uint8_t c = (uint8_t)client.read();
        //SERIAL.write(c);
        int len = client.read(buf, sizeof(buf));
        SERIAL.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    tnow_ms = millis(); // update it
    int avail = SERIAL.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        int len = SERIAL.read(buf, sizeof(buf));
        client.write(buf, len);
    }

} else
if (g_protocol == WIRELESS_PROTOCOL_UDP || g_protocol == WIRELESS_PROTOCOL_UDPSTA) {
//-- WiFi UDP, UDPSTA

    if (g_protocol == WIRELESS_PROTOCOL_UDPSTA){
        if (!is_connected && WiFi.status() != WL_CONNECTED) {
            udp.stop();
            setup_wifi(); // attempt to reconnect if WiFi got disconnected
        }
    }

    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(buf, sizeof(buf));
        SERIAL.write(buf, len);
#ifdef WIFI_USE_BROADCAST_FOR_UDP
        if (!is_connected) { // first received UDP packet
            ip_udp = udp.remoteIP(); // stop broadcast, switch to unicast to avoid Aurdino performance issue
            port_udp = udp.remotePort();
        }
#endif
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = SERIAL.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        int len = SERIAL.read(buf, sizeof(buf));
        udp.beginPacket(ip_udp, port_udp);
        udp.write(buf, len);
        udp.endPacket();
    }

} else
if (g_protocol == WIRELESS_PROTOCOL_BT) {
//-- Bluetooth
#ifdef USE_WIRELESS_PROTOCOL_BLUETOOTH

    int len = SerialBT.available();
    if (len > 0) {
        if (len > sizeof(buf)) len = sizeof(buf);
        for (int i = 0; i < len; i++) buf[i] = SerialBT.read();
        SERIAL.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = SERIAL.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        int len = SERIAL.read(buf, sizeof(buf));
        SerialBT.write(buf, len);
    }

#endif // USE_WIRELESS_PROTOCOL_BLUETOOTH
} else
if (g_protocol == WIRELESS_PROTOCOL_BLE) {
//-- BLE
#ifdef USE_WIRELESS_PROTOCOL_BLE

    if (ble_device_connected) {
        // check serial for data to send over BLE
        tnow_ms = millis();
        int avail = SERIAL.available();
        if (avail <= 0) {
            serial_data_received_tfirst_ms = tnow_ms;
        } else
        if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) {
            serial_data_received_tfirst_ms = tnow_ms;

            // calculate the number of bytes to read, limit to MTU - 3
            uint16_t bytesToRead = min((uint16_t)avail, (uint16_t)(ble_negotiated_mtu - 3));
            if (bytesToRead > sizeof(buf)) bytesToRead = sizeof(buf);

            int len = SERIAL.read(buf, bytesToRead);
            ble_tx_characteristic->setValue(buf, len);
            ble_tx_characteristic->notify();
        }
    } else {
        // not connected, restart advertising every 5 sec if needed
        serialFlushRx();
        is_connected = false;
        if (tnow_ms - ble_adv_tlast_ms > 5000) {
            ble_adv_tlast_ms = tnow_ms;
            ble_server->startAdvertising();
        }
    }

#endif // USE_WIRELESS_PROTOCOL_BLE
} else
if (g_protocol == WIRELESS_PROTOCOL_UDPCl) {
//-- WiFi UDPCl (STA mode)

    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(buf, sizeof(buf));
        SERIAL.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    if (!is_connected) {
        // we wait for a first message from the remote
        // remote's ip and port not known, so jump out
        serialFlushRx();
        return;
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = SERIAL.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        int len = SERIAL.read(buf, sizeof(buf));
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write(buf, len);
        udp.endPacket();
    }

} else
if (g_protocol == WIRELESS_PROTOCOL_ESPNOW || g_protocol == WIRELESS_PROTOCOL_ESPNOW_STD) {
//-- ESP-NOW (LR or Standard)
#ifdef USE_WIRELESS_PROTOCOL_ESPNOW

    // Save peer MAC to NVS if newly paired
    #ifdef USE_AT_MODE
    if (espnow_need_save && !espnow_peer_saved) {
        preferences.putBytes(G_PEERMAC_STR, espnow_peer_mac, 6);
        espnow_peer_saved = true;
        espnow_need_save = false;
        DBG_PRINT("Paired and saved MAC: ");
        for (int i = 0; i < 6; i++) {
            if (i > 0) DBG_PRINT(":");
            if (espnow_peer_mac[i] < 16) DBG_PRINT("0");
            DBG_PRINT(String(espnow_peer_mac[i], HEX));
        }
        DBG_PRINTLN("");
    }
    #endif

    // Check unpair button (hold for 5 seconds)
    if (digitalRead(UNPAIR_BUTTON_PIN) == LOW) {
        if (!unpair_button_held) {
            unpair_button_held = true;
            unpair_button_press_start = tnow_ms;
            DBG_PRINTLN("Button pressed - hold 5 sec to unpair...");
        } else if (tnow_ms - unpair_button_press_start >= UNPAIR_HOLD_TIME) {
            // Button held for 5 seconds - unpair!
            DBG_PRINTLN("UNPAIRING!");
            #ifdef USE_AT_MODE
            preferences.remove(G_PEERMAC_STR);
            #endif
            // Fast blink to confirm
            for (int i = 0; i < 10; i++) {
                led_on(true); delay(50);
                led_off(); delay(50);
            }
            DBG_PRINTLN("Pairing cleared - restarting...");
            delay(500);
            ESP.restart();
        }
    } else {
        if (unpair_button_held) {
            DBG_PRINTLN("Button released - cancelled");
        }
        unpair_button_held = false;
    }

    // Read data from ESP-NOW RX buffer and write to Serial
    int espnow_avail = espnow_available();
    if (espnow_avail > 0) {
        int len = espnow_read(buf, min(espnow_avail, (int)sizeof(buf)));
        if (len > 0) {
            SERIAL.write(buf, len);
        }
    }

    // Read data from Serial and send via ESP-NOW
    tnow_ms = millis();
    int avail = SERIAL.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;

        // Send heartbeat periodically when no serial data (for discovery)
        if (espnow_send_ready && (tnow_ms - espnow_heartbeat_tlast_ms) > ESPNOW_HEARTBEAT_INTERVAL) {
            espnow_heartbeat_tlast_ms = tnow_ms;
            uint8_t heartbeat[2] = {ESPNOW_HEARTBEAT_MAGIC >> 8, ESPNOW_HEARTBEAT_MAGIC & 0xFF};
            espnow_send_ready = false;
            esp_now_send(espnow_peer_mac, heartbeat, 2);
        }
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 5 || avail > 200) { // 5ms timeout, max 200 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        // Only send if previous send is complete
        if (espnow_send_ready) {
            int len = SERIAL.read(buf, min(avail, ESPNOW_MAX_PACKET_SIZE));
            if (len > 0) {
                espnow_send_ready = false;
                esp_now_send(espnow_peer_mac, buf, len);
            }
        }
    }

#endif // USE_WIRELESS_PROTOCOL_ESPNOW
} else
if (g_protocol == WIRELESS_PROTOCOL_ESPNOW_BT) {
//-- ESP-NOW + BLE combined
#ifdef USE_WIRELESS_PROTOCOL_ESPNOW_BLE

    if (ble_espnow_device_connected) {
        // Read data from ESP-NOW RX buffer and send to BLE (to tablet)
        int espnow_avail = espnow_ble_available();
        if (espnow_avail > 0) {
            // Limit to MTU - 3
            uint16_t maxLen = min((uint16_t)(ble_espnow_negotiated_mtu - 3), (uint16_t)sizeof(buf));
            int len = espnow_ble_read(buf, min(espnow_avail, (int)maxLen));
            if (len > 0) {
                ble_espnow_tx_characteristic->setValue(buf, len);
                ble_espnow_tx_characteristic->notify();
            }
        }

        // Read data from BLE RX buffer and send via ESP-NOW (to Air Unit)
        int ble_avail = ble_espnow_available();
        if (ble_avail > 0) {
            // Only send if previous send is complete
            if (espnow_ble_send_ready) {
                int len = ble_espnow_read(buf, min(ble_avail, ESPNOW_BLE_MAX_PACKET_SIZE));
                if (len > 0) {
                    espnow_ble_send_ready = false;
                    esp_now_send(espnow_ble_peer_mac, buf, len);
                }
            }
        }
    }

#endif // USE_WIRELESS_PROTOCOL_ESPNOW_BLE
} else
if (g_protocol == WIRELESS_PROTOCOL_ESPNOW_BT_SPP) {
//-- ESP-NOW + classic Bluetooth SPP
#ifdef USE_WIRELESS_PROTOCOL_ESPNOW_BT_SPP

    // Read data from ESP-NOW RX buffer and send to Bluetooth (to tablet)
    int espnow_avail = espnow_bt_available();
    if (espnow_avail > 0) {
        int len = espnow_bt_read(buf, min(espnow_avail, (int)sizeof(buf)));
        if (len > 0) {
            SerialBT_ESP.write(buf, len);
        }
    }

    // Read data from Bluetooth and send via ESP-NOW (to Air Unit)
    int bt_avail = SerialBT_ESP.available();
    if (bt_avail > 0) {
        // Only send if previous send is complete
        if (espnow_bt_send_ready) {
            int len = 0;
            while (SerialBT_ESP.available() && len < ESPNOW_BT_MAX_PACKET_SIZE) {
                buf[len++] = SerialBT_ESP.read();
            }
            if (len > 0) {
                espnow_bt_send_ready = false;
                esp_now_send(espnow_bt_peer_mac, buf, len);
                is_connected = true;
                is_connected_tlast_ms = millis();
            }
        }
    }

#endif // USE_WIRELESS_PROTOCOL_ESPNOW_BT_SPP
}

    delay(2); // give it always a bit of time
}
