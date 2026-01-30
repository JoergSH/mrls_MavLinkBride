# MAVLink Wireless Bridge

Transparente WiFi / ESP-NOW <-> Serial Bridge für ESP32.
*Transparent WiFi / ESP-NOW <-> Serial Bridge for ESP32.*

Basiert auf [mLRS Wireless Bridge](https://github.com/olliw42/mLRS) von OlliW.
*Based on [mLRS Wireless Bridge](https://github.com/olliw42/mLRS) by OlliW.*

---

## Unterstützte Modi / Supported Modes

| Protocol | Name         | Beschreibung / Description              | Status          |
| -------- | ------------ | --------------------------------------- | --------------- |
| 1        | WiFi UDP AP  | Air Unit erstellt WLAN / creates WiFi   | ✅ Works        |
| 2        | WiFi UDP STA | Ground Unit verbindet / connects to AP  | ✅ Works        |
| 6        | ESP-NOW LR   | ESP-NOW Long Range (~500m+)             | ✅ Works        |
| 8        | ESP-NOW STD  | ESP-NOW Standard (~100m)                | ✅ Works        |

---

## Hardware

### Getestete Boards / Tested Boards

- **ESP32-C3 Super Mini** - Kompakt, günstig / Compact, cheap

### Pinbelegung / Pinout ESP32-C3 Super Mini

| Funktion / Function   | GPIO   |
| --------------------- | ------ |
| Serial RX (from FC)   | GPIO20 |
| Serial TX (to FC)     | GPIO21 |
| LED                   | GPIO8  |
| Boot/Unpair Button    | GPIO9  |

---

## Build Environments

### WiFi Mode

```ini
[env:air_unit]        # Air Unit - erstellt WLAN "MAVLink-Air" / creates WiFi
[env:ground_unit]     # Ground Unit - verbindet zu Air Unit / connects to Air Unit
```

### ESP-NOW Long Range

```ini
[env:air_unit_espnow]     # Air Unit - ESP-NOW LR
[env:ground_unit_espnow]  # Ground Unit - ESP-NOW LR
```

### ESP-NOW Standard

```ini
[env:air_unit_espnow_std]      # Air Unit - ESP-NOW Standard
[env:ground_unit_espnow_std]   # Ground Unit - ESP-NOW Standard
```

---

## Flashen / Flashing

```bash
# Air Unit flashen (mit Erase für saubere Einstellungen)
# Flash Air Unit (with erase for clean settings)
pio run -e air_unit_espnow -t erase -t upload

# Ground Unit flashen / Flash Ground Unit
pio run -e ground_unit_espnow -t erase -t upload

# Serial Monitor
pio device monitor -e air_unit_espnow
```

---

## LED Status

| LED                  | Deutsch                | English              |
| -------------------- | ---------------------- | -------------------- |
| Schnelles Blinken    | Keine Verbindung       | No connection        |
| Langsames Blinken    | Verbunden              | Connected            |

- Fast blink (200ms) = No connection
- Slow blink (500ms) = Connected

---

## ESP-NOW Pairing

### Automatisches Pairing / Automatic Pairing

**Deutsch:**
Beim ersten Start verbinden sich Air Unit und Ground Unit automatisch:
1. Beide Units starten im "Unpaired" Modus
2. Die erste Unit die ein Paket empfängt, speichert die MAC-Adresse des Senders
3. Ab dann kommunizieren nur noch diese beiden Units miteinander
4. Die Pairing-Info wird im Flash gespeichert und überlebt Neustarts

**English:**
On first start, Air Unit and Ground Unit connect automatically:
1. Both units start in "Unpaired" mode
2. The first unit to receive a packet saves the sender's MAC address
3. From then on, only these two units communicate with each other
4. Pairing info is stored in flash and survives restarts

### Unpair (Pairing löschen / Clear Pairing)

**BOOT-Button 5 Sekunden gedrückt halten** während der ESP läuft:
**Hold BOOT button for 5 seconds** while ESP is running:

1. Serial: `Button pressed - hold 5 sec to unpair...`
2. Nach 5 Sek / After 5 sec: LED blinkt schnell / LED blinks fast
3. Serial: `UNPAIRING!` → `Pairing cleared - restarting...`
4. ESP startet neu / ESP restarts

### Neu Pairen / Re-Pair

```bash
# Auf beiden Units Unpair durchführen (Button 5 Sek halten)
# Perform unpair on both units (hold button 5 sec)

# ODER neu flashen mit erase / OR reflash with erase:
pio run -e air_unit_espnow -t erase -t upload
pio run -e ground_unit_espnow -t erase -t upload
```

---

## Verbindung mit QGroundControl / Connecting to QGroundControl

### WiFi Mode

**Deutsch:**
1. QGC öffnen
2. Mit WLAN "MAVLink-Air" verbinden (Passwort: mavlink123)
3. QGC verbindet automatisch über UDP Port 14550

**English:**
1. Open QGC
2. Connect to WiFi "MAVLink-Air" (Password: mavlink123)
3. QGC connects automatically via UDP Port 14550

### ESP-NOW Mode

**Deutsch:**
1. Ground Unit per USB an PC/Tablet anschließen
2. QGC → Comm Links → Add → Serial
3. COM Port der Ground Unit auswählen
4. Baud Rate: 115200

**English:**
1. Connect Ground Unit via USB to PC/Tablet
2. QGC → Comm Links → Add → Serial
3. Select COM port of Ground Unit
4. Baud Rate: 115200

---

## Anpassung an andere Boards / Adapting to Other Boards

### 1. Board in `platformio.ini` definieren / Define board

```ini
[env:my_board_air]
board = esp32-c3-devkitm-1  ; or other board
build_flags =
    -D MODULE_MY_BOARD      ; custom module name
    -D UNIT_AIR
    -D WIRELESS_PROTOCOL=6  ; desired protocol
```

### 2. Board-Definition in `mlrs-wireless-bridge-boards.h`

```cpp
#elif defined MODULE_MY_BOARD
    #define SERIAL        Serial
    #define SERIAL_RXD    20    // RX Pin
    #define SERIAL_TXD    21    // TX Pin
    #define USE_SERIAL1_DBG
    #define SERIAL_DBG    Serial1
    #define LED_IO        8
    #define USE_LED
    #define GPIO0_IO      9     // Unpair button
```

### Protokoll-Auswahl / Protocol Selection

`-D WIRELESS_PROTOCOL=X` in platformio.ini:

- `1` = WiFi UDP AP (Air Unit)
- `2` = WiFi UDP STA (Ground Unit)
- `6` = ESP-NOW Long Range
- `8` = ESP-NOW Standard

---

## WiFi Einstellungen / WiFi Settings

In `main.cpp`:

```cpp
String ssid = "MAVLink-Air";      // WiFi name
String password = "mavlink123";    // WiFi password (min 8 chars)
int port_udp = 14550;              // UDP port (QGC default)
```

---

## Troubleshooting

### LEDs blinken schnell / LEDs blink fast

**Deutsch:**
- Beide Units mit gleichem Protokoll flashen
- `erase` verwenden um alte Einstellungen zu löschen
- Bei gepairten Units: Unpair auf beiden durchführen

**English:**
- Flash both units with same protocol
- Use `erase` to clear old settings
- For paired units: Unpair both

### Units verbinden sich falsch / Units connect incorrectly

- Unpair auf beiden Units / Unpair both units
- Dann neu pairen lassen / Then let them re-pair

---

## Hinweis zu Bluetooth / Note on Bluetooth

**Deutsch:**
Eine direkte Bluetooth-Verbindung (BLE oder Classic BT) zwischen ESP32 und QGroundControl/Mission Planner funktioniert leider nicht zuverlässig. Die Verbindung wird zwar aufgebaut, aber QGC meldet "Connection to service failed". Dies betrifft sowohl Android als auch Windows. Daher wird für ESP-NOW die Verbindung zum PC/Tablet über USB empfohlen.

**English:**
A direct Bluetooth connection (BLE or Classic BT) between ESP32 and QGroundControl/Mission Planner unfortunately does not work reliably. The connection is established, but QGC reports "Connection to service failed". This affects both Android and Windows. Therefore, for ESP-NOW, connection to PC/Tablet via USB is recommended.

---

## Lizenz / License

GPL v3 - https://www.gnu.org/licenses/gpl-3.0.de.html
