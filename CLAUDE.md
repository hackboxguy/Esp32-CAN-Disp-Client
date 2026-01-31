# CLAUDE.md - Project Context for AI Assistants

## Project Overview

**ESP32-CAN-Disp-Client** is an ESP32-C3 firmware project that acts as a CAN bus client, receiving multi-sensor data from a CAN network and displaying it in real-time on a 128x64 SSD1306 OLED display.

**Key Technologies:**
- ESP-IDF v5.x (FreeRTOS-based framework)
- TWAI (CAN 2.0) driver for CAN bus communication
- I2C driver for SSD1306 OLED display
- Custom CAN protocol for sensor data transmission

**Purpose:** Display environmental sensors data (light, temperature, humidity, pressure, air quality, CO2, alcohol, presence detection) received via CAN bus on a compact OLED screen.

---

## Architecture

### System Design

```
┌─────────────────────────────────────────┐
│         ESP32-C3 Firmware               │
│                                         │
│  ┌───────────┐         ┌─────────────┐ │
│  │ CAN RX    │────────▶│ Sensor Data │ │
│  │ Task      │  Mutex  │ Structure   │ │
│  │ (Prio 5)  │◀────────│ (Shared)    │ │
│  └───────────┘         └─────────────┘ │
│                              │          │
│                              │ Mutex    │
│  ┌───────────┐               │          │
│  │Display UI │◀──────────────┘          │
│  │Task       │                          │
│  │(Prio 4,   │                          │
│  │ 10Hz)     │                          │
│  └───────────┘                          │
└─────────────────────────────────────────┘
        │                   │
        ▼                   ▼
    CAN Bus            SSD1306 OLED
   (500kbps)          (128x64, I2C)
```

### Module Responsibilities

| Module | File | Purpose |
|--------|------|---------|
| **Main Controller** | [main/main.c](main/main.c) | Application entry, initialization, system monitoring |
| **CAN Receiver** | [main/can_receiver.c](main/can_receiver.c) | TWAI driver setup, CAN message reception & parsing |
| **Display Driver** | [main/display_driver.c](main/display_driver.c) | Low-level SSD1306 I2C communication, frame buffer management |
| **Display UI** | [main/display_ui.c](main/display_ui.c) | UI layout, sensor data rendering logic |
| **Sensor Data** | [main/sensor_data.h](main/sensor_data.h) | Shared data structure with thread-safe mutex access |
| **CAN Protocol** | [main/can_protocol.h](main/can_protocol.h) | CAN message IDs and packed structure definitions |

---

## Hardware Configuration

### Pin Assignments

| Function | GPIO | Connected To |
|----------|------|--------------|
| CAN TX | GPIO 4 | SN65HVD230 TX |
| CAN RX | GPIO 5 | SN65HVD230 RX |
| I2C SDA | GPIO 6 | SSD1306 SDA |
| I2C SCL | GPIO 7 | SSD1306 SCL |

**CAN Bus:** 500 kbps, 120Ω termination resistors required at bus ends
**I2C:** 400 kHz, SSD1306 at address 0x3C

**Hardware:**
- ESP32-C3 SuperMini (or compatible ESP32 board)
- SSD1306 OLED Display (128x64, I2C)
- SN65HVD230 CAN Transceiver (3.3V compatible)

---

## CAN Protocol Details

### Message IDs and Formats

All CAN messages use **standard 11-bit identifiers** and **8-byte payloads**.

| ID | Sensor | Data Fields | Update Rate | Checksum |
|----|--------|-------------|-------------|----------|
| **0x100** | VEML7700 | Lux (24-bit), Status, Sequence, Config | 1 Hz | Yes |
| **0x101** | BME680 Env | Temp (°C×100), Humidity (%), Pressure (hPa×10), Status | 0.33 Hz | Yes |
| **0x102** | BME680 IAQ | IAQ (0-500), Accuracy (0-3), CO2_equiv (ppm), VOC (ppm), Status | 0.33 Hz | No |
| **0x109** | MQ-3 | Raw ADC, Rs/R0 (×1000), PPM estimate, Status, Sequence | 1 Hz | No |
| **0x107** | LD2410 | Presence (bool), Distance (cm), Move Energy, Static Energy, Status | 10 Hz | Yes |
| **0x10F** | System | Active sensors mask, Free heap (KB), Uptime (sec), Sequence | 0.1 Hz | No |

**All multi-byte values use little-endian encoding.**

### Example: BME680 IAQ Message (0x102)

```c
typedef struct __attribute__((packed)) {
    uint16_t iaq;           // Bytes 0-1: IAQ (0-500)
    uint8_t accuracy;       // Byte 2: Accuracy (0=stabilizing, 3=high)
    uint16_t co2_equiv;     // Bytes 3-4: CO2 equivalent (ppm)
    uint16_t breath_voc;    // Bytes 5-6: Breath VOC (ppm)
    uint8_t status;         // Byte 7: Status (0x00=OK, 0x01=Error)
} can_msg_bme680_iaq_t;
```

See [main/can_protocol.h](main/can_protocol.h) for all message structure definitions.

---

## Data Flow

### Reception Pipeline

1. **CAN RX Task** (`can_receive_task()`) continuously polls TWAI driver for incoming messages
2. **Message Parser** (`process_can_message()`) routes messages by ID to specific parsers
3. **Sensor Parsers** decode binary data, verify checksums, and lock the shared data structure
4. **Shared Data Update** writes decoded values to `g_sensor_data` with mutex protection
5. **Timestamp Update** records reception time for staleness detection

### Display Pipeline

1. **Display UI Task** (`display_update_task()`) runs at 10 Hz
2. **Data Lock** acquires mutex to safely read `g_sensor_data`
3. **Staleness Check** determines if data is older than 5 seconds
4. **Rendering** draws sensor values to frame buffer with appropriate formatting
5. **Buffer Update** sends frame buffer to SSD1306 via I2C

### Thread Safety

All access to `g_sensor_data` is protected by a FreeRTOS mutex:
- **Writers:** CAN receiver task parsers
- **Readers:** Display UI task, main task (system monitoring)
- **Timeout:** 50-100ms for lock acquisition

---

## Display Layout

Current implementation (`display_ui_render_main_screen()`):

```
┌────────────────────────────────┐  Y=0
│ 12345 lux          (FONT_LARGE)│  ← Top section
├────────────────────────────────┤  Y=22
│ 23.5C   65%   1013hPa          │  ← Middle section (3 columns)
├────────────────────────────────┤  Y=36
│ IAQ:120 Good        CO2:450    │  ← Bottom section (IAQ + CO2)
│ Alc:50ppm                      │  Y=46
├────────────────────────────────┤  Y=56
│ P:75cm                  STALE  │  ← Status line
└────────────────────────────────┘  Y=64
```

**Font Sizes:**
- `FONT_LARGE`: 2x scaled (12px height)
- `FONT_SMALL`: 1x (6px height)

**Coordinates:** (x, y) in pixels, origin at top-left (0, 0)

---

## Key Code Locations

### Adding a New Sensor

1. **Define CAN message ID** in [main/can_protocol.h](main/can_protocol.h):
   ```c
   #define CAN_ID_NEW_SENSOR  0x10A  // Example: offset 0x0A from node 0 base
   ```

2. **Define message structure** in [main/can_protocol.h](main/can_protocol.h):
   ```c
   typedef struct __attribute__((packed)) {
       uint16_t value;
       uint8_t status;
       // ... fields ...
   } can_msg_new_sensor_t;
   ```

3. **Add data fields** to `sensor_readings_t` in [main/sensor_data.h](main/sensor_data.h):
   ```c
   uint16_t new_sensor_value;
   uint8_t new_sensor_status;
   ```

4. **Add sensor index** to enum in [main/sensor_data.h](main/sensor_data.h):
   ```c
   typedef enum {
       // ...
       SENSOR_IDX_NEW_SENSOR,
       SENSOR_IDX_COUNT
   } sensor_index_t;
   ```

5. **Create parser function** in [main/can_receiver.c](main/can_receiver.c):
   ```c
   static void parse_new_sensor_msg(const uint8_t *data) {
       // Decode, lock, write to g_sensor_data, unlock
   }
   ```

6. **Add case to message router** in [main/can_receiver.c](main/can_receiver.c) (`process_can_message()`):
   ```c
   case CAN_ID_NEW_SENSOR:
       parse_new_sensor_msg(message->data);
       break;
   ```

7. **Update display rendering** in [main/display_ui.c](main/display_ui.c) (`display_ui_render_main_screen()`):
   ```c
   if (!sensor_data_is_stale(SENSOR_IDX_NEW_SENSOR)) {
       display_draw_printf(x, y, FONT_SMALL, "Value:%d", g_sensor_data.new_sensor_value);
   }
   ```

### Modifying Display Layout

- **Main screen rendering:** [main/display_ui.c](main/display_ui.c) line 29 (`display_ui_render_main_screen()`)
- **Font selection:** Change `FONT_SMALL` or `FONT_LARGE` parameter
- **Positioning:** Adjust `x`, `y` coordinates (0-127 x, 0-63 y)
- **Formatting:** Use `display_draw_printf()` with standard printf format specifiers

### Changing CAN Configuration

- **Bitrate:** [main/can_receiver.h](main/can_receiver.h) line 18 (`CAN_BITRATE`)
- **GPIO pins:** [main/can_receiver.h](main/can_receiver.h) lines 16-17 (`CAN_TX_GPIO`, `CAN_RX_GPIO`)
- **Queue sizes:** [main/can_receiver.c](main/can_receiver.c) lines 27-28 (`tx_queue_len`, `rx_queue_len`)

### Staleness Timeout

- **Timeout value:** [main/sensor_data.h](main/sensor_data.h) line 19 (`SENSOR_DATA_STALE_TIMEOUT_MS`)
- **Check logic:** [main/main.c](main/main.c) line 59 (`sensor_data_is_stale()`)

---

## Build System

**Framework:** ESP-IDF v5.x with CMake

**Project Structure:**
```
Esp32-CAN-Disp-Client/
├── CMakeLists.txt           (Top-level CMake config)
├── sdkconfig.defaults       (Default ESP-IDF configuration)
├── main/
│   ├── CMakeLists.txt       (Main component build rules)
│   ├── main.c               (Application entry point)
│   ├── can_receiver.[ch]    (CAN module)
│   ├── display_driver.[ch]  (Display driver)
│   ├── display_ui.[ch]      (UI rendering)
│   ├── sensor_data.h        (Shared data structures)
│   └── can_protocol.h       (CAN protocol definitions)
├── components/              (External components if any)
├── docs/                    (Documentation)
└── build/                   (Generated build artifacts)
```

**Build Commands:**
```bash
idf.py build              # Compile firmware
idf.py flash              # Flash to device
idf.py monitor            # Serial monitor
idf.py flash monitor      # Flash and monitor
idf.py fullclean          # Clean all build artifacts
```

**Configuration:**
```bash
idf.py menuconfig         # Interactive configuration menu
```

---

## Common Tasks

### Debugging CAN Issues

1. **Check TWAI driver status:**
   - Look for `"TWAI driver started successfully"` in serial output
   - If missing, verify GPIO pins and CAN transceiver wiring

2. **Monitor CAN message reception:**
   - Enable debug logs: Set `ESP_LOG_DEBUG` for `CAN_RX` tag
   - Watch for `"VEML7700: Lux=..."`, `"BME680 ENV: ..."` messages
   - If seeing `"No CAN messages received for N seconds"`, check bus connection

3. **Verify checksum errors:**
   - Look for `"checksum mismatch"` warnings
   - May indicate electrical noise or incorrect sender implementation

### Debugging Display Issues

1. **Check I2C initialization:**
   - Look for `"SSD1306 detected successfully"` in serial output
   - If `"SSD1306 not found"`, verify I2C wiring and address (0x3C)

2. **Test display without CAN:**
   - Temporarily add test rendering code in `display_ui_render_main_screen()`
   - Verify display update task is running at 10 Hz

3. **Font rendering issues:**
   - Font glyphs defined in [main/display_driver.c](main/display_driver.c) lines 44-140
   - Only supports ASCII 32-126 (printable characters)

### Performance Tuning

- **Display update rate:** [main/display_ui.h](main/display_ui.h) line 17 (`DISPLAY_UPDATE_RATE_HZ`)
- **CAN RX task priority:** [main/can_receiver.h](main/can_receiver.h) line 22 (default: 5)
- **Display UI task priority:** [main/display_ui.h](main/display_ui.h) line 16 (default: 4)
- **Stack sizes:** Adjust if seeing stack overflow warnings

---

## Recent Changes

### 2025-10-05: Added CO2 Display
- **Modified:** [main/display_ui.c](main/display_ui.c) lines 87-92
- **Change:** Added CO2 equivalent value (ppm) from BME680 IAQ message to display layout
- **Location:** Right side of IAQ line, format: `CO2:450`
- **Data source:** `g_sensor_data.co2_equiv` (already parsed from CAN ID 0x102)

---

## Testing Notes

### Expected Serial Output on Startup

```
I (123) MAIN: ===========================================
I (124) MAIN: ESP32-C3 CAN Display Client
I (125) MAIN: ===========================================
I (126) MAIN: Step 1/4: Initializing sensor data structure
I (127) MAIN: Sensor data structure initialized
I (128) MAIN: Step 2/4: Initializing CAN receiver
I (129) CAN_RX: Initializing TWAI (CAN) driver
I (130) CAN_RX: TX: GPIO4, RX: GPIO5, Bitrate: 500000 bps
I (145) CAN_RX: TWAI driver started successfully
I (146) MAIN: Step 3/4: Initializing display
I (147) DISPLAY: Initializing SSD1306 display
I (148) DISPLAY: I2C SDA: GPIO6, SCL: GPIO7, Addr: 0x3C
I (149) DISPLAY: Probing for SSD1306 at address 0x3C...
I (160) DISPLAY: SSD1306 detected successfully
I (165) DISPLAY: Display initialized successfully
I (166) MAIN: Step 4/4: Starting tasks
I (167) CAN_RX: CAN receive task created
I (168) DISPLAY_UI: Display update task started (rate: 10 Hz)
I (169) MAIN: ===========================================
I (170) MAIN: Initialization complete!
I (171) MAIN: Waiting for CAN messages...
```

### Normal Operation Logs

```
D (5234) CAN_RX: VEML7700: Lux=1234, Status=0, Seq=5
D (8345) CAN_RX: BME680 ENV: T=23.45°C, H=65%, P=1013.2hPa
D (8456) CAN_RX: BME680 IAQ: IAQ=120, Acc=3, CO2=450ppm, VOC=0ppm
D (9567) CAN_RX: MQ3: ADC=1234, Rs/R0=0.850, PPM=50
D (9678) CAN_RX: LD2410: Present=1, Dist=75cm, Move=45, Static=30
```

---

## Known Limitations

1. **Font Support:** Only basic 5×7 ASCII font, no international characters
2. **Display Screens:** Presence detail screen implemented but not accessible (no button input yet)
3. **Error Handling:** CAN driver errors logged but not exposed on display
4. **Calibration:** No runtime calibration for sensors, values pass-through from sender
5. **Storage:** No data logging to SD card or flash

---

## Future Enhancement Ideas

- **Automatic brightness:** Adjust display contrast based on ambient lux reading
- **Button input:** GPIO button to cycle between display screens
- **Alert animations:** Blinking indicators for high alcohol or poor air quality
- **Data logging:** Write sensor readings to SD card with timestamps
- **WiFi connectivity:** MQTT publishing of sensor data to broker
- **u8g2 library:** Replace custom font with u8g2 for better typography
- **Configuration UI:** Menu system for adjusting thresholds and display options

---

## References

- **ESP-IDF TWAI Driver:** https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/twai.html
- **SSD1306 Datasheet:** https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
- **CAN Bus Basics:** ISO 11898-1 standard
- **SN65HVD230 Transceiver:** https://www.ti.com/product/SN65HVD230

---

## Tips for AI Assistants

1. **Thread Safety:** Always use `sensor_data_lock()` / `sensor_data_unlock()` when accessing `g_sensor_data`
2. **Packed Structures:** CAN message structures use `__attribute__((packed))` to prevent padding
3. **Endianness:** All multi-byte CAN fields are little-endian (ESP32-C3 is also little-endian, so direct casting works)
4. **Display Coordinates:** Remember Y increases downward; character width is 6px (FONT_SMALL), 12px (FONT_LARGE)
5. **Update Timestamps:** Don't forget to update `g_sensor_data.last_update_ms[SENSOR_IDX_*]` when parsing new messages
6. **Error Propagation:** Check ESP-IDF function return values (`esp_err_t`) and log errors appropriately

---

**This file was last updated:** 2025-10-05
**Project Repository:** https://github.com/yourusername/Esp32-CAN-Disp-Client
