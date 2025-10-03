# ESP32 CAN Display Client

ESP32 CAN bus client firmware that receives sensor data from a CAN network and displays it on a 128x64 SSD1306 OLED display.

## Features

- **Multi-sensor CAN data reception**: Receives and parses data from VEML7700 (light), BME680 (temp/humidity/pressure/IAQ), MQ-3 (alcohol), and LD2410 (presence) sensors
- **Real-time OLED display**: 128x64 SSD1306 I2C display with organized sensor data layout
- **Modular architecture**: Clean separation between CAN receiver, display driver, and UI rendering
- **Thread-safe data handling**: FreeRTOS tasks with mutex-protected shared data structure
- **Staleness detection**: Alerts when sensor data hasn't been received recently (>5 seconds)
- **CAN bus health monitoring**: Watchdog for CAN disconnection detection

## Hardware Requirements

- **ESP32-C3 SuperMini** (or any ESP32 board)
- **SSD1306 OLED Display** (128x64, I2C, address 0x3C)
- **SN65HVD230 CAN Transceiver** (or compatible 3.3v CAN transceiver)
- Power supply (3.3V for ESP32-C3, 5V for CAN transceiver if needed)

### Pin Connections

| Function | GPIO | Description |
|----------|------|-------------|
| CAN TX   | 4    | Connect to SN65HVD230 TX pin |
| CAN RX   | 5    | Connect to SN65HVD230 RX pin |
| I2C SDA  | 6    | Connect to SSD1306 SDA |
| I2C SCL  | 7    | Connect to SSD1306 SCL |

See [docs/WIRING.md](docs/WIRING.md) for detailed wiring diagrams.

## CAN Protocol

**CAN Bus Speed**: 500 kbps

The client receives the following CAN message IDs:

| ID    | Sensor       | Data                              | Update Rate |
|-------|--------------|-----------------------------------|-------------|
| 0x0A2 | VEML7700     | Lux value (3-byte, 0-120K), status | 1 Hz        |
| 0x0A3 | BME680       | Temperature, humidity, pressure   | 0.33 Hz     |
| 0x0A4 | BME680       | IAQ, CO2 equiv, VOC               | 0.33 Hz     |
| 0x0A5 | MQ-3         | Alcohol PPM, ADC, Rs/R0 ratio     | 1 Hz        |
| 0x0A6 | LD2410       | Presence, distance, energy levels | 10 Hz       |
| 0x0A7 | System       | Active sensors, heap, uptime      | 0.1 Hz      |

See [main/can_protocol.h](main/can_protocol.h) for complete message format definitions.

## Display Layout

```
┌──────────────────────────────┐
│ 12345 lux           (Large)  │  ← Light level
├──────────────────────────────┤
│ 23.5C   65%   1013hPa        │  ← Environmental data
├──────────────────────────────┤
│ IAQ:120 Good                 │  ← Air quality
│ Alc:50ppm                    │  ← Alcohol level
├──────────────────────────────┤
│ P:75cm              STALE    │  ← Presence / Status
└──────────────────────────────┘
```

- **Top**: Lux value (large font)
- **Middle**: Temperature, Humidity, Pressure (3 columns)
- **Bottom**: IAQ with quality indicator, Alcohol level
- **Status**: Presence detection, staleness warning

## Software Architecture

```
main.c
  ├── sensor_data.h         (Shared data structure with mutex)
  ├── can_receiver task     (Receives & parses CAN messages)
  └── display_ui task       (Renders UI at 10 Hz)
```

### File Responsibilities

- **[main/main.c](main/main.c)**: Application entry point, task coordination, system monitoring
- **[main/can_receiver.c](main/can_receiver.c)**: TWAI driver initialization, CAN message reception and parsing
- **[main/display_driver.c](main/display_driver.c)**: SSD1306 I2C driver wrapper, low-level display functions
- **[main/display_ui.c](main/display_ui.c)**: UI layout and rendering logic
- **[main/can_protocol.h](main/can_protocol.h)**: CAN message ID and structure definitions
- **[main/sensor_data.h](main/sensor_data.h)**: Shared sensor data structure with thread-safe access

## Build Instructions

### Prerequisites

1. Install [ESP-IDF v5.x](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/)
2. Set up ESP-IDF environment:
   ```bash
   . $HOME/esp/esp-idf/export.sh  # Or your IDF path
   ```

### Build and Flash

```bash
# Clone the repository
git clone https://github.com/yourusername/Esp32-CAN-Disp-Client.git
cd Esp32-CAN-Disp-Client

# Configure the project (optional - defaults are set in sdkconfig.defaults)
idf.py menuconfig

# Build the firmware
idf.py build

# Flash to ESP32-C3
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor
```

**Tip**: Use `idf.py -p /dev/ttyUSB0 flash monitor` to flash and monitor in one command.

### Configuration

Default settings are in [sdkconfig.defaults](sdkconfig.defaults). You can customize:

- GPIO pin assignments in `can_receiver.h` and `display_driver.h`
- CAN bitrate (default: 500 kbps)
- Display update rate (default: 10 Hz)
- Staleness timeout (default: 5 seconds)

## Usage

1. Connect the hardware according to the wiring diagram
2. Flash the firmware to the ESP32-C3
3. Connect the CAN bus to your sensor network
4. The display will show sensor data as it's received
5. Monitor serial output for debugging and CAN message logs

### Expected Serial Output

```
I (123) MAIN: ESP32-C3 CAN Display Client
I (456) CAN_RX: Initializing TWAI (CAN) driver
I (789) DISPLAY: Initializing SSD1306 display
I (1012) MAIN: Initialization complete!
I (1234) CAN_RX: VEML7700: Lux=1234.0, Status=0
I (2345) CAN_RX: BME680 ENV: T=23.45°C, H=65%, P=1013.2hPa
```

## Troubleshooting

### No CAN messages received
- Check CAN TX/RX wiring to TJA1050
- Verify CAN bus termination resistors (120Ω at each end)
- Check CAN bitrate matches sender (500 kbps)
- Use logic analyzer or CAN bus analyzer to verify messages on bus

### Display not working
- Verify I2C connections (SDA/SCL)
- Check I2C address (default 0x3C, some displays use 0x3D)
- Test display with I2C scanner
- Check power supply to display (3.3V or 5V depending on module)

### Display shows "STALE"
- CAN messages not being received for >5 seconds
- Check CAN bus connection and sender status
- Verify sender is transmitting at expected rates

### Build errors
- Ensure ESP-IDF v5.x is installed and activated
- Run `idf.py fullclean` then rebuild
- Check that all source files are present

## Future Enhancements

- [ ] Auto-brightness based on lux sensor reading
- [ ] add sensor data logging in sdcard
- [ ] Button input to toggle between display screens
- [ ] Alert animations (blinking) for high alcohol detection
- [ ] WiFi configuration portal
- [ ] MQTT forwarding of sensor data
- [ ] Integrate u8g2 library for better font support

## License

See [LICENSE](LICENSE) file.

## Contributing

Pull requests are welcome! Please ensure:
- Code follows existing style and structure
- Comments explain CAN protocol and message formats
- Commit messages are descriptive

## References

- [ESP-IDF TWAI Driver Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/twai.html)
- [SSD1306 Datasheet](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf)
- [TJA1050 CAN Transceiver Datasheet](https://www.nxp.com/docs/en/data-sheet/TJA1050.pdf)
