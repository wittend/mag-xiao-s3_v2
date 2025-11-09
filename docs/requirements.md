# Magnetometer Firmware Requirements (mag-xiao-s3)

## 1. Overview
- Problem statement:
    * Create firmware for a Seeed Studio XIAO ESP32S3 module to act as an IoT edge data collector for a small number of sensors. 
    * Its function will be to interrogate these sensors and relay the data to an aggregating network for relay to a central host.
    * It must use a low bandwidth radio frequency protocol such as LoRa for this transfer.
    * It must provide power management to maintain sufficient charge in battery storage (using LiPO, LiFePO, NaPO, SLA, etc.) from a small solar panel.
    * It must support a small local GNSS receiver to maintain local time and provide PPS triggering of transmissions. 
   
## 2. Hardware
- Board: Seeed Studio XIAO ESP32S3 
- Sensors/ICs: 
    * PNI RM3100cb (magnetometer)
    * Microchip MCP9804/MCP9808 temperature sensor.
    * Seeed Studio L76K GPS/GNSS module for XIAO.
    * Seeed Studio Wio-SX1262 for XIAO (uses Semtech SX1262 module)
    
- Power: (battery/USB), voltage(s)
    * Uing LiPO, LiFePO, NaPO, SLA, etc. battery.
    * 3.3 v operation.
    * Solar panel and conditioning  as required.
    
- Wiring: diagram/photo + table (pin → signal)
  
- Buses: I2C/UART; target clock rates; pull-ups
    * I2C bus default timing.
    * UART (if required for GNSS) 115200 bps.
     
- Notes: level shifting, analog refs, shielding
    * not at present.

## 3. Functional Requirements
- REQ-001 [MUST]: sample sensors once per second and send data samples from RM3100, MCP9804/8 within that interval.

- REQ-002 [MUST]:  Use GNSS PPS signal to trigger sample transmission and acquisition cadence.
	
- REQ-003 [MUST]:  Data shall be sent as JSON objects.
	
- REQ-004 [SHOULD]: Check GNSS at startup and 1/hour to set local real time clock to maintain timing accuracy.
 

## 4. Non-Functional Requirements
- Performance: sample rate 1 Hz
- Accuracy: ±(TBD) (units), stability (TBD)
- Power: avg (TBD) mA, sleep (tbd) mA
- Memory: max RAM (TBD) KB, flash (TBD) KB
- Startup time: < (TBD) ms to ready
- Reliability: watchdog, brown-out behavior
- Advertised RM3100-cb performance:
	* Sensitivity	13nT
	* Noise	15nT
	* Linearity over +/- 200 uT	0.50%
	* Field Measurement Range	+/- 1100 uT
	* Current @ 8Hz, 3 Axes	260 uA
	* Operating Temperature	-40°C to +85°C

## 5. Interfaces & Protocols
Baseline schema
{ "ts": "DD Mon YYYY HH:MM:SS", "rt": <float> "x": <float>, "y": <float>, "z": <float> }
ts (string): UTC timestamp formatted like 25 Oct 2025 14:02:33 (RFC‑2822‑like time portion without timezone offset).

rt (number): temperature (degrees C) of remote node.
 
x, y, z (number): Field components in nanoTesla (nT), with 3 decimal places printed.

Example:

{ "ts":"26 Oct 2025 14:20:00", "rt": 25.000, "x":12345.678, "y":-234.500, "z":987.001 }

### 5.1 USB/Serial
- Baud: 115200 (or other)
- Message format (JSON/binary):
  - Example request:
    ```json
    {"cmd":"get_sample"}
    ```
  - Example response:
    ```json
    {"ts":"DD Mon YYYY HH:MM:SS", "rt":NN.NNN, "x": NNN.NNN, "y": NNN.NNN, "z": NNN.NNN }
    ```
- Timeouts, retries, error codes

### 5.2 Other interfaces (BLE or Wi‑Fi)

- A Wifi or BLE interface must be provided at initial start up to allow configuration of operational parameters, node identification, location (latitude, longitude, elevation) and owner, perhaps more.
 
- A button connected to a GPIO pin (TBD) allows the user to reset the device to configuration mode and causes it tp present the HTML page that allows entry of configuration information.


## 6. Data Processing & Algorithms

### Orientation and Axes

This explains how the device handles magnetometer orientation and how to configure 90° rotations around each axis.

### Axis conventions
- The RM3100 outputs three orthogonal components X, Y, Z.
- mag-usb treats these axes as a right‑handed coordinate system.
- Positive rotations follow the right‑hand rule (thumb points along the axis of rotation; curl of fingers is direction of positive rotation).

### Why orientation adjustments?
Depending on how the sensor PCB is mounted in your enclosure, its axes may not align with your desired reference frame. mag-usb can rotate the measured vector by multiples of 90° around the X, Y, and Z axes to align with your installation.

### Configuration
Set the following keys in `config.toml` under `[mag_orientation]`:
```
[mag_orientation]
mag_translate_x = <int>   # -180, -90, 0, 90, 180
mag_translate_y = <int>   # -180, -90, 0, 90, 180
mag_translate_z = <int>   # -180, -90, 0, 90, 180
```
Rules:
- Allowed values: `-180, -90, 0, 90, 180`. Any other value is treated as `0`.
- `180` and `-180` are treated the same (flip both perpendicular axes).
- Rotations are applied in this order: rotate around X, then around Y, then around Z.
- The rotations affect the other two components:
  - Rotate around X → affects Y and Z.
  - Rotate around Y → affects X and Z.
  - Rotate around Z → affects X and Y.

### Examples
Let the input vector be `(X, Y, Z)`.

- Rotate +90° about X:
  - `(X, Y, Z)` → `(X, -Z, Y)`
- Rotate -90° about X:
  - `(X, Y, Z)` → `(X, Z, -Y)`
- Rotate 180° about X:
  - `(X, Y, Z)` → `(X, -Y, -Z)`

- Magnetic Field Units: nanotesla (nT).


## 7. User Interaction
- HTML Form configuration interface:
    * node identification 
    * location (latitude, longitude, elevation) 
    * owner
    * orientation
    * LoRa signal parameters
    
- after initial boot or button reset the system presents the configuration HTML page to allow input of operating parameters.

## 8. Calibration & Test Procedures
- Procedure steps (with timings) (TBD)
- Acceptance thresholds (TBD)

## 9. Validation & Acceptance Criteria
- ACC-001 validates REQ-001: method, tool, threshold (TBD)
- ACC-002 validates REQ-___: … (TBD)

## 10. Operating Conditions
- Temperature range: 40 Degrees C to 45 degrees C (depending on battery subsystem) 
- Magnetic environment < 1 mT


## 11. Risks & Constraints
- Library dependencies (with versions)
- PlatformIO: version, espressif32 platform version (current)
- Arduino‑ESP32 core version (current)
- USB CDC on boot needed? (`ARDUINO_USB_MODE=1`, `ARDUINO_USB_CDC_ON_BOOT=1`) Yes

## 12. Deliverables
- Firmware binary (.pio/build/seeed_xiao_esp32s3/firmware.bin)
- Source code structure
- Test results : on-device
- User guide / quickstart

## 13. References
- data concerning reading RM3100 registers is contained in src/rm3100.h
- example arduino code for reading the RM3100 device: docs/RM3100_Arduino_I2C.ino.txt
- data concerning reading MCP9804/8 registers is contained in src/MCP9808.h
- MCP9808 Temperature Sensor datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/MCP9808-0.5C-Maximum-Accuracy-Digital-Temperature-Sensor-Data-Sheet-DS20005095B.pdf
- ESP32S3 XIAO Pinouts: https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/
- ESP32S3 WiFi information: https://wiki.seeedstudio.com/xiao_esp32s3_wifi_usage/
- ESP32S3 Bluetooth LE information: https://wiki.seeedstudio.com/xiao_esp32s3_bluetooth/
- LoRa Wio-sx1262 module: https://wiki.seeedstudio.com/wio_sx1262/
- Seeed L76k GNSS board for XAIO: https://wiki.seeedstudio.com/get_start_l76k_gnss/#hardware-overview

