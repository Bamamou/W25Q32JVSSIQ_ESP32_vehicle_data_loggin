# ESP32 Vehicle Data Logger

This project implements a vehicle data logging system using an ESP32 microcontroller and a W25Q32JVSSIQ Flash memory chip. The system captures comprehensive vehicle telemetry data and stores it in external flash memory, with both serial and Bluetooth interfaces for control and data retrieval.

## Hardware Requirements

- ESP32 Development Board
- W25Q32JVSSIQ Flash Memory Chip (32Mbit/4MB)
- SPI Connection Setup:
  - CS: GPIO 26
  - MISO: GPIO 12
  - CLK: GPIO 14
  - MOSI: GPIO 13

## Features

### Data Logging
- Captures comprehensive vehicle telemetry including:
  - Odometer and trip readings
  - Speed and RPM
  - Battery management system (BMS) data
  - Temperature readings (controller and motor)
  - Vehicle status indicators
  - Switch states (kickstand, killswitch, etc.)
  - Voltage and current measurements
  - Error states

### Storage Management
- Implements a ring buffer system in flash memory
- 4MB total storage capacity
- 256-byte pages for individual data entries
- 4KB sector size for efficient memory management
- Automatic sector erasing before writing new data

### Communication Interfaces

#### Serial Interface
- Baud Rate: 115200
- Commands:
  - '0': Switch to write mode
  - '1': Switch to read mode

#### Bluetooth Interface
- Device Name: "ESP32_Vehicle_Logger"
- Commands:
  - '1': Dump all logged vehicle data
  - '0': Abort dump operation

### Real-time Operation
- Utilizes FreeRTOS for concurrent operations
- Separate tasks for:
  - Writing data (Core 0)
  - Reading data (Core 1)
  - Bluetooth data dumping
- Mutex-protected SPI access for thread safety

## Data Structure
The system logs the following vehicle parameters:
```
- odometerKm                    (float)
- tripKm                        (float)
- speedKmh                      (float)
- isInReverseMode              (bool)
- ridingMode                    (uint8_t)
- busCurrent                    (float)
- bmsCurrent                    (float)
- vehicleStatusBytes           (2x uint8_t)
- throttle                      (float)
- controllerTemperature        (float)
- motorTemperature             (float)
- bmsVoltage                   (float)
- bmsCellVoltages             (highest/lowest, float)
- soc                          (State of Charge, uint8_t)
- rpm                          (uint16_t)
- boardSupplyVoltage           (float)
- chargerVoltage               (float)
- chargerCurrent               (float)
- errorStates                  (count and sum)
- Various switch states        (bool)
```

## Operation Modes

### Write Mode
- Automatically writes vehicle data every 500ms
- Implements circular buffer for continuous logging
- Automatically manages sector erasure

### Read Mode
- Reads stored data every 2 seconds
- Displays formatted data via Serial interface

### Bluetooth Dump Mode
- Reads and transmits all stored data
- Can be interrupted with abort command
- Provides progress updates and entry counts

## Building and Flashing

This project is built using PlatformIO with the ESP32 platform. To build and flash:

1. Ensure PlatformIO is installed
2. Connect your ESP32 board
3. Build and upload the project using PlatformIO CLI or IDE

## Usage

1. Power up the system
2. Connect via Serial Monitor (115200 baud) or Bluetooth
3. Use command interface to control operation:
   - Serial: '0'/'1' for write/read modes
   - Bluetooth: '1' to dump all data, '0' to abort dump

## Implementation Notes

- The system uses FreeRTOS tasks for concurrent operation
- SPI access is protected by mutex for thread safety
- Flash memory operations include proper error checking
- Data is stored in CSV format for easy parsing
- Bluetooth interface provides human-readable formatted output
