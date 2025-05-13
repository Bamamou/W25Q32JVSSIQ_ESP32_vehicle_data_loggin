# ESP32 Flash Memory Writer/Reader

This project implements a flash memory testing and data logging system using an ESP32 microcontroller and external SPI flash memory. The system provides continuous writing capabilities with real-time monitoring and control through both serial and Bluetooth interfaces.

## Hardware Requirements

- ESP32 Development Board
- SPI Flash Memory Chip (32Mbit/4MB)
- SPI Connection Setup:
  - CS: GPIO 5
  - MISO: GPIO 19
  - CLK: GPIO 18
  - MOSI: GPIO 23

## Features

### Features

#### Storage Management
- 4MB total storage capacity (32Mbit)
- 256-byte page size for efficient writing
- 4KB sector size for memory management
- Automatic sector erasing before writing
- Circular buffer implementation for continuous writing
- Built-in chip erase functionality

#### Data Writing
- Continuous automated writing with address tracking
- Page-level writing with address stamping
- Automatic sector management
- Configurable write timing

#### Communication Interfaces

##### Serial Interface
- Baud Rate: 115200
- Debug output and status messages

##### Bluetooth Interface
- Device Name: "ESP32_Flash_Writer"
- Interactive Command System:
  - `dump`: Show last 5 pages of written data
  - `dumpall`: Dump entire flash memory content
  - `dumpadd [address] [length]`: Dump specific memory range
  - `status`: Show current write position and status
  - `erase`: Perform full chip erase
  - `help`: Display available commands

### Real-time Operation
- Utilizes FreeRTOS for concurrent operations
- Separate tasks for:
  - Writing data (Core 0)
  - Bluetooth command handling (Core 1)
- Mutex-protected SPI access for thread safety
- Pausable write operations for data reading

## Data Format
The system writes test data in pages with the following format:
- Each page is 256 bytes
- Address information is embedded every 4 bytes
- Data is viewable in both hex and ASCII formats via Bluetooth interface
- Sequential pattern for data verification

## Operation Modes

### Continuous Write Mode
- Automatically writes test data every 500ms
- Implements circular buffer for continuous writing
- Automatically manages sector erasure
- Embeds address information in data for verification

### Interactive Mode (via Bluetooth)
- Real-time monitoring of write operations
- On-demand memory dumps with hex and ASCII view
- Status reporting and chip management
- Pausable write operations for safe reading

## Building and Flashing

This project is built using PlatformIO with the ESP32 platform. To build and flash:

1. Ensure PlatformIO is installed
2. Connect your ESP32 board
3. Build and upload the project using PlatformIO CLI or IDE

## Usage

1. Power up the system
2. Connect via Bluetooth (device name: "ESP32_Flash_Writer")
3. Available commands:
   - `dump`: View last 5 pages of written data
   - `dumpall`: View entire flash memory content
   - `dumpadd 0xADDRESS LENGTH`: View specific memory range
   - `status`: Check current write position and status
   - `erase`: Perform complete chip erase
   - `help`: Show all available commands

## Implementation Notes

- FreeRTOS tasks handle concurrent operations
- Mutex-protected SPI access ensures thread safety
- Automatic sector management for reliable writing
- Built-in data verification through address embedding
- Comprehensive error checking and status reporting
- Human-readable output format with hex and ASCII views
