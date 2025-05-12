#include <Arduino.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

BluetoothSerial SerialBT;

// SPI pins for W25Q32JVSSIQ
const uint8_t pinHSPI_CS_Flash = 26;
const uint8_t pinHSPI_MISO = 12;
const uint8_t pinHSPI_CLK = 14;
const uint8_t pinHSPI_MOSI = 13;

// Flash memory commands
#define CMD_WRITE_ENABLE      0x06
#define CMD_WRITE_DISABLE     0x04
#define CMD_READ_STATUS       0x05
#define CMD_WRITE_STATUS      0x01
#define CMD_READ_DATA         0x03
#define CMD_PAGE_PROGRAM      0x02
#define CMD_ERASE_SECTOR      0x20
#define CMD_ERASE_BLOCK_32K   0x52
#define CMD_ERASE_BLOCK_64K   0xD8
#define CMD_ERASE_CHIP        0xC7
#define CMD_READ_ID           0x90

// Flash memory specifications
#define FLASH_PAGE_SIZE       256     // 256 bytes per page
#define FLASH_SECTOR_SIZE     4096    // 4KB sector size
#define FLASH_BLOCK_SIZE      65536   // 64KB block size
#define FLASH_CHIP_SIZE       4194304 // 32Mbit = 4MB
#define MAXPAGESIZE           256     // Max data size for one entry

// Vehicle data structure
struct VehicleData {
  float odometerKm;
  float tripKm;
  float speedKmh;
  bool isInReverseMode;
  uint8_t ridingMode;
  float busCurrent;
  float bmsCurrent;
  uint8_t vehicleStatusByte1;
  uint8_t vehicleStatusByte2;
  float throttle;
  float controllerTemperature;
  float motorTemperature;
  float bmsVoltage;
  float bmsCellHighestVoltageValue;
  float bmsCellLowestVoltageValue;
  uint8_t soc;
  uint16_t rpm;
  float boardSupplyVoltage;
  float chargerVoltage;
  float chargerCurrent;
  uint8_t numActiveErrors;
  uint16_t sumActiveErrors;
  bool headlightHighBeam;
  bool turnLeftSwitch;
  bool turnRightSwitch;
  bool modeButton;
  bool kickstand;
  bool killswitch;
  bool key;
  bool breakSwitch;
};

// FreeRTOS handles
TaskHandle_t writeTaskHandle;
TaskHandle_t readTaskHandle;
TaskHandle_t btDumpTaskHandle;
SemaphoreHandle_t spiMutex;

// Operation modes
volatile bool operationMode = true;      // true = write, false = read
volatile bool btReadingAllData = false;  // Flag for reading all data over BT

// Track write position for ring buffer implementation
volatile uint32_t currentWriteAddress = 0;
volatile uint32_t totalEntriesWritten = 0;
volatile uint32_t lastSectorErased = 0xFFFFFFFF;

// Flash memory functions
void setupFlash() {
  pinMode(pinHSPI_CS_Flash, OUTPUT);
  digitalWrite(pinHSPI_CS_Flash, HIGH); // Deselect the flash chip
  SPI.begin(pinHSPI_CLK, pinHSPI_MISO, pinHSPI_MOSI, pinHSPI_CS_Flash);
  SPI.setFrequency(10000000); // Set SPI clock to 10MHz
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
}

// Wait for flash to finish processing
void waitForFlashReady() {
  digitalWrite(pinHSPI_CS_Flash, LOW);
  SPI.transfer(CMD_READ_STATUS);
  while (SPI.transfer(0) & 0x01) {
    // Wait until BUSY flag is cleared
  }
  digitalWrite(pinHSPI_CS_Flash, HIGH);
}

// Enable writing to flash
void writeEnable() {
  digitalWrite(pinHSPI_CS_Flash, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(pinHSPI_CS_Flash, HIGH);
}

// Read flash ID
uint32_t readFlashID() {
  uint32_t id = 0;
  digitalWrite(pinHSPI_CS_Flash, LOW);
  SPI.transfer(CMD_READ_ID);
  SPI.transfer(0x00); // Dummy byte
  SPI.transfer(0x00); // Dummy byte
  id |= SPI.transfer(0) << 16;
  id |= SPI.transfer(0) << 8;
  id |= SPI.transfer(0);
  digitalWrite(pinHSPI_CS_Flash, HIGH);
  return id;
}

// Erase a sector (4KB)
void eraseSector(uint32_t sectorAddr) {
  // Align the address to sector boundary
  sectorAddr = sectorAddr & ~(FLASH_SECTOR_SIZE - 1);
  
  writeEnable();
  digitalWrite(pinHSPI_CS_Flash, LOW);
  SPI.transfer(CMD_ERASE_SECTOR);
  SPI.transfer((sectorAddr >> 16) & 0xFF);
  SPI.transfer((sectorAddr >> 8) & 0xFF);
  SPI.transfer(sectorAddr & 0xFF);
  digitalWrite(pinHSPI_CS_Flash, HIGH);
  waitForFlashReady();
  
  lastSectorErased = sectorAddr;
}

// Write a page (up to 256 bytes)
void writePage(uint32_t addr, uint8_t* data, uint16_t length) {
  if (length > FLASH_PAGE_SIZE) {
    length = FLASH_PAGE_SIZE;
  }
  
  writeEnable();
  digitalWrite(pinHSPI_CS_Flash, LOW);
  SPI.transfer(CMD_PAGE_PROGRAM);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  
  for (uint16_t i = 0; i < length; i++) {
    SPI.transfer(data[i]);
  }
  
  digitalWrite(pinHSPI_CS_Flash, HIGH);
  waitForFlashReady();
}

// Read data from flash
void readData(uint32_t addr, uint8_t* data, uint16_t length) {
  digitalWrite(pinHSPI_CS_Flash, LOW);
  SPI.transfer(CMD_READ_DATA);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  
  for (uint16_t i = 0; i < length; i++) {
    data[i] = SPI.transfer(0);
  }
  
  digitalWrite(pinHSPI_CS_Flash, HIGH);
}

// Generate random vehicle data
VehicleData generateRandomVehicleData() {
  VehicleData data;
  
  data.odometerKm = random(0, 100000) / 10.0;
  data.tripKm = random(0, 10000) / 10.0;
  data.speedKmh = random(0, 1200) / 10.0;
  data.isInReverseMode = random(0, 2);
  data.ridingMode = random(0, 4);
  data.busCurrent = random(-200, 1000) / 10.0;
  data.bmsCurrent = random(-200, 1000) / 10.0;
  data.vehicleStatusByte1 = random(0, 256);
  data.vehicleStatusByte2 = random(0, 256);
  data.throttle = random(0, 1000) / 10.0;
  data.controllerTemperature = random(200, 850) / 10.0 - 20.0; // -20 to 65C
  data.motorTemperature = random(200, 1000) / 10.0 - 20.0;     // -20 to 80C
  data.bmsVoltage = random(480, 600) / 10.0;                   // 48V to 60V
  data.bmsCellHighestVoltageValue = random(38, 42) / 10.0;     // 3.8V to 4.2V
  data.bmsCellLowestVoltageValue = random(30, 42) / 10.0;      // 3.0V to 4.2V
  data.soc = random(0, 101);                                   // 0-100%
  data.rpm = random(0, 8000);
  data.boardSupplyVoltage = random(110, 140) / 10.0;           // 11.0V to 14.0V
  data.chargerVoltage = random(0, 630) / 10.0;                 // 0 to 63V
  data.chargerCurrent = random(0, 100) / 10.0;                 // 0 to 10A
  data.numActiveErrors = random(0, 10);
  data.sumActiveErrors = random(0, 65535);
  data.headlightHighBeam = random(0, 2);
  data.turnLeftSwitch = random(0, 2);
  data.turnRightSwitch = random(0, 2);
  data.modeButton = random(0, 2);
  data.kickstand = random(0, 2);
  data.killswitch = random(0, 2);
  data.key = random(0, 2);
  data.breakSwitch = random(0, 2);
  
  return data;
}

// Convert vehicle data to CSV string
String vehicleDataToCSV(const VehicleData& data) {
  String datalog = ";";
  datalog.concat(data.odometerKm);
  datalog.concat(";");
  datalog.concat(data.tripKm);
  datalog.concat(";");
  datalog.concat(data.speedKmh);
  datalog.concat(";");
  datalog.concat(data.isInReverseMode);
  datalog.concat(";");
  datalog.concat(data.ridingMode);
  datalog.concat(";");
  datalog.concat(data.busCurrent);
  datalog.concat(";");
  datalog.concat(data.bmsCurrent);
  datalog.concat(";");
  datalog.concat(data.vehicleStatusByte1);
  datalog.concat(";");
  datalog.concat(data.vehicleStatusByte2);
  datalog.concat(";");
  datalog.concat(data.throttle);
  datalog.concat(";");
  datalog.concat(data.controllerTemperature);
  datalog.concat(";");
  datalog.concat(data.motorTemperature);
  datalog.concat(";");
  datalog.concat(data.bmsVoltage);
  datalog.concat(";");
  datalog.concat(data.bmsCellHighestVoltageValue);
  datalog.concat(";");
  datalog.concat(data.bmsCellLowestVoltageValue);
  datalog.concat(";");
  datalog.concat(data.soc);
  datalog.concat(";");
  datalog.concat(data.rpm);
  datalog.concat(";");
  datalog.concat(data.boardSupplyVoltage);
  datalog.concat(";");
  datalog.concat(data.chargerVoltage);
  datalog.concat(";");
  datalog.concat(data.chargerCurrent);
  datalog.concat(";");
  datalog.concat(data.numActiveErrors);
  datalog.concat(";");
  datalog.concat(data.sumActiveErrors);
  datalog.concat(";");
  datalog.concat(data.headlightHighBeam);
  datalog.concat(";");
  datalog.concat(data.turnLeftSwitch);
  datalog.concat(";");
  datalog.concat(data.turnRightSwitch);
  datalog.concat(";");
  datalog.concat(data.modeButton);
  datalog.concat(";");
  datalog.concat(data.kickstand);
  datalog.concat(";");
  datalog.concat(data.killswitch);
  datalog.concat(";");
  datalog.concat(data.key);
  datalog.concat(";");
  datalog.concat(data.breakSwitch);
  datalog.concat(";");
  
  // Pad the string to fill the page
  for (int i = datalog.length(); i < MAXPAGESIZE-1; i++) {
    datalog.concat(".");
  }
  
  return datalog;
}

// Parse CSV data string back to readable format
String parseCSVData(const String& csvData) {
  String result = "";
  int fieldIndex = 0;
  int startPos = 0;
  int endPos = 0;
  
  // Define field names for better readability
  const char* fieldNames[] = {
    "OdometerKm", "TripKm", "SpeedKmh", "InReverse", "RidingMode",
    "BusCurrent", "BMSCurrent", "StatusByte1", "StatusByte2", "Throttle",
    "ControllerTemp", "MotorTemp", "BMSVoltage", "HighestCellV", "LowestCellV",
    "SOC", "RPM", "BoardVoltage", "ChargerV", "ChargerCurrent",
    "ActiveErrors", "SumActiveErrors", "HighBeam", "TurnLeft", "TurnRight",
    "ModeButton", "Kickstand", "Killswitch", "Key", "BreakSwitch"
  };
  
  // Skip initial semicolon
  startPos = 1;
  
  // Parse each field
  while ((endPos = csvData.indexOf(';', startPos)) != -1 && fieldIndex < 30) {
    String value = csvData.substring(startPos, endPos);
    if (value.length() > 0 && value[0] != '.') {
      result += fieldNames[fieldIndex];
      result += ": ";
      result += value;
      result += "\n";
    }
    startPos = endPos + 1;
    fieldIndex++;
  }
  
  return result;
}

// Task to write data to flash
void writeTask(void *parameter) {
  char bufferWrite[MAXPAGESIZE];
  
  while (true) {
    if (operationMode && !btReadingAllData) {  // Write mode and not reading BT data
      if (xSemaphoreTake(spiMutex, portMAX_DELAY)) {
        // Check if we need to erase a sector before writing
        uint32_t currentSector = currentWriteAddress & ~(FLASH_SECTOR_SIZE - 1);
        if (currentSector != lastSectorErased) {
          Serial.printf("Erasing sector at address 0x%06X\n", currentSector);
          SerialBT.printf("Erasing sector at address 0x%06X\n", currentSector);
          eraseSector(currentSector);
        }
        
        // Generate random vehicle data
        VehicleData vehicleData = generateRandomVehicleData();
        
        // Convert to CSV format
        String datalog = vehicleDataToCSV(vehicleData);
        datalog.toCharArray(bufferWrite, datalog.length() + 1);
        
        // Write the data
        Serial.printf("Writing to address 0x%06X (Entry #%lu)\n", currentWriteAddress, totalEntriesWritten + 1);
        writePage(currentWriteAddress, (uint8_t*)bufferWrite, MAXPAGESIZE);
        
        // Update address for next write (ring buffer style)
        currentWriteAddress = (currentWriteAddress + FLASH_PAGE_SIZE) % FLASH_CHIP_SIZE;
        totalEntriesWritten++;
        
        xSemaphoreGive(spiMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(500));  // Write every 500ms
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));  // Check flag every 100ms
    }
  }
}

// Task to read data from flash
void readTask(void *parameter) {
  char bufferRead[MAXPAGESIZE];
  uint32_t readAddress = 0;

  while (true) {
    if (!operationMode && !btReadingAllData) {  // Read mode and not BT reading
      if (xSemaphoreTake(spiMutex, portMAX_DELAY)) {
        // Read the data
        memset(bufferRead, 0, MAXPAGESIZE);
        readData(readAddress, (uint8_t*)bufferRead, MAXPAGESIZE);
        
        // Convert to readable format and print
        String csvData = String(bufferRead);
        String parsedData = parseCSVData(csvData);
        
        Serial.printf("Data from address 0x%06X:\n", readAddress);
        Serial.println(parsedData);
        
        // Update address for next read
        readAddress = (readAddress + FLASH_PAGE_SIZE) % FLASH_CHIP_SIZE;
        
        xSemaphoreGive(spiMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(2000));  // Read every 2 seconds
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));  // Check flag every 100ms
    }
  }
}

// Task to read all data from flash and send over BT
void btDumpTask(void *parameter) {
  char bufferRead[MAXPAGESIZE];
  uint32_t address = 0;
  uint32_t entriesRead = 0;
  uint32_t maxEntries = totalEntriesWritten > 0 ? min(static_cast<int>(totalEntriesWritten), FLASH_CHIP_SIZE / FLASH_PAGE_SIZE) : 
    FLASH_CHIP_SIZE / FLASH_PAGE_SIZE;
  
  SerialBT.println("\n==== STARTING VEHICLE DATA DUMP ====");
  SerialBT.printf("Total entries written: %lu\n", totalEntriesWritten);
  SerialBT.println("Reading all available data...\n");
  
  // If we've wrapped around, start reading from the oldest data
  if (totalEntriesWritten > FLASH_CHIP_SIZE / FLASH_PAGE_SIZE) {
    address = currentWriteAddress;
  }
  
  while (entriesRead < maxEntries && btReadingAllData) {
    if (xSemaphoreTake(spiMutex, portMAX_DELAY)) {
      // Read the data
      memset(bufferRead, 0, MAXPAGESIZE);
      readData(address, (uint8_t*)bufferRead, MAXPAGESIZE);
      
      // Convert to readable format and print over BT
      String csvData = String(bufferRead);
      if (csvData.length() > 0 && csvData[0] == ';') {
        String parsedData = parseCSVData(csvData);
        
        SerialBT.printf("Entry #%lu (Addr: 0x%06X):\n", entriesRead + 1, address);
        SerialBT.println(parsedData);
        SerialBT.println("----------------------------");
      }
      
      // Update address for next read (ring buffer style)
      address = (address + FLASH_PAGE_SIZE) % FLASH_CHIP_SIZE;
      entriesRead++;
      
      xSemaphoreGive(spiMutex);
    }
    
    // Check for BT commands to abort the dump
    if (SerialBT.available() > 0) {
      if (SerialBT.read() == '0') {
        SerialBT.println("Aborting dump operation!");
        break;
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));  // Small delay between reads
  }
  
  SerialBT.println("==== VEHICLE DATA DUMP COMPLETE ====");
  SerialBT.printf("Total entries read: %lu\n", entriesRead);
  SerialBT.printf("Resuming write operations at address 0x%06X\n", currentWriteAddress);
  
  btReadingAllData = false;
  vTaskDelete(NULL);  // Delete this task
}

void processSerialCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '1') {
      operationMode = false;  // Switch to read mode
      Serial.println("Mode changed to READ");
    } else if (cmd == '0') {
      operationMode = true;   // Switch to write mode
      Serial.println("Mode changed to WRITE");
    }
  }
}

void processBTCommand() {
  if (SerialBT.available() > 0) {
    char cmd = SerialBT.read();
    
    if (cmd == '1' && !btReadingAllData) {
      SerialBT.println("Received command to read all vehicle data");
      // Stop writing operations
      bool prevMode = operationMode;
      operationMode = false;
      btReadingAllData = true;
      
      // Create a task to read and print all data
      xTaskCreatePinnedToCore(
        btDumpTask,         // Task function
        "btDumpTask",       // Name
        8192,               // Stack size (bytes) - larger stack for BT operations
        NULL,               // Parameters
        2,                  // Priority (higher than read/write)
        &btDumpTaskHandle,  // Task handle
        0                   // Core
      );
      
      // After BT dump completes, mode will revert to previous state
      vTaskDelay(pdMS_TO_TICKS(50));  // Small delay to allow task to start
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Give time for serial to initialize
  
  // Initialize random seed
  randomSeed(analogRead(0));
  
  // Initialize Bluetooth Serial
  SerialBT.begin("ESP32_Vehicle_Logger");  // Bluetooth device name
  delay(1000);  // Give BT time to initialize
  
  Serial.println("\nESP32 Vehicle Data Logger with W25Q32JVSSIQ Flash");
  SerialBT.println("\nESP32 Vehicle Data Logger with W25Q32JVSSIQ Flash");
  
  // Initialize SPI for flash
  setupFlash();
  
  // Read and print the flash ID
  uint32_t flashID = readFlashID();
  Serial.printf("Flash ID: 0x%06X\n", flashID);
  SerialBT.printf("Flash ID: 0x%06X\n", flashID);
  
  // Create mutex for SPI access
  spiMutex = xSemaphoreCreateMutex();
  
  // Create tasks
  xTaskCreatePinnedToCore(
    writeTask,        // Task function
    "writeTask",      // Name
    8192,             // Stack size (bytes)
    NULL,             // Parameters
    1,                // Priority
    &writeTaskHandle, // Task handle
    0                 // Core
  );
  
  xTaskCreatePinnedToCore(
    readTask,         // Task function
    "readTask",       // Name
    8192,             // Stack size (bytes)
    NULL,             // Parameters
    1,                // Priority
    &readTaskHandle,  // Task handle
    1                 // Core
  );
  
  Serial.println("Vehicle data logging started");
  Serial.println("Send '0' to start writing, '1' to start reading");
  SerialBT.println("Vehicle data logging started");
  SerialBT.println("Send '1' over BT to dump all logged vehicle data");
  SerialBT.println("Send '0' over BT to abort dump operation");
}

void loop() {
  processSerialCommand();
  processBTCommand();
  delay(50); // Small delay to reduce CPU usage
}