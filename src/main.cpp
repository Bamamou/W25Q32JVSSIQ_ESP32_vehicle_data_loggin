#include <SPI.h>
#include <BluetoothSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

// Flash memory commands
#define WRITE_ENABLE    0x06
#define WRITE_DISABLE   0x04
#define WRITE_STATUS    0x01  // Add this line
#define CHIP_ERASE      0xC7
#define READ_STATUS     0x05
#define READ_DATA       0x03
#define PAGE_PROGRAM    0x02
#define SECTOR_ERASE    0x20
#define RELEASE_POWERDOWN  0xAB
#define READ_DEVICE_ID    0x90
#define READ_JEDEC_ID     0x9F

// Pin definitions
#define SPI_CS_PIN      26
#define SPI_MOSI_PIN    13
#define SPI_MISO_PIN    12
#define SPI_CLK_PIN     14

// Flash memory constants
#define FLASH_PAGE_SIZE 256
#define FLASH_SECTOR_SIZE 4096
#define FLASH_TOTAL_SIZE 4194304  // 32 Mbit = 4 MB

// Ring buffer constants
#define RING_BUFFER_START_ADDR 0x1000  // Start after metadata
#define RING_BUFFER_END_ADDR   (FLASH_TOTAL_SIZE - 0x1000)  // Leave some space at the end
#define METADATA_ADDR  0x000   // Store metadata (current write position) at beginning of flash
#define MAXPAGESIZE    FLASH_PAGE_SIZE // Maximum size for a data record

// FreeRTOS task handles and synchronization objects
TaskHandle_t writeTaskHandle;
TaskHandle_t btTaskHandle;
SemaphoreHandle_t flashMutex;

// BluetoothSerial instance
BluetoothSerial SerialBT;

// Global variables
uint32_t currentAddress = RING_BUFFER_START_ADDR;
bool pauseWriting = false;
uint8_t writeBuffer[FLASH_PAGE_SIZE];
char bufferWrite[FLASH_PAGE_SIZE];

// Structures to store vehicle data
struct VehicleInfo {
  bool isInReverseMode;
  uint8_t ridingMode;
};

struct mCUData {
  float busCurrent;
  uint8_t throttle;
  float controllerTemperature;
  float motorTemperature;
};

struct bMSData {
  float current;
  float voltage;
  uint8_t SOC;
};

struct InfoToSave {
  float odometerKm;
  float tripKm;
  float speedKmh;
  uint8_t vehicleStatuByte1;
  uint8_t vehicleStatuByte2;
  float BMSCellHighestVoltageValue;
  float BMSCellLowestVoltageValue;
  uint16_t rpm;
  float boardSupplyVoltage;
  float chargerVoltage;
  float chargerCurrent;
  uint8_t numActiveErrors;
  uint8_t sumActiveErrors;
};

struct Inputs {
  bool headlightHighBeam;
  bool turnLeftSwitch;
  bool turnRightSwitch;
  bool modeButton;
  bool kickstand;
  bool killswitch;
  bool key;
  bool breakSwitch;
};

// Sample data for testing
VehicleInfo vehicleInfo = {false, 1};
mCUData MCUData = {10.5, 25, 45.2, 50.3};
bMSData BMSData = {5.2, 48.3, 85};
InfoToSave infoToSave = {1234.5, 25.3, 35.0, 0x01, 0x02, 4.15, 4.05, 3500, 12.5, 54.2, 2.5, 0, 0};
Inputs inputs = {true, false, false, false, false, true, true, false};

// Function prototypes
void writeTask(void *parameter);
void btTask(void *parameter);
void flashWriteEnable();
void flashWaitForReady();
void flashEraseSector(uint32_t address);
void flashEraseChip();
void flashReadData(uint32_t address, uint8_t *buffer, int length);
void flashWritePage(uint32_t address, uint8_t *data, int length);
void dumpFlashData(uint32_t startAddress, uint32_t length);
void processCommand(String command);
void saveMetadata();
void loadMetadata();
void prepareVehicleDataForSaving();
bool isDataValid(uint8_t *buffer, int length);
void flashInit();

// Helper function to check if data is valid (not empty/erased)
bool isDataValid(uint8_t *buffer, int length) {
  // Check if the buffer starts with a semicolon (our data format)
  if (buffer[0] == ';') {
    return true;
  }
  
  // Check if the buffer is all 0xFF (erased flash)
  bool allFF = true;
  for (int i = 0; i < length && i < 16; i++) { // Check first 16 bytes
    if (buffer[i] != 0xFF) {
      allFF = false;
      break;
    }
  }
  
  return !allFF; // If not all 0xFF, it might contain valid data
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(1000);  // Give time for serial to initialize
  Serial.println("\nESP32 Flash Memory Writer");
  
  // Initialize SPI
  SPI.begin(SPI_CLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
  SPI.setFrequency(8000000); // 8 MHz
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  
  // Configure CS pin as output
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  
  // Initialize flash chip and verify communication
  flashInit();
  
  // Initialize Bluetooth Serial
  SerialBT.begin("ESP32_Flash_Writer");
  
  // Create mutex for flash access
  flashMutex = xSemaphoreCreateMutex();
  
  // Load the last write position from metadata
  loadMetadata();
  
  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(
    writeTask,           // Task function
    "WriteTask",         // Task name
    4096,                // Stack size in words
    NULL,                // Task parameters
    1,                   // Priority (0 to 24, higher is higher priority)
    &writeTaskHandle,    // Task handle
    0                    // Core (0 or 1)
  );
  
  xTaskCreatePinnedToCore(
    btTask,              // Task function
    "BTTask",            // Task name
    4096,                // Stack size in words
    NULL,                // Task parameters
    2,                   // Priority (higher than write task)
    &btTaskHandle,       // Task handle
    1                    // Core (0 or 1)
  );
  
  Serial.println("System initialized. Auto writing will begin...");
  Serial.printf("Current write position: 0x%06X\n", currentAddress);
}

void loop() {
  // Main loop is empty since we're using FreeRTOS tasks
  delay(1000);
}

// Load the current write position from flash memory metadata
void loadMetadata() {
  uint8_t buffer[8];
  
  // Read metadata from the beginning of flash
  flashReadData(METADATA_ADDR, buffer, 8);
  
  // Check if metadata has a valid signature ('RBUF')
  if (buffer[0] == 'R' && buffer[1] == 'B' && buffer[2] == 'U' && buffer[3] == 'F') {
    // Valid signature found, extract the current write position
    currentAddress = (buffer[4] << 24) | (buffer[5] << 16) | (buffer[6] << 8) | buffer[7];
    
    // If address is outside our ring buffer range, reset it
    if (currentAddress < RING_BUFFER_START_ADDR || currentAddress >= RING_BUFFER_END_ADDR) {
      currentAddress = RING_BUFFER_START_ADDR;
    }
    
    Serial.printf("Loaded current address from metadata: 0x%06X\n", currentAddress);
  } else {
    // No valid metadata found, start from the beginning of the ring buffer
    currentAddress = RING_BUFFER_START_ADDR;
    Serial.println("No valid metadata found, starting from beginning of ring buffer");
    
    // Save initial metadata
    saveMetadata();
  }
}

// Save the current write position to flash memory metadata
void saveMetadata() {
  uint8_t buffer[8];
  
  // Set signature ('RBUF')
  buffer[0] = 'R';
  buffer[1] = 'B'; 
  buffer[2] = 'U';
  buffer[3] = 'F';
  
  // Set current write position
  buffer[4] = (currentAddress >> 24) & 0xFF;
  buffer[5] = (currentAddress >> 16) & 0xFF;
  buffer[6] = (currentAddress >> 8) & 0xFF;
  buffer[7] = currentAddress & 0xFF;
  
  // Erase metadata sector if needed (this is only done when initializing)
  static bool metadataInitialized = false;
  if (!metadataInitialized) {
    flashEraseSector(0);
    metadataInitialized = true;
  }
  
  // Write the metadata
  flashWritePage(METADATA_ADDR, buffer, 8);
  Serial.printf("Saved metadata with address: 0x%06X\n", currentAddress);
}

// Prepare vehicle data for saving
void prepareVehicleDataForSaving() {
  // Create the concatenated data string
  String datalog = ";";
  datalog.concat(infoToSave.odometerKm); datalog.concat(";"); 
  datalog.concat(infoToSave.tripKm); datalog.concat(";"); 
  datalog.concat(infoToSave.speedKmh); datalog.concat(";"); 
  datalog.concat(vehicleInfo.isInReverseMode); datalog.concat(";"); 
  datalog.concat(vehicleInfo.ridingMode); datalog.concat(";"); 
  datalog.concat(MCUData.busCurrent); datalog.concat(";"); 
  datalog.concat(BMSData.current); datalog.concat(";"); 
  datalog.concat(infoToSave.vehicleStatuByte1); datalog.concat(";"); 
  datalog.concat(infoToSave.vehicleStatuByte2); datalog.concat(";"); 
  datalog.concat(MCUData.throttle); datalog.concat(";"); 
  datalog.concat(MCUData.controllerTemperature); datalog.concat(";"); 
  datalog.concat(MCUData.motorTemperature); datalog.concat(";"); 
  datalog.concat(BMSData.voltage); datalog.concat(";"); 
  datalog.concat(infoToSave.BMSCellHighestVoltageValue); datalog.concat(";"); 
  datalog.concat(infoToSave.BMSCellLowestVoltageValue); datalog.concat(";"); 
  datalog.concat(BMSData.SOC); datalog.concat(";"); 
  datalog.concat(infoToSave.rpm); datalog.concat(";"); 
  datalog.concat(infoToSave.boardSupplyVoltage); datalog.concat(";"); 
  datalog.concat(infoToSave.chargerVoltage); datalog.concat(";"); 
  datalog.concat(infoToSave.chargerCurrent); datalog.concat(";"); 
  datalog.concat(infoToSave.numActiveErrors); datalog.concat(";"); 
  datalog.concat(infoToSave.sumActiveErrors); datalog.concat(";"); 
  datalog.concat(inputs.headlightHighBeam); datalog.concat(";"); 
  datalog.concat(inputs.turnLeftSwitch); datalog.concat(";"); 
  datalog.concat(inputs.turnRightSwitch); datalog.concat(";"); 
  datalog.concat(inputs.modeButton); datalog.concat(";"); 
  datalog.concat(inputs.kickstand); datalog.concat(";"); 
  datalog.concat(inputs.killswitch); datalog.concat(";"); 
  datalog.concat(inputs.key); datalog.concat(";"); 
  datalog.concat(inputs.breakSwitch); datalog.concat(";");
  
  // Add timestamp (millis since boot)
  datalog.concat(millis()); datalog.concat(";");
  
  // Pad with dots to fill the page
  for(int i = datalog.length(); i < MAXPAGESIZE-1; i++) {
    datalog.concat(".");
  }
  
  // Debug: Print the data that's about to be written
  Serial.println("Data being prepared:");
  Serial.println(datalog);
  Serial.println("Data length: " + String(datalog.length()));
  
  // Convert to char array for writing
  datalog.toCharArray(bufferWrite, datalog.length() + 1);
  
  // Debug: Print first few bytes of the buffer
  Serial.println("First 32 bytes of write buffer in hex:");
  for(int i = 0; i < 32; i++) {
    Serial.printf("%02X ", (uint8_t)bufferWrite[i]);
  }
  Serial.println();
  
  // Copy data to write buffer
  memcpy(writeBuffer, bufferWrite, MAXPAGESIZE);
  
  // Debug: Print first few bytes of the final buffer
  Serial.println("First 32 bytes of final buffer in hex:");
  for(int i = 0; i < 32; i++) {
    Serial.printf("%02X ", writeBuffer[i]);
  }
  Serial.println();
}

// Task to continuously write data to flash memory
void writeTask(void *parameter) {
  // Wait a brief moment before starting to allow system to stabilize
  vTaskDelay(pdMS_TO_TICKS(2000));
  
  Serial.println("Write task started");
  
  while (true) {
    if (!pauseWriting) {
      if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Check if we need to wrap around in the ring buffer
        if (currentAddress >= RING_BUFFER_END_ADDR) {
          currentAddress = RING_BUFFER_START_ADDR;
          Serial.println("Reached end of ring buffer, wrapping to beginning");
        }
        
        // Check if we need to erase a sector (sectors are 4KB)
        if (currentAddress % FLASH_SECTOR_SIZE == 0) {
          Serial.printf("Erasing sector at address 0x%06X\n", currentAddress);
          flashEraseSector(currentAddress);
        }
        
        // Prepare vehicle data for saving
        prepareVehicleDataForSaving();
        
        // Write a page of data
        Serial.printf("Writing page at address 0x%06X\n", currentAddress);
        flashWritePage(currentAddress, writeBuffer, FLASH_PAGE_SIZE);
        
        // Verify the write by reading back
        uint8_t verifyBuffer[FLASH_PAGE_SIZE];
        flashReadData(currentAddress, verifyBuffer, FLASH_PAGE_SIZE);
        
        // Print first 32 bytes of verification
        Serial.println("First 32 bytes after write verification:");
        for(int i = 0; i < 32; i++) {
            Serial.printf("%02X ", verifyBuffer[i]);
        }
        Serial.println();
        
        // Compare write and verify buffers
        bool writeVerified = true;
        for(int i = 0; i < FLASH_PAGE_SIZE; i++) {
            if(writeBuffer[i] != verifyBuffer[i]) {
                Serial.printf("Mismatch at byte %d: wrote %02X, read %02X\n", 
                            i, writeBuffer[i], verifyBuffer[i]);
                writeVerified = false;
                break;
            }
        }
        
        if(writeVerified) {
            Serial.println("Write verified successfully");
        } else {
            Serial.println("Write verification failed!");
        }
        
        // Move to next page
        currentAddress += FLASH_PAGE_SIZE;
        
        // Save the current write position to metadata
        saveMetadata();
        
        xSemaphoreGive(flashMutex);
      }
      
      // Small delay between write operations - adjust as needed
      vTaskDelay(pdMS_TO_TICKS(2000));
    } else {
      // If writing is paused, just yield CPU
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

// Task to handle Bluetooth commands
void btTask(void *parameter) {
  String btCommand = "";
  
  while (true) {
    if (SerialBT.available()) {
      char c = SerialBT.read();
      
      if (c == '\n' || c == '\r') {
        if (btCommand.length() > 0) {
          Serial.printf("Received BT command: %s\n", btCommand.c_str());
          processCommand(btCommand);
          btCommand = "";
        }
      } else {
        btCommand += c;
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(20)); // Small delay to prevent CPU hogging
  }
}

// Process Bluetooth commands
void processCommand(String command) {
  command.trim();
  command.toLowerCase();
  
  if (command == "dump") {
    // Pause writing and dump a portion of flash memory
    pauseWriting = true;
    SerialBT.println("Pausing write operations and dumping data...");
    
    // Wait a moment to ensure write task is paused
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Dump some data near the current write position (last 5 pages)
    uint32_t startAddr = (currentAddress >= 5*FLASH_PAGE_SIZE) ? 
                          (currentAddress - 5*FLASH_PAGE_SIZE) : RING_BUFFER_START_ADDR;
    SerialBT.println("Dumping 5 pages of data:");
    dumpFlashData(startAddr, 5 * FLASH_PAGE_SIZE);
    
    pauseWriting = false;
    SerialBT.println("Resuming write operations from address 0x" + String(currentAddress, HEX));
  }
  else if (command == "dumpall") {
    // Pause writing and dump all written data in the ring buffer
    pauseWriting = true;
    SerialBT.println("Pausing write operations and dumping all data...");
    
    // Wait a moment to ensure write task is paused
    vTaskDelay(pdMS_TO_TICKS(500));
    
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
      uint32_t bufferSize = RING_BUFFER_END_ADDR - RING_BUFFER_START_ADDR;
      
      // First, determine if the buffer has wrapped around and find valid data blocks
      uint8_t checkBuffer[FLASH_PAGE_SIZE];
      uint32_t validDataStart = RING_BUFFER_START_ADDR;
      uint32_t validDataEnd = currentAddress;
      bool foundStart = false;
      
      SerialBT.println("Scanning for valid data in ring buffer...");
      
      // Check if we've wrapped around by examining data validity
      if (currentAddress > RING_BUFFER_START_ADDR) {
        // Check a few pages after current position to see if they contain valid data
        // which would indicate wrap-around
        uint32_t checkAddr = currentAddress;
        bool hasWrapped = false;
        
        // Check a few pages after current position
        for (int i = 0; i < 5 && checkAddr < RING_BUFFER_END_ADDR; i++) {
          flashReadData(checkAddr, checkBuffer, FLASH_PAGE_SIZE);
          if (isDataValid(checkBuffer, FLASH_PAGE_SIZE)) {
            hasWrapped = true;
            break;
          }
          checkAddr += FLASH_PAGE_SIZE;
          if (checkAddr >= RING_BUFFER_END_ADDR) break;
        }
        
        if (hasWrapped) {
          SerialBT.println("Detected wrap-around in the ring buffer");
          
          // Find the oldest data (the first valid page after current position)
          validDataStart = currentAddress;
          while (validDataStart < RING_BUFFER_END_ADDR) {
            flashReadData(validDataStart, checkBuffer, FLASH_PAGE_SIZE);
            if (isDataValid(checkBuffer, FLASH_PAGE_SIZE)) {
              foundStart = true;
              break;
            }
            validDataStart += FLASH_PAGE_SIZE;
          }
          
          if (!foundStart) {
            // If not found after current position, check from beginning
            validDataStart = RING_BUFFER_START_ADDR;
            while (validDataStart < currentAddress) {
              flashReadData(validDataStart, checkBuffer, FLASH_PAGE_SIZE);
              if (isDataValid(checkBuffer, FLASH_PAGE_SIZE)) {
                foundStart = true;
                break;
              }
              validDataStart += FLASH_PAGE_SIZE;
            }
          }
          
          // If we found a valid start, dump from start to end of buffer, then from beginning to current
          if (foundStart) {
            if (validDataStart >= currentAddress) {
              // Dump from valid start to end of buffer
              SerialBT.printf("Dumping data from 0x%06X to 0x%06X\n", validDataStart, RING_BUFFER_END_ADDR);
              dumpFlashData(validDataStart, RING_BUFFER_END_ADDR - validDataStart);
              
              // Then dump from beginning to current position
              SerialBT.printf("Dumping data from 0x%06X to 0x%06X\n", RING_BUFFER_START_ADDR, currentAddress);
              dumpFlashData(RING_BUFFER_START_ADDR, currentAddress - RING_BUFFER_START_ADDR);
              
              uint32_t totalBytes = (RING_BUFFER_END_ADDR - validDataStart) + (currentAddress - RING_BUFFER_START_ADDR);
              SerialBT.printf("Total data dumped: %d bytes\n", totalBytes);
            } else {
              // Found valid start before current position
              SerialBT.printf("Dumping data from 0x%06X to 0x%06X\n", validDataStart, currentAddress);
              dumpFlashData(validDataStart, currentAddress - validDataStart);
              SerialBT.printf("Total data dumped: %d bytes\n", currentAddress - validDataStart);
            }
          } else {
            SerialBT.println("No valid data found in buffer");
          }
        } else {
          // No wrap-around, dump from beginning to current position
          SerialBT.printf("Dumping all data from 0x%06X to 0x%06X\n", RING_BUFFER_START_ADDR, currentAddress);
          dumpFlashData(RING_BUFFER_START_ADDR, currentAddress - RING_BUFFER_START_ADDR);
          SerialBT.printf("Total data dumped: %d bytes\n", currentAddress - RING_BUFFER_START_ADDR);
        }
      } else {
        SerialBT.println("No data written to buffer yet");
      }
      
      xSemaphoreGive(flashMutex);
    } else {
      SerialBT.println("Error: Could not acquire mutex for full data dump");
    }
    
    pauseWriting = false;
    SerialBT.println("Resuming write operations from address 0x" + String(currentAddress, HEX));
  }
  else if (command == "status") {
    SerialBT.println("Current write address: 0x" + String(currentAddress, HEX));
    SerialBT.println("Writing is " + String(pauseWriting ? "paused" : "active"));
    
    uint32_t bufferSize = RING_BUFFER_END_ADDR - RING_BUFFER_START_ADDR;
    uint32_t relativePos = currentAddress - RING_BUFFER_START_ADDR;
    float percentUsed = (float)relativePos / bufferSize * 100.0;
    
    SerialBT.printf("Ring buffer usage: %.2f%% (%d bytes out of %d)\n", 
                    percentUsed, relativePos, bufferSize);
  }
  else if (command == "erase") {
    // Pause writing and erase the entire flash chip
    pauseWriting = true;
    SerialBT.println("Pausing write operations and erasing entire flash chip...");
    
    // Wait a moment to ensure write task is paused
    vTaskDelay(pdMS_TO_TICKS(500));
    
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
      flashEraseChip();
      SerialBT.println("Chip erase complete.");
      
      // Reset the current address to start from the beginning of ring buffer
      currentAddress = RING_BUFFER_START_ADDR;
      
      // Save the reset position to metadata
      saveMetadata();
      
      xSemaphoreGive(flashMutex);
    } else {
      SerialBT.println("Error: Could not acquire mutex for chip erase");
    }
    
    pauseWriting = false;
    SerialBT.println("Resuming write operations from address 0x" + String(currentAddress, HEX));
  }
  else if (command.startsWith("dumpadd ")) {
    // Format: "dumpadd 0x123456 1024" - dumps 1024 bytes starting from address 0x123456
    pauseWriting = true;
    String params = command.substring(8);
    int spacePos = params.indexOf(' ');
    
    if (spacePos > 0) {
      String addrStr = params.substring(0, spacePos);
      String lenStr = params.substring(spacePos + 1);
      
      uint32_t addr = 0;
      uint32_t len = 0;
      
      // Parse the address
      if (addrStr.startsWith("0x")) {
        addr = strtoul(addrStr.c_str(), NULL, 16);
      } else {
        addr = strtoul(addrStr.c_str(), NULL, 10);
      }
      
      // Parse the length
      len = strtoul(lenStr.c_str(), NULL, 10);
      
      // Limit length to avoid buffer overflows
      if (len > 4096) {
        len = 4096;
        SerialBT.println("Warning: Limiting dump length to 4096 bytes");
      }
      
      SerialBT.printf("Dumping %d bytes from address 0x%06X\n", len, addr);
      dumpFlashData(addr, len);
    } else {
      SerialBT.println("Error: Invalid format. Use 'dumpadd 0xADDRESS LENGTH'");
    }
    
    pauseWriting = false;
    SerialBT.println("Resuming write operations");
  }
  else if (command == "help") {
    SerialBT.println("Available commands:");
    SerialBT.println("  dump     - Dump last 5 pages of data before current write position");
    SerialBT.println("  dumpall  - Dump all written data in the ring buffer");
    SerialBT.println("  dumpadd 0xADDRESS LENGTH - Dump specified bytes from address");
    SerialBT.println("  status   - Show current write status and ring buffer usage");
    SerialBT.println("  erase    - Erase entire flash chip");
    SerialBT.println("  help     - Show this help message");
  }
  else {
    SerialBT.println("Unknown command. Type 'help' for available commands");
  }
}

// Dump flash memory data via Bluetooth
void dumpFlashData(uint32_t startAddress, uint32_t length) {
  if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
   {
    uint8_t buffer[64]; // Use a smaller buffer for BT transmission
    
    for (uint32_t addr = startAddress; addr < startAddress + length; addr += sizeof(buffer)) {
      uint32_t chunkSize = min((uint32_t)sizeof(buffer), startAddress + length - addr);
      
      flashReadData(addr, buffer, chunkSize);
      
      // Print address
      SerialBT.printf("0x%06X: ", addr);
      
      // Print hex values
      for (uint32_t i = 0; i < chunkSize; i++) {
        SerialBT.printf("%02X ", buffer[i]);
        
        // Add a space every 8 bytes for readability
        if ((i + 1) % 8 == 0 && i < chunkSize - 1) {
          SerialBT.print(" ");
        }
      }
      
      // Print ASCII representation
      SerialBT.print(" | ");
      for (uint32_t i = 0; i < chunkSize; i++) {
        char c = buffer[i];
        if (c >= 32 && c <= 126) { // Printable ASCII
          SerialBT.print(c);
        } else {
          SerialBT.print(".");
        }
      }
      
      SerialBT.println();
      yield(); // Allow for BT processing
    }
    
    xSemaphoreGive(flashMutex);
  }
  
  else {
    SerialBT.println("Error: Could not acquire mutex for flash read");
  }

}

// --- Flash Memory Interface Functions ---

void flashWriteEnable() {
  SPISettings settings(10000000, MSBFIRST, SPI_MODE0);
  
  // Begin SPI transaction
  SPI.beginTransaction(settings);
  
  // Send write enable command
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(WRITE_ENABLE);
  digitalWrite(SPI_CS_PIN, HIGH);
  delayMicroseconds(50);  // Give time for command to process
  
  // Verify Write Enable Latch is set
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(READ_STATUS);
  uint8_t status = SPI.transfer(0);
  digitalWrite(SPI_CS_PIN, HIGH);
  
  // End SPI transaction
  SPI.endTransaction();
  
  Serial.printf("Write Enable Status: 0x%02X\n", status);
  
  // Check if Write Enable Latch (WEL) is set
  if (!(status & 0x02)) {
    Serial.println("ERROR: Write Enable Latch (WEL) not set!");
    Serial.println("Retrying write enable...");
    
    // Try one more time
    SPI.beginTransaction(settings);
    digitalWrite(SPI_CS_PIN, LOW);
    SPI.transfer(WRITE_ENABLE);
    digitalWrite(SPI_CS_PIN, HIGH);
    delayMicroseconds(50);
    
    digitalWrite(SPI_CS_PIN, LOW);
    SPI.transfer(READ_STATUS);
    status = SPI.transfer(0);
    digitalWrite(SPI_CS_PIN, HIGH);
    SPI.endTransaction();
    
    Serial.printf("Second attempt status: 0x%02X\n", status);
    
    if (!(status & 0x02)) {
      Serial.println("ERROR: Write Enable failed after retry!");
    } else {
      Serial.println("Write Enable successful after retry");
    }
  } else {
    Serial.println("Write Enable Latch (WEL) successfully set");
  }
}

void flashWaitForReady() {
  uint8_t status = 1;
  
  while (status & 0x01) {
    digitalWrite(SPI_CS_PIN, LOW);
    SPI.transfer(READ_STATUS);
    status = SPI.transfer(0);
    digitalWrite(SPI_CS_PIN, HIGH);
    delayMicroseconds(10);
  }
}

void flashEraseSector(uint32_t address) {
  flashWriteEnable();
  
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(SECTOR_ERASE);
  SPI.transfer((address >> 16) & 0xFF);  // Address byte 1
  SPI.transfer((address >> 8) & 0xFF);   // Address byte 2
  SPI.transfer(address & 0xFF);          // Address byte 3
  digitalWrite(SPI_CS_PIN, HIGH);
  
  flashWaitForReady();
}

void flashEraseChip() {
  SerialBT.println("Starting full chip erase. This may take several seconds...");
  
  flashWriteEnable();
  
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(CHIP_ERASE);
  digitalWrite(SPI_CS_PIN, HIGH);
  
  // Chip erase can take a while, so we'll wait
  Serial.println("Waiting for chip erase to complete...");
  flashWaitForReady();
  Serial.println("Chip erase completed.");
}

void flashReadData(uint32_t address, uint8_t *buffer, int length) {
  digitalWrite(SPI_CS_PIN, LOW);
  
  SPI.transfer(READ_DATA);
  SPI.transfer((address >> 16) & 0xFF);  // Address byte 1
  SPI.transfer((address >> 8) & 0xFF);   // Address byte 2
  SPI.transfer(address & 0xFF);          // Address byte 3
  
  for (int i = 0; i < length; i++) {
    buffer[i] = SPI.transfer(0);
  }
  
  digitalWrite(SPI_CS_PIN, HIGH);
}

void flashWritePage(uint32_t address, uint8_t *data, int length) {
  SPISettings settings(10000000, MSBFIRST, SPI_MODE0);
  
  // Check initial status
  SPI.beginTransaction(settings);
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(READ_STATUS);
  uint8_t statusBefore = SPI.transfer(0);
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.endTransaction();
  Serial.printf("Status before write enable: 0x%02X\n", statusBefore);

  // Enable writing
  flashWriteEnable();
  
  // Verify write enable was successful before proceeding
  SPI.beginTransaction(settings);
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(READ_STATUS);
  uint8_t statusAfterEnable = SPI.transfer(0);
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.endTransaction();
  Serial.printf("Status after write enable: 0x%02X\n", statusAfterEnable);

  if (!(statusAfterEnable & 0x02)) {
    Serial.println("ERROR: Write enable failed, aborting write operation");
    return;
  }

  // Perform the write operation
  SPI.beginTransaction(settings);
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(PAGE_PROGRAM);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  
  for (int i = 0; i < length; i++) {
    SPI.transfer(data[i]);
  }
  
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.endTransaction();
  
  // Wait for write to complete
  flashWaitForReady();
  
  // Final status check
  SPI.beginTransaction(settings);
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(READ_STATUS);
  uint8_t statusFinal = SPI.transfer(0);
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.endTransaction();
  
  Serial.printf("Status after write complete: 0x%02X\n", statusFinal);
}

void flashInit() {
  SPISettings settings(10000000, MSBFIRST, SPI_MODE0);
  
  // Begin SPI transaction
  SPI.beginTransaction(settings);
  
  // Release from deep power down
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(RELEASE_POWERDOWN);
  digitalWrite(SPI_CS_PIN, HIGH);
  delay(10);  // Wait for chip to wake up
  
  // Read JEDEC ID to verify communication
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(READ_JEDEC_ID);
  uint8_t manufacturer = SPI.transfer(0);
  uint8_t memtype = SPI.transfer(0);
  uint8_t capacity = SPI.transfer(0);
  digitalWrite(SPI_CS_PIN, HIGH);
  
  Serial.printf("Flash JEDEC ID: 0x%02X 0x%02X 0x%02X\n", manufacturer, memtype, capacity);
  
  // Read Device ID and unique ID
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(READ_DEVICE_ID);
  SPI.transfer(0);    // Dummy byte
  SPI.transfer(0);    // Dummy byte
  SPI.transfer(0);    // Dummy byte
  uint8_t deviceId = SPI.transfer(0);
  digitalWrite(SPI_CS_PIN, HIGH);
  
  Serial.printf("Flash Device ID: 0x%02X\n", deviceId);
  
  SPI.endTransaction();
  
  // If we can't read a valid manufacturer ID, there's likely a connection problem
  if (manufacturer == 0x00 || manufacturer == 0xFF) {
    Serial.println("ERROR: Could not detect flash chip!");
    Serial.println("Check SPI connections and power to the chip.");
    while(1) {
      delay(1000);  // Halt execution
    }
  }
  
  Serial.println("Flash chip initialized successfully");
}