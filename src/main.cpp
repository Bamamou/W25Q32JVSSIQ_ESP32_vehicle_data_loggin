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
#define CHIP_ERASE      0xC7
#define READ_STATUS     0x05
#define READ_DATA       0x03
#define PAGE_PROGRAM    0x02
#define SECTOR_ERASE    0x20

// Pin definitions
#define SPI_CS_PIN      5
#define SPI_MOSI_PIN    23
#define SPI_MISO_PIN    19
#define SPI_CLK_PIN     18

// Flash memory constants
#define FLASH_PAGE_SIZE 256
#define FLASH_SECTOR_SIZE 4096
#define FLASH_TOTAL_SIZE 4194304  // 32 Mbit = 4 MB

// FreeRTOS task handles and synchronization objects
TaskHandle_t writeTaskHandle;
TaskHandle_t btTaskHandle;
SemaphoreHandle_t flashMutex;

// BluetoothSerial instance
BluetoothSerial SerialBT;

// Global variables
uint32_t currentAddress = 0;
bool pauseWriting = false;
uint8_t testData[FLASH_PAGE_SIZE];

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

void setup() {
  // Initialize Serial for debugging (optional)
  Serial.begin(115200);
  
  // Initialize SPI
  SPI.begin(SPI_CLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
  SPI.setFrequency(10000000); // 10 MHz
  SPI.setDataMode(SPI_MODE0);
  
  // Configure CS pin as output
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  
  // Initialize Bluetooth Serial
  SerialBT.begin("ESP32_Flash_Writer");
  
  // Create mutex for flash access
  flashMutex = xSemaphoreCreateMutex();
  
  // Initialize test data pattern
  for (int i = 0; i < FLASH_PAGE_SIZE; i++) {
    testData[i] = i % 256;
  }
  
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
}

void loop() {
  // Main loop is empty since we're using FreeRTOS tasks
  delay(1000);
}

// Task to continuously write data to flash memory
void writeTask(void *parameter) {
  // Wait a brief moment before starting to allow system to stabilize
  vTaskDelay(pdMS_TO_TICKS(2000));
  
  Serial.println("Write task started");
  
  while (true) {
    if (!pauseWriting) {
      if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Check if we need to wrap around to start
        if (currentAddress >= FLASH_TOTAL_SIZE) {
          currentAddress = 0;
          Serial.println("Reached end of flash memory, wrapping to beginning");
        }
        
        // Check if we need to erase a sector (sectors are 4KB)
        if (currentAddress % FLASH_SECTOR_SIZE == 0) {
          Serial.printf("Erasing sector at address 0x%06X\n", currentAddress);
          flashEraseSector(currentAddress);
        }
        
        // Update test data to include address information
        uint32_t pageAddr = currentAddress;
        for (int i = 0; i < FLASH_PAGE_SIZE; i += 4) {
          if (i + 3 < FLASH_PAGE_SIZE) {
            // First 4 bytes contain the address
            testData[i] = (pageAddr >> 24) & 0xFF;
            testData[i+1] = (pageAddr >> 16) & 0xFF;
            testData[i+2] = (pageAddr >> 8) & 0xFF;
            testData[i+3] = pageAddr & 0xFF;
          }
          pageAddr += 4;
        }
        
        // Write a page of data
        Serial.printf("Writing page at address 0x%06X\n", currentAddress);
        flashWritePage(currentAddress, testData, FLASH_PAGE_SIZE);
        
        // Move to next page
        currentAddress += FLASH_PAGE_SIZE;
        
        xSemaphoreGive(flashMutex);
      }
      
      // Small delay between write operations
      vTaskDelay(pdMS_TO_TICKS(500));
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
                          (currentAddress - 5*FLASH_PAGE_SIZE) : 0;
    SerialBT.println("Dumping 5 pages of data:");
    dumpFlashData(startAddr, 5 * FLASH_PAGE_SIZE);
    
    pauseWriting = false;
    SerialBT.println("Resuming write operations from address 0x" + String(currentAddress, HEX));
  }
  else if (command == "status") {
    SerialBT.println("Current write address: 0x" + String(currentAddress, HEX));
    SerialBT.println("Writing is " + String(pauseWriting ? "paused" : "active"));
    float percentUsed = (float)currentAddress / FLASH_TOTAL_SIZE * 100.0;
    SerialBT.printf("Flash usage: %.2f%% (%d bytes out of %d)\n", 
                    percentUsed, currentAddress, FLASH_TOTAL_SIZE);
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
      
      // Reset the current address to start from the beginning
      currentAddress = 0;
      
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
    SerialBT.println("  dumpadd 0xADDRESS LENGTH - Dump specified bytes from address");
    SerialBT.println("  status   - Show current write status");
    SerialBT.println("  erase    - Erase entire flash chip");
    SerialBT.println("  help     - Show this help message");
  }
  else {
    SerialBT.println("Unknown command. Type 'help' for available commands");
  }
}

// Dump flash memory data via Bluetooth
void dumpFlashData(uint32_t startAddress, uint32_t length) {
  if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
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
  } else {
    SerialBT.println("Error: Could not acquire mutex for flash read");
  }
}

// --- Flash Memory Interface Functions ---

void flashWriteEnable() {
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(WRITE_ENABLE);
  digitalWrite(SPI_CS_PIN, HIGH);
  delayMicroseconds(10);
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
  flashWriteEnable();
  
  digitalWrite(SPI_CS_PIN, LOW);
  
  SPI.transfer(PAGE_PROGRAM);
  SPI.transfer((address >> 16) & 0xFF);  // Address byte 1
  SPI.transfer((address >> 8) & 0xFF);   // Address byte 2
  SPI.transfer(address & 0xFF);          // Address byte 3
  
  for (int i = 0; i < length; i++) {
    SPI.transfer(data[i]);
  }
  
  digitalWrite(SPI_CS_PIN, HIGH);
  
  flashWaitForReady();
}