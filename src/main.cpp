// #include <Arduino.h>
// #include <SPI.h>

// // // Pin definitions for W25Q256JVEIQ
// #define pinHSPI_CS        26     // Chip Select pin
// #define pinHSPI_MISO      12    // HSPI_MISO
// #define pinHSPI_MOSI      2    // HSPI_MOSI or GPIO13 depending on board
// #define pinHSPI_CLK       14    // HSPI_CLK

// // SPI command definitions
// #define CMD_JEDEC_ID           0x9F
// #define CMD_READ_STATUS_REG1   0x05
// #define CMD_READ_STATUS_REG2   0x35
// #define CMD_WRITE_ENABLE       0x06
// #define CMD_WRITE_STATUS_REG2  0x31

// uint8_t manufacturerID = 0;
// uint8_t deviceType = 0;
// uint8_t capacityCode = 0;

// // 🔁 Function Declarations (Prototypes)
// void setup();
// void loop();

// void detectChip();
// void readStatusRegisters();
// void printMemoryInfo();
// void enableQuadMode();
// void checkQuadEnable();
// uint8_t readRegister(uint8_t regCmd);

// // ==============================
// //        SETUP FUNCTION
// // ==============================
// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   pinMode(pinHSPI_CS, OUTPUT);
//   digitalWrite(pinHSPI_CS, HIGH); // Deselect chip

//   // Begin SPI using HSPI pins: SCLK=14, MISO=12, MOSI=13
//   SPI.begin(pinHSPI_CLK, pinHSPI_MISO, pinHSPI_MOSI, pinHSPI_CS);

//   Serial.println("🔍 SPI Flash Memory Test Tool");
//   Serial.println("=============================");
//   detectChip();
//   readStatusRegisters();
//   printMemoryInfo();
//   enableQuadMode();
//   checkQuadEnable();
// }

// // ==============================
// //        MAIN LOOP
// // ==============================
// void loop() {
//   delay(5000);
//   readStatusRegisters();
// }

// // ==============================
// //        FUNCTION DEFINITIONS
// // ==============================

// // Detect JEDEC ID
// void detectChip() {
//   digitalWrite(pinHSPI_CS, LOW);
//   manufacturerID = SPI.transfer(CMD_JEDEC_ID);
//   deviceType = SPI.transfer(0x00);
//   capacityCode = SPI.transfer(0x00);
//   digitalWrite(pinHSPI_CS, HIGH);

//   Serial.print("🔢 Manufacturer ID: 0x"); Serial.println(manufacturerID, HEX);
//   Serial.print("🔢 Device Type: 0x"); Serial.println(deviceType, HEX);
//   Serial.print("🔢 Capacity Code: 0x"); Serial.println(capacityCode, HEX);

//   if (manufacturerID == 0xEF && deviceType == 0x40 && capacityCode == 0x19) {
//     Serial.println("✅ Detected Winbond W25Q256JVEIQ (32MB)");
//   } else {
//     Serial.println("❌ Unknown or no flash chip detected.");
//   }
// }

// // Read Status Registers
// void readStatusRegisters() {
//   uint8_t sr1 = readRegister(CMD_READ_STATUS_REG1);
//   uint8_t sr2 = readRegister(CMD_READ_STATUS_REG2);

//   Serial.print("🚦 Status Register 1 (SR1): 0x"); Serial.println(sr1, HEX);
//   Serial.print("🚦 Status Register 2 (SR2): 0x"); Serial.println(sr2, HEX);

//   // Interpret bits
//   Serial.println("📝 SR1 Bit Interpretation:");
//   Serial.print("  - Write In Progress (WIP): "); Serial.println((sr1 & 0x01) ? "Yes" : "No");
//   Serial.print("  - Write Enable Latch (WEL): "); Serial.println((sr1 & 0x02) ? "Enabled" : "Disabled");

//   Serial.println("📝 SR2 Bit Interpretation:");
//   Serial.print("  - Quad Enable (QE): "); Serial.println((sr2 & (1 << 6)) ? "Enabled" : "Disabled");
// }

// // Helper function to read a register
// uint8_t readRegister(uint8_t regCmd) {
//   digitalWrite(pinHSPI_CS, LOW);
//   SPI.transfer(regCmd);
//   uint8_t data = SPI.transfer(0x00);
//   digitalWrite(pinHSPI_CS, HIGH);
//   return data;
// }

// // Print readable memory info
// void printMemoryInfo() {
//   uint32_t totalSizeKB = 0;

//   switch (capacityCode) {
//     case 0x15: totalSizeKB = 32 * 1024; break;   // 32Mbit => 4MB
//     case 0x16: totalSizeKB = 64 * 1024; break;   // 64Mbit => 8MB
//     case 0x17: totalSizeKB = 128 * 1024; break;  // 128Mbit => 16MB
//     case 0x19: totalSizeKB = 256 * 1024; break;  // 256Mbit => 32MB
//     default:
//       Serial.println("⚠️ Unknown capacity code");
//       return;
//   }

//   Serial.print("💾 Total Flash Size: ");
//   Serial.print(totalSizeKB / 1024);
//   Serial.println(" MB");
// }

// // Enable Quad Mode (QE bit in SR2)
// void enableQuadMode() {
//   // Step 1: Send Write Enable
//   digitalWrite(pinHSPI_CS, LOW);
//   SPI.transfer(CMD_WRITE_ENABLE);
//   digitalWrite(pinHSPI_CS, HIGH);
//   delayMicroseconds(10);

//   // Step 2: Read current SR2
//   uint8_t sr2 = readRegister(CMD_READ_STATUS_REG2);
//   Serial.print("📄 Current SR2: 0x"); Serial.println(sr2, HEX);

//   // Step 3: Set QE bit (bit 6)
//   sr2 |= (1 << 6);  // QE bit = 1

//   // Step 4: Write updated SR2
//   digitalWrite(pinHSPI_CS, LOW);
//   SPI.transfer(CMD_WRITE_STATUS_REG2);
//   SPI.transfer(sr2);
//   digitalWrite(pinHSPI_CS, HIGH);
//   delay(10); // Allow time for write

//   Serial.println("✅ Quad Enable bit set.");
// }

// // Confirm QE bit is set
// void checkQuadEnable() {
//   uint8_t sr2 = readRegister(CMD_READ_STATUS_REG2);
//   bool qeEnabled = (sr2 & (1 << 6)) != 0;

//   Serial.print("🚦 Quad Enable (QE) Bit: ");
//   Serial.println(qeEnabled ? "Enabled ✅" : "Disabled ❌");
// }