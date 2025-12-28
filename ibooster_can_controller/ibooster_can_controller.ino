/*
 * iBooster CAN Research Controller
 * 
 * Hardware:
 * - Arduino Mega2560
 * - 2x MCP2515 CAN modules
 *   - YAW CAN: CS=4, INT=3 (RX 0x39D for rod position decode)
 *   - VEH CAN: CS=5, INT=2 (TX 0x38D, 0x38B, 0x38C)
 * 
 * Features:
 * - Nonblocking 10ms TX loop
 * - 50ms serial print loop
 * - Serial CLI commands: tx, a, s, r, arm, set, help
 * - Message counters with rollover
 * - Selectable CRC8 J1850 or SUM checksum
 * 
 * License: GPLv3
 * 
 * DISCLAIMER: This is for research/bench testing only.
 * NO WARRANTY. NOT FOR ROAD USE.
 */

#include <SPI.h>
#include <mcp2515.h>

// Pin definitions for MCP2515 modules
#define YAW_CS_PIN 4
#define YAW_INT_PIN 3
#define VEH_CS_PIN 5
#define VEH_INT_PIN 2

// CAN IDs
#define YAW_RX_ID 0x39D  // Rod position from YAW CAN
#define VEH_TX_ID_38D 0x38D
#define VEH_TX_ID_38B 0x38B
#define VEH_TX_ID_38C 0x38C

// Timing intervals
#define TX_INTERVAL_MS 10
#define PRINT_INTERVAL_MS 50

// Checksum modes
enum ChecksumMode {
  CHECKSUM_CRC8_J1850,
  CHECKSUM_SUM
};

// Global objects
MCP2515 yaw_can(YAW_CS_PIN);
MCP2515 veh_can(VEH_CS_PIN);

// State variables
bool tx_enabled = false;
bool armed = false;
ChecksumMode checksum_mode = CHECKSUM_CRC8_J1850;

// Message counters (4-bit, 0-15)
uint8_t counter_38d = 0;
uint8_t counter_38b = 0;
uint8_t counter_38c = 0;

// Rod position from YAW CAN
int16_t rod_position = 0;
uint32_t last_rod_rx_time = 0;

// Timing
unsigned long last_tx_time = 0;
unsigned long last_print_time = 0;

// CAN message buffers
struct can_frame veh_frame_38d;
struct can_frame veh_frame_38b;
struct can_frame veh_frame_38c;
struct can_frame yaw_rx_frame;

// Serial input buffer
String serial_input = "";

// CRC8 J1850 lookup table
const uint8_t crc8_j1850_table[256] = {
  0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
  0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
  0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
  0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
  0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
  0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
  0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
  0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
  0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
  0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
  0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
  0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
  0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
  0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
  0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
  0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

// Function prototypes
void setup();
void loop();
void init_can_messages();
uint8_t calculate_crc8_j1850(uint8_t *data, uint8_t len);
uint8_t calculate_sum(uint8_t *data, uint8_t len);
void update_message_checksums();
void send_veh_can_messages();
void receive_yaw_can_messages();
void process_serial_commands();
void print_status();
void print_help();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait up to 3 seconds for serial
  
  Serial.println(F("\n=== iBooster CAN Research Controller ==="));
  Serial.println(F("Version: 1.0"));
  Serial.println(F("License: GPLv3"));
  Serial.println(F("\nDISCLAIMER: Research/bench use only. NO WARRANTY. NOT FOR ROAD USE.\n"));
  
  // Initialize SPI
  SPI.begin();
  
  // Initialize YAW CAN (RX)
  yaw_can.reset();
  yaw_can.setBitrate(CAN_500KBPS, MCP_8MHZ);
  yaw_can.setNormalMode();
  Serial.println(F("YAW CAN initialized (CS4/INT3, RX 0x39D)"));
  
  // Initialize VEH CAN (TX)
  veh_can.reset();
  veh_can.setBitrate(CAN_500KBPS, MCP_8MHZ);
  veh_can.setNormalMode();
  Serial.println(F("VEH CAN initialized (CS5/INT2, TX 0x38D/0x38B/0x38C)"));
  
  // Initialize CAN messages
  init_can_messages();
  
  Serial.println(F("\nType 'help' for commands"));
  Serial.print(F("> "));
}

void loop() {
  unsigned long current_time = millis();
  
  // Nonblocking TX loop (10ms)
  if (tx_enabled && (current_time - last_tx_time >= TX_INTERVAL_MS)) {
    last_tx_time = current_time;
    send_veh_can_messages();
  }
  
  // Receive from YAW CAN
  receive_yaw_can_messages();
  
  // Nonblocking print loop (50ms)
  if (current_time - last_print_time >= PRINT_INTERVAL_MS) {
    last_print_time = current_time;
    if (tx_enabled) {
      print_status();
    }
  }
  
  // Process serial commands
  process_serial_commands();
}

void init_can_messages() {
  // Initialize 0x38D message (8 bytes)
  veh_frame_38d.can_id = VEH_TX_ID_38D;
  veh_frame_38d.can_dlc = 8;
  memset(veh_frame_38d.data, 0, 8);
  
  // Initialize 0x38B message (8 bytes)
  veh_frame_38b.can_id = VEH_TX_ID_38B;
  veh_frame_38b.can_dlc = 8;
  memset(veh_frame_38b.data, 0, 8);
  
  // Initialize 0x38C message (8 bytes)
  veh_frame_38c.can_id = VEH_TX_ID_38C;
  veh_frame_38c.can_dlc = 8;
  memset(veh_frame_38c.data, 0, 8);
  
  Serial.println(F("CAN messages initialized"));
}

uint8_t calculate_crc8_j1850(uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc = crc8_j1850_table[crc ^ data[i]];
  }
  return crc ^ 0xFF;
}

uint8_t calculate_sum(uint8_t *data, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum;
}

void update_message_checksums() {
  // Update counters (4-bit, in nibbles)
  veh_frame_38d.data[0] = (veh_frame_38d.data[0] & 0xF0) | (counter_38d & 0x0F);
  veh_frame_38b.data[0] = (veh_frame_38b.data[0] & 0xF0) | (counter_38b & 0x0F);
  veh_frame_38c.data[0] = (veh_frame_38c.data[0] & 0xF0) | (counter_38c & 0x0F);
  
  // Calculate and set checksums (last byte)
  if (checksum_mode == CHECKSUM_CRC8_J1850) {
    veh_frame_38d.data[7] = calculate_crc8_j1850(veh_frame_38d.data, 7);
    veh_frame_38b.data[7] = calculate_crc8_j1850(veh_frame_38b.data, 7);
    veh_frame_38c.data[7] = calculate_crc8_j1850(veh_frame_38c.data, 7);
  } else {
    veh_frame_38d.data[7] = calculate_sum(veh_frame_38d.data, 7);
    veh_frame_38b.data[7] = calculate_sum(veh_frame_38b.data, 7);
    veh_frame_38c.data[7] = calculate_sum(veh_frame_38c.data, 7);
  }
}

void send_veh_can_messages() {
  if (!armed) return;
  
  // Update checksums and counters
  update_message_checksums();
  
  // Send messages
  veh_can.sendMessage(&veh_frame_38d);
  veh_can.sendMessage(&veh_frame_38b);
  veh_can.sendMessage(&veh_frame_38c);
  
  // Increment counters (4-bit rollover at 16)
  counter_38d = (counter_38d + 1) & 0x0F;
  counter_38b = (counter_38b + 1) & 0x0F;
  counter_38c = (counter_38c + 1) & 0x0F;
}

void receive_yaw_can_messages() {
  if (yaw_can.readMessage(&yaw_rx_frame) == MCP2515::ERROR_OK) {
    if (yaw_rx_frame.can_id == YAW_RX_ID && yaw_rx_frame.can_dlc >= 2) {
      // Decode rod position (assumed 16-bit, bytes 0-1, little-endian)
      rod_position = (int16_t)(yaw_rx_frame.data[0] | (yaw_rx_frame.data[1] << 8));
      last_rod_rx_time = millis();
    }
  }
}

void process_serial_commands() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (serial_input.length() > 0) {
        serial_input.trim();
        serial_input.toLowerCase();
        
        // Process commands
        if (serial_input == "help" || serial_input == "h") {
          print_help();
        }
        else if (serial_input == "tx") {
          tx_enabled = !tx_enabled;
          Serial.print(F("\nTX "));
          Serial.println(tx_enabled ? F("enabled") : F("disabled"));
        }
        else if (serial_input == "a" || serial_input == "arm") {
          armed = !armed;
          Serial.print(F("\nSystem "));
          Serial.println(armed ? F("ARMED") : F("disarmed"));
          if (!armed) {
            tx_enabled = false;
            Serial.println(F("TX disabled"));
          }
        }
        else if (serial_input == "s" || serial_input == "status") {
          Serial.println();
          print_status();
        }
        else if (serial_input == "r" || serial_input == "reset") {
          counter_38d = 0;
          counter_38b = 0;
          counter_38c = 0;
          Serial.println(F("\nCounters reset"));
        }
        else if (serial_input.startsWith("set ")) {
          String param = serial_input.substring(4);
          param.trim();
          
          if (param == "crc8" || param == "crc") {
            checksum_mode = CHECKSUM_CRC8_J1850;
            Serial.println(F("\nChecksum mode: CRC8 J1850"));
          }
          else if (param == "sum") {
            checksum_mode = CHECKSUM_SUM;
            Serial.println(F("\nChecksum mode: SUM"));
          }
          else {
            Serial.println(F("\nUnknown parameter. Use: set crc8|sum"));
          }
        }
        else if (serial_input.length() > 0) {
          Serial.print(F("\nUnknown command: "));
          Serial.println(serial_input);
          Serial.println(F("Type 'help' for commands"));
        }
        
        serial_input = "";
        Serial.print(F("> "));
      }
    }
    else if (c >= 32 && c <= 126) {  // Printable characters
      serial_input += c;
    }
  }
}

void print_status() {
  Serial.print(F("ARM:"));
  Serial.print(armed ? F("Y") : F("N"));
  Serial.print(F(" TX:"));
  Serial.print(tx_enabled ? F("Y") : F("N"));
  Serial.print(F(" CHK:"));
  Serial.print(checksum_mode == CHECKSUM_CRC8_J1850 ? F("CRC8") : F("SUM"));
  Serial.print(F(" CTR:"));
  Serial.print(counter_38d);
  Serial.print(F("/"));
  Serial.print(counter_38b);
  Serial.print(F("/"));
  Serial.print(counter_38c);
  Serial.print(F(" ROD:"));
  Serial.print(rod_position);
  
  // Show age of rod position
  if (last_rod_rx_time > 0) {
    unsigned long age = millis() - last_rod_rx_time;
    Serial.print(F(" ("));
    Serial.print(age);
    Serial.print(F("ms)"));
  } else {
    Serial.print(F(" (no data)"));
  }
  
  Serial.println();
}

void print_help() {
  Serial.println(F("\n=== Commands ==="));
  Serial.println(F("help, h       - Show this help"));
  Serial.println(F("tx            - Toggle TX enable/disable"));
  Serial.println(F("arm, a        - Toggle system arm/disarm"));
  Serial.println(F("status, s     - Print current status"));
  Serial.println(F("reset, r      - Reset message counters"));
  Serial.println(F("set crc8      - Use CRC8 J1850 checksum"));
  Serial.println(F("set sum       - Use SUM checksum"));
  Serial.println(F("\nSafety: Must ARM before TX can be enabled"));
  Serial.println(F("YAW CAN RX: 0x39D (rod position)"));
  Serial.println(F("VEH CAN TX: 0x38D, 0x38B, 0x38C (10ms interval)"));
  Serial.println();
}
