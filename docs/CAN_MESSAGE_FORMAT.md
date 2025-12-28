# CAN Message Format Documentation

## Overview

This document describes the CAN message formats used for communicating with the Bosch iBooster Gen2 on the bench testing setup.

## CAN Bus Configuration

### YAW CAN Bus (Receive)
- **Purpose:** Monitor iBooster feedback
- **Bitrate:** 500 kbps
- **Direction:** Receive only
- **Messages:**
  - 0x39D - Rod position feedback

### VEH CAN Bus (Transmit)
- **Purpose:** Control iBooster operation
- **Bitrate:** 500 kbps
- **Direction:** Transmit only
- **Messages:**
  - 0x38D - Control message 1
  - 0x38B - Control message 2
  - 0x38C - Control message 3

## Message Structure

All messages follow this general structure:

```
Byte 0: Counter (lower nibble, 4-bit) + Additional data (upper nibble)
Byte 1-6: Message-specific data
Byte 7: Checksum (CRC8 J1850 or SUM)
```

### Message Counter

- **Size:** 4 bits (0-15)
- **Location:** Lower nibble of Byte 0
- **Behavior:** Increments with each transmission, rolls over at 16
- **Purpose:** Message sequence tracking and validation

### Checksum

Two checksum modes are supported:

#### CRC8 J1850
- **Algorithm:** SAE J1850 CRC8
- **Polynomial:** 0x1D
- **Initial value:** 0xFF
- **Final XOR:** 0xFF
- **Data:** Bytes 0-6
- **Location:** Byte 7

#### Simple SUM
- **Algorithm:** 8-bit sum of bytes
- **Data:** Bytes 0-6
- **Location:** Byte 7
- **Calculation:** SUM(Byte 0 through Byte 6) & 0xFF

## Message Details

### 0x39D - Rod Position (YAW CAN RX)

**Purpose:** Feedback from iBooster indicating pushrod position

**Format:**
```
Byte 0-1: Rod position (16-bit signed integer, little-endian)
Byte 2-7: Additional sensor data or status (to be documented)
```

**Decoding Example:**
```c
int16_t rod_position = (int16_t)(data[0] | (data[1] << 8));
```

**Units:** To be determined through testing (likely encoder counts or millimeters)

**Update Rate:** Varies based on iBooster operation (typically 10-100 Hz)

### 0x38D - VEH CAN Control Message 1 (TX)

**Purpose:** Primary control message for iBooster

**Format:**
```
Byte 0: [7:4] TBD | [3:0] Counter (0-15)
Byte 1-6: Control data (pressure/force requests, modes, etc.)
Byte 7: Checksum
```

**Transmission Rate:** 10 ms (100 Hz)

**Note:** Exact byte definitions require reverse engineering or OEM documentation

### 0x38B - VEH CAN Control Message 2 (TX)

**Purpose:** Secondary control message for iBooster

**Format:**
```
Byte 0: [7:4] TBD | [3:0] Counter (0-15)
Byte 1-6: Control data
Byte 7: Checksum
```

**Transmission Rate:** 10 ms (100 Hz)

**Note:** May contain status requests, mode selections, or additional control parameters

### 0x38C - VEH CAN Control Message 3 (TX)

**Purpose:** Tertiary control message for iBooster

**Format:**
```
Byte 0: [7:4] TBD | [3:0] Counter (0-15)
Byte 1-6: Control data
Byte 7: Checksum
```

**Transmission Rate:** 10 ms (100 Hz)

**Note:** Purpose and exact format to be determined through testing

## Timing Requirements

### Transmission Timing
- **TX Period:** 10 ms (100 Hz)
- **Implementation:** Nonblocking timer-based transmission
- **Jitter:** Should be minimized for stable operation

### Reception Timing
- **RX Monitoring:** Continuous polling
- **Timeout:** Track age of received messages to detect loss

### Status Display
- **Print Period:** 50 ms (20 Hz)
- **Purpose:** Human-readable monitoring without overwhelming serial output

## Checksum Validation

### Transmission (TX)
1. Populate bytes 0-6 with data
2. Calculate checksum over bytes 0-6
3. Store checksum in byte 7
4. Transmit message

### Reception (RX)
1. Receive all 8 bytes
2. Calculate checksum over bytes 0-6
3. Compare with byte 7
4. Accept message if checksums match

## Error Handling

### Checksum Mismatch
- Log error
- Discard message
- Continue operation

### Missing Messages
- Track last reception timestamp
- Alert if timeout exceeded (e.g., > 100 ms)

### Counter Errors
- Detect non-sequential counters
- May indicate message loss
- Log but continue operation

## Research Notes

### Reverse Engineering Process
To fully document these messages:

1. **Log messages** during known operations
2. **Vary inputs** and observe output changes
3. **Map bits** to functions through systematic testing
4. **Validate findings** with multiple test runs
5. **Document patterns** and constraints

### Safety Considerations
- Always test with bench-mounted components
- Verify checksum calculations before live testing
- Monitor for unexpected behavior
- Implement emergency stop capability

### Testing Methodology
1. Start with passive monitoring (RX only)
2. Verify checksum calculations match observed values
3. Begin with minimal TX messages (zeros with valid checksums)
4. Gradually activate message fields
5. Document behavioral responses

## CRC8 J1850 Algorithm

### Lookup Table Method (Used in Code)
```c
uint8_t crc8_j1850(uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc ^ 0xFF;
}
```

### Properties
- **Fast computation** using pre-calculated lookup table
- **Standard automotive** checksum algorithm
- **Detects:** Single-bit errors, burst errors, most multi-bit errors

## Message Evolution

As research progresses, this document should be updated with:
- Exact bit field definitions
- Value ranges and scaling factors
- Mode selections and flags
- Observed behaviors and responses
- Timing constraints discovered
- Error codes and diagnostics

## References

- SAE J1850 Standard
- ISO 11898 (CAN Bus Standard)
- Bosch iBooster technical documentation (if available)
- Reverse engineering findings from community

## Contributing

If you discover additional message details:
1. Document your findings thoroughly
2. Include test methodology and conditions
3. Submit updates via pull request
4. Verify with multiple independent tests

---

**Note:** This is a living document. As research progresses, message details will be refined and expanded.

**Last Updated:** December 2024
