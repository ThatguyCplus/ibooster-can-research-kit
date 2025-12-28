# iBooster CAN Research Kit

CAN research and bench-control experiments for Bosch iBooster Gen2. This repository contains Arduino code, documentation, and research notes for interfacing with the iBooster's CAN bus system.

**DISCLAIMER: This is for research and bench testing ONLY. NO WARRANTY. NOT FOR USE ON PUBLIC ROADS.**

## Overview

This project provides an Arduino-based controller for researching the Bosch iBooster Gen2 CAN bus interface. It uses an Arduino Mega2560 with dual MCP2515 CAN modules to:

- **Monitor** the YAW CAN bus (0x39D for rod position decoding)
- **Control** via VEH CAN bus (0x38D, 0x38B, 0x38C messages)
- Provide a serial CLI for interactive control and monitoring

## Hardware Requirements

- **Arduino Mega2560**
- **2x MCP2515 CAN Bus Modules** with 8MHz crystal
- **Bosch iBooster Gen2** unit
- Appropriate CAN bus wiring and termination resistors (120Ω)
- Power supply for iBooster (12V automotive)

## Wiring Diagram

### MCP2515 Module Connections

**YAW CAN Module (Receive - Rod Position)**
- VCC → 5V
- GND → GND
- CS → Digital Pin 4
- INT → Digital Pin 3
- SCK → Digital Pin 52 (SPI)
- MISO → Digital Pin 50 (SPI)
- MOSI → Digital Pin 51 (SPI)
- CAN-H/L → iBooster YAW CAN bus

**VEH CAN Module (Transmit - Control)**
- VCC → 5V
- GND → GND
- CS → Digital Pin 5
- INT → Digital Pin 2
- SCK → Digital Pin 52 (SPI - shared)
- MISO → Digital Pin 50 (SPI - shared)
- MOSI → Digital Pin 51 (SPI - shared)
- CAN-H/L → iBooster VEH CAN bus

### CAN Bus Configuration
- **Bitrate:** 500 kbps
- **Termination:** 120Ω resistors at each end of CAN bus
- **Crystal:** 8 MHz (MCP2515 modules)

## Software Dependencies

This project requires the following Arduino library:

- **mcp2515** by autowp - Install via Arduino Library Manager

To install:
1. Open Arduino IDE
2. Go to Sketch → Include Library → Manage Libraries
3. Search for "mcp2515" by autowp
4. Click Install

## Features

### CAN Bus Interface
- **YAW CAN RX:** Receives 0x39D messages containing rod position
- **VEH CAN TX:** Transmits 0x38D, 0x38B, 0x38C messages at 10ms intervals
- **Nonblocking operation:** Uses timing-based loops for TX and status printing

### Message Format
- **8-byte messages** with counter and checksum
- **4-bit rolling counter** in byte 0 (lower nibble), rolls over 0-15
- **Checksum in byte 7:**
  - CRC8 J1850 (default)
  - Simple SUM (selectable)

### Serial CLI Commands

Connect at **115200 baud** and use these commands:

| Command | Alias | Description |
|---------|-------|-------------|
| `help` | `h` | Display command list |
| `tx` | - | Toggle TX enable/disable |
| `arm` | `a` | Toggle system arm/disarm |
| `status` | `s` | Print current status |
| `reset` | `r` | Reset message counters to 0 |
| `set crc8` | - | Use CRC8 J1850 checksum |
| `set sum` | - | Use SUM checksum |

### Safety Features
- **Two-step activation:** Must ARM system before TX can be enabled
- **Status monitoring:** 50ms status updates when TX is active
- **Manual control:** All operations require explicit commands

## Usage

### Initial Setup

1. Wire hardware according to wiring diagram
2. Install required Arduino libraries
3. Upload sketch to Arduino Mega2560
4. Open Serial Monitor at 115200 baud

### Basic Operation

```
1. Type 'arm' to arm the system
   > arm
   System ARMED

2. Type 'tx' to enable transmission
   > tx
   TX enabled

3. Monitor status output (printed every 50ms)
   ARM:Y TX:Y CHK:CRC8 CTR:5/5/5 ROD:1234 (23ms)

4. Type 'tx' again to stop transmission
   > tx
   TX disabled

5. Type 'arm' to disarm
   > arm
   System disarmed
```

### Status Display Format

```
ARM:Y TX:Y CHK:CRC8 CTR:5/5/5 ROD:1234 (23ms)
```

- **ARM:** System armed (Y/N)
- **TX:** Transmission active (Y/N)
- **CHK:** Checksum mode (CRC8 or SUM)
- **CTR:** Message counters for 0x38D/0x38B/0x38C
- **ROD:** Rod position value from 0x39D
- **Time:** Age of last rod position message in milliseconds

## Project Structure

```
ibooster-can-research-kit/
├── ibooster_can_controller/
│   └── ibooster_can_controller.ino  # Main Arduino sketch
├── docs/
│   └── CAN_MESSAGE_FORMAT.md        # CAN message documentation
├── README.md                         # This file
├── DISCLAIMER.md                     # Legal disclaimer
├── ACKNOWLEDGEMENTS.md               # Credits and references
├── LICENSE                           # GPLv3 license
└── .gitignore                        # Git ignore rules
```

## Documentation

See the `docs/` directory for detailed technical documentation:

- **CAN_MESSAGE_FORMAT.md** - Detailed CAN message specifications

## Development

### Building
1. Open `ibooster_can_controller/ibooster_can_controller.ino` in Arduino IDE
2. Select board: Arduino Mega2560
3. Select appropriate COM port
4. Click Upload

### Testing
- Use a CAN bus analyzer to verify message timing and format
- Monitor Serial output for rod position decoding
- Test all CLI commands before connecting to actual iBooster

## Safety and Legal

⚠️ **IMPORTANT SAFETY INFORMATION**

- This code is for **RESEARCH and BENCH TESTING ONLY**
- **NO WARRANTY** expressed or implied
- **NOT FOR USE ON PUBLIC ROADS** or in vehicles
- Improper use of brake systems can cause serious injury or death
- Always follow proper safety procedures when working with automotive components
- Disconnect from vehicle systems during testing
- Use proper grounding and isolation

See `DISCLAIMER.md` for complete legal disclaimer.

## License

This project is licensed under the GNU General Public License v3.0 - see the `LICENSE` file for details.

## Acknowledgements

See `ACKNOWLEDGEMENTS.md` for credits, references, and resources that made this project possible.

## Contributing

This is a research archive project. If you have improvements or findings:

1. Fork the repository
2. Create a feature branch
3. Submit a pull request with detailed description

## Support

This is an experimental research project with no official support. Use at your own risk.

For questions or discussion, please open an issue on GitHub.
