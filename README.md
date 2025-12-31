# ibooster-can-research-kit

CAN research + bench-control experiments for Bosch iBooster Gen2 (notes, logs, code, and references). This is a "one place" archive of what I learned and what sources helped. No warranty. Experimental. Do not use on public roads.

---

> **SAFETY / LEGAL DISCLAIMER (READ FIRST)**
>
> - **EXPERIMENTAL RESEARCH ONLY. BENCH USE ONLY.**
> - **NOT safety-certified. NOT validated for public-road use.**
> - Braking is safety-critical. A mistake can cause injury, death, or property damage.
> - **NO WARRANTY. NO LIABILITY.** You assume all risk for use, misuse, or modification.
> - Do not use this repo to operate a vehicle on public roads.
> - Respect laws, regulations, and responsible disclosure.

---

## What This Repository Contains

### ✅ Main Implementation: `Disablewhendriverpress/Disableondriveinput/Disableondriveinput.ino`

A complete, production-ready Arduino sketch for bench control of a Bosch iBooster Gen2 over CAN. This is the **primary working code** with extensive safety features.

#### Core Features

**CAN Communication (YAW Bus @ 500 kbps)**
- **TX @ 100Hz**: Sends control frames `0x38B`, `0x38C`, `0x38D` with CRC8 J1850 checksum
- **RX**: Monitors `0x39D` (status), `0x38E`, `0x38F` for bus health and rod position
- **Bus Watchdog**: Monitors all YAW bus messages with time-windowed health detection
  - Tolerates frame loss (requires 5+ frames in 500ms window)
  - Automatically inhibits control when bus goes dead
  - Continues sending minimal keep-alive frames even when bus is dead

**Safety Systems**
- **Driver Brake Detection**: Decodes `driver_brake_apply` from `0x39D` and automatically inhibits control when driver applies brakes
- **Rod Position Feedback**: Extracts 12-bit rod position from `0x39D` and converts to millimeters (0mm to 16.4mm)
- **End Stop Protection**: Hard limits at 0mm (minimum) and 16.4mm (maximum) to prevent commanding beyond physical limits
- **Target Validation**: Explicit `target_valid` flag prevents PID from auto-resuming after safety events
- **Rod Freshness Check**: PID only operates when rod feedback is fresh (< 250ms old)
- **Slew Rate Limiting**: Separate limits for joystick (`0x0100`) and PID (`0x1000`) to prevent sudden changes
- **TX Gating**: Multiple layers of checks before allowing control frames

**Control Modes**

1. **Manual Flow Control**: Direct hexadecimal flow command via serial
2. **Joystick Control**: Analog joystick on pin A0 (VRy) for real-time control
   - Center position = HOLD (neutral)
   - Above center = APPLY (extend rod)
   - Below center = RELEASE (retract rod)
   - Configurable deadband and center calibration
3. **PID Position Control**: Closed-loop position control targeting specific rod positions in millimeters
   - Configurable gains (Kp, Ki, Kd) via serial commands
   - Direction inversion support (`piddir 1` / `piddir -1`)
   - Anti-windup protection with directional unwinding
   - Integral decay near deadband to prevent buzzing
   - Synchronized to 100Hz update rate

**PID Calibration Mode** (Optional - can be disabled via `#define ENABLE_CALIBRATION`)
- Non-blocking, multi-phase state machine
- Tests multiple step sizes (2mm, 5mm, 10mm) in both extend and retract directions
- Measures rise time, overshoot, settling time, and steady-state error
- Automatically calculates optimal PID gains from aggregated statistics
- Stores calibrated gains in EEPROM with sanity checks
- Skips tests that would hit mechanical stops
- Detects wrong direction and aborts if `pid_dir` is incorrect

**Serial CLI** (@ 500000 baud)
- `tx1` / `tx0` - Enable/disable CAN transmission
- `ext1` / `ext0` - Enable/disable external request bit
- `joy1` / `joy0` - Enable/disable joystick control
- `joydbg1` / `joydbg0` - Enable/disable joystick debug output
- `cal` - Calibrate joystick center position
- `db N` - Set joystick deadband (0-300)
- `flow HEX` - Set manual flow command (e.g., `flow 7E00`)
- `hold` - Set flow to neutral (HOLD)
- `pid1` / `pid0` - Enable/disable PID control
- `target N` - Set PID target position in mm (e.g., `target 5.2`)
- `Nmm` - Shorthand for target (e.g., `5mm` = `target 5`)
- `pidkp N` / `pidki N` / `pidkd N` - Set PID gains
- `piddir 1` / `piddir -1` - Set PID direction
- `status` - Display comprehensive system status
- `calibstart` - Start PID calibration (if enabled)
- `calibfinish` - Finish and calculate optimal gains
- `calibcancel` - Cancel calibration
- `calibsave` - Save calibrated gains to EEPROM
- `calibload` - Load calibrated gains from EEPROM

**Status Display** (`status` command)
- Bus health (alive/dead, frame counts per ID)
- Driver brake state
- Rod position (mm)
- TX/RX enable states
- PID state (enabled, target, error, gains)
- Control mode priority (PID > Joystick > Manual)
- Flow command values

### 📁 Additional Files

**Reference Materials**
- `NotMadebyMebutwerereallyhelpfultoolsinoneplace/ibooster_gen_2_tesla_model_3_right_hand_drive (1).dbc` - DBC file for reference (not used directly in code)
- `NotMadebyMebutwerereallyhelpfultoolsinoneplace/ibooster EXTERNAL_BRAKE_REQUEST.csv` - Reference CSV for CAN message structure

**Legacy/Proof-of-Concept** (Not Recommended)
- `Disablewhendriverpress/DONOTUSEcrudeproofofconcept/CrudeControlNOSAFTEYJoystickModel.ino` - Early proof-of-concept without safety features

---

## Quick Start (Bench Setup)

### Hardware Requirements
- **Arduino Mega2560** (or compatible)
- **1× MCP2515 CAN module** (8MHz or 16MHz crystal - configure in code)
- **Analog joystick** (optional, for joystick control mode)
- **120Ω termination resistors** (one at each end of CAN bus - iBooster does not terminate internally)

### Wiring
- **MCP2515**: CS=4, INT=3 (configurable in code)
- **Joystick**: VRy connected to A0 (optional)
- **LED**: Built-in LED on pin 13 (shows driver brake state and bus health)
- **CAN Bus**: Connect CANH/CANL with proper termination

### Software Setup

1. **Install Dependencies**
   - Arduino IDE or PlatformIO
   - `mcp2515` library (standard Arduino library)
   - Standard libraries: `SPI`, `string.h`, `stdlib.h`
   - Optional: `EEPROM.h` (only if `ENABLE_CALIBRATION` is defined)

2. **Configure Hardware**
   - Set `MCP_CLOCK` to `MCP_8MHZ` or `MCP_16MHZ` based on your MCP2515 crystal
   - Adjust `CS_PIN` and interrupt pin if needed
   - Set serial baud rate to 500000 (or adjust in code)

3. **Optional: Disable Calibration Mode**
   - Comment out `#define ENABLE_CALIBRATION` near the top of the file to remove all calibration-related code

4. **Upload and Connect**
   - Upload sketch to Arduino
   - Open serial monitor at 500000 baud
   - Type `status` to see system state
   - Type `help` or any invalid command to see available commands

### Basic Usage

1. **Enable Control**
   ```
   tx1        # Enable CAN transmission
   ext1       # Enable external request
   ```

2. **Check Status**
   ```
   status     # View system state, bus health, rod position
   ```

3. **Control Rod Position**
   - **Manual**: `flow 7E00` (neutral), `flow 9200` (apply), `flow 6A00` (release)
   - **Joystick**: `joy1` then move joystick
   - **PID**: `pid1`, `target 5.2` (go to 5.2mm), `5mm` (shorthand)

4. **Monitor Safety**
   - LED solid ON = driver brake detected (control inhibited)
   - LED slow blink = bus dead (no YAW messages received)
   - LED off = normal operation

---

## Safety Features

This implementation includes multiple layers of safety:

1. **Bus Watchdog**: Stops control if CAN bus goes silent
2. **Driver Brake Detection**: Automatically inhibits control when driver applies brakes
3. **End Stop Protection**: Prevents commanding beyond 0mm (min) and 16.4mm (max)
4. **Target Validation**: PID target invalidated on safety events (driver brake, fault, bus dead)
5. **Rod Freshness**: PID only runs with fresh feedback (< 250ms old)
6. **Slew Rate Limiting**: Prevents sudden flow command changes
7. **TX Gating**: Multiple checks before allowing control frames
8. **Integral Anti-Windup**: Prevents PID integral from accumulating when control is blocked
9. **Direction Detection**: Calibration can detect if `pid_dir` is wrong and abort

---

## Technical Details

### CAN Message IDs (YAW Bus)
- **0x38B**: Control request frame (DLC=4)
- **0x38C**: Flow command frame (DLC=4) - only sent when bus alive and driver not braking
- **0x38D**: Control configuration frame (DLC=7)
- **0x39D**: Status/feedback frame (DLC=8) - contains rod position and driver brake state
- **0x38E**: Unknown iBooster message (monitored for bus health)
- **0x38F**: Unknown iBooster message (monitored for bus health)

### Rod Position Encoding
- 12-bit value in `0x39D` (bits 21-32, little-endian)
- Range: 0mm to 16.4mm (physical end stops)
- Resolution: ~0.004mm per bit

### Flow Command Encoding
- Neutral: `0x7E00` (HOLD - no movement)
- Apply: `0x7E00` to `0x9200` (extend rod)
- Release: `0x7E00` to `0x6A00` (retract rod)

### PID Control
- Update rate: 100Hz (synchronized with TX rate)
- Default gains: Kp=500.0, Ki=0.0, Kd=0.0 (P-only, aggressive for speed)
- Deadband: 0.05mm (no adjustment if error < deadband)
- Integral limit: ±2000.0 (prevents windup)
- Slew rate: 4096 units/10ms (faster than joystick for responsive control)

---

## What This Repository Is *Not*

- Not an OEM DBC file (reference DBC included but not used directly)
- Not a complete reverse-engineering of every iBooster signal
- Not a production brake controller
- Not a "plug-and-play" automotive braking solution
- Not validated for use on public roads

This is **research + experimentation**, written to be readable, verifiable, and safe for bench use.

---

## Attribution / Credits

This project is built from **many public sources** (forums, open repos, documentation, tool authors, and community discoveries). Credits and acknowledgements:

- https://github.com/open-vehicle-control-system/dbc/tree/main/ibooster
- https://openinverter.org/forum/viewtopic.php?t=4997
- https://github.com/open-vehicle-control-system
- https://github.com/RetroPilot/ocelot/tree/main

### Contributing

If you contributed ideas or code, open an issue/PR and I'll add you to acknowledgements. If anything is missing or attributed incorrectly, please tell me — I want this to be respectful and accurate.

---

## License

This repo uses a copyleft license so improvements remain open.  
See `LICENSE` for details.

---

## Final Warning (Seriously)

Brakes can hurt people. Don't use experimental code on public roads.  
Bench-test with safe setups:
- Current limiting
- Fusing
- Physical restraints
- Emergency stop (E-stop)
- Proper CAN bus termination
- Isolated power supplies
- Clear workspace with no obstructions

**This code is for research and educational purposes only.**
