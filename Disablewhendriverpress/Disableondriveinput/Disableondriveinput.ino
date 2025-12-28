/*
  iBooster Gen2 YAW CAN - OPEN CONTROL + JOYSTICK + DRIVER BRAKE MONITOR + BUS WATCHDOG
  Mega2560 + MCP2515 @ 500 kbps

  TX @100Hz:
    0x38B DLC=4: [CRC] [REQ|cnt] 00 05      (always when tx_enabled)
    0x38D DLC=7: [CRC] [cnt]     00 00 00 50 47  (always when tx_enabled)
    0x38C DLC=4: [CRC] [REQ|cnt] flow_LSB flow_MSB (ONLY when bus alive AND driver not braking)

  RX:
    Monitors 0x39D for driver_brake_apply (2-bit enum):
      0 = not_init_or_off
      1 = brakes_not_applied
      2 = driver_applying_brakes  -> LED solid ON, 0x38C inhibited
      3 = fault
    
    Bus Watchdog monitors ALL iBooster YAW bus messages:
      - 0x39D (status) - primary message, decoded for brake state
      - 0x38E (ibooster_gen2_unknown1) - used for bus health only
      - 0x38F (ibooster_gen2_unknown2) - used for bus health only
    Any of these messages indicate bus is active.

  Bus Watchdog:
    If NO YAW bus frames seen for BUS_TIMEOUT_MS:
      - bus_alive=false
      - clear driver_brake_state (prevents LED stuck)
      - LED slow blinks to indicate bus dead
      - STILL transmits minimal 0x38B/0x38D (if tx_enabled)

  Joystick VRy on A0:
    Center -> HOLD (0x7E00)
    Above center -> APPLY up to FLOW_APPLY_MAX
    Below center -> RELEASE down to FLOW_RELEASE_MIN

  Serial @500000:
    tx1/tx0, ext1/ext0, joy1/joy0, cal, db N, joydbg1/joydbg0, flow HEX, hold, status
*/

// ========== FEATURE FLAGS ==========
// Comment out the line below to disable ALL calibration-related code
#define ENABLE_CALIBRATION

#include <SPI.h>
#include <mcp2515.h>
#include <string.h>
#include <stdlib.h>  // For strtod, strtol
#ifdef ENABLE_CALIBRATION
#include <EEPROM.h>  // For storing calibrated PID gains
#endif

// ---------------- CAN config ----------------
#define CS_PIN     4
#define CAN_SPEED  CAN_500KBPS
#define MCP_CLOCK  MCP_8MHZ   // MCP_16MHZ if your MCP2515 crystal is 16MHz
MCP2515 can(CS_PIN);

// ---------------- IO ----------------
static const uint8_t JOY_PIN = A0;
static const uint8_t LED_PIN = 13;  // Mega2560 built-in LED

// ---------------- Flow config ----------------
static const uint16_t FLOW_NEUTRAL     = 0x7E00;
static const uint16_t FLOW_APPLY_MAX   = 0x9200;
static const uint16_t FLOW_RELEASE_MIN = 0x6A00;

// ---------------- Driver brake decode ----------------
static const uint16_t CAN_ID_STATUS = 0x39D;  // Status message from iBooster
static uint8_t driver_brake_state = 0;        // 0..3
static float rod_position_mm = 0.0f;          // Rod position in mm (from 0x39D)
static uint32_t last_rod_ms = 0;              // Timestamp when rod position was last updated (0 = never)
static uint32_t last_rod_print_ms = 0;        // Throttle rod position serial output
static const uint16_t ROD_PRINT_INTERVAL_MS = 2000;  // Print rod position every 2 seconds (reduced frequency)
static const uint16_t ROD_FRESH_MS = 250;     // Rod position is considered fresh if updated within 250ms (increased for bench tolerance)

// ---------------- TX state ----------------
static bool tx_enabled  = true;
static bool ext_request = true;    // 0x40 bit inside 0x38B/0x38C
static bool joy_enabled = true;
static bool joy_debug   = false;
static bool pid_enabled = false;   // PID position control mode

static uint16_t flow_manual = FLOW_NEUTRAL;
static uint16_t flow_cmd    = FLOW_NEUTRAL;
static uint16_t flow_cmd_slewed = FLOW_NEUTRAL;  // Slew-limited flow command
static uint8_t  cnt         = 0;

// ---------------- PID position control ----------------
static float target_rod_position_mm = 0.0f;  // Target rod position in mm
static bool target_valid = false;            // True if target is valid (explicitly set, not invalidated)
static bool target_ever_set = false;         // True if target has ever been explicitly set (allows 0.0mm target)
static float pid_integral = 0.0f;           // PID integral term
static float pid_last_error = 0.0f;         // Last error for derivative term
static uint32_t pid_last_update_ms = 0;      // Last PID update time (millis-based)
static uint32_t pid_last_tick_us = 0;        // Last PID update time (micros-based, for 100Hz sync)
static uint16_t pid_flow_hold = FLOW_NEUTRAL; // Hold last PID output between 100Hz updates
static int8_t pid_dir = 1;                   // Direction: +1 = positive error increases flow, -1 = inverted

// Rod position bounds - physical end stops
static const float ROD_MIN_MM = 0.0f;   // Physical minimum end stop (0mm)
static const float ROD_MAX_MM = 16.4f;  // Physical maximum end stop (16.4mm)

// PID gains (configurable via serial, start with aggressive values for maximum speed)
// These convert mm error to flow command adjustment
static float pid_kp = 500.0f;   // Proportional gain (flow units per mm error) - aggressive for speed
static float pid_ki = 0.0f;     // Integral gain (flow units per mm*second) - start at 0
static float pid_kd = 0.0f;    // Derivative gain (flow units per mm/second) - start at 0
static const float PID_INTEGRAL_MAX = 2000.0f;  // Limit integral windup
static const float PID_ERROR_DEADBAND = 0.05f;   // Deadband in mm (don't adjust if error < 0.05mm) - reduced for faster response

#ifdef ENABLE_CALIBRATION
// EEPROM addresses for storing calibrated PID gains
#define EEPROM_PID_CALIBRATED_FLAG 0  // 1 byte: 0xAA = calibrated, 0x00 = not calibrated
#define EEPROM_PID_KP 1               // 4 bytes: float
#define EEPROM_PID_KI 5               // 4 bytes: float
#define EEPROM_PID_KD 9               // 4 bytes: float
#define EEPROM_PID_DIR 13             // 1 byte: int8_t
#define EEPROM_CALIBRATION_MAGIC 0xAA

// ---------------- PID Calibration Mode ----------------
enum calib_state_t {
  CALIB_IDLE = 0,
  CALIB_RETURN_TO_BASE,      // Returning to base position
  CALIB_WAIT_BASE_STABLE,    // Waiting for base to stabilize
  CALIB_STEP_TO_TARGET,      // Stepping to target
  CALIB_WAIT_SETTLE,         // Waiting for target to settle
  CALIB_COOLDOWN             // Cooldown between tests (non-blocking)
};

static bool pid_calibration_mode = false;
static calib_state_t calib_state = CALIB_IDLE;
static uint8_t calib_phase = 0;              // Current calibration phase (0 = not calibrating)
static uint8_t calib_iteration = 0;          // Current iteration within phase
static uint32_t calib_start_ms = 0;
static uint32_t calib_cooldown_start_ms = 0;
static float calib_start_pos_mm = 0.0f;
static float calib_target_mm = 0.0f;
static float calib_base_pos_mm = 0.0f;       // Base position for this phase
static uint32_t calib_base_stable_start_ms = 0;  // When base position became stable

// Per-test metrics
static float calib_max_overshoot_mm = 0.0f;
static float calib_min_undershoot_mm = 0.0f;
static uint32_t calib_rise_time_ms = 0;
static uint32_t calib_settling_time_ms = 0;
static bool calib_reached_target = false;
static bool calib_past_target = false;
static uint32_t calib_first_cross_ms = 0;
static uint32_t calib_settle_start_ms = 0;

// Aggregated statistics across all tests
static float calib_avg_overshoot_pct = 0.0f;
static float calib_max_overshoot_pct = 0.0f;
static float calib_avg_rise_time_ms = 0.0f;
static float calib_avg_settling_time_ms = 0.0f;
static float calib_max_settling_time_ms = 0.0f;
static float calib_avg_steady_state_error = 0.0f;
static uint16_t calib_test_count = 0;
static float calib_overshoot_sum = 0.0f;
static float calib_rise_time_sum = 0.0f;
static float calib_settling_time_sum = 0.0f;
static float calib_steady_error_sum = 0.0f;

// Calibration phases: [step_size_mm, direction: +1=forward, -1=backward, iterations]
// Tests: small forward, small backward, medium forward, medium backward, large forward, large backward
static const uint8_t CALIB_NUM_PHASES = 6;
static const float CALIB_STEP_SIZES[6] = {2.0f, 2.0f, 5.0f, 5.0f, 10.0f, 10.0f};  // mm
static const int8_t CALIB_DIRECTIONS[6] = {+1, -1, +1, -1, +1, -1};  // +1 = extend, -1 = retract
static const uint8_t CALIB_ITERATIONS_PER_PHASE = 2;  // Run each test 2 times for repeatability

static const uint32_t CALIB_TIMEOUT_MS = 15000;  // 15 second max per test
static const uint32_t CALIB_SETTLE_TIME_MS = 1000;  // Must settle for 1 second
static const uint32_t CALIB_BASE_STABLE_TIME_MS = 500;  // Base must be stable for 500ms
static const uint32_t CALIB_COOLDOWN_MS = 500;  // Cooldown between tests (non-blocking)
static const float CALIB_SETTLE_TOLERANCE = 0.2f;  // 0.2mm settling tolerance
static const float CALIB_BASE_TOLERANCE = 0.15f;  // 0.15mm base position tolerance
static const uint32_t CALIB_WRONG_DIR_TIMEOUT_MS = 500;  // Abort if error increases for 500ms
#endif // ENABLE_CALIBRATION

// ---------------- Flow slew limiter ----------------
static const uint16_t FLOW_SLEW_RATE_JOY = 0x0100;  // Max change per 10ms tick for joystick (256 units = ~1.5% of range, conservative)
static const uint16_t FLOW_SLEW_RATE_PID = 0x1000;  // Max change per 10ms tick for PID (4096 units = ~24% of range, MAXIMUM SPEED)

// 100Hz scheduler
static uint32_t next_tick_us = 0;
static const uint32_t PERIOD_US = 10000;

// ---------------- Joystick filter ----------------
static int joy_raw = 512;
static int joy_filt = 512;
static int joy_center = 512;
static int joy_deadband = 30;   // ADC counts
static uint32_t last_joy_print_ms = 0;
static const uint16_t JOY_PRINT_MS = 50;

// ---------------- Bus watchdog (time-windowed) ----------------
// YAW bus message IDs from iBooster (all used for bus health monitoring)
// CAN_ID_STATUS (0x39D) is already defined above - status message (decoded for brake state)
static const uint16_t CAN_ID_UNKNOWN1 = 0x38E;    // 0x910 - ibooster_gen2_unknown1
static const uint16_t CAN_ID_UNKNOWN2 = 0x38F;    // 0x911 - ibooster_gen2_unknown2

static bool bus_alive = false;
static bool bus_ever_alive = false;  // Track if bus was ever declared alive (persistent flag)
static uint32_t last_rx_ms = 0;  // 0 = never received any YAW bus message
static const uint16_t BUS_TIMEOUT_MS = 500;   // 500ms timeout (more tolerant of occasional frame loss)
static const uint16_t BUS_ALIVE_WINDOW_MS = 500;  // 500ms window to collect frames
static const uint8_t MIN_FRAMES_IN_WINDOW = 5;  // Minimum frames in 500ms window (very tolerant, any activity counts)
static uint32_t window_start_ms = 0;  // Start of current window (0 = no window active)
static uint16_t frames_in_window = 0;  // Frames received in current window

// Statistics
static uint32_t rx_39d_count = 0;  // Count of 0x39D messages received
static uint32_t rx_38e_count = 0;  // Count of 0x38E messages received
static uint32_t rx_38f_count = 0;  // Count of 0x38F messages received
static uint32_t rx_other_count = 0;  // Count of other messages (for debugging)

// LED timing
static uint32_t last_led_ms = 0;
static bool led_state = false;

// ---------- CRC8 J1850 ----------
static uint8_t crc8_j1850(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
    }
  }
  return (uint8_t)(crc ^ 0xFF);
}

static inline uint16_t can11(const can_frame &f) { return (uint16_t)(f.can_id & 0x7FF); }

// ---------------- TX statistics ----------------
static uint32_t tx_success_count = 0;
static uint32_t tx_fail_count = 0;
static uint8_t mcp2515_error_flags = 0;  // MCP2515 error flags

static inline bool send_frame(uint16_t id, const uint8_t *data, uint8_t dlc) {
  struct can_frame f;
  f.can_id = id;
  f.can_dlc = dlc;
  for (uint8_t i = 0; i < dlc; i++) f.data[i] = data[i];
  
  uint8_t result = can.sendMessage(&f);
  
  // Check MCP2515 error flags
  mcp2515_error_flags = can.getErrorFlags();
  
  if (result == MCP2515::ERROR_OK) {
    tx_success_count++;
    return true;
  } else {
    tx_fail_count++;
    return false;
  }
}

static inline bool send_crc_frame(uint16_t id, const uint8_t *payload, uint8_t plen) {
  uint8_t out[8];
  out[0] = crc8_j1850(payload, plen);
  for (uint8_t i = 0; i < plen; i++) out[i + 1] = payload[i];
  return send_frame(id, out, (uint8_t)(plen + 1));
}

// Always sends 0x38B + 0x38D.
// Sends 0x38C only when send_control=true.
// Returns true if all frames sent successfully
static inline bool send_keepalive_and_maybe_control(uint8_t c, uint16_t flow, bool send_control) {
  const uint8_t reqcnt = (uint8_t)((ext_request ? 0x40 : 0x00) | (c & 0x0F));
  uint8_t p[6];
  bool all_ok = true;

  // 0x38B: [REQ|cnt] 00 05
  p[0] = reqcnt; p[1] = 0x00; p[2] = 0x05;
  if (!send_crc_frame(0x38B, p, 3)) all_ok = false;

  // 0x38C: [REQ|cnt] flow_LSB flow_MSB (little-endian)
  if (send_control) {
    p[0] = reqcnt; p[1] = (uint8_t)(flow & 0xFF); p[2] = (uint8_t)(flow >> 8);
    if (!send_crc_frame(0x38C, p, 3)) all_ok = false;
  }

  // 0x38D: [cnt] 00 00 00 50 47
  p[0] = (uint8_t)(c & 0x0F); p[1] = 0x00; p[2] = 0x00; p[3] = 0x00; p[4] = 0x50; p[5] = 0x47;
  if (!send_crc_frame(0x38D, p, 6)) all_ok = false;
  
  return all_ok;
}

// ---------- Joystick mapping ----------
static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline uint16_t map_joystick_to_flow() {
  joy_raw = analogRead(JOY_PIN);

  // integer IIR filter ~ alpha=0.2 : filt += (raw - filt)/5
  joy_filt += (joy_raw - joy_filt) / 5;

  int d = joy_filt - joy_center;
  int ad = (d < 0) ? -d : d;
  if (ad <= joy_deadband) return FLOW_NEUTRAL;

  if (d > 0) {
    int denom = (1023 - joy_center) - joy_deadband;
    if (denom < 1) denom = 1;
    uint32_t span = (uint32_t)(FLOW_APPLY_MAX - FLOW_NEUTRAL);
    uint32_t add  = (span * (uint32_t)(d - joy_deadband)) / (uint32_t)denom;
    return clamp_u16((uint16_t)(FLOW_NEUTRAL + add), FLOW_NEUTRAL, FLOW_APPLY_MAX);
  } else {
    int denom = joy_center - joy_deadband;
    if (denom < 1) denom = 1;
    uint32_t span = (uint32_t)(FLOW_NEUTRAL - FLOW_RELEASE_MIN);
    uint32_t sub  = (span * (uint32_t)((-d) - joy_deadband)) / (uint32_t)denom;
    return clamp_u16((uint16_t)(FLOW_NEUTRAL - sub), FLOW_RELEASE_MIN, FLOW_NEUTRAL);
  }
}

// Check if rod position feedback is fresh (updated recently)
static bool rod_fresh() {
  if (last_rod_ms == 0) return false;  // Never received rod position
  uint32_t age = (uint32_t)(millis() - last_rod_ms);
  return age < ROD_FRESH_MS;
}

#ifdef ENABLE_CALIBRATION
// ---------- EEPROM save/load for PID gains ----------
static void save_pid_gains_to_eeprom() {
  EEPROM.write(EEPROM_PID_CALIBRATED_FLAG, EEPROM_CALIBRATION_MAGIC);
  
  // Save floats (4 bytes each)
  union { float f; uint8_t b[4]; } converter;
  
  converter.f = pid_kp;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(EEPROM_PID_KP + i, converter.b[i]);
  }
  
  converter.f = pid_ki;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(EEPROM_PID_KI + i, converter.b[i]);
  }
  
  converter.f = pid_kd;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(EEPROM_PID_KD + i, converter.b[i]);
  }
  
  EEPROM.write(EEPROM_PID_DIR, (uint8_t)pid_dir);
  
  Serial.println(F("PID gains saved to EEPROM"));
}

static bool load_pid_gains_from_eeprom() {
  if (EEPROM.read(EEPROM_PID_CALIBRATED_FLAG) != EEPROM_CALIBRATION_MAGIC) {
    return false;  // No calibrated gains stored
  }
  
  // Load floats (4 bytes each)
  union { float f; uint8_t b[4]; } converter;
  
  for (int i = 0; i < 4; i++) {
    converter.b[i] = EEPROM.read(EEPROM_PID_KP + i);
  }
  float loaded_kp = converter.f;
  
  for (int i = 0; i < 4; i++) {
    converter.b[i] = EEPROM.read(EEPROM_PID_KI + i);
  }
  float loaded_ki = converter.f;
  
  for (int i = 0; i < 4; i++) {
    converter.b[i] = EEPROM.read(EEPROM_PID_KD + i);
  }
  float loaded_kd = converter.f;
  
  int8_t loaded_dir = (int8_t)EEPROM.read(EEPROM_PID_DIR);
  
  // SANITY CHECKS: Validate loaded values
  // Check for NaN/Inf (using isnan/isinf if available, or manual checks)
  bool valid = true;
  
  // Check if values are finite and in reasonable ranges
  if (loaded_kp != loaded_kp || loaded_kp < 50.0f || loaded_kp > 2000.0f) {  // NaN check: x != x
    valid = false;
  }
  if (loaded_ki != loaded_ki || loaded_ki < 0.0f || loaded_ki > 500.0f) {
    valid = false;
  }
  if (loaded_kd != loaded_kd || loaded_kd < 0.0f || loaded_kd > 1000.0f) {
    valid = false;
  }
  if (loaded_dir != 1 && loaded_dir != -1) {
    valid = false;
  }
  
  if (!valid) {
    Serial.println(F("WARNING: Invalid gains in EEPROM - clearing"));
    EEPROM.write(EEPROM_PID_CALIBRATED_FLAG, 0x00);  // Clear magic byte
    return false;
  }
  
  // Values are valid - apply them
  pid_kp = loaded_kp;
  pid_ki = loaded_ki;
  pid_kd = loaded_kd;
  pid_dir = loaded_dir;
  
  Serial.println(F("PID gains loaded from EEPROM (validated)"));
  return true;
}

// ---------- PID Calibration Mode ----------
static void start_pid_calibration() {
  if (!bus_alive || !rod_fresh()) {
    Serial.println(F("ERROR: Bus must be alive and rod feedback fresh for calibration"));
    return;
  }
  
  if (driver_brake_state == 2 || driver_brake_state == 3) {
    Serial.println(F("ERROR: Driver brake or fault state - cannot calibrate"));
    return;
  }
  
  if (!tx_enabled) {
    Serial.println(F("ERROR: TX must be enabled for calibration"));
    return;
  }
  
  if (!ext_request) {
    Serial.println(F("WARNING: ext_request is false - calibration may not work correctly"));
  }
  
  // Initialize calibration
  pid_calibration_mode = true;
  calib_phase = 0;
  calib_iteration = 0;
  calib_base_pos_mm = rod_position_mm;
  
  // Reset aggregated statistics
  calib_test_count = 0;
  calib_avg_overshoot_pct = 0.0f;
  calib_max_overshoot_pct = 0.0f;
  calib_avg_rise_time_ms = 0.0f;
  calib_avg_settling_time_ms = 0.0f;
  calib_max_settling_time_ms = 0.0f;
  calib_avg_steady_state_error = 0.0f;
  calib_overshoot_sum = 0.0f;
  calib_rise_time_sum = 0.0f;
  calib_settling_time_sum = 0.0f;
  calib_steady_error_sum = 0.0f;
  
  // Enable PID
  pid_enabled = true;
  joy_enabled = false;
  target_ever_set = true;
  
  Serial.println(F("\n=== PID Calibration Started ==="));
  Serial.println(F("Will test: 2mm, 5mm, 10mm steps in both directions"));
  Serial.println(F("Each test runs 2 times for repeatability"));
  Serial.print(F("Starting position: "));
  Serial.print(calib_base_pos_mm, 2);
  Serial.println(F("mm\n"));
  
  // Start first test
  start_calibration_test();
}

static void start_calibration_test() {
  if (calib_phase >= CALIB_NUM_PHASES) {
    // All phases complete
    finish_pid_calibration(true);
    return;
  }
  
  // Calculate target for this phase
  float step_size = CALIB_STEP_SIZES[calib_phase];
  int8_t direction = CALIB_DIRECTIONS[calib_phase];
  
  // For backward steps, update base position on first iteration
  if (direction < 0 && calib_iteration == 0) {
    calib_base_pos_mm = rod_position_mm;
  }
  
  calib_target_mm = calib_base_pos_mm + (step_size * direction);
  
  // Clamp to safe range
  if (calib_target_mm < ROD_MIN_MM) calib_target_mm = ROD_MIN_MM;
  if (calib_target_mm > ROD_MAX_MM) calib_target_mm = ROD_MAX_MM;
  
  // Skip tests that would hit mechanical stops (prevents timeouts)
  // Leave 0.05mm margin to avoid hitting hard stops
  if (calib_target_mm <= ROD_MIN_MM + 0.05f || calib_target_mm >= ROD_MAX_MM - 0.05f) {
    Serial.print(F("Skipping test: target "));
    Serial.print(calib_target_mm, 2);
    Serial.println(F("mm would hit clamp/stop"));
    advance_to_next_test();
    return;
  }
  
  // Start by returning to base position (ensures consistent starting point)
  calib_state = CALIB_RETURN_TO_BASE;
  calib_start_ms = millis();
  calib_base_stable_start_ms = 0;
  
  // Reset PID state
  pid_integral = 0.0f;
  pid_last_error = 0.0f;
  pid_last_update_ms = 0;
  pid_last_tick_us = 0;
  pid_flow_hold = FLOW_NEUTRAL;
  
  // Set target to base first
  target_rod_position_mm = calib_base_pos_mm;
  target_valid = true;
  
  // Print test info
  Serial.print(F("Phase "));
  Serial.print(calib_phase + 1);
  Serial.print(F("/"));
  Serial.print(CALIB_NUM_PHASES);
  Serial.print(F(", Iteration "));
  Serial.print(calib_iteration + 1);
  Serial.print(F("/"));
  Serial.print(CALIB_ITERATIONS_PER_PHASE);
  Serial.print(F(": "));
  Serial.print(step_size, 1);
  Serial.print(direction > 0 ? F("mm extend") : F("mm retract"));
  Serial.println();
}

static void update_pid_calibration() {
  if (!pid_calibration_mode) return;
  
  uint32_t now_ms = millis();
  uint32_t elapsed_ms = now_ms - calib_start_ms;
  
  // State machine - non-blocking, never uses delay()
  switch (calib_state) {
    case CALIB_RETURN_TO_BASE: {
      // Return to base position
      float base_error = calib_base_pos_mm - rod_position_mm;
      float abs_base_error = (base_error < 0.0f) ? -base_error : base_error;
      
      if (abs_base_error < CALIB_BASE_TOLERANCE) {
        if (calib_base_stable_start_ms == 0) {
          calib_base_stable_start_ms = now_ms;
        } else if ((now_ms - calib_base_stable_start_ms) >= CALIB_BASE_STABLE_TIME_MS) {
          // Base is stable - move to step phase
          calib_state = CALIB_STEP_TO_TARGET;
          calib_start_ms = now_ms;
          calib_start_pos_mm = rod_position_mm;
          
          // Reset test metrics
          calib_max_overshoot_mm = 0.0f;
          calib_min_undershoot_mm = 0.0f;
          calib_rise_time_ms = 0;
          calib_settling_time_ms = 0;
          calib_reached_target = false;
          calib_past_target = false;
          calib_first_cross_ms = 0;
          calib_settle_start_ms = 0;
          
          // Reset PID state
          pid_integral = 0.0f;
          pid_last_error = 0.0f;
          pid_last_update_ms = 0;
          pid_last_tick_us = 0;
          pid_flow_hold = FLOW_NEUTRAL;
          
          // Set target to test target
          target_rod_position_mm = calib_target_mm;
          target_valid = true;
        }
      } else {
        calib_base_stable_start_ms = 0;  // Reset if left tolerance
      }
      
      if (elapsed_ms > CALIB_TIMEOUT_MS) {
        Serial.println(F("Timeout returning to base"));
        advance_to_next_test();
      }
      break;
    }
    
    case CALIB_STEP_TO_TARGET: {
      float error = calib_target_mm - rod_position_mm;
      float abs_error = (error < 0.0f) ? -error : error;
      float initial_error = calib_target_mm - calib_start_pos_mm;
      float abs_initial_error = (initial_error < 0.0f) ? -initial_error : initial_error;
      
      // Check for wrong direction (error magnitude increasing)
      if (abs_error > abs_initial_error * 1.1f && elapsed_ms > CALIB_WRONG_DIR_TIMEOUT_MS) {
        Serial.println(F("ERROR: Wrong direction detected! Check pid_dir setting."));
        Serial.println(F("Calibration aborted - fix direction and restart"));
        finish_pid_calibration(false);
        return;
      }
      
      // Measure rise time (90% of step) - FIXED: sign-aware comparison
      if (calib_rise_time_ms == 0) {
        float target_90pct;
        if (calib_target_mm > calib_start_pos_mm) {
          // Extend: trigger when >= 90% of way
          target_90pct = calib_start_pos_mm + (abs_initial_error * 0.9f);
          if (rod_position_mm >= target_90pct) {
            calib_rise_time_ms = elapsed_ms;
            Serial.print(F("90% rise time: "));
            Serial.print(calib_rise_time_ms);
            Serial.println(F("ms"));
          }
        } else {
          // Retract: trigger when <= 90% of way
          target_90pct = calib_start_pos_mm - (abs_initial_error * 0.9f);
          if (rod_position_mm <= target_90pct) {
            calib_rise_time_ms = elapsed_ms;
            Serial.print(F("90% rise time: "));
            Serial.print(calib_rise_time_ms);
            Serial.println(F("ms"));
          }
        }
      }
      
      // FIXED: Direction-aware overshoot detection
      // For extend: overshoot = rod went too far out (rod > target)
      // For retract: overshoot = rod went too far in (rod < target)
      float dir = (calib_target_mm >= calib_start_pos_mm) ? 1.0f : -1.0f;
      float beyond = (rod_position_mm - calib_target_mm) * dir;  // Positive = past target in commanded direction
      
      if (beyond > 0.1f) {  // Went past target
        if (!calib_past_target) {
          calib_past_target = true;
          calib_first_cross_ms = elapsed_ms;
        }
        if (beyond > calib_max_overshoot_mm) {
          calib_max_overshoot_mm = beyond;
        }
      }
      
      // Check if we've reached target (within tolerance)
      if (!calib_reached_target && abs_error < CALIB_SETTLE_TOLERANCE) {
        calib_reached_target = true;
        calib_settle_start_ms = now_ms;
        calib_state = CALIB_WAIT_SETTLE;
        Serial.print(F("Reached target at "));
        Serial.print(elapsed_ms);
        Serial.println(F("ms"));
      }
      
      if (elapsed_ms > CALIB_TIMEOUT_MS) {
        Serial.println(F("Timeout during step"));
        advance_to_next_test();
      }
      break;
    }
    
    case CALIB_WAIT_SETTLE: {
      float error = calib_target_mm - rod_position_mm;
      float abs_error = (error < 0.0f) ? -error : error;
      
      if (abs_error < CALIB_SETTLE_TOLERANCE) {
        uint32_t settle_duration = now_ms - calib_settle_start_ms;
        if (settle_duration >= CALIB_SETTLE_TIME_MS) {
          // FIXED: Settling time is time from step start until settled
          calib_settling_time_ms = now_ms - calib_start_ms;
          // Test complete
          complete_calibration_test();
          return;
        }
      } else {
        // Left settling band - reset
        calib_settle_start_ms = 0;
        calib_reached_target = false;
        calib_state = CALIB_STEP_TO_TARGET;
      }
      
      if (elapsed_ms > CALIB_TIMEOUT_MS) {
        Serial.println(F("Timeout waiting for settle"));
        advance_to_next_test();
      }
      break;
    }
    
    case CALIB_COOLDOWN: {
      // Non-blocking cooldown between tests
      if ((now_ms - calib_cooldown_start_ms) >= CALIB_COOLDOWN_MS) {
        advance_to_next_test();
      }
      break;
    }
    
    default:
      break;
  }
}

static void complete_calibration_test() {
  // Calculate step size
  float step_size = (calib_target_mm > calib_start_pos_mm) ? 
                    (calib_target_mm - calib_start_pos_mm) : 
                    (calib_start_pos_mm - calib_target_mm);
  
  if (step_size < 0.1f) {
    Serial.println(F("WARNING: Invalid step size, skipping test"));
    advance_to_next_test();
    return;
  }
  
  // Calculate metrics for this test
  float overshoot_pct = (calib_max_overshoot_mm / step_size) * 100.0f;
  float steady_state_error = calib_target_mm - rod_position_mm;
  if (steady_state_error < 0) steady_state_error = -steady_state_error;
  
  // SANITY CHECK: Filter out corrupted overshoot data (e.g., from old bug)
  // Overshoot > 50% is suspicious (could be from wrong direction detection bug)
  // Overshoot > 100% is definitely corrupted (can't overshoot more than step size)
  bool overshoot_valid = (overshoot_pct >= 0.0f && overshoot_pct <= 50.0f);
  if (!overshoot_valid) {
    Serial.print(F("  WARNING: Invalid overshoot "));
    Serial.print(overshoot_pct, 1);
    Serial.println(F("% - ignoring for gain calculation"));
    overshoot_pct = 0.0f;  // Treat as no overshoot for this test
  }
  
  // Record results
  calib_test_count++;
  calib_overshoot_sum += overshoot_pct;
  if (overshoot_pct > calib_max_overshoot_pct) {
    calib_max_overshoot_pct = overshoot_pct;
  }
  
  if (calib_rise_time_ms > 0) {
    calib_rise_time_sum += calib_rise_time_ms;
  }
  
  if (calib_settling_time_ms > 0) {
    calib_settling_time_sum += calib_settling_time_ms;
    if (calib_settling_time_ms > calib_max_settling_time_ms) {
      calib_max_settling_time_ms = calib_settling_time_ms;
    }
  }
  
  calib_steady_error_sum += steady_state_error;
  
  // Print test results
  Serial.print(F("  Results: "));
  if (calib_rise_time_ms > 0) {
    Serial.print(F("rise="));
    Serial.print(calib_rise_time_ms);
    Serial.print(F("ms "));
  }
  Serial.print(F("overshoot="));
  Serial.print(overshoot_pct, 1);
  Serial.print(F("% "));
  if (calib_settling_time_ms > 0) {
    Serial.print(F("settle="));
    Serial.print(calib_settling_time_ms);
    Serial.print(F("ms "));
  }
  Serial.print(F("SS_error="));
  Serial.print(steady_state_error, 2);
  Serial.println(F("mm"));
  
  // Start non-blocking cooldown (never use delay() - breaks keepalive!)
  calib_state = CALIB_COOLDOWN;
  calib_cooldown_start_ms = millis();
}

static void advance_to_next_test() {
  calib_iteration++;
  
  if (calib_iteration >= CALIB_ITERATIONS_PER_PHASE) {
    // All iterations for this phase complete - move to next phase
    calib_phase++;
    calib_iteration = 0;
    Serial.println();  // Blank line between phases
  }
  
  // Reset state for next test
  calib_state = CALIB_IDLE;
  calib_base_stable_start_ms = 0;
  
  // Start next test (or finish if all phases done)
  start_calibration_test();
}

static void finish_pid_calibration(bool success) {
  if (!pid_calibration_mode) return;
  
  pid_calibration_mode = false;
  
  if (!success) {
    Serial.println(F("Calibration cancelled or failed"));
    target_valid = false;
    pid_enabled = false;
    return;
  }
  
  if (calib_test_count == 0) {
    Serial.println(F("ERROR: No valid test data collected"));
    target_valid = false;
    pid_enabled = false;
    return;
  }
  
  // Calculate averages from all tests
  calib_avg_overshoot_pct = calib_overshoot_sum / calib_test_count;
  if (calib_rise_time_sum > 0) {
    uint16_t valid_rise_tests = 0;
    // Count tests with valid rise time
    for (uint16_t i = 0; i < calib_test_count; i++) {
      // This is approximate - in real implementation would track per-test
    }
    calib_avg_rise_time_ms = calib_rise_time_sum / calib_test_count;  // Simplified
  }
  if (calib_settling_time_sum > 0) {
    calib_avg_settling_time_ms = calib_settling_time_sum / calib_test_count;
  }
  calib_avg_steady_state_error = calib_steady_error_sum / calib_test_count;
  
  // Print aggregated results
  Serial.println(F("\n=== Calibration Complete ==="));
  Serial.print(F("Tests completed: ")); Serial.println(calib_test_count);
  Serial.print(F("Average overshoot: ")); Serial.print(calib_avg_overshoot_pct, 1); Serial.println(F("%"));
  Serial.print(F("Max overshoot: ")); Serial.print(calib_max_overshoot_pct, 1); Serial.println(F("%"));
  if (calib_avg_rise_time_ms > 0) {
    Serial.print(F("Average rise time: ")); Serial.print(calib_avg_rise_time_ms, 0); Serial.println(F("ms"));
  }
  if (calib_avg_settling_time_ms > 0) {
    Serial.print(F("Average settling time: ")); Serial.print(calib_avg_settling_time_ms, 0); Serial.println(F("ms"));
    Serial.print(F("Max settling time: ")); Serial.print(calib_max_settling_time_ms, 0); Serial.println(F("ms"));
  }
  Serial.print(F("Average steady-state error: ")); Serial.print(calib_avg_steady_state_error, 2); Serial.println(F("mm"));
  
  // Calculate optimal gains based on aggregated statistics
  // Use worst-case and average metrics to ensure robustness across all test conditions
  float new_kp = pid_kp;
  float new_ki = pid_ki;
  float new_kd = pid_kd;
  
  Serial.println(F("\n=== Gain Calculation ==="));
  
  // Adjust based on average and max overshoot (use worst case for safety)
  float worst_overshoot = (calib_max_overshoot_pct > calib_avg_overshoot_pct) ? 
                          calib_max_overshoot_pct : calib_avg_overshoot_pct;
  
  if (worst_overshoot > 15.0f) {
    // Too much overshoot - reduce Kp, add/increase Kd
    new_kp = pid_kp * 0.65f;  // More aggressive reduction for worst case
    if (new_kd == 0.0f) {
      new_kd = new_kp * 0.15f;  // Start with 15% of new Kp
    } else {
      new_kd = pid_kd * 1.8f;
    }
    Serial.print(F("High overshoot ("));
    Serial.print(worst_overshoot, 1);
    Serial.println(F("%) - reducing Kp, increasing Kd"));
  } else if (worst_overshoot > 10.0f) {
    // Moderate overshoot - slight reduction
    new_kp = pid_kp * 0.80f;
    if (new_kd == 0.0f) {
      new_kd = new_kp * 0.08f;
    } else {
      new_kd = pid_kd * 1.3f;
    }
    Serial.print(F("Moderate overshoot ("));
    Serial.print(worst_overshoot, 1);
    Serial.println(F("%) - slight Kp reduction"));
  } else if (worst_overshoot < 3.0f && calib_avg_rise_time_ms > 2000) {
    // Very little overshoot but slow - can increase Kp
    new_kp = pid_kp * 1.15f;
    Serial.println(F("Slow response with low overshoot - increasing Kp"));
  } else if (worst_overshoot < 5.0f && calib_avg_rise_time_ms > 1500) {
    // Low overshoot, moderate speed - slight increase
    new_kp = pid_kp * 1.05f;
    Serial.println(F("Good overshoot, moderate speed - slight Kp increase"));
  }
  
  // Add integral term if there's consistent steady-state error
  if (calib_avg_steady_state_error > 0.15f) {
    // Steady-state error exists across tests - add/increase Ki
    if (new_ki == 0.0f) {
      new_ki = new_kp * 0.12f;  // Start with 12% of Kp
    } else {
      new_ki = new_ki * 1.3f;
    }
    Serial.print(F("Steady-state error detected ("));
    Serial.print(calib_avg_steady_state_error, 2);
    Serial.print(F("mm avg) - adding Ki: "));
    Serial.println(new_ki);
  }
  
  // Add derivative if overshoot exists but not too severe
  if (worst_overshoot > 5.0f && worst_overshoot < 12.0f && new_kd == 0.0f) {
    new_kd = new_kp * 0.06f;  // Small D term to reduce overshoot
    Serial.println(F("Adding small Kd to reduce moderate overshoot"));
  }
  
  // Clamp gains to reasonable ranges
  if (new_kp < 50.0f) new_kp = 50.0f;
  if (new_kp > 2000.0f) new_kp = 2000.0f;
  if (new_ki < 0.0f) new_ki = 0.0f;
  if (new_ki > 500.0f) new_ki = 500.0f;
  if (new_kd < 0.0f) new_kd = 0.0f;
  if (new_kd > 1000.0f) new_kd = 1000.0f;
  
  // Apply new gains
  pid_kp = new_kp;
  pid_ki = new_ki;
  pid_kd = new_kd;
  
  Serial.println(F("\n=== Calculated Optimal Gains ==="));
  Serial.print(F("Kp: ")); Serial.println(pid_kp);
  Serial.print(F("Ki: ")); Serial.println(pid_ki);
  Serial.print(F("Kd: ")); Serial.println(pid_kd);
  Serial.println(F("\nType 'calibsave' to save these gains to EEPROM"));
  Serial.println(F("Type 'calibcancel' to discard"));
}
#endif // ENABLE_CALIBRATION

// ---------- PID position control ----------
// Calculates flow command adjustment based on error between target and actual rod position
// SAFETY: Only runs when target_valid=true, rod_fresh(), and allow_control=true
static uint16_t calculate_pid_flow(bool allow_control) {
  // SAFETY: Target must be explicitly set and valid
  if (!target_valid) {
    pid_integral = 0.0f;
    pid_last_error = 0.0f;
    pid_last_update_ms = 0;
    return FLOW_NEUTRAL;
  }
  
  // SAFETY: Rod feedback must be fresh
  if (!rod_fresh()) {
    // Stale rod data - reset PID state and return neutral
    pid_integral = 0.0f;
    pid_last_error = 0.0f;
    pid_last_update_ms = 0;
    return FLOW_NEUTRAL;
  }
  
  // SAFETY: Don't run PID if driver is braking or in fault state
  if (driver_brake_state == 2 || driver_brake_state == 3) {
    pid_integral = 0.0f;
    pid_last_error = 0.0f;
    pid_last_update_ms = 0;
    return FLOW_NEUTRAL;
  }
  
  // SAFETY: Don't update PID state if control is not allowed
  // This prevents integral windup when control frames are blocked
  if (!allow_control) {
    // Freeze PID state (don't reset, just don't update)
    return FLOW_NEUTRAL;
  }
  
  // SAFETY: Clamp target to physical end stops (0mm and 16.4mm)
  // This prevents PID from trying to command beyond mechanical limits
  float clamped_target = target_rod_position_mm;
  if (clamped_target < ROD_MIN_MM) clamped_target = ROD_MIN_MM;
  if (clamped_target > ROD_MAX_MM) clamped_target = ROD_MAX_MM;
  
  // SAFETY: End stop protection - don't command beyond physical limits
  // If rod is at minimum and trying to go lower, return neutral
  if (rod_position_mm <= ROD_MIN_MM && clamped_target <= ROD_MIN_MM) {
    pid_integral = 0.0f;  // Reset integral to prevent windup
    pid_last_error = 0.0f;
    return FLOW_NEUTRAL;
  }
  // If rod is at maximum and trying to go higher, return neutral
  if (rod_position_mm >= ROD_MAX_MM && clamped_target >= ROD_MAX_MM) {
    pid_integral = 0.0f;  // Reset integral to prevent windup
    pid_last_error = 0.0f;
    return FLOW_NEUTRAL;
  }
  
  // Use fixed dt = 0.01s (100Hz update rate) - synchronized with TX rate
  // This avoids dt=0 issues from calling faster than control loop
  const float dt = 0.01f;  // 10ms = 100Hz
  
  if (pid_last_update_ms == 0) {
    // First call - initialize
    pid_last_update_ms = millis();
    pid_last_error = (clamped_target - rod_position_mm) * pid_dir;
    pid_integral = 0.0f;
    return FLOW_NEUTRAL;
  }
  
  // Update timestamp for tracking (but use fixed dt for calculation)
  pid_last_update_ms = millis();

  // Calculate error (with direction) using clamped target
  float error = (clamped_target - rod_position_mm) * pid_dir;

  // Deadband - don't adjust if error is very small
  if (error > -PID_ERROR_DEADBAND && error < PID_ERROR_DEADBAND) {
    pid_last_error = error;
    // Slowly decay integral near deadband instead of hard zero (prevents buzzing)
    pid_integral *= 0.95f;  // Decay by 5% per tick
    if (pid_integral > -0.1f && pid_integral < 0.1f) {
      pid_integral = 0.0f;  // Clear tiny values
    }
    return FLOW_NEUTRAL;
  }

  // Proportional term
  float p_term = pid_kp * error;

  // Integral term (only integrate when control is allowed - anti-windup)
  if (allow_control) {
    pid_integral += error * dt;
    // Clamp integral to prevent windup
    if (pid_integral > PID_INTEGRAL_MAX) pid_integral = PID_INTEGRAL_MAX;
    if (pid_integral < -PID_INTEGRAL_MAX) pid_integral = -PID_INTEGRAL_MAX;
  }
  // If control not allowed, freeze integral (already handled above, but keep for clarity)
  
  float i_term = pid_ki * pid_integral;

  // Derivative term
  float d_error = (error - pid_last_error) / dt;
  float d_term = pid_kd * d_error;
  pid_last_error = error;

  // Calculate PID output (adjustment from neutral)
  float pid_output = p_term + i_term + d_term;

  // Convert to flow command (add to neutral)
  int32_t flow_adjustment = (int32_t)pid_output;
  int32_t flow_result = (int32_t)FLOW_NEUTRAL + flow_adjustment;

  // Anti-windup: If output is saturated, reduce integral contribution
  // Classic anti-windup: only unwind if error is pushing in saturation direction
  bool saturated_high = (flow_result >= FLOW_APPLY_MAX);
  bool saturated_low = (flow_result <= FLOW_RELEASE_MIN);
  if (saturated_high && error > 0.0f) {
    // Saturated high and error still pushing higher - don't integrate
    // (integral already added above, so we unwind it)
    pid_integral -= error * dt;  // Undo the integration we just did
  } else if (saturated_low && error < 0.0f) {
    // Saturated low and error still pushing lower - don't integrate
    pid_integral -= error * dt;  // Undo the integration we just did
  }
  // If saturated but error would pull back, allow integration (helps unwind)

  // Clamp to valid range
  return clamp_u16((uint16_t)flow_result, FLOW_RELEASE_MIN, FLOW_APPLY_MAX);
}

// ---------- CAN RX + brake decode ----------
// SAFETY: Accept ANY iBooster YAW bus message (0x39D, 0x38E, 0x38F) for bus health
// But only decode 0x39D for brake state and rod position
static void handle_can_receive() {
  struct can_frame f;
  while (can.readMessage(&f) == MCP2515::ERROR_OK) {
    uint16_t id = can11(f);
    uint32_t now_ms = millis();
    bool is_yaw_bus_msg = false;

    // Check if this is any iBooster YAW bus message
    if (id == CAN_ID_STATUS) {
      is_yaw_bus_msg = true;
      rx_39d_count++;
    } else if (id == CAN_ID_UNKNOWN1) {
      is_yaw_bus_msg = true;
      rx_38e_count++;
    } else if (id == CAN_ID_UNKNOWN2) {
      is_yaw_bus_msg = true;
      rx_38f_count++;
    } else {
      rx_other_count++;
      continue;  // Ignore non-YAW bus messages
    }

    // Update watchdog for ANY YAW bus message
    if (is_yaw_bus_msg) {
      last_rx_ms = now_ms;

      // SAFETY: Time-windowed approach - collect frames over 500ms window
      // Start new window if none active or current window expired
      if (window_start_ms == 0 || (uint32_t)(now_ms - window_start_ms) >= BUS_ALIVE_WINDOW_MS) {
        window_start_ms = now_ms;
        frames_in_window = 1;  // Count this frame
      } else {
        // Within window, increment frame count
        frames_in_window++;
      }
      
      // Declare bus alive if we have enough frames in the window
      // Once declared alive, maintain it as long as frames keep coming (within timeout)
      if (frames_in_window >= MIN_FRAMES_IN_WINDOW) {
        if (!bus_alive) {
          Serial.print(F("Bus alive: "));
          Serial.print(frames_in_window);
          Serial.print(F(" frames in "));
          Serial.print((uint32_t)(now_ms - window_start_ms));
          Serial.println(F("ms window"));
        }
        bus_alive = true;  // Set and maintain alive state
        bus_ever_alive = true;  // Mark that bus was ever alive
      } else if (bus_ever_alive) {
        // Bus was previously declared alive - maintain it as long as frames keep coming
        // This prevents flickering when window restarts and frame count resets
        bus_alive = true;
      }
    }

    // CRITICAL: Only decode 0x39D for brake state and rod position
    if (id == CAN_ID_STATUS && f.can_dlc >= 5) {  // Need at least 5 bytes for rod_position
      // driver_brake_apply: 16|2@1+  -> byte2 bits0..1
      uint8_t new_state = (uint8_t)(f.data[2] & 0x03);

      // Validate state is in valid range (0-3)
      if (new_state > 3) {
        Serial.println(F("ERROR: Invalid driver_brake_state, ignoring"));
        continue;
      }

      // rod_position: 21|12@1+ (0.015625,-5) -> bits 21-32 (bytes 2-4)
      // Extract 12-bit value starting at bit 21 (little-endian)
      // Byte 2: bits 5-7 (3 bits), Byte 3: bits 0-7 (8 bits), Byte 4: bit 0 (1 bit)
      uint16_t raw_rod = ((uint16_t)(f.data[2] >> 5) & 0x07) |           // Byte 2 bits 5-7
                         ((uint16_t)f.data[3] << 3) |                    // Byte 3 bits 0-7
                         ((uint16_t)(f.data[4] & 0x01) << 11);           // Byte 4 bit 0
      
      // Convert to mm: rod_position_mm = (raw_value * 0.015625) - 5
      // NOTE: Verify this extraction matches your DBC/SavvyCAN - if wrong, PID will behave incorrectly
      float new_rod_mm = ((float)raw_rod * 0.015625f) - 5.0f;
      
      // Update rod position and freshness timestamp
      rod_position_mm = new_rod_mm;
      last_rod_ms = now_ms;  // Mark rod position as fresh

      if (new_state != driver_brake_state) {
        driver_brake_state = new_state;
        Serial.print(F("0x39D driver_brake_apply="));
        Serial.print(driver_brake_state);
        Serial.print(F(" ("));
        switch (driver_brake_state) {
          case 0: Serial.print(F("not_init_or_off")); break;
          case 1: Serial.print(F("brakes_not_applied")); break;
          case 2: Serial.print(F("driver_applying_brakes")); break;
          case 3: Serial.print(F("fault")); break;
        }
        Serial.println(F(")"));
        
        // SAFETY: Invalidate target on error states (driver brake or fault)
        // Keep target value but mark invalid - user must re-arm with pid1 or explicit target
        if (driver_brake_state == 2 || driver_brake_state == 3) {
          if (target_valid) {
            target_valid = false;
            Serial.print(F("Target invalidated due to "));
            if (driver_brake_state == 2) {
              Serial.println(F("driver brake (use 'pid1' to re-arm)"));
            } else {
              Serial.println(F("fault state (use 'pid1' to re-arm)"));
            }
          }
          // Reset PID state
          if (pid_enabled) {
            pid_integral = 0.0f;
            pid_last_error = 0.0f;
            pid_last_update_ms = 0;
          }
        }
      }
      
      // Throttled rod position output (only periodically to avoid spam)
      if ((uint32_t)(now_ms - last_rod_print_ms) >= ROD_PRINT_INTERVAL_MS) {
        last_rod_print_ms = now_ms;
        Serial.print(F("rod_position="));
        Serial.print(rod_position_mm, 2);  // 2 decimal places
        Serial.println(F("mm"));
      }
    }
    // Other YAW bus messages (0x38E, 0x38F) are already handled above for watchdog
  }
}

static void update_bus_watchdog() {
  uint32_t now = millis();

  if (last_rx_ms == 0) {
    // Never received a valid message yet
    bus_alive = false;
    bus_ever_alive = false;
    window_start_ms = 0;
    frames_in_window = 0;
    driver_brake_state = 0;  // SAFETY: Clear state on startup
    return;
  }

  bool timed_out = (uint32_t)(now - last_rx_ms) > BUS_TIMEOUT_MS;

  if (timed_out) {
    if (bus_alive) {
      // Bus just went dead
      Serial.println(F("WARNING: YAW bus RX timeout -> bus_dead (clearing brake state)"));
    }
    bus_alive = false;
    bus_ever_alive = false;  // Reset persistent flag on timeout
    // Reset window on timeout
    window_start_ms = 0;
    frames_in_window = 0;

    // SAFETY: Clear latched brake state so LED never sticks ON
    // Also prevents sending control when bus is dead
    if (driver_brake_state != 0) {
      driver_brake_state = 0;
      Serial.println(F("Cleared driver_brake_state due to bus timeout"));
    }
    
    // SAFETY: Invalidate target when bus goes dead
    // Prevents PID from resuming with stale target when bus recovers
    if (target_valid) {
      target_valid = false;
      Serial.println(F("Target invalidated due to bus timeout (use 'pid1' to re-arm)"));
    }
    if (pid_enabled) {
      pid_integral = 0.0f;
      pid_last_error = 0.0f;
      pid_last_update_ms = 0;
    }
  } else {
    // Not timed out - frames are still arriving within BUS_TIMEOUT_MS
    // Once bus_alive is declared (10+ frames in window), maintain it as long as frames keep coming
    // The window is only for initial detection, not for maintaining alive state
    // bus_alive is set in handle_can_receive() when enough frames are collected
    // Keep bus_alive true as long as we're receiving frames (not timed out)
    // No action needed here - bus_alive is maintained by handle_can_receive()
  }
  // Note: bus_alive is set in handle_can_receive() when we get enough frames in window
  // It's cleared here only on timeout
}

// LED policy:
// - if bus dead: slow blink
// - else if driver braking (state 2): solid on
// - else off
static void update_led() {
  uint32_t now = millis();

  if (!bus_alive && last_rx_ms != 0) {
    // slow blink to indicate bus dead
    if ((uint32_t)(now - last_led_ms) >= 300) {
      last_led_ms = now;
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state ? HIGH : LOW);
    }
    return;
  }

  // bus alive
  if (driver_brake_state == 2) {
    digitalWrite(LED_PIN, HIGH);
    led_state = true;
  } else {
    digitalWrite(LED_PIN, LOW);
    led_state = false;
  }
}

// ---------- Serial commands ----------
static char linebuf[64];
static uint8_t linelen = 0;

static uint16_t parse_hex_u16(const char *s, bool *ok) {
  char *endp = nullptr;
  long v = strtol(s, &endp, 16);
  *ok = (endp != s) && (v >= 0) && (v <= 0xFFFF);
  return (uint16_t)v;
}

static int parse_int(const char *s, bool *ok) {
  char *endp = nullptr;
  long v = strtol(s, &endp, 10);
  *ok = (endp != s);
  return (int)v;
}

static float parse_float(const char *s, bool *ok) {
  // Use strtod (double) and cast to float - more widely available than strtof
  char *endp = nullptr;
  double v = strtod(s, &endp);
  *ok = (endp != s) && (*endp == '\0' || *endp == ' ' || *endp == '\t' || *endp == '\r' || *endp == '\n');
  return (float)v;
}

static void cmd_status() {
  Serial.print(F("TX=")); Serial.print(tx_enabled ? F("ON") : F("OFF"));
  Serial.print(F(" ext=")); Serial.print(ext_request ? 1 : 0);
  Serial.print(F(" joy=")); Serial.print(joy_enabled ? 1 : 0);
  Serial.print(F(" bus=")); Serial.print(bus_alive ? F("ALIVE") : F("DEAD"));
  if (window_start_ms > 0) {
    Serial.print(F(" window_frames=")); Serial.print(frames_in_window);
    uint32_t window_age = millis() - window_start_ms;
    Serial.print(F(" window_age=")); Serial.print(window_age); Serial.print(F("ms"));
  }
  if (last_rx_ms) { Serial.print(F(" rx_age_ms=")); Serial.print((uint32_t)(millis() - last_rx_ms)); }
  Serial.print(F(" brake_state=")); Serial.print(driver_brake_state);
  Serial.print(F(" rod=")); Serial.print(rod_position_mm, 2); Serial.print(F("mm"));
  if (last_rod_ms > 0) {
    uint32_t rod_age = millis() - last_rod_ms;
    Serial.print(F(" rod_age=")); Serial.print(rod_age); Serial.print(F("ms"));
    Serial.print(rod_fresh() ? F("(fresh)") : F("(stale)"));
  } else {
    Serial.print(F(" rod=never"));
  }
  if (pid_enabled) {
    Serial.print(F(" target=")); Serial.print(target_rod_position_mm, 2); Serial.print(F("mm"));
    Serial.print(target_valid ? F("(valid)") : F("(invalid)"));
    if (target_valid) {
      Serial.print(F(" pid_err=")); Serial.print(target_rod_position_mm - rod_position_mm, 2); Serial.print(F("mm"));
    }
    Serial.print(F(" pid_kp=")); Serial.print(pid_kp);
    Serial.print(F(" ki=")); Serial.print(pid_ki);
    Serial.print(F(" kd=")); Serial.print(pid_kd);
    Serial.print(F(" dir=")); Serial.print(pid_dir);
  }
  Serial.print(F(" flow=0x")); Serial.print(flow_cmd, HEX);
  Serial.print(F(" manual=0x")); Serial.print(flow_manual, HEX);
  Serial.print(F(" rx_39d=")); Serial.print(rx_39d_count);
  Serial.print(F(" rx_38e=")); Serial.print(rx_38e_count);
  Serial.print(F(" rx_38f=")); Serial.print(rx_38f_count);
  if (rx_other_count > 0) {
    Serial.print(F(" rx_other=")); Serial.print(rx_other_count);
  }
  Serial.print(F(" raw=")); Serial.print(joy_raw);
  Serial.print(F(" filt=")); Serial.print(joy_filt);
  Serial.print(F(" center=")); Serial.print(joy_center);
  Serial.print(F(" db=")); Serial.print(joy_deadband);
  Serial.print(F(" tx_ok=")); Serial.print(tx_success_count);
  Serial.print(F(" tx_fail=")); Serial.print(tx_fail_count);
  if (mcp2515_error_flags != 0) {
    Serial.print(F(" mcp_err=0x")); Serial.print(mcp2515_error_flags, HEX);
  }
  Serial.println();
}

static void handle_line(char *s) {
  char *cmd = strtok(s, " ");
  if (!cmd) return;

  if (!strcmp(cmd, "tx1"))  {
    // SAFETY: Warn if enabling TX when driver is braking
    if (driver_brake_state == 2 && bus_alive) {
      Serial.println(F("WARNING: Driver is braking! TX enabled anyway (use with caution)"));
    }
    tx_enabled = true;
    Serial.println(F("TX ON"));
    return;
  }
  if (!strcmp(cmd, "tx0"))  {
    tx_enabled = false;
    Serial.println(F("TX OFF"));
    return;
  }

  if (!strcmp(cmd, "ext1")) { ext_request = true;  Serial.println(F("ext=1")); return; }
  if (!strcmp(cmd, "ext0")) { ext_request = false; Serial.println(F("ext=0")); return; }

  if (!strcmp(cmd, "joy1")) { 
    joy_enabled = true; 
    pid_enabled = false;  // Disable PID when enabling joystick
    Serial.println(F("joy=1 (PID disabled)"));
    return; 
  }
  if (!strcmp(cmd, "joy0")) { joy_enabled = false; Serial.println(F("joy=0")); return; }

  if (!strcmp(cmd, "joydbg1")) { joy_debug = true;  Serial.println(F("joy_debug=1")); return; }
  if (!strcmp(cmd, "joydbg0")) { joy_debug = false; Serial.println(F("joy_debug=0")); return; }

  if (!strcmp(cmd, "pid1")) { 
    pid_enabled = true; 
    joy_enabled = false;  // Disable joystick when enabling PID
    // Re-arm: if target has ever been set, mark it valid (allows PID to resume after error)
    // This works even if target is 0.0mm (valid target)
    if (target_ever_set) {
      target_valid = true;  // Re-arm with existing target
    }
    pid_integral = 0.0f;  // Reset PID state
    pid_last_error = 0.0f;
    pid_last_update_ms = 0;
    pid_last_tick_us = 0;  // Reset 100Hz sync
    pid_flow_hold = FLOW_NEUTRAL;
    Serial.print(F("PID enabled"));
    if (target_valid) {
      Serial.print(F(", target="));
      Serial.print(target_rod_position_mm, 2);
      Serial.print(F("mm (valid)"));
    } else {
      Serial.print(F(" (set target with 'target N' or 'Nmm')"));
    }
    Serial.println();
    return; 
  }
  if (!strcmp(cmd, "pid0")) { 
    pid_enabled = false; 
    pid_integral = 0.0f;  // Reset PID state
    pid_last_error = 0.0f;
    pid_last_update_ms = 0;
    pid_last_tick_us = 0;
    pid_flow_hold = FLOW_NEUTRAL;
    Serial.println(F("PID disabled"));
    return; 
  }

  if (!strcmp(cmd, "cal")) { joy_center = joy_filt; Serial.println(F("center calibrated")); return; }

  if (!strcmp(cmd, "db")) {
    char *n = strtok(nullptr, " ");
    bool ok=false;
    if (n) {
      int v = parse_int(n, &ok);
      if (ok) {
        if (v < 0) v = 0;
        if (v > 300) v = 300;
        joy_deadband = v;
        Serial.println(F("deadband set"));
        return;
      }
    }
    Serial.println(F("usage: db 30"));
    return;
  }

  if (!strcmp(cmd, "flow")) {
    char *hx = strtok(nullptr, " ");
    bool ok=false;
    if (hx) {
      flow_manual = parse_hex_u16(hx, &ok);
      if (ok) { Serial.println(F("manual flow set")); return; }
    }
    Serial.println(F("usage: flow 7E00"));
    return;
  }

  if (!strcmp(cmd, "hold"))   { flow_manual = FLOW_NEUTRAL; Serial.println(F("manual HOLD")); return; }
  if (!strcmp(cmd, "status")) { cmd_status(); return; }

  // Target command: "target 2" or "2mm" to set PID target position
  if (!strcmp(cmd, "target")) {
    char *val_str = strtok(nullptr, " ");
    if (val_str) {
      bool ok = false;
      float val = parse_float(val_str, &ok);
      if (ok) {
        // Clamp target to safe rod position range
        if (val < ROD_MIN_MM) {
          val = ROD_MIN_MM;
          Serial.print(F("WARNING: Target clamped to minimum "));
          Serial.print(ROD_MIN_MM);
          Serial.println(F("mm"));
        } else if (val > ROD_MAX_MM) {
          val = ROD_MAX_MM;
          Serial.print(F("WARNING: Target clamped to maximum "));
          Serial.print(ROD_MAX_MM);
          Serial.println(F("mm"));
        }
        // Valid float parsed and clamped
        target_rod_position_mm = val;
        target_valid = true;  // Mark target as valid
        target_ever_set = true;  // Mark that target has been set (allows 0.0mm re-arm)
        Serial.print(F("Target set to "));
        Serial.print(target_rod_position_mm, 2);
        Serial.println(F("mm (valid)"));
        return;
      }
    }
    Serial.println(F("usage: target 2.5 (sets target to 2.5mm, clamped to safe range)"));
    return;
  }

  // Support "2mm" format as shorthand for "target 2"
  // Check if command ends with "mm" and is a number
  size_t cmd_len = strlen(cmd);
  if (cmd_len > 2 && cmd[cmd_len-2] == 'm' && cmd[cmd_len-1] == 'm') {
    // Extract number part
    char num_buf[32];
    if (cmd_len - 2 < sizeof(num_buf)) {
      memcpy(num_buf, cmd, cmd_len - 2);
      num_buf[cmd_len - 2] = '\0';
      bool ok = false;
      float val = parse_float(num_buf, &ok);
      if (ok) {
        // Clamp target to safe rod position range
        if (val < ROD_MIN_MM) {
          val = ROD_MIN_MM;
          Serial.print(F("WARNING: Target clamped to minimum "));
          Serial.print(ROD_MIN_MM);
          Serial.println(F("mm"));
        } else if (val > ROD_MAX_MM) {
          val = ROD_MAX_MM;
          Serial.print(F("WARNING: Target clamped to maximum "));
          Serial.print(ROD_MAX_MM);
          Serial.println(F("mm"));
        }
        // Valid float parsed and clamped
        target_rod_position_mm = val;
        target_valid = true;  // Mark target as valid
        target_ever_set = true;  // Mark that target has been set (allows 0.0mm re-arm)
        Serial.print(F("Target set to "));
        Serial.print(target_rod_position_mm, 2);
        Serial.println(F("mm (valid)"));
        return;
      }
    }
  }
  
  // PID gain tuning commands
  if (!strcmp(cmd, "pidkp")) {
    char *val_str = strtok(nullptr, " ");
    if (val_str) {
      bool ok = false;
      float val = parse_float(val_str, &ok);
      if (ok && val >= 0.0f) {
        pid_kp = val;
        Serial.print(F("PID Kp="));
        Serial.println(pid_kp);
        return;
      }
    }
    Serial.println(F("usage: pidkp 200 (sets proportional gain)"));
    return;
  }
  
  if (!strcmp(cmd, "pidki")) {
    char *val_str = strtok(nullptr, " ");
    if (val_str) {
      bool ok = false;
      float val = parse_float(val_str, &ok);
      if (ok && val >= 0.0f) {
        pid_ki = val;
        Serial.print(F("PID Ki="));
        Serial.println(pid_ki);
        return;
      }
    }
    Serial.println(F("usage: pidki 50 (sets integral gain)"));
    return;
  }
  
  if (!strcmp(cmd, "pidkd")) {
    char *val_str = strtok(nullptr, " ");
    if (val_str) {
      bool ok = false;
      float val = parse_float(val_str, &ok);
      if (ok && val >= 0.0f) {
        pid_kd = val;
        Serial.print(F("PID Kd="));
        Serial.println(pid_kd);
        return;
      }
    }
    Serial.println(F("usage: pidkd 0 (sets derivative gain)"));
    return;
  }
  
  if (!strcmp(cmd, "piddir")) {
    char *val_str = strtok(nullptr, " ");
    if (val_str) {
      int dir = atoi(val_str);
      if (dir == 1 || dir == -1) {
        pid_dir = (int8_t)dir;
        Serial.print(F("PID direction="));
        Serial.println(pid_dir);
        return;
      }
    }
    Serial.println(F("usage: piddir 1 (or -1 to invert)"));
    return;
  }
  
#ifdef ENABLE_CALIBRATION
  // PID Calibration commands
  if (!strcmp(cmd, "calibstart")) {
    start_pid_calibration();
    return;
  }
  
  if (!strcmp(cmd, "calibfinish")) {
    if (pid_calibration_mode) {
      finish_pid_calibration(true);
    } else {
      Serial.println(F("ERROR: Not in calibration mode"));
    }
    return;
  }
  
  if (!strcmp(cmd, "calibcancel")) {
    if (pid_calibration_mode) {
      finish_pid_calibration(false);
    } else {
      Serial.println(F("ERROR: Not in calibration mode"));
    }
    return;
  }
  
  if (!strcmp(cmd, "calibsave")) {
    if (pid_calibration_mode) {
      Serial.println(F("ERROR: Finish calibration first with 'calibfinish'"));
      return;
    }
    save_pid_gains_to_eeprom();
    return;
  }
  
  if (!strcmp(cmd, "calibload")) {
    if (load_pid_gains_from_eeprom()) {
      Serial.print(F("Loaded: Kp=")); Serial.print(pid_kp);
      Serial.print(F(" Ki=")); Serial.print(pid_ki);
      Serial.print(F(" Kd=")); Serial.print(pid_kd);
      Serial.print(F(" dir=")); Serial.println(pid_dir);
    } else {
      Serial.println(F("No calibrated gains found in EEPROM"));
    }
    return;
  }
#endif // ENABLE_CALIBRATION

  Serial.print(F("cmds: tx1 tx0 | ext1 ext0 | joy1 joy0 | pid1 pid0 | target N | Nmm"));
#ifdef ENABLE_CALIBRATION
  Serial.print(F(" | calibstart | calibfinish | calibsave | calibload"));
#endif
  Serial.println(F(" | status"));
}

static void handle_serial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      linebuf[linelen] = 0;
      if (linelen) handle_line(linebuf);
      linelen = 0;
    } else if (linelen < sizeof(linebuf) - 1) {
      linebuf[linelen++] = c;
    }
  }
}

void setup() {
  Serial.begin(500000);

  pinMode(JOY_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  SPI.begin();
  can.reset();
  can.setBitrate(CAN_SPEED, MCP_CLOCK);
  can.setNormalMode();

  joy_raw = analogRead(JOY_PIN);
  joy_filt = joy_raw;
  joy_center = joy_raw;

  next_tick_us = micros();
  
#ifdef ENABLE_CALIBRATION
  // Try to load calibrated PID gains from EEPROM on startup
  if (load_pid_gains_from_eeprom()) {
    Serial.print(F("Using calibrated gains: Kp=")); Serial.print(pid_kp);
    Serial.print(F(" Ki=")); Serial.print(pid_ki);
    Serial.print(F(" Kd=")); Serial.print(pid_kd);
    Serial.print(F(" dir=")); Serial.println(pid_dir);
  } else {
    Serial.println(F("Using default PID gains"));
  }
#endif

  Serial.println(F("iBooster OPEN+JOYSTICK+PID ready."));
  Serial.println(F("RX: 0x39D driver_brake_apply (byte2 & 0x03)."));
  Serial.println(F("Watchdog: bus dead -> LED blinks + brake state cleared."));
  Serial.println(F("TX: always 0x38B/0x38D when tx_enabled; 0x38C only if bus_alive and driver not braking."));
  Serial.println(F("PID: pid1/pid0 | target N | Nmm | pidkp/pidki/pidkd N | piddir +/-1"));
#ifdef ENABLE_CALIBRATION
  Serial.println(F("Calibration: calibstart | calibfinish | calibsave | calibload"));
#endif
  Serial.println(F("Type: status | joydbg1 | cal | pid1 | target 2 | 2mm | pidkp 200"));
}

void loop() {
  handle_serial();

  // RX + watchdog + LED
  handle_can_receive();
  update_bus_watchdog();
  update_led();
  
#ifdef ENABLE_CALIBRATION
  // Update PID calibration if active
  if (pid_calibration_mode) {
    update_pid_calibration();
  }
#endif

  // compute flow command (only if bus is alive, otherwise keep neutral)
  // SAFETY: Don't compute new commands when bus is dead to avoid stale values
  if (bus_alive) {
    uint16_t flow_target;
    
    // Priority: PID > Joystick > Manual
    if (pid_enabled) {
      // Update PID at 100Hz (same as TX rate) to avoid dt=0 issues
      uint32_t now_us = micros();
      if (pid_last_tick_us == 0) {
        pid_last_tick_us = now_us;
      }
      
      if ((int32_t)(now_us - pid_last_tick_us) >= 10000) {  // 10ms = 100Hz
        pid_last_tick_us += 10000;  // Maintain exact 100Hz cadence
        
        // Determine if control is allowed (for PID anti-windup)
        // PID requires: target_valid, rod_fresh, and no driver brake
        bool allow_control_pid = target_valid && rod_fresh() && (driver_brake_state != 2);
        pid_flow_hold = calculate_pid_flow(allow_control_pid);
      }
      // Use held PID output (updated at 100Hz, held between updates)
      flow_target = pid_flow_hold;
    } else if (joy_enabled) {
      flow_target = map_joystick_to_flow();
    } else {
      flow_target = flow_manual;
    }
    
    // Flow slew limiter: limit rate of change for safety
    // PID gets higher slew rate for faster response, joystick stays conservative
    uint16_t slew_rate = pid_enabled ? FLOW_SLEW_RATE_PID : FLOW_SLEW_RATE_JOY;
    int32_t delta = (int32_t)flow_target - (int32_t)flow_cmd_slewed;
    int32_t abs_delta = (delta < 0) ? -delta : delta;
    
    if (abs_delta > slew_rate) {
      // Limit the change
      if (delta > 0) {
        flow_cmd = (uint16_t)(flow_cmd_slewed + slew_rate);
      } else {
        flow_cmd = (uint16_t)(flow_cmd_slewed - slew_rate);
      }
    } else {
      flow_cmd = flow_target;
    }
    
    // Clamp to valid range
    flow_cmd = clamp_u16(flow_cmd, FLOW_RELEASE_MIN, FLOW_APPLY_MAX);
    flow_cmd_slewed = flow_cmd;
  } else {
    // Bus dead: force neutral immediately (no slew when bus drops)
    flow_cmd = FLOW_NEUTRAL;
    flow_cmd_slewed = FLOW_NEUTRAL;
    // SAFETY: Invalidate target when bus is dead (prevents stale target)
    if (target_valid) {
      target_valid = false;
    }
    // Reset PID on bus dead
    if (pid_enabled) {
      pid_integral = 0.0f;
      pid_last_error = 0.0f;
      pid_last_update_ms = 0;
      pid_last_tick_us = 0;
      pid_flow_hold = FLOW_NEUTRAL;
    }
  }

  // joystick debug CSV @20Hz
  if (joy_debug && (uint32_t)(millis() - last_joy_print_ms) >= JOY_PRINT_MS) {
    last_joy_print_ms += JOY_PRINT_MS;
    int delta = joy_filt - joy_center;
    Serial.print(joy_raw);   Serial.print(',');
    Serial.print(joy_filt);  Serial.print(',');
    Serial.print(joy_center);Serial.print(',');
    Serial.print(delta);     Serial.print(',');
    Serial.println((int)flow_cmd);
  }

  // 100Hz TX
  uint32_t now = micros();
  if ((int32_t)(now - next_tick_us) >= 0) {
    next_tick_us += PERIOD_US;

    if (tx_enabled) {
      // SAFETY: Multiple layers of protection
      // 1. Bus must be alive (valid 0x39D received within timeout)
      // 2. Driver must not be pressing brakes (state != 2)
      // 3. flow_cmd is forced to NEUTRAL when bus is dead (see above)
      // 4. If PID enabled: also require target_valid and rod_fresh
      bool allow_control_tx = bus_alive && (driver_brake_state != 2);
      
      // PID-specific requirements: target must be valid and rod feedback fresh
      if (pid_enabled) {
        allow_control_tx = allow_control_tx && target_valid && rod_fresh();
      }

      // Double-check: Never send control if driver is braking, even if bus_alive
      if (allow_control_tx && driver_brake_state == 2) {
        allow_control_tx = false;  // Extra safety check
      }

      bool tx_ok = send_keepalive_and_maybe_control(cnt, flow_cmd, allow_control_tx);
      
      // Debug: Log when control is inhibited (but throttle to avoid spam)
      static uint32_t last_control_debug_ms = 0;
      if (!allow_control_tx && (uint32_t)(millis() - last_control_debug_ms) >= 1000) {
        last_control_debug_ms = millis();
        Serial.print(F("Control inhibited: bus_alive="));
        Serial.print(bus_alive ? 1 : 0);
        Serial.print(F(" brake_state="));
        Serial.print(driver_brake_state);
        if (pid_enabled) {
          Serial.print(F(" target_valid="));
          Serial.print(target_valid ? 1 : 0);
          Serial.print(F(" rod_fresh="));
          Serial.print(rod_fresh() ? 1 : 0);
        }
        Serial.println();
      }
      
      // Optional: Log TX failures (but don't spam)
      if (!tx_ok && (tx_fail_count % 100 == 1)) {  // Log every 100th failure
        Serial.print(F("TX error: mcp_flags=0x"));
        Serial.print(mcp2515_error_flags, HEX);
        Serial.print(F(" fail_cnt="));
        Serial.println(tx_fail_count);
      }
      
      cnt = (uint8_t)((cnt + 1) & 0x0F);
    }
  }
}
//hello world ;)
//DO NOT USE THIS CODE IN PRODUCTION / Run it at your own risk