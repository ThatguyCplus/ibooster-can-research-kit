/*
  iBooster Gen2 YAW CAN - OPEN CONTROL + JOYSTICK (Mega2560 + MCP2515)
  100 Hz: 0x38B(4), 0x38C(4), 0x38D(7)
  Serial @ 500000: tx1/tx0 ext1/ext0 joy1/joy0 cal db N joydbg1/joydbg0 flow HEX hold status
  
  Monitors 0x39D (status) for driver_brake_apply signal:
    0 = not_init_or_off
    1 = brakes_not_applied
    2 = driver_applying_brakes  <- LED blinks when detected
    3 = fault
*/

#include <SPI.h>
#include <mcp2515.h>

// ---- CAN config ----
#define CS_PIN     4
#define CAN_SPEED  CAN_500KBPS
#define MCP_CLOCK  MCP_8MHZ   // MCP_16MHZ if your module is 16MHz
MCP2515 can(CS_PIN);

// ---- IO ----
static const uint8_t JOY_PIN = A0;
static const uint8_t LED_PIN = 13;  // Built-in LED on Mega2560

// ---- Flow config ----
static const uint16_t FLOW_NEUTRAL     = 0x7E00;
static const uint16_t FLOW_APPLY_MAX   = 0x9200;
static const uint16_t FLOW_RELEASE_MIN = 0x6A00;

// ---- State ----
static bool tx_enabled  = true;
static bool ext_request = true;
static bool joy_enabled = true;
static bool joy_debug   = false;

static uint16_t flow_manual = FLOW_NEUTRAL;
static uint16_t flow_cmd    = FLOW_NEUTRAL;
static uint8_t  cnt         = 0;

// 100 Hz scheduler
static uint32_t next_tick_us = 0;
static const uint32_t PERIOD_US = 10000;

// joystick (integer filter)
static int joy_raw = 512;
static int joy_filt = 512;
static int joy_center = 512;
static int joy_deadband = 30;   // ADC counts
static uint32_t last_joy_print_ms = 0;
static const uint16_t JOY_PRINT_MS = 50; // 20 Hz debug

// Driver brake detection (0x39D = 925 decimal)
#define CAN_ID_STATUS 0x39D  // Status message from iBooster
static uint8_t driver_brake_state = 0;  // Last decoded driver_brake_apply value
static uint32_t last_led_toggle_ms = 0;
static bool led_state = false;
static const uint16_t LED_BLINK_MS = 200;  // 200ms = 2.5 Hz blink rate

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

static inline void send_frame(uint16_t id, const uint8_t *data, uint8_t dlc) {
  struct can_frame f;
  f.can_id = id;
  f.can_dlc = dlc;
  for (uint8_t i = 0; i < dlc; i++) f.data[i] = data[i];
  can.sendMessage(&f);
}

static inline void send_crc_frame(uint16_t id, const uint8_t *payload, uint8_t plen) {
  uint8_t out[8];                 // enough for max DLC here
  out[0] = crc8_j1850(payload, plen);
  for (uint8_t i = 0; i < plen; i++) out[i + 1] = payload[i];
  send_frame(id, out, (uint8_t)(plen + 1));
}

static inline void send_triplet(uint8_t c, uint16_t flow) {
  const uint8_t reqcnt = (uint8_t)((ext_request ? 0x40 : 0x00) | (c & 0x0F));
  uint8_t p[6];

  // 0x38B: [REQ|cnt] 00 05
  p[0] = reqcnt; p[1] = 0x00; p[2] = 0x05;
  send_crc_frame(0x38B, p, 3);

  // 0x38C: [REQ|cnt] flow_LSB flow_MSB (little-endian)
  p[0] = reqcnt; p[1] = (uint8_t)(flow & 0xFF); p[2] = (uint8_t)(flow >> 8);
  send_crc_frame(0x38C, p, 3);

  // 0x38D: [cnt] 00 00 00 50 47
  p[0] = (uint8_t)(c & 0x0F); p[1] = 0x00; p[2] = 0x00; p[3] = 0x00; p[4] = 0x50; p[5] = 0x47;
  send_crc_frame(0x38D, p, 6);
}

// ---------- CAN Receive & Decode driver_brake_apply ----------
// Decode driver_brake_apply from 0x39D status message
// Signal: bit 16, length 2 bits, little-endian
// States: 0=not_init_or_off, 1=brakes_not_applied, 2=driver_applying_brakes, 3=fault
static void handle_can_receive() {
  struct can_frame frame;
  if (can.readMessage(&frame) == MCP2515::ERROR_OK) {
    if (frame.can_id == CAN_ID_STATUS && frame.can_dlc >= 3) {
      // Extract driver_brake_apply: bits 16-17 (byte 2, bits 0-1)
      // DBC: SG_ driver_brake_apply : 16|2@1+ (1,0) [0|3]
      driver_brake_state = frame.data[2] & 0x03;
    }
  }
}

static void update_led() {
  uint32_t now = millis();
  
  if (driver_brake_state == 2) {  // driver_applying_brakes
    // Blink LED
    if ((uint32_t)(now - last_led_toggle_ms) >= LED_BLINK_MS) {
      last_led_toggle_ms = now;
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state ? HIGH : LOW);
    }
  } else {
    // Turn off LED when not applying brakes
    if (led_state) {
      led_state = false;
      digitalWrite(LED_PIN, LOW);
    }
  }
}

// ---------- Joystick update + map ----------
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
    // APPLY
    int denom = (1023 - joy_center) - joy_deadband;
    if (denom < 1) denom = 1;
    uint32_t span = (uint32_t)(FLOW_APPLY_MAX - FLOW_NEUTRAL);
    uint32_t add  = (span * (uint32_t)(d - joy_deadband)) / (uint32_t)denom;
    return clamp_u16((uint16_t)(FLOW_NEUTRAL + add), FLOW_NEUTRAL, FLOW_APPLY_MAX);
  } else {
    // RELEASE
    int denom = joy_center - joy_deadband;
    if (denom < 1) denom = 1;
    uint32_t span = (uint32_t)(FLOW_NEUTRAL - FLOW_RELEASE_MIN);
    uint32_t sub  = (span * (uint32_t)((-d) - joy_deadband)) / (uint32_t)denom;
    return clamp_u16((uint16_t)(FLOW_NEUTRAL - sub), FLOW_RELEASE_MIN, FLOW_NEUTRAL);
  }
}

// ---------- Serial parsing ----------
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

static void cmd_status() {
  Serial.print(F("TX=")); Serial.print(tx_enabled ? F("ON") : F("OFF"));
  Serial.print(F(" ext=")); Serial.print(ext_request ? 1 : 0);
  Serial.print(F(" joy=")); Serial.print(joy_enabled ? 1 : 0);
  Serial.print(F(" flow=0x")); Serial.print(flow_cmd, HEX);
  Serial.print(F(" manual=0x")); Serial.print(flow_manual, HEX);
  Serial.print(F(" raw=")); Serial.print(joy_raw);
  Serial.print(F(" filt=")); Serial.print(joy_filt);
  Serial.print(F(" center=")); Serial.print(joy_center);
  Serial.print(F(" db=")); Serial.print(joy_deadband);
  Serial.print(F(" brake_state=")); Serial.print(driver_brake_state);
  Serial.print(F(" ("));
  switch (driver_brake_state) {
    case 0: Serial.print(F("not_init_or_off")); break;
    case 1: Serial.print(F("brakes_not_applied")); break;
    case 2: Serial.print(F("driver_applying_brakes")); break;
    case 3: Serial.print(F("fault")); break;
    default: Serial.print(F("unknown")); break;
  }
  Serial.println(F(")"));
}

static void handle_line(char *s) {
  char *cmd = strtok(s, " ");
  if (!cmd) return;

  if (!strcmp(cmd, "tx1"))  { tx_enabled = true;  Serial.println(F("TX ON")); return; }
  if (!strcmp(cmd, "tx0"))  { tx_enabled = false; Serial.println(F("TX OFF")); return; }

  if (!strcmp(cmd, "ext1")) { ext_request = true;  Serial.println(F("ext=1")); return; }
  if (!strcmp(cmd, "ext0")) { ext_request = false; Serial.println(F("ext=0")); return; }

  if (!strcmp(cmd, "joy1")) { joy_enabled = true;  Serial.println(F("joy=1")); return; }
  if (!strcmp(cmd, "joy0")) { joy_enabled = false; Serial.println(F("joy=0")); return; }

  if (!strcmp(cmd, "joydbg1")) { joy_debug = true;  Serial.println(F("joy_debug=1")); return; }
  if (!strcmp(cmd, "joydbg0")) { joy_debug = false; Serial.println(F("joy_debug=0")); return; }

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

  Serial.println(F("cmds: tx1 tx0 | ext1 ext0 | joy1 joy0 | cal | db N | joydbg1 joydbg0 | flow HEX | hold | status"));
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

  Serial.println(F("iBooster OPEN+JOYSTICK ready."));
  Serial.println(F("Monitoring 0x39D for driver brake input..."));
  Serial.println(F("Type: status | joydbg1 | cal"));
}

void loop() {
  handle_serial();

  // Check for incoming CAN messages (0x39D status)
  handle_can_receive();

  // Update LED based on driver brake state
  update_led();

  // update flow command (and capture joy_raw/joy_filt once)
  flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;

  // joystick debug @20Hz (CSV for Serial Plotter)
  if (joy_debug && (uint32_t)(millis() - last_joy_print_ms) >= JOY_PRINT_MS) {
    last_joy_print_ms += JOY_PRINT_MS;
    int delta = joy_filt - joy_center;
    Serial.print(joy_raw);   Serial.print(',');
    Serial.print(joy_filt);  Serial.print(',');
    Serial.print(joy_center);Serial.print(',');
    Serial.print(delta);     Serial.print(',');
    Serial.println((int)flow_cmd);
  }

  // 100Hz scheduler
  uint32_t now = micros();
  if ((int32_t)(now - next_tick_us) >= 0) {
    next_tick_us += PERIOD_US;
    if (tx_enabled) {
      send_triplet(cnt, flow_cmd);
      cnt = (uint8_t)((cnt + 1) & 0x0F);
    }
  }
}
