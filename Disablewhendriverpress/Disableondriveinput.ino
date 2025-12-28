/*
  iBooster Gen2 YAW CAN - OPEN CONTROL + JOYSTICK + JOYSTICK DEBUG (Mega2560 + MCP2515)

  Sends every 10ms (100 Hz) on Yaw CAN:
    0x38B DLC=4: [CRC] [REQ|cnt] 00 05
    0x38C DLC=4: [CRC] [REQ|cnt] flow_LSB flow_MSB   (little-endian flow)
    0x38D DLC=7: [CRC] [cnt]     00 00 00 50 47

  Joystick (VRy) on A0:
    Center -> HOLD (0x7E00)
    Above center -> APPLY (up to FLOW_APPLY_MAX)
    Below center -> RELEASE (down to FLOW_RELEASE_MIN)

  Serial @ 500000:
    tx1 / tx0            : enable/disable sending
    ext1 / ext0          : external_request on/off (0x40 bit in 0x38B/0x38C)
    joy1 / joy0          : enable/disable joystick control (joy overrides manual flow)
    cal                  : set joystick center to current position
    db 30                : set deadband in ADC counts (default 30)
    joydbg1 / joydbg0    : print joystick values at 20Hz (CSV for Serial Plotter)
    flow 7E00            : manual flow (only used when joy0)
    hold                 : manual hold (only used when joy0)
    status               : print current settings

  Debug output format (when joydbg1):
    raw,filt,center,delta,flow_cmd_dec

  Tip:
    Start this sending BEFORE powering the iBooster.
*/

#include <SPI.h>
#include <mcp2515.h>

// ---- CAN config ----
#define CS_PIN     4
#define CAN_SPEED  CAN_500KBPS
#define MCP_CLOCK  MCP_8MHZ   // change to MCP_16MHZ if your MCP2515 crystal is 16MHz
MCP2515 can(CS_PIN);

// ---- Joystick config ----
const uint8_t JOY_PIN = A0;

// ---- Flow config ----
static const uint16_t FLOW_NEUTRAL     = 0x7E00; // hold
static const uint16_t FLOW_APPLY_MAX   = 0x9200; // clamp apply
static const uint16_t FLOW_RELEASE_MIN = 0x6A00; // clamp release

// ---- State ----
static bool tx_enabled   = true;
static bool ext_request  = true;   // 0x40 bit in 0x38B/0x38C
static bool joy_enabled  = true;   // joystick overrides manual flow when true

static uint16_t flow_manual = FLOW_NEUTRAL; // used when joy_enabled=false
static uint16_t flow_cmd    = FLOW_NEUTRAL; // actual flow being sent

static uint8_t cnt = 0;

// 100Hz scheduler
static uint32_t next_tick_us = 0;
static const uint32_t PERIOD_US = 10000;

// joystick filtering/cal
static float joy_filt = 512.0f;
static int   joy_center = 512;
static int   joy_deadband = 30;     // ADC counts (0..1023)
static float joy_alpha = 0.20f;     // filter strength (0..1)

// joystick debug
static bool joy_debug = false;
static uint32_t last_joy_print_ms = 0;
static const uint16_t JOY_PRINT_MS = 50; // 50ms = 20 Hz

// ---------- CRC8 J1850 ----------
uint8_t crc8_j1850(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x1D);
      else           crc = (uint8_t)(crc << 1);
    }
  }
  return (uint8_t)(crc ^ 0xFF);
}

static inline void send_frame(uint16_t id, const uint8_t *data, uint8_t dlc) {
  struct can_frame frame;
  frame.can_id  = id;
  frame.can_dlc = dlc;
  for (uint8_t i = 0; i < dlc; i++) frame.data[i] = data[i];
  can.sendMessage(&frame);
}

void send_triplet(uint8_t c, uint16_t flow) {
  uint8_t reqcnt = (uint8_t)((ext_request ? 0x40 : 0x00) | (c & 0x0F));

  // 0x38B: [CRC] [REQ|cnt] 00 05
  {
    uint8_t payload[3] = { reqcnt, 0x00, 0x05 };
    uint8_t out[4]     = { crc8_j1850(payload, 3), payload[0], payload[1], payload[2] };
    send_frame(0x38B, out, 4);
  }

  // 0x38C: [CRC] [REQ|cnt] flow_LSB flow_MSB
  {
    uint8_t payload[3] = { reqcnt, (uint8_t)(flow & 0xFF), (uint8_t)((flow >> 8) & 0xFF) };
    uint8_t out[4]     = { crc8_j1850(payload, 3), payload[0], payload[1], payload[2] };
    send_frame(0x38C, out, 4);
  }

  // 0x38D: [CRC] [cnt] 00 00 00 50 47
  {
    uint8_t payload[6] = { (uint8_t)(c & 0x0F), 0x00, 0x00, 0x00, 0x50, 0x47 };
    uint8_t out[7]     = { crc8_j1850(payload, 6), payload[0], payload[1], payload[2], payload[3], payload[4], payload[5] };
    send_frame(0x38D, out, 7);
  }
}

// ---------- Joystick -> flow ----------
uint16_t joystick_to_flow() {
  int raw = analogRead(JOY_PIN);

  // low-pass filter
  joy_filt = joy_filt + joy_alpha * ((float)raw - joy_filt);
  int v = (int)(joy_filt + 0.5f);

  int d = v - joy_center; // signed delta
  if (abs(d) <= joy_deadband) return FLOW_NEUTRAL;

  if (d > joy_deadband) {
    int maxPos = 1023 - joy_center;
    if (maxPos < 1) maxPos = 1;
    int mag = d - joy_deadband;
    if (mag > maxPos) mag = maxPos;

    uint32_t span = (uint32_t)FLOW_APPLY_MAX - (uint32_t)FLOW_NEUTRAL;
    uint32_t add  = (span * (uint32_t)mag) / (uint32_t)maxPos;
    uint16_t f = (uint16_t)(FLOW_NEUTRAL + add);
    if (f > FLOW_APPLY_MAX) f = FLOW_APPLY_MAX;
    return f;
  } else {
    int maxNeg = joy_center;
    if (maxNeg < 1) maxNeg = 1;
    int mag = (-d) - joy_deadband;
    if (mag > maxNeg) mag = maxNeg;

    uint32_t span = (uint32_t)FLOW_NEUTRAL - (uint32_t)FLOW_RELEASE_MIN;
    uint32_t sub  = (span * (uint32_t)mag) / (uint32_t)maxNeg;
    uint16_t f = (uint16_t)(FLOW_NEUTRAL - sub);
    if (f < FLOW_RELEASE_MIN) f = FLOW_RELEASE_MIN;
    return f;
  }
}

// ---------- Serial parsing ----------
static char linebuf[64];
static uint8_t linelen = 0;

void cmd_status() {
  Serial.print(F("TX=")); Serial.print(tx_enabled ? F("ON") : F("OFF"));
  Serial.print(F(" ext=")); Serial.print(ext_request ? F("1") : F("0"));
  Serial.print(F(" joy=")); Serial.print(joy_enabled ? F("1") : F("0"));
  Serial.print(F(" flow_cmd=0x")); Serial.print(flow_cmd, HEX);
  Serial.print(F(" manual=0x")); Serial.print(flow_manual, HEX);
  Serial.print(F(" center=")); Serial.print(joy_center);
  Serial.print(F(" db=")); Serial.print(joy_deadband);
  Serial.print(F(" raw=")); Serial.print(analogRead(JOY_PIN));
  Serial.print(F(" filt=")); Serial.println((int)(joy_filt + 0.5f));
}

uint16_t parse_hex_u16(const char *s, bool *ok) {
  char *endp = nullptr;
  long v = strtol(s, &endp, 16);
  *ok = (endp != s) && (v >= 0) && (v <= 0xFFFF);
  return (uint16_t)v;
}

int parse_int(const char *s, bool *ok) {
  char *endp = nullptr;
  long v = strtol(s, &endp, 10);
  *ok = (endp != s);
  return (int)v;
}

void handle_line(char *s) {
  char *cmd = strtok(s, " ");
  if (!cmd) return;

  if (!strcmp(cmd, "tx1")) { tx_enabled = true;  Serial.println(F("TX ON")); return; }
  if (!strcmp(cmd, "tx0")) { tx_enabled = false; Serial.println(F("TX OFF")); return; }

  if (!strcmp(cmd, "ext1")) { ext_request = true;  Serial.println(F("ext=1")); return; }
  if (!strcmp(cmd, "ext0")) { ext_request = false; Serial.println(F("ext=0")); return; }

  if (!strcmp(cmd, "joy1")) { joy_enabled = true;  Serial.println(F("joy=1")); return; }
  if (!strcmp(cmd, "joy0")) { joy_enabled = false; Serial.println(F("joy=0")); return; }

  if (!strcmp(cmd, "joydbg1")) { joy_debug = true;  Serial.println(F("joy_debug=1")); return; }
  if (!strcmp(cmd, "joydbg0")) { joy_debug = false; Serial.println(F("joy_debug=0")); return; }

  if (!strcmp(cmd, "cal"))  { joy_center = (int)(joy_filt + 0.5f); Serial.println(F("center calibrated")); return; }

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

  if (!strcmp(cmd, "hold")) { flow_manual = FLOW_NEUTRAL; Serial.println(F("manual HOLD")); return; }

  if (!strcmp(cmd, "status")) { cmd_status(); return; }

  Serial.println(F("cmds: tx1 tx0 | ext1 ext0 | joy1 joy0 | cal | db N | joydbg1 joydbg0 | flow HEX | hold | status"));
}

void handle_serial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      linebuf[linelen] = 0;
      if (linelen > 0) handle_line(linebuf);
      linelen = 0;
    } else if (linelen < sizeof(linebuf) - 1) {
      linebuf[linelen++] = c;
    }
  }
}

void setup() {
  Serial.begin(500000);

  pinMode(JOY_PIN, INPUT);

  SPI.begin();
  can.reset();
  can.setBitrate(CAN_SPEED, MCP_CLOCK);
  can.setNormalMode();

  int r = analogRead(JOY_PIN);
  joy_filt = (float)r;
  joy_center = r;

  next_tick_us = micros();

  Serial.println(F("iBooster OPEN+JOYSTICK ready."));
  Serial.println(F("Type: status | joydbg1 | cal (center)"));
}

void loop() {
  handle_serial();

  // compute flow command
  flow_cmd = joy_enabled ? joystick_to_flow() : flow_manual;

  // debug print at 20Hz (does not affect 100Hz CAN scheduler)
  if (joy_debug && (uint32_t)(millis() - last_joy_print_ms) >= JOY_PRINT_MS) {
    last_joy_print_ms += JOY_PRINT_MS;

    int raw = analogRead(JOY_PIN);
    int filt = (int)(joy_filt + 0.5f);
    int delta = filt - joy_center;

    // CSV for Serial Plotter: raw,filt,center,delta,flow_cmd_dec
    Serial.print(raw);
    Serial.print(',');
    Serial.print(filt);
    Serial.print(',');
    Serial.print(joy_center);
    Serial.print(',');
    Serial.print(delta);
    Serial.print(',');
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
