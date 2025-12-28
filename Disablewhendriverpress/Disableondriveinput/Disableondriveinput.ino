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

  Bus Watchdog:
    If NO RX frames seen for BUS_TIMEOUT_MS:
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

#include <SPI.h>
#include <mcp2515.h>

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
static const uint16_t CAN_ID_STATUS = 0x39D;  // you said driver_brake_apply is hereis hereis hereis here
static uint8_t driver_brake_state = 0;        // 0..3
static uint16_t flow_cmd_slewed = FLOW_NEUTRAL;  // Slew-limited flow command
static uint8_t  cnt         = 0;

// ---------------- Flow slew limiter ----------------
static const uint16_t FLOW_SLEW_RATE = 0x0100;  // Max change per 10ms tick (256 units = ~1.5% of range)

// 100Hz scheduler
static uint32_t next_tick_us = 0;
static const uint32_t PERIOD_US = 10000;
    return true;

  can.sendMessage(&f); if (!send_crc_frame(0x38C, p, 3)) all_ok = false;
static inline void send_frame(uint16_t id, const uint8_t *data, uint8_t dlc) {
  send_frame(id, out, (uint8_t)(plen + 1));
}
  can.sendMessage(&f); send_crc_frame(0x38C, p, 3);
static inline void send_frame(uint16_t id, const uint8_t *data, uint8_t dlc) {
  send_frame(id, out, (uint8_t)(plen + 1));
    if (denom < 1) denom = 1;
  can.sendMessage(&f); send_crc_frame(0x38C, p, 3);
static inline void send_frame(uint16_t id, const uint8_t *data, uint8_t dlc) {
  send_frame(id, out, (uint8_t)(plen + 1));
        rx_count_in_window = 0;
  can.sendMessage(&f); send_crc_frame(0x38C, p, 3);st_valid_rx_ms) > BUS_ALIVE_WINDOW_MS) {if (!bus_alive) {
static inline void send_frame(uint16_t id, const uint8_t *data, uint8_t dlc) {se; Serial.println(F("ext=0")); return; }
static uint32_t last_rx_ms = 0;
static const uint16_t BUS_TIMEOUT_MS = 200;   // tweak 100..500ms
  send_frame(id, out, (uint8_t)(plen + 1));
        flow_cmd = (uint16_t)(flow_cmd_slewed - FLOW_SLEW_RATE);
      }
  } else {
  can.sendMessage(&f); send_crc_frame(0x38C, p, 3);s drops)
static inline void send_frame(uint16_t id, const uint8_t *data, uint8_t dlc) {
  send_frame(id, out, (uint8_t)(plen + 1));
      // Double-check: Never send control if driver is braking, even if bus_alive
      if (allow_control && driver_brake_state == 2) {
  } else {
  can.sendMessage(&f); send_crc_frame(0x38C, p, 3);
static inline void send_frame(uint16_t id, const uint8_t *data, uint8_t dlc) {
  send_frame(id, out, (uint8_t)(plen + 1));
  } else {
    // Bus is alive (valid 0x39D received within timeout)
  } else {
  can.sendMessage(&f);    last_rx_ms = millis();
      bus_alive = true;  // Only set when we get valid 0x39D

    if (id != CAN_ID_STATUS) continue;
    if (f.can_dlc < 3) continue;    // driver_brake_apply: 16|2@1+  -> byte2 bits0..1
    uint8_t new_state = (uint8_t)(f.data[2] & 0x03);

    if (new_state != driver_brake_state) {
      driver_brake_state = new_state;
      Serial.print(F("0x39D driver_brake_apply="));
      Serial.print(driver_brake_state);
      Serial.print(F(" ("));
      switch (driver_brake_state) {
        case 0: Serial.print(F("not_init_or_off")); break;
  } else {
    // Bus is alive (valid 0x39D received within timeout)
    if (!bus_alive) {
      // Bus just came back alive
      Serial.println(F("Bus watchdog: iBooster responding"));
    }
    bus_alive = true;
  } driver_brake_state = 0;    // bus is alive  if (!strcmp(cmd, "tx1"))  { tx_enabled = true;  Serial.println(F("TX ON")); return; }
  if (!strcmp(cmd, "tx0"))  { tx_enabled = false; Serial.println(F("TX OFF")); return; }  // compute flow command
  flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;      // Keepalive always (even if bus dead), but inhibit control if:
      //  - bus not alive, OR
      //  - driver is pressing brakes (state 2)    last_rx_ms = millis();
    bus_alive = true;

    if (id != CAN_ID_STATUS) continue;
    if (f.can_dlc < 3) continue;    // driver_brake_apply: 16|2@1+  -> byte2 bits0..1
    uint8_t new_state = (uint8_t)(f.data[2] & 0x03);

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
    }
  }    // IMPORTANT: clear latched brake state so LED never sticks ON
    driver_brake_state = 0;    // bus is alive  if (!strcmp(cmd, "tx1"))  { tx_enabled = true;  Serial.println(F("TX ON")); return; }
  if (!strcmp(cmd, "tx0"))  { tx_enabled = false; Serial.println(F("TX OFF")); return; }  // compute flow command
  flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;      // Keepalive always (even if bus dead), but inhibit control if:
      //  - bus not alive, OR
      //  - driver is pressing brakes (state 2)    last_rx_ms = millis();
    bus_alive = true;

    if (id != CAN_ID_STATUS) continue;
    if (f.can_dlc < 3) continue;    // driver_brake_apply: 16|2@1+  -> byte2 bits0..1
    uint8_t new_state = (uint8_t)(f.data[2] & 0x03);

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
    }
  }    // IMPORTANT: clear latched brake state so LED never sticks ON
    driver_brake_state = 0;    // bus is alive  if (!strcmp(cmd, "tx1"))  { tx_enabled = true;  Serial.println(F("TX ON")); return; }
    bus_alive = true;
    last_rx_ms = millis();
    bus_alive = true;
    last_rx_ms = millis();
    bus_alive = true;


    if (id != CAN_ID_STATUS) continue;
    if (f.can_dlc < 3) continue;
    uint8_t new_state = (uint8_t)(f.data[2] & 0x03); case 0: Serial.print(F("not_init_or_off")); break;
    // driver_brake_apply: 16|2@1+  -> byte2 bits0..1
    uint8_t new_state = (uint8_t)(f.data[2] & 0x03); case 0: Serial.print(F("not_init_or_off")); break;
        case 1: Serial.print(F("brakes_not_applied")); break;
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
    }      // Bus just went dead
    // bus is aliver latched brake state so LED never sticks ON
    // Also prevents sending control when bus is dead
    if (driver_brake_state != 0) {
      driver_brake_state = 0;
      Serial.println(F("Cleared driver_brake_state due to bus timeout"));
  if (!strcmp(cmd, "tx1"))  { tx_enabled = true;  Serial.println(F("TX ON")); return; }
  if (!strcmp(cmd, "tx0"))  { tx_enabled = false; Serial.println(F("TX OFF")); return; }
      Serial.println(F("Bus watchdog: iBooster responding"));
    }  // compute flow command (only if bus is alive, otherwise keep neutral)
    // IMPORTANT: clear latched brake state so LED never sticks ON
    driver_brake_state = 0;low_cmd = FLOW_NEUTRAL;
  }  if (!strcmp(cmd, "tx1"))  { 
    // bus is alivex_enabled = true;  
    Serial.println(F("TX ON")); 
    return; 
  }
  if (!strcmp(cmd, "tx0"))  { 
    tx_enabled = false; 
  if (!strcmp(cmd, "tx1"))  { tx_enabled = true;  Serial.println(F("TX ON")); return; }
  if (!strcmp(cmd, "tx0"))  { tx_enabled = false; Serial.println(F("TX OFF")); return; }
  }  // compute flow command (only if bus is alive, otherwise keep neutral)
  // SAFETY: Don't compute new commands when bus is dead to avoid stale values
  if (bus_alive) {
    flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;
  } else {
    // Bus dead: force neutral to prevent sending stale commands when bus recovers
    flow_cmd = FLOW_NEUTRAL;
  }  // compute flow command (only if bus is alive, otherwise keep neutral)
  // SAFETY: Don't compute new commands when bus is dead to avoid stale values
  if (bus_alive) {
    flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;
  } else {
    // Bus dead: force neutral to prevent sending stale commands when bus recovers
    flow_cmd = FLOW_NEUTRAL;
  }      // SAFETY: Multiple layers of protection
      // 1. Bus must be alive (valid 0x39D received within timeout)
      // 2. Driver must not be pressing brakes (state != 2)
      // 3. flow_cmd is forced to NEUTRAL when bus is dead (see above)      
      // Double-check: Never send control if driver is braking, even if bus_alive
  if (!strcmp(cmd, "tx1"))  { tx_enabled = true;  Serial.println(F("TX ON")); return; }
  if (!strcmp(cmd, "tx0"))  { tx_enabled = false; Serial.println(F("TX OFF")); return; }   // 2. Driver must not be pressing brakes (state != 2)
      // 3. flow_cmd is forced to NEUTRAL when bus is dead (see above)      
      // Double-check: Never send control if driver is braking, even if bus_alive
      if (allow_control && driver_brake_state == 2) {
        allow_control = false;  // Extra safety check
      }
      
  // compute flow command
  flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;      // Keepalive always (even if bus dead), but inhibit control if:
      //  - bus not alive, OR
      //  - driver is pressing brakes (state 2)  // compute flow command
  flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;      // Keepalive always (even if bus dead), but inhibit control if:
      //  - bus not alive, OR
      //  - driver is pressing brakes (state 2)  // compute flow command
  flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;  // compute flow command
  flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;  // compute flow command (only if bus is alive, otherwise keep neutral)
  // SAFETY: Don't compute new commands when bus is dead to avoid stale values
  if (bus_alive) {
    flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;
  } else {
    // Bus dead: force neutral to prevent sending stale commands when bus recovers
    flow_cmd = FLOW_NEUTRAL;
  }  // compute flow command (only if bus is alive, otherwise keep neutral)
  // SAFETY: Don't compute new commands when bus is dead to avoid stale values
  if (bus_alive) {
    flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;
  } else {
    // Bus dead: force neutral to prevent sending stale commands when bus recovers
  // compute flow command
  flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;ep neutral)
  // SAFETY: Don't compute new commands when bus is dead to avoid stale values
  if (bus_alive) {
    flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;
  } else {
    // Bus dead: force neutral to prevent sending stale commands when bus recovers
    flow_cmd = FLOW_NEUTRAL;
  }      // SAFETY: Multiple layers of protection
      // 1. Bus must be alive (valid 0x39D received within timeout)
      // 2. Driver must not be pressing brakes (state != 2)
      // 3. flow_cmd is forced to NEUTRAL when bus is dead (see above)      
      // Double-check: Never send control if driver is braking, even if bus_alive
      if (allow_control && driver_brake_state == 2) {
        allow_control = false;  // Extra safety check
      }
      
      // SAFETY: Multiple layers of protection
      // 1. Bus must be alive (valid 0x39D received within timeout)
      // 2. Driver must not be pressing brakes (state != 2)
      // Keepalive always (even if bus dead), but inhibit control if:
      //  - bus not alive, OR
      //  - driver is pressing brakes (state 2)
      }
  // compute flow command
  flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;      // Keepalive always (even if bus dead), but inhibit control if:
      //  - bus not alive, OR
      //  - driver is pressing brakes (state 2)  // compute flow command
  flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;  // compute flow command
  flow_cmd = joy_enabled ? map_joystick_to_flow() : flow_manual;