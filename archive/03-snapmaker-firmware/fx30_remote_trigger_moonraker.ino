/*
 * OpenClick C3 WiFi - Sony FX30 remote over PTP/IP, Moonraker-triggered
 *
 * Board  : ESP32-C3 (0.42" OLED board, or a plain SuperMini)
 * Status : LED, and the OLED as well if the board has one
 * Trigger: Moonraker websocket (primary) - the external limit switch still
 *          works too, as a manual/test-shot button
 * Camera : ILME-FX30 via a SHARED WI-FI ROUTER, PC Remote / PTP-IP on port
 *          15740 - NOT the camera's own Wi-Fi Direct hotspot anymore
 *
 * WHAT CHANGED FROM THE SWITCH-ONLY VERSION
 * The ESP32 now joins the same router as the printer and camera, instead of
 * the camera's own "DIRECT-..." hotspot. It opens a websocket straight to
 * Moonraker (the API the U1's own Fluidd/Mainsail web UI already uses - no
 * SSH, nothing installed on the printer beyond a "[respond]" line in
 * printer.cfg, most likely already there). The Timelapse G-code box sends a
 * stock RESPOND message once per layer; this firmware watches for it over
 * that websocket and fires the shutter the instant it sees it - no physical
 * switch travel, no press, no switch wear.
 *
 * The physical limit switch wiring and logic are left in untouched, and
 * still take a photo on demand - handy for focus-checking and framing
 * before a print starts, or a manual shot mid-print. It is no longer the
 * per-layer trigger; the Timelapse G-code box no longer drives the head to
 * it at all.
 *
 * WIRING - 3-pin limit switch, all three terminals used (manual/test button
 * only now - see above)
 *
 *      switch NC ------> GND
 *      switch NO ------> 3V3        (the board's 3V3 pin - NEVER 5V, the
 *                                    ESP32-C3 is not 5V tolerant)
 *      switch C  ------> GPIO3
 *
 * BUTTON
 *   tap        -> record start/stop, or take a photo in PHOTO mode
 *   hold 1.5 s -> switch VIDEO <-> PHOTO
 *   hold 6 s   -> reboot
 *
 * Protocol notes, verified against alpha-fairy's ptpsonycodes.h:
 *   SDIOConnect            0x9201
 *   SDIOGetExtDeviceInfo   0x9202
 *   SetControlDeviceB      0x9207
 *   Movie (record button)  0xD2C8
 *   Capture (S2 shutter)   0xD2C2
 *   AutoFocus (S1 focus)   0xD2C1
 * Button-style properties take 0x0002 to press and 0x0001 to release.
 *
 * LED STATUS (works with or without the OLED)
 *   fast blink   joining Wi-Fi / handshaking
 *   solid        connected and idle
 *   double blink recording
 *
 * Libraries: U8g2 by oliver (only if OC_USE_OLED is 1),
 *            Adafruit NeoPixel (only if OC_LED_MODE is 2),
 *            WebSockets by Markus Sattler (Links2004) - new, required.
 */

#include <Arduino.h>

// ---------------------------------------------------------------------------
// These two type declarations sit above everything, including the includes,
// deliberately. The Arduino IDE auto-generates prototypes for the functions in
// a .ino and injects them near the top of the file, ahead of your own code. A
// type named in a function signature must already exist at that injection
// point, or the build fails with "'BtnEvent' does not name a type" - which is
// what happens if these are declared further down beside btnService().
// Nothing here depends on the config below, so the top of the file is safe.
// ---------------------------------------------------------------------------
struct Button {
  uint8_t  pin;
  bool     activeLow;
  bool     stable, lastRaw, longFired;
  uint32_t lastChange, pressStart;
};
enum BtnEvent { BTN_NONE, BTN_TAP, BTN_LONG };

#include <WiFi.h>
#include <HTTPClient.h>
#include <WebSocketsClient.h>

// ---------------------------------------------------------------- config ---
// Shared router - BOTH the ESP32 and the camera join this network as
// ordinary clients now, instead of the camera's own Wi-Fi Direct hotspot.
// Point the FX30's own Wi-Fi settings at this same router too (camera menu:
// Network > Wi-Fi Settings > join the router, not "PC Remote"/Wi-Fi Direct).
#define ROUTER_SSID "your-wifi-ssid"
#define ROUTER_PASS "your-wifi-password"

// Camera's IP on that router. Leave blank ("") to auto-scan the subnet for
// it at startup - simple, but costs a few seconds and could pick the wrong
// device if something else on your network happens to answer on port 15740.
// Strongly recommended: give the FX30 a DHCP reservation on your router and
// hardcode that IP here instead - faster and unambiguous every boot.
#define CAM_IP_STR "192.168.100.240"

// Moonraker - the U1 already runs this and exposes it on the LAN with no
// SSH needed, the same API Fluidd/Mainsail's own web UI uses. Confirmed
// from this printer's own moonraker.conf ([server] host: all, port: 7125)
// that it listens directly on 7125, no nginx proxy in front of it here.
#define MOONRAKER_HOST "192.168.100.6"   // <-- SET THIS to your printer's IP
#define MOONRAKER_PORT 7125
#define TRIGGER_TOKEN  "FX30_TRIGGER"    // must match the Timelapse G-code box

// --- buttons ----------------------------------------------------------------
//   REC   : on-board BOOT button, GPIO9, active low (internal pull-up).
//           tap = record start/stop.   hold 6 s = reboot.
//   PHOTO : external limit switch, GPIO3, active high with the NC/NO/C wiring.
//           tap = take a photo (manual/test shot - no longer the per-layer
//           trigger, see header notes).
// GPIO9 is a strapping pin but that is fine for BOOT: it already has the right
// pull and nothing else hangs off it. Keep the external switch off GPIO2/8/9.
#define OC_REC_PIN          9
#define OC_REC_ACTIVE_LOW   1

#define OC_PHOTO_PIN        3
#define OC_PHOTO_ACTIVE_LOW 0     // 0 suits NC->GND, NO->3V3, C->GPIO

// --- display ----------------------------------------------------------------
// 1 for the 0.42" OLED board, 0 for a plain SuperMini with no screen.
#define OC_USE_OLED   1
#define OC_OLED_SDA   5
#define OC_OLED_SCL   6

// --- status LED -------------------------------------------------------------
// Off by default: the OLED is the status display, exactly as on the build that
// works. Only turn this on if you also want a separate indicator LED.
//   0 = none  (default)
//   1 = plain LED on a GPIO (the SuperMini's blue LED is GPIO8, active LOW)
//   2 = WS2812 addressable LED (the 01space 0.42" board has one; the pin
//       differs between clones, commonly GPIO2 or GPIO8)
#define OC_LED_MODE       0
#define OC_LED_PIN        8
#define OC_LED_ACTIVE_LOW 1        // plain-LED mode only; WS2812 ignores this
#define OC_LED_BRIGHTNESS 40       // WS2812 mode only

// --- photo tuning -----------------------------------------------------------
// 1 = half-press to focus first, then shutter. 0 = shutter only, which is what
// you want on manual focus or if the camera is set to release priority.
// 0 = fire once and trust it (default: this is what works).
// 1 = confirm each shot via the camera's ObjectAdded event and escalate the
//     timing if nothing arrives. Only enable this if captures start failing -
//     if the camera ever omits the event, a good shot looks like a failure and
//     you get three frames instead of one.
#define OC_PHOTO_VERIFY     0

// 0 for a 3D-print timelapse. Put the camera on MANUAL focus and manual
// exposure and leave this at 0:
//   - autofocus would hunt slightly between frames, and every tiny refocus
//     shows up as a visible pop in the finished video
//   - it removes the AF settle wait, so the shot happens while the head is
//     still parked instead of after it has moved on
//   - one less thing that can fail on frame 900 of 1200
// Set to 1 only for handheld use where you want AF per shot.
#define OC_PHOTO_USE_AF     0
// Autofocus needs real time. The old 120 ms was far too short: on focus
// priority the camera simply ignores a shutter press that arrives before
// focus locks, and still answers 0x2001, so it looks like it worked.
#define OC_AF_SETTLE_MS     900
#define OC_SHUTTER_HOLD_MS  300

// Ignore a second trigger within this window - covers a stray duplicate
// Moonraker notification, or the switch bouncing on a manual test press.
#define OC_RETRIGGER_LOCKOUT_MS 1500

// Extra settle time after a SWITCH tap before firing (manual/test shots
// only now). In the old design this covered "press, then travel away, then
// settle" - keep it for the switch, since a manual test press might still
// be moving the head when it lands.
#define OC_TRIGGER_DELAY_MS 600

// Extra settle time after a MOONRAKER trigger before firing. The G-code box
// already does M400 (wait for the park move to finish) before sending the
// trigger, so the head is already stationary by the time this fires - this
// only needs to cover network/processing jitter, not travel time.
#define OC_MOONRAKER_TRIGGER_DELAY_MS 100

// Some bodies want SetControlDeviceA (0x9205) rather than B (0x9207) for the
// shutter. Flip this if the sequence still does nothing.
#define OC_SHUTTER_OPCODE   OP_SetControlDeviceB

#define OC_TAP_MAX_MS    800
#define OC_MODE_HOLD_MS  1500
#define OC_REBOOT_HOLD_MS 6000
#define OC_DEBOUNCE_MS   25

#define PTPIP_PORT 15740

// Set to 1 for ONE upload to wipe NVS, then put it back to 0. NVS survives a
// normal sketch upload, so leftovers from earlier work stay on the board:
// BLE bonds from the Bluetooth attempts, and Wi-Fi credentials that the ESP32
// stores automatically. This clears them without a full chip erase.
#define OC_WIPE_NVS_ON_BOOT 0

#include <nvs_flash.h>

// Includes that depend on the options above.
#if OC_USE_OLED
  #include <U8g2lib.h>
#endif
#if OC_LED_MODE == 2
  #include <Adafruit_NeoPixel.h>
#endif

// ------------------------------------------------------------- protocol ----
#define OP_GetDeviceInfo        0x1001
#define OP_OpenSession          0x1002
#define OP_GetStorageIDs        0x1004
#define OP_SDIOConnect          0x9201
#define OP_SDIOGetExtDeviceInfo 0x9202
#define OP_SetControlDeviceA    0x9205
#define OP_SetControlDeviceB    0x9207

#define PROP_AutoFocus 0xD2C1     // S1 / half press
#define PROP_Capture   0xD2C2     // S2 / full press
#define PROP_Movie     0xD2C8     // record button

#define EVT_ObjectAdded     0xC201   // Sony: a new image exists = capture worked
#define EVT_ObjectRemoved   0xC202
#define EVT_PropertyChanged 0xC203

#define BTN_DOWN 0x0002
#define BTN_UP   0x0001

#if OC_USE_OLED
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OC_OLED_SCL, OC_OLED_SDA);
#endif
#if OC_LED_MODE == 2
Adafruit_NeoPixel rgb(1, OC_LED_PIN, NEO_GRB + NEO_KHZ800);
#endif

static WiFiClient cmdSock, evtSock;
static WebSocketsClient wsClient;
static bool       gMoonrakerConnected = false;
static uint32_t   gConnNum = 0;
static uint32_t   gTxn     = 1;
static uint32_t   gLastActivity = 0;   // for the idle keepalive
static volatile uint32_t gObjectAdded = 0;  // counts confirmed captures
static uint32_t gFrames      = 0;           // shutter triggers this session
static uint32_t gLastTrigger = 0;           // for the re-trigger lockout

enum State { ST_WIFI, ST_HANDSHAKE, ST_READY, ST_LOST };

static Button gRecBtn   = { OC_REC_PIN,   (bool)OC_REC_ACTIVE_LOW,   false,false,false, 0,0 };
static Button gPhotoBtn = { OC_PHOTO_PIN, (bool)OC_PHOTO_ACTIVE_LOW, false,false,false, 0,0 };

static State    gState     = ST_WIFI;
static bool     gRecording = false;
static char     gStatus[20]= "";
static uint32_t gStatusTil = 0;
static bool     gDirty     = true;

static const uint8_t CLIENT_GUID[16] = {
  0x4f,0x70,0x65,0x6e,0x43,0x6c,0x69,0x63,0x6b,0x43,0x33,0x00,0x01,0x02,0x03,0x04 };

// ------------------------------------------------------------- utilities ---
static void setStatus(const char* m, uint32_t ms = 1500) {
  strncpy(gStatus, m, sizeof(gStatus)-1); gStatus[sizeof(gStatus)-1] = 0;
  gStatusTil = millis() + ms; gDirty = true;
}
static void put32(uint8_t* p, uint32_t v){ p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }
static uint32_t get32(const uint8_t* p){
  return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static size_t putUtf16(uint8_t* p, const char* s){
  size_t n=0; while(*s){ p[n++]=(uint8_t)*s++; p[n++]=0; } p[n++]=0; p[n++]=0; return n; }

static int readPacket(WiFiClient& s, uint8_t* buf, size_t cap, uint32_t toMs=4000) {
  uint32_t t0 = millis();
  while (s.available() < 4) {
    if (!s.connected()) return -1;
    if (millis()-t0 > toMs) return -1;
    delay(5);
  }
  s.read(buf,4);
  uint32_t len = get32(buf);
  if (len < 8 || len > cap) return -1;
  size_t got = 4;
  while (got < len) {
    if (millis()-t0 > toMs) return -1;
    if (s.available()) got += s.read(buf+got, len-got); else delay(2);
  }
  return (int)len;
}

// Read and discard anything waiting on the event channel. Cheap, non-blocking,
// and must be called often. Events are informational for us - we only need to
// stop them backing up.
static void pumpEvents() {
  uint8_t rx[512];
  int guard = 0;
  while (evtSock.connected() && evtSock.available() >= 4 && guard++ < 8) {
    int len = readPacket(evtSock, rx, sizeof(rx), 300);
    if (len < 0) break;
    uint32_t type = get32(rx+4);
    if (type == 0x00000008 && len >= 14) {
      uint16_t evt = rx[8] | (rx[9] << 8);
      if (evt == EVT_ObjectAdded) {
        gObjectAdded = gObjectAdded + 1;   // ++ on volatile is deprecated
        Serial.printf("   <event 0x%04X ObjectAdded - capture confirmed>\n", evt);
      } else {
        Serial.printf("   <event 0x%04X%s>\n", evt,
                      evt == EVT_PropertyChanged ? " PropertyChanged" : "");
      }
    } else {
      Serial.printf("   <event pkt type 0x%08lX, %d bytes>\n", (unsigned long)type, len);
    }
  }
}

// LED status, independent of whether an OLED is fitted:
//   fast blink   = joining Wi-Fi / handshaking
//   solid        = connected and idle
//   double blink = recording
static void ledStatus() {
#if OC_LED_MODE
  uint32_t t = millis();
  bool on;
  if (gState != ST_READY)  on = (t % 300) < 150;                       // fast
  else if (gRecording)     on = (t % 1000) < 120 || ((t % 1000) > 220 && (t % 1000) < 340);
  else                     on = true;                                  // solid
#endif
#if OC_LED_MODE == 1
  digitalWrite(OC_LED_PIN, (on ^ OC_LED_ACTIVE_LOW) ? HIGH : LOW);
#elif OC_LED_MODE == 2
  uint32_t c;
  if (!on)                       c = rgb.Color(0, 0, 0);
  else if (gState != ST_READY)   c = rgb.Color(0, 0, OC_LED_BRIGHTNESS);              // blue
  else if (gRecording)           c = rgb.Color(OC_LED_BRIGHTNESS, 0, 0);              // red
  else                           c = rgb.Color(0, OC_LED_BRIGHTNESS, 0);              // green
  rgb.setPixelColor(0, c);
  rgb.show();
#endif
}

// --------------------------------------------------------------- display ---
static void draw() {
  ledStatus();
#if OC_USE_OLED
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tr);
  const char* top = "?";
  switch (gState) {
    case ST_WIFI:      top = "WIFI..";   break;
    case ST_HANDSHAKE: top = "LINKING";  break;
    case ST_READY:     top = gMoonrakerConnected ? "FX30+MR" : "FX30";  break;
    case ST_LOST:      top = "LOST";     break;
  }
  u8g2.drawStr(0,7,top);
  u8g2.drawHLine(0,9,72);

  if (gState == ST_READY) {
    u8g2.setFont(u8g2_font_7x13B_tr);
    if (gRecording) { u8g2.drawStr(0,25,"REC"); u8g2.drawDisc(52,20,5); }
    else if (gFrames) {                       // timelapse: show the frame count
      char b[12]; snprintf(b, sizeof(b), "%lu", (unsigned long)gFrames);
      u8g2.drawStr(0,25,b);
    }
    else            { u8g2.drawStr(0,25,"READY"); }
  } else {
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(0,22,"connecting");
  }

  u8g2.setFont(u8g2_font_5x8_tr);
  if (gStatus[0] && millis() < gStatusTil) u8g2.drawStr(0,38,gStatus);
  else { gStatus[0]=0; u8g2.drawStr(0,38, gState==ST_READY ? (gFrames ? "frames" : "BOOT=rec SW=pic") : ""); }
  u8g2.sendBuffer();
#endif
}

// -------------------------------------------------------------- PTP/IP -----
// Wait for the Operation Response (type 7), discarding any data packets that
// precede it. Data-in operations send Start/Data/End before the response.
static bool waitResponse(uint16_t* codeOut) {
  uint8_t rx[1024];
  const uint32_t deadline = millis() + 3000;      // total budget, not per packet
  while ((int32_t)(deadline - millis()) > 0) {
    pumpEvents();                                 // never let events back up
    if (cmdSock.available() < 4) { delay(2); continue; }
    int len = readPacket(cmdSock, rx, sizeof(rx), 500);
    if (len < 0) break;
    uint32_t type = get32(rx+4);
    if (type == 0x00000007) {                     // Operation Response
      uint16_t code = rx[8] | (rx[9]<<8);
      if (codeOut) *codeOut = code;
      return code == 0x2001;
    }
    // 9 / 0x0A / 0x0C are data-phase packets, 8 is an event - keep reading
  }
  if (codeOut) *codeOut = 0xFFFF;                 // timed out
  return false;
}

// One PTP operation. data==nullptr means no data phase; otherwise data-out.
static bool ptpOp(uint16_t opcode, const uint32_t* params, int nParams,
                  const uint8_t* data, size_t dataLen, const char* label) {
  uint32_t txn = gTxn++;
  uint8_t pkt[64];
  size_t n = 8;
  put32(pkt+n, data ? 0x00000002 : 0x00000001); n += 4;   // data phase
  pkt[n++] = opcode & 0xFF; pkt[n++] = opcode >> 8;
  put32(pkt+n, txn); n += 4;
  for (int i = 0; i < nParams; i++) { put32(pkt+n, params[i]); n += 4; }
  put32(pkt+0, (uint32_t)n);
  put32(pkt+4, 0x00000006);
  cmdSock.write(pkt, n);

  if (data) {
    uint8_t sd[20];
    put32(sd+0, 20); put32(sd+4, 0x00000009); put32(sd+8, txn);
    put32(sd+12, (uint32_t)dataLen); put32(sd+16, 0);       // 64-bit length
    cmdSock.write(sd, 20);

    uint8_t ed[12+16];
    put32(ed+0, (uint32_t)(12+dataLen)); put32(ed+4, 0x0000000C); put32(ed+8, txn);
    memcpy(ed+12, data, dataLen);
    cmdSock.write(ed, 12+dataLen);
  }
  cmdSock.flush();

  uint16_t code = 0;
  bool ok = waitResponse(&code);
  gLastActivity = millis();
  Serial.printf("   %-22s op=0x%04X -> 0x%04X %s\n", label, opcode, code,
                ok ? "OK" : (code == 0xFFFF ? "TIMEOUT" : "FAIL"));
  if (code == 0xFFFF && gState == ST_READY) {
    Serial.println("   no response - marking link lost");
    gState = ST_LOST; gDirty = true;
    cmdSock.stop(); evtSock.stop();
  }
  return ok;
}

static bool ptpHandshakeInner(IPAddress cam);

static bool ptpHandshake(IPAddress cam) {
  // Always leave gState resolved: READY on success, LOST on any failure.
  // Anything else strands the retry logic.
  cmdSock.stop();
  evtSock.stop();
  bool ok = ptpHandshakeInner(cam);
  if (!ok) {
    Serial.println("   handshake failed - will retry");
    gState = ST_LOST;
    gDirty = true;
    cmdSock.stop();
    evtSock.stop();
  }
  return ok;
}

static bool ptpHandshakeInner(IPAddress cam) {
  gState = ST_HANDSHAKE; draw();
  gTxn = 1;

  if (!cmdSock.connect(cam, PTPIP_PORT)) {
    // Usually means the camera still holds the previous session. It only
    // accepts one PTP client, and a reset board leaves the old one dangling
    // until the camera times it out. Retrying is the cure; nothing is wrong.
    Serial.println("cmd TCP failed (camera may still hold the old session)");
    return false;
  }
  uint8_t pkt[128]; size_t n = 8;
  memcpy(pkt+n, CLIENT_GUID, 16); n += 16;
  n += putUtf16(pkt+n, "OpenClick");
  put32(pkt+n, 0x00010000); n += 4;
  put32(pkt+0, (uint32_t)n); put32(pkt+4, 0x00000001);
  cmdSock.write(pkt, n); cmdSock.flush();

  uint8_t rx[512];
  int len = readPacket(cmdSock, rx, sizeof(rx));
  if (len < 0 || get32(rx+4) != 0x00000002) { Serial.println("no Init Command Ack"); return false; }
  gConnNum = get32(rx+8);
  Serial.printf("   Init Command Ack, conn %lu\n", (unsigned long)gConnNum);

  if (!evtSock.connect(cam, PTPIP_PORT)) { Serial.println("evt TCP failed"); return false; }
  uint8_t ev[12];
  put32(ev+0,12); put32(ev+4,0x00000003); put32(ev+8,gConnNum);
  evtSock.write(ev,12); evtSock.flush();
  len = readPacket(evtSock, rx, sizeof(rx));
  if (len < 0 || get32(rx+4) != 0x00000004) { Serial.println("no Init Event Ack"); return false; }
  Serial.println("   Init Event Ack");

  uint32_t p1[1] = {1};
  if (!ptpOp(OP_OpenSession, p1, 1, nullptr, 0, "OpenSession")) return false;

  // Sony's connect handshake, order taken from alpha-fairy's init_table.
  uint32_t z[3]   = {0,0,0};
  uint32_t c1[3]  = {1,0,0};
  uint32_t c2[3]  = {2,0,0};
  uint32_t c3[3]  = {3,0,0};
  uint32_t inf[3] = {0x12C,0,0};
  ptpOp(OP_GetDeviceInfo,        z,   0, nullptr, 0, "GetDeviceInfo");
  // 0x2013 "Store Not Available" here is normal and does not matter;
  // we never touch storage. Logged as FAIL only because 0x2001 is the
  // generic success code.
  ptpOp(OP_GetStorageIDs,        z,   0, nullptr, 0, "GetStorageIDs (opt)");
  if (!ptpOp(OP_SDIOConnect,     c1,  3, nullptr, 0, "SDIOConnect(1)")) return false;
  if (!ptpOp(OP_SDIOConnect,     c2,  3, nullptr, 0, "SDIOConnect(2)")) return false;
  ptpOp(OP_SDIOGetExtDeviceInfo, inf, 3, nullptr, 0, "GetExtDeviceInfo");
  if (!ptpOp(OP_SDIOConnect,     c3,  3, nullptr, 0, "SDIOConnect(3)")) return false;
  ptpOp(OP_SDIOGetExtDeviceInfo, inf, 3, nullptr, 0, "GetExtDeviceInfo");

  gState = ST_READY; gDirty = true;
  gLastActivity = millis();
  setStatus("ready");
  Serial.println("   *** CAMERA READY ***");
  return true;
}

// --------------------------------------------------------- camera lookup ---
// Fixed IP (recommended - set a DHCP reservation on your router and put the
// address in CAM_IP_STR), or a simple sequential scan of the local /24 for
// whatever answers on the PTP/IP port. The scan is blocking and can take a
// few seconds; that is fine here since it only runs at startup/reconnect,
// never per-shot.
static bool resolveCameraIP(IPAddress& out) {
  if (CAM_IP_STR[0] != '\0') {
    if (out.fromString(CAM_IP_STR)) return true;
    Serial.println("!! CAM_IP_STR is set but did not parse as an IP - check it");
  }

  IPAddress local = WiFi.localIP();
  Serial.println("   scanning subnet for the camera's PTP/IP port...");
  for (int host = 1; host <= 254; host++) {
    IPAddress candidate(local[0], local[1], local[2], host);
    if (candidate == local) continue;
    WiFiClient probe;
    if (probe.connect(candidate, PTPIP_PORT, 150)) {
      probe.stop();
      Serial.printf("   found a device on port %d at %s\n", PTPIP_PORT,
                    candidate.toString().c_str());
      out = candidate;
      return true;
    }
  }
  Serial.println("!! no device answering on the PTP/IP port found on the subnet");
  return false;
}

// A Sony "button" property: press then release.
static bool sonyButtonOp(uint16_t opcode, uint16_t prop, uint16_t value) {
  uint32_t p[1] = { prop };
  uint8_t v[2] = { (uint8_t)(value & 0xFF), (uint8_t)(value >> 8) };
  char lbl[28]; snprintf(lbl, sizeof(lbl), "prop %04X = %d", prop, value);
  return ptpOp(opcode, p, 1, v, 2, lbl);
}
static bool sonyButton(uint16_t prop, uint16_t value) {
  return sonyButtonOp(OP_SetControlDeviceB, prop, value);
}

static void doRecordToggle() {
  Serial.printf("[%lu] record %s\n", (unsigned long)millis(), gRecording ? "STOP" : "START");
  sonyButton(PROP_Movie, BTN_DOWN);
  delay(60);
  sonyButton(PROP_Movie, BTN_UP);
  gRecording = !gRecording;
  setStatus(gRecording ? "rec start" : "rec stop");
  gDirty = true;
}

// Wait for the camera to report a new image, pumping events meanwhile.
static bool waitForCapture(uint32_t before, uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    pumpEvents();
    if (gObjectAdded != before) return true;
    delay(20);
  }
  return false;
}

// One shutter attempt. afSettle == 0 means skip the half-press entirely.
static bool photoAttempt(uint16_t afSettle, uint16_t hold, uint32_t confirmMs, const char* what) {
  uint32_t before = gObjectAdded;
  Serial.printf("   attempt: %s (af %u ms, hold %u ms)\n", what, afSettle, hold);

  if (afSettle) {
    sonyButtonOp(OC_SHUTTER_OPCODE, PROP_AutoFocus, BTN_DOWN);
    for (uint16_t i = 0; i < afSettle / 50; i++) { pumpEvents(); delay(50); }
  }
  sonyButtonOp(OC_SHUTTER_OPCODE, PROP_Capture, BTN_DOWN);
  // Holding the full press matters in AF-C: the camera fires the moment focus
  // is good enough, which can be well after the press. Releasing too early
  // cancels the shot, and the camera still answered 0x2001 to every write.
  for (uint16_t i = 0; i < hold / 50; i++) { pumpEvents(); delay(50); }
  sonyButtonOp(OC_SHUTTER_OPCODE, PROP_Capture, BTN_UP);
  delay(60);
  if (afSettle) sonyButtonOp(OC_SHUTTER_OPCODE, PROP_AutoFocus, BTN_UP);

  return waitForCapture(before, confirmMs);
}

static void doPhoto() {
  Serial.printf("[%lu] photo\n", (unsigned long)millis());
  setStatus("focusing...", 3000);
  draw();

#if !OC_PHOTO_VERIFY
  // Single shot, no confirmation, no retries.
  photoAttempt(OC_PHOTO_USE_AF ? OC_AF_SETTLE_MS : 0, OC_SHUTTER_HOLD_MS, 0, "single");
  setStatus("shot");
  return;
#elif OC_PHOTO_USE_AF
  // Escalating attempts. AF-S usually lands on the first; AF-C never reports a
  // focus lock, so it needs the longer full-press hold of the second.
  if (photoAttempt(OC_AF_SETTLE_MS, OC_SHUTTER_HOLD_MS, 1200, "AF normal")) {
    setStatus("shot"); Serial.println("   -> captured"); return;
  }
  Serial.println("   no image yet - retrying with AF-C timing");
  setStatus("retry AF-C", 2000); draw();
  if (photoAttempt(OC_AF_SETTLE_MS * 2, 1200, 1800, "AF-C long hold")) {
    setStatus("shot (AF-C)"); Serial.println("   -> captured on AF-C retry"); return;
  }
  Serial.println("   still nothing - firing without half-press");
  setStatus("retry no-AF", 2000); draw();
  if (photoAttempt(0, 1200, 1800, "shutter only")) {
    setStatus("shot (no AF)"); Serial.println("   -> captured without AF"); return;
  }
#else
  if (photoAttempt(0, OC_SHUTTER_HOLD_MS, 1500, "shutter only")) {
    setStatus("shot"); Serial.println("   -> captured"); return;
  }
#endif

  Serial.println("   !! no ObjectAdded event - camera did not take a photo.");
  Serial.println("      Check it is in STILLS mode, and that AF can find the subject.");
  setStatus("no shot!", 2500);
}

// Shared entry point for every trigger source (switch, Moonraker). Applies
// the same lockout and camera-ready check regardless of who asked.
static void triggerPhoto(const char* source, uint32_t settleMs) {
  uint32_t now = millis();
  if (now - gLastTrigger < OC_RETRIGGER_LOCKOUT_MS && gLastTrigger != 0) {
    Serial.printf("[%lu] %s trigger ignored, only %lu ms since the last one\n",
                  (unsigned long)now, source, (unsigned long)(now - gLastTrigger));
    return;
  }
  if (gState != ST_READY) {
    setStatus("no camera");
    Serial.printf("!! %s trigger MISSED - camera not connected\n", source);
    return;
  }
  gLastTrigger = now;
  gFrames++;
  if (settleMs > 0) {
    Serial.printf("[%lu] %s trigger, waiting %lu ms to settle\n",
                  (unsigned long)now, source, (unsigned long)settleMs);
    setStatus("waiting...", settleMs + 200);
    draw();
    uint32_t t0 = millis();
    while (millis() - t0 < settleMs) { pumpEvents(); wsClient.loop(); delay(10); }
  }
  doPhoto();
  Serial.printf("[%lu] frame %lu (%s)\n", (unsigned long)millis(), (unsigned long)gFrames, source);
}

// ---------------------------------------------------------------- button ---
// Debounced edge detection for one button. longMs == 0 disables long-press.
static BtnEvent btnService(Button& b, uint32_t longMs) {
  bool raw = (digitalRead(b.pin) == (b.activeLow ? LOW : HIGH));
  uint32_t now = millis();

  if (raw != b.lastRaw) { b.lastRaw = raw; b.lastChange = now; }
  if (now - b.lastChange < OC_DEBOUNCE_MS) return BTN_NONE;

  if (raw == b.stable) {
    if (b.stable && longMs && !b.longFired && (now - b.pressStart) >= longMs) {
      b.longFired = true;
      return BTN_LONG;
    }
    return BTN_NONE;
  }

  b.stable = raw;
  if (raw) { b.pressStart = now; b.longFired = false; return BTN_NONE; }
  if (!b.longFired && (now - b.pressStart) <= OC_TAP_MAX_MS) return BTN_TAP;
  return BTN_NONE;
}

static void serviceButtons() {
  switch (btnService(gRecBtn, OC_REBOOT_HOLD_MS)) {          // BOOT button
    case BTN_TAP:
      if (gState != ST_READY) setStatus("no camera");
      else                    doRecordToggle();
      break;
    case BTN_LONG:
      setStatus("reboot", 800); draw(); delay(400); ESP.restart();
      break;
    default: break;
  }

  if (btnService(gPhotoBtn, 0) == BTN_TAP) {                 // limit switch -
    triggerPhoto("switch", OC_TRIGGER_DELAY_MS);             // manual/test shot
  }
}

// ------------------------------------------------------------- moonraker ---
// Moonraker requires a short-lived, single-use "oneshot" auth token on the
// websocket URL, even for an IP already in trusted_clients - confirmed by
// capturing the request Fluidd's own browser client sends. Tokens expire in
// 5 seconds and can only be used once, so we fetch a fresh one immediately
// before every connection attempt rather than trusting the WebSocketsClient
// library's own built-in reconnect (which would just keep retrying the same
// now-expired, already-used token forever - exactly the symptom we saw).
#define MOONRAKER_RECONNECT_MS 4000   // stays under the 5 s token expiry

static uint32_t gMoonrakerLastAttempt = 0;

static bool fetchOneshotToken(String& tokenOut) {
  HTTPClient http;
  String url = String("http://") + MOONRAKER_HOST + ":" + String(MOONRAKER_PORT)
             + "/access/oneshot_token";
  http.begin(url);
  http.setTimeout(3000);
  int code = http.GET();
  if (code != 200) {
    Serial.printf("   oneshot_token request failed, HTTP %d\n", code);
    http.end();
    return false;
  }
  String body = http.getString();
  http.end();
  Serial.printf("   oneshot_token HTTP %d, body: %s\n", code, body.c_str());
  // Handles either a bare "TOKEN" response or a {"result":"TOKEN"} wrapper -
  // Moonraker's HTTP API normally wraps results, so check for that first.
  int idx = body.indexOf("\"result\"");
  int start = (idx >= 0) ? body.indexOf('"', idx + 8) : body.indexOf('"');
  if (start < 0) { Serial.println("   oneshot_token response unrecognized"); return false; }
  int end = body.indexOf('"', start + 1);
  if (end < 0) { Serial.println("   oneshot_token response unrecognized"); return false; }
  tokenOut = body.substring(start + 1, end);
  Serial.printf("   parsed token: [%s] (len %d)\n", tokenOut.c_str(), tokenOut.length());
  return tokenOut.length() > 0;
}

static void sendMoonrakerIdentify() {
  static char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"jsonrpc\":\"2.0\",\"method\":\"server.connection.identify\","
    "\"params\":{\"client_name\":\"fx30-esp32\",\"version\":\"1.0.0\","
    "\"type\":\"agent\",\"url\":\"https://github.com/local/fx30-esp32\"},"
    "\"id\":1}");
  wsClient.sendTXT(buf);
}

static void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("   Moonraker websocket connected");
      gMoonrakerConnected = true; gDirty = true;
      sendMoonrakerIdentify();
      break;
    case WStype_DISCONNECTED:
      Serial.println("   Moonraker websocket disconnected");
      gMoonrakerConnected = false; gDirty = true;
      break;
    case WStype_TEXT: {
      // Lightweight substring match rather than a full JSON parser - the
      // trigger token is distinctive enough that a false match is not a
      // real concern, and this keeps the sketch free of an extra JSON
      // library dependency.
      if (strstr((const char*)payload, "notify_gcode_response") &&
          strstr((const char*)payload, TRIGGER_TOKEN)) {
        Serial.println("   trigger seen in Moonraker gcode response");
        triggerPhoto("moonraker", OC_MOONRAKER_TRIGGER_DELAY_MS);
      }
      break;
    }
    case WStype_ERROR:
      Serial.printf("   Moonraker websocket ERROR: %.*s\n", (int)length, (const char*)payload);
      break;
    case WStype_PING:
      Serial.println("   Moonraker websocket PING");
      break;
    case WStype_PONG:
      Serial.println("   Moonraker websocket PONG");
      break;
    default:
      Serial.printf("   Moonraker websocket event type %d, len %u\n", (int)type, (unsigned)length);
      break;
  }
}

// Fetches a fresh token and (re)connects. Safe to call repeatedly - each
// call gets its own new token, so a stale in-flight attempt is simply
// superseded rather than reused.
static void connectMoonraker() {
  String token;
  if (!fetchOneshotToken(token)) {
    Serial.println("   could not get a oneshot_token - will retry");
    return;
  }
  // begin() does NOT close/release whatever socket a previous begin() call
  // opened - calling it repeatedly without disconnecting first orphans a
  // TCP socket on the ESP32 every time, which silently exhausts the small
  // socket pool within seconds and makes even a same-millisecond fresh
  // token fail to connect. disconnect() first releases it properly.
  wsClient.disconnect();

  // The library's own default Origin header ("Origin: file://", set in its
  // constructor) fails Moonraker's websocket CORS check on every attempt -
  // confirmed via Moonraker's own access log: every single request came
  // back 403, tagged [_TRUSTED_USER_], regardless of the token being fresh,
  // valid and IP-matched. Root cause found in Moonraker's own source
  // (websockets.py check_origin() falls back to authorization.py
  // check_cors(), which flatly rejects an origin that isn't in
  // cors_domains). Tornado's own default same-origin check runs BEFORE
  // that fallback though, and passes as soon as the Origin's host:port
  // matches our own request's Host header - a browser gets this for free
  // automatically (it always sends a real Origin); we have to set it
  // ourselves since this library defaults to a placeholder instead. No
  // moonraker.conf change needed.
  String origin = String("Origin: http://") + MOONRAKER_HOST + ":" + String(MOONRAKER_PORT);
  wsClient.setExtraHeaders(origin.c_str());

  String path = String("/websocket?token=") + token;
  Serial.printf("   connecting ws://%s:%d%s\n", MOONRAKER_HOST, MOONRAKER_PORT, path.c_str());
  wsClient.begin(MOONRAKER_HOST, MOONRAKER_PORT, path);
  wsClient.onEvent(webSocketEvent);
  // The library ALWAYS auto-reconnects internally - there is no way to turn
  // this off, only to change the interval (default 500ms). Left alone, it
  // hammers the same one-shot token - already used/expired - roughly 8x
  // between each of our own MOONRAKER_RECONNECT_MS-spaced attempts, each a
  // further orphaned socket. Slow its internal retry down to match ours so
  // it stops fighting our own reconnect logic.
  wsClient.setReconnectInterval(MOONRAKER_RECONNECT_MS);
}

// ----------------------------------------------------------------- setup ---
static bool joinWifi() {
  gState = ST_WIFI; draw();
  Serial.printf("joining %s\n", ROUTER_SSID);
  // Stop the ESP32 writing the SSID and password into NVS on every begin().
  // It saves flash wear, and it stops the board silently reusing a stale AP
  // from a previous sketch instead of the one we asked for.
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ROUTER_SSID, ROUTER_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis()-t0 < 20000) { delay(300); serviceButtons(); }
  if (WiFi.status() != WL_CONNECTED) { Serial.println("wifi join failed"); return false; }
  Serial.printf("joined, ip %s gw %s rssi %d\n",
                WiFi.localIP().toString().c_str(),
                WiFi.gatewayIP().toString().c_str(), WiFi.RSSI());
  return true;
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis()-t0 < 1500) delay(10);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(20);
#endif
  pinMode(OC_REC_PIN,   OC_REC_ACTIVE_LOW   ? INPUT_PULLUP : INPUT_PULLDOWN);
  pinMode(OC_PHOTO_PIN, OC_PHOTO_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);

#if OC_LED_MODE == 1
  pinMode(OC_LED_PIN, OUTPUT);
  digitalWrite(OC_LED_PIN, OC_LED_ACTIVE_LOW ? HIGH : LOW);   // start off
#elif OC_LED_MODE == 2
  rgb.begin(); rgb.clear(); rgb.show();
#endif

#if OC_USE_OLED
  u8g2.begin();
  u8g2.setContrast(180);
#endif
  setStatus("booting", 800); draw();

  Serial.println("\n=== OpenClick C3 WiFi (Moonraker-triggered) " __DATE__ " " __TIME__ " ===");
  Serial.println("BOOT button tap = record start/stop, hold 6s = reboot");
  Serial.println("limit switch tap = manual/test photo");

#if OC_WIPE_NVS_ON_BOOT
  Serial.println("wiping NVS (OC_WIPE_NVS_ON_BOOT=1) - set it back to 0 after this upload");
  nvs_flash_erase();
  nvs_flash_init();
  Serial.println("NVS cleared");
#endif
  Serial.printf("REC   btn GPIO%d idle=%d (expect %d released)\n",
                OC_REC_PIN, digitalRead(OC_REC_PIN), OC_REC_ACTIVE_LOW ? 1 : 0);
  Serial.printf("PHOTO btn GPIO%d idle=%d (expect %d released)\n",
                OC_PHOTO_PIN, digitalRead(OC_PHOTO_PIN), OC_PHOTO_ACTIVE_LOW ? 1 : 0);
  Serial.println("NOTE: the FX30 must be in STILLS mode for photo capture -");
  Serial.println("      in movie mode the shutter commands are accepted but ignored.");

  if (strcmp(MOONRAKER_HOST, "192.168.1.xxx") == 0) {
    Serial.println("!! MOONRAKER_HOST is still the placeholder - set it to your printer's IP");
  }

  if (joinWifi()) {
    connectMoonraker();
    gMoonrakerLastAttempt = millis();
    IPAddress cam;
    if (resolveCameraIP(cam)) ptpHandshake(cam);
    else { gState = ST_LOST; gDirty = true; }
  }
}

void loop() {
  serviceButtons();
  pumpEvents();
  wsClient.loop();

  // Reconnect if either the Wi-Fi or the command socket drops.
  if (gState == ST_READY && (!cmdSock.connected() || WiFi.status() != WL_CONNECTED)) {
    Serial.println("connection lost");
    gState = ST_LOST; gRecording = false; gDirty = true;
    cmdSock.stop(); evtSock.stop();
  }
  if (gState != ST_READY) {
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 5000) {
      lastTry = millis();
      Serial.printf("[%lu] reconnect attempt (state=%d)\n", (unsigned long)millis(), gState);
      if (WiFi.status() != WL_CONNECTED) { joinWifi(); }
      if (WiFi.status() == WL_CONNECTED) {
        IPAddress cam;
        if (resolveCameraIP(cam)) ptpHandshake(cam);
      }
    }
  }

  // Moonraker websocket has its own independent reconnect timer - it needs
  // to keep retrying (fetch a fresh oneshot_token, reopen the socket) even
  // while the camera side is already ST_READY, and vice versa. Kept under
  // the 5s token expiry so a fetched token is always used promptly.
  if (!gMoonrakerConnected && WiFi.status() == WL_CONNECTED &&
      millis() - gMoonrakerLastAttempt > MOONRAKER_RECONNECT_MS) {
    gMoonrakerLastAttempt = millis();
    connectMoonraker();
  }

  // The FX30 closes an idle PTP/IP session after about 50 seconds. A cheap
  // no-op operation well inside that window keeps it open, so the trigger is
  // always live instead of costing a 3-second reconnect after a quiet minute.
  if (gState == ST_READY && millis() - gLastActivity > 8000) {
    uint32_t inf[3] = {0x12C, 0, 0};
    if (!ptpOp(OP_SDIOGetExtDeviceInfo, inf, 3, nullptr, 0, "keepalive")) {
      Serial.println("keepalive failed - dropping to reconnect");
      gState = ST_LOST; gDirty = true;
      cmdSock.stop(); evtSock.stop();
    }
  }

  static uint32_t lastBeat = 0, lastDraw = 0;
  if (millis() - lastBeat > 5000) {
    lastBeat = millis();
    Serial.printf("[%lu] state=%d wifi=%d cmd=%d moonraker=%d rec=%d frames=%lu rssi=%d\n",
                  (unsigned long)millis(), gState, WiFi.status()==WL_CONNECTED,
                  cmdSock.connected(), gMoonrakerConnected, gRecording,
                  (unsigned long)gFrames, WiFi.status()==WL_CONNECTED ? WiFi.RSSI() : 0);
  }
  if (gDirty || millis()-lastDraw > 250) { gDirty=false; lastDraw=millis(); draw(); }
  delay(5);
}
