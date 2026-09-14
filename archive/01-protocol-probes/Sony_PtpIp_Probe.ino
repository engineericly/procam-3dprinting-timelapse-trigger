/*
 * Sony_PtpIp_Probe - stage 1: can we reach the camera and complete the
 * PTP/IP handshake? Nothing else. No buttons, no display, no shutter.
 *
 * Why staged
 * ----------
 * The BLE route failed at one specific step (bonding) after a lot of code was
 * built on the assumption it would work. So this proves the risky part first:
 *
 *   1. join the camera's Wi-Fi Direct access point
 *   2. open TCP to port 15740
 *   3. PTP/IP Init Command Request  -> expect Init Command Ack
 *   4. PTP/IP Init Event Request    -> expect Init Event Ack
 *   5. PTP OpenSession              -> expect response code 0x2001 (OK)
 *
 * If all five pass, the camera is controllable and the rest is just sending
 * the right opcodes. If step 3 fails, this route is dead too and we go wired,
 * having spent one sketch finding out instead of ten.
 *
 * CAMERA SETUP
 *   MENU > Network > Transfer/Remote > PC Remote Function
 *      PC Remote             : On
 *      PC Remote Cnct Method : Wi-Fi Direct
 *      Wi-Fi Direct Info     : read the SSID and password, put them below
 *
 * Fill these in before flashing:
 */
// ---------------------------------------------------------------------------
// CONN_MODE 0 = Wi-Fi Direct  (camera is its own access point)
//             + no pairing step, camera is always the gateway address
//             + works on location with no router at all
//             - the ESP32 has no other network while connected
//
// CONN_MODE 1 = Wi-Fi Access Point (camera and ESP32 both join your router)
//             + keeps the ESP32 on your normal network
//             + convenient at a desk
//             - the camera menu lists a PAIRING step for this mode, which may
//               be the same kind of wall we hit over Bluetooth
//             - the camera gets a DHCP address, so we must find it
// ---------------------------------------------------------------------------
#define CONN_MODE 0

#if CONN_MODE == 0
  // From the camera: Wi-Fi Direct Info
  #define WIFI_SSID  "DIRECT-kPU1:ILME-FX30"
  #define WIFI_PASS  "1Z4RBaE7"
#else
  // Your router
  #define WIFI_SSID  "office-wifi"
  #define WIFI_PASS  "your-password"
  // The camera's IP on that network. The camera shows it under
  // Network > Wi-Fi > Wi-Fi Information (or your router's client list).
  // Leave as "" to sweep the subnet for a device with port 15740 open.
  #define CAM_IP_STR ""
#endif

#include <Arduino.h>
#include <WiFi.h>

#define PTPIP_PORT 15740

static WiFiClient cmdSock;    // command channel
static WiFiClient evtSock;    // event channel
static uint32_t   gConnNumber = 0;

// Any fixed 16-byte GUID identifying this client to the camera.
static const uint8_t CLIENT_GUID[16] = {
  0x4f, 0x70, 0x65, 0x6e, 0x43, 0x6c, 0x69, 0x63,
  0x6b, 0x43, 0x33, 0x00, 0x01, 0x02, 0x03, 0x04
};
static const char* CLIENT_NAME = "OpenClick";

// ------------------------------------------------------------- utilities ---

static void hexdump(const uint8_t* d, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (d[i] < 0x10) Serial.print('0');
    Serial.print(d[i], HEX);
    Serial.print(' ');
    if ((i & 15) == 15) Serial.println();
  }
  Serial.println();
}

static void put32(uint8_t* p, uint32_t v) {
  p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}
static uint32_t get32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// PTP/IP strings are UTF-16LE, null terminated.
static size_t putUtf16(uint8_t* p, const char* s) {
  size_t n = 0;
  while (*s) { p[n++] = (uint8_t)*s++; p[n++] = 0x00; }
  p[n++] = 0x00; p[n++] = 0x00;
  return n;
}

// Read one PTP/IP packet: 4-byte length prefix, then the rest.
static int readPacket(WiFiClient& sock, uint8_t* buf, size_t cap, uint32_t timeoutMs = 5000) {
  uint32_t t0 = millis();
  while (sock.available() < 4) {
    if (!sock.connected()) { Serial.println("   socket closed while waiting"); return -1; }
    if (millis() - t0 > timeoutMs) { Serial.println("   TIMEOUT waiting for reply"); return -1; }
    delay(10);
  }
  sock.read(buf, 4);
  uint32_t len = get32(buf);
  if (len < 8 || len > cap) { Serial.printf("   bad packet length %lu\n", (unsigned long)len); return -1; }
  size_t got = 4;
  while (got < len) {
    if (millis() - t0 > timeoutMs) { Serial.println("   TIMEOUT mid-packet"); return -1; }
    if (sock.available()) got += sock.read(buf + got, len - got);
    else delay(5);
  }
  return (int)len;
}

// ------------------------------------------------------------ handshake ----

static bool initCommandChannel(IPAddress camIp) {
  Serial.printf("\n[3] Init Command Request -> %s:%d\n", camIp.toString().c_str(), PTPIP_PORT);
  if (!cmdSock.connect(camIp, PTPIP_PORT)) {
    Serial.println("   TCP connect FAILED - is PC Remote set to On?");
    return false;
  }
  Serial.println("   TCP connected");

  uint8_t pkt[128];
  size_t n = 8;
  memcpy(pkt + n, CLIENT_GUID, 16); n += 16;
  n += putUtf16(pkt + n, CLIENT_NAME);
  put32(pkt + n, 0x00010000); n += 4;      // protocol version
  put32(pkt + 0, (uint32_t)n);             // length
  put32(pkt + 4, 0x00000001);              // type: Init Command Request

  cmdSock.write(pkt, n);
  cmdSock.flush();

  uint8_t rx[512];
  int len = readPacket(cmdSock, rx, sizeof(rx));
  if (len < 0) return false;
  uint32_t type = get32(rx + 4);
  Serial.printf("   reply %d bytes, type 0x%08lX\n", len, (unsigned long)type);
  hexdump(rx, len > 64 ? 64 : len);

  if (type != 0x00000002) {
    Serial.println("   NOT an Init Command Ack (expected type 2)");
    if (type == 0x00000005) Serial.println("   -> camera sent Init Fail: it refused this client");
    return false;
  }
  gConnNumber = get32(rx + 8);
  Serial.printf("   ACK, connection number %lu\n", (unsigned long)gConnNumber);
  return true;
}

static bool initEventChannel(IPAddress camIp) {
  Serial.println("\n[4] Init Event Request");
  if (!evtSock.connect(camIp, PTPIP_PORT)) { Serial.println("   event TCP connect FAILED"); return false; }

  uint8_t pkt[12];
  put32(pkt + 0, 12);
  put32(pkt + 4, 0x00000003);            // type: Init Event Request
  put32(pkt + 8, gConnNumber);
  evtSock.write(pkt, 12);
  evtSock.flush();

  uint8_t rx[128];
  int len = readPacket(evtSock, rx, sizeof(rx));
  if (len < 0) return false;
  uint32_t type = get32(rx + 4);
  Serial.printf("   reply type 0x%08lX %s\n", (unsigned long)type,
                type == 0x00000004 ? "(Init Event Ack - good)" : "(unexpected)");
  return type == 0x00000004;
}

// Operation Request: type 6. dataPhase 1 = no data, opcode, txn id, up to 5 params.
static bool ptpOperation(uint16_t opcode, uint32_t txn, const uint32_t* params, int nParams,
                         const char* label) {
  uint8_t pkt[64];
  size_t n = 8;
  put32(pkt + n, 0x00000001); n += 4;      // data phase info: no data
  pkt[n++] = opcode & 0xFF; pkt[n++] = (opcode >> 8) & 0xFF;
  put32(pkt + n, txn); n += 4;
  for (int i = 0; i < nParams; i++) { put32(pkt + n, params[i]); n += 4; }
  put32(pkt + 0, (uint32_t)n);
  put32(pkt + 4, 0x00000006);              // type: Operation Request

  Serial.printf("\n[5] %s (opcode 0x%04X)\n", label, opcode);
  cmdSock.write(pkt, n);
  cmdSock.flush();

  uint8_t rx[512];
  int len = readPacket(cmdSock, rx, sizeof(rx));
  if (len < 0) return false;
  uint32_t type = get32(rx + 4);
  if (type == 0x00000007) {                // Operation Response
    uint16_t code = rx[8] | (rx[9] << 8);
    Serial.printf("   response code 0x%04X %s\n", code,
                  code == 0x2001 ? "(OK)" : "(not OK)");
    return code == 0x2001;
  }
  Serial.printf("   unexpected packet type 0x%08lX\n", (unsigned long)type);
  hexdump(rx, len > 48 ? 48 : len);
  return false;
}

// ---------------------------------------------------------------- setup ----

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) delay(10);
  delay(300);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(20);
#endif

  Serial.println("\n=== Sony_PtpIp_Probe " __DATE__ " " __TIME__ " ===");
  Serial.printf("[1] mode %s, joining \"%s\"\n",
                CONN_MODE == 0 ? "Wi-Fi Direct" : "router", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 25000) {
    delay(400); Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
#if CONN_MODE == 0
    Serial.println("   JOIN FAILED. Check SSID/password from Wi-Fi Direct Info,");
    Serial.println("   and that the camera is showing its Wi-Fi Direct screen.");
#else
    Serial.println("   JOIN FAILED. Check the router SSID/password.");
#endif
    return;
  }

  IPAddress ip = WiFi.localIP();
  Serial.printf("[2] joined. our IP %s, RSSI %d\n", ip.toString().c_str(), WiFi.RSSI());

  IPAddress gw;
#if CONN_MODE == 0
  // The camera IS the access point, so it is the gateway. No guessing.
  gw = WiFi.gatewayIP();
  Serial.printf("    camera (gateway) %s\n", gw.toString().c_str());
#else
  if (strlen(CAM_IP_STR) > 0) {
    if (!gw.fromString(CAM_IP_STR)) { Serial.println("    CAM_IP_STR is not a valid IP"); return; }
    Serial.printf("    camera (configured) %s\n", gw.toString().c_str());
  } else {
    // Sweep the /24 for anything answering on the PTP/IP port.
    Serial.println("    no CAM_IP_STR set, sweeping subnet for port 15740 ...");
    IPAddress base = ip; bool found = false;
    for (int host = 1; host <= 254 && !found; host++) {
      IPAddress cand(base[0], base[1], base[2], host);
      if (cand == ip) continue;
      WiFiClient probe;
      if (probe.connect(cand, PTPIP_PORT, 150)) {
        probe.stop();
        gw = cand; found = true;
        Serial.printf("    found camera at %s\n", gw.toString().c_str());
      }
      if ((host % 32) == 0) Serial.print('.');
    }
    Serial.println();
    if (!found) {
      Serial.println("    no device with port 15740 open.");
      Serial.println("    Causes: PC Remote off, camera on another subnet, or the");
      Serial.println("    router isolates clients from each other (common on office");
      Serial.println("    and guest networks) - that alone will block this route.");
      return;
    }
  }
#endif

  if (!initCommandChannel(gw)) { Serial.println("\n>>> STOPPED at command channel"); return; }
  if (!initEventChannel(gw))   { Serial.println("\n>>> STOPPED at event channel");   return; }

  // OpenSession, session id 1
  uint32_t p[1] = { 1 };
  if (!ptpOperation(0x1002, 0, p, 1, "OpenSession")) {
    Serial.println("\n>>> STOPPED at OpenSession");
    return;
  }

  Serial.println("\n**************************************************");
  Serial.println("*  HANDSHAKE COMPLETE - the camera is talking.   *");
  Serial.println("*  Next step: Sony vendor opcodes for record.    *");
  Serial.println("**************************************************");
}

void loop() {
  static uint32_t b = 0;
  if (millis() - b > 5000) {
    b = millis();
    Serial.printf("[%lu] wifi=%d cmdSock=%d evtSock=%d\n", (unsigned long)millis(),
                  WiFi.status() == WL_CONNECTED, cmdSock.connected(), evtSock.connected());
  }
  delay(20);
}
