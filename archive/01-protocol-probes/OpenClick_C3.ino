/*
 * OpenClick C3 - BLE remote for Sony cameras (tested target: ILME-FX30)
 *
 * Board   : ESP32-C3 with 0.42" SSD1306 OLED (01space / SuperMini+OLED style)
 * Button  : on-board BOOT button, GPIO9, active LOW, internal pull-up
 * Display : SSD1306 72x40 over I2C, SDA=GPIO5, SCL=GPIO6
 *
 * Based on the OpenClick Lite concept by Gokux, rewritten for the C3 and
 * fixed in the places that stop the original from actually connecting:
 *
 *   1. BLESecurity was never instantiated in the original, so the ESP32 never
 *      advertised bonding capability and 2020+ Sony bodies (FX30 included)
 *      drop the link right after connect. Now configured for bond + Just Works.
 *   2. BLEDevice::setEncryptionLevel() no longer exists in arduino-esp32 3.x.
 *      Removed; replaced by the BLESecurity config above.
 *   3. The photo sequence released focus before the shutter
 *      (0107 -> 0109 -> 0106 -> 0108). Correct order is
 *      0107 -> 0109 -> 0108 -> 0106.
 *   4. Scanning was blocking for 30 s inside loop(), so the button did nothing
 *      most of the time. Scan is now asynchronous.
 *   5. connectToServer() leaked a new BLEClient on every retry. One client is
 *      created and reused.
 *
 * Button map (single BOOT button):
 *   tap      (< 800 ms)   -> record start/stop, or take photo
 *   hold     (>= 1.5 s)   -> switch VIDEO <-> PHOTO
 *   long hold(>= 6 s)     -> erase BLE bonds and reboot (re-pair from scratch)
 *
 * Libraries needed (Library Manager):
 *   U8g2 by oliver
 *   Adafruit NeoPixel   (only if OC_USE_RGB_LED is set to 1)
 *
 * Board package: esp32 by Espressif, 3.x. Select "ESP32C3 Dev Module",
 * and make sure USB CDC On Boot is Enabled if you want Serial over USB.
 */

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLESecurity.h>
#include <BLEAdvertisedDevice.h>
#include <nvs_flash.h>

// arduino-esp32 3.x ships in two flavours: Bluedroid or NimBLE. The BLE
// wrapper API is the same for almost everything we use, but bond storage and
// the auth-complete callback differ, so branch on the stack that is present.
#if defined(CONFIG_BLUEDROID_ENABLED)
  #include <esp_gap_ble_api.h>
#elif defined(CONFIG_NIMBLE_ENABLED)
  #include <host/ble_store.h>
#else
  #error "No BLE host found. In Arduino IDE pick an ESP32-C3 board with Bluetooth enabled."
#endif

// ---------------------------------------------------------------- config ---

// Leave empty to connect to any Sony camera that is advertising remote
// control support. Set it to part of your camera's Bluetooth name to lock
// onto one specific body, e.g. "ILME-FX30".
#define OC_CAMERA_NAME_FILTER ""

// Strongest way to lock on: the camera's own Bluetooth address, which the FX30
// prints in its Bluetooth menu. When this is set the name and the advertising
// capability flags are ignored completely, so it still works if the body
// advertises without a name or with flags this sketch does not recognise.
// Separators and case do not matter. Empty = fall back to name/flag matching.
#define OC_CAMERA_MAC "9C:50:D1:AE:AC:28"

#define OC_BUTTON_PIN        9      // on-board BOOT button
#define OC_BUTTON_ACTIVE_LOW 1

#define OC_OLED_SDA          5
#define OC_OLED_SCL          6

// The 01space board also has a WS2812. Pin varies between clones (GPIO2 or
// GPIO8) and GPIO8 is a strapping pin, so it is off by default. The OLED is
// the status indicator. The generic C3 variant exposes PIN_RGB_LED (8) if you
// want the core's own idea of where it lives.
// Every macro here is OC_-prefixed: the esp32c3 variant header already defines
// RGB_BRIGHTNESS, and plain names collide with it.
#define OC_USE_RGB_LED   0
#define OC_RGB_LED_PIN   2
#define OC_RGB_BRIGHTNESS 40

#define OC_TAP_MAX_MS        800
#define OC_MODE_HOLD_MS      1500
#define OC_WIPE_HOLD_MS      6000
#define OC_DEBOUNCE_MS       25

#define OC_SCAN_SECONDS      6
#define OC_CONNECT_TIMEOUT_MS 8000
// After a drop, try connecting straight to the known address this many times
// before falling back to scanning. A bonded Sony body often stops putting out
// the advertisement we matched on during pairing, so scanning can never find
// it again - but a direct connect to its address still works.
#define OC_DIRECT_ATTEMPTS   3
#define OC_BOND_TIMEOUT_MS   12000

// A stale bond is the classic cause of "camera never answers the pairing
// request". If the ESP32 still holds an LTK for this camera but the camera's
// paired-device list is empty, ble_gap_security_initiate() asks to ENCRYPT with
// a key the camera has never seen, instead of asking to PAIR. The camera has no
// matching record, so it just ignores it. Wiping our side forces a fresh pair.
#define OC_WIPE_BONDS_ON_BOOT 1

// 1 = we initiate pairing. 0 = stay quiet and let the camera initiate, which is
// what happens when the body is sitting in its Bluetooth > Pairing screen.
// If 1 times out, try 0.
#define OC_INITIATE_SECURITY  1

// Pairing flavour. Sony bodies of this generation are widely reported to use
// LEGACY Just Works pairing. If we advertise LE Secure Connections and the
// camera only speaks legacy, some peers answer with Pairing Failed - but others
// simply say nothing, which is exactly what the FX30 is doing.
//   0 = bond only, legacy Just Works   (try this first)
//   1 = bond + LE Secure Connections
//   2 = bond + SC + MITM
#define OC_PAIR_MODE 0

// ------------------------------------------------------------ BLE protocol -

// Sony remote control service. Command channel is written to, notify channel
// reports back shutter / record state.
static BLEUUID SONY_REMOTE_SERVICE("8000FF00-FF00-FFFF-FFFF-FFFFFFFFFFFF");
static const uint16_t CHAR_COMMAND = 0xFF01;
static const uint16_t CHAR_NOTIFY  = 0xFF02;

static uint8_t CMD_RECORD_DOWN[]  = {0x01, 0x0F};
static uint8_t CMD_RECORD_UP[]    = {0x01, 0x0E};
static uint8_t CMD_FOCUS_DOWN[]   = {0x01, 0x07};
static uint8_t CMD_FOCUS_UP[]     = {0x01, 0x06};
static uint8_t CMD_SHUTTER_DOWN[] = {0x01, 0x09};
static uint8_t CMD_SHUTTER_UP[]   = {0x01, 0x08};

// --------------------------------------------------------------- globals ---

U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, /*reset=*/U8X8_PIN_NONE,
                                   /*clock=*/OC_OLED_SCL, /*data=*/OC_OLED_SDA);

#if OC_USE_RGB_LED
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel rgb(1, OC_RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
#endif

enum Mode { VIDEO_MODE, PHOTO_MODE };
enum LinkState { ST_IDLE, ST_SCANNING, ST_CONNECTING, ST_READY };

static BLEClient*              gClient        = nullptr;
static BLERemoteCharacteristic* gCommandChar  = nullptr;
static BLERemoteCharacteristic* gNotifyChar   = nullptr;
static BLEAdvertisedDevice      gFoundDevice;

static volatile bool  gHaveTarget   = false;   // set by scan callback
static volatile bool  gConnected    = false;
static volatile bool  gCameraInPairingMode = false;
static volatile bool  gScanRunning  = false;

static LinkState gState      = ST_IDLE;
static Mode      gMode       = VIDEO_MODE;
static bool      gRecording  = false;
static char      gCameraName[24] = "";
static char      gStatusLine[20] = "";
static uint32_t  gStatusUntil = 0;
static uint32_t  gLastScanStart = 0;
static uint32_t  gConnectedAt   = 0;
static uint16_t  gDropCount     = 0;
static char      gLastError[16]  = "";
static bool      gEverConnected  = false;
static uint8_t   gDirectTries    = 0;
static bool      gTryDirect      = false;
static volatile bool gBondDone   = false;
static volatile bool gBondOk     = false;
static bool      gDisplayDirty = true;

// ------------------------------------------------------------- utilities ---

static void setError(const char* e) {
  strncpy(gLastError, e, sizeof(gLastError) - 1);
  gLastError[sizeof(gLastError) - 1] = '\0';
  gDisplayDirty = true;
}

static void setStatus(const char* msg, uint32_t ms = 1200) {
  strncpy(gStatusLine, msg, sizeof(gStatusLine) - 1);
  gStatusLine[sizeof(gStatusLine) - 1] = '\0';
  gStatusUntil = millis() + ms;
  gDisplayDirty = true;
}

// Compare two MAC strings ignoring case and any separator characters, so
// "9C:50:D1:AE:AC:28", "9c50d1aeac28" and "9c-50-d1-ae-ac-28" all match.
static bool macEquals(const char* a, const char* b) {
  if (!a || !b) return false;
  while (*a && *b) {
    while (*a && !isxdigit((unsigned char)*a)) a++;
    while (*b && !isxdigit((unsigned char)*b)) b++;
    if (!*a || !*b) break;
    if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
    a++; b++;
  }
  while (*a && !isxdigit((unsigned char)*a)) a++;
  while (*b && !isxdigit((unsigned char)*b)) b++;
  return (*a == '\0' && *b == '\0');
}

static void printHex(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
}

// Re-applied right before each pairing attempt. BLEDevice::init() can settle
// asynchronously on NimBLE, so a config written once at boot is not guaranteed
// to be the config in force when we actually initiate.
static void applySecurityConfig() {
  BLESecurity* sec = new BLESecurity();
#if OC_PAIR_MODE == 0
  sec->setAuthenticationMode(true, false, false);   // bond, no MITM, no SC
#elif OC_PAIR_MODE == 1
  sec->setAuthenticationMode(true, false, true);    // bond + SC
#else
  sec->setAuthenticationMode(true, true, true);     // bond + SC + MITM
#endif
  sec->setCapability(ESP_IO_CAP_NONE);              // Just Works
  sec->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  sec->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
}

// Print what the stack is really going to send, rather than what we asked for.
static void dumpSecurityConfig(const char* when) {
#if defined(CONFIG_NIMBLE_ENABLED)
  Serial.printf("   sm cfg @%s: bonding=%d mitm=%d sc=%d iocap=%d our_key=0x%02X their_key=0x%02X\n",
                when, ble_hs_cfg.sm_bonding, ble_hs_cfg.sm_mitm, ble_hs_cfg.sm_sc,
                ble_hs_cfg.sm_io_cap, ble_hs_cfg.sm_our_key_dist, ble_hs_cfg.sm_their_key_dist);
#endif
}

static void clearBonds() {
#if defined(CONFIG_BLUEDROID_ENABLED)
  int n = esp_ble_get_bond_device_num();
  if (n <= 0) {
    Serial.println("No bonds stored.");
    return;
  }
  esp_ble_bond_dev_t* list = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * n);
  if (!list) return;
  esp_ble_get_bond_device_list(&n, list);
  for (int i = 0; i < n; i++) {
    esp_ble_remove_bond_device(list[i].bd_addr);
  }
  free(list);
  Serial.printf("Removed %d bond(s).\n", n);
#elif defined(CONFIG_NIMBLE_ENABLED)
  int rc = ble_store_clear();
  Serial.printf("ble_store_clear() -> %d\n", rc);
#endif
}

// --------------------------------------------------------------- display ---

static void drawDisplay() {
  u8g2.clearBuffer();

  // Top line: link state
  u8g2.setFont(u8g2_font_5x8_tr);
  const char* top = "IDLE";
  switch (gState) {
    case ST_SCANNING:   top = gCameraInPairingMode ? "PAIRING" : "SEARCHING"; break;
    case ST_CONNECTING: top = "CONNECTING"; break;
    case ST_READY:      top = gCameraName[0] ? gCameraName : "CONNECTED"; break;
    default:            top = "IDLE"; break;
  }
  u8g2.drawStr(0, 7, top);
  u8g2.drawHLine(0, 9, 72);

  // Middle: mode + big state
  if (gState == ST_READY) {
    u8g2.setFont(u8g2_font_7x13B_tr);
    if (gMode == VIDEO_MODE) {
      if (gRecording) {
        u8g2.drawStr(0, 24, "REC");
        u8g2.drawDisc(50, 19, 5);           // filled dot = rolling
      } else {
        u8g2.drawStr(0, 24, "VIDEO");
      }
    } else {
      u8g2.drawStr(0, 24, "PHOTO");
    }
  } else {
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(0, 18, gLastError[0] ? gLastError : "no camera");
    if (gDropCount) {
      char d[16];
      snprintf(d, sizeof(d), "drops:%u", gDropCount);
      u8g2.drawStr(0, 27, d);
    }
  }

  // Bottom: transient status or hint
  u8g2.setFont(u8g2_font_5x8_tr);
  if (gStatusLine[0] && millis() < gStatusUntil) {
    u8g2.drawStr(0, 38, gStatusLine);
  } else {
    gStatusLine[0] = '\0';
    u8g2.drawStr(0, 38, gState == ST_READY ? "tap=go hold=mode" : "hold 6s = reset");
  }

  u8g2.sendBuffer();
}

static void updateRgb() {
#if OC_USE_RGB_LED
  uint32_t c;
  if (gState != ST_READY)      c = rgb.Color(0, 0, OC_RGB_BRIGHTNESS);              // blue
  else if (gMode == PHOTO_MODE) c = rgb.Color(OC_RGB_BRIGHTNESS / 2, 0, OC_RGB_BRIGHTNESS); // violet
  else if (gRecording)          c = rgb.Color(OC_RGB_BRIGHTNESS, 0, 0);             // red
  else                          c = rgb.Color(0, OC_RGB_BRIGHTNESS, 0);             // green
  rgb.setPixelColor(0, c);
  rgb.show();
#endif
}

// ------------------------------------------------------------- BLE events --

static void onCameraNotify(BLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify) {
  Serial.print("notify: ");
  printHex(data, len);
  Serial.println();

  // 02 D5 xx : recording state. 0x20 = started, 0x00 = stopped.
  if (len >= 3 && data[0] == 0x02 && data[1] == 0xD5) {
    bool rec = (data[2] == 0x20);
    if (rec != gRecording) {
      gRecording = rec;
      gDisplayDirty = true;
    }
  }
}

class ScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (gHaveTarget || gConnected) return;

    auto nameObj = advertisedDevice.getName();     // String on 3.x, std::string on 2.x
    const char* name = nameObj.c_str();
    auto addrObj = advertisedDevice.getAddress().toString();
    const char* addr = addrObj.c_str();
    const bool  named = (name && name[0]);

    uint8_t* payload = advertisedDevice.getPayload();
    size_t   plen    = advertisedDevice.getPayloadLength();

    Serial.printf("[%lu] adv %s %-16s rssi=%4d  ", (unsigned long)millis(),
                  addr, named ? name : "(no name)", advertisedDevice.getRSSI());
    printHex(payload, plen);
    Serial.println();

    bool pairing = false;
    for (size_t i = 1; i < plen; i++) {
      if (payload[i - 1] == 0x22 && (payload[i] & 0x40)) pairing = true;
    }

    if (strlen(OC_CAMERA_MAC) > 0) {
      // Address match: trust it over everything else.
      if (!macEquals(addr, OC_CAMERA_MAC)) return;
      Serial.println("   ^ MAC match - taking it, ignoring name and flags");
    } else {
      // No address configured: fall back to name + capability flags. This path
      // needs a name, and needs the 0x22 flag byte this sketch knows about.
      if (!named) return;
      if (strlen(OC_CAMERA_NAME_FILTER) > 0 && strstr(name, OC_CAMERA_NAME_FILTER) == nullptr) {
        return;
      }
      bool isSony = false;
      for (size_t i = 1; i < plen; i++) {
        if (payload[i - 1] == 0x22 && (payload[i] & 0x42)) isSony = true;
      }
      if (!isSony) {
        Serial.println("   ^ no 0x22 remote-control flag, skipping");
        return;
      }
      Serial.println("   ^ name/flag match");
    }

    snprintf(gCameraName, sizeof(gCameraName), "%s", named ? name : addr);
    gCameraInPairingMode = pairing;
    gFoundDevice = advertisedDevice;
    gHaveTarget = true;

    advertisedDevice.getScan()->stop();
  }
};

static void onScanFinished(BLEScanResults results) {
  gScanRunning = false;
}

class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* c) override {
    Serial.println("BLE connected");
  }
  void onDisconnect(BLEClient* c) override {
    uint32_t up = gConnectedAt ? (millis() - gConnectedAt) : 0;
    gDropCount++;
    Serial.printf("[%lu] BLE disconnected after %lu ms (drop #%u)\n",
                  (unsigned long)millis(), (unsigned long)up, gDropCount);
    gConnected    = false;
    gRecording    = false;
    gCommandChar  = nullptr;
    gNotifyChar   = nullptr;
    gState        = ST_IDLE;
    gDisplayDirty = true;
  }
};

class SecurityCallbacks : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() override {
    Serial.println("passkey requested");
    return 0;
  }
  void onPassKeyNotify(uint32_t pass_key) override {
    Serial.printf("passkey: %06u\n", pass_key);
  }
  bool onSecurityRequest() override {
    Serial.println("security request -> accept");
    return true;
  }
  bool onConfirmPIN(uint32_t pin) override {
    return true;
  }
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override {
    if (cmpl.success) {
      Serial.println("bonded OK");
      gBondOk = true;
      setStatus("paired");
    } else {
      Serial.printf("!! bonding FAILED, reason 0x%02X\n", cmpl.fail_reason);
      gBondOk = false;
      setStatus("pair failed", 2500);
    }
    gBondDone = true;
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onAuthenticationComplete(ble_gap_conn_desc* desc) override {
    if (desc && desc->sec_state.encrypted) {
      Serial.printf("bonded OK (encrypted=1 bonded=%d authenticated=%d)\n",
                    desc->sec_state.bonded, desc->sec_state.authenticated);
      gBondOk = true;
      setStatus("paired");
    } else {
      Serial.println("!! bonding FAILED / link not encrypted");
      gBondOk = false;
      setStatus("pair failed", 2500);
    }
    gBondDone = true;
  }
#endif
};

// ------------------------------------------------------------ BLE control --

static void startScan() {
  if (gScanRunning || gConnected) return;

  gHaveTarget = false;
  gState = ST_SCANNING;
  gDisplayDirty = true;

  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  gScanRunning = true;
  gLastScanStart = millis();
  scan->start(OC_SCAN_SECONDS, onScanFinished, false);
  Serial.println("scanning...");
}

static bool connectToCamera(bool byAddress) {
  gState = ST_CONNECTING;
  gDisplayDirty = true;
  drawDisplay();

  if (!gClient) {
    gClient = BLEDevice::createClient();
    gClient->setClientCallbacks(new ClientCallbacks());
  }

  bool ok;
  if (byAddress) {
    Serial.printf("[%lu] direct connect to %s\n", (unsigned long)millis(), OC_CAMERA_MAC);
    // No advertisement on this path, so nothing has set the name. Keep whatever
    // a previous scan learned, otherwise label it by address.
    if (!gCameraName[0]) snprintf(gCameraName, sizeof(gCameraName), "%s", OC_CAMERA_MAC);
    BLEAddress addr = BLEAddress(String(OC_CAMERA_MAC));
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    // Never use the default timeout here: it is portMAX_DELAY, which would
    // block loop() forever and freeze the button.
    ok = gClient->connect(addr, 0, OC_CONNECT_TIMEOUT_MS);
#else
    ok = gClient->connect(addr);
#endif
  } else {
    Serial.printf("[%lu] connecting to %s\n", (unsigned long)millis(), gCameraName);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ok = gClient->connectTimeout(&gFoundDevice, OC_CONNECT_TIMEOUT_MS);
#else
    ok = gClient->connect(&gFoundDevice);
#endif
  }
  if (!ok) {
    Serial.printf("[%lu] connect failed\n", (unsigned long)millis());
    setError("CONN FAIL");
    gState = ST_IDLE;
    setStatus("connect fail", 1500);
    return false;
  }

  Serial.printf("[%lu] link up, starting security\n", (unsigned long)millis());
  setStatus("pairing...", 6000);
  drawDisplay();

  gBondDone = false;
  gBondOk   = false;

#if !OC_INITIATE_SECURITY
  Serial.println("   OC_INITIATE_SECURITY=0: waiting for the camera to start pairing");
#elif defined(CONFIG_NIMBLE_ENABLED)
  // Deliberately NOT BLEClient::secureConnection(): that waits on the pairing
  // result with BLE_NPL_TIME_FOREVER, so a camera that never answers wedges the
  // whole sketch with no heartbeat and a dead button. startSecurity() only
  // kicks off ble_gap_security_initiate() and returns, so we can time it out.
  applySecurityConfig();          // re-assert, in case init() clobbered it
  dumpSecurityConfig("connect");
  int rc = 0;
  bool started = BLESecurity::startSecurity(gClient->getConnId(), &rc);
  Serial.printf("[%lu] startSecurity started=%d rc=%d (%s)\n", (unsigned long)millis(),
                started ? 1 : 0, rc, BLEUtils::returnCodeToString(rc));
  if (!started) {
    Serial.println("   security not enabled - BLESecurity config did not take");
    setError("SEC OFF");
  }
#else
  bool started = gClient->secureConnection();   // no-op / instant on Bluedroid
  Serial.printf("[%lu] secureConnection -> %d\n", (unsigned long)millis(), started ? 1 : 0);
  gBondDone = true;
  gBondOk = started;
#endif

  uint32_t bondStart = millis();
  while (!gBondDone && (millis() - bondStart) < OC_BOND_TIMEOUT_MS) {
    if (!gClient->isConnected()) {
      Serial.printf("[%lu] camera dropped us during pairing after %lu ms\n",
                    (unsigned long)millis(), (unsigned long)(millis() - bondStart));
      setError("DROP@PAIR");
      gState = ST_IDLE;
      return false;
    }
    delay(50);
  }

  if (!gBondDone) {
    Serial.printf("[%lu] pairing TIMED OUT after %d ms - camera never responded.\n",
                  (unsigned long)millis(), OC_BOND_TIMEOUT_MS);
    Serial.println("   Put the camera in Bluetooth > Pairing and try again.");
    setError("PAIR T/O");
  } else if (!gBondOk) {
    setError("NO BOND");
  }

  // Service discovery can briefly race the end of the security exchange, so
  // give it a few attempts rather than one shot.
  BLERemoteService* svc = nullptr;
  for (int attempt = 0; attempt < 4 && svc == nullptr; attempt++) {
    if (attempt) delay(400);
    svc = gClient->getService(SONY_REMOTE_SERVICE);
    if (!gClient->isConnected()) {
      Serial.println("dropped during service discovery");
      gState = ST_IDLE;
      return false;
    }
  }
  if (svc == nullptr) {
    Serial.println("remote service not found - is 'Bluetooth Rmt Ctrl' on?");
    setError("NO SERVICE");
    gClient->disconnect();
    gState = ST_IDLE;
    setStatus("no service", 2000);
    return false;
  }

  gCommandChar = svc->getCharacteristic(BLEUUID(CHAR_COMMAND));
  gNotifyChar  = svc->getCharacteristic(BLEUUID(CHAR_NOTIFY));

  if (gCommandChar == nullptr) {
    Serial.println("command characteristic missing");
    setError("NO CMD CHR");
    gClient->disconnect();
    gState = ST_IDLE;
    setStatus("no cmd char", 2000);
    return false;
  }

  if (gCommandChar->canNotify()) gCommandChar->registerForNotify(onCameraNotify);
  if (gNotifyChar && gNotifyChar->canNotify()) gNotifyChar->registerForNotify(onCameraNotify);

  if (gBondOk) gLastError[0] = '\0';   // keep PAIR T/O / NO BOND visible
  gEverConnected = true;
  gDirectTries = 0;
  gConnected = true;
  gConnectedAt = millis();
  gRecording = false;
  gState = ST_READY;
  gDisplayDirty = true;
  setStatus("ready");
  Serial.println("camera ready");
  return true;
}

static bool writeCommand(uint8_t* cmd) {
  if (!gConnected || gCommandChar == nullptr) {
    Serial.println("write skipped: not connected");
    return false;
  }
  bool ok = gCommandChar->writeValue(cmd, 2, true);
  Serial.printf("[%lu] write %02X %02X -> %s\n", (unsigned long)millis(),
                cmd[0], cmd[1], ok ? "ACK" : "REJECTED (needs bonding?)");
  return ok;
}

static void toggleRecording() {
  if (!gConnected) return;
  // The record button is a momentary press: down then up. The camera toggles.
  writeCommand(CMD_RECORD_DOWN);
  delay(60);
  writeCommand(CMD_RECORD_UP);
  // Optimistic flip; the 02 D5 notify will correct us if the camera disagrees.
  gRecording = !gRecording;
  gDisplayDirty = true;
  setStatus(gRecording ? "rec start" : "rec stop");
}

static void takePhoto() {
  if (!gConnected) return;
  writeCommand(CMD_FOCUS_DOWN);    // S1 down
  delay(120);
  writeCommand(CMD_SHUTTER_DOWN);  // S2 down
  delay(120);
  writeCommand(CMD_SHUTTER_UP);    // S2 up
  delay(60);
  writeCommand(CMD_FOCUS_UP);      // S1 up
  setStatus("shot");
}

static void switchMode() {
  gMode = (gMode == VIDEO_MODE) ? PHOTO_MODE : VIDEO_MODE;
  gRecording = false;
  gDisplayDirty = true;
  setStatus(gMode == VIDEO_MODE ? "video mode" : "photo mode");
  Serial.println(gMode == VIDEO_MODE ? "VIDEO mode" : "PHOTO mode");
}

// ---------------------------------------------------------------- button ---

static bool buttonRaw() {
#if OC_BUTTON_ACTIVE_LOW
  return digitalRead(OC_BUTTON_PIN) == LOW;
#else
  return digitalRead(OC_BUTTON_PIN) == HIGH;
#endif
}

static void serviceButton() {
  static bool     stable      = false;
  static bool     lastRaw     = false;
  static uint32_t lastChange  = 0;
  static uint32_t pressStart  = 0;
  static bool     modeFired   = false;
  static bool     wipeFired   = false;

  bool raw = buttonRaw();
  uint32_t now = millis();

  if (raw != lastRaw) {
    lastRaw = raw;
    lastChange = now;
  }

  if ((now - lastChange) < OC_DEBOUNCE_MS) return;
  if (raw == stable) {
    // held: check the two hold thresholds while still down
    if (stable) {
      uint32_t held = now - pressStart;
      if (!wipeFired && held >= OC_WIPE_HOLD_MS) {
        wipeFired = true;
        setStatus("erasing bonds", 1500);
        drawDisplay();
        clearBonds();
        delay(600);
        ESP.restart();
      }
      if (!modeFired && held >= OC_MODE_HOLD_MS) {
        modeFired = true;
        switchMode();
      }
    }
    return;
  }

  stable = raw;

  if (stable) {                 // pressed
    pressStart = now;
    modeFired = false;
    wipeFired = false;
  } else {                      // released
    uint32_t held = now - pressStart;
    if (held <= OC_TAP_MAX_MS && !modeFired && !wipeFired) {
      if (!gConnected) {
        setStatus("not linked", 1200);
      } else if (gMode == VIDEO_MODE) {
        toggleRecording();
      } else {
        takePhoto();
      }
    }
  }
}

// ----------------------------------------------------------------- setup ---

void setup() {
  Serial.begin(115200);
  // Native USB CDC enumerates after boot; without this the first prints are
  // lost. Needs Tools > USB CDC On Boot: Enabled.
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) delay(10);
  delay(300);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  // Without this a write blocks indefinitely when nothing is reading the port.
  Serial.setTxTimeoutMs(20);
#endif

  pinMode(OC_BUTTON_PIN, OC_BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);

  u8g2.begin();
  u8g2.setContrast(180);

#if OC_USE_RGB_LED
  rgb.begin();
  rgb.clear();
  rgb.show();
#endif

  setStatus("booting", 800);
  drawDisplay();

  Serial.println();
  Serial.println("========================================");
  Serial.println("OpenClick C3 - Sony BLE remote");
  Serial.println("build    : v7 heartbeat  " __DATE__ " " __TIME__);
#if defined(CONFIG_BLUEDROID_ENABLED)
  Serial.println("BLE host : Bluedroid");
#elif defined(CONFIG_NIMBLE_ENABLED)
  Serial.println("BLE host : NimBLE");
#endif
  Serial.printf("core     : %d.%d.%d\n", ESP_ARDUINO_VERSION_MAJOR,
                ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  Serial.printf("button   : GPIO%d (reads %d idle)\n", OC_BUTTON_PIN, digitalRead(OC_BUTTON_PIN));
  Serial.printf("filter   : \"%s\"\n", OC_CAMERA_NAME_FILTER);
  Serial.println("tap = shutter/record, hold 1.5s = mode, hold 6s = erase bonds");
  Serial.println("========================================");

  Serial.printf("init: heap before BLE = %u bytes\n", (unsigned)ESP.getFreeHeap());
  Serial.println("init: BLEDevice::init()...");
  esp_err_t nv = nvs_flash_init();
  if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    Serial.println("init: NVS full or stale, erasing");
    nvs_flash_erase();
    nv = nvs_flash_init();
  }
  Serial.printf("init: nvs_flash_init -> %d\n", (int)nv);

  BLEDevice::init("OpenClick C3");
  Serial.printf("init: BLE stack up, heap = %u bytes\n", (unsigned)ESP.getFreeHeap());

  // This is the part the original project was missing. Sony bodies from the
  // 2020 generation onward require a bonded, encrypted link before they will
  // accept anything on FF01, and they drop the connection outright if the
  // central cannot bond. No display and no keyboard on our side, so Just
  // Works with bonding is the right profile.
  applySecurityConfig();
  BLEDevice::setSecurityCallbacks(new SecurityCallbacks());
  Serial.println("init: security configured");
  dumpSecurityConfig("boot");
#if OC_WIPE_BONDS_ON_BOOT
  Serial.println("init: wiping stored bonds (OC_WIPE_BONDS_ON_BOOT=1)");
  clearBonds();
#endif

  BLEDevice::getScan()->setAdvertisedDeviceCallbacks(new ScanCallbacks(), false, true);

  // Do not connect from setup(). A direct connect blocks for up to
  // OC_CONNECT_TIMEOUT_MS, and doing that here means no display updates and a
  // dead button for the whole attempt, which looks exactly like a crash.
  // loop() owns all connection work.
  if (strlen(OC_CAMERA_MAC) > 0) {
    Serial.println("init: known address set, loop will try direct connect first");
    gTryDirect = true;
  } else {
    startScan();
  }
  Serial.println("init: done, entering loop");
}

void loop() {
  serviceButton();

  // A camera we spotted in a scan: connect using the advertisement we captured.
  if (gHaveTarget && !gConnected) {
    gHaveTarget = false;
    gScanRunning = false;
    BLEDevice::getScan()->clearResults();
    connectToCamera(false);
  }

  // Once we have bonded at least once, prefer connecting straight to the known
  // address. A bonded Sony body often stops emitting the advertisement we
  // matched during pairing, so waiting to see one again can hang forever, while
  // a direct connect to the same address still succeeds.
  static uint32_t lastDirect = 0;
  if (!gConnected && (gEverConnected || gTryDirect) && strlen(OC_CAMERA_MAC) > 0
      && gDirectTries < OC_DIRECT_ATTEMPTS
      && (lastDirect == 0 || (millis() - lastDirect) > 3000)) {
    lastDirect = millis();
    gDirectTries++;
    if (gScanRunning) {
      BLEDevice::getScan()->stop();
      gScanRunning = false;
    }
    Serial.printf("[%lu] direct reconnect attempt %u/%u\n", (unsigned long)millis(),
                  gDirectTries, OC_DIRECT_ATTEMPTS);
    setStatus("reconnect...", 2500);
    connectToCamera(true);
  }

  // Re-arm the scan when we are not connected and the previous one has ended.
  // After OC_DIRECT_ATTEMPTS failed direct connects we fall back here, then
  // reset the counter so the two strategies keep alternating.
  if (!gConnected && !gScanRunning && (millis() - gLastScanStart) > (OC_SCAN_SECONDS * 1000UL + 800)) {
    if (gDirectTries >= OC_DIRECT_ATTEMPTS) { gDirectTries = 0; gTryDirect = false; }
    startScan();
  }

  // Prints regardless of what BLE is doing. If this stops, the board hung or
  // reset; if it keeps going, any silence is just nothing happening.
  static uint32_t lastBeat = 0;
  if ((millis() - lastBeat) > 2000) {
    lastBeat = millis();
    const char* st = "?";
    switch (gState) {
      case ST_IDLE:       st = "IDLE";       break;
      case ST_SCANNING:   st = "SCANNING";   break;
      case ST_CONNECTING: st = "CONNECTING"; break;
      case ST_READY:      st = "READY";      break;
    }
    Serial.printf("[%lu] hb state=%-10s conn=%d scan=%d direct=%u drops=%u btn=%d heap=%u err=%s\n",
                  (unsigned long)millis(), st, gConnected ? 1 : 0, gScanRunning ? 1 : 0,
                  gDirectTries, gDropCount, digitalRead(OC_BUTTON_PIN),
                  (unsigned)ESP.getFreeHeap(), gLastError[0] ? gLastError : "-");
  }

  static uint32_t lastDraw = 0;
  if (gDisplayDirty || (millis() - lastDraw) > 250) {
    gDisplayDirty = false;
    lastDraw = millis();
    drawDisplay();
    updateRgb();
  }

  delay(5);
}
