/*
 * Sony_GattProbe - listen first, then ask.
 *
 * The GATT dump from the FX30 shows the pairing service really is there, and
 * that it has a NOTIFY channel we were ignoring:
 *
 *   8000EE00  EE01 [W]   pairing command
 *             EE02 [RW]
 *             EE03 [N]   <-- the camera answers here
 *             EE04 [R]
 *
 * Last run wrote to EE01 while subscribed to nothing, so any reply the camera
 * sent went into the void. This sketch subscribes to every notify
 * characteristic on the camera BEFORE writing anything, reads every readable
 * one, and only then sends the pairing command.
 *
 * Order of operations:
 *   1. connect
 *   2. subscribe to all [N] characteristics, logging which UUID each notify
 *      arrives on
 *   3. read and hexdump all [R] characteristics
 *   4. write 06 08 01 00 00 00 to EE01
 *   5. watch 25 s for notifications, a security request, or a disconnect
 *
 * Nothing here writes anything except the documented pairing command. The
 * power-off command is deliberately absent.
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLESecurity.h>
#include <nvs_flash.h>
#if defined(CONFIG_NIMBLE_ENABLED)
  #include <host/ble_store.h>
#endif

#define CAMERA_MAC   "9C:50:D1:AE:AC:28"
#define DO_READS     1     // 0 to skip the read sweep and go straight to pairing
#define PAIR_VARIANT 1     // 1 = 06 08 01 .. , 2 = 06 08 02 .. (with disconnect)

static BLEUUID SVC_PAIRING("8000EE00-EE00-FFFF-FFFF-FFFFFFFFFFFF");

static uint8_t PAIR_1[] = {0x06, 0x08, 0x01, 0x00, 0x00, 0x00};
static uint8_t PAIR_2[] = {0x06, 0x08, 0x02, 0x00, 0x00, 0x00};

static volatile bool gConnected = false;
static volatile bool gBondDone  = false;
static volatile bool gBondOk    = false;
static volatile int  gNotifyCount = 0;
static BLEClient* gClient = nullptr;

static void hexdump(const uint8_t* d, size_t n) {
  for (size_t i = 0; i < n; i++) { if (d[i] < 0x10) Serial.print('0'); Serial.print(d[i], HEX); Serial.print(' '); }
}

// Every notification, tagged with the characteristic it arrived on. This is the
// part that was missing: the camera may well have been replying already.
static void onAnyNotify(BLERemoteCharacteristic* c, uint8_t* d, size_t n, bool isNotify) {
  gNotifyCount++;
  Serial.printf("\n<<< NOTIFY from %s (%u bytes): ", c->getUUID().toString().c_str(), (unsigned)n);
  hexdump(d, n);
  Serial.println();
}

class ClientCB : public BLEClientCallbacks {
  void onConnect(BLEClient*) override { gConnected = true; Serial.println(">> connected"); }
  void onDisconnect(BLEClient*) override { gConnected = false; Serial.println(">> DISCONNECTED"); }
};

class SecCB : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() override { Serial.println("SEC: passkey requested -> 0"); return 0; }
  void onPassKeyNotify(uint32_t k) override { Serial.printf("SEC: *** CAMERA SHOWS PASSKEY %06u ***\n", k); }
  bool onSecurityRequest() override { Serial.println("SEC: *** CAMERA REQUESTED SECURITY *** -> accept"); return true; }
  bool onConfirmPIN(uint32_t p) override { Serial.printf("SEC: confirm %06u -> yes\n", p); return true; }
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onAuthenticationComplete(esp_ble_auth_cmpl_t c) override {
    Serial.printf("SEC: auth complete success=%d\n", c.success); gBondOk = c.success; gBondDone = true;
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onAuthenticationComplete(ble_gap_conn_desc* d) override {
    bool ok = d && d->sec_state.encrypted;
    Serial.printf("SEC: auth complete encrypted=%d bonded=%d\n", ok, (d && d->sec_state.bonded) ? 1 : 0);
    gBondOk = ok; gBondDone = true;
  }
#endif
};

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) delay(10);
  delay(300);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(20);
#endif
  Serial.println("\n=== Sony_GattProbe " __DATE__ " " __TIME__ " ===");

  esp_err_t nv = nvs_flash_init();
  if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }

  BLEDevice::init("OpenClick");
  BLESecurity* sec = new BLESecurity();
  sec->setAuthenticationMode(true, false, false);
  sec->setCapability(ESP_IO_CAP_NONE);
  sec->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  sec->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  BLEDevice::setSecurityCallbacks(new SecCB());
#if defined(CONFIG_NIMBLE_ENABLED)
  Serial.printf("ble_store_clear() -> %d\n", ble_store_clear());
#endif

  gClient = BLEDevice::createClient();
  gClient->setClientCallbacks(new ClientCB());

  Serial.printf("connecting to %s ...\n", CAMERA_MAC);
  BLEAddress addr = BLEAddress(String(CAMERA_MAC));
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!gClient->connect(addr, 0, 10000)) { Serial.println("connect FAILED"); return; }
#else
  if (!gClient->connect(addr)) { Serial.println("connect FAILED"); return; }
#endif
  delay(700);

  auto services = gClient->getServices();
  if (!services) { Serial.println("no services"); return; }

  // ---- 1. subscribe to everything that can notify, BEFORE writing ----
  Serial.println("\n--- subscribing to all notify characteristics ---");
  int subs = 0;
  for (auto& sp : *services) {
    auto chars = sp.second->getCharacteristics();
    if (!chars) continue;
    for (auto& cp : *chars) {
      BLERemoteCharacteristic* c = cp.second;
      if (c->canNotify() || c->canIndicate()) {
        c->registerForNotify(onAnyNotify);
        Serial.printf("   subscribed %s\n", c->getUUID().toString().c_str());
        subs++;
        delay(30);
      }
    }
  }
  Serial.printf("subscribed to %d characteristics\n", subs);

#if DO_READS
  // ---- 2. read everything readable; values may say what the camera expects ----
  Serial.println("\n--- reading all readable characteristics ---");
  for (auto& sp : *services) {
    auto chars = sp.second->getCharacteristics();
    if (!chars) continue;
    for (auto& cp : *chars) {
      BLERemoteCharacteristic* c = cp.second;
      if (!c->canRead()) continue;
      String v = c->readValue();
      Serial.printf("   %s = (%u) ", c->getUUID().toString().c_str(), (unsigned)v.length());
      if (v.length()) hexdump((const uint8_t*)v.c_str(), v.length());
      else Serial.print("<empty or read denied>");
      Serial.println();
      delay(30);
      if (!gConnected) { Serial.println("   ...dropped during reads"); break; }
    }
    if (!gConnected) break;
  }
#endif

  if (!gConnected) { Serial.println("disconnected before pairing write"); return; }

  // ---- 3. now ask to pair ----
  BLERemoteService* ps = gClient->getService(SVC_PAIRING);
  if (!ps) { Serial.println("pairing service vanished"); return; }
  BLERemoteCharacteristic* pc = ps->getCharacteristic(BLEUUID((uint16_t)0xEE01));
  if (!pc) { Serial.println("EE01 not found"); return; }

  uint8_t* cmd = (PAIR_VARIANT == 2) ? PAIR_2 : PAIR_1;
  size_t   len = (PAIR_VARIANT == 2) ? sizeof(PAIR_2) : sizeof(PAIR_1);
  Serial.print("\n>>> writing pairing command to EE01: ");
  hexdump(cmd, len);
  bool w = pc->writeValue(cmd, len, true);
  Serial.printf("-> %s\n", w ? "ACK" : "REJECTED");

  Serial.println("watching 25 s for notifications / security / disconnect ...");
  uint32_t t = millis();
  while (!gBondDone && gConnected && (millis() - t) < 25000) delay(50);

  Serial.println("\n===================== RESULT =====================");
  Serial.printf("notifications received : %d\n", gNotifyCount);
  Serial.printf("bonding completed      : %s\n", gBondDone ? (gBondOk ? "YES - SUCCESS" : "yes but FAILED") : "no");
  Serial.printf("still connected        : %s\n", gConnected ? "yes" : "no (camera dropped us)");
  Serial.println("==================================================");
  Serial.println("-- press RESET to run again --");
}

void loop() {
  static uint32_t b = 0;
  if (millis() - b > 5000) {
    b = millis();
    Serial.printf("[%lu] connected=%d notifies=%d bondDone=%d bondOk=%d\n",
                  (unsigned long)millis(), gConnected, gNotifyCount, gBondDone, gBondOk);
  }
  delay(20);
}
