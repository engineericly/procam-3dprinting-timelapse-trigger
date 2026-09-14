/*
 * Sony_GattExplore - dump the camera's entire GATT database, then try the
 * documented Sony PAIRING service.
 *
 * Why
 * ---
 * Initiating SMP pairing cold gets ignored by the FX30: the request goes out
 * with rc=0 and the camera never answers. HYPOXIC's protocol notes describe a
 * separate pairing service that nobody has confirmed:
 *
 *   service 8000EE00-EE00-FFFF-FFFF-FFFFFFFFFFFF
 *     characteristic EE01
 *       06 08 01 00 00 00   writePairingCommand
 *       06 08 02 00 00 00   writePairingCommandWithDisconnect
 *       03 08 13            writePowerOff        <-- NOT sent by this sketch
 *
 * The theory is that the camera wants to be TOLD to pair over GATT, and only
 * then runs the security procedure. This sketch tests that, and regardless of
 * the outcome it prints the full service/characteristic table so we stop
 * guessing what the FX30 actually exposes.
 *
 * Camera setup: Bluetooth Function On, Bluetooth Rmt Ctrl On, and open the
 * Pairing screen before resetting the board.
 *
 * Set TRY_PAIRING_WRITE to 0 if you only want the dump with nothing written.
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLESecurity.h>
#include <nvs_flash.h>
#if defined(CONFIG_NIMBLE_ENABLED)
  #include <host/ble_store.h>
#endif

#define CAMERA_MAC        "9C:50:D1:AE:AC:28"
#define TRY_PAIRING_WRITE 1
#define WIPE_BONDS        1

static BLEUUID SVC_PAIRING("8000EE00-EE00-FFFF-FFFF-FFFFFFFFFFFF");
static BLEUUID SVC_REMOTE ("8000FF00-FF00-FFFF-FFFF-FFFFFFFFFFFF");

static uint8_t PAIR_CMD[]            = {0x06, 0x08, 0x01, 0x00, 0x00, 0x00};
static uint8_t PAIR_CMD_DISCONNECT[] = {0x06, 0x08, 0x02, 0x00, 0x00, 0x00};

static volatile bool gConnected = false;
static volatile bool gBondDone  = false;
static volatile bool gBondOk    = false;
static BLEClient* gClient = nullptr;

static void hexdump(const uint8_t* d, size_t n) {
  for (size_t i = 0; i < n; i++) { if (d[i] < 0x10) Serial.print('0'); Serial.print(d[i], HEX); Serial.print(' '); }
}

class ClientCB : public BLEClientCallbacks {
  void onConnect(BLEClient*) override { gConnected = true; Serial.println(">> connected"); }
  void onDisconnect(BLEClient*) override { gConnected = false; Serial.println(">> DISCONNECTED"); }
};

class SecCB : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() override { Serial.println("SEC: passkey requested"); return 0; }
  void onPassKeyNotify(uint32_t k) override { Serial.printf("SEC: passkey %06u\n", k); }
  bool onSecurityRequest() override { Serial.println("SEC: *** CAMERA REQUESTED SECURITY *** -> accept"); return true; }
  bool onConfirmPIN(uint32_t p) override { Serial.printf("SEC: confirm %06u -> yes\n", p); return true; }
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onAuthenticationComplete(esp_ble_auth_cmpl_t c) override {
    Serial.printf("SEC: auth complete success=%d\n", c.success);
    gBondOk = c.success; gBondDone = true;
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onAuthenticationComplete(ble_gap_conn_desc* d) override {
    bool ok = d && d->sec_state.encrypted;
    Serial.printf("SEC: auth complete encrypted=%d bonded=%d\n", ok,
                  (d && d->sec_state.bonded) ? 1 : 0);
    gBondOk = ok; gBondDone = true;
  }
#endif
};

static void dumpGatt() {
  Serial.println("\n================ GATT DATABASE ================");
  auto services = gClient->getServices();
  if (!services || services->empty()) {
    Serial.println("(no services returned)");
    return;
  }
  for (auto& sp : *services) {
    BLERemoteService* svc = sp.second;
    Serial.printf("\nSERVICE %s\n", svc->getUUID().toString().c_str());
    auto chars = svc->getCharacteristics();
    if (!chars) continue;
    for (auto& cp : *chars) {
      BLERemoteCharacteristic* c = cp.second;
      Serial.printf("   CHAR %s  [%s%s%s%s%s]\n",
                    c->getUUID().toString().c_str(),
                    c->canRead()            ? "R"  : "",
                    c->canWrite()           ? "W"  : "",
                    c->canWriteNoResponse() ? "w"  : "",
                    c->canNotify()          ? "N"  : "",
                    c->canIndicate()        ? "I"  : "");
    }
  }
  Serial.println("\n==============================================\n");
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) delay(10);
  delay(300);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(20);
#endif

  Serial.println("\n=== Sony_GattExplore " __DATE__ " " __TIME__ " ===");

  esp_err_t nv = nvs_flash_init();
  if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase(); nv = nvs_flash_init();
  }

  BLEDevice::init("OpenClick");
  BLESecurity* sec = new BLESecurity();
  sec->setAuthenticationMode(true, false, false);      // bond, no MITM, legacy
  sec->setCapability(ESP_IO_CAP_NONE);
  sec->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  sec->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  BLEDevice::setSecurityCallbacks(new SecCB());

#if WIPE_BONDS && defined(CONFIG_NIMBLE_ENABLED)
  Serial.printf("ble_store_clear() -> %d\n", ble_store_clear());
#endif

  gClient = BLEDevice::createClient();
  gClient->setClientCallbacks(new ClientCB());

  Serial.printf("connecting to %s ...\n", CAMERA_MAC);
  BLEAddress addr = BLEAddress(String(CAMERA_MAC));
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  bool ok = gClient->connect(addr, 0, 10000);
#else
  bool ok = gClient->connect(addr);
#endif
  if (!ok) { Serial.println("connect FAILED"); return; }
  delay(700);

  dumpGatt();

#if TRY_PAIRING_WRITE
  BLERemoteService* ps = gClient->getService(SVC_PAIRING);
  if (ps == nullptr) {
    Serial.println("!! pairing service 8000EE00 NOT present on this camera");
    Serial.println("   (so the pairing-command theory is dead - see the dump above)");
  } else {
    Serial.println("** pairing service 8000EE00 IS present **");
    BLERemoteCharacteristic* pc = ps->getCharacteristic(BLEUUID((uint16_t)0xEE01));
    if (pc == nullptr) {
      Serial.println("!! characteristic EE01 not found in it");
    } else {
      Serial.print("writing pairing command: ");
      hexdump(PAIR_CMD, sizeof(PAIR_CMD));
      bool w = pc->writeValue(PAIR_CMD, sizeof(PAIR_CMD), true);
      Serial.printf(" -> %s\n", w ? "ACK" : "REJECTED");
      Serial.println("watching 20 s for the camera to start pairing...");
      uint32_t t = millis();
      while (!gBondDone && gConnected && (millis() - t) < 20000) delay(50);
      if (gBondDone) Serial.printf("RESULT: bonding %s\n", gBondOk ? "SUCCEEDED" : "failed");
      else if (!gConnected) Serial.println("RESULT: camera disconnected us");
      else Serial.println("RESULT: no pairing response");
    }
  }
#endif

  Serial.println("\n-- done. press RESET to run again --");
}

void loop() {
  static uint32_t b = 0;
  if (millis() - b > 5000) {
    b = millis();
    Serial.printf("[%lu] connected=%d bondDone=%d bondOk=%d\n",
                  (unsigned long)millis(), gConnected, gBondDone, gBondOk);
  }
  delay(20);
}
