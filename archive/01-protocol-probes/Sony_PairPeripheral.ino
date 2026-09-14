/*
 * Sony_PairPeripheral - test whether the FX30 will pair with us when WE
 * advertise and let the CAMERA connect.
 *
 * Why this exists
 * ---------------
 * Every attempt so far has had the ESP32 act as a BLE central: it connects to
 * the camera and asks to pair. The camera accepts the connection, exposes its
 * GATT service, then ignores the pairing request completely and hangs up.
 *
 * Sony's own procedure for the RMT-P1BT explains that: you open [Pairing] on
 * the camera, then put the REMOTE into pairing mode, and the camera discovers
 * it and shows a confirm dialog. The camera is the one scanning. A device that
 * never advertises can never appear on that screen, which is exactly the
 * behaviour we have been seeing.
 *
 * So this sketch inverts the roles. It advertises as a peripheral carrying the
 * Sony remote-control service UUID and waits for the camera to come to it.
 *
 * What to do
 * ----------
 *   1. Camera: Network > Bluetooth > Bluetooth Function     : On
 *   2. Camera: Network > Bluetooth > Bluetooth Rmt Ctrl     : On
 *   3. Flash this, open Serial Monitor at 115200
 *   4. Camera: Network > Bluetooth > Pairing
 *   5. Watch both the camera screen and the serial log
 *
 * SUCCESS looks like: a device appears on the camera's pairing screen, and/or
 * the log shows "CAMERA CONNECTED" followed by "*** BONDED ***".
 *
 * If nothing appears on the camera screen, Sony is filtering on something in
 * the advertisement we have not reproduced, and cloning a remote this way is
 * not going to work without capturing what a real RMT-P1BT broadcasts.
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLESecurity.h>
#include <nvs_flash.h>
#if defined(CONFIG_NIMBLE_ENABLED)
  #include <host/ble_store.h>
#endif

// Sony remote-control service, same UUIDs the camera exposes.
static BLEUUID SVC_REMOTE("8000FF00-FF00-FFFF-FFFF-FFFFFFFFFFFF");
static BLEUUID CHR_CMD((uint16_t)0xFF01);
static BLEUUID CHR_NTF((uint16_t)0xFF02);

// Try different identities if the camera ignores the first. Sony may filter on
// the advertised name, the service UUID, or the manufacturer payload.
#define ADV_NAME "OpenClick"

static volatile bool gConnected = false;
static volatile bool gBonded    = false;
static uint32_t gConnectedAt = 0;

static void hexdump(const uint8_t* d, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (d[i] < 0x10) Serial.print('0');
    Serial.print(d[i], HEX);
    Serial.print(' ');
  }
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    gConnected = true;
    gConnectedAt = millis();
    Serial.printf("\n*** CAMERA CONNECTED to us at %lu ms ***\n", (unsigned long)millis());
  }
  void onDisconnect(BLEServer* s) override {
    Serial.printf("*** disconnected after %lu ms (bonded=%d) ***\n",
                  (unsigned long)(millis() - gConnectedAt), gBonded ? 1 : 0);
    gConnected = false;
    BLEDevice::startAdvertising();     // go back to being discoverable
    Serial.println("re-advertising");
  }
};

class SecCB : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() override {
    Serial.println("SEC: passkey requested -> 0");
    return 0;
  }
  void onPassKeyNotify(uint32_t k) override {
    Serial.printf("SEC: show this passkey on the camera: %06u\n", k);
  }
  bool onSecurityRequest() override {
    Serial.println("SEC: camera requested security -> accepting");
    return true;
  }
  bool onConfirmPIN(uint32_t pin) override {
    Serial.printf("SEC: confirm PIN %06u -> yes\n", pin);
    return true;
  }
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onAuthenticationComplete(esp_ble_auth_cmpl_t c) override {
    Serial.printf("SEC: auth complete, success=%d\n", c.success);
    gBonded = c.success;
    if (c.success) Serial.println("*** BONDED ***");
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onAuthenticationComplete(ble_gap_conn_desc* d) override {
    bool ok = d && d->sec_state.encrypted;
    Serial.printf("SEC: auth complete, encrypted=%d bonded=%d\n",
                  ok ? 1 : 0, (d && d->sec_state.bonded) ? 1 : 0);
    gBonded = ok;
    if (ok) Serial.println("*** BONDED ***");
  }
#endif
};

class CmdCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();
    Serial.print("CAMERA WROTE to FF01: ");
    hexdump((const uint8_t*)v.c_str(), v.length());
    Serial.println();
  }
};

static void clearAllBonds() {
#if defined(CONFIG_NIMBLE_ENABLED)
  int rc = ble_store_clear();
  Serial.printf("ble_store_clear() -> %d\n", rc);
#endif
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) delay(10);
  delay(300);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(20);
#endif

  Serial.println();
  Serial.println("=== Sony_PairPeripheral " __DATE__ " " __TIME__ " ===");
  Serial.println("Advertising as a remote. Open Pairing on the camera now.");

  esp_err_t nv = nvs_flash_init();
  if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nv = nvs_flash_init();
  }

  BLEDevice::init(ADV_NAME);

  // Bondable, Just Works, no display and no keyboard on our side.
  BLESecurity* sec = new BLESecurity();
  sec->setAuthenticationMode(true, false, false);   // bond, no MITM, no SC
  sec->setCapability(ESP_IO_CAP_NONE);
  sec->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  sec->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  BLEDevice::setSecurityCallbacks(new SecCB());

  // Start clean so the camera always sees a brand new device.
  clearAllBonds();

  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  BLEService* svc = server->createService(SVC_REMOTE, 20);
  BLECharacteristic* cmd = svc->createCharacteristic(
      CHR_CMD, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
             | BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_NOTIFY);
  cmd->setCallbacks(new CmdCB());

  BLECharacteristic* ntf = svc->createCharacteristic(
      CHR_NTF, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  (void)ntf;
  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_REMOTE);
  adv->setScanResponse(true);
  adv->setAppearance(0x0180);        // generic remote control
  adv->setMinPreferred(0x06);
  BLEDevice::startAdvertising();

  Serial.printf("advertising as \"%s\", our address %s\n",
                ADV_NAME, BLEDevice::getAddress().toString().c_str());
  Serial.println("waiting for the camera...");
}

void loop() {
  static uint32_t beat = 0;
  if (millis() - beat > 3000) {
    beat = millis();
    Serial.printf("[%lu] advertising, connected=%d bonded=%d\n",
                  (unsigned long)millis(), gConnected ? 1 : 0, gBonded ? 1 : 0);
  }
  delay(20);
}
