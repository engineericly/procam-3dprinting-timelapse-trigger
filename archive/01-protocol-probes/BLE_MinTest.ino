/*
 * BLE_MinTest - isolate whether BLEDevice::init() is the problem.
 *
 * No display, no button, no bonding, no camera logic. Just: is the BLE stack
 * able to start on this board, with this core, with these IDE settings?
 *
 * Board settings that matter:
 *   Tools > USB CDC On Boot      : Enabled
 *   Tools > Partition Scheme     : anything with a big app partition
 *                                  ("Huge APP" is the safe choice)
 *
 * Expected healthy output: the four init lines, then a stream of "adv"
 * lines and a heartbeat every 2 s.
 *
 * If it stops at "calling BLEDevice::init()", the BLE stack itself will not
 * start on this setup and no amount of camera code will help.
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <nvs_flash.h>

static uint32_t gSeen = 0;

class Cb : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) override {
    gSeen++;
    auto n = d.getName();
    auto a = d.getAddress().toString();
    Serial.printf("  adv %s  rssi=%4d  %s\n", a.c_str(), d.getRSSI(),
                  n.length() ? n.c_str() : "(no name)");
  }
};

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) delay(10);
  delay(300);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(20);
#endif

  Serial.println();
  Serial.println("=== BLE_MinTest " __DATE__ " " __TIME__ " ===");
#if defined(CONFIG_BLUEDROID_ENABLED)
  Serial.println("stack: Bluedroid");
#elif defined(CONFIG_NIMBLE_ENABLED)
  Serial.println("stack: NimBLE");
#else
  Serial.println("stack: NONE - Bluetooth is not enabled for this board!");
#endif
  Serial.printf("core : %d.%d.%d\n", ESP_ARDUINO_VERSION_MAJOR,
                ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  Serial.printf("heap : %u bytes free\n", (unsigned)ESP.getFreeHeap());

  esp_err_t nv = nvs_flash_init();
  if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    Serial.println("nvs  : full/stale, erasing");
    nvs_flash_erase();
    nv = nvs_flash_init();
  }
  Serial.printf("nvs  : init -> %d\n", (int)nv);

  Serial.println("calling BLEDevice::init()   <-- if output stops here, this is the fault");
  delay(50);
  BLEDevice::init("MinTest");
  Serial.printf("BLEDevice::init() RETURNED. heap now %u\n", (unsigned)ESP.getFreeHeap());

  BLEScan* sc = BLEDevice::getScan();
  sc->setAdvertisedDeviceCallbacks(new Cb(), true, true);
  sc->setActiveScan(true);
  Serial.println("scanning continuously...");
}

void loop() {
  static uint32_t lastBeat = 0;
  static uint32_t lastScan = 0;

  if (millis() - lastScan > 6000) {
    lastScan = millis();
    BLEDevice::getScan()->clearResults();
    BLEDevice::getScan()->start(5, [](BLEScanResults r) {}, false);
  }

  if (millis() - lastBeat > 2000) {
    lastBeat = millis();
    Serial.printf("[%lu] alive, %lu advertisements seen, heap=%u\n",
                  (unsigned long)millis(), (unsigned long)gSeen,
                  (unsigned)ESP.getFreeHeap());
  }
  delay(10);
}
