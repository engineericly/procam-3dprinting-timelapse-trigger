/*
 * WiFi_Scan - is the camera's access point even visible to this board?
 *
 * The join failed with no further information, which has two very different
 * causes that look identical from the outside:
 *
 *   a) wrong password  -> the SSID IS in the scan, we just can't authenticate
 *   b) wrong BAND      -> the SSID is NOT in the scan at all
 *
 * (b) matters because the ESP32-C3 has a 2.4 GHz radio ONLY. The FX30 supports
 * 5 GHz, and if it brought up Wi-Fi Direct on 5 GHz then no amount of correct
 * password will help - the board cannot see that band, at all, ever.
 *
 * This sketch just scans and reports. Leave the camera on its Wi-Fi Direct
 * Info screen while it runs.
 */

#include <Arduino.h>
#include <WiFi.h>

// From the camera's QR code:
//   W01:S:DIRECT-kPU1:ILME-FX30;P:1Z4RBaE7;C:ILME-FX30;M:9C50D1AEAC27;
// The M field is the camera's Wi-Fi MAC (its BLE address is ...AC:28).
// Matching on BSSID catches the AP even if it hides its SSID.
#define CAM_BSSID "9C:50:D1:AE:AC:27"

static const char* encName(int t) {
  switch (t) {
    case WIFI_AUTH_OPEN:            return "open";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
    default:                        return "?";
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) delay(10);
  delay(300);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(20);
#endif
  Serial.println("\n=== WiFi_Scan " __DATE__ " " __TIME__ " ===");
  Serial.println("This board's radio: 2.4 GHz only (ESP32-C3).");
  Serial.println("Anything on 5 GHz is invisible here by design.\n");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
}

void loop() {
  Serial.println("scanning...");
  int n = WiFi.scanNetworks();
  if (n <= 0) {
    Serial.println("  no networks found at all (unexpected - check the antenna)\n");
  } else {
    Serial.printf("  %d networks visible on 2.4 GHz:\n", n);
    bool foundDirect = false;
    for (int i = 0; i < n; i++) {
      String ss = WiFi.SSID(i);
      String bs = WiFi.BSSIDstr(i);
      bool isDirect = ss.startsWith("DIRECT-") || ss.indexOf("ILME") >= 0
                      || bs.equalsIgnoreCase(CAM_BSSID);
      if (isDirect) foundDirect = true;
      Serial.printf("   %2d) ch%-3d %4d dBm  %-10s %-24s %s%s\n",
                    i + 1, WiFi.channel(i), WiFi.RSSI(i),
                    encName(WiFi.encryptionType(i)), ss.c_str(), bs.c_str(),
                    isDirect ? "   <<<< THE CAMERA" : "");
    }
    Serial.println();
    if (foundDirect) {
      Serial.println(">>> Camera AP IS visible on 2.4 GHz.");
      Serial.println("    Band is fine, and the QR code confirms the credentials are");
      Serial.println("    correct, so the join failure is something else: try moving");
      Serial.println("    closer, or power-cycling the camera's Wi-Fi.");
    } else {
      Serial.println(">>> Camera AP is NOT visible on 2.4 GHz.");
      Serial.println("    The QR code proves SSID and password are correct, so this");
      Serial.println("    is the band, not the credentials.");
      Serial.println("    Either it is on 5 GHz (this board can never see it), or the");
      Serial.println("    camera is not currently advertising. Look for a band setting:");
      Serial.println("    MENU > Network > Wi-Fi > Wi-Fi Frequency Band -> set 2.4 GHz,");
      Serial.println("    then re-open Wi-Fi Direct Info and rerun.");
    }
  }
  WiFi.scanDelete();
  Serial.println("\n-- rescanning in 10 s --\n");
  delay(10000);
}
