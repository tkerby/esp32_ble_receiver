#include <Arduino.h>
#include <BLEDevice.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <FastLED.h>

// provides the PRIx64 macro
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "config.h"

#define DATA_PIN 27
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

WiFiClient client;
PubSubClient mqtt(client);
BLEScan* scanner;
String station;
char clientid[20];

const char mqtt_host[] = MQTT_HOST;
int mqtt_port = MQTT_PORT;
const char mqtt_topic[] = MQTT_TOPIC;

bool scanning = false;
volatile int publish_ok = 0;
volatile int publish_fail = 0;
time_t last_ok = 0;

String hexlify(const uint8_t bytes[], int len)
{
  String output;
  output.reserve(len*2);
  for (int i=0; i<len; i++) {
    char hex[3];
    sprintf(hex, "%02x", bytes[i]);
    output.concat(hex);
  }
  return output;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getPayloadLength() == 0) {
      return;
    }
    // Turn the neopixel on the ATOM Lite blue - this will give a quick flash for the received advertisement and turns off after the message is sent
    leds[0] = CRGB::Blue;
    FastLED.show();

    // Start to build a JSON document object. This expects const char inputs so type conversion may be required. Later stages dont like binary so hexlify as required
    StaticJsonDocument<1024> doc;

    // Capture the time and ESP station from local data
    doc["millis"] = millis();
    doc["station"] = station;

    // Capture the BLE device address (should be there on all transmisisons)
    doc["address"] = advertisedDevice.getAddress().toString();
    // Find the address type - public addresses should stay fixed and uniquely identify the device
    switch (advertisedDevice.getAddressType())
    {
    case esp_ble_addr_type_t::BLE_ADDR_TYPE_PUBLIC:
      doc["addrtype"]="public";
      break;
    case esp_ble_addr_type_t::BLE_ADDR_TYPE_RANDOM:
      doc["addrtype"]="random";
      break;
    case esp_ble_addr_type_t::BLE_ADDR_TYPE_RPA_PUBLIC:
      doc["addrtype"]="rpa_public";
      break;
    case esp_ble_addr_type_t::BLE_ADDR_TYPE_RPA_RANDOM:
      doc["addrtype"]="rpa_random";
      break;    
    default:
      break;
    } 

    // Convert the binary payload into hex characters and capture both it and the length. Should be on all transmissions
    doc["payload"] = hexlify(advertisedDevice.getPayload(), advertisedDevice.getPayloadLength());
    doc["payloadlen"] = advertisedDevice.getPayloadLength();

    // The next fields are all optional as decoded

    // Get the RSSI (returns an int)
    if (advertisedDevice.haveRSSI()) {
      doc["rssi"] = advertisedDevice.getRSSI();
    }

    // Get the device name (returns a string) and should be human readable (e.g. for pairing)
    if (advertisedDevice.haveName()) {
      doc["name"] = advertisedDevice.getName();
    }

    // Get the manufacturer data. This returns as a strig but may be binary. Would recommend turning to hex and considering per device in later code
    if (advertisedDevice.haveManufacturerData()) {
      doc["mfg"] = hexlify((const uint8_t *)advertisedDevice.getManufacturerData().c_str(),advertisedDevice.getManufacturerData().length());
    }

    // Get the appearance. This is a 16 bit int representing the device icon defined in the bluetooth assigned numbers. 
    // See http://developer.nordicsemi.com/nRF51_SDK/nRF51_SDK_v5.x.x/doc/5.2.0/html/a01225.html for examples
    if (advertisedDevice.haveAppearance()) {
      doc["appearance"] = advertisedDevice.getAppearance();
    }

    // Get the transmission power of the BLE device. Needed for range finding with RSSI data. 8 bit int.
    if (advertisedDevice.haveTXPower()) {
      doc["txpow"] = advertisedDevice.getTXPower();
    }

    /*
    // Get the service UUID. This is deprecated in the library and replaced with the ServiceData and ServiceDataUUID calls
    if (advertisedDevice.haveServiceUUID()) {
      doc["serviceuuid"] = advertisedDevice.getServiceUUID().toString();
    }
    */
   
    // Get any custom service data and UUIDs associated with the data
    if (advertisedDevice.haveServiceData()) {
      if (advertisedDevice.getServiceData().size() > 0){
        doc["servicedata"] = hexlify((const uint8_t *)advertisedDevice.getServiceData().c_str(), advertisedDevice.getServiceData().size());
      }
      doc["servicedatauuid"] = advertisedDevice.getServiceDataUUID().toString();
    }
    
    String json;
    serializeJson(doc, json);
    /*
    if (advertisedDevice.haveServiceData()) {
      Serial.print(json);
    }
    */
    doc.clear(); //Clear the document ready for next use and release memory

    // Publish to mqtt server
    if (mqtt.connected()) {
      if (mqtt.publish(mqtt_topic, json.c_str())) {
        publish_ok++;
        last_ok = millis();
      } else { // We've failed - count a failure and light the LED red
        publish_fail++;
        leds[0] = CRGB::Red;
        FastLED.show();
        delay(20);
      }
      
      leds[0] = CRGB::Black;
      FastLED.show();
    }
  }
};

void scan_complete(BLEScanResults results) {
  Serial.print("ok=");
  Serial.print(publish_ok, DEC);
  Serial.print(" fail=");
  Serial.println(publish_fail, DEC);

  scanner->clearResults();

  scanning = false;
  publish_ok = 0;
  publish_fail = 0;
}

void setup() {
  snprintf(clientid, sizeof(clientid), "esp%" PRIx64, ESP.getEfuseMac());

  Serial.begin(115200);
  Serial.println();
  Serial.print(ESP.getSketchMD5());
  Serial.print(" ");
  Serial.println(clientid);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  leds[0] = CRGB::Red;
  FastLED.show();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.waitForConnectResult();
  leds[0] = CRGB::Yellow;
  FastLED.show();

  mqtt.setServer(mqtt_host, mqtt_port);
  mqtt.connect(clientid, MQTT_USER, MQTT_PASSWORD);

  BLEDevice::init("");
  station = BLEDevice::getAddress().toString().c_str();
  Serial.print("Station: ");
  Serial.println(station);

  scanner = BLEDevice::getScan();
  scanner->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  scanner->setActiveScan(true); //true gets results quicker but uses more power
  scanner->setInterval(100);
  scanner->setWindow(99);
  leds[0] = CRGB::Green;
  FastLED.show();
  delay(300);
  leds[0] = CRGB::Black;
  FastLED.show();

}

void loop() {
  static time_t last_reconnect_check = 0;

  if (millis() - last_ok > WATCHDOG_TIMEOUT) {
    Serial.println("application watchdog triggered");
    ESP.restart();
  }

  if (millis() - last_reconnect_check > 1000) {
    if (!mqtt.connected()) {
      Serial.println("reconnecting mqtt");
      mqtt.connect(clientid, MQTT_USER, MQTT_PASSWORD);
    }
    last_reconnect_check = millis();
  }

  mqtt.loop();

  if (!scanning && mqtt.connected()) {
    scanner->start(31, scan_complete, false);
    scanning = true;
  }
}
