#include <Arduino.h>
#include "DHT.h"
#include <DHT_U.h>
#include "SoundSwitcher.h"
#include "PubSubClient.h"
#include "WiFi.h"

#if !(USING_DEFAULT_ARDUINO_LOOP_STACK_SIZE)
uint16_t USER_CONFIG_ARDUINO_LOOP_STACK_SIZE = 16384;
#endif

#define RED_LED 5

#define DHT_TYPE DHT22
DHT_Unified dht(27, DHT_TYPE);
uint64_t loopCounter = 0;
uint32_t tempLoopCtr = 0;

 const char* ssid = "WLAN SSID";
const char* wifi_password = "password";

// MQTT
const char* mqtt_server = "192.4.4.4";
const char* humidity_topic = "humidity_topic";
const char* temperature_topic = "temperature_topic";
const char* loudness_topic = "loudness_topic";
const char* mqtt_username = "MQTT username";
const char* mqtt_password = "MQTT password";
const char* clientID = "kellerkind-esp32";

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
// 1883 is the listener port for the Broker
PubSubClient client(mqtt_server, 1883, wifiClient); 

// Possible configuration for Adafruit Huzzah Esp32
static const i2s_pin_config_t pin_config_Adafruit_Huzzah_Esp32 = {
    .bck_io_num = 14,                  // BCKL
    .ws_io_num = 15,                   // LRCL
    .data_out_num = I2S_PIN_NO_CHANGE, // not used
    .data_in_num = 32                  // DOUT
};

MicType usedMicType = MicType::INMP441;
SoundSwitcher soundSwitcher(pin_config_Adafruit_Huzzah_Esp32, usedMicType);
float soundValue = 0;

#define SOUNDSWITCHER_UPDATEINTERVAL 400  // in ms, update averaging buffer ever x ms
#define SOUNDSWITCHER_READ_DELAYTIME 4000 // read soundlevel x ms after toggling to display as analog value
#define SOUNDSWITCHER_THRESHOLD "220"     // arbitrary sound switching level

int soundSwitcherUpdateInterval = SOUNDSWITCHER_UPDATEINTERVAL;
uint32_t soundSwitcherReadDelayTime = SOUNDSWITCHER_READ_DELAYTIME;
char sSwiThresholdStr[6] = SOUNDSWITCHER_THRESHOLD;

FeedResponse feedResult;

void setup()
{
  // Run once
  Serial.begin(115200);
  
  pinMode(RED_LED, OUTPUT);

  Serial.println("Starting");

  soundSwitcher.begin(atoi((char *)sSwiThresholdStr), Hysteresis::Percent_20, soundSwitcherUpdateInterval, soundSwitcherReadDelayTime);
  soundSwitcher.SetCalibrationParams(-5.0);
  soundSwitcher.SetActive();

  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected; IP address: ");
  Serial.println(WiFi.localIP());

  dht.begin();
}

void connect_MQTT(){
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
} 

void loop()
{
  // Send data every 100000 th round
  if (++loopCounter % 100000 == 0) 
  {
    delay(200);

    feedResult = soundSwitcher.feed();
    if (feedResult.isValid) {
      if (feedResult.hasToggled) {
        Serial.println("Toggling");
        Serial.println(feedResult.avValue);
      }
    
      Serial.print("Volume: ");
      soundValue = feedResult.avValue;
      Serial.println(soundValue);

      // Failure sound levels; Blink and try to restart device.
      if (soundValue > 100000) {
        for (int i = 0; i < 20; i++) {
           digitalWrite(RED_LED, HIGH);
           delay(500);
           digitalWrite(RED_LED, LOW);
           delay(500);
        }

        ESP.restart();
      }
    }

    // Send every 5 seconds
    if (++tempLoopCtr % 20 == 0)
    {
      tempLoopCtr = 0;
      digitalWrite(RED_LED, HIGH);
      connect_MQTT();
 
      // MQTT can only transmit strings
      String humidity = "Hum: ";
      String temperature = "Temp: ";
      sensors_event_t event;
      dht.temperature().getEvent(&event);
      if (isnan(event.temperature)) {
        Serial.println(F("Error reading temperature!"));
      }
      else {
       temperature = String((float)event.temperature);
        Serial.print(F("Temperature: "));
        Serial.print(event.temperature);
        Serial.println(F("°C"));
      }

      // Get humidity event and print its value.
      dht.humidity().getEvent(&event);
      if (isnan(event.relative_humidity)) {
        Serial.println(F("Error reading humidity!"));
      }
      else {
        humidity = String((float)event.relative_humidity);
        Serial.print(F("Humidity: "));
        Serial.print(event.relative_humidity);
        Serial.println(F("%"));
      }

      // PUBLISH to the MQTT Broker (topic = Temperature, defined at the beginning)
      if (client.publish(temperature_topic, String(temperature).c_str())) {
        Serial.println("Temperature sent!");
      }
      // Again, client.publish will return a boolean value depending on whether it succeded or not.
      // If the message failed to send, we will try again, as the connection may have broken.
      else {
        Serial.println("Temperature failed to send. Reconnecting to MQTT Broker and trying again");
        client.connect(clientID, mqtt_username, mqtt_password);
        delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
        client.publish(temperature_topic, String(temperature).c_str());
      }

      // PUBLISH to the MQTT Broker (topic = Humidity, defined at the beginning)
      if (client.publish(humidity_topic, String(humidity).c_str())) {
        Serial.println("Humidity sent!");
      }
      // Again, client.publish will return a boolean value depending on whether it succeded or not.
      // If the message failed to send, we will try again, as the connection may have broken.
      else {
        Serial.println("Humidity failed to send. Reconnecting to MQTT Broker and trying again");
        client.connect(clientID, mqtt_username, mqtt_password);
        delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
        client.publish(humidity_topic, String(humidity).c_str());
      }

      // PUBLISH to the MQTT Broker (topic = Sound sensor, defined at the beginning)
      if (client.publish(loudness_topic, String(soundValue).c_str())) {
        Serial.println("Sound sent!");
      }
      // Again, client.publish will return a boolean value depending on whether it succeded or not.
      // If the message failed to send, we will try again, as the connection may have broken.
      else {
        Serial.println("Sound failed to send. Reconnecting to MQTT Broker and trying again");
        client.connect(clientID, mqtt_username, mqtt_password);
        delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
        client.publish(loudness_topic, String(soundValue).c_str());
      }
 
      client.disconnect();  // disconnect from the MQTT broker
      digitalWrite(RED_LED, LOW);
    }
  }
}