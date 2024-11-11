#include <WiFiNINA.h>
#include <Firebase_ESP_Client.h>
#include <PubSubClient.h>

// Wi-Fi credentials
#define WIFI_SSID1 "Vanshika1"             // Primary Wi-Fi SSID
#define WIFI_PASSWORD1 "123456788"         // Primary Wi-Fi password
#define WIFI_SSID2 "BackupNetwork"         // Backup Wi-Fi SSID
#define WIFI_PASSWORD2 "BackupPassword"    // Backup Wi-Fi password

// Firebase credentials
#define FIREBASE_HOST "industrial-smoke-detection-default-rtdb.firebaseio.com"
#define FIREBASE_API_KEY "AIzaSyA2TK3dXFY9YLXf0VdFgHu5D2Hv5H8hix4"
#define FIREBASE_EMAIL "vanshikadhawan587@gmail.com"
#define FIREBASE_PASSWORD "VanshikaDhawan@6398"

// MQTT broker details
#define MQTT_BROKER "broker.emqx.io" 
#define MQTT_PORT 1883
#define MQTT_TOPIC "sensor/data"

// Sensor Pins
const int analogPinMQ2 = A0;      // Gas sensor pin
const int measurePinDust = A1;    // Dust sensor pin
const int ledPowerDust = 3;       // Power LED for dust sensor

// Detection thresholds
const int GAS_THRESHOLD = 400;    // Gas detection threshold
const int DUST_THRESHOLD = 250;   // Dust detection threshold

// Firebase configuration
FirebaseData firebaseData;
FirebaseConfig config;
FirebaseAuth auth;

// MQTT client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
    Serial.begin(9600);

    // Step 1: Connected to Wi-Fi with fault tolerance
    connectToWiFi();

    // Step 2: Configured Firebase
    config.host = FIREBASE_HOST;
    config.api_key = FIREBASE_API_KEY;
    auth.user.email = FIREBASE_EMAIL;
    auth.user.password = FIREBASE_PASSWORD;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    // Step 3: Configured the dust sensor power pin
    pinMode(ledPowerDust, OUTPUT);

    // Step 4: Set up MQTT server details and connected to MQTT
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    connectToMQTT();
}

void loop() {
    // Step 5: Checked Wi-Fi connection status and reconnected if disconnected
    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
    }

    // Step 6: Checked MQTT connection and reconnected if disconnected
    if (!mqttClient.connected()) {
        connectToMQTT();
    }
    mqttClient.loop();

    // Step 7: Read sensor data
    int gasLevel = analogRead(analogPinMQ2);
    float dustDensity = analogRead(measurePinDust) * 0.2;

    // Step 8: Printed sensor readings for debugging
    Serial.print("Gas Level: ");
    Serial.println(gasLevel);
    Serial.print("Dust Density: ");
    Serial.println(dustDensity);

    // Step 9: Created Firebase JSON data object and set values
    FirebaseJson json;
    json.set("timestamp", millis());        // Set timestamp
    json.set("gas_level", gasLevel);        // Set gas level
    json.set("dust_density", dustDensity);  // Set dust density

    // Step 10: Set alert status based on thresholds
    String alert = (gasLevel > GAS_THRESHOLD || dustDensity > DUST_THRESHOLD) 
                   ? "Early smoke detection alert!" : "No smoke detected";
    json.set("alert", alert);

    // Step 11: Created unique Firebase path with timestamp to save reading
    String path = "/SmokeDetectionData/Area1/" + String(millis());

    // Step 12: Sent data to Firebase and printed result
    if (Firebase.RTDB.setJSON(&firebaseData, path, &json)) {
        Serial.println("Data sent successfully to Firebase");
    } else {
        Serial.print("Failed to send data to Firebase: ");
        Serial.println(firebaseData.errorReason());
    }

    // Step 13: Sent data to MQTT broker
    String mqttMessage = "{";
    mqttMessage += "\"gas_level\":" + String(gasLevel) + ",";
    mqttMessage += "\"dust_density\":" + String(dustDensity) + ",";
    mqttMessage += "\"alert\":\"" + alert + "\"";
    mqttMessage += "}";
    mqttClient.publish(MQTT_TOPIC, mqttMessage.c_str());

    // Step 14: Added delay between readings
    delay(1000);  // Adjust delay as needed
}

void connectToWiFi() {
    int status;

    // Step A1: Attempted to connect to primary Wi-Fi
    Serial.println("Connecting to primary Wi-Fi...");
    status = WiFi.begin(WIFI_SSID1, WIFI_PASSWORD1);
    if (status != WL_CONNECTED) {
        // Step A2: If primary Wi-Fi failed, attempted backup network
        Serial.println("Primary Wi-Fi failed, connecting to backup...");
        status = WiFi.begin(WIFI_SSID2, WIFI_PASSWORD2);
        while (status != WL_CONNECTED) {
            delay(1000);
            Serial.println("Retrying Wi-Fi connection...");
            status = WiFi.begin(WIFI_SSID2, WIFI_PASSWORD2);
        }
    }
    Serial.println("Connected to Wi-Fi!");
}

void connectToMQTT() {
    // Step B1: Attempted to connect to the MQTT broker
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT broker...");
        if (mqttClient.connect("ArduinoClient")) {
            Serial.println("connected");
        } else {
            Serial.print("failed with state ");
            Serial.println(mqttClient.state());
            delay(2000);  // Retry delay
        }
    }
}
