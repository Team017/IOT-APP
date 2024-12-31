#include <WiFi.h>
#include <WiFiClientSecure.h> // For secure connections
#include <PubSubClient.h>
#include <math.h>
#include "arduino_secrets.h.ino" // Include the secrets.h file for sensitive information

WiFiClientSecure secureClient;
PubSubClient client(secureClient);

// Sensor pins
const int pHSensorPin = 32;
const int turbiditySensorPin = 35;

// Wi-Fi connection setup
void setupWiFi() {
    Serial.print("Connecting to Wi-Fi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi connected");
}

// Connect to MQTT with TLS/SSL
void connectToMQTT() {
    secureClient.setCACert(rootCA);

    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect(mqttClientID, mqttUsername, mqttPassword)) {
            Serial.println("connected");
        } else {
            Serial.print("failed with state ");
            Serial.println(client.state());
            delay(5000);
        }
    }
}

// Read turbidity sensor with input validation
float readTurbidity(int sensorPin) {
    analogReadResolution(12); 
    analogSetAttenuation(ADC_11db);
    
    float totalVoltage = 0;
    const int sampleCount = 800;

    for (int i = 0; i < sampleCount; i++) {
        totalVoltage += ((float)analogRead(sensorPin) / 4095.0) * 3.3;
    }
    float averageVoltage = totalVoltage / sampleCount;

    float ntu = 3000.0 * (1.0 - (averageVoltage / 2.8));
    ntu = fmax(0.0, fmin(3000.0, ntu));

    return isnan(ntu) ? 0.0 : ntu; // Validate output to avoid NaN
}

// Read pH sensor with input validation
float readPH(int sensorPin) {
    float totalVoltage = 0;
    const int sampleCount = 800; // Number of samples to average

    // Read and average the sensor voltage
    for (int i = 0; i < sampleCount; i++) {
        totalVoltage += ((float)analogRead(sensorPin) / 4095.0) * 3.3; // ADC to voltage (3.3V reference)
    }
    float averageVoltage = totalVoltage / sampleCount;

    // Calibration constants based on your measured data
    const float voltageAtPH4 = 2.62;
    const float voltageAtPH6_86 = 2.54;
    const float voltageAtPH9_18 = 2.46;

    const float slope1 = (6.86 - 4.0) / (voltageAtPH6_86 - voltageAtPH4); // Between pH 4 and pH 6.86
    const float slope2 = (9.18 - 6.86) / (voltageAtPH9_18 - voltageAtPH6_86); // Between pH 6.86 and pH 9.18

    float pHValue = 7.0; // Default to neutral
    if (averageVoltage >= voltageAtPH6_86) {
        // Calculate pH in the acidic range (6.86 to 4.0)
        pHValue = 6.86 + slope1 * (averageVoltage - voltageAtPH6_86);
    } else if (averageVoltage < voltageAtPH6_86) {
        // Calculate pH in the basic range (6.86 to 9.18)
        pHValue = 6.86 + slope2 * (averageVoltage - voltageAtPH6_86);
    }

    return isnan(pHValue) ? 7.0 : pHValue; // Validate output to avoid NaN
}

void setup() {
    Serial.begin(115200);
    setupWiFi();
    client.setServer(mqttServer, mqttPort);
    connectToMQTT();

    pinMode(turbiditySensorPin, INPUT);
    pinMode(pHSensorPin, INPUT);
}

void loop() {
    // Ensure the MQTT client is connected
    if (!client.connected()) {
        Serial.println("MQTT connection lost, reconnecting...");
        connectToMQTT();  // Reconnect if necessary
    }

    client.loop();  // Handle any network activity, including keep-alive

    // Read sensors with validation
    float pHValue = readPH(pHSensorPin);
    float turbidityValue = readTurbidity(turbiditySensorPin);

    // Prepare and send secure MQTT payload
    String payload = "field1=" + String(pHValue, 2) + "&field2=" + String(turbidityValue, 2) + "&status=MQTTPUBLISH";

    int retryCount = 0;
    bool published = false;

    // Retry publishing the data if it fails, with exponential backoff
    while (!published && retryCount < 5) {
        if (client.publish(mqttTopic, payload.c_str())) {
            Serial.println("Data published securely to ThingSpeak");
            published = true;
        } else {
            retryCount++;
            Serial.print("Failed to publish, retrying... (Attempt ");
            Serial.print(retryCount);
            Serial.println(")");
            delay(1000 * retryCount); // Exponential backoff delay
        }
    }

    // If the data is not published after retries, report failure
    if (!published) {
        Serial.println("Failed to publish after retries.");
    }

    // Set MQTT Keep-Alive interval to 60 seconds (you can adjust this as needed)
    client.setKeepAlive(60);  // Ensure client sends a ping to keep the connection alive

    delay(5000);  // Wait for 20 seconds before the next loop
}

