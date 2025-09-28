#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include "DHT.h"

// Pin definitions
#define DHTPIN 15
#define DHTTYPE DHT11
#define LED_PIN 2  // Built-in LED on most ESP32 boards

// AWS IoT topics
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

// Connection retry settings
#define WIFI_TIMEOUT 10000      // 10 seconds
#define MQTT_TIMEOUT 5000       // 5 seconds
#define MAX_RECONNECT_ATTEMPTS 3

// LED blink patterns (milliseconds)
#define BLINK_WIFI_CONNECTING 200    // Fast blink - WiFi connecting
#define BLINK_MQTT_CONNECTING 500    // Medium blink - MQTT connecting
#define BLINK_ERROR 100              // Very fast blink - Error
#define BLINK_SUCCESS 2000           // Slow blink - All good

// Global variables
float h, t;
unsigned long lastConnectionCheck = 0;
unsigned long lastPublish = 0;
unsigned long lastLedUpdate = 0;
bool ledState = false;
int reconnectAttempts = 0;

enum ConnectionStatus {
  STATUS_WIFI_CONNECTING,
  STATUS_MQTT_CONNECTING,
  STATUS_CONNECTED,
  STATUS_ERROR
};
ConnectionStatus currentStatus = STATUS_WIFI_CONNECTING;

DHT dht(DHTPIN, DHTTYPE);
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// Function declarations
void connectWiFi();
void connectMQTT();
void publishMessage();
void messageHandler(char* topic, byte* payload, unsigned int length);
void updateLED();
void checkConnections();

void setup() {
  Serial.begin(115200);
  
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Disable WiFi power saving
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  
  // Initialize DHT sensor
  dht.begin();
  
  Serial.println("Starting ESP32 AWS IoT Client...");
  
  // Initial connection
  connectWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    connectMQTT();
  }
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check connections every 5 seconds
  if (currentTime - lastConnectionCheck >= 5000) {
    checkConnections();
    lastConnectionCheck = currentTime;
  }
  
  // Publish data every 10 seconds (only if connected)
  if (currentStatus == STATUS_CONNECTED && 
      currentTime - lastPublish >= 10000) {
    
    // Read sensor data
    h = dht.readHumidity();
    t = dht.readTemperature();
    
    if (!isnan(h) && !isnan(t)) {
      Serial.printf("Humidity: %.1f%%  Temperature: %.1f°C\n", h, t);
      publishMessage();
    } else {
      Serial.println("Failed to read from DHT sensor!");
    }
    
    lastPublish = currentTime;
  }
  
  // Update LED status
  updateLED();
  
  // Keep MQTT alive
  if (client.connected()) {
    client.loop();
  }
  
  delay(100); // Small delay to prevent watchdog issues
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  
  currentStatus = STATUS_WIFI_CONNECTING;
  Serial.println("Connecting to WiFi...");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && 
         millis() - startTime < WIFI_TIMEOUT) {
    updateLED();
    delay(100);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Signal strength: %d dBm\n", WiFi.RSSI());
    reconnectAttempts = 0;
  } else {
    Serial.println("WiFi connection failed!");
    currentStatus = STATUS_ERROR;
    reconnectAttempts++;
  }
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  
  currentStatus = STATUS_MQTT_CONNECTING;
  Serial.println("Connecting to AWS IoT...");
  
  // Configure certificates
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
  
  // Set MQTT server
  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setCallback(messageHandler);
  
  // Set longer keepalive and socket timeout
  client.setKeepAlive(60);
  client.setSocketTimeout(30);
  
  unsigned long startTime = millis();
  while (!client.connected() && 
         millis() - startTime < MQTT_TIMEOUT) {
    if (client.connect(THINGNAME)) {
      break;
    }
    updateLED();
    delay(100);
  }
  
  if (client.connected()) {
    Serial.println("AWS IoT connected!");
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    currentStatus = STATUS_CONNECTED;
    reconnectAttempts = 0;
  } else {
    Serial.printf("AWS IoT connection failed! State: %d\n", client.state());
    currentStatus = STATUS_ERROR;
    reconnectAttempts++;
  }
}

void checkConnections() {
  // Check WiFi first
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Attempting reconnection...");
    client.disconnect();
    
    if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
      connectWiFi();
    } else {
      Serial.println("Max WiFi reconnect attempts reached. Will retry in next cycle.");
      currentStatus = STATUS_ERROR;
      reconnectAttempts = 0; // Reset for next cycle
    }
    return;
  }
  
  // Check MQTT connection
  if (!client.connected()) {
    Serial.println("MQTT disconnected! Attempting reconnection...");
    
    if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
      connectMQTT();
    } else {
      Serial.println("Max MQTT reconnect attempts reached. Will retry in next cycle.");
      currentStatus = STATUS_ERROR;
      reconnectAttempts = 0; // Reset for next cycle
    }
    return;
  }
  
  // Both connections are good
  if (currentStatus != STATUS_CONNECTED) {
    currentStatus = STATUS_CONNECTED;
    Serial.println("All connections stable.");
  }
}

void publishMessage() {
  if (!client.connected()) {
    Serial.println("Cannot publish - MQTT not connected");
    return;
  }
  
  StaticJsonDocument<200> doc;
  doc["humidity"] = h;
  doc["temperature"] = t;
  doc["timestamp"] = millis();
  doc["wifi_rssi"] = WiFi.RSSI();
  
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  
  if (client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer)) {
    Serial.println("Message published successfully");
  } else {
    Serial.println("Failed to publish message");
    currentStatus = STATUS_ERROR;
  }
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.printf("Message received on topic: %s\n", topic);
  
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) {
    Serial.printf("JSON parsing failed: %s\n", error.c_str());
    return;
  }
  
  const char* message = doc["message"];
  if (message) {
    Serial.printf("Message: %s\n", message);
  }
}

void updateLED() {
  unsigned long currentTime = millis();
  unsigned long blinkInterval;
  
  // Determine blink pattern based on status
  switch (currentStatus) {
    case STATUS_WIFI_CONNECTING:
      blinkInterval = BLINK_WIFI_CONNECTING;
      break;
    case STATUS_MQTT_CONNECTING:
      blinkInterval = BLINK_MQTT_CONNECTING;
      break;
    case STATUS_CONNECTED:
      blinkInterval = BLINK_SUCCESS;
      break;
    case STATUS_ERROR:
      blinkInterval = BLINK_ERROR;
      break;
    default:
      blinkInterval = BLINK_ERROR;
  }
  
  // Update LED state
  if (currentTime - lastLedUpdate >= blinkInterval) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    lastLedUpdate = currentTime;
  }
}