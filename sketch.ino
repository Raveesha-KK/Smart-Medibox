

#include <PubSubClient.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include "DHTesp.h"

WiFiClient espClient;
PubSubClient mqttClient(espClient);
Servo servo;
DHTesp dhtSensor;

// Pin Configurations for Components
#define LIGHT_SENSOR_PIN 34
#define SERVO_PIN 27
#define DHT_PIN 15

const char* MQTT_SERVER = "broker.hivemq.com";
// Producer topics
const char* LIGHT_INTENSITY_TOPIC = "LIGHT_INTENSITY";
const char* SERVO_MOTOR_ANGLE_TOPIC = "Servo Motor Angle";
const char* AVG_LIGHT_INTENSITY_TOPIC = "Average Light Intensity";
// Subscribe topics 
const char* SERVO_MIN_ANGLE_TOPIC = "Minimum Angle";
const char* SERVO_CONTROL_FACTOR_TOPIC = "SERVO_CONTROLLING_FACTOR";
const char* SAMPLING_INTERVAL_TOPIC = "Sampling Interval";
const char* SENDING_INTERVAL_TOPIC = "Sending Interval";
const char* IDEAL_MED_TEMP_TOPIC = "Ideal Temperature";

// Servo motor angle parameters
int offset_angle = 30;
float intensity = 0;
float control_factor = 0.75;
float T_med = 30.0;  // Default value

// Array to store values which used to send data.
char intensityArr[6];
char servoAngleArr[10];
char tempAr[6];

// Variables for new averaging feature
unsigned long lastSampleTime = 0;
unsigned long lastSendTime = 0;

int sampling_interval = 5;  // seconds, default
int sending_interval = 10; // seconds, default

#define MAX_SAMPLES 1440  // To handle maximum (assuming ts as low as 5s, tu as long as 2 hours)
float intensitySamples[MAX_SAMPLES];
int sampleCount = 0;

void setup() {
  Serial.begin(9600);

  setupWifi();

  setupMqtt();

  servo.attach(SERVO_PIN, 500, 2400);

  lastSampleTime = millis();
  lastSendTime = millis();

  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);

}

void loop() {
  if (!mqttClient.connected()) {
    connectToBroker();
  }

  mqttClient.loop();

  updateTemperature();
  Serial.println(tempAr);
  mqttClient.publish("Temperature", tempAr);
  delay(1000);

  unsigned long currentTime = millis();

  // Sampling at ts interval
  if (currentTime - lastSampleTime >= sampling_interval * 1000) {
    lastSampleTime = currentTime;
    readAndBufferIntensity();
  }

  // Sending at tu interval
  if (currentTime - lastSendTime >= sending_interval * 1000) {
    lastSendTime = currentTime;
    sendAverageIntensity();
  }

  delay(10);
}

/**
   Function to setup wifi connection
*/
void setupWifi() {
  WiFi.begin("Wokwi-GUEST", "", 6);
  Serial.print("Connecting to WIFI");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Connected to WIFI");
}

/**
   Function to setup mqtt server & receiveCallback
*/
void setupMqtt() {
  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(receiveCallback);
}

/**
   Function to connect to mqtt broker
*/
void connectToBroker() {
  while (!mqttClient.connected()) {
    Serial.println("Attempting MQTT Connection....");

    if (mqttClient.connect("EPS32-41234123")) {
      Serial.println("Connected");
      mqttClient.subscribe(SERVO_MIN_ANGLE_TOPIC);
      mqttClient.subscribe(SERVO_CONTROL_FACTOR_TOPIC);
      mqttClient.subscribe(SAMPLING_INTERVAL_TOPIC);
      mqttClient.subscribe(SENDING_INTERVAL_TOPIC);
      mqttClient.subscribe(IDEAL_MED_TEMP_TOPIC);
    } else {
      Serial.println("Failed");
      Serial.println(mqttClient.state());
      delay(5000);
    }
  }
}

void updateTemperature() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  String(data.temperature, 2).toCharArray(tempAr, 6);
}
/**
   Function to read light intensity from LDR sensor & publish values
*/
void readIntensity() {
  int analogValue = analogRead(LIGHT_SENSOR_PIN);

  Serial.println("Analog Value = " + String(analogValue));

  float prevValue = intensity;

  intensity = map(analogValue, 0, 4095, 100, 0) / 100.00;

  String(intensity, 2).toCharArray(intensityArr, 6);

  mqttClient.publish(LIGHT_INTENSITY_TOPIC, intensityArr);

  Serial.println("Light Intensity = " + String(intensity));

  if (prevValue != intensity) {
    calculateServoPos();
  }
}

/**
   Function to read and buffer intensity (every ts)
*/
void readAndBufferIntensity() {
  int analogValue = analogRead(LIGHT_SENSOR_PIN);
  float currentIntensity = map(analogValue, 0, 4095, 100, 0) / 100.00;

  if (sampleCount < MAX_SAMPLES) {
    intensitySamples[sampleCount++] = currentIntensity;
  } else {
    // If buffer full, shift left (simple circular buffer logic could also be used)
    for (int i = 1; i < MAX_SAMPLES; i++) {
      intensitySamples[i - 1] = intensitySamples[i];
    }
    intensitySamples[MAX_SAMPLES - 1] = currentIntensity;
  }
  calculateServoPos();


  Serial.println("Buffered Intensity = " + String(currentIntensity));
}

/**
   Function to send average light intensity to dashboard (every tu)
*/
void sendAverageIntensity() {
  if (sampleCount == 0) return;

  float sum = 0;
  for (int i = 0; i < sampleCount; i++) {
    sum += intensitySamples[i];
  }

  float avgIntensity = sum / sampleCount;

  Serial.println("Average Intensity = " + String(avgIntensity));

  char avgIntensityArr[6];
  String(avgIntensity, 2).toCharArray(avgIntensityArr, 6);

  mqttClient.publish(AVG_LIGHT_INTENSITY_TOPIC, avgIntensityArr);

  // Reset buffer for next cycle
  sampleCount = 0;
}

/**
   Function to calculate servo motor angle to adjust sliding window
*/
float I = getAverageIntensity();  // Use average instead of latest

void calculateServoPos() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  float T = data.temperature;
  float T_med = 30.0;

  if (isnan(T) || T <= 0) {
    Serial.println("Invalid temperature reading.");
    return;
  }

  if (sampling_interval == 0 || sending_interval == 0) {
    Serial.println("Invalid time intervals.");
    return;
  }

  float I = getAverageIntensity();  // Use average instead of last reading
  float gamma = control_factor;

  float lnRatio = -log((float)sampling_interval / (float)sending_interval);
  if (isnan(lnRatio) || isinf(lnRatio)) lnRatio = 1.0;

  float factor = I * gamma * lnRatio * (T / T_med);
  float servo_angle = offset_angle + (180.0 - offset_angle) * factor;
  servo_angle = constrain(servo_angle, 0, 180);

  servo.write(servo_angle);

  String(servo_angle, 2).toCharArray(servoAngleArr, 10);
  mqttClient.publish(SERVO_MOTOR_ANGLE_TOPIC, servoAngleArr);
}


float getAverageIntensity() {
  if (sampleCount == 0) return 0.0;

  float sum = 0;
  for (int i = 0; i < sampleCount; i++) {
    sum += intensitySamples[i];
  }
  return sum / sampleCount;
}


/**
   Callback function to listening to incoming mqtt messages.
*/
void receiveCallback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Message received [" + String(topic) + "]");

  char payloadArr[length + 1];
  for (int i = 0; i < length; i++) {
    payloadArr[i] = (char) payload[i];
  }
  payloadArr[length] = '\0';

  if (strcmp(topic, SERVO_MIN_ANGLE_TOPIC) == 0) {
    offset_angle = atof(payloadArr);
    Serial.println("Offset Angle = " + String(offset_angle));
    calculateServoPos();
  } else if (strcmp(topic, SERVO_CONTROL_FACTOR_TOPIC) == 0) {
    control_factor = atof(payloadArr);
    Serial.println("Controlling Factor = " + String(control_factor));
    calculateServoPos();
  } else if (strcmp(topic, SAMPLING_INTERVAL_TOPIC) == 0) {
    sampling_interval = atoi(payloadArr);
    Serial.println("Updated Sampling Interval = " + String(sampling_interval) + "s");
  } else if (strcmp(topic, SENDING_INTERVAL_TOPIC) == 0) {
    sending_interval = atoi(payloadArr);
    Serial.println("Updated Sending Interval = " + String(sending_interval) + "s");
  }
  else if (strcmp(topic, IDEAL_MED_TEMP_TOPIC) == 0) {
  T_med = atof(payloadArr);
  Serial.println("Updated T_med = " + String(T_med) + "°C");
  calculateServoPos();
  }
}
