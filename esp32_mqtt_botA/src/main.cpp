#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "MPU6050.h"

// === WiFi & MQTT ===
#define BOT_ID "A"
const char* ssid = "mario";
const char* password = "westvirginia";
const char* mqtt_server = "10.130.145.225";

WiFiClient espClient;
PubSubClient client(espClient);

// === Pins ===
#define ENA 5
#define IN1 27
#define IN2 14
#define ENB 4
#define IN3 13
#define IN4 12
#define SERVO_PIN 25
#define TRIG_PIN 33
#define ECHO_PIN 26

Servo scanner;
MPU6050 mpu;
bool isBusy = false;

// === Utilities ===
void mqttLog(const String& msg) {
  Serial.println(msg);
  client.publish("/bot/" BOT_ID "/logs", msg.c_str());
}

// === WiFi / MQTT ===
void setup_wifi() {
  WiFi.begin(ssid, password);
  mqttLog("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); mqttLog(".");
  }
  mqttLog("WiFi connected: " + WiFi.localIP().toString());
}

void reconnect() {
  while (!client.connected()) {
    mqttLog("Reconnecting MQTT...");
    String clientId = String("ESP32Client_") + BOT_ID;
    if (client.connect(clientId.c_str())) {
      mqttLog("MQTT connected.");
      client.subscribe("/bot/" BOT_ID);
    } else {
      mqttLog("MQTT failed: " + String(client.state()));
      delay(2000);
    }
  }
}

// === Motor Control ===
void setup_motors() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
}

void moveForward() {
  analogWrite(ENA, 255); analogWrite(ENB, 255);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void turnRight() {
  analogWrite(ENA, 255); analogWrite(ENB, 255);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnLeft() {
  analogWrite(ENA, 255); analogWrite(ENB, 255);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

// === Sensing ===
long readDistanceCM() {
  long total = 0;
  int validReads = 0;
  for (int i = 0; i < 5; i++) {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 20000);
    long cm = duration * 0.034 / 2;
    if (cm > 2 && cm < 300) {
      total += cm;
      validReads++;
    }
    delay(10);
  }
  return (validReads > 0) ? total / validReads : 999;
}

// === Movement & Navigation ===
void turnToAngle(int targetAngle) {
  mqttLog("Turning to angle: " + String(targetAngle));

  int diff = targetAngle - 90;
  if (abs(diff) < 5) {
    mqttLog("No turn needed.");
    return;
  }

  if (diff < 0) {
    turnRight();
    delay(abs(diff) * 6); // Adjust 6ms per degree
  } else {
    turnLeft();
    delay(diff * 6);
  }

  stopMotors();
  mqttLog("Turn complete.");
}

void travelScanDistance(int totalCM) {
  mqttLog("Target distance: " + String(totalCM) + " cm");
  const float step = 18.25;
  int steps = round(totalCM / step);

  for (int i = 0; i < steps; i++) {
    mqttLog("Step " + String(i + 1) + " of " + String(steps));

    long dist = readDistanceCM();
    mqttLog("Distance: " + String(dist) + " cm");

    // Obstacle check + wait loop
    if (dist <= 15) {
      mqttLog("Obstacle too close. Waiting...");
      stopMotors();
      while (readDistanceCM() <= 15) {
        mqttLog("Still blocked: " + String(readDistanceCM()) + " cm");
        delay(300);
        client.loop(); yield();
      }
      mqttLog("Path clear. Resuming.");
    }

    // Move forward one burst
    moveForward();
    delay(250);
    stopMotors();
    delay(150);
    client.loop(); yield();
  }

  mqttLog("Movement complete.");
}

// === Command Handling ===
void handleCommand(String msg) {
  if (msg.startsWith("MOVE")) {
    if (isBusy) {
      mqttLog("I'm busy. Passing task to Bot B.");
      client.publish("/bot/B", msg.c_str());  // forward to Bot B
      return;
    }

    isBusy = true;
    int space1 = msg.indexOf(' ');
    int space2 = msg.indexOf(' ', space1 + 1);
    if (space1 < 0 || space2 < 0) {
      mqttLog("Invalid MOVE command. Use: MOVE <angle> <cm>");
      isBusy = false;
      return;
    }

    int angle = msg.substring(space1 + 1, space2).toInt();
    int dist = msg.substring(space2 + 1).toInt();
    turnToAngle(angle);
    travelScanDistance(dist);
    isBusy = false;
  }
  else if (msg == "STOP") {
    stopMotors();
  }
  else if (msg == "SCAN") {
    scanner.write(0); delay(300);
    scanner.write(90); delay(300);
    scanner.write(180); delay(300);
    scanner.write(90);
  }
}

// === MQTT Callback ===
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  mqttLog("Received: " + msg);
  handleCommand(msg);
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  setup_motors();

  Wire.begin(21, 22);
  mpu.initialize();
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  scanner.attach(SERVO_PIN);
  scanner.write(90);
  mqttLog(mpu.testConnection() ? "MPU6050 connected!" : "MPU6050 failed!");
}

// === Loop ===
void loop() {
  if (!client.connected()) reconnect();
  client.loop();
}

