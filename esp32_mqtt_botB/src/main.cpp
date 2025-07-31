#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "MPU6050.h"

#define BOT_ID "B"
#define NEXT_BOT_TOPIC "/bot/C"
const char* ssid = "Heckers-dad";
const char* password = "hjahify1";
const char* mqtt_server = "192.168.161.246";

WiFiClient espClient;
PubSubClient client(espClient);
Servo scanner;
MPU6050 mpu;

#define ENA 5
#define IN1 27
#define IN2 14
#define ENB 4
#define IN3 13
#define IN4 12
#define SERVO_PIN 25
#define TRIG_PIN 33
#define ECHO_PIN 26

bool busy = false;

void mqttLog(const String& msg) {
  Serial.println(msg);
  client.publish("/bot/" BOT_ID "/logs", msg.c_str());
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  mqttLog("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    mqttLog(".");
  }
  mqttLog("WiFi connected: " + WiFi.localIP().toString());
}

void reconnect() {
  while (!client.connected()) {
    String clientId = "ESP32Client_" BOT_ID;
    if (client.connect(clientId.c_str())) {
      mqttLog("MQTT connected.");
      client.subscribe("/bot/" BOT_ID);
    } else {
      mqttLog("MQTT failed: " + String(client.state()));
      delay(2000);
    }
  }
}

void setup_motors() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
}

void moveForward() {
  analogWrite(ENA, 150); analogWrite(ENB, 150);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void turnRight() {
  analogWrite(ENA, 70); analogWrite(ENB, 70);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnLeft() {
  analogWrite(ENA, 70); analogWrite(ENB, 70);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

long readDistanceCM() {
  long total = 0;
  int valid = 0;
  for (int i = 0; i < 5; i++) {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 20000);
    long cm = duration * 0.034 / 2;
    if (cm > 2 && cm < 300) {
      total += cm;
      valid++;
    }
    delay(10);
  }
  return (valid > 0) ? total / valid : 999;
}

void turnToAngle(int targetAngle) {
  mqttLog("Turning to angle: " + String(targetAngle));
  int diff = targetAngle - 90;
  if (abs(diff) < 5) {
    mqttLog("No turn needed.");
    return;
  }

  if (diff < 0) {
    turnRight(); delay(abs(diff) * 6);
  } else {
    turnLeft(); delay(diff * 6);
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

    // Start moving forward
    moveForward();
    unsigned long start = millis();
    while (millis() - start < 250) {
      long dist = readDistanceCM();
      if (dist <= 15) {
        stopMotors();
        mqttLog("Obstacle detected at " + String(dist) + " cm. Stopping...");

        // Wait until path clears
        while (readDistanceCM() <= 15) {
          mqttLog("Still blocked: " + String(readDistanceCM()) + " cm");
          delay(300);
          client.loop(); yield();
        }

        mqttLog("Path clear. Resuming...");
        moveForward();  // Resume remaining movement in this burst
        start = millis(); // Reset burst timer
      }

      client.loop(); yield();
    }

    stopMotors();
    delay(150);
  }

  mqttLog("Movement complete.");
}

void handleCommand(String msg) {
  if (busy) {
    mqttLog("Bot B busy. Forwarding to Bot C...");
    client.publish(NEXT_BOT_TOPIC, msg.c_str());
    return;
  }

  if (msg.startsWith("MOVE")) {
    busy = true;
    int s1 = msg.indexOf(' ');
    int s2 = msg.indexOf(' ', s1 + 1);
    int angle = msg.substring(s1 + 1, s2).toInt();
    int dist = msg.substring(s2 + 1).toInt();
    turnToAngle(angle);
    travelScanDistance(dist);
    busy = false;
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  mqttLog("Received: " + msg);
  handleCommand(msg);
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  setup_motors();
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  Wire.begin(21, 22);
  mpu.initialize();
  scanner.attach(SERVO_PIN);
  scanner.write(90);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();
}

