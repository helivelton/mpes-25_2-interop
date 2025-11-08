#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <ESP32Servo.h>

#define WIFI_SSID "uaifai-brum"
#define WIFI_PASSWORD "env.wifi_password"

#define DATABASE_URL "env.firebasedatabase_url"
#define DATABASE_SECRET "env.firebasedatabase_secret"

#define SERVO_PIN 25

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long lastFirebaseWrite = 0;
const unsigned long firebaseWriteInterval = 5000;

Servo myservo;

void configureWiFi();
void configureFirebase();
void writeUptimeToFirebase();
void attachServo();

void setup() {
  Serial.begin(9600);

  configureWiFi();
  configureFirebase();
  myservo.attach(SERVO_PIN);
}

void loop() {
  attachServo();

  int open = 0;
  int close = 180;

  myservo.write(open);
  delay(2000);
  myservo.write(close);
  delay(2000);

  if (millis() - lastFirebaseWrite >= firebaseWriteInterval) {
    lastFirebaseWrite = millis();

    writeUptimeToFirebase();
  }
}

void configureWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print('.');
  }
  Serial.println("WiFi connected");
}

void configureFirebase() {
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = DATABASE_SECRET;

  Firebase.reconnectNetwork(true);
  fbdo.setBSSLBufferSize(4096, 1024);

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void writeUptimeToFirebase() {
  String uptimePath = "/data/uptime";
  uint32_t uptime = millis() / 1000;

  if (Firebase.RTDB.setInt(&fbdo, uptimePath.c_str(), uptime)) {
    Serial.println("RTDB: uptime written");
  } else {
    Serial.print("RTDB write failed: ");
    Serial.println(fbdo.errorReason());
  }
}

void attachServo() {
	if (!myservo.attached()) {
		myservo.setPeriodHertz(50);
		myservo.attach(SERVO_PIN, 1000, 2000);
	}
}
