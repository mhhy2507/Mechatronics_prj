#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "HX711.h"

/* ===== WiFi / MQTT (HiveMQ Cloud TLS) ===== */
const char* ssid          = "P1204";
const char* password      = "p1204ktxbk";

const char* mqtt_server   = "3fb18e12037a4d608f5aa36c054675d2.s1.eu.hivemq.cloud";
const int   mqtt_port     = 8883;         // TLS
const char* mqtt_username = "mhhy2507";
const char* mqtt_password = "Huye1s123";

/* Topic xuất dữ liệu telemetry và topic nhận (đang để trùng để xem lại gói) */
const char* pub_topic     = "lf/telemetry";
const char* sub_topic     = "lf/telemetry";   // nếu không muốn nhận lại gói của chính mình, đổi sang "lf/cmd" chẳng hạn

WiFiClientSecure tlsClient;
PubSubClient client(tlsClient);

/* ===== Pins ===== */
/* CẢNH BÁO: GPIO 12,14,25,26,27 là ADC2 (xung đột Wi-Fi). Khuyến nghị dùng ADC1: 32,33,34,35,36,39. */
#define ss1 32
#define ss2 33
#define ss3 25  // ADC2 (cân nhắc đổi sang 34)
#define ss4 26  // ADC2 (cân nhắc đổi sang 35)
#define ss5 27  // ADC2 (cân nhắc đổi sang 36/39)
#define ss6 14  // ADC2
#define ss7 12  // ADC2

// HX711
#define DATA_PIN  2
#define CLOCK_PIN 4

/* ===== Config đo lường ===== */
int   threshold = 1200;
float CALIBRATION_FACTOR = -437.27;

/* ===== Biến đo lường ===== */
HX711 scale;

int s[8];  // s[1]..s[7]
int d[8];
int D = 0;

float sensorValue1, sensorValue2, sensorValue3, sensorValue4, sensorValue5, sensorValue6, sensorValue7;
float TB = 0.0f;
float x  = 0.0f;

long weight_In_g = 0;

/* ===== State ===== */
bool printedHeader = false;
unsigned long lastPub = 0;  // publish mỗi 100 ms

/* ===== WiFi & MQTT ===== */
void setup_wifi() {
  Serial.print("Connecting to "); Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientID = "ESPClient-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (client.connect(clientID.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(sub_topic);
      Serial.print("Subscribed: "); Serial.println(sub_topic);
    } else {
      Serial.print("failed, rc="); Serial.print(client.state());
      Serial.println(" retry in 5 seconds");
      delay(5000);
    }
  }
}

/* ===== Đọc line sensors ===== */
void readLineSensors() {
  s[1] = analogRead(ss1);
  s[2] = analogRead(ss2);
  s[3] = analogRead(ss3);
  s[4] = analogRead(ss4);
  s[5] = analogRead(ss5);
  s[6] = analogRead(ss6);
  s[7] = analogRead(ss7);

  D = 0;
  for (int i = 1; i <= 7; i++) {
    d[i] = (s[i] > threshold) ? 1 : 0;
    D += d[i];
  }

  // Calib tuyến tính theo hệ số bạn cung cấp
  sensorValue1 = 123.4286 + 1.0109 * (s[1] - 112);
  sensorValue2 = 123.4286 + 0.9597 * (s[2] - 115);
  sensorValue3 = 123.4286 + 1.0116 * (s[3] - 115);
  sensorValue4 = 123.4286 + 1.0659 * (s[4] - 97);
  sensorValue5 = 123.4286 + 1.0399 * (s[5] - 100);
  sensorValue6 = 123.4286 + 0.9791 * (s[6] - 160);
  sensorValue7 = 123.4286 + 0.9443 * (s[7] - 165);

  float num = 3 * (sensorValue1 - sensorValue7)
            + 2 * (sensorValue2 - sensorValue6)
            +     (sensorValue3 - sensorValue5);
  float den = sensorValue1 + sensorValue2 + sensorValue3 +
              sensorValue4 + sensorValue5 + sensorValue6 + sensorValue7;

  TB = (den != 0.0f) ? (num * 17.0f / den) : 0.0f;

  if (D == 3)       x = 1000;
  else if (D == 0)  x = 999;
  else              x = TB * 0.9822 - 1.6627;
}

/* ===== Đọc loadcell ===== */
void readLoadcell() {
  if (scale.is_ready()) {
    weight_In_g = scale.get_units(10);  // trung bình 10 mẫu
  }
}

/* ===== Publish telemetry JSON ===== */
void publishTelemetry() {
  // Xây JSON phẳng đúng key
  String js = "{";
  js += "\"time_ms\":" + String(millis()) + ",";
  js += "\"weight_g\":" + String(weight_In_g) + ",";
  js += "\"s1\":" + String(s[1]) + ",";
  js += "\"s2\":" + String(s[2]) + ",";
  js += "\"s3\":" + String(s[3]) + ",";
  js += "\"s4\":" + String(s[4]) + ",";
  js += "\"s5\":" + String(s[5]) + ",";
  js += "\"s6\":" + String(s[6]) + ",";
  js += "\"s7\":" + String(s[7]) + ",";
  js += "\"d1\":" + String(d[1]) + ",";
  js += "\"d2\":" + String(d[2]) + ",";
  js += "\"d3\":" + String(d[3]) + ",";
  js += "\"d4\":" + String(d[4]) + ",";
  js += "\"d5\":" + String(d[5]) + ",";
  js += "\"d6\":" + String(d[6]) + ",";
  js += "\"d7\":" + String(d[7]) + ",";
  js += "\"D\":"  + String(D) + ",";
  js += "\"TB\":" + String(TB, 5) + ",";
  js += "\"x\":"  + String(x, 5);
  js += "}";

  client.publish(pub_topic, js.c_str());
}

/* ===== MQTT callback: nhận JSON và in CSV ===== */
void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) { Serial.print("JSON error: "); Serial.println(err.f_str()); return; }

  uint32_t time_ms = doc["time_ms"] | 0;
  long     weight  = doc["weight_g"] | 0;

  int s1 = doc["s1"] | 0, s2 = doc["s2"] | 0, s3 = doc["s3"] | 0, s4 = doc["s4"] | 0;
  int s5 = doc["s5"] | 0, s6 = doc["s6"] | 0, s7 = doc["s7"] | 0;

  int d1 = doc["d1"] | 0, d2 = doc["d2"] | 0, d3 = doc["d3"] | 0, d4 = doc["d4"] | 0;
  int d5 = doc["d5"] | 0, d6 = doc["d6"] | 0, d7 = doc["d7"] | 0;

  int   D_rx  = doc["D"]  | 0;
  float TB_rx = doc["TB"] | 0.0f;
  float x_rx  = doc["x"]  | 0.0f;

  if (!printedHeader) {
    Serial.println("time_ms,weight_g,s1,s2,s3,s4,s5,s6,s7,d1,d2,d3,d4,d5,d6,d7,D,TB,x");
    printedHeader = true;
  }

  Serial.print(time_ms); Serial.print(",");
  Serial.print(weight);  Serial.print(",");
  Serial.print(s1); Serial.print(","); Serial.print(s2); Serial.print(","); Serial.print(s3); Serial.print(",");
  Serial.print(s4); Serial.print(","); Serial.print(s5); Serial.print(","); Serial.print(s6); Serial.print(",");
  Serial.print(s7); Serial.print(",");
  Serial.print(d1); Serial.print(","); Serial.print(d2); Serial.print(","); Serial.print(d3); Serial.print(",");
  Serial.print(d4); Serial.print(","); Serial.print(d5); Serial.print(","); Serial.print(d6); Serial.print(",");
  Serial.print(d7); Serial.print(",");
  Serial.print(D_rx);  Serial.print(",");
  Serial.print(TB_rx, 5); Serial.print(",");
  Serial.println(x_rx, 5);
}

/* ===== Setup / Loop ===== */
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Line sensors
  pinMode(ss1, INPUT); pinMode(ss2, INPUT); pinMode(ss3, INPUT);
  pinMode(ss4, INPUT); pinMode(ss5, INPUT); pinMode(ss6, INPUT); pinMode(ss7, INPUT);

  // HX711
  scale.begin(DATA_PIN, CLOCK_PIN);
  scale.set_scale(CALIBRATION_FACTOR);
  scale.tare();

  // Wi-Fi + MQTT TLS
  setup_wifi();
  tlsClient.setInsecure();                 // test nhanh; production nên dùng CA chuẩn
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setBufferSize(4096);              // JSON lớn an toàn hơn
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  readLineSensors();
  readLoadcell();

  unsigned long now = millis();
  if (now - lastPub >= 100) {  // 10 Hz
    publishTelemetry();
    lastPub = now;
  }
}
