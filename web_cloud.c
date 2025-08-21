#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "HX711.h"

/* ========= WiFi / MQTT (HiveMQ Cloud TLS) ========= */
const char* ssid          = "P1204";
const char* password      = "p1204ktxbk";

const char* mqtt_server   = "3fb18e12037a4d608f5aa36c054675d2.s1.eu.hivemq.cloud";
const int   mqtt_port     = 8883;  // TLS
const char* mqtt_username = "mhhy2507";
const char* mqtt_password = "Huye1s123";

const char* pub_topic     = "lf/telemetry";  // ESP32 -> server/web (sensor JSON)
const char* cmd_topic     = "lf/cmd";        // server/web -> ESP32 (motor command)
const char* status_topic  = "lf/status";     // ESP32 -> server/web (ACK + RPM)

/* ========= Pins: 7 line sensors & HX711 ========= */
/* Lưu ý: 12,14,25,26,27 là ADC2 (dễ nhiễu khi Wi-Fi). Nếu raw sai, chuyển sang ADC1: 32,33,34,35,36,39 */
#define ss1 32
#define ss2 33
#define ss3 25   // khuyến nghị đổi sang 34 nếu có thể
#define ss4 26   // khuyến nghị đổi sang 35 nếu có thể
#define ss5 27   // khuyến nghị đổi sang 36/39 nếu có thể
#define ss6 14
#define ss7 12

// HX711
#define DATA_PIN  2    // DT
#define CLOCK_PIN 4    // SCK

/* ========= Motor driver & encoders ========= */
// LEFT motor
const int L_IN1 = 22;     // AIN1
const int L_IN2 = 18;     // AIN2
const int L_PWM = 23;     // PWM
const int L_ENC_A = 34;   // input-only, NO internal pull-up
const int L_ENC_B = 35;   // input-only, NO internal pull-up
const int L_PPR   = 660;

// RIGHT motor
const int R_IN1 = 21;     // AIN1
const int R_IN2 = 19;     // AIN2
const int R_PWM = 5;      // PWM
const int R_ENC_A = 17;   // có PU nội
const int R_ENC_B = 16;   // có PU nội
const int R_PPR   = 660;

/* ========= Intervals ========= */
const unsigned long LOG_INTERVAL_MS    = 50;    // tính RPM + log serial
const unsigned long TELEMETRY_INTERVAL = 300;   // publish telemetry (~3.3 Hz)
const unsigned long STATUS_INTERVAL_MS = 1000;  // publish status
const unsigned long HX_MIN_INTERVAL_MS = 30;    // HX711 poll tối thiểu (non-block)

/* ========= Line-sensor & loadcell config ========= */
int   threshold = 1200;
float CALIBRATION_FACTOR = -437.27;

/* ========= Globals ========= */
HX711 scale;

int s[8];  // s[1..7]
int d[8];  // d[1..7]
int D = 0;

float sensorValue1, sensorValue2, sensorValue3, sensorValue4, sensorValue5, sensorValue6, sensorValue7;
float TB = 0.0f;
float x  = 0.0f;

long weight_In_g = 0;
unsigned long hx_last_read_ms = 0;   // anti-blocking timing

/* Encoders & motor state */
volatile long  L_counts = 0;
volatile int   L_lastA  = LOW;
volatile long  R_counts = 0;
volatile int   R_lastA  = LOW;

int   L_pwm_cmd = 0;   // -255..+255
int   R_pwm_cmd = 0;   // -255..+255
float L_rpm = 0.0f;
float R_rpm = 0.0f;

/* MQTT client */
WiFiClientSecure tlsClient;
PubSubClient client(tlsClient);

/* Flags / timers */
bool printedHeader = false;
unsigned long lastLogTime    = 0;
unsigned long lastTelemTime  = 0;
unsigned long lastStatTime   = 0;

/* ========= Motor helpers (analogWrite → LEDC nội bộ) ========= */
inline void writePWM(int pin, int duty) {
  duty = constrain(duty, 0, 255);
  analogWrite(pin, duty);
}

void leftStop() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  writePWM(L_PWM, 0);
  L_pwm_cmd = 0;
}
void rightStop() {
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
  writePWM(R_PWM, 0);
  R_pwm_cmd = 0;
}

void leftSetPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  L_pwm_cmd = pwm;
  if (pwm > 0) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    writePWM(L_PWM, pwm);
  } else if (pwm < 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    writePWM(L_PWM, -pwm);
  } else {
    leftStop();
  }
}
void rightSetPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  R_pwm_cmd = pwm;
  if (pwm > 0) {
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    writePWM(R_PWM, pwm);
  } else if (pwm < 0) {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    writePWM(R_PWM, -pwm);
  } else {
    rightStop();
  }
}

/* Refresh PWM/DIR mỗi vòng để tránh “rụng lệnh” */
void applyMotorOutputs() {
  if (L_pwm_cmd > 0)    { digitalWrite(L_IN1, LOW);  digitalWrite(L_IN2, HIGH); writePWM(L_PWM,  L_pwm_cmd); }
  else if (L_pwm_cmd<0) { digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);  writePWM(L_PWM, -L_pwm_cmd); }
  else                  { leftStop(); }

  if (R_pwm_cmd > 0)    { digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);  writePWM(R_PWM,  R_pwm_cmd); }
  else if (R_pwm_cmd<0) { digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH); writePWM(R_PWM, -R_pwm_cmd); }
  else                  { rightStop(); }
}

/* ========= Encoder ISRs ========= */
void IRAM_ATTR L_handleEncoder() {
  int a = digitalRead(L_ENC_A);
  int b = digitalRead(L_ENC_B);
  if (a != L_lastA) {
    L_counts += (a == b) ? +1 : -1;
    L_lastA = a;
  }
}
void IRAM_ATTR R_handleEncoder() {
  int a = digitalRead(R_ENC_A);
  int b = digitalRead(R_ENC_B);
  if (a != R_lastA) {
    R_counts += (a == b) ? +1 : -1;
    R_lastA = a;
  }
}

/* ========= Line sensors ========= */
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

  sensorValue1 = 123.4286 + 1.0109 * (s[1] - 112);
  sensorValue2 = 123.4286 + 0.9597 * (s[2] - 115);
  sensorValue3 = 123.4286 + 1.0116 * (s[3] - 115);
  sensorValue4 = 123.4286 + 1.0659 * (s[4] - 97);
  sensorValue5 = 123.4286 + 1.0399 * (s[5] - 100);
  sensorValue6 = 123.4286 + 0.9791 * (s[6] - 160);
  sensorValue7 = 123.4286 + 0.9443 * (s[7] - 165);

  float num = 3*(sensorValue1 - sensorValue7)
            + 2*(sensorValue2 - sensorValue6)
            +   (sensorValue3 - sensorValue5);
  float den = sensorValue1 + sensorValue2 + sensorValue3 +
              sensorValue4 + sensorValue5 + sensorValue6 + sensorValue7;

  TB = (den != 0.0f) ? (num * 17.0f / den) : 0.0f;

  if (D == 3)       x = 1000;
  else if (D == 0)  x = 999;
  else              x = TB * 0.9822 - 1.6627;
}

/* ========= Loadcell non-blocking ========= */
void pollLoadcell() {
  unsigned long now = millis();
  if (now - hx_last_read_ms < HX_MIN_INTERVAL_MS) return; // giới hạn tần số đọc
  // Chỉ đọc khi HX711 đã sẵn sàng -> 1 mẫu -> không block
  if (scale.is_ready()) {
    // Lấy 1 mẫu theo scale/tare đã set -> đơn vị "gram" (theo CALIBRATION_FACTOR của bạn)
    weight_In_g = scale.get_units(1);
    hx_last_read_ms = now;
  }
}

/* ========= Telemetry publish (ArduinoJson + buffer tĩnh) ========= */
void publishTelemetry() {
  if (!client.connected()) return;

  StaticJsonDocument<512> doc;
  doc["time_ms"]  = millis();
  doc["weight_g"] = weight_In_g;

  doc["s1"]=s[1]; doc["s2"]=s[2]; doc["s3"]=s[3]; doc["s4"]=s[4];
  doc["s5"]=s[5]; doc["s6"]=s[6]; doc["s7"]=s[7];

  doc["d1"]=d[1]; doc["d2"]=d[2]; doc["d3"]=d[3]; doc["d4"]=d[4];
  doc["d5"]=d[5]; doc["d6"]=d[6]; doc["d7"]=d[7];

  doc["D"]  = D;
  doc["TB"] = TB;
  doc["x"]  = x;

  char buf[600];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  client.publish(pub_topic, buf, n);
}

/* ========= Status publish ========= */
void publishStatus(const char* why = "ack") {
  if (!client.connected()) return;
  StaticJsonDocument<256> doc;
  doc["type"]  = why;
  doc["L_pwm"] = L_pwm_cmd;
  doc["R_pwm"] = R_pwm_cmd;
  doc["L_rpm"] = L_rpm;
  doc["R_rpm"] = R_rpm;
  char buf[256];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  client.publish(status_topic, buf, n);
}

/* ========= Command handlers ========= */
// JSON: {"L":120}, {"R":-80}, {"L":100,"R":100}, {"B":90}, {"cmd":"L","pwm":120}, {"cmd":"STOP"}
void handleCommandJSON(JsonVariant v) {
  if (!v.is<JsonObject>()) return;
  JsonObject o = v.as<JsonObject>();

  if (o.containsKey("cmd")) {
    String cmd = o["cmd"].as<String>(); cmd.toUpperCase();
    if (cmd == "STOP") { leftStop(); rightStop(); publishStatus("stop"); return; }
    int pwm = o["pwm"] | 0;
    if (cmd == "L") { leftSetPWM(pwm); publishStatus("cmd_L"); return; }
    if (cmd == "R") { rightSetPWM(pwm); publishStatus("cmd_R"); return; }
    if (cmd == "B") { leftSetPWM(pwm); rightSetPWM(pwm); publishStatus("cmd_B"); return; }
  }

  bool touched = false;
  if (o.containsKey("L")) { leftSetPWM((int)o["L"]); touched = true; }
  if (o.containsKey("R")) { rightSetPWM((int)o["R"]); touched = true; }
  if (o.containsKey("B")) { int p=(int)o["B"]; leftSetPWM(p); rightSetPWM(p); touched = true; }
  if (touched) publishStatus("set_LR");
}

// Chuỗi: "L120", "R-80", "B90", "STOP"
void handleCommandString(const String& s) {
  String t = s; t.trim();
  String up = t; up.toUpperCase();
  if (up == "STOP") { leftStop(); rightStop(); publishStatus("stop"); return; }

  if (t.length() >= 2) {
    char c = t.charAt(0);
    int val = t.substring(1).toInt();
    if (c == 'L' || c == 'l') { leftSetPWM(val); publishStatus("cmd_L"); return; }
    if (c == 'R' || c == 'r') { rightSetPWM(val); publishStatus("cmd_R"); return; }
    if (c == 'B' || c == 'b') { leftSetPWM(val); rightSetPWM(val); publishStatus("cmd_B"); return; }
  }
}

/* ========= MQTT callback ========= */
void callback(char* topic, byte* payload, unsigned int length) {
  if (String(topic) != String(cmd_topic)) return;   // chỉ xử lý lệnh

  // Debug lệnh nhận
  Serial.print("[CMD] topic="); Serial.print(topic);
  Serial.print(" payload=");
  for (unsigned int i=0;i<length;i++) Serial.print((char)payload[i]);
  Serial.print(" len="); Serial.println(length);

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (!err) { handleCommandJSON(doc.as<JsonVariant>()); return; }

  // Không phải JSON → chuỗi
  String msg; msg.reserve(length);
  for (unsigned int i=0;i<length;i++) msg += (char)payload[i];
  handleCommandString(msg);
}

/* ========= WiFi / MQTT connect ========= */
void setup_wifi() {
  Serial.print("Connecting to "); Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);  // tránh modem sleep
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
}
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientID = "ESPClient-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (client.connect(clientID.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(cmd_topic);
      Serial.print("Subscribed: "); Serial.println(cmd_topic);
    } else {
      Serial.print("failed, rc="); Serial.print(client.state());
      Serial.println(" retry in 5 seconds");
      delay(5000);
    }
  }
}

/* ========= Setup / Loop ========= */
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

  // Motor DIR + PWM
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(L_PWM, OUTPUT); pinMode(R_PWM, OUTPUT);
  leftStop(); rightStop();

  // Encoders
  // ⚠️ 34/35 KHÔNG có pull-up nội → dùng INPUT thường + pull-up ngoài 10k (hoặc đổi chân)
  pinMode(L_ENC_A, INPUT);
  pinMode(L_ENC_B, INPUT);
  L_lastA = digitalRead(L_ENC_A);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), L_handleEncoder, CHANGE);

  // 16/17 có pull-up nội
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);
  R_lastA = digitalRead(R_ENC_A);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), R_handleEncoder, CHANGE);

  // Wi-Fi + MQTT TLS
  setup_wifi();
  tlsClient.setInsecure();        // test nhanh; production nên dùng CA chuẩn
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setBufferSize(4096);
  client.setKeepAlive(30);
  client.setSocketTimeout(5);

  if (!printedHeader) {
    Serial.println("Time,LPWM,RPWM,LRPM,RRPM");
    printedHeader = true;
  }
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  readLineSensors();
  pollLoadcell();   // <— không chặn

  unsigned long now = millis();

  if (now - lastTelemTime >= TELEMETRY_INTERVAL) {
    publishTelemetry();
    lastTelemTime = now;
  }

  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    static long L_lastCounts = 0;
    static long R_lastCounts = 0;

    long L_delta = L_counts - L_lastCounts;
    long R_delta = R_counts - R_lastCounts;
    L_lastCounts = L_counts;
    R_lastCounts = R_counts;

    unsigned long dt = now - lastLogTime;
    L_rpm = (L_delta / (float)L_PPR) * (60000.0f / dt);
    R_rpm = (R_delta / (float)R_PPR) * (60000.0f / dt);

    lastLogTime = now;
    Serial.printf("%lu,%d,%d,%.2f,%.2f\n", now, L_pwm_cmd, R_pwm_cmd, L_rpm, R_rpm);
  }

  if (now - lastStatTime >= STATUS_INTERVAL_MS) {
    publishStatus("tick");
    lastStatTime = now;
  }

  // áp lại PWM/DIR mỗi vòng
  applyMotorOutputs();

  // Nhả CPU một chút (an toàn WDT)
  delay(1);
}
