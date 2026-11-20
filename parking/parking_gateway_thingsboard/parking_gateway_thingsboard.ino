
/*
 * Parking LoRa Gateway -> ThingsBoard (ESP32-S3 + Ra-02 + WiFi)
 *
 * - รับข้อความจากโหนดทาง LoRa  "P01,OCC,<mm>,<seq>" / "P01,FREE,<mm>,<seq>"
 * - ตอบ ACK ให้โหนดทันที (ไม่รอ WiFi/MQTT)
 * - ส่งขึ้น ThingsBoard ด้วย Gateway MQTT API
 *     แต่ละช่องจอดจะถูกสร้างเป็น "อุปกรณ์แยก" อัตโนมัติ (ชื่อ = DEVICE_PREFIX + NODE_ID เช่น Parking-P01)
 *     telemetry ของแต่ละช่อง: occupied (1/0), distance (mm), rssi
 *     telemetry ของตัว Gateway เอง: occupied_total, free_total
 * - ใช้เวลาจาก NTP ติดไปกับข้อมูล (ts) ถ้า WiFi หลุดแล้วส่งย้อนหลัง เวลาก็ยังตรงกับเหตุการณ์จริง
 *
 * ไลบรารี: "LoRa" by Sandeep Mistry, "PubSubClient" by Nick O'Leary
 *
 * ตั้งค่าใน ThingsBoard: สร้าง Device 1 ตัวแล้วติ๊ก "Is gateway" แล้วคัดลอก Access token มาใส่ TB_TOKEN
 */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>

// ================= ตั้งค่า =================
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ThingsBoard Cloud: mqtt.thingsboard.cloud  (โซนยุโรป eu.thingsboard.cloud ใช้ mqtt.eu.thingsboard.cloud)
#define TB_HOST       "mqtt.thingsboard.cloud"
#define TB_PORT       1883
#define TB_TOKEN      "u9c86lIG9pXixsiBuiis"

#define DEVICE_PREFIX "Parking-"        // ชื่ออุปกรณ์ใน ThingsBoard = DEVICE_PREFIX + NODE_ID
#define DEVICE_TYPE   "parking_slot"    // ใช้ตัวกรองตอนทำ dashboard

// LoRa (ESP32-S3)
#define PIN_LORA_SCK     4
#define PIN_LORA_MISO    5
#define PIN_LORA_MOSI    6
#define PIN_LORA_CS      7    // NSS
#define PIN_LORA_RST     10
#define PIN_LORA_DIO0    16
#define LORA_FREQ        433E6   // ต้องตรงกับโหนด
#define LORA_SYNC_WORD   0x34

#define MAX_SLOTS        32
// ===========================================

struct Slot {
  String   id;
  bool     occupied;
  uint16_t dist;
  int      rssi;
  uint64_t ts;           // เวลา UTC (ms) ตอนรับข้อมูล, 0 = ยังไม่รู้เวลา
  bool     pending;      // ยังไม่ได้ส่งขึ้น ThingsBoard
  bool     announced;    // ส่ง gateway/connect ใน session MQTT นี้แล้วหรือยัง
};
Slot slots[MAX_SLOTS];
int  slotCount = 0;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
uint32_t lastWifiTry = 0, lastMqttTry = 0;
bool wasConnected = false;

// ---------- เวลา ----------
uint64_t nowMs() {
  time_t t = time(nullptr);
  if (t < 1700000000) return 0;               // NTP ยังไม่ซิงค์
  return (uint64_t)t * 1000ULL;
}

Slot* findSlot(const String &id) {
  for (int i = 0; i < slotCount; i++) if (slots[i].id == id) return &slots[i];
  if (slotCount < MAX_SLOTS) {
    slots[slotCount] = {id, false, 0, 0, 0, false, false};
    return &slots[slotCount++];
  }
  return nullptr;
}

void sendAck(const String &id, const String &seq) {
  delay(50);                                   // ให้โหนดสลับเป็นโหมดรับก่อน
  LoRa.beginPacket();
  LoRa.print("ACK," + id + "," + seq);
  LoRa.endPacket();
}

// ---------- ส่งขึ้น ThingsBoard ----------
bool announceSlot(Slot &s) {
  char buf[128];
  snprintf(buf, sizeof(buf), "{\"device\":\"%s%s\",\"type\":\"%s\"}",
           DEVICE_PREFIX, s.id.c_str(), DEVICE_TYPE);
  return mqtt.publish("v1/gateway/connect", buf);
}

bool publishSlot(Slot &s) {
  char buf[256];
  if (s.ts) {
    snprintf(buf, sizeof(buf),
      "{\"%s%s\":[{\"ts\":%llu,\"values\":{\"occupied\":%d,\"distance\":%u,\"rssi\":%d}}]}",
      DEVICE_PREFIX, s.id.c_str(), (unsigned long long)s.ts,
      s.occupied ? 1 : 0, s.dist, s.rssi);
  } else {
    snprintf(buf, sizeof(buf),
      "{\"%s%s\":[{\"occupied\":%d,\"distance\":%u,\"rssi\":%d}]}",
      DEVICE_PREFIX, s.id.c_str(), s.occupied ? 1 : 0, s.dist, s.rssi);
  }
  bool ok = mqtt.publish("v1/gateway/telemetry", buf);
  Serial.println(String(ok ? "TB <- " : "TB fail: ") + buf);
  return ok;
}

void publishTotals() {
  int occ = 0, fr = 0;
  for (int i = 0; i < slotCount; i++) (slots[i].occupied ? occ : fr)++;
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"occupied_total\":%d,\"free_total\":%d}", occ, fr);
  mqtt.publish("v1/devices/me/telemetry", buf);   // telemetry ของตัว gateway เอง
}

void flushPending() {
  if (!mqtt.connected()) return;
  bool any = false;
  for (int i = 0; i < slotCount; i++) {
    Slot &s = slots[i];
    if (!s.pending) continue;
    if (!s.announced) {
      if (!announceSlot(s)) continue;
      s.announced = true;
    }
    if (publishSlot(s)) { s.pending = false; any = true; }
  }
  if (any) publishTotals();
}

void maintainConnections() {
  uint32_t now = millis();

  bool connected = mqtt.connected();
  if (wasConnected && !connected) {              // เพิ่งหลุด ต้อง announce ใหม่ตอนต่อกลับมา
    for (int i = 0; i < slotCount; i++) slots[i].announced = false;
  }
  wasConnected = connected;

  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiTry > 10000) { lastWifiTry = now; WiFi.reconnect(); }
    return;
  }
  if (!connected && now - lastMqttTry > 5000) {
    lastMqttTry = now;
    String cid = "parking-gw-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.print("MQTT connecting... ");
    if (mqtt.connect(cid.c_str(), TB_TOKEN, NULL)) {
      Serial.println("ok");
      for (int i = 0; i < slotCount; i++) slots[i].pending = true;   // sync สถานะล่าสุดของทุกช่อง
    } else {
      Serial.printf("fail rc=%d\n", mqtt.state());
    }
  }
}

// ---------- รับ LoRa ----------
void handleLora() {
  if (!LoRa.parsePacket()) return;

  String msg;
  while (LoRa.available()) msg += (char)LoRa.read();
  int rssi = LoRa.packetRssi();

  int p1 = msg.indexOf(',');
  int p2 = msg.indexOf(',', p1 + 1);
  int p3 = msg.indexOf(',', p2 + 1);
  if (p1 < 0 || p2 < 0 || p3 < 0) return;

  String   id    = msg.substring(0, p1);
  String   state = msg.substring(p1 + 1, p2);
  uint16_t dist  = msg.substring(p2 + 1, p3).toInt();
  String   seq   = msg.substring(p3 + 1);
  if (state != "OCC" && state != "FREE") return;

  sendAck(id, seq);                            // ตอบโหนดก่อนเสมอ

  Slot* s = findSlot(id);
  if (!s) return;
  s->occupied = (state == "OCC");
  s->dist     = dist;
  s->rssi     = rssi;
  s->ts       = nowMs();
  s->pending  = true;

  Serial.printf("[%s] %s dist=%u RSSI=%d seq=%s\n", id.c_str(),
                s->occupied ? "OCC" : "FREE", dist, rssi, seq.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(300);

  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed");
    while (true) delay(1000);
  }
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  configTime(0, 0, "pool.ntp.org", "time.google.com");   // เวลา UTC สำหรับ timestamp

  mqtt.setServer(TB_HOST, TB_PORT);
  mqtt.setBufferSize(512);

  Serial.println("Gateway ready");
}

void loop() {
  handleLora();            // รับ LoRa ก่อนเสมอ
  maintainConnections();
  mqtt.loop();
  flushPending();
}
