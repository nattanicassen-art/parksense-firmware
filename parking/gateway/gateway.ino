#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <LoRa.h>

// ================= 1. ตั้งค่า Wi-Fi และ ThingsBoard (แก้ตรงนี้) =================
const char* WIFI_SSID       = "nom priao";
const char* WIFI_PASSWORD   = "Dam_3749";

// URL ของ ThingsBoard (เลือกใช้ตามเซิร์ฟเวอร์ที่คุณสมัครไว้)
// - ThingsBoard Cloud : "https://thingsboard.cloud"
// - Demo Server       : "http://demo.thingsboard.io"
// - Local Server      : "http://192.168.1.xxx:8080"
const char* TB_SERVER       = "https://thingsboard.cloud";
const char* TB_ACCESS_TOKEN = "u9c86lIG9pXixsiBuiis";

// ================= 2. ตั้งค่าขาอุปกรณ์ฝั่ง GATEWAY =================
#define LORA_SCK       12
#define LORA_MISO      13
#define LORA_MOSI      11
#define LORA_CS        10     // NSS
#define LORA_RST       14     // ขา RESET ของ RA-02 (แก้เลขให้ตรงกับสายที่ต่อจริง)
#define LORA_DIO0      15     // DIO0
#define LORA_BAND      433E6  // ความถี่ 433 MHz ตรงกับฝั่ง Node
#define LORA_SYNC_WORD 0xF3   // รหัสจับคู่ตรงกับฝั่ง Node

// หลอดไฟ RGB บนบอร์ด ESP32-S3 (Common Anode: LOW = ติด, HIGH = ดับ)
const int LED_R = 38;
const int LED_G = 39;
const int LED_B = 40;

int lastObjState = -1;
unsigned long lastWifiCheck = 0;

// ================= 3. ฟังก์ชันไฟสถานะและจัดการข้อความ =================
void setLed(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? LOW : HIGH);
  digitalWrite(LED_G, g ? LOW : HIGH);
  digitalWrite(LED_B, b ? LOW : HIGH);
}

void updateStatusLed(int objState) {
  if (objState == 1) {
    setLed(1, 0, 0); // สีแดงค้างเมื่อ Node แจ้งว่ามีวัตถุ
  } else if (objState == 0) {
    setLed(0, 1, 0); // สีเขียวค้างเมื่อ Node แจ้งว่าไม่มีวัตถุ
  } else {
    setLed(0, 0, 0);
  }
}

String extractJsonValue(const String& json, const String& key) {
  String pattern = "\"" + key + "\":";
  int startIdx = json.indexOf(pattern);
  if (startIdx == -1) return "-";
  startIdx += pattern.length();
  int endIdx = json.indexOf(',', startIdx);
  if (endIdx == -1) endIdx = json.indexOf('}', startIdx);
  if (endIdx == -1) return "-";
  return json.substring(startIdx, endIdx);
}

// ================= 4. ฟังก์ชันเชื่อมต่อ Wi-Fi และส่งขึ้น ThingsBoard =================
void connectWiFi() {
  Serial.printf("[WIFI] กำลังเชื่อมต่อ %s ", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 30) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] เชื่อมต่อสำเร็จ! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WIFI ERROR] เชื่อมต่อไม่สำเร็จ (ตรวจสอบชื่อ/รหัสผ่าน และต้องเป็นคลื่น 2.4GHz)");
  }
}

void sendToThingsBoard(const String& jsonPayload) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] ไม่ได้เชื่อมต่อ Wi-Fi ข้ามการส่งขึ้น ThingsBoard");
    return;
  }

  String url = String(TB_SERVER) + "/api/v1/" + String(TB_ACCESS_TOKEN) + "/telemetry";
  HTTPClient http;
  WiFiClientSecure secureClient;
  WiFiClient plainClient;

  bool begun = false;
  if (url.startsWith("https://")) {
    secureClient.setInsecure(); // รองรับ HTTPS โดยไม่ต้องลงไฟล์ SSL Certificate
    begun = http.begin(secureClient, url);
  } else {
    begun = http.begin(plainClient, url);
  }

  if (begun) {
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.POST(jsonPayload);

    if (httpCode == 200) {
      Serial.println("[ThingsBoard] ส่งข้อมูลขึ้นเว็บสำเร็จ! (HTTP 200)");
    } else {
      Serial.printf("[ThingsBoard ERROR] ส่งไม่สำเร็จ รหัส HTTP: %d (%s)\n",
                    httpCode, http.errorToString(httpCode).c_str());
    }
    http.end();
  } else {
    Serial.println("[ThingsBoard ERROR] รูปแบบ URL เซิร์ฟเวอร์ไม่ถูกต้อง");
  }
}

// ================= 5. SETUP & LOOP =================
void setup() {
  Serial.begin(115200);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  // กะพริบขาว 2 ครั้งตอนเปิดเครื่อง
  for (int i = 0; i < 2; i++) {
    setLed(1, 1, 1); delay(150);
    setLed(0, 0, 0); delay(150);
  }
  delay(1000);

  // 1. เชื่อมต่อ Wi-Fi
  connectWiFi();

  // 2. เริ่มต้นโมดูล LoRa RA-02
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("[GATEWAY ERROR] เริ่มต้น RA-02 ไม่สำเร็จ! เช็กสาย NSS / RST / SPI");
    while (1) {
      setLed(1, 0, 0); delay(300);
      setLed(0, 0, 0); delay(300);
    }
  }

  LoRa.setSyncWord(LORA_SYNC_WORD);

  Serial.println("==========================================");
  Serial.println("[GATEWAY] พร้อมรับข้อมูล LoRa และส่งขึ้น ThingsBoard...");
  Serial.println("==========================================");
}

void loop() {
  // ตรวจสอบและต่อ Wi-Fi ใหม่อัตโนมัติทุก 15 วินาทีหากเน็ตหลุด
  if (WiFi.status() != WL_CONNECTED && (millis() - lastWifiCheck >= 15000)) {
    lastWifiCheck = millis();
    Serial.println("[WIFI] สัญญาณหลุด กำลังเชื่อมต่อใหม่...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  // รอรับแพ็กเก็ตจาก Node
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    String nodeId = extractJsonValue(incoming, "node");
    String obj    = extractJsonValue(incoming, "obj");
    String vbat   = extractJsonValue(incoming, "vbat");
    String bat    = extractJsonValue(incoming, "bat");

    lastObjState = obj.toInt();
    String objText = (lastObjState == 1) ? "มีวัตถุ (DETECTED)" : "ไม่มีวัตถุ (CLEAR)";

    // สร้าง JSON ฉบับสมบูรณ์ (รวมค่าจาก Node + ค่าความแรงสัญญาณจาก Gateway)
    String tbPayload = "{\"node\":" + nodeId +
                       ",\"obj\":" + obj +
                       ",\"vbat\":" + vbat +
                       ",\"bat\":" + bat +
                       ",\"rssi\":" + String(rssi) +
                       ",\"snr\":" + String(snr, 1) + "}";

    Serial.println("------------------------------------------");
    Serial.printf("Node #%s | สถานะ: %s | แบตเตอรี่: %s%% (%sV)\n",
                  nodeId.c_str(), objText.c_str(), bat.c_str(), vbat.c_str());
    Serial.printf("สัญญาณ: RSSI = %d dBm | SNR = %.1f dB\n", rssi, snr);

    // กะพริบสีน้ำเงินสั้นๆ แล้วแสดงไฟแดง/เขียวตามสถานะ
    setLed(0, 0, 1);
    delay(80);
    updateStatusLed(lastObjState);

    // ส่งข้อมูลขึ้นหน้าเว็บ ThingsBoard
    sendToThingsBoard(tbPayload);
    Serial.flush();
  }
}