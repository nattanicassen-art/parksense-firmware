#include <SPI.h>
#include <LoRa.h>

// ================= 1. ตั้งค่าขาอุปกรณ์ฝั่ง NODE (ตรงตามผังวงจร) =================
#define NODE_ID        1

// เซนเซอร์ FC-51 (ต่อเข้าช่อง SCL = GPIO 9)
#define PIN_SENSOR     9
#define SENSOR_ACTIVE  LOW    // LOW = มีวัตถุ, HIGH = ไม่มีวัตถุ

// ขาวัดแรงดันแบตเตอรี่ VBAT_ADC (GPIO 1, วงจรแบ่งแรงดัน 1M / 1M -> คูณ 2.0)
#define VBAT_PIN       1

// ขาโมดูล LoRa RA-02 (SX1278 433MHz) ตามผังวงจร U1
#define LORA_SCK       4      // IO4
#define LORA_MISO      5      // IO5
#define LORA_MOSI      6      // IO6
#define LORA_CS        7      // IO7  (LORA_NSS)
#define LORA_RST       10     // IO10 (LORA_RST)
#define LORA_DIO0      16     // IO16 (LORA_DIO0)
#define LORA_BAND      433E6  // RA-02 ใช้ความถี่ 433 MHz
#define LORA_SYNC_WORD 0xF3   // รหัสจับคู่ให้ตรงกับฝั่ง Gateway

// หลอดไฟ RGB บนบอร์ด (Common Anode: LOW = ติด, HIGH = ดับ)
const int LED_R = 38;
const int LED_G = 39;
const int LED_B = 40;

// ตั้งค่าเวลา
const unsigned long HEARTBEAT_MS = 30000; // ส่งอัปเดตสถานะและแบตทุก 30 วินาที
const unsigned long DEBOUNCE_MS  = 50;    // กรองสัญญาณกระเพื่อม 50 มิลลิวินาที

int stableState = HIGH;
int lastReading = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long lastSendTime = 0;
bool firstBootSend = true;

// ================= 2. ฟังก์ชันไฟสถานะและแบตเตอรี่ =================
void setLed(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? LOW : HIGH);
  digitalWrite(LED_G, g ? LOW : HIGH);
  digitalWrite(LED_B, b ? LOW : HIGH);
}

// ฟังก์ชันอัปเดตสีไฟตามสถานะวัตถุ (มีวัตถุ = แดง, ไม่มีวัตถุ = เขียว)
void updateStatusLed(bool detected) {
  if (detected) {
    setLed(1, 0, 0); // สีแดงค้างเมื่อเจอวัตถุ
  } else {
    setLed(0, 1, 0); // สีเขียวค้างเมื่อไม่มีวัตถุ
  }
}

float readBatteryVoltage() {
  uint32_t sum_mv = 0;
  for (int i = 0; i < 16; i++) {
    sum_mv += analogReadMilliVolts(VBAT_PIN);
    delay(2);
  }
  float adc_v = (sum_mv / 16.0f) / 1000.0f;
  return adc_v * 2.0f; // คูณ 2 ตามตัวต้านทานแบ่งแรงดัน 1MΩ / 1MΩ
}

int voltageToPercent(float v) {
  if (v >= 4.18f) return 100;
  if (v <= 3.40f) return 0;
  if (v >= 3.95f) return map((int)(v * 100), 395, 418, 75, 100);
  if (v >= 3.80f) return map((int)(v * 100), 380, 395, 50, 75);
  if (v >= 3.65f) return map((int)(v * 100), 365, 380, 25, 50);
  return map((int)(v * 100), 340, 365, 0, 25);
}

// ================= 3. ฟังก์ชันส่งข้อมูลผ่าน LoRa =================
void sendLoRaPacket(bool detected) {
  float vbat = readBatteryVoltage();
  int batPercent = voltageToPercent(vbat);
  int objState = detected ? 1 : 0; // 1 = มีวัตถุ, 0 = ไม่มีวัตถุ

  String payload = "{\"node\":" + String(NODE_ID) +
                   ",\"obj\":" + String(objState) +
                   ",\"vbat\":" + String(vbat, 2) +
                   ",\"bat\":" + String(batPercent) + "}";

  Serial.println("[NODE] กำลังส่ง -> " + payload);
  Serial.flush();

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();

  // กะพริบไฟสีน้ำเงินสั้นๆ 1 ครั้งเพื่อยืนยันการส่ง แล้วคืนค่าสีแดง/เขียวตามสถานะ
  setLed(0, 0, 1);
  delay(80);
  updateStatusLed(detected);
}

// ================= 4. SETUP & LOOP =================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(PIN_SENSOR, INPUT);

  // กะพริบขาว 2 ครั้งตอนเปิดเครื่อง
  for (int i = 0; i < 2; i++) {
    setLed(1, 1, 1); delay(150);
    setLed(0, 0, 0); delay(150);
  }
  delay(1000);

  // เริ่มต้น SPI และ RA-02 ด้วยขาจริงตามผังวงจร
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("[ERROR] เริ่มต้น RA-02 ไม่สำเร็จ! ตรวจสอบไฟเลี้ยง 3.3V และรอยบัดกรี U2");
    while (1) {
      setLed(1, 0, 0); delay(300); // กะพริบแดงแจ้งเตือนเมื่อไม่พบโมดูล LoRa
      setLed(0, 0, 0); delay(300);
    }
  }

  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(17);

  stableState = digitalRead(PIN_SENSOR);
  lastReading = stableState;
  updateStatusLed(stableState == SENSOR_ACTIVE);

  Serial.println("[NODE] เริ่มทำงานพร้อมส่งข้อมูล (433 MHz)");
}

void loop() {
  int reading = digitalRead(PIN_SENSOR);

  if (reading != lastReading) {
    lastDebounceTime = millis();
  }

  bool stateChanged = false;
  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != stableState) {
      stableState = reading;
      stateChanged = true;
    }
  }
  lastReading = reading;

  bool detected = (stableState == SENSOR_ACTIVE);

  // แสดงไฟสีแดงค้างเมื่อเจอวัตถุ / สีเขียวค้างเมื่อไม่มีวัตถุ
  updateStatusLed(detected);

  // ส่งข้อมูลเมื่อ: เปิดเครื่องครั้งแรก, สถานะวัตถุเปลี่ยน, หรือครบทุก 30 วินาที
  if (firstBootSend || stateChanged || (millis() - lastSendTime >= HEARTBEAT_MS)) {
    firstBootSend = false;
    lastSendTime = millis();
    sendLoRaPacket(detected);
  }

  delay(10);
}