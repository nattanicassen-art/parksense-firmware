/*
  parking_node.ino  --  Node ที่ติดช่องจอดรถ
  ESP32-S3-WROOM-1 + TOF400C (VL53L1X) + RA-02 (SX1278, 433 MHz)

  พฤติกรรม
  1) หลับลึก ปล่อยให้เซนเซอร์วัดเองต่อเนื่อง (XSHUT ค้างสูง) และตั้งหน้าต่างระยะไว้ที่ตัวเซนเซอร์
  2) วัตถุเข้าใกล้กว่า DETECT_MM  -> ขา INT ปลุก ESP32
  3) ตื่นมาติดตามตำแหน่ง ถ้านิ่งนาน CONFIRM_MS -> ส่ง "ไม่ว่าง" ผ่าน LoRa
  4) วัตถุไม่ขยับครบ SLEEP_STILL_MS (15 วินาที) -> ตั้งหน้าต่างปลุกรอบตำแหน่งนิ่ง แล้วหลับลึก
  5) วัตถุขยับหลุดหน้าต่าง/ออกจากระยะ -> ตื่น ถ้าหายไปนาน CONFIRM_FREE_MS -> ส่ง "ว่าง" แล้วหลับ
  6) ตื่นทุก HEARTBEAT_S วินาทีส่ง heartbeat (แรงดันแบต + สถานะ)

  ไลบรารีที่ต้องติดตั้ง (Library Manager)
    - LoRa  (Sandeep Mistry)
    - SparkFun VL53L1X 4m Laser Distance Sensor
  Arduino IDE: Board = ESP32S3 Dev Module, USB CDC On Boot = Enabled
               (Node แนะนำปิด PSRAM เพื่อลดเวลาบูตและกินไฟ)

  หมายเหตุ: โค้ดนี้ยังไม่ได้คอมไพล์หรือทดสอบกับฮาร์ดแวร์จริง ชื่อฟังก์ชันของไลบรารี
  เขียนตามเวอร์ชันที่จำได้ ถ้าคอมไพล์ไม่ผ่านให้เทียบกับตัวอย่างในไลบรารี
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <SparkFun_VL53L1X.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>

// ======================= ค่าที่ปรับได้ =======================
#define NODE_ID        1          // ตั้งไม่ซ้ำกันในแต่ละช่องจอด
#define LORA_FREQ      433E6      // ตรวจกฎ กสทช. ก่อนใช้งานจริง
#define USE_TOF_INT    1          // 1 = ปลุกด้วยขา INT ของเซนเซอร์ (ต้องต่อสาย TOF_INT -> IO17)
                                  // 0 = ไม่ใช้ INT ตื่นตาม timer มาวัด (ทดสอบเท่านั้น กินไฟมาก)
#define USE_LED        1          // 1 = กระพริบ RGB LED (common anode, LOW = ติด)
#define DEBUG          1

constexpr uint16_t DETECT_MM       = 1000;   // วัตถุใกล้กว่านี้ = มีวัตถุ (โหมด Short ได้ไม่เกิน ~1300)
constexpr uint16_t LEAVE_HYST_MM   = 150;    // ต้องไกลกว่า DETECT+HYST ถึงนับว่าออกไป
constexpr uint16_t MOVE_TOL_MM     = 40;     // เปลี่ยนเกินค่านี้ = ขยับ
constexpr uint16_t WINDOW_MM       = 60;     // หน้าต่างปลุกรอบตำแหน่งนิ่ง (ตอนหลับ)
constexpr uint32_t CONFIRM_MS      = 3000;   // นิ่งนานเท่านี้ถึงรายงาน "ไม่ว่าง"
constexpr uint32_t CONFIRM_FREE_MS = 2000;   // หายไปนานเท่านี้ถึงรายงาน "ว่าง"
constexpr uint32_t SLEEP_STILL_MS  = 15000;  // นิ่งนานเท่านี้ -> หลับ
constexpr uint32_t SAMPLE_MS       = 200;    // ช่วงวัดตอนตื่น
constexpr uint16_t SLEEP_PERIOD_MS = 500;    // รอบวัดของเซนเซอร์ตอน ESP32 หลับ (ยาว = ประหยัด)
constexpr uint32_t HEARTBEAT_S     = 600;    // ตื่นส่ง heartbeat ทุกกี่วินาที
constexpr uint32_t POLL_S          = 2;      // ใช้เฉพาะ USE_TOF_INT = 0
constexpr uint32_t ACK_TIMEOUT_MS  = 700;
constexpr uint8_t  TX_RETRIES      = 3;

// ======================= ขา (ตรวจให้ตรงกับ schematic ล่าสุด) =======================
constexpr int PIN_SDA      = 8;    // I2C_SDA
constexpr int PIN_SCL      = 9;    // I2C_SCL
constexpr int PIN_XSHUT    = 2;    // TOF_XSHUT
constexpr int PIN_TOF_INT  = 17;   // ต้องเพิ่ม: ต่อ INT ของ TOF400C มาที่ IO17 (ขา RTC)
constexpr int PIN_VBAT_ADC = 1;    // VBAT_ADC (ตัวแบ่ง 1M/1M -> คูณ 2)
constexpr int LORA_SCK = 4, LORA_MISO = 5, LORA_MOSI = 6, LORA_NSS = 7;
constexpr int LORA_RST = 10, LORA_DIO0 = 16;
constexpr int LED_R = 38, LED_G = 39, LED_B = 40;

// ======================= แพ็กเก็ต =======================
enum : uint8_t { TYPE_STATE = 1, TYPE_HEARTBEAT = 2, TYPE_ACK = 3 };
constexpr uint16_t NO_TARGET = 0xFFFF;

struct __attribute__((packed)) Packet {
  uint8_t  magic;     // 0xA5
  uint8_t  type;
  uint8_t  nodeId;
  uint8_t  state;     // 0 = ว่าง, 1 = ไม่ว่าง
  uint16_t distMm;
  uint16_t battMv;
  uint16_t seq;
  uint8_t  crc;
};

uint8_t crc8(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (c << 1) ^ 0x07 : (c << 1);
  }
  return c;
}

// ======================= ข้อมูลที่อยู่ข้ามการหลับ (RTC) =======================
RTC_DATA_ATTR uint8_t  g_state       = 0;      // สถานะที่รายงานล่าสุด 0 = ว่าง, 1 = ไม่ว่าง
RTC_DATA_ATTR uint16_t g_rest        = 0;      // ระยะตอนนิ่ง
RTC_DATA_ATTR uint16_t g_seq         = 0;
RTC_DATA_ATTR bool     g_needResend  = false;  // ส่งสถานะครั้งก่อนไม่ได้รับ ACK
RTC_DATA_ATTR uint32_t g_hbElapsed   = 0;
RTC_DATA_ATTR uint32_t g_lastSleepS  = 0;
RTC_DATA_ATTR int      g_wakeLevel   = LOW;    // ระดับของขา INT ที่ใช้ปลุก

SFEVL53L1X tof;

#if DEBUG
  #define DBG(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG(...)
#endif

static const int HOLD_PINS[] = {PIN_XSHUT, LORA_NSS, LORA_RST, LED_R, LED_G, LED_B};

// ======================= LED =======================
void ledsOff() {
  digitalWrite(LED_R, HIGH); digitalWrite(LED_G, HIGH); digitalWrite(LED_B, HIGH);
}
void ledFlash(int pin, uint16_t ms = 120) {
#if USE_LED
  digitalWrite(pin, LOW); delay(ms); digitalWrite(pin, HIGH);
#endif
}

// ======================= แบตเตอรี่ =======================
uint16_t readBatteryMv() {
  analogSetPinAttenuation(PIN_VBAT_ADC, ADC_11db);
  analogReadMilliVolts(PIN_VBAT_ADC);          // อ่านทิ้ง 1 ครั้ง
  delay(5);
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) { sum += analogReadMilliVolts(PIN_VBAT_ADC); delay(2); }
  return (uint16_t)((sum / 8) * 2);            // ตัวแบ่ง 1M + 1M
}

// ======================= LoRa =======================
bool loraBegin() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) return false;
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x34);
  LoRa.setTxPower(14);
  LoRa.enableCrc();
  return true;
}

bool sendPacket(uint8_t type, uint8_t state, uint16_t dist) {
  Packet p{};
  p.magic = 0xA5; p.type = type; p.nodeId = NODE_ID; p.state = state;
  p.distMm = dist; p.battMv = readBatteryMv(); p.seq = ++g_seq;
  p.crc = crc8((uint8_t *)&p, sizeof(p) - 1);

  for (uint8_t attempt = 0; attempt < TX_RETRIES; attempt++) {
    LoRa.beginPacket();
    LoRa.write((uint8_t *)&p, sizeof(p));
    LoRa.endPacket();
    DBG("TX type=%u state=%u dist=%u batt=%u seq=%u try=%u\n", type, state, dist, p.battMv, p.seq, attempt + 1);

    uint32_t t0 = millis();
    while (millis() - t0 < ACK_TIMEOUT_MS) {
      int n = LoRa.parsePacket();
      if (n == (int)sizeof(Packet)) {
        Packet a{};
        LoRa.readBytes((uint8_t *)&a, sizeof(a));
        if (a.magic == 0xA5 && a.type == TYPE_ACK && a.nodeId == NODE_ID && a.seq == p.seq &&
            a.crc == crc8((uint8_t *)&a, sizeof(a) - 1)) {
          DBG("ACK ok\n");
          return true;
        }
      }
    }
  }
  DBG("ไม่ได้รับ ACK\n");
  return false;
}

void reportState(uint8_t state, uint16_t dist) {
  ledFlash(state ? LED_R : LED_G);
  g_needResend = !sendPacket(TYPE_STATE, state, dist);
}

void heartbeat(uint8_t state, uint16_t dist) {
  ledFlash(LED_B, 60);
  sendPacket(TYPE_HEARTBEAT, state, dist);
  g_hbElapsed = 0;
}

// ======================= เซนเซอร์ =======================
bool sensorStart() {
  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, LOW);  delay(5);     // รีเซ็ตเซนเซอร์ให้อยู่ในสถานะสะอาดทุกครั้ง
  digitalWrite(PIN_XSHUT, HIGH); delay(10);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  if (tof.begin() != 0) return false;          // SparkFun: คืน 0 เมื่อสำเร็จ
  tof.setDistanceModeShort();
  tof.setTimingBudgetInMs(50);
  tof.setIntermeasurementPeriod(SAMPLE_MS);
  tof.startRanging();
  return true;
}

uint16_t readDistance() {                      // NO_TARGET ถ้าไม่เจอ/ค่าไม่ถูกต้อง
  uint32_t t0 = millis();
  while (!tof.checkForDataReady()) {
    if (millis() - t0 > 500) return NO_TARGET;
    delay(2);
  }
  uint16_t d = tof.getDistance();
  uint8_t st = tof.getRangeStatus();
  tof.clearInterrupt();
  return (st == 0) ? d : NO_TARGET;
}

uint16_t readMedian3() {
  uint16_t v[3];
  for (int i = 0; i < 3; i++) v[i] = readDistance();
  if (v[0] > v[1]) { uint16_t t = v[0]; v[0] = v[1]; v[1] = t; }
  if (v[1] > v[2]) { uint16_t t = v[1]; v[1] = v[2]; v[2] = t; }
  if (v[0] > v[1]) { uint16_t t = v[0]; v[0] = v[1]; v[1] = t; }
  return v[1];
}

static inline bool nearObj(uint16_t d) { return d != NO_TARGET && d < DETECT_MM; }
static inline bool isGone(uint16_t d)  { return d == NO_TARGET || d > (uint16_t)(DETECT_MM + LEAVE_HYST_MM); }

// ตื่นอยู่และติดตามตำแหน่ง จนนิ่งครบ SLEEP_STILL_MS หรือสถานะเปลี่ยนเป็น "ว่าง"
void monitorLoop() {
  uint32_t lastMove = millis();
  uint32_t absentSince = 0;
  uint16_t rest = g_rest;
  bool first = true;

  for (;;) {
    uint16_t d = readDistance();
    uint32_t now = millis();
    DBG("d=%u rest=%u state=%u\n", d, rest, g_state);

    if (isGone(d)) {
      if (absentSince == 0) absentSince = now;
      if (g_state == 1 && now - absentSince >= CONFIRM_FREE_MS) {
        g_state = 0; g_rest = 0;
        reportState(0, (d == NO_TARGET) ? 0 : d);              // ว่างแล้ว
        return;
      }
      if (g_state == 0 && now - absentSince >= 1000) return;    // ผ่านไปเฉยๆ ยังไม่ถึงขั้นรายงาน
    } else {
      absentSince = 0;
      if (first || abs((int)d - (int)rest) > (int)MOVE_TOL_MM) { rest = d; lastMove = now; }
      first = false;

      if (g_state == 0 && now - lastMove >= CONFIRM_MS) {       // นิ่งพอ -> ไม่ว่าง
        g_state = 1; g_rest = rest;
        reportState(1, rest);
      }
      if (now - lastMove >= SLEEP_STILL_MS) {                   // นิ่งครบ 15 วินาที -> ไปหลับ
        g_rest = rest;
        return;
      }
    }
    delay(SAMPLE_MS);
  }
}

void runSession(bool timerWake, bool cold) {
  uint16_t d = readMedian3();
  if (timerWake) g_hbElapsed += g_lastSleepS;
  bool hbDue = (g_hbElapsed >= HEARTBEAT_S);
  DBG("session d=%u state=%u timer=%d cold=%d\n", d, g_state, timerWake, cold);

  if (g_needResend) reportState(g_state, g_state ? g_rest : (d == NO_TARGET ? 0 : d));

  if (g_state == 0) {
    if (nearObj(d)) monitorLoop();
    else if (cold || (timerWake && hbDue)) heartbeat(0, d == NO_TARGET ? 0 : d);
  } else {
    if (isGone(d) || abs((int)d - (int)g_rest) > (int)MOVE_TOL_MM) monitorLoop();
    else if (timerWake && hbDue) heartbeat(1, d);
  }
}

// ตั้งเซนเซอร์ให้วัดเองระหว่างหลับ คืน true ถ้า INT ยังเงียบ (หลับได้)
bool armSensor() {
#if USE_TOF_INT
  tof.stopRanging();
  tof.setIntermeasurementPeriod(SLEEP_PERIOD_MS);
  if (g_state == 0) {
    tof.setDistanceThreshold(DETECT_MM, DETECT_MM, 0, 0);              // ปลุกเมื่อ ระยะ < DETECT_MM
  } else {
    uint16_t lo = (g_rest > WINDOW_MM) ? g_rest - WINDOW_MM : 0;
    tof.setDistanceThreshold(lo, g_rest + WINDOW_MM, 2, 1);            // ปลุกเมื่อหลุดหน้าต่าง หรือไม่เจอวัตถุ
  }
  g_wakeLevel = tof.getInterruptPolarity() ? HIGH : LOW;               // ถ้าปลุกผิดระดับ ให้สลับบรรทัดนี้
  tof.clearInterrupt();
  tof.startRanging();
  delay(SLEEP_PERIOD_MS + 150);                                        // รอวัดอย่างน้อย 1 รอบ
  pinMode(PIN_TOF_INT, INPUT);
  return digitalRead(PIN_TOF_INT) != g_wakeLevel;
#else
  tof.stopRanging();
  digitalWrite(PIN_XSHUT, LOW);                                        // ปิดเซนเซอร์ระหว่างหลับ
  return true;
#endif
}

void goToSleep() {
  LoRa.sleep();
  digitalWrite(LORA_NSS, HIGH);
  digitalWrite(LORA_RST, HIGH);
  ledsOff();
#if USE_TOF_INT
  digitalWrite(PIN_XSHUT, HIGH);                                       // เซนเซอร์ต้องทำงานต่อระหว่างหลับ
  g_lastSleepS = HEARTBEAT_S;
#else
  digitalWrite(PIN_XSHUT, LOW);
  g_lastSleepS = POLL_S;
#endif
  for (int p : HOLD_PINS) gpio_hold_en((gpio_num_t)p);
  gpio_deep_sleep_hold_en();

#if USE_TOF_INT
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_TOF_INT, g_wakeLevel);
  if (g_wakeLevel == HIGH) { rtc_gpio_pulldown_en((gpio_num_t)PIN_TOF_INT); rtc_gpio_pullup_dis((gpio_num_t)PIN_TOF_INT); }
  else                     { rtc_gpio_pullup_en((gpio_num_t)PIN_TOF_INT);   rtc_gpio_pulldown_dis((gpio_num_t)PIN_TOF_INT); }
#endif
  esp_sleep_enable_timer_wakeup((uint64_t)g_lastSleepS * 1000000ULL);
  DBG("sleep state=%u rest=%u level=%d\n", g_state, g_rest, g_wakeLevel);
  Serial.flush();
  esp_deep_sleep_start();
}

void errorSleep() {
  for (int i = 0; i < 3; i++) { ledFlash(LED_R, 80); delay(80); }
  LoRa.sleep();
  esp_sleep_enable_timer_wakeup(60ULL * 1000000ULL);
  esp_deep_sleep_start();
}

void setup() {
  gpio_deep_sleep_hold_dis();
  for (int p : HOLD_PINS) gpio_hold_dis((gpio_num_t)p);
  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT); ledsOff();
  Serial.begin(115200);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool timerWake = (cause == ESP_SLEEP_WAKEUP_TIMER);
  bool cold      = (cause == ESP_SLEEP_WAKEUP_UNDEFINED);
#if USE_TOF_INT
  if (cause == ESP_SLEEP_WAKEUP_EXT0) rtc_gpio_deinit((gpio_num_t)PIN_TOF_INT);
#endif
  analogReadResolution(12);

  if (!loraBegin())    { DBG("LoRa เริ่มไม่ได้\n");    errorSleep(); }
  if (!sensorStart())  { DBG("เซนเซอร์เริ่มไม่ได้\n"); errorSleep(); }

  for (int round = 0; round < 4; round++) {
    runSession(timerWake, cold && round == 0);
    timerWake = false;
    if (armSensor()) break;                    // INT เงียบ -> หลับได้
    if (!sensorStart()) errorSleep();          // เงื่อนไขเกิดระหว่างรอ -> ประมวลผลรอบใหม่
  }
  goToSleep();
}

void loop() {}
