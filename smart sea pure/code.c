#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ─── Pin Definitions ────────────────────────────────────────
#define TDS_PIN           A0
#define WATER_LEVEL_PIN   A1
#define PUMP_RELAY_PIN    7
#define SERVO_PIN         9

// ─── Thresholds & Constants ──────────────────────────────────
#define TDS_THRESHOLD        500   // ppm — above this, brine valve opens
#define WATER_LEVEL_LOW      300   // analog — below this, pump turns ON
#define WATER_LEVEL_HIGH     700   // analog — above this, pump turns OFF
#define BRINE_VALVE_OPEN     90    // servo angle for open valve
#define BRINE_VALVE_CLOSED   0     // servo angle for closed valve
#define VREF                 5.0
#define SCOUNT               30    // samples for TDS averaging

// ─── Objects ────────────────────────────────────────────────
Servo brineValve;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ─── Global Variables ────────────────────────────────────────
int analogBuffer[SCOUNT];
int analogBufferIndex = 0;
float tdsValue        = 0;
float waterLevelValue = 0;
bool  pumpState       = false;
bool  valveState      = false;

unsigned long lastReadTime  = 0;
unsigned long lastPrintTime = 0;

// ─── Median Filter ───────────────────────────────────────────
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (int i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];
  for (int j = 0; j < iFilterLen - 1; j++) {
    for (int i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        int temp    = bTab[i];
        bTab[i]     = bTab[i + 1];
        bTab[i + 1] = temp;
      }
    }
  }
  return (iFilterLen & 1)
    ? bTab[(iFilterLen - 1) / 2]
    : (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
}

// ─── Read TDS ────────────────────────────────────────────────
float readTDS() {
  int median      = getMedianNum(analogBuffer, SCOUNT);
  float voltage   = median * VREF / 1024.0;
  float compCoeff = 1.0 + 0.02 * (25.0 - 25.0); // 25°C assumed
  float compVolt  = voltage / compCoeff;
  float tds = (133.42 * pow(compVolt, 3)
             - 255.86 * pow(compVolt, 2)
             + 857.39 * compVolt) * 0.5;
  return tds;
}

// ─── Read Water Level ────────────────────────────────────────
float readWaterLevel() {
  return analogRead(WATER_LEVEL_PIN);
}

// ─── Control Pump ────────────────────────────────────────────
void controlPump(float level) {
  if (level < WATER_LEVEL_LOW && !pumpState) {
    digitalWrite(PUMP_RELAY_PIN, LOW);  // Active LOW relay
    pumpState = true;
    Serial.println("[PUMP] ON  — Water level low, filling chamber.");
  } else if (level > WATER_LEVEL_HIGH && pumpState) {
    digitalWrite(PUMP_RELAY_PIN, HIGH);
    pumpState = false;
    Serial.println("[PUMP] OFF — Optimal water level reached.");
  }
}

// ─── Control Brine Valve ─────────────────────────────────────
void controlBrineValve(float tds) {
  if (tds > TDS_THRESHOLD && !valveState) {
    brineValve.write(BRINE_VALVE_OPEN);
    valveState = true;
    Serial.print("[VALVE] OPEN  — TDS: "); Serial.print(tds); Serial.println(" ppm");
  } else if (tds <= TDS_THRESHOLD && valveState) {
    brineValve.write(BRINE_VALVE_CLOSED);
    valveState = false;
    Serial.print("[VALVE] CLOSED — TDS: "); Serial.print(tds); Serial.println(" ppm");
  }
}

// ─── Update LCD ──────────────────────────────────────────────
void updateLCD(float tds, float level) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TDS:");
  lcd.print((int)tds);
  lcd.print("ppm ");
  lcd.print(valveState ? "V:OPEN" : "V:CLSD");
  lcd.setCursor(0, 1);
  lcd.print("LVL:");
  lcd.print((int)level);
  lcd.print(" ");
  lcd.print(pumpState ? "PUMP:ON " : "PUMP:OFF");
}

// ─── Serial Status ───────────────────────────────────────────
void printStatus(float tds, float level) {
  Serial.println("=============================");
  Serial.print("  TDS Value    : "); Serial.print(tds);   Serial.println(" ppm");
  Serial.print("  Water Level  : "); Serial.print(level); Serial.println(" (raw)");
  Serial.print("  Pump         : "); Serial.println(pumpState  ? "ON"   : "OFF");
  Serial.print("  Brine Valve  : "); Serial.println(valveState ? "OPEN" : "CLOSED");
  Serial.println("=============================");
}

// ─── SETUP ───────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  pinMode(PUMP_RELAY_PIN, OUTPUT);
  digitalWrite(PUMP_RELAY_PIN, HIGH); // Pump OFF initially

  brineValve.attach(SERVO_PIN);
  brineValve.write(BRINE_VALVE_CLOSED);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("SmartSeaPure");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();

  Serial.println("==============================");
  Serial.println("  SmartSeaPure System Ready   ");
  Serial.println("  Solar Desalination Unit v1  ");
  Serial.println("==============================");
}

// ─── LOOP ────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // Continuously sample TDS every 40ms for averaging
  if (now - lastReadTime >= 40) {
    lastReadTime = now;
    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);
    analogBufferIndex = (analogBufferIndex + 1) % SCOUNT;
  }

  // Every 1 second: process and act
  if (now - lastPrintTime >= 1000) {
    lastPrintTime = now;

    tdsValue        = readTDS();
    waterLevelValue = readWaterLevel();

    controlPump(waterLevelValue);
    controlBrineValve(tdsValue);
    updateLCD(tdsValue, waterLevelValue);
    printStatus(tdsValue, waterLevelValue);
  }
}