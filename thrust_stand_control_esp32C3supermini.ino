/*
  ESP32-C3 Super Mini
  ESC + 3x HX711 Load Cell
  Tu dong chay SWEEP roi STEP test khi khoi dong, khong can nhap lenh.
  Chi in du lieu tho ra Serial, moi xu ly/luu/ve do thi lam ben Python.
*/

#include <Arduino.h>
#include <ESP32Servo.h>
#include "HX711.h"

#define LC1_DOUT_PIN   1
#define LC1_SCK_PIN    5
#define LC2_DOUT_PIN   2
#define LC2_SCK_PIN    6
#define LC3_DOUT_PIN   3
#define LC3_SCK_PIN    7

HX711 loadcell1;
HX711 loadcell2;
HX711 loadcell3;

#define LC1_CAL_FACTOR   -407.19f
#define LC2_CAL_FACTOR    423.92f
#define LC3_CAL_FACTOR    426.47f

const float LC23_DISTANCE_M = 0.0673f;
const float LC23_RADIUS_M   = LC23_DISTANCE_M / 2.0f;

#define HX711_TIMEOUT_MS 150
#define HX711_PERIOD_MS  100

#define ESC_PIN 4
#define ESC_MIN_US 1000
#define ESC_MAX_US 2000

// ==== NHAP THU CONG DIEN AP PIN TRUOC KHI NAP CODE ====
const float BATTERY_VOLTAGE = 11.1f;   // <-- doi so nay truoc moi lan chay test

Servo esc;

// ==== Cau hinh cac lan STEP test se chay tu dong ====
// Moi dong: {throttle nen %, throttle dich %, so lan lap}
struct StepCfg { float base; float target; int repeats; };

StepCfg stepTests[] = {
  {50, 60, 20},
};
const int NUM_STEP_TESTS = sizeof(stepTests) / sizeof(stepTests[0]);


void loadCellsInit()
{
  loadcell1.begin(LC1_DOUT_PIN, LC1_SCK_PIN);
  loadcell2.begin(LC2_DOUT_PIN, LC2_SCK_PIN);
  loadcell3.begin(LC3_DOUT_PIN, LC3_SCK_PIN);

  loadcell1.set_scale(LC1_CAL_FACTOR);
  loadcell2.set_scale(LC2_CAL_FACTOR);
  loadcell3.set_scale(LC3_CAL_FACTOR);

  loadcell1.tare();
  loadcell2.tare();
  loadcell3.tare();
}

bool waitReady_ms(HX711 &lc, uint32_t timeout_ms)
{
  uint32_t start = millis();
  while (!lc.is_ready())
  {
    if (millis() - start >= timeout_ms) return false;
  }
  return true;
}

float readThrust_g(uint8_t samples = 1)
{
  if (!waitReady_ms(loadcell1, HX711_TIMEOUT_MS)) return NAN;
  return loadcell1.get_units(samples);
}

float readLC2_g(uint8_t samples = 1)
{
  if (!waitReady_ms(loadcell2, HX711_TIMEOUT_MS)) return NAN;
  return loadcell2.get_units(samples);
}

float readLC3_g(uint8_t samples = 1)
{
  if (!waitReady_ms(loadcell3, HX711_TIMEOUT_MS)) return NAN;
  return loadcell3.get_units(samples);
}

float readMoment_Nm()
{
  float f2_g = readLC2_g();
  float f3_g = readLC3_g();
  if (isnan(f2_g) || isnan(f3_g)) return NAN;

  float f2_N = f2_g * 0.00980665f;
  float f3_N = f3_g * 0.00980665f;

  return (f2_N + f3_N) * LC23_RADIUS_M;
}

void setThrottleUs(int us)
{
  us = constrain(us, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(us);
}

void setThrottlePercent(float percent)
{
  percent = constrain(percent, 0.0f, 100.0f);
  int us = ESC_MIN_US + (int)((ESC_MAX_US - ESC_MIN_US) * percent / 100.0f);
  setThrottleUs(us);
}


void runSweep()
{
  Serial.println("# BEGIN SWEEP");
  Serial.println("time_ms,voltage,throttle_pct,thrust_g,moment_Nm");

  for (int pct = 1; pct <= 100; pct++)
  {
    setThrottlePercent(pct);
    uint32_t stepStart = millis();

    while (millis() - stepStart < 500)
    {
      float thrust_g  = readThrust_g();
      float moment_Nm = readMoment_Nm();

      Serial.print(millis());
      Serial.print(",");
      Serial.print(BATTERY_VOLTAGE, 2);
      Serial.print(",");
      Serial.print(pct);
      Serial.print(",");
      if (isnan(thrust_g)) Serial.print("NAN"); else Serial.print(thrust_g, 2);
      Serial.print(",");
      if (isnan(moment_Nm)) Serial.print("NAN"); else Serial.print(moment_Nm, 4);
      Serial.println();
    }
  }

  setThrottleUs(1000);
  Serial.println("# END SWEEP");
}


void runEquivalentTimeStep(float basePct, float targetPct, int repeats, uint32_t captureMs = 1500)
{
  Serial.print("# BEGIN STEP base=");
  Serial.print(basePct, 1);
  Serial.print(" target=");
  Serial.print(targetPct, 1);
  Serial.print(" repeats=");
  Serial.println(repeats);

  Serial.println("repeat,t_since_step_ms,voltage,thrust_g");

  for (int i = 0; i < repeats; i++)
  {
    uint32_t iterStart = millis();   // moc dau lan lap, de tinh phan dem sau

    setThrottlePercent(basePct);
    delay(600);

    float phaseDelay = (float)i * (HX711_PERIOD_MS / (float)repeats);
    delay((uint32_t)phaseDelay);

    uint32_t t0 = micros();
    setThrottlePercent(targetPct);

    while ((micros() - t0) < (uint32_t)captureMs * 1000UL)
    {
      float thrust_g = readThrust_g();   // chi doc loadcell1, khong doc moment nua

      if (!isnan(thrust_g))
      {
        float t_since_ms = (micros() - t0) / 1000.0f;

        Serial.print(i);
        Serial.print(",");
        Serial.print(t_since_ms, 2);
        Serial.print(",");
        Serial.print(BATTERY_VOLTAGE, 2);
        Serial.print(",");
        Serial.println(thrust_g, 2);
      }
    }

    // dem them de tong thoi gian lan lap nay la boi so nguyen cua HX711_PERIOD_MS
    // => lan lap ke tiep luon bat dau dung pha 0, khong bi troi/cong don
    uint32_t elapsed = millis() - iterStart;
    uint32_t pad = HX711_PERIOD_MS - (elapsed % HX711_PERIOD_MS);
    delay(pad);
  }

  setThrottlePercent(basePct);
  Serial.println("# END STEP");
}

void setup()
{
  Serial.begin(115200);
  delay(5000);

  Serial.print("# VOLTAGE=");
  Serial.println(BATTERY_VOLTAGE, 2);

  Serial.println("# BEGIN AUTO TEST SEQUENCE");

  esc.setPeriodHertz(400);
  esc.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);

  setThrottleUs(1000);
  delay(3000);   // ARM ESC

  loadCellsInit();

  // ---- Chay tuan tu, tu dong, khong can nhap gi ----
  runSweep();
  delay(2000);   

  for (int i = 0; i < NUM_STEP_TESTS; i++)
  {
    runEquivalentTimeStep(stepTests[i].base, stepTests[i].target, stepTests[i].repeats);

    Serial.print("# STEP TEST ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(NUM_STEP_TESTS);
    Serial.println(" DONE, pausing...");
    delay(1500);   // nghi giua 2 test step
  }

  setThrottleUs(1000);
  Serial.println("# ALL TESTS DONE");
}


void loop()
{
  // Khong lam gi them - test da chay xong trong setup()
  delay(1000);
}