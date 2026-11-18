
#include <Wire.h>
#include <VL53L0X.h>
#include <DHT.h>

// =================================================
// PIN DEFINITIONS
// =================================================

#define LDR_PIN A0

#define DHT_PIN 4
#define DHT_TYPE DHT11

#define BUTTON_MODE 2
#define BUTTON_BEAM 3

#define MOTOR_ENA 5
#define LED_ENABLE 6
#define RELAY_PIN 7
#define MOTOR_ENB 9

#define MOTOR_IN1 10
#define MOTOR_IN2 11
#define MOTOR_IN3 12
#define MOTOR_IN4 13

// =================================================
// SENSOR OBJECTS
// =================================================

VL53L0X tof;
DHT dht(DHT_PIN, DHT_TYPE);

// =================================================
// MOTOR SETTINGS
// =================================================

const int minimumSpeed = 90;

// =================================================
// SPEED AND TTC SETTINGS
// =================================================

const float MY_SPEED_KMH = 1.0;
const float SPEED_LIMIT = 1.0;
const float PASSING_MARGIN = 0.5;

const uint16_t MAX_DISTANCE_MM = 2000;
const uint16_t MIN_DISTANCE_MM = 50;

const unsigned long SAMPLE_INTERVAL = 100;

// =================================================
// LDR SETTINGS
// =================================================

const bool LDR_REVERSED = true;
const int LDR_THRESHOLD = 500;

// =================================================
// FOG SETTINGS
// =================================================

const float FOG_TEMP_LIMIT = 10.0;
const float FOG_HUM_MIN = 80.0;

// =================================================
// ToF VARIABLES
// =================================================

float previousDistance = 0;
float currentDistance = 0;

float relativeVelocity = 0;
float ttc = 0;

unsigned long previousSampleTime = 0;

bool previousReadingValid = false;

int approachingCount = 0;

bool approaching = false;
bool lowBeamLatched = false;
bool releaseScheduled = false;

unsigned long releaseTime = 0;

// =================================================
// LDR / DHT VARIABLES
// =================================================

int ldrValue = 0;

float temperature = 0;
float humidity = 0;

bool isNight = false;
bool fogLikely = false;

unsigned long lastDHTTime = 0;
unsigned long lastPrintTime = 0;

// =================================================
// MODE VARIABLES
// =================================================

bool manualMode = false;
bool manualDipped = false;

// =================================================
// HEADLIGHT CONTROL
//
// D6 LOW  = LIGHT ON
// D6 HIGH = LIGHT OFF
//
// Relay LOW  = HIGH BEAM
// Relay HIGH = LOW BEAM
// =================================================

void highBeam()
{
  digitalWrite(LED_ENABLE, LOW);
  digitalWrite(RELAY_PIN, LOW);
}

void dippedBeam()
{
  digitalWrite(LED_ENABLE, LOW);
  digitalWrite(RELAY_PIN, HIGH);
}

void allLightOff()
{
  digitalWrite(LED_ENABLE, HIGH);
}

// =================================================
// MOTOR INITIALIZATION
// =================================================

void initializeMotors()
{
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);

  // Left motor FORWARD

  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);

  // Right motor FORWARD

  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);

  analogWrite(MOTOR_ENA, 0);
  analogWrite(MOTOR_ENB, 0);
}

// =================================================
// CONTINUOUS MOTOR FORWARD
// =================================================

void runMotorsForward()
{
  int motorSpeed = minimumSpeed;

  analogWrite(MOTOR_ENA, motorSpeed);
  analogWrite(MOTOR_ENB, motorSpeed);
}

// =================================================
// DHT11 READING
// =================================================

void readDHT()
{
  if (millis() - lastDHTTime < 2000)
    return;

  lastDHTTime = millis();

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t))
    return;

  humidity = h;
  temperature = t;

  // Fog condition

  if (temperature < FOG_TEMP_LIMIT &&
      humidity >= FOG_HUM_MIN)
  {
    fogLikely = true;
  }
  else
  {
    fogLikely = false;
  }
}

// =================================================
// ToF + RELATIVE SPEED + TTC
// =================================================

void updateToF()
{
  unsigned long now = millis();

  if (now - previousSampleTime < SAMPLE_INTERVAL)
    return;

  uint16_t distanceMM =
    tof.readRangeContinuousMillimeters();

  unsigned long measurementTime = millis();

  float dt =
    (measurementTime - previousSampleTime) / 1000.0;

  previousSampleTime = measurementTime;

  // Invalid reading

  if (tof.timeoutOccurred() ||
      distanceMM < MIN_DISTANCE_MM ||
      distanceMM > MAX_DISTANCE_MM)
  {
    previousReadingValid = false;
    approachingCount = 0;
    approaching = false;

    relativeVelocity = 0;
    ttc = 0;

    // Existing passing timer is not cancelled.

    return;
  }

  currentDistance = distanceMM / 1000.0;

  // First valid reading

  if (!previousReadingValid)
  {
    previousDistance = currentDistance;
    previousReadingValid = true;

    relativeVelocity = 0;
    ttc = 0;

    approachingCount = 0;
    approaching = false;

    return;
  }

  if (dt <= 0)
    return;

  // Relative closing speed

  float rawVelocity =
    ((previousDistance - currentDistance) / dt) * 3.6;

  // Speed smoothing

  relativeVelocity =
    (0.5 * rawVelocity) +
    (0.5 * relativeVelocity);

  // Approaching detection

  if (relativeVelocity > SPEED_LIMIT)
  {
    approachingCount++;
  }
  else
  {
    approachingCount = 0;
  }

  approaching = (approachingCount >= 2);

  // TTC calculation

  if (relativeVelocity > 0)
  {
    ttc =
      currentDistance / (relativeVelocity / 3.6);
  }
  else
  {
    ttc = 0;
  }

  // Confirm approach and start DIP

  if (approaching &&
      !lowBeamLatched &&
      !releaseScheduled)
  {
    lowBeamLatched = true;

    unsigned long passingDuration =
      (unsigned long)((ttc + PASSING_MARGIN) * 1000.0);

    releaseTime =
      measurementTime + passingDuration;

    releaseScheduled = true;

    Serial.println("APPROACHING VEHICLE - DIP");
    Serial.println("TTC TIMER STARTED");
  }

  previousDistance = currentDistance;
}

// =================================================
// TTC PASSING TIMER
// =================================================

void checkPassingTime()
{
  if (releaseScheduled &&
      (long)(millis() - releaseTime) >= 0)
  {
    lowBeamLatched = false;
    releaseScheduled = false;

    approachingCount = 0;
    approaching = false;

    relativeVelocity = 0;
    ttc = 0;

    previousReadingValid = false;

    Serial.println("PASSING COMPLETE");
    Serial.println("RESTORING HIGH BEAM");
  }

  if (lowBeamLatched)
  {
    dippedBeam();
  }
}

// =================================================
// BUTTON CONTROL
// =================================================

void handleButtons()
{
  static bool lastModeReading = HIGH;
  static bool stableModeState = HIGH;
  static unsigned long modeChangedAt = 0;

  static bool lastBeamReading = HIGH;
  static bool stableBeamState = HIGH;
  static unsigned long beamChangedAt = 0;

  const unsigned long debounceMs = 50;

  unsigned long now = millis();

  // BUTTON 1: AUTO / MANUAL

  bool modeReading = digitalRead(BUTTON_MODE);

  if (modeReading != lastModeReading)
  {
    lastModeReading = modeReading;
    modeChangedAt = now;
  }

  if ((now - modeChangedAt) >= debounceMs &&
      modeReading != stableModeState)
  {
    stableModeState = modeReading;

    if (stableModeState == LOW)
    {
      manualMode = !manualMode;
      manualDipped = false;

      if (manualMode)
      {
        highBeam();
        Serial.println("MANUAL MODE");
      }
      else
      {
        Serial.println("AUTO MODE");
      }
    }
  }

  // BUTTON 2: HIGH / DIPPED
  // MANUAL MODE ONLY

  bool beamReading = digitalRead(BUTTON_BEAM);

  if (beamReading != lastBeamReading)
  {
    lastBeamReading = beamReading;
    beamChangedAt = now;
  }

  if ((now - beamChangedAt) >= debounceMs &&
      beamReading != stableBeamState)
  {
    stableBeamState = beamReading;

    if (stableBeamState == LOW && manualMode)
    {
      manualDipped = !manualDipped;

      if (manualDipped)
      {
        dippedBeam();
        Serial.println("MANUAL DIPPED");
      }
      else
      {
        highBeam();
        Serial.println("MANUAL HIGH");
      }
    }
  }
}

// =================================================
// AUTOMATIC HEADLIGHT CONTROL
// =================================================

void automaticHeadlightControl()
{
  // Manual mode

  if (manualMode)
  {
    if (manualDipped)
      dippedBeam();
    else
      highBeam();

    return;
  }

  // Fog override

  if (fogLikely)
  {
    dippedBeam();
    return;
  }

  // Day

  if (!isNight)
  {
    allLightOff();
    return;
  }

  // Night + approaching / passing

  if (lowBeamLatched)
  {
    dippedBeam();
    return;
  }

  // Normal night

  highBeam();
}

// =================================================
// SETUP
// =================================================

void setup()
{
  Serial.begin(9600);

  Wire.begin();

  pinMode(LDR_PIN, INPUT);

  pinMode(BUTTON_MODE, INPUT_PULLUP);
  pinMode(BUTTON_BEAM, INPUT_PULLUP);

  pinMode(LED_ENABLE, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  // Initialize motors

  initializeMotors();

  // Start DHT

  dht.begin();

  allLightOff();

  // Initialize ToF

  tof.setTimeout(500);

  Serial.println();
  Serial.println("================================");
  Serial.println("SMART HEADLIGHT PROJECT");
  Serial.println("================================");

  if (!tof.init())
  {
    Serial.println("VL53L0X INIT FAILED!");

    while (1)
    {
      highBeam();
      runMotorsForward();
      delay(100);
    }
  }

  tof.startContinuous(SAMPLE_INTERVAL);

  previousSampleTime = millis();

  Serial.println("VL53L0X FOUND");
  Serial.println("MOTORS INITIALIZED");
  Serial.println("LEFT MOTOR FORWARD");
  Serial.println("RIGHT MOTOR FORWARD");
  Serial.println("MOTOR PWM = 90");
  Serial.println("OWN SPEED = 1 KM/H");
  Serial.println("SPEED LIMIT = 1 KM/H");
  Serial.println("TTC PASSING TIMER ENABLED");
  Serial.println("AUTO MODE");
  Serial.println("SYSTEM READY");
}

// =================================================
// MAIN LOOP
// =================================================

void loop()
{
  // BUTTONS

  handleButtons();

  // MOTORS ALWAYS FORWARD

  runMotorsForward();

  // LDR

  ldrValue = analogRead(LDR_PIN);

  if (LDR_REVERSED)
  {
    isNight = (ldrValue > LDR_THRESHOLD);
  }
  else
  {
    isNight = (ldrValue < LDR_THRESHOLD);
  }

  // DHT

  readDHT();

  // ToF + SPEED + TTC

  updateToF();

  // Passing timer

  checkPassingTime();

  // Headlight control

  automaticHeadlightControl();

  // Serial Monitor

  if (millis() - lastPrintTime >= 1000)
  {
    lastPrintTime = millis();

    Serial.println("--------------------------------");

    Serial.print("MODE: ");
    Serial.println(manualMode ? "MANUAL" : "AUTO");

    Serial.print("MOTOR SPEED PWM: ");
    Serial.println(minimumSpeed);

    Serial.print("LDR: ");
    Serial.println(ldrValue);

    Serial.print("LIGHT CONDITION: ");
    Serial.println(isNight ? "NIGHT" : "DAY");

    Serial.print("DISTANCE: ");

    if (currentDistance > 0)
    {
      Serial.print(currentDistance);
      Serial.println(" m");
    }
    else
    {
      Serial.println("NO VALID OBJECT");
    }

    Serial.print("OWN SPEED: ");
    Serial.print(MY_SPEED_KMH);
    Serial.println(" km/h");

    Serial.print("RELATIVE SPEED: ");
    Serial.print(relativeVelocity);
    Serial.println(" km/h");

    Serial.print("TTC: ");
    Serial.print(ttc);
    Serial.println(" seconds");

    Serial.print("APPROACHING COUNT: ");
    Serial.println(approachingCount);

    Serial.print("APPROACHING: ");
    Serial.println(approaching ? "YES" : "NO");

    Serial.print("LOW BEAM LATCHED: ");
    Serial.println(lowBeamLatched ? "YES" : "NO");

    Serial.print("RELEASE SCHEDULED: ");
    Serial.println(releaseScheduled ? "YES" : "NO");

    Serial.print("TEMPERATURE: ");
    Serial.print(temperature);
    Serial.println(" C");

    Serial.print("HUMIDITY: ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("FOG: ");
    Serial.println(fogLikely ? "YES" : "NO");

    Serial.println("--------------------------------");
  }

  delay(20);
}
