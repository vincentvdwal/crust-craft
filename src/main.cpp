#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"
#include <ArduinoJson.h>
#include "max6675.h"
#include <ElegantOTA.h>

#include "secrets.h"
#include "pid.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Pin assignments
constexpr int relay = 21;
constexpr int thermoDO = 19;
constexpr int thermoCS = 23;
constexpr int thermoCLK = 5;

// Timer variables
unsigned long lastSensor = 0;
const unsigned long SENSOR_INTERVAL = 100;

unsigned long lastWebSocket = 0;
const unsigned long WS_INTERVAL = 500;

float temperature = 0;
float targetTemp = 350.0f; // default target temp for preheating, adjustable via UI

unsigned long lastSwitch = 0;
float pwmSwitchDelayOn = 2000;  // 2s
float pwmSwitchDelayOff = 4000; // 4s

float power = 100;

float kp = 0.6;
float ki = 0.03;
float kd = 0.0;

// Heating modes. PREHEAT, PAUSE and BAKING are all PI-regulated and only
// differ in the setpoint they drive towards. The oven always boots in PREHEAT
// (never MANUAL with the relay latched on).
enum class Mode { MANUAL, PWM, PREHEAT, PAUSE, BAKING };
Mode mode = Mode::PREHEAT;

// Setpoints / timing for the PI-based modes (pauseTemp / bakeBoost are UI-adjustable)
float pauseTemp = 275.0f;                                   // low "keep-warm" hold
float bakeBoost = 40.0f;                                    // °C added on top of target
constexpr unsigned long BAKE_WAIT_MS = 60UL * 1000;         // 1 min warm-up before baking
constexpr unsigned long BAKE_DURATION_MS = 3UL * 60 * 1000; // 3 min bake, then -> PREHEAT

unsigned long bakeStart = 0; // millis() when BAKING was entered

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// integral capped at 70 (not 100): near the setpoint P≈0, so the integral alone
// supplies the holding power. Letting it reach 100 means full power right as we
// hit the setpoint → big overshoot on this high-lag oven. 70 still covers the
// steady-state duty needed at high setpoints.
PIDController pid(kp, ki, kd, 100.0, 0.0, 70.0); // Kp, Ki, Kd, max, min, max_integral

static const char *modeToStr(Mode m)
{
  switch (m)
  {
  case Mode::PWM:
    return "pwm";
  case Mode::PREHEAT:
    return "preheat";
  case Mode::PAUSE:
    return "pause";
  case Mode::BAKING:
    return "baking";
  default:
    return "manual";
  }
}

static bool isPidMode(Mode m)
{
  return m == Mode::PREHEAT || m == Mode::PAUSE || m == Mode::BAKING;
}

// Effective setpoint the PI controller drives towards for the current mode.
static float activeSetpoint()
{
  switch (mode)
  {
  case Mode::PAUSE:
    return pauseTemp;
  case Mode::BAKING:
    return targetTemp + bakeBoost;
  default: // PREHEAT (and any other PI mode)
    return targetTemp;
  }
}

// Centralised mode switch so every entry point stays consistent and safe.
static void applyMode(Mode m)
{
  Mode prev = mode;
  mode = m;

  if (m == Mode::BAKING)
    bakeStart = millis();

  if (m == Mode::MANUAL)
  {
    digitalWrite(relay, LOW); // never leave MANUAL with the relay latched on
    lastSwitch = millis();
  }

  if (isPidMode(m))
  {
    pid.setSetpoint(activeSetpoint());
    // Reset the integral when arriving from a non-PID mode (its value is stale
    // and would dump straight into an overshoot) or when the new target is
    // below the current temp (e.g. PREHEAT -> PAUSE, or BAKING -> PREHEAT).
    // A PID -> PID hand-off at a higher target (e.g. PREHEAT -> BAKING) keeps
    // the integral so the boost adds to the existing holding power.
    if (!isPidMode(prev) || activeSetpoint() < temperature)
      pid.reset();
  }

  Serial.printf("\nMode set to %s", modeToStr(mode));
}

// Get Sensor Readings and return JSON object
String getSensorReadings()
{
  JsonDocument readings;
  String jsonString;

  readings["mode"] = modeToStr(mode);

  temperature = thermocouple.readCelsius();
  readings["temperature"] = temperature;

  readings["relais"] = digitalRead(relay) == HIGH ? 1 : 0;

  readings["target_temp"] = targetTemp;
  readings["pause_temp"] = pauseTemp;
  readings["bake_boost"] = bakeBoost;

  readings["pwm_on"] = pwmSwitchDelayOn;
  readings["pwm_off"] = pwmSwitchDelayOff;

  readings["pid"] = power;

  readings["kp"] = kp;
  readings["ki"] = ki;
  readings["kd"] = kd;

  // Baking runs in two phases: a "wait" warm-up, then the actual "bake".
  // Report the current phase and the seconds left within that phase.
  const char *bakePhase = "";
  unsigned long bakeRemaining = 0;
  if (mode == Mode::BAKING)
  {
    unsigned long elapsed = millis() - bakeStart;
    if (elapsed < BAKE_WAIT_MS)
    {
      bakePhase = "wait";
      bakeRemaining = (BAKE_WAIT_MS - elapsed) / 1000;
    }
    else
    {
      bakePhase = "bake";
      unsigned long total = BAKE_WAIT_MS + BAKE_DURATION_MS;
      bakeRemaining = elapsed >= total ? 0 : (total - elapsed) / 1000;
    }
  }
  readings["bake_phase"] = bakePhase;
  readings["bake_remaining"] = bakeRemaining;

  serializeJson(readings, jsonString);
  return jsonString;
}

// Initialize LittleFS
void initLittleFS()
{
  if (!LittleFS.begin(true))
  {
    Serial.println("An error has occurred while mounting LittleFS");
  }
  Serial.println("LittleFS mounted successfully");
}

// Initialize WiFi
void initWiFi()
{

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    if (millis() - startAttemptTime > 10000)
    {
      Serial.println("Wifi not found, starting AP mode");
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ap_ssid, ap_password);
      Serial.print("AP IP address: ");
      Serial.println(WiFi.softAPIP());
      break;
    }
    Serial.print('.');
    delay(SENSOR_INTERVAL);
    yield();
  }
  Serial.println(WiFi.localIP());
}

void notifyClients(String sensorReadings)
{
  ws.textAll(sensorReadings);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, (char *)data);
    if (err)
    {
      Serial.printf("WS JSON parse error: %s\n", err.c_str());
      return;
    }

    const char *cmd = doc["cmd"];
    if (!cmd)
      return;

    if (strcmp(cmd, "getReadings") == 0)
    {
      notifyClients(getSensorReadings());
    }
    else if (strcmp(cmd, "switchRelais") == 0)
    {
      // Manual override: tapping the relay drops into MANUAL and toggles it.
      Serial.printf("\nSwitch Relais");
      bool turnOn = digitalRead(relay) != HIGH;
      mode = Mode::MANUAL;
      digitalWrite(relay, turnOn ? HIGH : LOW);
      lastSwitch = millis();
    }
    else if (strcmp(cmd, "setTargetTemp") == 0)
    {
      float value = doc["value"].as<float>();
      targetTemp = constrain(value, 0.0f, 600.0f); // clamp to the UI's range
      pid.setSetpoint(activeSetpoint());            // respects the BAKING boost
      Serial.printf("\nTarget temp set to %.1f°C", targetTemp);
    }
    else if (strcmp(cmd, "setPauseTemp") == 0)
    {
      pauseTemp = constrain(doc["value"].as<float>(), 0.0f, 600.0f);
      if (isPidMode(mode))
        pid.setSetpoint(activeSetpoint());
      Serial.printf("\nPause temp set to %.1f°C", pauseTemp);
    }
    else if (strcmp(cmd, "setBakeOffset") == 0)
    {
      bakeBoost = constrain(doc["value"].as<float>(), 0.0f, 150.0f);
      if (isPidMode(mode))
        pid.setSetpoint(activeSetpoint());
      Serial.printf("\nBake offset set to %.1f°C", bakeBoost);
    }
    else if (strcmp(cmd, "setMode") == 0)
    {
      const char *value = doc["value"];
      if (!value)
        return;
      if (strcmp(value, "pwm") == 0)
        applyMode(Mode::PWM);
      else if (strcmp(value, "preheat") == 0)
        applyMode(Mode::PREHEAT);
      else if (strcmp(value, "pause") == 0)
        applyMode(Mode::PAUSE);
      else if (strcmp(value, "baking") == 0)
        applyMode(Mode::BAKING);
      else
        applyMode(Mode::MANUAL);
    }
    else if (strcmp(cmd, "setKp") == 0)
    {
      kp = doc["value"].as<float>();
      pid.setSetKp(kp);
    }
    else if (strcmp(cmd, "setKi") == 0)
    {
      ki = doc["value"].as<float>();
      pid.setSetKi(ki);
    }
    else if (strcmp(cmd, "setKd") == 0)
    {
      kd = doc["value"].as<float>();
      pid.setSetKd(kd);
    }
    else if (strcmp(cmd, "setPWMOn") == 0)
    {
      pwmSwitchDelayOn = doc["value"].as<float>() * 1000;
      Serial.printf("\nPWM switch ON set to %.0fms", pwmSwitchDelayOn);
    }
    else if (strcmp(cmd, "setPWMOff") == 0)
    {
      pwmSwitchDelayOff = doc["value"].as<float>() * 1000;
      Serial.printf("\nPWM switch OFF set to %.0fms", pwmSwitchDelayOff);
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

// Translate a 0-100% PI output into relay switching with a fixed on-time and a
// computed off-time. Handles the saturation cases explicitly so the relay is
// never left in the wrong state (the old code could get stuck OFF at 100%).
void applyPidRelay(float pwr)
{
  if (pwr >= 100.0f)
  {
    pwmSwitchDelayOff = 0.0f; // full power: keep the relay on continuously
    if (digitalRead(relay) != HIGH)
      digitalWrite(relay, HIGH);
    return;
  }
  if (pwr <= 0.0f)
  {
    pwmSwitchDelayOff = 0.0f; // no power: relay off
    if (digitalRead(relay) != LOW)
      digitalWrite(relay, LOW);
    return;
  }

  pwmSwitchDelayOff = (pwmSwitchDelayOn / (pwr / 100.0f)) - pwmSwitchDelayOn;

  unsigned long elapsed = millis() - lastSwitch;
  if (digitalRead(relay) == HIGH && elapsed > pwmSwitchDelayOn)
  {
    digitalWrite(relay, LOW);
    lastSwitch = millis();
  }
  else if (digitalRead(relay) == LOW && elapsed > pwmSwitchDelayOff)
  {
    digitalWrite(relay, HIGH);
    lastSwitch = millis();
  }
}

void regulateRelais()
{
  if (mode == Mode::PWM)
  {
    unsigned long elapsed = millis() - lastSwitch;
    if (digitalRead(relay) == HIGH && elapsed > pwmSwitchDelayOn)
    {
      digitalWrite(relay, LOW);
      lastSwitch = millis();
    }
    else if (digitalRead(relay) == LOW && elapsed > pwmSwitchDelayOff)
    {
      digitalWrite(relay, HIGH);
      lastSwitch = millis();
    }
    return;
  }

  if (isPidMode(mode))
  {
    // Baking = 1 min warm-up + 3 min bake; when it's done, return to PREHEAT.
    if (mode == Mode::BAKING && (millis() - bakeStart) >= (BAKE_WAIT_MS + BAKE_DURATION_MS))
      applyMode(Mode::PREHEAT);

    pid.setSetpoint(activeSetpoint()); // track target / boost / pause changes
    power = pid.compute(temperature);
    applyPidRelay(power);
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(relay, OUTPUT);

  initWiFi();
  initLittleFS();
  initWebSocket();

  digitalWrite(relay, LOW); // boot with heating OFF; the PI loop drives the relay

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html", "text/html"); });

  server.serveStatic("/", LittleFS, "/");

  // Create PID controller instance.
  // NOTE: SENSOR_INTERVAL / 1000 is integer division (100/1000 == 0), which set
  // dt to 0 and silently disabled the integral term — that was the real cause
  // of the steady-state offset (curve flat-lining well below the setpoint).
  pid.setSetpoint(targetTemp);
  pid.setDt(SENSOR_INTERVAL / 1000.0f); // 0.1 s

  ElegantOTA.begin(&server);

  // Start server
  server.begin();
}

void loop()
{
  if ((millis() - lastSensor) > SENSOR_INTERVAL)
  {
    regulateRelais();
    lastSensor = millis();
  }

  if ((millis() - lastWebSocket) > WS_INTERVAL)
  {
    String sensorReadings = getSensorReadings();
    notifyClients(sensorReadings);
    lastWebSocket = millis();
  }

  ws.cleanupClients();
  ElegantOTA.loop();
}