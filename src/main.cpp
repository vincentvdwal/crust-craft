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
float targetTemp = 460;

unsigned long lastSwitch = 0;
float pwmSwitchDelayOn = 2000;  // 2s
float pwmSwitchDelayOff = 4000; // 4s

float power = 100;

float kp = 0.6;
float ki = 0.1;
float kd = 0.0;

enum class Mode { MANUAL, PWM, PID };
Mode mode = Mode::MANUAL;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

PIDController pid(kp, ki, kd, 100.0, 0.0, 100.0); // Kp, Ki, Kd, max, min, max_integral

static const char *modeToStr(Mode m)
{
  switch (m)
  {
  case Mode::PWM:
    return "pwm";
  case Mode::PID:
    return "pid";
  default:
    return "manual";
  }
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

  readings["pwm_on"] = pwmSwitchDelayOn;
  readings["pwm_off"] = pwmSwitchDelayOff;

  readings["pid"] = power;

  readings["kp"] = kp;
  readings["ki"] = ki;
  readings["kd"] = kd;

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
      Serial.printf("\nSwitch Relais");
      if (digitalRead(relay) == HIGH)
      {
        digitalWrite(relay, LOW);
        lastSwitch = millis();
        mode = Mode::MANUAL;
      }
      else
      {
        digitalWrite(relay, HIGH);
        lastSwitch = millis();
      }
    }
    else if (strcmp(cmd, "setTargetTemp") == 0)
    {
      targetTemp = doc["value"].as<float>();
      pid.setSetpoint(targetTemp);
      Serial.printf("\nTarget temp set to %.1f°C", targetTemp);
    }
    else if (strcmp(cmd, "setMode") == 0)
    {
      const char *value = doc["value"];
      if (value && strcmp(value, "pwm") == 0)
      {
        mode = Mode::PWM;
      }
      else if (value && strcmp(value, "pid") == 0)
      {
        mode = Mode::PID;
      }
      else
      {
        mode = Mode::MANUAL;
        digitalWrite(relay, LOW);
        lastSwitch = millis();
      }
      Serial.printf("\nMode set to %s", modeToStr(mode));
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

void regulateRelais()
{
  if (mode == Mode::PWM)
  {
    if ((millis() - lastSwitch) > pwmSwitchDelayOn)
    {
      if (digitalRead(relay) == HIGH)
      {
        digitalWrite(relay, LOW);
        lastSwitch = millis();
      }
    }

    if ((millis() - lastSwitch) > pwmSwitchDelayOff)
    {
      if (digitalRead(relay) == LOW)
      {
        digitalWrite(relay, HIGH);
        lastSwitch = millis();
      }
    }
  }
  if (mode == Mode::PID)
  {
    // output = Kp×error + Ki×∫error×dt + Kd×(Δerror/Δt)
    power = pid.compute(temperature);
    if (power > 0.0f)
    {
      pwmSwitchDelayOff = (pwmSwitchDelayOn / (power / 100.0f)) - pwmSwitchDelayOn;
    }
    else
    {
      pwmSwitchDelayOff = 0.0f;
      digitalWrite(relay, LOW);
      lastSwitch = millis();
    }

    if (pwmSwitchDelayOff > 0)
    {
      if ((millis() - lastSwitch) > pwmSwitchDelayOn)
      {
        if (digitalRead(relay) == HIGH)
        {
          digitalWrite(relay, LOW);
          lastSwitch = millis();
        }
      }
    }

    if (pwmSwitchDelayOff > 0)
    {

      if ((millis() - lastSwitch) > pwmSwitchDelayOff)
      {
        if (digitalRead(relay) == LOW)
        {
          digitalWrite(relay, HIGH);
          lastSwitch = millis();
        }
      }
    }
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(relay, OUTPUT);

  initWiFi();
  initLittleFS();
  initWebSocket();

  digitalWrite(relay, HIGH);

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html", "text/html"); });

  server.serveStatic("/", LittleFS, "/");

  // Create PID controller instance
  pid.setSetpoint(targetTemp);
  pid.setDt(SENSOR_INTERVAL / 1000);

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