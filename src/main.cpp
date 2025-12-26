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

// #define RELAY 21;
// #define THERMO_DO 19;
// #define THERMO_CS 23;
// #define THERMO_CLK 5;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Timer variables
unsigned long lastSensor = 0;
unsigned long SENSOR_INTERVAL = 100;

unsigned long lastWebSocket = 0;
unsigned long WS_INTERVAL = 500;

int relay = 21;
int thermoDO = 19;
int thermoCS = 23;
int thermoCLK = 5;

float temperature = 0;
int targetTemp = 300;

unsigned long lastSwitch = 0;
float pwmSwitchDelayOn = 2000;  // 2s
float pwmSwitchDelayOff = 4000; // 4s

float power = 0;

float kp = 0.540721828;
float ki = 0.004960751;
float kd = 0.0;

String mode = "manual";

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

PIDController pid(kp, ki, kd, 100.0, 0.0, 100.0); // Kp, Ki, Kd, max, min, max_integral

// Get Sensor Readings and return JSON object
String getSensorReadings()

{
  JsonDocument readings;
  String jsonString;

  readings["mode"] = mode;

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
  while (WiFi.status() != WL_CONNECTED)
  {
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
    String message = (char *)data;
    if (message.startsWith("getReadings"))
    {
      String sensorReadings = getSensorReadings();
      notifyClients(sensorReadings);
    };
    if (message.startsWith("switchRelais"))
    {
      Serial.printf("\nSwitch Relais");
      if (digitalRead(relay) == HIGH)
      {
        digitalWrite(relay, LOW);
        lastSwitch = millis();
        mode = "manual";
      }
      else
      {
        digitalWrite(relay, HIGH);
        lastSwitch = millis();
      }
    }
    if (message.startsWith("setTargetTemp"))
    {
      String target = message.substring(15, 18);
      targetTemp = target.toFloat();
      pid.setSetpoint(targetTemp);
      Serial.printf("\nTarget temp set to ");
      Serial.print(targetTemp);
      Serial.printf("°C");
    }
    if (message.startsWith("setMode"))
    {
      if (message.indexOf("pwm") > 0)
      {
        mode = "pwm";
      }
      else if (message.indexOf("pid") > 0)
      {
        mode = "pid";
      }
      else
      {
        mode = "manual";
        digitalWrite(relay, LOW);
        lastSwitch = millis();
      }
      Serial.printf("\nMode set to ");
      Serial.print(mode);
    }
    if (message.startsWith("setKp"))
    {
      String k = message.substring(6, 12);
      kp = k.toFloat();
      pid.setSetKp(kp);
    }
    if (message.startsWith("setKi"))
    {
      String k = message.substring(6, 12);
      ki = k.toFloat();
      pid.setSetKi(ki);
    }
    if (message.startsWith("setKd"))
    {
      String k = message.substring(6, 12);
      kd = k.toFloat();
      pid.setSetKd(kd);
    }
    if (message.startsWith("setPWMOn"))
    {
      String target = message.substring(10, 15);
      pwmSwitchDelayOn = target.toFloat() * 1000;
      Serial.printf("\nPWM switch ON set to ");
      Serial.print(pwmSwitchDelayOn);
      Serial.printf("°C");
    }

    if (message.startsWith("setPWMOff"))
    {
      String target = message.substring(11, 16);
      pwmSwitchDelayOff = target.toFloat() * 1000;
      Serial.printf("\nPWM switch OFF set to ");
      Serial.print(pwmSwitchDelayOff);
      Serial.printf("°C");
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
  if (mode == "pwm")
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
  if (mode == "pid")
  {
    // output = Kp×error + Ki×∫error×dt + Kd×(Δerror/Δt)
    power = pid.compute(temperature);
    pwmSwitchDelayOff = (pwmSwitchDelayOn / (power / 100)) - pwmSwitchDelayOn;

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