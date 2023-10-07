// Libreria conexion wifi y tago
#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <SPI.h>
#include <ArduinoJson.hpp>
#include <ArduinoJson.h>
// Libreria RTC
#include "RTClib.h"

// Librerias One Wire
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FS.h>
#include <SPIFFS.h>

// LastWill MQTT
const char willTopic[] = "device/lastwill";
int willQoS = 0;
bool willRetain = false;
const char willMessage[] = "disconnected";
uint16_t mqttBufferSize = 512; // LuisLucena, bufferSize in bytes

// Variables
const int Pin_compresor = 25; // Y
const int Pin_vent = 26;      // G

// Variables Sensor temperatura
const int oneWireBus = 18;
const int oneWireAmb = 2;
#define PIN_SENMOV 5
#define REMOTE_BUTTON 17

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
OneWire oneWireAmbTemp(oneWireAmb);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
DallasTemperature sensorAmb(&oneWireAmbTemp);

// TAGO.IO VARIABLES
unsigned long lastConnectionTime = 0; // last time a message was sent to the broker, in milliseconds
unsigned long lastControllerTime = 0;
unsigned long lastSaluteTime = 0;
unsigned long lastButtonPress = 0;
unsigned long lastCompressorTurnOff = 0;
unsigned long lastWifiReconnect = 0;
unsigned long lastMqttReconnect = 0;
unsigned long postingInterval = 2L * 30000L;   // delay between updates, in milliseconds.. 1 minute || 60_000 ms
unsigned long CompressorTimeOut = 4L * 30000L; // 2 minutos de retardo para habilitar el compresor..
unsigned long MovingSensorTime = 0;
unsigned long AutoTimeOut = 0;                           // wati time for moving sensor and setpoint change
const unsigned long buttonTimeOut = 5L * 1000L;          // rebound 5seconds
const unsigned long controllerInterval = 2L * 5000L;     // delay between sensor updates, 10 seconds
const unsigned long SaluteTimer = 1L * 30000L;           // Tiempo para enviar que el dispositivo esta conectado,
const unsigned long wifiReconnectInterval = 1 * 30000L;  // 30 segundos para intentar reconectar al wifi.
const unsigned long mqttReconnectInterval = 1L * 10000L; // 10 segundos para intentar reconectar al broker mqtt.
// MQTT Brokers
const char *mqtt_broker = "mqtt.tago.io";
const char *topic = "system/operation/settings";
const char *topic_2 = "system/operation/state";
const char *topic_3 = "system/operation/setpoint";
const char *mqtt_username = "Token";
const char *mqtt_password = "ebc8915c-9510-480b-a7fc-b057586bdf39";
const int mqtt_port = 1883;
bool Envio = false;

// Handler de tareas de lectura de sensores para nucleo 0 o 1
TaskHandle_t Task1;

// WIFI VARIABLES
int i = 0;
int statusCode;
int mqttReconnectAttempts = 0;
const char *ssid = "PHLCONTROLLER";
const char *passphrase = "12345678";
String st;
String content;
String esid = "";
String epass = "";
// Function Decalration
// bool testWifi(void);
void launchWeb(void);
void setupAP(void);
void createWebServer(void);
// Establishing Local server at port 80
WebServer server(80);
// Initialize the Wifi client library
WiFiClient espClient;
PubSubClient client(espClient);

// RTC
RTC_DS1307 DS1307_RTC;
char Week_days[7][12] = {"Domingo", "Lunes", "Martes", "Miercoles", "Jueves", "Viernes", "Sabado"};

// Variables de configuracion

String SysState;        // on, off, sleep
String SysMode;         // auto, fan, cool
String SysFunc = "fan"; // cooling, fan
String on_condition;
String off_condition;
String Sleep = "undef"; // first value for Sleep variable
float SelectTemp;
float AutoTemp;
float CurrentSysTemp;
bool Salute = false;
bool MovingSensor = false;
bool schedule_enabled; // Variables de activacion del modo sleep
double Tc;
double t;

// NTP
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -4 * 3600;
const int daylightOffset_sec = 0;

// Guarda el estado del sistema
void spiffs_save_state() //[OK]
{

  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/Encendido.txt", "w"); // Borra el contenido anterior del archivo

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    Serial.println("Archivo no existe, creandolo...");
    delay(200);
    // return;
  }
  if (!f)
  {
    Serial.println("Error al abrir el archivo");
    delay(200);
  }
  String SEncendido = SysState;
  f.print(SEncendido);
  f.close();
}

// unsigned long getTime() {
void getTime()
{
  struct tm timeinfo;
  if (getLocalTime(&timeinfo))
  {
    int dia = timeinfo.tm_mday;
    int mes = timeinfo.tm_mon + 1;
    int ano = timeinfo.tm_year + 1900;
    int hora = timeinfo.tm_hour;
    int minuto = timeinfo.tm_min;
    int segundo = timeinfo.tm_sec;

    DS1307_RTC.adjust(DateTime(ano, mes, dia, hora, minuto, segundo));
    Serial.println("Datetime updated..");
    Serial.println("Day: " + String(dia) + "-" + String(hora) + ":" + String(minuto));
  }
  else
  {
    Serial.println("Could not obtain time info from NTP Server, Skiping RTC update");
  }
}

void SaveSettings(bool value, String onCondition, String offCondition) //[OK]
{
  String json;
  StaticJsonDocument<96> doc;

  doc["value"] = value;
  doc["on_condition"] = onCondition;
  doc["off_condition"] = offCondition;

  serializeJson(doc, json);

  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/Settings.txt", "w"); // Borra el contenido anterior del archivo

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    Serial.println("Archivo no existe, creandolo...");
    delay(200);
    // return;
  }
  if (!f)
  {
    Serial.println("Error al abrir el archivo");
    delay(200);
  }
  f.print(json);
  f.close();
}

void SettingsSetup() //[OK]
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File f = SPIFFS.open("/Settings.txt");

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    Serial.println("Error al abrir el archivo.");
    return;
  }
  String Settings = f.readString();
  // String input;

  StaticJsonDocument<128> doc1;

  DeserializationError error = deserializeJson(doc1, Settings);

  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  schedule_enabled = doc1["value"];
  String onCondition = doc1["on_condition"];
  String offCondition = doc1["off_condition"];
  on_condition = onCondition;
  off_condition = offCondition;
  f.close();
}

// Guarda el horario de apgado de cada dia
void SaveSpiff(int HON, int HOFF, String Day, bool enable) //[OK]
{

  String Hours;
  StaticJsonDocument<96> doc;

  doc["ON"] = HON;
  doc["OFF"] = HOFF;
  doc["enable"] = enable;

  serializeJson(doc, Hours);

  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/" + Day + ".txt", "w"); // Borra el contenido anterior del archivo

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    Serial.println("Archivo no existe, creandolo...");
    delay(200);
    // return;
  }
  if (!f)
  {
    Serial.println("Error al abrir el archivo");
    delay(200);
  }
  f.print(Hours);
  f.close();
}

// Aqui se guardan la configuracion del modo Auto que llega en el topico
void SpiffSaveAuto(int wait, int temp)
{
  StaticJsonDocument<32> doc;
  String output;

  doc["wait"] = wait;
  doc["temp"] = temp;

  serializeJson(doc, output);
  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/Auto.txt", "w"); // Borra el contenido anterior del archivo

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    Serial.println("Archivo no existe, creandolo...");
    delay(200);
    // return;
  }
  if (!f)
  {
    Serial.println("Error al abrir el archivo");
    delay(200);
  }
  f.println(output);
  f.close();
  AutoTimeOut = (unsigned long)wait * 60000L; // convert minutes to miliseconds
  AutoTemp = temp;                            // 24
}

// Aqui se guarda el modo que llega desde el topico
void SpiffSaveModo(String Modo) //[OK]
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/Modo.txt", "w"); // Borra el contenido anterior del archivo

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    Serial.println("Archivo no existe, creandolo...");
    delay(200);
    // return;
  }
  if (!f)
  {
    Serial.println("Error al abrir el archivo");
    delay(200);
  }
  f.print(Modo);
  f.close();
  SysMode = Modo;
}

// Lee la variable de encendio o apadado de tago
void set_sys_state(String json) //[NEW, OK]
{
  // String input;

  StaticJsonDocument<96> doc;

  DeserializationError error = deserializeJson(doc, json);
  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }
  String variable = doc["variable"]; // "system_state"
  String value = doc["value"];       // "on"

  if (value == "off" || value == "on")
  {
    SysState = value;
  }
  else
  {
    Serial.println("Error: invalid value received from broker on -system_state-");
  }
  spiffs_save_state();
}

// Establece el encendido o apagado al inicio
void SetEncendido() //[OK]
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File file = SPIFFS.open("/Encendido.txt");

  // Mensaje de fallo al leer el contenido
  if (!file)
  {
    Serial.println("Error al abrir el archivo.");
    return;
  }
  String ESave = file.readString();
  file.close();

  if (ESave == "on" || ESave == "off" || ESave == "sleep")
  {
    SysState = ESave;
  }
  else
  {
    Serial.println("Error: Bad value stored in Encendido.txt");
  }

  return;
}

// Funcion que asigna las temperaturas predeterminadas a las variables globales
void SetTemps() //[DEBUG]
{

  // Temperatura SetPoint
  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File file = SPIFFS.open("/temp.txt");

  // Mensaje de fallo al leer el contenido
  if (!file)
  {
    Serial.println("Error al abrir el archivo.");
    return;
  }
  String ReadTemp = file.readString();

  SelectTemp = ReadTemp.toFloat();
  file.close();

  // Temperatura en Auto

  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File f = SPIFFS.open("/Auto.txt");

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    Serial.println("Error al abrir el archivo.");
    return;
  }
  String AutoSetUp = f.readString();

  StaticJsonDocument<64> doc;

  DeserializationError error = deserializeJson(doc, AutoSetUp);

  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  int wait = doc["wait"];
  AutoTimeOut = (unsigned long)wait * 60000L;
  AutoTemp = doc["temp"]; // 24

  f.close();
}

// Funcion que asigna el Modo almacenado a una variable local [x]
void SetFuncModo() //[OK]
{
  // Temperatura SetPoint
  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File file = SPIFFS.open("/Modo.txt");

  // Mensaje de fallo al leer el contenido
  if (!file)
  {
    Serial.println("Error al abrir el archivo.");
    return;
  }
  String ReadModo = file.readString();

  SysMode = ReadModo;
  file.close();
}

// Guarda la configuracion que se envia en el topico
void Settings(String json) //[OK]
{

  StaticJsonDocument<768> doc;

  DeserializationError error = deserializeJson(doc, json);

  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  const char *variable = doc["variable"]; // "timectrl"
  Serial.println(variable);

  if (String(variable) == "timectrl")
  {

    // Aqui hay que guardar la configuracion del control de apagado encendido
    // Cambia el horario de encendido o apagado
    bool value = doc["enabled"];                //
    String onCondition = doc["on_condition"];   //
    String offCondition = doc["off_condition"]; //
    SaveSettings(value, onCondition, offCondition);
    delay(500);
    SettingsSetup();

    for (JsonPair schedule_item : doc["schedule"].as<JsonObject>())
    {
      const char *schedule_item_key = schedule_item.key().c_str(); // "1", "2", "3", "4", "5", "6", "7"
      int intDay = atoi(schedule_item_key);
      String Day = Week_days[intDay - 1];

      int schedule_item_value_on = schedule_item.value()["on"];
      int schedule_item_value_off = schedule_item.value()["off"];
      bool schedule_item_value_enabled = schedule_item.value()["enabled"]; // true
      Serial.println(Day);
      // String input;, false, true, true, true, ...
      SaveSpiff(schedule_item_value_on, schedule_item_value_off, Day, schedule_item_value_enabled);
    }
  }
  else if (String(variable) == "mode_config")
  {
    // Cambia la configuracion del modo
    const char *value = doc["value"]; // "auto"
    int wait = doc["wait"];           // 1234
    int temp = doc["temp"];           // 24
    SpiffSaveAuto(wait, temp);
  }
  else if (String(variable) == "system_mode")
  {
    // Cambia el modo de operacion
    String value = doc["value"]; // "cool"
    SpiffSaveModo(value);
  }
}

// Guarda el Setpoint en SPIFF
void setpoint(String json) //[OK]
{

  StaticJsonDocument<96> doc;

  DeserializationError error = deserializeJson(doc, json);

  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  String variable = doc["variable"]; // "user_setpoint"
  int value = doc["value"];          // 24

  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/temp.txt", "w"); // Borra el contenido anterior del archivo

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    Serial.println("Archivo no existe, creandolo...");
    delay(200);
    // return;
  }
  if (!f)
  {
    Serial.println("Error al abrir el archivo");
    delay(200);
  }
  f.print(value);
  SelectTemp = value;
  f.close();
}

// Funcion que recibe la data de Tago por MQTT
void callback(char *topicp, byte *payload, unsigned int length) //[OK]
{
  Serial.print("Message arrived in topic: ");
  Serial.println(topicp);
  Serial.print("Message:");
  String Mensaje;
  for (int i = 0; i < length; i++)
  {

    Serial.print((char)payload[i]);

    Mensaje = Mensaje + (char)payload[i];
  }
  Serial.println("Este es el mensaje " + Mensaje);

  // Selecciona la funcion acorde al topico al cual llego el mensaje
  if (String(topicp) == "system/operation/settings")
  {
    // Cambia la configuracion del sistema
    Serial.println("-Settings-");
    Settings(Mensaje);
  }
  else if (String(topicp) == "system/operation/state")
  {
    // Apaga o enciende el sistema
    Serial.println("-System State-");
    set_sys_state(Mensaje);
  }
  else if (String(topicp) == "system/operation/setpoint")
  {
    // Ajusta la temperatura del ambiente
    Serial.println("-Setpoint-");
    setpoint(Mensaje);
  }

  Serial.println();
  Serial.println("-----------------------");
  Mensaje = "";

  // Levanta el Flag para envio de datos
  delay(100);
  Envio = true;
}

// Funcion que regula la temperatura
void TempRegulator(float temp) // [OK]
{

  if (SysMode == "fan")
  {
    CurrentSysTemp = SelectTemp;
    SysFunc = "fan";
    return;
  }

  if (SysMode == "cool")
  {
    CurrentSysTemp = SelectTemp;
  }
  else if (SysMode == "auto")
  {
    if (MovingSensor)
    {
      MovingSensorTime = millis();
      CurrentSysTemp = SelectTemp;
    }
    else if (millis() - MovingSensorTime > AutoTimeOut)
    {
      CurrentSysTemp = AutoTemp;
      Serial.print("- CurrentSysTemp = AutoTemp- ");
    }
  }

  // control de las salidas en func. de la temperatura

  if (temp < CurrentSysTemp - 0.5)
  {
    SysFunc = "fan";
  }
  else if (temp > CurrentSysTemp + 0.5)
  {
    SysFunc = "cooling";
  }

  return;
}

// Funcion que apaga el aire segun la hora
// Guardar el dia en una varibale local y compararlo con el RTC, y cuando sean diferentes guardar el horario de este dia
void AutoOn() //[OK]
{

  if (!schedule_enabled)
  {
    Serial.println("Schedule disabled..");
    Sleep = "off";
    return;
  }

  DateTime now = DS1307_RTC.now();
  String Day = Week_days[now.dayOfTheWeek()];

  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File file = SPIFFS.open("/" + Day + ".txt");

  // Mensaje de fallo al leer el contenido
  if (!file)
  {
    Serial.println("Error al abrir el archivo.");
    return;
  }
  String AutoOff = file.readString();
  file.close();

  StaticJsonDocument<64> doc;

  DeserializationError error = deserializeJson(doc, AutoOff);

  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  int ON = doc["ON"];   // "0730"
  int OFF = doc["OFF"]; // "2130"

  bool enable = doc["enable"];
  if (!enable)
  {
    Sleep = "off";
    return;
  }

  String hora = String(now.hour());
  int Min = int(now.minute());
  String minuto = "0";
  if (Min < 10)
  {
    Serial.println("minuto menor a 10");
    minuto.concat(Min);
  }
  else
  {
    Serial.println("minuto 10 o mayor");
    minuto = String(Min);
  }
  String tiempo = hora + minuto;
  Serial.println("tiempo: " + tiempo);
  Serial.print("WakeUp at: ");
  Serial.println(ON);
  Serial.print("Sleep at: ");
  Serial.println(OFF);

  if (OFF > tiempo.toInt() && ON < tiempo.toInt())
  {
    // Si el sleep está activado, o si es la primera verificación después del arranque..
    if (Sleep == "on" || Sleep == "undef")
    {
      if (on_condition == "presence" && MovingSensor)
      {
        Sleep = "off";
      }
      else if (on_condition == "on_time")
      {
        Sleep = "off";
      }
      if (Sleep == "off")
      {
        Serial.println("New [variable] Sleep: " + String(Sleep));
        SysState = "on";
        spiffs_save_state();
      }
    }
  }
  else
  {
    if (Sleep == "off" || Sleep == "undef")
    {
      if (off_condition == "presence" && !MovingSensor)
      {
        // SleepState
        Sleep = "on";
      }
      else if (off_condition == "on_time")
      {
        // SleepState
        Sleep = "on";
      }
      if (Sleep == "on")
      {
        Serial.println("New [variable] Sleep: " + String(Sleep));
        SysState = "sleep";
        spiffs_save_state();
      }
    }
  }
  return;
}
// Lee temperatura
double Temperature() //[ok]
{
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  Serial.println(temperatureC);
  return temperatureC;
}

double AmbTemperature() //[ok]
{
  sensorAmb.requestTemperatures();
  float temperatureAmbC = sensorAmb.getTempCByIndex(0);
  Serial.println(temperatureAmbC);
  return temperatureAmbC;
}

// Enciende o apaga las salidas a relé
void EncenderApagar() //[ok]
{
  bool YState = false;
  bool GState = false;
  bool Y = false; // off by default
  bool G = false; // off by default

  // Validacion si hay algun cambio de estado en los pines
  if (digitalRead(Pin_vent) == 1)
  {
    GState = true; // update GState
  }

  if (digitalRead(Pin_compresor) == 1)
  {
    YState = true; // update YState
  }
  // SysFunc == "off || sleep" will get Y and G to false, as default
  if (SysFunc == "fan" && SysState == "on")
  {
    Y = false;
    G = true;
  }
  else if (SysFunc == "cooling" && SysState == "on")
  {
    Y = true;
    G = true;
  }
  // Si no hubo cambio en las salidas..
  if (Y == YState && G == GState)
  {
    return;
  }

  if (SysState == "off" || SysState == "sleep")
  {
    Serial.println("Turning off the system..");
    digitalWrite(Pin_compresor, LOW);
    delay(250); // delay between relays
    digitalWrite(Pin_vent, LOW);
    lastCompressorTurnOff = millis(); // update lastCompressorTurnOff value
    getTime();                        // update RTC
    Envio = true;
    return;
  }
  // si el cambio es para que el sistema enfríe..
  if (SysFunc == "cooling")
  {
    digitalWrite(Pin_vent, HIGH);
    if (millis() - lastCompressorTurnOff > CompressorTimeOut)
    {
      Serial.println("Turning on the compressor.. Cooling..");
      delay(250);                        // delay between relays
      digitalWrite(Pin_compresor, HIGH); // aquí se produce el cambio  del YState, y para el próximo loop no entra en esta parte del código..
    }
    else
    {
      return; // prevent set Envio to true...
    }
  }
  // Si el cambio es para que el sistema no enfríe, pero funcione la turbina del evaporador..
  else if (SysFunc == "fan")
  {
    digitalWrite(Pin_vent, HIGH);
    digitalWrite(Pin_compresor, LOW);
    lastCompressorTurnOff = millis(); // update LastCompressorTurnOff value
  }
  // Si el cambio es para apagar el sistema..

  Envio = true;
  return;
}

// Este loop verifica que el wifi este conectado correctamente
void wifiloop() //[ok]
{

  // only turn hotspot on when the button in D15 is pressed or esid has never been set.
  if ((digitalRead(15) == 1) || esid == "") // debug: find a better way for esid...
  {
    Serial.println("D15 HIGH or esid has never been set");
    Serial.println("Turning the HotSpot On");
    launchWeb();
    setupAP(); // Setup HotSpot
    Serial.println("Waiting...");
    while ((WiFi.status() != WL_CONNECTED))
    {
      Serial.print(".");
      delay(500);
      server.handleClient();
    }
    delay(500);
  }

  if ((WiFi.status() == WL_CONNECTED))
  {
    // WiFi connected...
    if ((!client.connected()) && (millis() - lastMqttReconnect >= mqttReconnectInterval))
    {
      // connecting to a mqtt broker
      lastMqttReconnect = millis();
      String client_id = "esp32-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("Intentando conectar al cliente: %s al broker MQTT\n", client_id.c_str());
      // Try mqtt connection to the broker.
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password, willTopic, willQoS, willRetain, willMessage))
      {
        Serial.printf("- Connected with MQTT broker on: %s -\n", mqtt_broker);
        // Publish and subscribe
        client.subscribe(topic);
        delay(1000); // debug: why this?
        client.subscribe(topic_2);
        delay(1000); // debug: why this?
        client.subscribe(topic_3);
        lastSaluteTime = millis();
        Salute = false; // flag to send connection message
        mqttReconnectAttempts = 0;
      }
      else
      {
        Serial.print("failed MQTT connection with state: ");
        Serial.println(client.state());
        mqttReconnectAttempts += 1;
        if (mqttReconnectAttempts >= 10)
        {
          Serial.println("Disconnecting from the Wifi network after 10 reconnection attempts to the MQTT broker");
          WiFi.disconnect();
          lastWifiReconnect = millis();
        }
      }
    }

    if (!Salute && millis() - lastSaluteTime > SaluteTimer)
    {
      client.publish("device/hello", "connected");
      Salute = true;
      Serial.println("MQTT Dispositivo conectado");
    }
  }
  // WiFi Reconnect
  if ((WiFi.status() != WL_CONNECTED) && (millis() - lastWifiReconnect >= wifiReconnectInterval))
  {
    // WiFi disconnected
    Serial.println("Device disconnected from WiFi network..");
    Serial.print("WiFi connection status: ");
    Serial.println(WiFi.status());
    Serial.println("Trying connection to WiFi network...");
    WiFi.disconnect();
    WiFi.begin(esid.c_str(), epass.c_str());
    lastWifiReconnect = millis();
    mqttReconnectAttempts = 0;
  }
}

// Codigo del boton de interrupcion
void PowerAC() //[ok]
{
  // Apaga o enciende el aire depediendo del estado

  if (digitalRead(REMOTE_BUTTON) && millis() - lastButtonPress > buttonTimeOut)
  {
    Serial.println("Remote Button pressed..");
    if (SysState == "on")
    {
      SysState = "off";
      spiffs_save_state();
    }
    else if (SysState == "off")
    {
      SysState = "on";
      spiffs_save_state();
    }
    else if (SysState == "sleep")
    {
      // on Sleep State, the user can't change the System State with the local button..
      // block State change and notify to the user with blinking led.
      for (int i = 0; i <= 2; i++) // three times
      {
        digitalWrite(16, HIGH);
        delay(125);
        digitalWrite(16, LOW);
        delay(125);
      }
    }

    lastButtonPress = millis();
  }
}

// Envio de datos a Tago.io
void DataMQTTSend() //[ok]
{
  if (!Envio || !client.connected())
  {
    return;
  }

  double tdecimal = (int)(t * 100 + 0.5) / 100.0;

  // TAGO.IO SEND DATA
  bool Y = false;
  bool G = false;
  // update Y and G with the output pins
  if (digitalRead(Pin_vent) == 1)
  {
    G = true;
  }

  if (digitalRead(Pin_compresor) == 1)
  {
    Y = true;
  }

  String timectrl;
  if (schedule_enabled)
  {
    timectrl = "on";
  }
  else
  {
    timectrl = "off";
  }
  // json todas las variables, falta recoleccion
  String output;
  StaticJsonDocument<256> doc;

  doc["variable"] = "ac_control";
  doc["value"] = tdecimal;

  JsonObject metadata = doc.createNestedObject("metadata");
  metadata["user_setpoint"] = SelectTemp;
  metadata["active_setpoint"] = CurrentSysTemp;
  metadata["re_temp"] = Tc;
  metadata["G"] = G;
  metadata["Y"] = Y;
  metadata["sys_state"] = SysState;
  metadata["sys_mode"] = SysMode;
  metadata["presence"] = digitalRead(PIN_SENMOV);
  metadata["timectrl"] = timectrl;

  serializeJson(doc, output);
  Serial.println(output);
  // Serial.println(output);
  bool mqtt_msg_sent = client.publish("tago/data/post", output.c_str());
  Serial.print("MQTT publish result: ");
  Serial.println(mqtt_msg_sent);

  // update postingInterval, lower if the system is on

  if (SysState == "on")
  {
    postingInterval = 2L * 30000L; // 1 minuto.
  }
  else
  {
    postingInterval = 3L * 60000L; // 3 minutos.
  }

  Envio = false;
  lastConnectionTime = millis();
}

// Hace la lectura y envio de datos de manera periodica
void SensorsRead(void *pvParameters)
{

  const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
  for (;;)
  {

    // Lectura de sensor de movimiento va aqui
    if (digitalRead(PIN_SENMOV))
    {
      MovingSensor = true;
    }
    else
    {
      MovingSensor = false;
    }

    if (millis() - lastControllerTime > controllerInterval)
    {
      lastControllerTime = millis();
      Tc = Temperature();
      t = AmbTemperature();
      // Funcion que controla el apagado y encendido automatico (Sleep)
      AutoOn();

      double tdecimal = (int)(t * 100 + 0.5) / 100.0;

      // Funcion que regula latemperatura segun el modo (Cool, auto, fan)
      TempRegulator(tdecimal);

      // Serial feedback
      Serial.println("SysState: " + String(SysState));
      Serial.println("SysFunc: " + String(SysFunc));
      Serial.print("Movement: ");
      Serial.println(MovingSensor);
      Serial.println("---**---");
    }

    // Check if the remote button is pressed
    PowerAC();

    // user feedback
    if (SysState == "on")
    {
      digitalWrite(16, HIGH); // remote board led on..
    }
    else
    {
      digitalWrite(16, LOW); // remote board led off...
    }

    // flag to send data to tago.io
    if (millis() - lastConnectionTime > postingInterval)
    {
      Envio = true;
    }

    // update outputs and send data to broker.
    EncenderApagar();
    DataMQTTSend();
    vTaskDelay(xDelay);
  }
}

void setup()
{
  Serial.begin(115200);
  // pins definition
  pinMode(15, INPUT);             // Wifi Restart
  pinMode(PIN_SENMOV, INPUT);     // sensor de presencia
  pinMode(REMOTE_BUTTON, INPUT);  // remote board button.
  pinMode(16, OUTPUT);            // remote board led.
  pinMode(Pin_compresor, OUTPUT); // Compresor
  pinMode(Pin_vent, OUTPUT);      // Ventilador

#ifndef ESP32
  while (!Serial)
    ; // wait for serial port to connect. Needed for native USB
#endif

  if (!DS1307_RTC.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }

  // // Boton de interrupcion
  // Interrupción está causando problemas... se reinicia la placa (Luis Lucena)
  // attachInterrupt(17, PowerAC, HIGH);

  // Inicio Sensores OneWire
  sensors.begin();
  delay(1000);
  sensorAmb.begin();
  delay(1000);

  // set default values from .txt files
  SetTemps();                       // user-temp and auto-temp
  SetEncendido();                   // on-off setting
  SetFuncModo();                    // function mode (cool, auto, fan)
  SettingsSetup();                  // timectrl settings
  lastCompressorTurnOff = millis(); // set now as the last time the compressor was turned off.. prevents blackout starts
  // --- continue

  // Crea la tarea de lectura de sensores en el segundo procesador.
  xTaskCreatePinnedToCore(
      SensorsRead, /* Function to implement the task */
      "Task1",     /* Name of the task */
      10000,       /* Stack size in words */
      NULL,        /* Task input parameter */
      0,           /* Priority of the task */
      &Task1,      /* Task handle. */
      0);

  // WifiSettings
  Serial.println("Disconnecting current wifi connection");
  WiFi.disconnect();
  EEPROM.begin(512); // Initialasing EEPROM
  delay(10);
  //
  Serial.println();
  Serial.println();
  Serial.println("Startup");
  //---------------------------------------- Read eeprom for ssid and pass
  Serial.println("Reading EEPROM ssid");
  for (int i = 0; i < 32; ++i)
  {
    esid += char(EEPROM.read(i));
  }
  Serial.println();
  Serial.print("SSID: ");
  Serial.println(esid);
  Serial.println("Reading EEPROM pass");
  for (int i = 32; i < 96; ++i)
  {
    epass += char(EEPROM.read(i));
  }
  Serial.print("PASS: ");
  Serial.println(epass);
  //--- wifi connection
  WiFi.begin(esid.c_str(), epass.c_str());
  //--- mqtt settings ---
  client.setBufferSize(mqttBufferSize);
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  // datetime from ntp server
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  getTime();
}

void loop()
{
  wifiloop();
  client.loop();
  delay(10);
}

//-----------------------------------------------Funciones para conexion Wifi, guardar credenciales y conectar, no hay que cambiar
// bool testWifi(void)
// {
//   int c = 0;
//   // Serial.println("Waiting for Wifi to connect");
//   while (c < 20)
//   {
//     if (WiFi.status() == WL_CONNECTED)
//     {
//       return true;
//     }
//     delay(500);
//     Serial.print("*");
//     c++;
//   }
//   Serial.println("");
//   Serial.println("Wifi connection timed out,");
//   return false;
// }
void launchWeb()
{
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("WiFi connected");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());
  createWebServer();
  // Start the server
  server.begin();
  Serial.println("Server started");
}
void setupAP(void)
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0)
    Serial.println("no networks found");
  else
  {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      // Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
      delay(10);
    }
  }
  Serial.println("");
  st = "<ol>";
  for (int i = 0; i < n; ++i)
  {
    // Print SSID and RSSI for each network found
    st += "<li>";
    st += WiFi.SSID(i);
    st += " (";
    st += WiFi.RSSI(i);
    st += ")";
    // st += (WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*";
    st += "</li>";
  }
  st += "</ol>";
  delay(100);
  WiFi.softAP("PHLCONTROLLER", "12345678");
  Serial.println("Initializing_softap_for_wifi credentials_modification");
  launchWeb();
  Serial.println("over");
}

void createWebServer()
{
  {
    server.on("/", []()
              {
      IPAddress ip = WiFi.softAPIP();
      String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
      content = "<!DOCTYPE HTML>\r\n<html>Welcome to Wifi Credentials Update page";
      content += "<form action=\"/scan\" method=\"POST\"><input type=\"submit\" value=\"scan\"></form>";
      content += ipStr;
      content += "<p>";
      content += st;
      content += "</p><form method='get' action='setting'><label>SSID: </label><input name='ssid' length=32><input name='pass' length=64><input type='submit'></form>";
      content += "</html>";
      server.send(200, "text/html", content); });
    server.on("/scan", []()
              {
      //setupAP();
      IPAddress ip = WiFi.softAPIP();
      String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
      content = "<!DOCTYPE HTML>\r\n<html>go back";
      server.send(200, "text/html", content); });
    server.on("/setting", []()
              {
      String qsid = server.arg("ssid");
      String qpass = server.arg("pass");
      if (qsid.length() > 0 && qpass.length() > 0) {
        Serial.println("clearing eeprom");
        for (int i = 0; i < 96; ++i) {
          EEPROM.write(i, 0);
        }
        Serial.println(qsid);
        Serial.println("");
        Serial.println(qpass);
        Serial.println("");
        Serial.println("writing eeprom ssid:");
        for (int i = 0; i < qsid.length(); ++i)
        {
          EEPROM.write(i, qsid[i]);
          Serial.print("Wrote: ");
          Serial.println(qsid[i]);
        }
        Serial.println("writing eeprom pass:");
        for (int i = 0; i < qpass.length(); ++i)
        {
          EEPROM.write(32 + i, qpass[i]);
          Serial.print("Wrote: ");
          Serial.println(qpass[i]);
        }
        EEPROM.commit();
        content = "{\"Success\":\"saved to eeprom... reset to boot into new wifi\"}";
        statusCode = 200;
        ESP.restart();
      } else {
        content = "{\"Error\":\"404 not found\"}";
        statusCode = 404;
        Serial.println("Sending 404");
      }
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(statusCode, "application/json", content); });
  }
}
