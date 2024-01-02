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
#include "sntp.h"
#include "time.h"

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
const int oneWirePipe = 18;
const int oneWireAmb = 19; // se cambia pin del sensor de temp. ambiente, para evitar errores al subir el código.

// definitions
#define BROKER_LED 32
#define NETWORK_LED 33
#define REMOTE_BUTTON 17
#define REMOTE_COMP_LED 16
#define REMOTE_STATE_LED 27
#define PIN_SENMOV 34 // se cambia pin del sensor de movimiento.
#define AP_BUTTON 14

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWirePipe);
OneWire oneWireAmbTemp(oneWireAmb);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
DallasTemperature sensorAmb(&oneWireAmbTemp);

// Time Variables
unsigned long lastConnectionTime = 0; // last time a message was sent to the broker, in milliseconds
unsigned long lastControllerTime = 0;
unsigned long lastSaluteTime = 0;
unsigned long lastButtonPress = 0;
unsigned long lastCompressorTurnOff = 0;
unsigned long lastWifiReconnect = 0;
unsigned long lastMqttReconnect = 0;
unsigned long postingInterval = 5L * 60000L;   // delay between updates, in milliseconds.. 5 minutes
unsigned long CompressorTimeOut = 4L * 30000L; // 2 minutos de retardo para habilitar el compresor..
unsigned long MovingSensorTime = 0;
unsigned long AutoTimeOut = 0;                           // wati time for moving sensor and setpoint change
const unsigned long buttonTimeOut = 5L * 1000L;          // rebound 5seconds
const unsigned long controllerInterval = 2L * 5000L;     // delay between sensor updates, 10 seconds
const unsigned long SaluteTimer = 1L * 30000L;           // Tiempo para enviar que el dispositivo esta conectado,
const unsigned long wifiReconnectInterval = 1 * 30000L;  // 30 segundos para intentar reconectar al wifi.
const unsigned long mqttReconnectInterval = 1L * 10000L; // 10 segundos para intentar reconectar al broker mqtt.

// MQTT Broker
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
int statusCode = 200;
const char *ssid = "PHLCONTROLLER";
const char *passphrase = "12345678";
String st;
String content;
String esid = "";
String epass = "";
// Function Decalration
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
String SysState;               // on, off, sleep
String PrevSysState = "undef"; // on, off, sleep
String SysMode;                // auto, fan, cool
String SysFunc = "fan";        // cooling, fan
String on_condition;
String off_condition;
String Sleep = "undef"; // first value for Sleep variable
float SelectTemp;
float AutoTemp;
float CurrentSysTemp;
bool Salute = false;
bool MovingSensor = false;
bool schedule_enabled; // Variables de activacion del modo sleep
double Tc = 22;
double t = 22;

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

void timeavailable(struct timeval *tml)
{
  // this should be called every hour automatically..
  Serial.println("Got time adjustment from NTP! latest datetime is now available");
  getTime(); // update RTC with latest time from NTP server.
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
void SetTemps()
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
  CurrentSysTemp = SelectTemp;

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
  Serial.print("$ Message arrived in topic: ");
  Serial.println(topicp);
  Serial.print("* Message:");
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
  // [bug] system does not responds on setpoint change when auto mode is set.
  //  SelectTemp by default.
  CurrentSysTemp = SelectTemp; // Use SelectTemp as default.

  if (SysMode == "fan")
  {
    SysFunc = "fan";
    return;
  }
  // condition to modify CurrentSysTemp.
  if (SysMode == "auto")
  {
    if (MovingSensor)
    {
      MovingSensorTime = millis();
    }
    else if (millis() - MovingSensorTime > AutoTimeOut)
    {
      // modify CurrentSysTemp to AutoTemp when AutoTimeOut is reached.
      CurrentSysTemp = AutoTemp;
    }
  }

  // control de las salidas en func. de la temperatura
  if (temp < CurrentSysTemp - 0.5)
  {
    if (CurrentSysTemp == AutoTemp)
    {
      SysFunc = "idle";
    }
    else
    {
      SysFunc = "fan";
    }
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
  if (temperatureC == 85 || temperatureC == -127)
  {
    Serial.println("$ Error reading OneWire sensor - [Evaporator Temperature]");
    return Tc; // return las valid value from Tc variable..
  }
  return temperatureC;
}

double AmbTemperature() //[ok]
{
  sensorAmb.requestTemperatures();
  float temperatureAmbC = sensorAmb.getTempCByIndex(0);
  Serial.println(temperatureAmbC);
  if (temperatureAmbC == 85 || temperatureAmbC == -127)
  {
    Serial.println("$ Error reading OneWire sensor - [Ambient temperature]");
    return t; // return las valid value from t variable..
  }
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
  // SysState == "off || sleep" will get Y and G to false, as default
  if (SysState == "on")
  {
    if (SysFunc == "fan")
    {
      // fan only mode
      Y = false;
      G = true;
    }
    else if (SysFunc == "cooling")
    {
      // cooling mode
      Y = true;
      G = true;
    }
    else if (SysFunc == "idle")
    {
      // idle mode
      Y = false;
      G = false;
    }
  }
  // Si no hubo cambio en las salidas..
  if (Y == YState && G == GState)
  {
    return;
  }

  // Si el cambio es para apagar el sistema,
  if (!Y && !G)
  {
    Serial.println("* SysUpdate: Turning off the fan and compressor..");
    digitalWrite(Pin_compresor, Y);
    digitalWrite(REMOTE_COMP_LED, Y);
    delay(250); // delay between relays
    digitalWrite(Pin_vent, G);
    lastCompressorTurnOff = millis(); // update lastCompressorTurnOff value
    Envio = true;
    return;
  }
  // si el cambio es para que el sistema enfríe..
  if (Y && G)
  {
    digitalWrite(Pin_vent, G); // Enciende el ventilador..
    if (millis() - lastCompressorTurnOff > CompressorTimeOut)
    {
      Serial.println("* SysUpdate: Turning on the compressor and the fan.. Cooling mode..");
      delay(250);                     // delay between relays
      digitalWrite(Pin_compresor, Y); // aquí se produce el cambio  del YState, y para el próximo loop no entra en esta parte del código..
      digitalWrite(REMOTE_COMP_LED, Y);
    }
    else
    {
      return; // prevent set Envio to true...
    }
  }
  // Si el cambio es para que el sistema no enfríe, pero funcione la turbina del evaporador..
  else if (!Y && G)
  {
    Serial.println("* SysUpdate: Fan only, compressor turned off..");
    digitalWrite(Pin_vent, G);
    digitalWrite(Pin_compresor, Y);
    digitalWrite(REMOTE_COMP_LED, Y);
    lastCompressorTurnOff = millis(); // update LastCompressorTurnOff value
  }

  Envio = true; // send message to the broker on any state change.
  return;
}

// Este loop verifica que el wifi este conectado correctamente
void wifiloop() //[ok]
{

  // only turn hotspot on when the button in D15 is pressed or esid has never been set.
  if ((digitalRead(AP_BUTTON) == 0) || esid == "") // debug: find a better way for esid...
  {
    Serial.println("* the AP button has been pressed. - Setting new Network config -");
    Serial.println("- Turning the HotSpot On");
    launchWeb();
    setupAP(); // Setup HotSpot
    Serial.println("- Waiting...");
    while ((WiFi.status() != WL_CONNECTED))
    {
      Serial.print(".");
      digitalWrite(NETWORK_LED, HIGH);
      delay(250);
      digitalWrite(NETWORK_LED, LOW);
      delay(250);
      server.handleClient();
    }
    delay(500);
  }

  if ((WiFi.status() == WL_CONNECTED))
  {
    // WiFi connected...
    digitalWrite(NETWORK_LED, HIGH);
    digitalWrite(BROKER_LED, client.connected());

    if ((!client.connected()) && (millis() - lastMqttReconnect >= mqttReconnectInterval))
    {
      // connecting to a mqtt broker
      Serial.println("* MQTT broker connection attempt..");
      lastMqttReconnect = millis();
      String client_id = "esp32-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("- client-id: %s\n", client_id.c_str());
      // Try mqtt connection to the broker.
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password, willTopic, willQoS, willRetain, willMessage))
      {
        Serial.printf("- Connected to MQTT broker: %s\n", mqtt_broker);
        // Publish and subscribe
        Serial.println("- Subscribing to mqtt topics:");
        client.subscribe(topic);
        Serial.println("# Topic 1 ok");
        delay(1000);
        client.subscribe(topic_2);
        Serial.println("# Topic 2 ok");
        delay(1000);
        client.subscribe(topic_3);
        Serial.println("# Topic 3 ok");
        lastSaluteTime = millis();
        Salute = false; // flag to send connection message
        Serial.println("** MQTT settings done. **");
      }
      else
      {
        Serial.print("- Failed MQTT connection with state: ");
        Serial.println(client.state());
        return;
      }
    }

    if (!Salute && millis() - lastSaluteTime > SaluteTimer)
    {
      client.publish("device/hello", "connected");
      Salute = true;
      Serial.println("** MQTT \"Salute\" message sent");
    }
    return;
  }
  // WiFi Reconnect
  digitalWrite(NETWORK_LED, LOW);
  digitalWrite(BROKER_LED, LOW);
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
    return;
  }
}

// Codigo del boton de interrupcion
void PowerAC() //[ok]
{
  // Apaga o enciende el aire depediendo del estado

  if (digitalRead(REMOTE_BUTTON) == 0 && millis() - lastButtonPress > buttonTimeOut)
  {
    Serial.println("* Remote Button has been pressed..");
    if (SysState == "on")
    {
      Serial.println("- Turning off the system.");
      SysState = "off";
      spiffs_save_state();
      Envio = true;
    }
    else if (SysState == "off")
    {
      Serial.println("- Turning on the system.");
      SysState = "on";
      spiffs_save_state();
      Envio = true;
    }
    else if (SysState == "sleep")
    {
      // on Sleep State, the user can't change the System State with the local button..
      // block State change and notify to the user with blinking led.
      Serial.println("- System is in Sleep mode, remote button has no effect.");
      for (int i = 0; i <= 2; i++) // three times
      {
        digitalWrite(REMOTE_STATE_LED, HIGH);
        delay(125);
        digitalWrite(REMOTE_STATE_LED, LOW);
        delay(125);
      }
    }
    lastButtonPress = millis();
  }
}

void NotifyStateChange()
{
  if (PrevSysState == SysState)
  {
    return;
  }
  String pass = mqtt_password;
  // String message = "{\"variable\":\"sys_state_update\",\"value\":\"";
  // message += SysState;
  // message += "\",\"metadata\":{\"pw\":\"";
  // message += pass;
  // message += "\"}}";
  String message = SysState;
  message += ",";
  message += pass;
  message += ",";
  message += PrevSysState;
  Serial.println("* Sending MQTT notification for SysState update..");
  // sending message.
  bool message_sent = client.publish("device/stateNotification", message.c_str());
  Serial.print("- MQTT publish result: ");
  Serial.println(message_sent);
  PrevSysState = SysState; // assign PrevSysState the current SysState
  return;
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
  Serial.println("* Sending msg to mqtt broker:");
  Serial.println(output);
  // Serial.println(output);
  bool mqtt_msg_sent = client.publish("tago/data/post", output.c_str());
  Serial.print("- MQTT publish result: ");
  Serial.println(mqtt_msg_sent);

  // notify the user about state update...
  NotifyStateChange();

  // // update postingInterval, lower if the system is on
  // if (SysState == "on")
  // {
  //   postingInterval = 2L * 30000L; // 1 minuto.
  // }
  // else
  // {
  //   postingInterval = 5L * 60000L; // 5 minutos.
  // }

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

    // flag to send data to tago.io
    if (millis() - lastConnectionTime > postingInterval)
    {
      Envio = true;
    }

    // remote board led feedback...
    digitalWrite(REMOTE_STATE_LED, SysState == "on");

    // update outputs and send data to broker.
    EncenderApagar();
    DataMQTTSend();

    // task delay
    vTaskDelay(xDelay);
  }
}

void setup()
{
  Serial.begin(115200);
  // pins definition
  Serial.println("** Hello!, System setup started... **");
  pinMode(BROKER_LED, OUTPUT);          // broker connection led.
  pinMode(NETWORK_LED, OUTPUT);         // network connection led.
  pinMode(REMOTE_BUTTON, INPUT_PULLUP); // remote board button.
  pinMode(REMOTE_COMP_LED, OUTPUT);     // remote comp led.
  pinMode(REMOTE_STATE_LED, OUTPUT);    // remote state feedback led.
  pinMode(PIN_SENMOV, INPUT_PULLDOWN);  // sensor de presencia
  pinMode(AP_BUTTON, INPUT);            // Wifi Restart and configuration.
  pinMode(Pin_compresor, OUTPUT);       // Compresor
  pinMode(Pin_vent, OUTPUT);            // Ventilador

#ifndef ESP32
  while (!Serial)
    ; // wait for serial port to connect. Needed for native USB
#endif

  Serial.println("* RTC start on I2C bus..");
  if (!DS1307_RTC.begin())
  {
    Serial.println("$ Couldn't find RTC, please reboot.");
    while (1)
      ;
  }
  if (!DS1307_RTC.isrunning())
  {
    Serial.println("- RTC is NOT running!, setting up sketch compiled datetime.");
    // following line sets the RTC to the date & time this sketch was compiled
    DS1307_RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.println("- RTC ok.");

  // Inicio Sensores OneWire
  Serial.println("* OneWire sensors inisialization");
  sensors.begin();
  delay(1000);
  sensorAmb.begin();
  delay(1000);
  Serial.println("- OneWire sensors ok.");

  // set default values from .txt files
  Serial.println("* Reading stored values from file system. (SPIFF)");
  SetTemps();      // user-temp and auto-temp
  SetEncendido();  // on-off setting
  SetFuncModo();   // function mode (cool, auto, fan)
  SettingsSetup(); // timectrl settings
  Serial.println("- SPIFF system ok.");
  lastCompressorTurnOff = millis(); // set now as the last time the compressor was turned off.. prevents startup after blackout
  // --- continue

  // Crea la tarea de lectura de sensores en el segundo procesador.
  Serial.println("* Creating 2nd core task...");
  xTaskCreatePinnedToCore(
      SensorsRead, /* Function to implement the task */
      "Task1",     /* Name of the task */
      10000,       /* Stack size in words */
      NULL,        /* Task input parameter */
      0,           /* Priority of the task */
      &Task1,      /* Task handle. */
      0);
  Serial.println("- Task created.");

  // WifiSettings
  Serial.println("* Disconnecting from any existing wifi network");
  WiFi.disconnect();
  EEPROM.begin(512); // Initialasing EEPROM
  delay(10);
  //
  Serial.println("#");
  Serial.println("#");
  //---------------------------------------- Read eeprom for ssid and pass
  Serial.println("* Reading EEPROM ssid");
  for (int i = 0; i < 32; ++i)
  {
    esid += char(EEPROM.read(i));
  }
  Serial.println();
  Serial.print("- SSID: ");
  Serial.println(esid);
  Serial.println("* Reading EEPROM pass");
  for (int i = 32; i < 96; ++i)
  {
    epass += char(EEPROM.read(i));
  }
  Serial.print("- PASS: ");
  Serial.println(epass);
  //--- wifi connection
  Serial.println("* Attempting wifi connection.");
  WiFi.begin(esid.c_str(), epass.c_str());
  //--- mqtt settings ---
  client.setBufferSize(mqttBufferSize);
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  // datetime from ntp server
  Serial.println("* creating NTP server configuration.");
  sntp_set_time_sync_notification_cb(timeavailable);        // sntp sync interval is 1 hour.
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); // configura tiempo desde el servidor NTP

  // --- end of setup ---
  Serial.println("** Setup completed **");
}

void loop()
{
  wifiloop();
  client.loop();
  delay(10);
}

//-----------------------------------------------Funciones para conexion Wifi, guardar credenciales y conectar, no hay que cambiar
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
  WiFi.softAP("HVAC-CONTROLLER", "climatio");
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
      IPAddress ip = WiFi.softAPIP();
      String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
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
        content = "<!DOCTYPE HTML>\r\n<html><h3>Success.. Saved SSID and PW in eeprom.. reboot to continue.</h3>";
        content += "<form action=\"/reboot\"><input type=\"submit\" value=\"Reboot board...\"/></form></html>";
        statusCode = 200;
      } else {
        content = "<!DOCTYPE HTML>\r\n<html><h3>Error.. invalid SSID or Password.</h3>";
        content += "<button onclick=\"history.back()\">Go back</button></html>";
        statusCode = 400;
        Serial.println("Sending 400");
      }
      server.send(statusCode, "text/html", content); });
    server.on("/reboot", []()
              {
                Serial.println("Reboot in progress...");
                ESP.restart(); });
  }
}