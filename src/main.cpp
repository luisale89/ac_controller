// Libreria conexion wifi y tago
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_now.h>
#include <esp_wifi.h>
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
int network_led_state = LOW;
int ap_button_state = HIGH;

// Variables Sensor temperatura
const int oneWirePipe = 18;
const int oneWireAmb = 19; // se cambia pin del sensor de temp. ambiente, para evitar errores al subir el código.

// definitions
// #define BROKER_LED 32
#define NETWORK_LED 32
#define REMOTE_BUTTON 17
#define REMOTE_COMP_LED 16
#define REMOTE_STATE_LED 27
#define PIN_SENMOV 34 // se cambia pin del sensor de movimiento.
#define AP_BUTTON 14

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWirePipe);
OneWire oneWireAmbTemp(oneWireAmb);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature vapor_temp_sensor(&oneWire);
DallasTemperature ambient_temp_sensor(&oneWireAmbTemp);

// Time Variables
unsigned long lastConnectionTime = 0; // last time a message was sent to the broker, in milliseconds
unsigned long lastControllerTime = 0;
unsigned long lastSaluteTime = 0;
unsigned long lastButtonPress = 0;
unsigned long lastCompressorTurnOff = 0;
unsigned long lastWifiReconnect = 0;
unsigned long lastMqttReconnect = 0;
unsigned long lastNetworkLedBlink = 0;
unsigned long postingInterval = 5L * 60000L;   // delay between updates, in milliseconds.. 5 minutes
unsigned long CompressorTimeOut = 4L * 30000L; // 2 minutos de retardo para habilitar el compresor..
unsigned long MovingSensorTime = 0;
unsigned long AutoTimeOut = 0;                           // wati time for moving sensor and setpoint change
const unsigned long buttonTimeOut = 5L * 1000L;          // rebound 5seconds
const unsigned long controllerInterval = 2L * 5000L;     // delay between sensor updates, 10 seconds
const unsigned long SaluteTimer = 1L * 30000L;           // Tiempo para enviar que el dispositivo esta conectado,
const unsigned long wifiReconnectInterval = 1 * 30000L;  // 30 segundos para intentar reconectar al wifi.
const unsigned long mqttReconnectInterval = 1L * 10000L; // 10 segundos para intentar reconectar al broker mqtt.
const unsigned long wifiDisconnectedLedInterval = 250;        // 250 ms

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
String st;
String content;
String esid = "";
String epass = "";
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

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
double vapor_temp = 22;
double ambient_temp = 22;

// NTP
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -4 * 3600;
const int daylightOffset_sec = 0;

// ESP-NOW VARS
esp_now_peer_info_t slave;
int WiFi_channel;
int msg_counter = 0;

enum MessageType {PAIRING, CONTROLLER_DATA, MONITOR_DATA,};
enum SenderID {SERVER, CONTROLLER, MONITOR,};
enum SystemModes {SYS_OFF, SYS_FAN, SYS_COOL, SYS_AUTOCL,};
MessageType espnow_msg_type;
SystemModes espnow_system_mode;
SenderID espnow_peer_id;

typedef struct pairing_data_struct {
  uint8_t msg_type;             // (1 byte)
  uint8_t sender_id;            // (1 byte)
  uint8_t macAddr[6];           // (6 bytes)
  uint8_t channel;              // (1 byte)
} pairing_data_struct;

typedef struct controller_data_struct {
  uint8_t msg_type;             // (1 byte)
  uint8_t sender_id;            // (1 byte)
  uint8_t system_mode;          // (1 byte)
  uint8_t system_setp;          // (1 byte) active temperature setpoint..
  uint8_t fault_code;           // (1 byte) 0-no_fault; 1..255 controller_fault_codes.
  float room_temp;              // (4 bytes)
  float evap_vapor_line_temp;   // (4 bytes)
  float evap_air_in_temp;       // (4 bytes)
  float evap_air_out_temp;      // (4 bytes)
} controller_data_struct;

typedef struct monitor_data_struct {
  uint8_t msg_type;             // (1 byte)
  uint8_t sender_id;            // (1 byte)
  uint8_t fault_code;           // (1 byte)   0-no_fault; 1..255 monitor_fault_codes.
  float vapor_temp;             // (4 bytes)
  float vapor_press[3];         // (12 bytes) min - avg - max | pressure readings
  float liquid_temp;            // (4 bytes)
  float liquid_press[3];        // (12 bytes) min - avg - max | pressure readings
  float discharge_temp;         // (4 bytes)
  float ambient_temp;           // (4 bytes)
  uint8_t compressor_amp[3];    // (3 bytes)  min - avg - max | compressor_current readings ; 100A max...
  uint8_t compressor_state;     // (1 byte)   255-compressor_on; 0-compressor_off;
} monitor_data_struct;


pairing_data_struct incoming_pairing_data;
controller_data_struct incoming_controller_data;
monitor_data_struct incoming_monitor_a_data;
monitor_data_struct incoming_monitor_b_data; //system B, optional... according to sender_id value.
monitor_data_struct outgoint_monitor_data;
controller_data_struct outgoint_controller_data;

// Guarda el estado del sistema en spiffs
void save_operation_state_in_fs() //[OK]
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

// update rtc with time from ntp server {
void update_rtc_from_ntp()
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
  update_rtc_from_ntp(); // update RTC with latest time from NTP server.
}

void save_timectrl_settings_in_fs(bool value, String onCondition, String offCondition) //[OK]
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

void load_timectrl_settings_from_fs() //[OK]
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
void save_schedule_in_fs(int HON, int HOFF, String Day, bool enable) //[OK]
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
void save_auto_config_in_fs(int wait, int temp)
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
void save_operation_mode_in_fs(String Modo) //[OK]
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
void process_op_state_from_broker(String json) //[NEW, OK]
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
  save_operation_state_in_fs();
}

// Establece el encendido o apagado al inicio
void load_operation_state_from_fs() //[OK]
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
void load_temp_setpoint_from_fs()
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
void load_operation_mode_from_fs() //[OK]
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
void process_settings_from_broker(String json) //[OK]
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
    save_timectrl_settings_in_fs(value, onCondition, offCondition);
    delay(500);
    load_timectrl_settings_from_fs();

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
      save_schedule_in_fs(schedule_item_value_on, schedule_item_value_off, Day, schedule_item_value_enabled);
    }
  }
  else if (String(variable) == "mode_config")
  {
    // Cambia la configuracion del modo
    const char *value = doc["value"]; // "auto"
    int wait = doc["wait"];           // 1234
    int temp = doc["temp"];           // 24
    save_auto_config_in_fs(wait, temp);
  }
  else if (String(variable) == "system_mode")
  {
    // Cambia el modo de operacion
    String value = doc["value"]; // "cool"
    save_operation_mode_in_fs(value);
  }
}

// Guarda el Setpoint en SPIFF
void process_temp_sp_from_broker(String json) //[OK]
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
void mqtt_message_callback(char *topicp, byte *payload, unsigned int length) //[OK]
{
  digitalWrite(NETWORK_LED, LOW);
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
    process_settings_from_broker(Mensaje);
  }
  else if (String(topicp) == "system/operation/state")
  {
    // Apaga o enciende el sistema
    Serial.println("-System State-");
    process_op_state_from_broker(Mensaje);
  }
  else if (String(topicp) == "system/operation/setpoint")
  {
    // Ajusta la temperatura del ambiente
    Serial.println("-Setpoint-");
    process_temp_sp_from_broker(Mensaje);
  }

  Serial.println();
  Serial.println("-----------------------");
  Mensaje = "";

  // Levanta el Flag para envio de datos
  delay(100);
  Envio = true;
  digitalWrite(NETWORK_LED, HIGH);
}

// Funcion que regula la temperatura
void temp_controller(float temp) // [OK]
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

//funcion que ajusta el estado del sistema en funcion del horario y otros parametros.
void system_state_controller() //[OK]
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
        save_operation_state_in_fs();
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
        save_operation_state_in_fs();
      }
    }
  }
  return;
}

// Lee temperatura
double read_vapor_temp_from_sensor() //[ok]
{
  vapor_temp_sensor.requestTemperatures();
  float temperatureC = vapor_temp_sensor.getTempCByIndex(0);
  Serial.println(temperatureC);
  if (temperatureC == 85 || temperatureC == -127)
  {
    Serial.println("$ Error reading OneWire sensor - [Evaporator Temperature]");
    return vapor_temp; // return las valid value from vapor_temp variable..
  }
  return temperatureC;
}

double read_amb_temp_from_sensor() //[ok]
{
  ambient_temp_sensor.requestTemperatures();
  float temperatureAmbC = ambient_temp_sensor.getTempCByIndex(0);
  Serial.println(temperatureAmbC);
  if (temperatureAmbC == 85 || temperatureAmbC == -127)
  {
    Serial.println("$ Error reading OneWire sensor - [Ambient temperature]");
    return ambient_temp; // return las valid value from ambient_temp variable..
  }
  return temperatureAmbC;
}

// Enciende o apaga las salidas a relé
void update_relay_outputs() //[ok]
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

// carga los datos de la red wifi desde el spiffs.
void load_wifi_data_from_fs()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File f = SPIFFS.open("/WiFi.txt");

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    Serial.println("Error al abrir el archivo.");
    return;
  }
  String wifi_data = f.readString();
  // String input;

  StaticJsonDocument<256> doc1;
  DeserializationError error = deserializeJson(doc1, wifi_data);

  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  String ESID = doc1["ssid"];
  String PW = doc1["pass"];

  esid = ESID;
  epass = PW;
  f.close();
  return;
}

// guarda la configuración wifi recibida por smartConfig
void save_wifi_data_in_fs()
{
  StaticJsonDocument<256> doc;
  String output;

  doc["ssid"] = WiFi.SSID();
  doc["pass"] = WiFi.psk();

  Serial.println("-> Saving WiFi data in spiffs: ");

  serializeJson(doc, output);
  if (!SPIFFS.begin(true))
  {
    Serial.println("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/WiFi.txt", "w"); // Borra el contenido anterior del archivo

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    Serial.println("Archivo no existe, debe crearlo...");
    delay(200);
    return;
  }
  f.println(output);
  f.close();
  return;
}

// Este loop verifica que el wifi este conectado correctamente
void wifiloop() //[ok]
{
  // only wait for SmartConfig when the AP button is pressed.
  if ((digitalRead(AP_BUTTON) == 0))
  {
    Serial.println("* the AP button has been pressed, setting up new wifi network*");
    Serial.println("- Waiting for SmartConfig...");
    WiFi.beginSmartConfig();

    while (!WiFi.smartConfigDone())
    {
      Serial.print(".");
      network_led_state = (network_led_state == LOW) ? HIGH : LOW;
      digitalWrite(NETWORK_LED, network_led_state);
      delay(500); //wait for smart config to arrive.
    }
    Serial.println("");
    Serial.println("-> SmartConfig received!");
    Serial.print("-> Testing new WiFi credentials...");

    // test wifi credentials received.
    while (WiFi.status() != WL_CONNECTED)
    {
      Serial.print(".");
      network_led_state = (network_led_state == LOW) ? HIGH : LOW;
      digitalWrite(NETWORK_LED, network_led_state);
      delay(500);
    }

    Serial.println("");
    Serial.print("-> Connected! * IP ADDRESS: ");
    Serial.println(WiFi.localIP());
    // save wifi credential in filesystem.
    save_wifi_data_in_fs();
  }

  if ((WiFi.status() == WL_CONNECTED))
  {
    // WiFi connected...
    digitalWrite(NETWORK_LED, HIGH); // solid led indicates active wifi connection.

    if ((!mqtt_client.connected()) && (millis() - lastMqttReconnect >= mqttReconnectInterval))
    {
      // connecting to a mqtt broker
      Serial.println("* MQTT broker connection attempt..");
      lastMqttReconnect = millis();
      String client_id = "esp32-mqtt_client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("- mqtt_client-id: %s\n", client_id.c_str());
      digitalWrite(NETWORK_LED, HIGH); //led pulse begin...
      // Try mqtt connection to the broker.
      if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password, willTopic, willQoS, willRetain, willMessage))
      {
        Serial.printf("- Connected to MQTT broker: %s\n", mqtt_broker);
        // Publish and subscribe
        Serial.println("- Subscribing to mqtt topics:");
        mqtt_client.subscribe(topic);
        Serial.println("# Topic 1 ok");
        delay(1000);
        mqtt_client.subscribe(topic_2);
        Serial.println("# Topic 2 ok");
        delay(1000);
        mqtt_client.subscribe(topic_3);
        Serial.println("# Topic 3 ok");
        lastSaluteTime = millis();
        Salute = false; // flag to send connection message
        Serial.println("** MQTT settings done. **");
      }
      else
      {
        Serial.print("- Failed MQTT connection with state: ");
        Serial.println(mqtt_client.state());
        digitalWrite(NETWORK_LED, LOW); // led pulse end...
        return;
      }
    }

    if (!Salute && millis() - lastSaluteTime > SaluteTimer)
    {
      mqtt_client.publish("device/hello", "connected");
      Salute = true;
      Serial.println("** MQTT \"Salute\" message sent");
    }
    return;
  }
  // WiFi Disconnected
  if (WiFi.status() != WL_CONNECTED)
  {
    unsigned long currentTime = millis();
    //led user feedback
    if (currentTime - lastNetworkLedBlink >= wifiDisconnectedLedInterval)
    {
      lastNetworkLedBlink = currentTime;
      network_led_state = (network_led_state == LOW) ? HIGH : LOW;
      digitalWrite(NETWORK_LED, network_led_state);
    }

    if (currentTime - lastWifiReconnect >= wifiReconnectInterval)
    {
      lastWifiReconnect = currentTime;
      Serial.println("Device disconnected from WiFi network..");
      Serial.print("WiFi connection status: ");
      Serial.println(WiFi.status());
      Serial.println("Trying to reconnect to SSID: ");
      Serial.println(WiFi.SSID());
      WiFi.disconnect();
      WiFi.begin(esid.c_str(), epass.c_str());
    }
    return;
  }
}

// Codigo del boton de interrupcion
void read_manual_button_state() //[ok]
{
  // Apaga o enciende el aire depediendo del estado

  if (digitalRead(REMOTE_BUTTON) == 0 && millis() - lastButtonPress > buttonTimeOut)
  {
    Serial.println("* Remote Button has been pressed..");
    if (SysState == "on")
    {
      Serial.println("- Turning off the system.");
      SysState = "off";
      save_operation_state_in_fs();
      Envio = true;
    }
    else if (SysState == "off")
    {
      Serial.println("- Turning on the system.");
      SysState = "on";
      save_operation_state_in_fs();
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

//notifica al broker el cambio de estado del sistema.
void notify_state_update_to_broker()
{
  if (PrevSysState == SysState)
  {
    return;
  }
  PrevSysState = SysState; // assign PrevSysState the current SysState
  String pass = mqtt_password;
  String message = SysState;
  message += ",";
  message += pass;
  message += ",";
  message += PrevSysState;
  Serial.println("* Sending MQTT notification for SysState update..");
  // sending message.
  bool message_sent = mqtt_client.publish("device/stateNotification", message.c_str());
  Serial.print("- MQTT publish result: ");
  Serial.println(message_sent);
  return;
}

// Envio de datos recurrente al broker mqtt
void send_data_to_broker() //[ok]
{
  if (!Envio || !mqtt_client.connected())
  {
    return;
  }

  double tdecimal = (int)(ambient_temp * 100 + 0.5) / 100.0;

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
  metadata["re_temp"] = vapor_temp;
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
  bool mqtt_msg_sent = mqtt_client.publish("tago/data/post", output.c_str());
  Serial.print("- MQTT publish result: ");
  Serial.println(mqtt_msg_sent);

  // notify the user about state update...
  notify_state_update_to_broker();

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

// Hace la lectura y envio de datos.
void sensors_read(void *pvParameters)
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
      vapor_temp = read_vapor_temp_from_sensor();
      ambient_temp = read_amb_temp_from_sensor();
      // Funcion que controla el apagado y encendido automatico (Sleep)
      system_state_controller();

      double tdecimal = (int)(ambient_temp * 100 + 0.5) / 100.0;

      // Funcion que regula latemperatura segun el modo (Cool, auto, fan)
      temp_controller(tdecimal);

      // Serial feedback
      Serial.println("SysState: " + String(SysState));
      Serial.println("SysFunc: " + String(SysFunc));
      Serial.print("Movement: ");
      Serial.println(MovingSensor);
      Serial.println("---**---");
    }

    // Check if the remote button is pressed
    read_manual_button_state();

    // flag to send data to tago.io
    if (millis() - lastConnectionTime > postingInterval)
    {
      Envio = true;
    }

    // remote board led feedback...
    digitalWrite(REMOTE_STATE_LED, SysState == "on");

    // update outputs.
    update_relay_outputs();

    //send data to the mqtt broker
    send_data_to_broker();

    // task delay
    vTaskDelay(xDelay);
  }
}

void setup()
{
  Serial.begin(115200);
  // pins definition
  Serial.println("** Hello!, System setup started... **");
  // pinMode(BROKER_LED, OUTPUT);          // broker connection led.
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
  Serial.println("* OneWire vapor_temp_sensor inisialization");
  vapor_temp_sensor.begin();
  delay(1000);
  ambient_temp_sensor.begin();
  delay(1000);
  Serial.println("- OneWire vapor_temp_sensor ok.");

  // set default values from .txt files
  Serial.println("* Reading stored values from file system. (SPIFFS)");
  load_temp_setpoint_from_fs();     // user-temp and auto-temp
  load_operation_state_from_fs();   // on-off setting
  load_operation_mode_from_fs();    // function mode (cool, auto, fan)
  load_timectrl_settings_from_fs(); // timectrl settings
  load_wifi_data_from_fs();         // load wifi data from filesystem
  Serial.println("- SPIFF system ok.");
  lastCompressorTurnOff = millis(); // set now as the last time the compressor was turned off.. prevents startup after blackout
  // --- continue

  // Crea la tarea de lectura de sensores en el segundo procesador.
  Serial.println("* Creating 2nd core task...");
  xTaskCreatePinnedToCore(
      sensors_read, /* Function to implement the task */
      "Task1",     /* Name of the task */
      10000,       /* Stack size in words */
      NULL,        /* Task input parameter */
      0,           /* Priority of the task */
      &Task1,      /* Task handle. */
      0);
  Serial.println("- Task created.");

  // WifiSettings
  Serial.println("* WiFi settings and config.");
  WiFi.disconnect();
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(esid.c_str(), epass.c_str());
  //---------------------------------------- esp-now settings

  //comming soon.
  //---------------------------------------- mqtt settings ---
  mqtt_client.setBufferSize(mqttBufferSize);
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(mqtt_message_callback);
  //---------------------------------------- datetime from ntp server
  Serial.println("* creating NTP server configuration.");
  sntp_set_time_sync_notification_cb(timeavailable);        // sntp sync interval is 1 hour.
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); // configura tiempo desde el servidor NTP

  //---------------------------------------- end of setup ---
  Serial.println("** Setup completed **");
}

void loop()
{
  wifiloop();
  mqtt_client.loop();
  delay(10);
}