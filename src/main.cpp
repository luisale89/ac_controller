// logging
#include <esp_log.h>
static const char* TAG = "main";
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
#include <secrets.h>

// LastWill MQTT
const char willTopic[20] = "device/lastwill";
const int willQoS = 0;
const bool willRetain = false;
const char willMessage[20] = "disconnected";
uint16_t mqttBufferSize = 512; // LuisLucena, bufferSize in bytes

// Variables
int network_led_state = LOW;

// pinout
#define NETWORK_LED 2
#define RADAR 26
#define AP_BTN 19
#define MANUAL_BTN 34
#define NOW_BTN 18
#define AMB_TEMP 32

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWireAmbTemp(AMB_TEMP);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature ambient_t_sensor(&oneWireAmbTemp);

// Time Variables
unsigned long lastConnectionTime = 0; // last time a message was sent to the broker, in milliseconds
unsigned long lastControllerTime = 0;
unsigned long lastSaluteTime = 0;
unsigned long lastButtonPress = 0;
unsigned long lastWifiReconnect = 0;
unsigned long lastMqttReconnect = 0;
unsigned long lastNetworkLedBlink = 0;
unsigned long postingInterval = 5L * 60000L;   // delay between updates, in milliseconds.. 5 minutes
unsigned long MovingSensorTime = 0;
unsigned long AutoTimeOut = 0;                           // wati time for moving sensor and setpoint change
unsigned long lastTempRequest = 0;
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
const char *mqtt_password = MQTT_PASSWORD;
const int mqtt_port = 1883;
bool Envio = false;

// Handler de tareas de lectura de sensores para nucleo 0 o 1
TaskHandle_t Task1;

// WIFI VARIABLES
char* esid = "";
char* epass = "";
const char* AP_SSID = "AC-HUB";
const char* AP_PASS = AP_PASSWORD;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);
int wifi_reconnect_attempt = 0;
const int MAX_RECONNECT_ATTEMPTS = 22; // two times on every channel.

// RTC
RTC_DS3231 DS3231_RTC;
char Week_days[7][12] = {"Domingo", "Lunes", "Martes", "Miercoles", "Jueves", "Viernes", "Sabado"};

// Variables de configuracion
enum SysModeEnum {AUTO, FAN, COOL};
enum SysStateEnum {ON_STATE, OFF_STATE, SLEEP};
SysStateEnum SysState = OFF_STATE;
SysStateEnum PrevSysState = OFF_STATE;
SysModeEnum SysMode = AUTO;

//--
String on_condition;
String off_condition;
String Sleep = "undef"; // first value for Sleep variable
float SelectTemp;
float AutoTemp;
float CurrentSysTemp;
bool Salute = false;
bool MovingSensor = false;
bool schedule_enabled; // Variables de activacion del modo sleep
double ambient_temp = 22;
int ds_sensor_resolution = 10; //bits
int tempRequestDelay = 0;

// NTP
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -4 * 3600;
const int daylightOffset_sec = 0;

// ESP-NOW VARS
esp_now_peer_info_t slave;
int32_t WiFi_channel = 1;

enum MessageType {PAIRING, DATA,};
enum SenderID {SERVER, CONTROLLER, MONITOR,};
enum SystemModes {SYS_OFF, SYS_FAN, SYS_COOL, SYS_AUTO_CL,};
enum WiFiModeState {ESPNOW_OFFLINE, ESPNOW_ONLINE, ESPNOW_IDLE,};
WiFiModeState espnow_connection_state = ESPNOW_IDLE;
// MessageType espnow_msg_type;
// SystemModes espnow_system_mode;
// SenderID espnow_peer_id;

typedef struct pairing_data_struct {
  uint8_t msg_type;             // (1 byte)
  uint8_t sender_id;            // (1 byte)
  uint8_t macAddr[6];           // (6 bytes)
  uint8_t channel;              // (1 byte)
} pairing_data_struct;          // TOTAL = 9 bytes

typedef struct controller_data_struct {
  uint8_t msg_type;             // (1 byte)
  uint8_t sender_id;            // (1 byte)
  uint8_t active_system_mode;   // (1 byte)
  uint8_t fault_code;           // (1 byte) 0-no_fault; 1..255 controller_fault_codes.
  float evap_satura_temp;       // (4 bytes)
  float evap_air_in_temp;       // (4 bytes)
  float evap_air_out_temp;      // (4 bytes)
} controller_data_struct;       // TOTAL = 21 bytes

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
  float compressor_amp[3];      // (12 bytes)  min - avg - max | compressor_current readings
  uint8_t compressor_state;     // (1 byte)   255-compressor_on; 0-compressor_off;
} monitor_data_struct;          // TOTAL = 56 bytes


typedef struct outgoing_settings_struct {
  uint8_t msg_type;             // (1 byte)
  uint8_t sender_id;            // (1 byte)
  uint8_t fault_restart;        // (1 byte)
  uint8_t system_setpoint;      // (1 byte)
  uint8_t system_mode;          // (1 byte)
  uint8_t system_state;         // (1 byte)
  uint8_t room_temp;            // (1 byte)
} outgoing_settings_struct;     // TOTAL = 7 bytes

pairing_data_struct pairing_data;
controller_data_struct controller_data;
monitor_data_struct monitor_data;
outgoing_settings_struct outgoing_settings_data;

//logger functions
void info_logger(const char *message) {
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "%s", message);
}

void error_logger(const char *message) {
  ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "%s", message);
}

//esp-now functions.
String printMAC(const uint8_t * mac_addr) {
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // return mac_str;
  String str = (char*) mac_str;
  return str;
}

// add peer to peer list.
bool addPeer(const uint8_t *peer_addr) {      // add pairing
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[esp-now] adding new peer to peer list");
  memset(&slave, 0, sizeof(slave));
  const esp_now_peer_info_t *peer = &slave;
  memcpy(slave.peer_addr, peer_addr, 6);
  
  slave.channel = WiFi_channel; // pick a channel.. 0 means it take the current STA channel (connected to AP)
  slave.encrypt = 0; // no encryption
  // check if the peer exists
  bool exists = esp_now_is_peer_exist(slave.peer_addr);
  if (exists) {
    // Slave already paired.
    info_logger("[esp-now] peer already exists, deleting existing peer.");
    esp_err_t delStatus = esp_now_del_peer(peer_addr);
    if (delStatus == ESP_OK) {
      info_logger("[esp-now] peer deleted!");
    } else {
      error_logger("[esp-now] error deleting peer!");
    }
  }
  esp_err_t addStatus = esp_now_add_peer(peer);
  if (addStatus == ESP_OK) {
    // Pair success
    info_logger("[esp-now] new peer added successfully.");
    return true;
  }
  else
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "[esp-now] Error: %d, while adding new peer.", addStatus);
    return false;
  }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  String printable_mac_address = printMAC(mac_addr);
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[esp-now] packet sent to: %s", printable_mac_address.c_str());
  if (status == ESP_NOW_SEND_SUCCESS) {
    info_logger("[esp-now] delivery success.");
  } else {
    error_logger("[esp-now] delivery fail.");
  }
}


void OnDataRecv(const uint8_t * mac_addr, const uint8_t * incomingData, int len) { 
  String printable_mac_address = printMAC(mac_addr);
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[esp-now] %d bytes of data received from: %s", len, printable_mac_address.c_str());
  if (espnow_connection_state == ESPNOW_IDLE) {
    info_logger("[esp-now] Waiting to finish AP connection. Ignoring Data..");
    return;
  }
  StaticJsonDocument<512> root;
  String payload;
  uint8_t type = incomingData[0];       // first message byte is the type of message 
  uint8_t sender_id = incomingData[1];  // second message byte is the sender_id.
  switch (type) {
  case DATA:                           // the message is data type
    info_logger("[esp-now] message of type DATA arrived.");
    if (sender_id == CONTROLLER) 
    {
      info_logger("[esp-now] message received from CONTROLLER device.");
      memcpy(&controller_data, incomingData, sizeof(controller_data));
      // create a JSON document with received data and send it by event to the web page
      root["system_mode"] = controller_data.active_system_mode;
      root["fault_code"] = controller_data.fault_code;
      root["evap_vapor_line_temp"] = controller_data.evap_satura_temp;
      root["evap_air_in_temp"] = controller_data.evap_air_in_temp;
      root["evap_air_out_temp"] = controller_data.evap_air_out_temp;
      serializeJson(root, payload);
      info_logger(payload.c_str());
    }

    if (sender_id == MONITOR)
    {
      info_logger("[esp-now] message received from MONITOR device.");
      memcpy(&monitor_data, incomingData, sizeof(monitor_data));
      root["fault_code"] = monitor_data.fault_code;
      root["vapor_temp"] = monitor_data.vapor_temp;
      root["min_vapor_press"] = monitor_data.vapor_press[0];
      root["avg_vapor_press"] = monitor_data.vapor_press[1];
      root["max_vapor_press"] = monitor_data.vapor_press[2];
      serializeJson(root, payload);
      info_logger(payload.c_str());
    }

    break;
  
  case PAIRING:                            // the message is a pairing request 
    info_logger("[esp-now] message of type PAIRING arrived.");
    memcpy(&pairing_data, incomingData, sizeof(pairing_data));
    if (pairing_data.sender_id > 0) {     // do not replay to server itself
      pairing_data.sender_id = 0;       // 0 is server
      // Server is in AP_STA mode: peers need to send data to server soft AP MAC address 
      WiFi.softAPmacAddress(pairing_data.macAddr);   
      pairing_data.channel = WiFi_channel; // current WiFi_channel value.
      if (addPeer(mac_addr) == true){
        info_logger("[esp-now] sending response to peer with PAIRING data.");
        esp_err_t send_result = esp_now_send(mac_addr, (uint8_t *) &pairing_data, sizeof(pairing_data));
        if (send_result ==  ESP_OK) {
          info_logger("[esp-now] respnse sent.");
        } else {
          ESP_LOGW(TAG, "[esp-now] error sending pairing msg to peer, reason: %d", send_result);
        }
      };
    }
  break;
  default: error_logger("[esp-now] Invalid DATA type received...");
  }
}

void set_AP_for_ESPNOW_offline_mode() {
  // this function allows offline esp-now communication, in case of getting disconnected from the WiFi network.
  info_logger("[wifi] Setting up to communicate over esp_now while device is offline.");
  WiFi.disconnect();
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] channel: %d", WiFi.channel());
  WiFi_channel = WiFi.channel();
  WiFi.softAP(AP_SSID, AP_PASS, WiFi_channel, 1); //channel, hidden ssid
  espnow_connection_state = ESPNOW_OFFLINE;
  return;
}

void test_AP_for_ESPNOW_online_mode(){
  info_logger("[wifi] Setting up to communicate over esp_now while device is online.");
  WiFi.begin(esid, epass);
  wifi_reconnect_attempt = 0;
}


void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] event code: %d", event);
  switch (event) {
    case ARDUINO_EVENT_WIFI_READY:               info_logger("[wifi] interface ready"); break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:           info_logger("[wifi] Completed scan for access points"); break;
    case ARDUINO_EVENT_WIFI_STA_START:           info_logger("[wifi] Client started"); break;
    case ARDUINO_EVENT_WIFI_STA_STOP:            info_logger("[wifi] Clients stopped"); break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:       
      WiFi_channel = WiFi.channel();
      espnow_connection_state = ESPNOW_ONLINE;
      wifi_reconnect_attempt = 0;
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] Connected to the AP on channel: %d", WiFi_channel);
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] RSSI: %d", WiFi.RSSI());
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] SOFT AP MAC Address: %s", WiFi.softAPmacAddress().c_str());
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] STA MAC Address: %s", WiFi.macAddress().c_str());
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      if (wifi_reconnect_attempt > MAX_RECONNECT_ATTEMPTS) {
        wifi_reconnect_attempt = 0;
        info_logger("[wifi] reached max_reconnect_attempts.");
        set_AP_for_ESPNOW_offline_mode();
        break;
      }
      wifi_reconnect_attempt ++;
      info_logger("[wifi] Disconnected from WiFi Access Point"); 
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] lost connection. Reason code: %d", info.wifi_sta_disconnected.reason);
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] Reconnect attempt # %d..", wifi_reconnect_attempt);
      espnow_connection_state = ESPNOW_IDLE;
      WiFi.reconnect();
      break;

    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE: info_logger("[wifi] Authentication mode of access point has changed"); break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] IP Address assigned: %s", WiFi.localIP().toString());
      break;

    case ARDUINO_EVENT_WIFI_STA_LOST_IP:        info_logger("[wifi] Lost IP address and IP address is reset to 0"); break;
    case ARDUINO_EVENT_WPS_ER_SUCCESS:          info_logger("[wifi] Protected Setup (WPS): succeeded in enrollee mode"); break;
    case ARDUINO_EVENT_WPS_ER_FAILED:           info_logger("[wifi] Protected Setup (WPS): failed in enrollee mode"); break;
    case ARDUINO_EVENT_WPS_ER_TIMEOUT:          info_logger("[wifi] Protected Setup (WPS): timeout in enrollee mode"); break;
    case ARDUINO_EVENT_WPS_ER_PIN:              info_logger("[wifi] Protected Setup (WPS): pin code in enrollee mode"); break;
    case ARDUINO_EVENT_WIFI_AP_START:           info_logger("[wifi] access point started"); break;
    case ARDUINO_EVENT_WIFI_AP_STOP:            info_logger("[wifi] access point stopped"); break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:    info_logger("[wifi] Client connected"); break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED: info_logger("[wifi] Client disconnected"); break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:   info_logger("[wifi] Assigned IP address to client"); break;
    case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:  info_logger("[wifi] Received probe request"); break;
    case ARDUINO_EVENT_WIFI_AP_GOT_IP6:         info_logger("[wifi] AP IPv6 is preferred"); break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP6:        info_logger("[wifi] STA IPv6 is preferred"); break;
    default:                                    break;
  }
}

// Guarda el estado del sistema en spiffs
void save_operation_state_in_fs() //[OK] [OK]
{
  info_logger("[spiffs] saving operation state in file system.");
  if (!SPIFFS.begin(true))
  {
    error_logger("[spiffs] Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/Encendido.txt", "w"); // Borra el contenido anterior del archivo

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    error_logger("[spiffs] Error al abrir el archivo solicitado.");
    delay(200);
    return;
  }
  char* buffer_state = "off";
  if (SysState == ON_STATE){buffer_state = "on";}
  else if (SysState == SLEEP){buffer_state = "sleep";}
  
  f.print(buffer_state);
  f.close();
}

// Establece el encendido o apagado al inicio
void load_operation_state_from_fs() //[OK] [OK]
{
  info_logger("[spiffs] loading operation state from fs");
  if (!SPIFFS.begin(true))
  {
    error_logger("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File file = SPIFFS.open("/Encendido.txt");

  // Mensaje de fallo al leer el contenido
  if (!file)
  {
    error_logger("Error al abrir el archivo.");
    return;
  }
  String ESave = file.readString();
  file.close();

  if (ESave == "on") {SysState = ON_STATE;}
  else if (ESave == "off") {SysState = OFF_STATE;}
  else if (ESave == "sleep") {SysState = SLEEP;}
  else {error_logger("Error: Bad value stored in Encendido.txt");}

  return;
}

// update rtc with time from ntp server
void update_rtc_from_ntp() // [OK] [OK]
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

    DS3231_RTC.adjust(DateTime(ano, mes, dia, hora, minuto, segundo));
    info_logger("[RTC] DateTime updated!");
    ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[RTC] Day: %s - %s:%s", String(dia).c_str(), String(hora).c_str(), String(minuto).c_str());
  }
  else
  {
    error_logger("[RTC] Could not obtain time info from NTP Server, Skiping RTC update");
  }
}

// -- callback when time is available from ntp server
void timeavailable(struct timeval *tml) // [OK] [OK] 
{
  // this should be called every hour automatically..
  info_logger("[RTC] Got time adjustment from NTP! latest datetime is now available");
  update_rtc_from_ntp(); // update RTC with latest time from NTP server.
}

// Save timectrl settings in filesystem.
void save_timectrl_settings_in_fs(bool value, String onCondition, String offCondition) //[OK]
{
  info_logger("[spiffs] saving timectrl settings in file system.");
  String timectrl_setting;
  StaticJsonDocument<96> doc;

  doc["value"] = value;
  doc["on_condition"] = onCondition;
  doc["off_condition"] = offCondition;

  serializeJson(doc, timectrl_setting);
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "data to save in filesystem: %s", timectrl_setting.c_str());

  if (!SPIFFS.begin(true))
  {
    error_logger("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/Settings.txt", "w"); // Borra el contenido anterior del archivo

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    error_logger("Error al abrir el archivo solicitado.");
    delay(200);
    return;
  }
  f.print(timectrl_setting);
  f.close();
}

// load timectrl settings from filesystem.
void load_timectrl_settings_from_fs() //[OK]
{
  info_logger("[spiffs] loading timectrl settings from file system.");
  if (!SPIFFS.begin(true))
  {
    error_logger("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File f = SPIFFS.open("/Settings.txt");

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    error_logger("Error al abrir el archivo solicitado.");
    return;
  }
  String Settings = f.readString();
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "data loaded from file: %s", Settings.c_str());
  // String input;
  StaticJsonDocument<128> doc1;
  DeserializationError error = deserializeJson(doc1, Settings);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
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
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "%s schedule to save in fs: %s", Day.c_str(), Hours.c_str());

  if (!SPIFFS.begin(true))
  {
    error_logger("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/" + Day + ".txt", "w"); // Borra el contenido anterior del archivo
  if (!f)
  {
    error_logger("Error al abrir el archivo solicitado.");
    delay(200);
    return;
  }
  f.print(Hours);
  f.close();
}

// Aqui se guardan la configuracion del modo Auto que llega en el topico
void save_auto_config_in_fs(int wait, int temp)
{
  info_logger("[spiffs] saving auto-mode settings in file system.");
  StaticJsonDocument<32> doc;
  String output;

  doc["wait"] = wait;
  doc["temp"] = temp;

  serializeJson(doc, output);
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "data to save in fs: %s", output.c_str());

  if (!SPIFFS.begin(true))
  {
    error_logger("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/Auto.txt", "w"); // Borra el contenido anterior del archivo

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    error_logger("Error al abrir el archivo");
    delay(200);
    return;
  }
  f.println(output);
  f.close();
  AutoTimeOut = (unsigned long)wait * 60000L; // convert minutes to miliseconds
  AutoTemp = temp;                            // 24
}

// Aqui se guarda el modo que llega desde el topico
void save_operation_mode_in_fs(String Modo) //[OK]
{
  info_logger("[spiffs] saving operation mode in file system.");
  if (!SPIFFS.begin(true))
  {
    error_logger("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/Modo.txt", "w"); // Borra el contenido anterior del archivo

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    error_logger("Error al abrir el archivo");
    delay(200);
    return;
  }
  char* mode_buffer = "auto";
  if (SysMode == FAN) {mode_buffer = "fan";}
  else if (SysMode == COOL) {mode_buffer = "cool";}

  f.print(mode_buffer);
  f.close();
}

// Funcion que asigna el Modo almacenado a una variable local [x]
void load_operation_mode_from_fs() //[OK]
{
  info_logger("loading operation mode from fs");
  // Temperatura SetPoint
  if (!SPIFFS.begin(true))
  {
    error_logger("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File file = SPIFFS.open("/Modo.txt");

  // Mensaje de fallo al leer el contenido
  if (!file)
  {
    error_logger("Error al abrir el archivo.");
    return;
  }
  String ReadModo = file.readString();
  if (ReadModo == "auto") {SysMode == AUTO;}
  else if (ReadModo == "fan") {SysMode == FAN;}
  else if (ReadModo == "cool") {SysMode == COOL;}

  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "operation mode loaded: %s Enum %s", ReadModo, SysMode);

  file.close();
}

// Funcion que asigna las temperaturas predeterminadas a las variables globales
void load_temp_setpoint_from_fs()
{
  info_logger("loading NORMAL temp. setpoint from fs");
  // Temperatura SetPoint
  if (!SPIFFS.begin(true))
  {
    error_logger("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File file = SPIFFS.open("/temp.txt");

  // Mensaje de fallo al leer el contenido
  if (!file)
  {
    error_logger("Error al abrir el archivo.");
    return;
  }
  String ReadTemp = file.readString();
  SelectTemp = ReadTemp.toFloat();
  file.close();
  CurrentSysTemp = SelectTemp;
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "NORMAL Temp. loaded: %3.2f °C", CurrentSysTemp);

  // Temperatura en Auto
  info_logger("loading AUTO setpoints from fs");
  if (!SPIFFS.begin(true))
  {
    error_logger("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File f = SPIFFS.open("/Auto.txt");
  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    error_logger("Error al abrir el archivo.");
    return;
  }
  String AutoSetUp = f.readString();
  StaticJsonDocument<64> doc;
  DeserializationError error = deserializeJson(doc, AutoSetUp);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
    return;
  }

  int wait = doc["wait"];
  AutoTimeOut = (unsigned long)wait * 60000L;
  AutoTemp = doc["temp"]; // 24

  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Auto setpoints loaded = wait: %d min., temp: %3.2f °C", wait, AutoTemp);
  f.close();
}

// Guarda el Setpoint en SPIFF
void process_temp_sp_from_broker(String json) //[OK]
{
  StaticJsonDocument<96> doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
    return;
  }

  String variable = doc["variable"]; // "user_setpoint"
  int value = doc["value"];          // 24
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Temp. sp received: %d °C", value);

  if (!SPIFFS.begin(true))
  {
    error_logger("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/temp.txt", "w"); // Borra el contenido anterior del archivo

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    error_logger("Error al abrir el archivo");
    delay(200);
    return;
  }
  f.print(value);
  SelectTemp = value;
  f.close();
}

// carga los datos de la red wifi desde el spiffs.
void load_wifi_data_from_fs()
{
  info_logger("[spiffs] loading WiFi data from fs.");
  if (!SPIFFS.begin(true))
  {
    error_logger("[spiffs] Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File f = SPIFFS.open("/WiFi.txt");

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    error_logger("[spiffs] Error al abrir el archivo.");
    return;
  }
  String wifi_data = f.readString();
  // String input;

  StaticJsonDocument<256> doc1;
  DeserializationError error = deserializeJson(doc1, wifi_data);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "[JSON] Deserialization error raised with code: %s", error.c_str());
    return;
  }

  esid = doc1["ssid"];
  epass = doc1["pass"];

  f.close();
  return;
}

// guarda la configuración wifi recibida por smartConfig
void save_wifi_data_in_fs()
{
  info_logger("[spiffs] Saving WiFi data in fs");
  StaticJsonDocument<256> doc;
  String output;

  doc["ssid"] = WiFi.SSID();
  doc["pass"] = WiFi.psk();

  serializeJson(doc, output);
  if (!SPIFFS.begin(true))
  {
    error_logger("[spiffs] Ocurrió un error al ejecutar SPIFFS.");
    return;
  }
  File f = SPIFFS.open("/WiFi.txt", "w"); // Borra el contenido anterior del archivo

  // Mensaje de fallo al leer el contenido
  if (!f)
  {
    error_logger("[spiffs] Error al abrir el archivo...");
    delay(200);
    return;
  }
  f.println(output);
  f.close();
  info_logger("[spiffs] WiFi data saved!");
  return;
}

// Guarda la configuracion que se envia en el topico
void process_settings_from_broker(String json) //[OK]
{
  StaticJsonDocument<768> doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
    return;
  }

  const char *variable = doc["variable"]; // "timectrl"

  if (variable == "timectrl")
  {
    info_logger("timectrl settings adjustment.");
    // Aqui hay que guardar la configuracion del control de apagado encendido
    // Cambia el horario de encendido o apagado
    bool value = doc["enabled"];                //
    const char *onCondition = doc["on_condition"];   //
    const char *offCondition = doc["off_condition"]; //
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
      // String input;, false, true, true, true, ...
      save_schedule_in_fs(schedule_item_value_on, schedule_item_value_off, Day, schedule_item_value_enabled);
    }
  }
  else if (variable == "mode_config")
  {
    info_logger("auto mode configuration settings.");
    // Cambia la configuracion del modo
    const char *value = doc["value"]; // "auto"
    int wait = doc["wait"];           // 1234
    int temp = doc["temp"];           // 24
    save_auto_config_in_fs(wait, temp);
  }
  else if (variable == "system_mode")
  {
    info_logger("system operation mode settings");
    // Cambia el modo de operacion
    const char *value = doc["value"]; // "cool"
    save_operation_mode_in_fs(value);
  }
}

// Lee la variable de encendio o apadado de tago
void process_op_state_from_broker(String json) //[OK, OK]
{
  // String input;
  StaticJsonDocument<96> doc;
  DeserializationError error = deserializeJson(doc, json);
  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
    return;
  }
  String variable = doc["variable"]; // "system_state"
  String value = doc["value"];       // "on"
  
  if (value == "on") {SysState = ON_STATE;}
  else if (value == "off") {SysState = OFF_STATE;}
  else {
    error_logger("Error: invalid value received from broker on -system_state-");
    return;
  }
  return;
}

//-- Operation functions --

// Funcion que recibe la data de Tago por MQTT
void mqtt_message_callback(char *topicp, byte *payload, unsigned int length) //[OK]
{
  String topic = String(topicp);
  String Mensaje;
  //begin.
  digitalWrite(NETWORK_LED, LOW);
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "$ MQTT Message arrived on topic: %s", topic.c_str());
  for (int i = 0; i < length; i++)
  {
    Mensaje = Mensaje + (char)payload[i];
  }
  //print message in logger.
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "+ Message: %s", Mensaje.c_str());

  // Selecciona la funcion acorde al topico al cual llego el mensaje
  if (topic == "system/operation/settings")
  {
    // Cambia la configuracion del sistema
    info_logger("processing settings received from broker");
    process_settings_from_broker(Mensaje);
  }
  else if (topic == "system/operation/state")
  {
    // Apaga o enciende el sistema
    info_logger("processing operation state received from broker");
    process_op_state_from_broker(Mensaje);
  }
  else if (topic == "system/operation/setpoint")
  {
    // Ajusta la temperatura del ambiente
    info_logger("processing temp. setpoint received from broker");
    process_temp_sp_from_broker(Mensaje);
  }
  else {
    error_logger("mqtt topic not implemented.");
  }

  // Levanta el Flag para envio de datos
  delay(100);
  Envio = true;
  digitalWrite(NETWORK_LED, HIGH);
}

// Funcion que regula la temperatura
void temp_setpoint_controller() // [OK]
{
  info_logger("Exec. temp_setpoint_controller");
  switch (SysMode)
  {
  case AUTO:
    if (MovingSensor) {
      MovingSensorTime = millis();
      CurrentSysTemp = SelectTemp;
    }
    else if (millis() - MovingSensorTime > AutoTimeOut) {
      CurrentSysTemp == AutoTemp;
    }
    break;

  case COOL:
    CurrentSysTemp = SelectTemp;
    break;

  default:
    CurrentSysTemp = SelectTemp;
  }

  return;
}

//funcion que ajusta el estado del sistema en funcion del horario y otros parametros.
void sleep_state_controller() //[OK]
{
  info_logger("Exec. sleep_state_controller");

  if (!schedule_enabled)
  {
    info_logger("Schedule disabled.");
    Sleep = "off";
    return;
  }

  DateTime now = DS3231_RTC.now();
  String Day = Week_days[now.dayOfTheWeek()];

  if (!SPIFFS.begin(true))
  {
    error_logger("Ocurrió un error al ejecutar SPIFFS.");
    return;
  }

  File file = SPIFFS.open("/" + Day + ".txt");

  // Mensaje de fallo al leer el contenido
  if (!file)
  {
    error_logger("Error al abrir el archivo.");
    return;
  }

  String AutoOff = file.readString();
  file.close();

  StaticJsonDocument<64> doc;
  DeserializationError error = deserializeJson(doc, AutoOff);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
    return;
  }

  int WAKE_TIME = doc["ON"];   // "0730"
  int SLEEP_TIME = doc["OFF"]; // "2130"
  bool SLEEP_ENABLED = doc["enable"];
  
  if (!SLEEP_ENABLED)
  {
    Sleep = "off";
    return;
  }

  String hora = String(now.hour());
  int Min = int(now.minute());
  String minuto = "0";
  if (Min < 10)
  {
    info_logger("actual minute is lower than dec.10.");
    minuto.concat(Min);
  }
  else
  {
    info_logger("actual minute is equal to dec.10 or grater.");
    minuto = String(Min);
  }
  String tiempo = hora + minuto;
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Current Time: %s", tiempo);
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Wake Up at: %i", WAKE_TIME);
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Sleep at: %i", SLEEP_TIME);

  if (SLEEP_TIME > tiempo.toInt() && WAKE_TIME < tiempo.toInt())
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
        ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "New Sleep.var value: %s", Sleep);
        SysState = ON_STATE;
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
        ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "New Sleep.var value: %s", Sleep);
        SysState = SLEEP;
      }
    }
  }
  return;
}

// Lee temperatura
void update_ambient_temperature() // defining
{
  if (millis() - lastTempRequest >= tempRequestDelay) {
    //-
    info_logger("reading temperature from DS18B20...");
    double AmbTempBuffer = ambient_t_sensor.getTempCByIndex(0);
    ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Ambient temperature: %3.2f °C", AmbTempBuffer);
    if (AmbTempBuffer == -127)
    {
      error_logger("Error reading OneWire sensor - [Ambient temperature]");
      return;
    }
    // update global variable.
    ambient_temp = AmbTempBuffer;
    ambient_t_sensor.requestTemperatures();
    lastTempRequest = millis();
  }
  return;
}

// Este loop verifica que el wifi este conectado correctamente
void wifiloop() //[ok]
{
  // only wait for SmartConfig when the AP button is pressed.
  const bool ap_btn_pressed = digitalRead(AP_BTN) ? false : true;
  if (ap_btn_pressed)
  {
    info_logger("[wifi] the AP button has been pressed, setting up new wifi network*");
    info_logger("[wifi] Waiting for SmartConfig...");
    WiFi.disconnect();
    WiFi.beginSmartConfig();

    while (!WiFi.smartConfigDone())
    {
      network_led_state = (network_led_state == LOW) ? HIGH : LOW;
      digitalWrite(NETWORK_LED, network_led_state);
      delay(500); //wait for smart config to arrive.
    }
    info_logger("[wifi] SmartConfig received!");
    info_logger("[wifi] Testing new WiFi credentials...");

    // test wifi credentials received.
    while (WiFi.status() != WL_CONNECTED)
    {
      network_led_state = (network_led_state == LOW) ? HIGH : LOW;
      digitalWrite(NETWORK_LED, network_led_state);
      delay(500);
    }

    info_logger("[wifi] Connected to new WiFi network! smart config finished..");
    // save wifi credential in filesystem.
    save_wifi_data_in_fs();
    info_logger("[esp] rebooting device. bye");
    ESP.restart(); // restart esp.
  }
  
  // WiFi connected
  if ((WiFi.status() == WL_CONNECTED))
  {
    digitalWrite(NETWORK_LED, HIGH); // solid led indicates active wifi connection.

    if ((!mqtt_client.connected()) && (millis() - lastMqttReconnect >= mqttReconnectInterval))
    {
      // connecting to a mqtt broker
      info_logger("[mqtt] MQTT broker connection attempt..");
      lastMqttReconnect = millis();
      String client_id = "esp32-mqtt_client-";
      client_id += String(WiFi.macAddress());
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[mqtt] client id: %s", client_id.c_str());
      digitalWrite(NETWORK_LED, HIGH); //led pulse begin...
      // Try mqtt connection to the broker.
      if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password, willTopic, willQoS, willRetain, willMessage))
      {
        info_logger("[mqtt] Connected to MQTT broker!");
        // Publish and subscribe
        info_logger("[mqtt] Subscribing to mqtt topics:");
        mqtt_client.subscribe(topic);
        info_logger("[mqtt] Topic 1 ok");
        delay(1000);
        mqtt_client.subscribe(topic_2);
        info_logger("[mqtt] Topic 2 ok");
        delay(1000);
        mqtt_client.subscribe(topic_3);
        info_logger("[mqtt] Topic 3 ok");
        lastSaluteTime = millis();
        Salute = false; // flag to send connection message.
        info_logger("[mqtt] MQTT connection done. **");
      }
      else
      {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "[mqtt] Fail MQTT Connection with state: %d", mqtt_client.state());
        digitalWrite(NETWORK_LED, LOW); // led pulse end...
        return;
      }
    }

    if (!Salute && millis() - lastSaluteTime > SaluteTimer)
    {
      mqtt_client.publish("device/hello", "connected");
      Salute = true;
      info_logger("[mqtt] 'Salute' message sent");
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
    return;
  }
}

// Atualiza estado de entradas y salidas digitales.
void update_IO() //[ok]
{

  // Lectura de sensor de movimiento.
  MovingSensor = digitalRead(RADAR) ? true : false;
  // Apaga o enciende el aire depediendo del estado
  const bool btn_manual = digitalRead(MANUAL_BTN) ? false : true; // input = 0 means button pressed
  if (btn_manual && millis() - lastButtonPress > buttonTimeOut)
  {
    info_logger("Manual Button has been pressed..");

    switch (SysState)
    {
    case ON_STATE:
      info_logger("- Turning off the system.");
      SysState = OFF_STATE;
      break;

    case OFF_STATE:
      info_logger("Turning on the system.");
      SysState = ON_STATE;
      break;
    
    case SLEEP:
      info_logger("imposible to turn on the system on sleep mode.");
      break;
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
  save_operation_state_in_fs();

  char* sysState_buffer = "off";
  switch (SysState)
  {
  case ON_STATE:
    sysState_buffer = "on";
    break;
  
  case SLEEP:
    sysState_buffer = "sleep";
    break;
  }

  String pass = mqtt_password;
  String message = sysState_buffer;
  message += ",";
  message += pass;
  message += ",";
  message += PrevSysState;
  info_logger("Notifying MQTT broker on System State update..");
  // sending message.
  bool message_sent = mqtt_client.publish("device/stateNotification", message.c_str());
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "MQTT publish result: %s", message_sent ? "message sent!" : "fail");
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

  String timectrl;
  timectrl = schedule_enabled ? "on": "off";
  // json todas las variables, falta recoleccion
  String output;
  StaticJsonDocument<256> doc;

  doc["variable"] = "ac_control";
  doc["value"] = tdecimal;

  JsonObject metadata = doc.createNestedObject("metadata");
  metadata["user_setpoint"] = SelectTemp;
  metadata["active_setpoint"] = CurrentSysTemp;
  metadata["sys_state"] = SysState;
  metadata["sys_mode"] = SysMode;
  metadata["presence"] = digitalRead(RADAR);
  metadata["timectrl"] = timectrl;

  serializeJson(doc, output);
  info_logger("Publishing system variables to mqtt broker.");
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Message: %s", output.c_str());
  bool mqtt_msg_sent = mqtt_client.publish("tago/data/post", output.c_str());
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "MQTT publish result: %s", mqtt_msg_sent ? "message sent!" : "fail");

  //update posting interval value
  switch (SysState)
  {
  case ON_STATE:
    postingInterval = 2L * 30000L; // 1 minuto.
    break;
  
  default:
    postingInterval = 5L * 60000L; // 5 minutos.
    break;
  }

  Envio = false;
  lastConnectionTime = millis();
}

// Hace la lectura y envio de datos.
void sensors_read(void *pvParameters)
{

  const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
  for (;;)
  { 
    if (millis() - lastControllerTime > controllerInterval)
    {
      // Funcion que controla el apagado y encendido automatico (Sleep)
      sleep_state_controller();
      // Funcion que regula latemperatura segun el modo (Cool, auto, fan)
      temp_setpoint_controller();

      // logging
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "System state: %s", SysState);
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Presence sensor: %s", MovingSensor ? "detecting": "empty room");

      // update time var
      lastControllerTime = millis();
    }

    // flag to send data to tago.io
    if (millis() - lastConnectionTime > postingInterval)
    {
      Envio = true;
    }

    //lee temperatura ambiente
    update_ambient_temperature();
    // Update inputs and outputs
    update_IO();
    //notify the user about state update...
    notify_state_update_to_broker();
    //send all the sensor data to the mqtt broker
    send_data_to_broker();

    // task delay
    vTaskDelay(xDelay);
  }
}

// -- Setup
void setup()
{
  Serial.begin(115200);
  esp_log_level_set("*", ESP_LOG_DEBUG); //set all logger on debug.
  info_logger("** Hello!, System setup started... **");
  // pins definition
  // pinMode(BROKER_LED, OUTPUT);          // broker connection led.
  pinMode(NETWORK_LED, OUTPUT);         // network connection led.
  pinMode(MANUAL_BTN, INPUT); // remote board button.
  pinMode(RADAR, INPUT);  // sensor de presencia
  pinMode(AP_BTN, INPUT);            // Wifi Restart and configuration.

#ifndef ESP32
  while (!Serial)
    ; // wait for serial port to connect. Needed for native USB
#endif

  info_logger("Setting up RTC on I2C bus.");
  if (!DS3231_RTC.begin())
  {
    error_logger("Couldn't find RTC, please reboot.");
    while (1)
      ;
  }
  if (DS3231_RTC.lostPower())
  {
    info_logger("RTC is NOT running!, setting up on sketch compiled datetime.");
    // following line sets the RTC to the date & time this sketch was compiled
    DS3231_RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  info_logger("RTC ok.");

  // Inicio Sensores OneWire
  info_logger("OneWire sensors configuration!");
  ambient_t_sensor.begin();
  ambient_t_sensor.setResolution(ds_sensor_resolution);
  ambient_t_sensor.setWaitForConversion(false);
  ambient_t_sensor.requestTemperatures();
  lastTempRequest = millis();
  tempRequestDelay = 750 / (1 << (12 - ds_sensor_resolution));
  info_logger("OneWire sensors ok.");

  // set default values from .txt files
  info_logger("Reading stored values from file system. (SPIFFS)");
  load_temp_setpoint_from_fs();     // user-temp and auto-temp
  load_operation_state_from_fs();   // on-off setting
  load_operation_mode_from_fs();    // function mode (cool, auto, fan)
  load_timectrl_settings_from_fs(); // timectrl settings
  load_wifi_data_from_fs();         // load wifi data from filesystem
  info_logger("SPIFF system ok.");
  // --- continue

  // WifiSettings
  info_logger("WiFi settings and config.");
  WiFi.onEvent(WiFiEvent);
  WiFi.disconnect();
  delay(1000);            // 1 second delay
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 1); //channel 1, hidden ssid
  WiFi.begin(esid, epass);
  info_logger("WiFi settings ok.");
  //---------------------------------------- esp_now settings
  info_logger("Setting up ESP_NOW");
  if (esp_now_init() != ESP_OK) {
    error_logger("-- Error initializing ESP-NOW, please reboot --");
    while (1){;}
  }
  // register esp_callbacks.
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  info_logger("ESP_NOW Setup Complete!");
  //---------------------------------------- mqtt settings
  info_logger("Setting up MQTT");
  mqtt_client.setBufferSize(mqttBufferSize);
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(mqtt_message_callback);
  info_logger("Mqtt settings done.");
  //---------------------------------------- datetime from ntp server
  info_logger("Creating NTP server configuration.");
  sntp_set_time_sync_notification_cb(timeavailable);        // sntp sync interval is 1 hour.
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); // configura tiempo desde el servidor NTP

  // Crea la tarea de lectura de sensores en el segundo procesador.
  info_logger("Creating 2nd core task...");
  xTaskCreatePinnedToCore(
      sensors_read, /* Function to implement the task */
      "Task1",     /* Name of the task */
      10000,       /* Stack size in words */
      NULL,        /* Task input parameter */
      0,           /* Priority of the task */
      &Task1,      /* Task handle. */
      0);

  //---------------------------------------- end of setup ---
  info_logger("** Setup completed **");
}

void loop()
{
  wifiloop();
  mqtt_client.loop();
  delay(10);
}