// logging
#include <esp_log.h>
static const char* TAG = "main";
// Libreria conexion wifi y tago
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <SPI.h>
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
const char willTopic[] = "achub/device/lastwill";
const int willQoS = 0;
const bool willRetain = false;
const char willMessage[] = "disconnected";
uint16_t mqttBufferSize = 768; // LuisLucena, bufferSize in bytes

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
unsigned long postingInterval = 1L * 60000L;   // delay between updates, in milliseconds.. 1 minute and updates later on.
unsigned long radarStateTime = 0;
unsigned long AutoTimeOut = 0;                           // wati time for moving sensor and setpoint change
unsigned long lastTempRequest = 0;
unsigned long lastRadarChange = 0;
const unsigned long radarDebounceTime = 1L * 60000L;      // 1 minute rebound for radar sensor.
const unsigned long buttonTimeOut = 5L * 1000L;          // rebound 5seconds
const unsigned long controllerInterval = 2L * 5000L;     // delay between sensor updates, 10 seconds
const unsigned long SaluteTimer = 1L * 30000L;           // Tiempo para enviar que el dispositivo esta conectado,
const unsigned long wifiReconnectInterval = 5L * 60000L;  // 5 minutos para intentar reconectar al wifi.
const unsigned long mqttReconnectInterval = 1L * 10000L; // 10 segundos para intentar reconectar al broker mqtt.
const unsigned long wifiDisconnectedLedInterval = 250;        // 250 ms

// MQTT Broker
const char *mqtt_broker = TEST_MQTT_BROKER;
const char *settings_topic = "achub/system/operation/settings";
const char *opstate_topic = "achub/system/operation/state";
const char *opsetpoint_topic = "achub/system/operation/setpoint";
const char *peer_list_topic = "achub/system/espnow/peer";
const char *mqtt_username = "Token";
const char *mqtt_password = MQTT_PASSWORD;
const int mqtt_port = 1883;
bool Envio = false;

// Handler de tareas de lectura de sensores para nucleo 0 o 1
TaskHandle_t Task1;

// WIFI VARIABLES
String esid = "";
String epass = "";
String hubDeviceID = "";
const char* AP_SSID = "AC-HUB";
const char* AP_PASS = AP_PASSWORD;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);
int wiFiReconnectAttempt = 0;
const int MAX_RECONNECT_ATTEMPTS = 33;

// RTC
RTC_DS3231 DS3231_RTC;
char Week_days[7][12] = {"Domingo", "Lunes", "Martes", "Miercoles", "Jueves", "Viernes", "Sabado"};

// Variables de configuracion
// Enum classes
enum SysModeEnum {AUTO_MODE, FAN_MODE, COOL_MODE};
enum SysStateEnum {SYSTEM_ON, SYSTEM_OFF, SYSTEM_SLEEP, UNKN};
enum FlowFlag {FLAG_UP, FLAG_DOWN, FLAG_UNSET};
enum SleepWakeCondition {SLEEP_ON_TIME, SLEEP_ON_ABSENCE, WAKE_ON_TIME, WAKE_ON_PRESENCE};
// Enum vars
SysStateEnum SysState = UNKN;
SysStateEnum PrevSysState = UNKN;
SysModeEnum SysMode = AUTO_MODE;
FlowFlag sleep_flag = FLAG_UNSET;
SleepWakeCondition sleep_condition = SLEEP_ON_TIME;
SleepWakeCondition wake_condition = WAKE_ON_TIME;

//--
float userSetpoint;
float AutoSetpoint;
float activeSetpoint;
bool Salute = false;
bool radarState = false;
bool lastRadarState = false;
bool sleepControlEnabled; // Variables de activacion del modo sleep
double ambient_temp = 22;
int tempSensorResolution = 12; //bits
int tempRequestDelay = 0;

// NTP
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -4 * 3600;
const int daylightOffset_sec = 0;

// ESP-NOW VARS
esp_now_peer_info_t slaveTemplate;
int32_t WiFi_channel = 1;
enum MessageTypeEnum {PAIRING, DATA,};
enum PeerRoleID {SERVER, CONTROLLER, MONITOR_A, MONITOR_B, UNSET};
enum EspNowState {ESPNOW_OFFLINE, ESPNOW_ONLINE, ESPNOW_IDLE,};
EspNowState espnow_connection_state = ESPNOW_IDLE;
// MessageTypeEnum espnow_msg_type;
// SystemModes espnow_system_mode;
// PeerRoleID espnow_peer_id;
typedef struct pairing_data_struct {
  MessageTypeEnum msg_type;             // (1 byte)
  PeerRoleID sender_role;       // (1 byte)
  uint8_t macAddr[6];           // (6 bytes)
  uint8_t channel;              // (1 byte)
} pairing_data_struct;          // TOTAL = 9 bytes

typedef struct controller_data_struct {
  MessageTypeEnum msg_type;    // (1 byte)
  PeerRoleID sender_role;       // (1 byte)
  uint8_t fault_code;      // (1 byte)
  float air_return_temp;   // (4 bytes) [°C]
  float air_inyect_temp;   // (4 bytes) [°C]
  bool drain_switch;       // (1 byte)
  bool cooling_relay;      // (1 byte)
  bool turbine_relay;      // (1 byte)
} controller_data_struct;  // TOTAL = 14 bytes

typedef struct monitor_data_struct {
  MessageTypeEnum msg_type;     // (1 byte)
  PeerRoleID sender_role;       // (1 byte)
  uint8_t fault_code;           // (1 byte) 0-no_fault; 1..255 monitor_fault_codes.
  float vapor_temp;             // (4 bytes) vapor line temperature readings [°C]
  float low_pressure[3];        // (12 bytes) min - avg - max | low pressure readings [psi]
  float discharge_temp;         // (4 bytes) discharge temperature readings [°C]
  float condenser_temp;         // (4 bytes) condenser saturated temp readings [°C]
  float liquid_temp;            // (4 bytes) liquid line temperature readings [°C]
  float compressor_amps[3];     // (12 bytes) min - avg - max | compressor_current readings [A]
  bool compressor_state;        // (1 byte)
  uint32_t running_time;        // (3 bytes) compressor hour-meeter. [hours]
} monitor_data_struct;          // TOTAL = 47 bytes

typedef struct outgoing_settings_struct {
  MessageTypeEnum msg_type;     // (1 byte)
  PeerRoleID sender_role;       // (1 byte)
  SysModeEnum system_mode;      // (1 byte)
  SysStateEnum system_state;    // (1 byte)
  float system_temp_sp;         // (4 bytes) [°C]
  float room_temp;              // (4 bytes) [°C]
} outgoing_settings_struct;     // TOTAL = 9 bytes

outgoing_settings_struct settings_data;
pairing_data_struct pairing_data;
controller_data_struct controller_data;
monitor_data_struct monitor_01;
monitor_data_struct monitor_02;

//logger functions

void debug_logger(const char *message) {
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "%s", message);
}

void info_logger(const char *message) {
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "%s", message);
}

void error_logger(const char *message) {
  ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "%s", message);
}

//fs functions.
// función que carga una String que contiene toda la información dentro del target_file del SPIFFS.
String load_data_from_fs(const char *target_file) {
  //-
  if(!SPIFFS.begin(true)) {
    error_logger("Ocurrió un error al ejecutar SPIFFS.");
    return "null";
  }

  File f = SPIFFS.open(target_file);
  if (!f) {
    error_logger("Error al abrir el archivo solicitado.");
    return "null";
  }

  String file_string = f.readString();
  f.close();

  //log
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "data loaded from SPIFFS: %s", file_string.c_str());
  return file_string;
}

// función que guarda una String en el target_file del SPIFFS.
void save_data_in_fs(String data_to_save, const char *target_file) {
  //savin data in filesystem.
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "saving in:%s this data:%s", target_file, data_to_save.c_str());
  if(!SPIFFS.begin(true)) {
    error_logger("Ocurrió un error al ejecutar SPIFFS.");
  }

  File f = SPIFFS.open(target_file, "w");
  if (!f){
    error_logger("Error al abrir el archivo solicitado.");
    return;
  }

  f.print(data_to_save);
  f.close();
  info_logger("data saved correctly in SPIFFS.");
  return;
}

//esp-now functions.
// función que actualiza la lista de pares guardada en el SPIFFS.
void update_peer_list_in_fs(String new_pl_data) {

  info_logger("updating peer info in filesystem.");
  String current_pl = load_data_from_fs("/Peer.txt");
  JsonDocument current_json_pl;
  JsonDocument new_json_pl_data;
  String output_string;
  DeserializationError error = deserializeJson(current_json_pl, current_pl);
  DeserializationError error_1 = deserializeJson(new_json_pl_data, new_pl_data);

  if (error || error_1)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
  }

  const char *controller_mac = new_json_pl_data["controller"] | "null";
  const char *monitor_01_mac = new_json_pl_data["monitor_01"] | "null";
  const char *monitor_02_mac = new_json_pl_data["monitor_02"] | "null";

  if (strcmp(controller_mac, "null") != 0) //don't match
  {
    info_logger("new mac address for controller received!");
    current_json_pl["controller"] = controller_mac;
  }
  if (strcmp(monitor_01_mac, "null") != 0) //don't match
  {
    info_logger("new mac address for monitor_01 received!");
    current_json_pl["monitor_01"] = monitor_01_mac;
  }

  if (strcmp(monitor_02_mac, "null") != 0) //don't match
  {
    info_logger("new mac address for monitor_02 received!");
    current_json_pl["monitor_02"] = monitor_02_mac;
  }

  //serialize new data.
  serializeJson(current_json_pl, output_string);
  save_data_in_fs(output_string, "/Peer.txt");

  return;
}

// función que verifica que la dirección mac solicitada está guardada en spiffs.
// y devuelve el rol del dispositivo.
PeerRoleID get_peer_role_from_fs(String target_dev_id){
  //-
  String peer_list = load_data_from_fs("/Peer.txt");
  JsonDocument json_doc;
  DeserializationError error = deserializeJson(json_doc, peer_list);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
    return UNSET;
  }

  const char * controller_mac_saved = json_doc["controller"] | "null";
  if (strcmp(controller_mac_saved, target_dev_id.c_str()) == 0){
    info_logger("device found as controller.");
    return CONTROLLER;
  }

  const char * monitor_01_mac_saved = json_doc["monitor_01"] | "null";
  if (strcmp(monitor_01_mac_saved, target_dev_id.c_str()) == 0){
    info_logger("device found as monitor_01");
    return MONITOR_A;
  }

  const char * monitor_02_mac_saved = json_doc["monitor_02"] | "null";
  if (strcmp(monitor_02_mac_saved, target_dev_id.c_str()) == 0){
    info_logger("device found as monitor_02");
    return MONITOR_B;
  }

  info_logger("device not found in SPIFFS!");
  return UNSET;

}

// get device id from device mac address.
String get_device_id(const uint8_t * mac_addr) {
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x%02x%02x%02x%02x%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // return mac_str;
  String str = (char*) mac_str;
  return str;
}

// add peer to peer list.
bool addPeer(const uint8_t *peer_addr) {      // add pairing
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[esp-now] adding new peer to peer list");

  //reset slaveTemplate variable
  memset(&slaveTemplate, 0, sizeof(slaveTemplate));
  //create reference to slaveTemplate memory loc.
  const esp_now_peer_info_t *peer = &slaveTemplate;

  //set values in peer template
  memcpy(slaveTemplate.peer_addr, peer_addr, 6);
  slaveTemplate.channel = WiFi_channel; // pick a channel.. 0 means it take the current STA channel (connected to AP)
  slaveTemplate.encrypt = 0; // no encryption

  // check if the peer exists and remove it from peerlist
  bool exists = esp_now_is_peer_exist(slaveTemplate.peer_addr);
  if (exists) {
    // Slave already paired.
    info_logger("peer already exists, deleting existing data.");
    esp_err_t deleteStatus = esp_now_del_peer(peer_addr);
    if (deleteStatus == ESP_OK) {
      info_logger("peer deleted!");
    } else {
      error_logger("error deleting peer!");
      return false;
    }
  }

  // save peer in peerlist
  esp_err_t addPeerResult = esp_now_add_peer(peer);
  switch (addPeerResult)
  {
  case ESP_OK:
    info_logger("new peer added successfully");
    return true;
  
  default:
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "Error: %d, while adding new peer.", addPeerResult);
    return false;
  }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  String device_id = get_device_id(mac_addr);
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[esp-now] packet sent to: %s", device_id.c_str());

  switch (status)
  {
  case ESP_NOW_SEND_SUCCESS:
    info_logger("[esp-now] message delivered!");
    break;
  
  default:
  error_logger("[esp-now] message fail delivering!");
    break;
  }
}


void OnDataRecv(const uint8_t * mac_addr, const uint8_t * incomingData, int len) { 
  //logging
  String device_id = get_device_id(mac_addr);
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[esp-now] %d bytes of data received from: %s", len, device_id.c_str());
  
  //no response on IDLE state
  if (espnow_connection_state == ESPNOW_IDLE) {
    info_logger("[esp-now] Waiting to finish AP connection. Ignoring Data..");
    return;
  }
  JsonDocument root;
  String payload;
  uint8_t type = incomingData[0];       // first message byte is the type of message 
  uint8_t sender_role = incomingData[1];  // second message byte is the sender_role.
  switch (type) {
  case DATA:                           // the message is data type
    info_logger("[esp-now] message of type DATA arrived.");
    if (sender_role == CONTROLLER) 
    {
      info_logger("[esp-now] message received from CONTROLLER device.");
      memcpy(&controller_data, incomingData, sizeof(controller_data));
      // create a JSON document with received data and send it by event to the web page
      root["air_return_temp"] = controller_data.air_return_temp;
      root["air_inyect_temp"] = controller_data.air_inyect_temp;
      root["cooling_relay"] = controller_data.cooling_relay;
      root["turbine_relay"] = controller_data.turbine_relay;
      root["drain_switch"] = controller_data.drain_switch;
      root["fault_code"] = controller_data.fault_code;
      serializeJson(root, payload);
      info_logger(payload.c_str());
    }

    if (sender_role == MONITOR_A)
    {
      info_logger("[esp-now] message received from MONITOR_A device.");
      memcpy(&monitor_01, incomingData, sizeof(monitor_01));
      root["fault_code"] = monitor_01.fault_code;
      root["vapor_temp"] = monitor_01.vapor_temp;
      root["low_pressure"][0] = monitor_01.low_pressure[0];
      root["low_pressure"][1] = monitor_01.low_pressure[1];
      root["low_pressure"][2] = monitor_01.low_pressure[2];
      root["discharge_temp"] = monitor_01.discharge_temp;
      root["condenser_temp"] = monitor_01.condenser_temp;
      root["liquid_temp"] = monitor_01.liquid_temp;
      root["comp_current"][0] = monitor_01.compressor_amps[0];
      root["comp_current"][1] = monitor_01.compressor_amps[1];
      root["comp_current"][2] = monitor_01.compressor_amps[2];
      root["compressor_state"] = monitor_01.compressor_state;

      serializeJson(root, payload);
      info_logger(payload.c_str());
    }

    break;
  
  case PAIRING:                            // the message is a pairing request 
    info_logger("[esp-now] message of type PAIRING arrived.");
    memcpy(&pairing_data, incomingData, sizeof(pairing_data));
    if (pairing_data.sender_role > 0) {     // do not replay to server itself
      pairing_data.sender_role = SERVER;       // 0 is server
      pairing_data.channel = WiFi_channel; // current WiFi_channel value.

      PeerRoleID peer_role = get_peer_role_from_fs(get_device_id(mac_addr));
      if (peer_role == UNSET) {
        info_logger("[esp-now] device unset, try again later.");
        return;
      }

      if (addPeer(mac_addr) == true){
        info_logger("[esp-now] sending response to peer with PAIRING data.");
        esp_err_t send_result = esp_now_send(mac_addr, (uint8_t *) &pairing_data, sizeof(pairing_data));

        switch (send_result)
        {
        case ESP_OK:
          info_logger("[esp-now] pairing response sent.");
          break;
        
        default:
          ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "[esp-now] error sending pairing msg to peer, reason: %d",  send_result);
          break;
        }
      }
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
  WiFi.begin(esid.c_str(), epass.c_str());
  wiFiReconnectAttempt = 0;
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
      wiFiReconnectAttempt = 0;
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] Connected to the AP on channel: %d", WiFi_channel);
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] RSSI: %d", WiFi.RSSI());
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] SOFT AP MAC Address: %s", WiFi.softAPmacAddress().c_str());
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] STA MAC Address: %s", WiFi.macAddress().c_str());
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      if (wiFiReconnectAttempt > MAX_RECONNECT_ATTEMPTS) {
        wiFiReconnectAttempt = 0;
        info_logger("[wifi] reached max_reconnect_attempts.");
        set_AP_for_ESPNOW_offline_mode();
        break;
      }
      wiFiReconnectAttempt ++;
      info_logger("[wifi] Disconnected from WiFi Access Point"); 
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] lost connection. Reason code: %d", info.wifi_sta_disconnected.reason);
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[wifi] Reconnect attempt # %d..", wiFiReconnectAttempt);
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
  const char * fileName = "/Encendido.txt";

  switch (SysState)
  {
  case SYSTEM_ON:
    save_data_in_fs("on", fileName);
    break;
  
  case SYSTEM_OFF:
    save_data_in_fs("off", fileName);
    break;

  case SYSTEM_SLEEP:
    save_data_in_fs("sleep", fileName);
    break;

  }
}

// Establece el encendido o apagado al inicio
void load_operation_state_from_fs() //[OK] [OK]
{
  // operation state.
  String ESave = load_data_from_fs("/Encendido.txt");

  if (ESave == "on") {SysState = SYSTEM_ON;}
  else if (ESave == "off") {SysState = SYSTEM_OFF;}
  else if (ESave == "sleep") {SysState = SYSTEM_SLEEP;}
  else {error_logger("Error: Bad value stored in /Encendido.txt");}

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
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "[RTC] Day: %s - %s:%s", String(dia).c_str(), String(hora).c_str(), String(minuto).c_str());
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
void save_timectrl_settings_in_fs() //[OK]
{
  String timectrl_setting;
  JsonDocument doc;

  doc["value"] = sleepControlEnabled;
  if (wake_condition == WAKE_ON_TIME) {doc["on_condition"] = "on_time";} else {doc["on_condition"] = "presence";}
  if (sleep_condition == SLEEP_ON_TIME) {doc["off_condition"] = "on_time";} else {doc["off_condition"] = "presence";}

  serializeJson(doc, timectrl_setting);
  save_data_in_fs(timectrl_setting, "/Settings.txt");
  return;
}

// load timectrl settings from filesystem.
void load_timectrl_settings_from_fs() //[OK]
{
  String Settings = load_data_from_fs("/Settings.txt");
  // String input;
  JsonDocument jsonSettings;
  DeserializationError error = deserializeJson(jsonSettings, Settings);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
    return;
  }

  sleepControlEnabled = jsonSettings["value"];
  if (strcmp(jsonSettings["on_condition"], "on_time") == 0) {
    wake_condition = WAKE_ON_TIME;} else {wake_condition = WAKE_ON_PRESENCE;
  }
  if (strcmp(jsonSettings["off_condition"], "on_time") == 0) {
    sleep_condition = SLEEP_ON_TIME;} else {sleep_condition = SLEEP_ON_ABSENCE;
  }

  return;
}

// Guarda el horario de apgado de cada dia
void save_schedule_in_fs(int HON, int HOFF, String Day, bool enable) //[OK]
{
  String Hours;
  String target_file = "/" + Day + ".txt";
  JsonDocument doc;

  doc["ON"] = HON;
  doc["OFF"] = HOFF;
  doc["enable"] = enable;

  serializeJson(doc, Hours);
  save_data_in_fs(Hours, target_file.c_str());
  return;
}

// Aqui se guardan la configuracion del modo Auto que llega en el topico
void save_auto_config_in_fs(int wait, int temp)
{
  JsonDocument doc;
  String output;

  doc["wait"] = wait;
  doc["temp"] = temp;
  serializeJson(doc, output);
  save_data_in_fs(output, "/Auto.txt");

  AutoTimeOut = (unsigned long)wait * 60000L; // convert minutes to miliseconds
  AutoSetpoint = temp;                            // 24

  return;
}

// Aqui se guarda el modo que llega desde el topico
void save_operation_mode_in_fs() //[OK]
{
  const char *fileName = "/Modo.txt";
  switch (SysMode)
  {
  case AUTO_MODE:
    save_data_in_fs("auto", fileName);
    break;

  case FAN_MODE:
    save_data_in_fs("fan", fileName);
    break;

  case COOL_MODE:
    save_data_in_fs("cool", fileName);
    break;
  }
  return;
}

// Funcion que asigna el Modo almacenado a una variable local [x]
void load_operation_mode_from_fs() //[OK]
{
  String ReadModo = load_data_from_fs("/Modo.txt");
  if (ReadModo == "auto") {SysMode = AUTO_MODE;}
  else if (ReadModo == "fan") {SysMode = FAN_MODE;}
  else if (ReadModo == "cool") {SysMode = COOL_MODE;}

  return;
}

// Funcion que asigna las temperaturas predeterminadas a las variables globales
void load_temp_setpoint_from_fs()
{
  // Temperature SetPoint

  String ReadTemp = load_data_from_fs("/temp.txt");
  String AutoSetUp = load_data_from_fs("/Auto.txt");
  JsonDocument doc;

  userSetpoint = ReadTemp.toFloat();
  activeSetpoint = userSetpoint;

  DeserializationError error = deserializeJson(doc, AutoSetUp);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
    return;
  }

  int wait = doc["wait"];
  AutoTimeOut = (unsigned long)wait * 60000L;
  AutoSetpoint = doc["temp"]; // 24

  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Auto setpoints loaded = wait: %d min., temp: %3.2f °C", wait, AutoSetpoint);
}

// Guarda el Setpoint en SPIFF
void process_temp_sp_from_broker(String json) //[OK]
{
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
    return;
  }

  const char *variable = doc["variable"] | "null"; // "user_setpoint"
  const int temp_value = doc["value"] | 0;                    // 24
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Temp. sp received: %d °C", temp_value);

  if (strcmp(variable, "user_setpoint") != 0) {
    error_logger("Invalid json variable, expected: 'user_setpoint'");
    return;
  }

  if (temp_value < 16 || temp_value > 28) {
    error_logger("Invalid temperature range for 'user_setpoint' variable");
    return;
  }
  //save data in filesystem
  char buffer[10];
  const char *temp_string = itoa(temp_value, buffer, 10);
  save_data_in_fs(buffer, "/temp.txt");
  userSetpoint = temp_value;
}

// carga los datos de la red wifi desde el spiffs.
void load_wifi_data_from_fs()
{
  String wifi_data = load_data_from_fs("/WiFi.txt");
  // String input;

  JsonDocument doc1;
  DeserializationError error = deserializeJson(doc1, wifi_data);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "[JSON] Deserialization error raised with code: %s", error.c_str());
    return;
  }
  String ESID = doc1["ssid"];
  String EPAS = doc1["pass"];
  esid = ESID;
  epass = EPAS;

  return;
}

// guarda la configuración wifi recibida por smartConfig
void save_wifi_data_in_fs()
{
  JsonDocument doc;
  String jsonData;

  doc["ssid"] = WiFi.SSID();
  doc["pass"] = WiFi.psk();

  serializeJson(doc, jsonData);
  save_data_in_fs(jsonData, "/WiFi.txt");
  return;
}

// Guarda la configuracion que se envia en el topico
void process_settings_from_broker(String json) //[OK]
{
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
    return;
  }

  const char *variable = doc["variable"] | "null";

  if (strcmp(variable, "timectrl") == 0)
  {
    info_logger("timectrl settings adjustment.");
    // Aqui hay que guardar la configuracion del control de apagado encendido
    // Cambia el horario de encendido o apagado
    sleepControlEnabled = doc["enabled"];
    const char *on_condition = doc["on_condition"];
    const char *off_condition = doc["off_condition"];

    if (strcmp(on_condition, "on_time") == 0) {
      wake_condition = WAKE_ON_TIME;} else {wake_condition = WAKE_ON_PRESENCE;
    }
    if (strcmp(off_condition, "on_time") == 0) {
      sleep_condition = SLEEP_ON_TIME;} else {sleep_condition = SLEEP_ON_ABSENCE;
    }
    save_timectrl_settings_in_fs();

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

  else if (strcmp(variable, "mode_config") == 0)
  {
    info_logger("auto mode configuration settings.");
    // Cambia la configuracion del modo
    const char *value = doc["value"]; // "auto"
    const int wait = doc["wait"] | 0;           // 1234
    const int temp = doc["temp"] | 0;           // 24

    if (strcmp(value, "auto") != 0) {
      error_logger("invalid value in json, expected: 'auto'");
      return;
    } 

    if (wait <= 0 || wait > 60) {
      error_logger("invalid range for wait value");
      return;
    }

    if (temp < 16 || temp > 28) {
      error_logger("invalid range for temp value");
      return;
    }

    save_auto_config_in_fs(wait, temp);
  }

  else if (strcmp(variable, "system_mode") == 0)
  {
    info_logger("system operation mode settings");
    // Cambia el modo de operacion
    const char *value = doc["value"]; // "cool"

    if (strcmp(value, "cool") == 0) {SysMode = COOL_MODE;}
    else if (strcmp(value, "fan") == 0) {SysMode = FAN_MODE;}
    else if (strcmp(value, "auto") == 0) {SysMode = AUTO_MODE;}
    else {error_logger("Invalid value mode in json");}
    
    // save in filesystem.
    save_operation_mode_in_fs();
  }

  else
  {
    error_logger("Invalid variable value in json document");
  }
}

// Lee la variable de encendio o apadado de tago
void process_op_state_from_broker(String json) //[OK, OK]
{
  // String input;
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);
  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
    return;
  }
  const char *variable = doc["variable"]; // "system_state"
  const char *value = doc["value"];

  if (strcmp(variable, "system_state") != 0) {
    error_logger("Error: Invalid variable name, 'system_state' expected.");
  }
  
  if (strcmp(value,"on") == 0) {SysState = SYSTEM_ON;}
  else if (strcmp(value, "off") == 0) {SysState = SYSTEM_OFF;}
  else {error_logger("Error: invalid value received from broker on -system_state-");}
  return;
}

//-- Operation functions --

// Funcion que regula la temperatura
void temp_setpoint_controller() // [OK]
{
  info_logger("* Exec. temp setpoint controller function");
  switch (SysMode)
  {
  case AUTO_MODE:
    if (radarState) { // restart the counter if the radar state is true (movement detection)
      radarStateTime = millis();
    }
    if (millis() - radarStateTime > AutoTimeOut) {
      info_logger("system Temp. adjust = 'AutoSetpoint'");
      activeSetpoint = AutoSetpoint;
    } else {
      activeSetpoint = userSetpoint;
    }
    break;

  case COOL_MODE:
    activeSetpoint = userSetpoint;
    info_logger("system Temp. adjust = 'UserTemp'");
    break;

  case FAN_MODE:
    activeSetpoint = userSetpoint;
    info_logger("system Temp. adjust = 'UserTemp'");
    break;

  }
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "System Temp. value: %2.1f", activeSetpoint);
}

// Funcion que recibe la data de Tago por MQTT
void mqtt_message_callback(char *message_topic, byte *payload, unsigned int length) //[OK]
{
  String Mensaje;
  //begin.
  digitalWrite(NETWORK_LED, LOW);
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "$ MQTT Message arrived on topic: %s", message_topic);
  for (int i = 0; i < length; i++)
  {
    Mensaje = Mensaje + (char)payload[i];
  }
  //print message in logger.
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "+ Message: %s", Mensaje.c_str());

  // Selecciona la funcion acorde al topico al cual llego el mensaje
  if (strcmp(message_topic, settings_topic) == 0)
  {
    // Cambia la configuracion del sistema
    info_logger("processing settings received from broker");
    process_settings_from_broker(Mensaje);
  }
  else if (strcmp(message_topic, opstate_topic) == 0)
  {
    // Apaga o enciende el sistema
    info_logger("processing operation state received from broker");
    process_op_state_from_broker(Mensaje);
  }
  else if (strcmp(message_topic, opsetpoint_topic) == 0)
  {
    // Ajusta la temperatura del ambiente
    info_logger("processing temp. setpoint received from broker");
    process_temp_sp_from_broker(Mensaje);
    //update system temp setpoint.
    temp_setpoint_controller();
  }
  else if (strcmp(message_topic, peer_list_topic) == 0)
  {
    // actualiza la lista de pares en el spiffs.
    info_logger("processing peer update.");
    update_peer_list_in_fs(Mensaje);
  }
  else {
    error_logger("mqtt topic not implemented.");
  }

  // Levanta el Flag para envio de datos
  delay(100);
  Envio = true;
  digitalWrite(NETWORK_LED, HIGH);
}

//funcion que ajusta el estado del sistema en funcion del horario y otros parametros.
void sleep_state_controller() //[OK]
{
  info_logger("* Excec. sleep state controller function.");
  if (!sleepControlEnabled)
  {
    info_logger("- Schedule disabled.");
    sleep_flag = FLAG_DOWN;
    return;
  }

  DateTime now = DS3231_RTC.now();
  String Day = Week_days[now.dayOfTheWeek()];
  String target_file = "/" + Day + ".txt";
  JsonDocument doc;

  String AutoOff = load_data_from_fs(target_file.c_str());
  DeserializationError error = deserializeJson(doc, AutoOff);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "JSON Deserialization error raised with code: %s", error.c_str());
    return;
  }

  const int WAKE_TIME = doc["ON"];   // "0730"
  const int SLEEP_TIME = doc["OFF"]; // "2130"
  const bool SLEEP_ENABLED = doc["enable"];
  
  if (!SLEEP_ENABLED)
  {
    sleep_flag = FLAG_DOWN;
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
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "Current Time: %s", tiempo);
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "Wake Up at: %i", WAKE_TIME);
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "Sleep at: %i", SLEEP_TIME);

  if (SLEEP_TIME > tiempo.toInt() && WAKE_TIME < tiempo.toInt()) // horario para el sleep...
  {
    // Si el sleep está activado, o si es la primera verificación después del arranque..
    if (sleep_flag == FLAG_UP || sleep_flag == FLAG_UNSET)
    {
      switch (wake_condition)
      {
      case WAKE_ON_PRESENCE:
        if (radarState) {
          sleep_flag = FLAG_DOWN;
          SysState = SYSTEM_ON;
        }
        break;
      
      case WAKE_ON_TIME:
        sleep_flag = FLAG_DOWN;
        SysState = SYSTEM_ON;
        break;
      }
    }
  }
  else
  { // horario fuera del sleep.
    // si el sleep está apagado, o si es la primera verificación después del arranque..
    if (sleep_flag == FLAG_DOWN || sleep_flag == FLAG_UNSET)
    {
      switch (sleep_condition)
      {
      case SLEEP_ON_ABSENCE:
        if(!radarState){
          sleep_flag = FLAG_UP;
          SysState = SYSTEM_SLEEP;
        }
        break;
      
      case SLEEP_ON_TIME:
        sleep_flag = FLAG_UP;
        SysState = SYSTEM_SLEEP;
        break;
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
  const bool apBtnPressed = digitalRead(AP_BTN) ? false : true;
  if (apBtnPressed)
  {
    info_logger("[wifi] the AP button has been pressed, setting up the new wifi network*");
    info_logger("[wifi] Waiting for SmartConfig...");
    WiFi.disconnect();
    WiFi.beginSmartConfig();

    while (!WiFi.smartConfigDone())
    {
      info_logger("[wiFi] Waiting SmartConfig data...");
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

    info_logger("[wifi] Connected to new the network! smart config finished..");
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
      String client_id = "ac-hub-";
      client_id += hubDeviceID;
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[mqtt] client id: %s", client_id.c_str());
      digitalWrite(NETWORK_LED, HIGH); //led pulse begin...
      // Try mqtt connection to the broker.
      if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password, willTopic, willQoS, willRetain, willMessage))
      {
        info_logger("[mqtt] Connected to MQTT broker!");
        // Publish and subscribe
        info_logger("[mqtt] Subscribing to mqtt topics:");
        mqtt_client.subscribe(settings_topic);
        info_logger("[mqtt] Topic 1 ok");
        mqtt_client.subscribe(opstate_topic);
        info_logger("[mqtt] Topic 2 ok");
        mqtt_client.subscribe(opsetpoint_topic);
        info_logger("[mqtt] Topic 3 ok");
        mqtt_client.subscribe(peer_list_topic);
        info_logger("[mqtt] Topic 4 ok");
        lastSaluteTime = millis();
        Salute = false; // flag to send connection message.
        info_logger("[mqtt] MQTT connection done. **");
      }
      else
      {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "[mqtt] Fail MQTT Connection with state: %d", mqtt_client.state());
        return;
      }
    }

    if (!Salute && millis() - lastSaluteTime > SaluteTimer)
    {
      mqtt_client.publish("achub/device/hello", "connected");
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
  const bool currentRadarReading = digitalRead(RADAR);
  const bool manualBtnPressed = digitalRead(MANUAL_BTN) ? false : true; // input = 0 means button pressed

  if (currentRadarReading != lastRadarState) {
    lastRadarChange = millis();
    lastRadarState = currentRadarReading;
  }

  // Lectura de sensor de movimiento.
  if (millis() - lastRadarChange > radarDebounceTime) {
    radarState = currentRadarReading;
    // radarState has been updated after radarDebounceTime period. this prevents false presence/absence readings.
  }

  // Apaga o enciende el sistema depediendo del estado, previene cambiar el estado durante el buttonTimeOut.
  if (manualBtnPressed && millis() - lastButtonPress > buttonTimeOut)
  {
    info_logger("Manual Button has been pressed..");

    switch (SysState)
    {
    case SYSTEM_ON:
      info_logger("- Turning off the system.");
      SysState = SYSTEM_OFF;
      break;

    case SYSTEM_OFF:
      info_logger("Turning on the system.");
      SysState = SYSTEM_ON;
      break;
    
    case SYSTEM_SLEEP:
      info_logger("imposible to turn on the system on sleep mode.");
      break;
    }

    lastButtonPress = millis();
  }
}

//notifica al broker el cambio de estado del sistema.
void notify_state_update_to_broker()
{
  if (PrevSysState == SysState || !mqtt_client.connected())
  {
    return;
  }

  String message;
  JsonDocument doc;
  doc["variable"] = "ac-hub";
  doc["value"] = "stateNotification";
  doc["metadata"]["systemPrevState"] = PrevSysState;
  doc["metadata"]["systemNewState"] = SysState;
  doc["metadata"]["deviceID"] = hubDeviceID;
  serializeJson(doc, message);

  PrevSysState = SysState; // assign PrevSysState the current SysState
  save_operation_state_in_fs();

  info_logger("Notifying MQTT broker on System State update..");
  // sending message.
  bool message_sent = mqtt_client.publish("achub/device/stateNotification", message.c_str());
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
  // json todas las variables, falta recoleccion
  String output;
  JsonDocument doc;

  doc["variable"] = "ac-hub";
  doc["value"] = tdecimal;
  doc["metadata"]["deviceID"] = hubDeviceID;
  doc["metadata"]["hub"][0] = SysState;
  doc["metadata"]["hub"][1] = SysMode;
  doc["metadata"]["hub"][2] = radarState;
  doc["metadata"]["hub"][3] = sleepControlEnabled;
  doc["metadata"]["hub"][4] = userSetpoint;
  doc["metadata"]["hub"][5] = activeSetpoint;

  serializeJson(doc, output);
  info_logger("Publishing system variables to mqtt broker.");
  bool mqtt_msg_sent = mqtt_client.publish("achub/tago/data/post", output.c_str());
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "MQTT publish result: %s", mqtt_msg_sent ? "message sent!" : "fail");

  //update posting interval value
  switch (SysState)
  {
  case SYSTEM_ON:
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
      //lee temperatura ambiente
      update_ambient_temperature();
      // Funcion que controla el apagado y encendido automatico (Sleep)
      sleep_state_controller();
      // Funcion que regula latemperatura segun el modo (Cool, auto, fan)
      temp_setpoint_controller();

      // logging
      ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Radar State: %s", radarState ? "Detecting": "Empty room");
      switch (SysState)
      {
      case SYSTEM_ON:
        info_logger("System State: RUNNING");
        break;
      
      case SYSTEM_OFF:
        info_logger("System State: OFF");
        break;

      case SYSTEM_SLEEP:
        info_logger("System State: SLEEP");
        break;
      }
      // SysMode feedback.
      switch (SysMode)
      {
      case AUTO_MODE:
        info_logger("System Mode: AUTO");
        break;

      case COOL_MODE:
        info_logger("System Mode: COOL");
        break;

      case FAN_MODE:
        info_logger("System Mode: FAN");
        break;
      }

      // update time var
      lastControllerTime = millis();
    }

    // flag to send data to tago.io
    if (millis() - lastConnectionTime > postingInterval)
    {
      Envio = true;
    }

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
  ambient_t_sensor.setResolution(tempSensorResolution);
  ambient_t_sensor.setWaitForConversion(false);
  ambient_t_sensor.requestTemperatures();
  lastTempRequest = millis();
  tempRequestDelay = 750 / (1 << (12 - tempSensorResolution));
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
  delay(500);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 1); //channel 1, hidden ssid
  WiFi.begin(esid.c_str(), epass.c_str());
  info_logger("WiFi settings ok.");
  //---------------------------------------- get device identifier
  uint8_t hubMacAddress[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_AP, hubMacAddress);
  if (ret != ESP_OK) {
    error_logger("could not read mac address.");
  }
  hubDeviceID = get_device_id(hubMacAddress);
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