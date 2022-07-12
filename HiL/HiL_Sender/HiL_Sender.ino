// Lolin32 1: 58:BF:25:36:DC:A4
// Lolin32 2: 58:BF:25:36:BF:80

#include "esp_now.h"
#include "WiFi.h"

// ********** User Defines ********** //
#define DebugOn 1

#if DebugOn == 1
#define debugInit(...) Serial.begin(__VA_ARGS__)
#define debug(...)  Serial.print(__VA_ARGS__)
#define debugln(...) Serial.println(__VA_ARGS__)
#else
#define debugInit(...)
#define debug(...)
#define debugln(...)
#endif

// ********** Global Variables ********* //
// Devices MAC Addresses
const uint8_t lolin1_addr[] = {0x58,0xbf,0x25,0x36,0xdc,0xa4};
const uint8_t lolin2_addr[] = {0x58,0xbf,0x25,0x36,0xbf,0x80};
// Broadcast to all
const uint8_t broadcast_addr[] = {0xff,0xff,0xff,0xff,0xff,0xff}; 

// Data struct definitions
typedef struct{
  float x1;
  float x2;
  float x3;
} struct_report;  // state report

typedef struct{
  float ctrl;  
} struct_control; // control value

typedef struct{
  float x1o_s;
  float x2o_s;
  float x3o_s;
  float ctrlo_s;
  float C1_s;
  float C2_s;
  float L_s;
  float R_s;
  float m0_s;
  float m1_s;
  float Bp_s;
} struct_iniCond; // initial conditons reset

// Data struct instance
struct_report myReport;
struct_control myControl;
struct_iniCond mySetting;

// Peer struct instance (for adding new peers to the network)
esp_now_peer_info_t peerInfo;

// ********** User variables ********** //
// State variables
float x1, x2, x3;

// Message string
char dataBuff[35];

// ********** User Functions ********** //
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int Len){
  memcpy(&myReport, incomingData, sizeof(myReport));
}

void setup() {
  // Initailizing serial communication
  debugInit(115200);

  // Setting WiFi mode as Station
  WiFi.mode(WIFI_STA);

  // Initializing ESP-NOW
  if(esp_now_init() != ESP_OK){
    // Terminate application on failure
    debugln("Error stating ESP-NOW");
    return;
  }

  // Adding a new peer - using peer_info struct
  memcpy(peerInfo.peer_addr, lolin2_addr ,6); // Setting peer address
  peerInfo.channel = 0;     // Setting communication channel
  peerInfo.encrypt = false; // Setting encryption

  // Adding the new peer
  if(esp_now_add_peer(&peerInfo) != ESP_OK){
    // Terminate application on failure
    debugln("Failed to add a peer");
    return;
  }

  // Register callback functions
  esp_now_register_recv_cb(OnDataRecv);
  myReport.x1 = 1.0;
  myReport.x2 = -0.2;
  myReport.x3 = 0.0;

  mySetting.C1_s=23.5e-6;
  mySetting.C2_s=235e-6;
  mySetting.L_s=47;
  mySetting.R_s=1800;
  mySetting.m0_s=-0.409e-3;
  mySetting.m1_s=-0.758e-3;
  mySetting.Bp_s=1.8;

  mySetting.x1o_s = myReport.x1;
  mySetting.x2o_s = myReport.x2;
  mySetting.x3o_s = myReport.x3;
  
  mySetting.ctrlo_s = 0.0;

  esp_now_send(lolin2_addr,(uint8_t*)&mySetting,sizeof(mySetting));
}

void loop() {
  myControl.ctrl = 0.0;
  esp_now_send(lolin2_addr,(uint8_t*)&myControl,sizeof(myControl));
  sprintf(dataBuff,"%2.6f,%2.6f,%2.6f",myReport.x1,myReport.x2,myReport.x3);
  debugln(dataBuff);
  delay(500);
}
