// Lolin32 1: 58:BF:25:36:DC:A4
// Lolin32 2: 58:BF:25:36:BF:80

#include "esp_now.h"
#include "WiFi.h"

// ********** User Defines ********** //
#define DebugOn 1

#if DebugOn == 1
#define debugInit(x) Serial.begin(x)
#define debug(x)  Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debugInit(x)
#define debug(x)
#define debugln(x)
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

// ********** User Variables ********** //

// Time step
float T=0.001;
//unsigned long tstart, tend;
bool setFlag;

// Initial conditions
float x1o = 1.0;
float x2o = -0.2;
float x3o = 0.0;

// State variables
float x1, x2, x3;
float ix1, ix2, ix3;

// Control
float ctrl;

// Model variables
float C1;
float C2;
float L;
float R;
float m0;
float m1;
float Bp;

// Runge-Kutta auxiliar variables
float k_11, k_21, k_31, k_41;
float k_12, k_22, k_32, k_42;
float k_13, k_23, k_33, k_43;
float x1_aux1, x2_aux1, x3_aux1, phi1;
float x1_aux2, x2_aux2, x3_aux2, phi2;
float x1_aux3, x2_aux3, x3_aux3, phi3;
float x1_aux4, x2_aux4, x3_aux4, phi4;

// ********** User Functions ********** //
// Runge-Kutta computation
void calcChuaRungeKutta(){ 
  
  // Auxiliar variables update. 
  x1_aux1 = x1;
  x2_aux1 = x2;
  x3_aux1 = x3;
  
  // K1 computation
  phi1 = m0*x1_aux1+0.5*(m1-m0)*(abs(x1_aux1+Bp)-abs(-x1_aux1+Bp));
  k_11 = (1/(C1*R))*(x2_aux1-x1_aux1)-(1/C1)*phi1;
  k_12 = (1/(C2*R))*(x1_aux1-x2_aux1)-(1/C2)*x3_aux1;
  k_13 = (1/L)*x2_aux1;

  // f_1(x+T/2)
  x1_aux2 = x1 + k_11*(T/2);
  x2_aux2 = x2 + k_12*(T/2);
  x3_aux2 = x3 + k_13*(T/2);

  // K2 computation
  phi2 = m0*x1_aux2+0.5*(m1-m0)*(abs(x1_aux2+Bp)-abs(-x1_aux2+Bp));
  k_21 = (1/(C1*R))*(x2_aux2-x1_aux2)-(1/C1)*phi2;
  k_22 = (1/(C2*R))*(x1_aux2-x2_aux2)-(1/C2)*x3_aux2;
  k_23 = (1/L)*x2_aux2;
  
  // f_2(x+T/2)
  x1_aux3 = x1 + k_21*(T/2);
  x2_aux3 = x2 + k_22*(T/2);
  x3_aux3 = x3 + k_23*(T/2);

  // K3 computation
  phi3 = m0*x1_aux3+0.5*(m1-m0)*(abs(x1_aux3+Bp)-abs(-x1_aux3+Bp));
  k_31 = (1/(C1*R))*(x2_aux3-x1_aux3)-(1/C1)*phi3;
  k_32 = (1/(C2*R))*(x1_aux3-x2_aux3)-(1/C2)*x3_aux3;
  k_33 = (1/L)*x2_aux3;

  // f_3(x+T)
  x1_aux4 = x1 + k_31*T;
  x2_aux4 = x2 + k_32*T;
  x3_aux4 = x3 + k_33*T;

  // K4 computation
  phi4 = m0*x1_aux4+0.5*(m1-m0)*(abs(x1_aux4+Bp)-abs(-x1_aux4+Bp));
  k_41 = (1/(C1*R))*(x2_aux4-x1_aux4)-(1/C1)*phi4;
  k_42 = (1/(C2*R))*(x1_aux4-x2_aux4)-(1/C2)*x3_aux4;
  k_43 = (1/L)*x2_aux4;

  // Final computation;
  ix1 = x1 + ((k_11 + (2*k_21) + (2*k_31) + k_41)*(T/6));
  ix2 = x2 + ((k_12 + (2*k_22) + (2*k_32) + k_42)*(T/6));
  ix3 = x3 + ((k_13 + (2*k_23) + (2*k_33) + k_43)*(T/6));

  x1=ix1;
  x2=ix2;
  x3=ix3;
}

// Report state values using ESP-Now
void sendStateReport(){
  myReport.x1 = x1;
  myReport.x2 = x2;
  myReport.x3 = x3;
  
  esp_now_send(broadcast_addr,(uint8_t*)&myReport,sizeof(myReport));
}

// ESP-Now reception
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int Len){
  if(Len == sizeof(myControl)){
    memcpy(&myControl, incomingData, sizeof(myControl));
    ctrl = myControl.ctrl;
  }
  else{
    memcpy(&mySetting, incomingData, sizeof(mySetting));
    x1=mySetting.x1o_s;  
    x2=mySetting.x2o_s;  
    x3=mySetting.x3o_s;
    
    ctrl = mySetting.ctrlo_s;
    
    C1=mySetting.C1_s;
    C2=mySetting.C2_s;
    L=mySetting.L_s;
    R=mySetting.R_s;
    m0=mySetting.m0_s;
    m1=mySetting.m1_s;
    Bp=mySetting.Bp_s; 
  }
  setFlag = true;
}

// ********** Setup Function ********** //
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
  memcpy(peerInfo.peer_addr, broadcast_addr ,6); // Setting peer address
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

  // Setting model initial conditions
  x1=x1o;  x2=x2o;  x3=x3o;
  ctrl = 0;
    
  C1=23.5e-6;
  C2=235e-6;
  L=47;
  R=1800;
  m0=-0.409e-3;
  m1=-0.758e-3;
  Bp=1.8;

  setFlag = false;
}

void loop() {
  if(setFlag){ 
    calcChuaRungeKutta();
    sendStateReport();
    setFlag = false;
  }
}
