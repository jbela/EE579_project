#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

const float DXL_PROTOCOL_VERSION = 2.0;

uint8_t IDs[]={2,3,101,7,2,3}; // first 2 are back leg, the rest are wheels
int zeroing_offset[]={60,240,150,60,60}; 
bool in_dead_zone[]={0,0,0,0,0,0}; //0=not, 1=in // TODO: make sure this is updates for number of motots

//for the wheels
const int CCW_VEL = 500; // (max CCW) 1023 <-- 0 (stop) 1024 --> 2047 (max CW)
const int CW_VEL = 1500; // (max CW) 1023 <-- 0 (stop) 1024 --> 2047 (max CW)

//for the legs
unsigned long start = 0.0;
const unsigned long period = 2000; //period of gait

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for(int i=0;i < sizeof(IDs); i++){
    dxl.ping(IDs[i]);
    dxl.torqueOff(IDs[i]);
    if(i < 2) {
      dxl.setOperatingMode(IDs[i], OP_POSITION);
    }
    else {
      dxl.setOperatingMode(IDs[i], OP_VELOCITY);
    }
    dxl.torqueOn(IDs[i]);
  }
  start = millis();
}



gait_phase(int time, int leg) {
  //we will set the neutral stright back position as the zero position
  float position = time * 90.0; // this is a placeholder val
  position -= 45.0; // ??? is this correct to center the zero?

  // TODO might need to fmod this by 360.
  if (leg == 0) {
    position = position+zeroing_offset[0];
  }
  else {
    position = position -zeroing_offset[1];
  }
  dxl.setGoalPosition(IDs[leg], position, UNIT_DEGREE);

}

void leg_gait() { // TODO: update this gait function to support our project's gait.
  unsigned long elapsed = millis() - start;
  float time = fmod((float)elapsed, period) / (float)period; // from 0 to 1

  // first leg
  gaitPhase(time, 0);

  float time2 = time + 0.5; // phase shift :TODO
  if (time2 > 1.0) {
    time2 -= 1.0;
  }

  gaitPhase(time2, 1);  //other leg

 
}


void loop() {

  // gait();
  unsigned long elapsed = millis() - start;
  float time = fmod((float)elapsed, period) / (float)period;
  
 //TODO: the gait for the legs
 
 //TDOD: clean this up once we figure out which wheel is which. 
  dxl.setGoalVelocity(IDs[2], CW_VEL); //(max CCW) 1023 <-- 0 (stop) 1024 --> 2047 (max CW)
  dxl.setGoalVelocity(IDs[3], CCW_VEL); //(max CCW) 1023 <-- 0 (stop) 1024 --> 2047 (max CW)
  dxl.setGoalVelocity(IDs[4], CCW_VEL); //(max CCW) 1023 <-- 0 (stop) 1024 --> 2047 (max CW)
}

/*
- how to measure "speed"
- what is considered "better performance"
- what params are we varying
- 
*/
