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

uint8_t IDs[]={2,3,101,7};  //enlarge this array to control additional motors
int Leg_zeroing_offset[]={60,240,150,150}; 


unsigned long start = 0.0;
const unsigned long period = 4000; //period of gait

const float L1 = 6.5; //leg lengths
const float L2 = 7.3;

// for 0-100 (part1)
const float t_0 = 0;
const float t_100 = 100;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for(int i=0;i<4;i++){
    dxl.ping(IDs[i]);
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_POSITION);
    dxl.torqueOn(IDs[i]);
  }
  start = millis();
}


void inverse(float x, float y, float &t1, float &t2) {
  float r2 = x*x + y*y;
  float in = (r2 - L1*L1 - L2*L2) / (2*L1*L2);
  in = constrain(in, -1.0, 1.0);
  t2 = acos(in);
  t1 = atan2(y,x) - atan2(L2*sin(t2), L1+L2*cos(t2));
}


void move(float x, float y, int leg = 0) {
  int hipIndex  = leg * 2;
  int kneeIndex = leg * 2 + 1;
  float t1, t2, hip_deg, knee_deg;
  inverse(x,y,t1,t2);

  if (leg == 0) {
    hip_deg  = Leg_zeroing_offset[hipIndex] - (t1 * 180.0 / PI);
    knee_deg = Leg_zeroing_offset[kneeIndex] + (t2 * 180.0 / PI);
  }
  if (leg == 1) { 
    hip_deg  = Leg_zeroing_offset[hipIndex] + (t1 * 180.0 / PI);
    knee_deg = Leg_zeroing_offset[kneeIndex] - (t2 * 180.0 / PI); 
  }
  dxl.setGoalPosition(IDs[hipIndex],  hip_deg,  UNIT_DEGREE);
  dxl.setGoalPosition(IDs[kneeIndex], knee_deg, UNIT_DEGREE);
}


void forward(float t1, float t2, float &x, float &y) {
  x = L1*cos(t1) + L2*cos(t1 + t2);
  y = L1*sin(t1) + L2*sin(t1 + t2);
}


// this is currently the gait for lab 2.
void gaitPhase(float time, int leg) { // TODO: updat this gait function to support our project's gait. 

  float R  = 2.0;
  float cx = 2.0; //center x&y
  float cy = 10.0;

  float x, y;

  //basically same as semicircle.
  if (time < 0.5) {
    float t = time / 0.5; 
    float angle = PI * t;

    x = cx + R * cos(angle);
    y = cy - R * sin(angle); 
  }
  else {
    float t = (time - 0.5) /0.5;
    x = (2*R*t);
    y = cy;
  }

  move(x, y, leg);
}


void gait() { // TODO: update this gait function to support our project's gait.
  unsigned long elapsed = millis() - start;
  float time = fmod((float)elapsed, period) / (float)period;

  // first leg
  gaitPhase(time, 0);

  float time2 = time + 0.5; // phase shift
  if (time2 > 1.0) {
    time2 -= 1.0;
  }

  gaitPhase(time2, 1);  //other leg

  //add in rotating the front wheels.
 
}


void loop() {

  // gait();

}
