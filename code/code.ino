/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
// this code was written from the starter code for position_mode

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

// Leg1 for parts 1&2
const uint8_t LEG_ID = 3;
const uint8_t HIP_ID = 2;

//for part 2&3
uint8_t IDs[]={2,3,101,7}; 
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
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  // dxl.ping(HIP_ID);
  // dxl.ping(LEG_ID);

  // // Turn off torque when configuring items in EEPROM area
  // dxl.torqueOff(LEG_ID);
  // dxl.torqueOff(HIP_ID);

  // // dxl.setOperatingMode(DXL_ID, OP_POSITION); // do this for both 
  // dxl.setOperatingMode(HIP_ID, OP_POSITION);
  // dxl.setOperatingMode(LEG_ID, OP_POSITION);

  // // dxl.torqueOn(DXL_ID); // do this for both
  // dxl.torqueOn(LEG_ID);
  // dxl.torqueOn(HIP_ID);

  // for later part. can comment out prior setup code.
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

void move(float x, float y, int leg = 3) {
  int hipIndex  = leg * 2;
  int kneeIndex = leg * 2 + 1;
  float t1, t2;
  inverse(x,y,t1,t2);

  if (leg == 0) {
    float hip_deg  = Leg_zeroing_offset[hipIndex] - (t1 * 180.0 / PI);
    float knee_deg = Leg_zeroing_offset[kneeIndex] + (t2 * 180.0 / PI);
    dxl.setGoalPosition(IDs[hipIndex],  hip_deg,  UNIT_DEGREE);
    dxl.setGoalPosition(IDs[kneeIndex], knee_deg, UNIT_DEGREE);
  }

  if (leg == 1) { // may or may not need to use based on the second leg. - depends on legs
    float hip_deg  = Leg_zeroing_offset[hipIndex] + (t1 * 180.0 / PI);
    float knee_deg = Leg_zeroing_offset[kneeIndex] - (t2 * 180.0 / PI); 
    dxl.setGoalPosition(IDs[hipIndex],  hip_deg,  UNIT_DEGREE);
    dxl.setGoalPosition(IDs[kneeIndex], knee_deg, UNIT_DEGREE);
  }
  
  if (leg == 3){ // basically the default - so i can repeat the task1&2 functions without editing function calls
    t1 = Leg_zeroing_offset[0] + (t1*180 / 3.1415);
    t2 = Leg_zeroing_offset[1] - (t2*180 / 3.1415);
    dxl.setGoalPosition(HIP_ID, t1, UNIT_DEGREE);
    dxl.setGoalPosition(LEG_ID, t2, UNIT_DEGREE);
    DEBUG_SERIAL.print("theta1=");
    DEBUG_SERIAL.print(t1);
    DEBUG_SERIAL.print(" theta2=");
    DEBUG_SERIAL.println(t2);
  }
}


void forward(float t1, float t2, float &x, float &y) {
  x = L1*cos(t1) + L2*cos(t1 + t2);
  y = L1*sin(t1) + L2*sin(t1 + t2);
}


void draw_rectangle() {
  int steps = 10;
  float dt = 1.0 / steps;
  float x,y;
  move(0,8);
  delay(100);

  // this was last-minute code written to get full 10 pts for demo
  // not the best way of doing this. 
  for (int i = 1; i <= 10; i++) {
    move(0,8+(2*i*dt));
    delay(100);
  }
  for (int i = 1; i <= 10; i++) {
    move(0+4*i*dt,10);
    delay(100);
  }
  for (int i = 1; i <= 10; i++) {
    move(4,10-(2*i*dt));
    delay(100);
  }
  for (int i = 1; i <= 10; i++) {
    move(4-(4*i*dt),8);
    delay(100);
  }
  //original method - just the 4 points. 
  // delay(1000);
  // move(0, 10);
  // delay(1000);
  // move(4,10);
  // delay(1000);
  // move(4,8);
  // delay(1000);
  // move(0,8);
}

void draw_semicircle() {
  float R = 2;
  float cx = 2;
  float cy = 10;

  int steps = 100;
  float dt = 1.0 / steps;
  float x,y;
  for (int i = steps; i >= 0; i--) {

    float t = i * dt;
    float angle = PI * t;
    x = cx + R * cos(angle);
    y = cy - R * sin(angle);

    move(x, y);
    delay(20);
  }

  for (int i = 0; i <= steps; i++) {
    float x = 2*R - (2*R*i /steps); // draw from end point of semicircle to start pt
    move(x, y);
    delay(20);
  }
}


void gaitPhase(float time, int leg) {

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

void part_1(bool option=false){
  float t1 = t_0+Leg_zeroing_offset[0];
  float t2 = Leg_zeroing_offset[1]-t_0; 
  if (option) { // option == true, then we do inverse 100-0
    t2 = Leg_zeroing_offset[1]-t_100;
  }

  int count = 0;   
  while(count<=100) {
    float inv_t2=360.0-t2;
    dxl.setGoalPosition(HIP_ID, t1, UNIT_DEGREE);
    dxl.setGoalPosition(LEG_ID, t2, UNIT_DEGREE);
    DEBUG_SERIAL.print("theta1=");
    DEBUG_SERIAL.print(t1);
    DEBUG_SERIAL.print(" theta2=");
    DEBUG_SERIAL.println(t2);
    t1+=1.0;
    if (option) {
      t2+=1.0; // inverse the theta 2 to be 100-0
    }
    else {
      t2-=1.0;
    }
    // SWITCH this when doing inverse -> t2+=1.0;
    count+=1;
    delay(50); 
  }
}

void gait() {
  unsigned long elapsed = millis() - start;
  float time = fmod((float)elapsed, period) / (float)period;

  // first leg
  gaitPhase(time, 0);

  float time2 = time + 0.5; // phase shift
  if (time2 > 1.0) {
    time2 -= 1.0;
  }

  gaitPhase(time2, 1);  //other leg
 
}


void loop() {
  // put your main code here, to run repeatedly:

  // part_1();
  // delay(5000);
  
  // part_1(true);
  // delay(5000);

  // draw_rectangle();
  //delay(5000);

  // draw_semicircle();
  // delay(5000);

  // gait();

}
