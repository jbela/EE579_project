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

//(max CCW) 1023 <-- 0 (stop) 1024 --> 2047 (max CW)
const int VELOCITY_CW = 1224;
const int VELOCITY_CCW = 200;

uint8_t IDs[] = {1, 2, 3, 4, 5, 6};
int offset[] = {150, 150, 150, 60, 60, 60};
float phase_difference[] = {0, 0, 0, 0, 0, 0};
bool direction[] = {1, 0, 1, 0, 1, 0};

const int gait_cycle_ms = 1000;
const float oscillation_range = 90.0;

float get_angle(int i, int gait_cycle_time) {
    float cycle_percentage = phase_difference[i] + gait_cycle_time / (float) gait_cycle_ms;

    if(cycle_percentage > 1) {
        cycle_percentage -= 1;
    }

    float angle = (oscillation_range / 2.0) * sin(2.0 * PI * cycle_percentage);

    if(direction[i] == 1) {
        angle = - angle + offset[i];
    } else {
        angle = angle + offset[i];
    }

    if(angle >= 360.0) {
        angle -= 360.0;
    } else if(angle < 0.0) {
        angle += 360;
    }

    return angle;
}

unsigned long start = 0;
DynamixelShield dxl;

void setup() {
    DEBUG_SERIAL.begin(115200);

    dxl.begin(1000000);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    for(int i = 0; i < sizeof(IDs); i++){
        dxl.ping(IDs[i]);
        dxl.torqueOff(IDs[i]);
        if(i < 2) {
            dxl.setOperatingMode(IDs[i], OP_POSITION);
        } else {
            dxl.setOperatingMode(IDs[i], OP_VELOCITY);
        }
        dxl.torqueOn(IDs[i]);
    }
    start = millis();
}

int prev_time = 0;

void loop() {
    unsigned long elapsed = millis() - start;

    int gait_cycle_time = elapsed - prev_time;
    if(gait_cycle_time > gait_cycle_ms) {
        prev_time = elapsed;
        gait_cycle_time = 0;
    }

    for(int i = 0; i < 2; i++) {
        float angle = get_angle(i, gait_cycle_time);
        dxl.setGoalAngle(IDs[i], angle);
    }

    for(int i = 2; i < 6; i++) {
        int velocity = direction[i] ? VELOCITY_CW : VELOCITY_CCW;
        dxl.setGoalVelocity(IDs[i], velocity);
    }
}