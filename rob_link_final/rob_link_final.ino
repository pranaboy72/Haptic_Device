#include <DynamixelShield.h>
#include <stdlib.h>
#include <math.h>


// Please modify it to suit your hardware.
//#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
//  SoftwareSerial soft_serial(0, 1);
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
//  #define DEBUG_SERIAL soft_serial
//#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
//  #define DEBUG_SERIAL SerialUSB
//#else
//  #define DEBUG_SERIAL Serial
//#endif

const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

const uint8_t ID_1 = 1; //CONTROLLER JOINT1
const uint8_t ID_2 = 2; //ROBOT JOINT1
const uint8_t ID_3 = 3; //ROBOT JOINT2
const uint8_t ID_4 = 4; //CONTROLLER JOINT2
const uint8_t ID_5 = 5; //ROBOT ROTATION1
const uint8_t ID_6 = 6; //CONTROLLER ROTATION
const uint8_t ID_7 = 7; //ROBOT ROTATION2
const uint8_t ID_8 = 8;
const uint8_t ID_9 = 9;
const uint8_t ID_10 = 10;
const int32_t BAUDRATE = 57600;

double a = 68.73, b = 46.82, c = 14.25;
double A, B, E, F, X, Y;
double theta2, theta3, theta5, alpha, beta, gamma, rot_ang;
double vel_1, vel_0, pos, vel1_1, vel0_1;
double back1, back2;
int i;

const int AXIS_X = A0;    // 0~1023의 아날로그값.
const int AXIS_Y = A1;    // Y값만 사용.
const int SW_P = 5;

//int DEBUG = 1;

using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxl.ping(ID_1);
  dxl.ping(ID_2);
  dxl.ping(ID_3);
  dxl.ping(ID_4);
  dxl.ping(ID_5);
  dxl.ping(ID_6);
  dxl.ping(ID_7);
  dxl.ping(ID_8);
  dxl.ping(ID_9);
  dxl.ping(ID_10);

  dxl.torqueOff(ID_1);
  dxl.torqueOff(ID_2);
  dxl.torqueOff(ID_3);
  dxl.torqueOff(ID_4);
  dxl.torqueOff(ID_5);
  dxl.torqueOff(ID_6);
  dxl.torqueOff(ID_7);
  dxl.torqueOff(ID_8);
  dxl.torqueOff(ID_9);
  dxl.torqueOff(ID_10);

  dxl.setOperatingMode(ID_1,  OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_2, OP_POSITION);
  dxl.setOperatingMode(ID_3, OP_POSITION);
  dxl.setOperatingMode(ID_4, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_5, OP_VELOCITY);
  dxl.setOperatingMode(ID_6, OP_VELOCITY);
  dxl.setOperatingMode(ID_7, OP_VELOCITY);
  dxl.setOperatingMode(ID_8, OP_VELOCITY);
  dxl.setOperatingMode(ID_9, OP_VELOCITY);
  dxl.setOperatingMode(ID_10, OP_VELOCITY);


  soft_serial.begin(115200);
  soft_serial.println("Set up complete");

  i = 0;

  dxl.torqueOn(ID_2);
  dxl.torqueOn(ID_3);
  dxl.torqueOn(ID_5);
  dxl.torqueOn(ID_7);
  dxl.torqueOn(ID_8);
  dxl.torqueOn(ID_9);
  dxl.torqueOn(ID_10);

  pinMode(SW_P, INPUT_PULLUP);
}

void loop() {
  for_back();

  get_angle();

  // robot의 각도를 직접 측정하여 구함
  theta3 = theta3 + 40;
  theta2 = 370.0 - theta2;

  vel_1 = dxl.getPresentVelocity(ID_1);
  vel_0 = dxl.getPresentVelocity(ID_4);

  /* 이 부분 로봇의 모터 위치에 맞게 수정 필요 */
  dxl.setGoalPosition(ID_2, theta3, UNIT_DEGREE);
  dxl.setGoalPosition(ID_3, theta2, UNIT_DEGREE);
  dxl.setGoalVelocity(ID_5, rot_ang, UNIT_RPM);
  dxl.setGoalVelocity(ID_7, -rot_ang, UNIT_RPM);

  back1 = dxl.getPresentPosition(ID_1, UNIT_DEGREE);
  back2 = dxl.getPresentPosition(ID_4, UNIT_DEGREE);

    if ((dxl.getPresentVelocity(ID_2) == 0 || dxl.getPresentVelocity(ID_3) == 0) && (vel_1 * dxl.getPresentVelocity(ID_1) > 0 || vel_0 * dxl.getPresentVelocity(ID_4))){
      // 로봇이 못 움직이는 경우
      while(1){
        for_back();
        dxl.setGoalVelocity(ID_2,0);
        dxl.setGoalVelocity(ID_3,0);
        dxl.torqueOn(ID_1);
        dxl.torqueOn(ID_4);
  
        dxl.setGoalPosition(ID_1, back1, UNIT_DEGREE);
        dxl.setGoalPosition(ID_4, back2, UNIT_DEGREE);
        //delay(1000);
  
        if (dxl.getPresentVelocity(ID_1) == 0 && dxl.getPresentVelocity(ID_4) == 0){
          dxl.torqueOff(ID_1);
          dxl.torqueOff(ID_4);
        }
  
        if (-vel_1 * dxl.getPresentVelocity(ID_1) || -vel_0 * dxl.getPresentVelocity(ID_4)){
          dxl.torqueOff(ID_1);
          dxl.torqueOff(ID_4);
          break;
        }
      }
    }
}

void get_angle() {
  theta2 = dxl.getPresentPosition(ID_1, UNIT_DEGREE);
  theta2 = theta2 - 270.0;
  theta5 = dxl.getPresentPosition(ID_4, UNIT_DEGREE);
  theta5 = theta5 + 45.0;
  A = b / a * (sin(PI / 180.0 * theta2) - sin(PI / 180.0 * theta5));
  B = b / a * (cos(PI / 180.0 * theta2) - cos(PI / 180.0 * theta5));
  E = (pow(A, 2) + pow(B, 2) + 4 * pow(c, 2) / pow(a, 2) + 4 * c / a * B) / (-2);
  F = sqrt(pow(A, 2) + pow(B + 2 * c / a, 2));
  if (asin((B + 2 * c / a) / F) * 180.0 / PI) alpha = 180 - asin((B + 2 * c / a) / F) * 180.0 / PI;
  else  alpha = asin((B + 2 * c / a) / F) * 180.0 / PI;
  if (asin(E / F) * 180.0 / PI < 0) theta3 = 180 - asin(E / F) * 180.0 / PI - theta2 - alpha;
  else  theta3 = 180 + asin(E / F) * 180.0 / PI - theta2 - alpha;

  rot_ang = dxl.getPresentVelocity(ID_6, UNIT_RPM);
}

void for_back() {
  if (analogRead(AXIS_Y) >= 625) {
    dxl.setGoalVelocity(ID_8, 25.8, UNIT_RPM);
    dxl.setGoalVelocity(ID_9, 25.8, UNIT_RPM);
    dxl.setGoalVelocity(ID_10, 25.8, UNIT_RPM);
    //delay(100);
  }
  else if (analogRead(AXIS_Y) <= 475) {
    dxl.setGoalVelocity(ID_8, -25.8, UNIT_RPM);
    dxl.setGoalVelocity(ID_9, -25.8, UNIT_RPM);
    dxl.setGoalVelocity(ID_10, -25.8, UNIT_RPM);
    //delay(100);
  }
  else {
    dxl.setGoalVelocity(ID_8, 0);
    dxl.setGoalVelocity(ID_9, 0);
    dxl.setGoalVelocity(ID_10, 0);
    //      delay(100);
  }
}
