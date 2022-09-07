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
const int32_t BAUDRATE = 57600;

float a=68.73, b=46.82, c=14.25;
double A,B,E,F;
double theta2, theta3, theta5, alpha, rot_ang;
double vel_1, vel_0, pos, vel1_1, vel0_1;
int i;
int DEBUG = 1;

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
  
  dxl.setOperatingMode(ID_1, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_2, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_3, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_4, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_5, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_6, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_7, OP_CURRENT_BASED_POSITION);
  
  dxl.ledOn(ID_1);
  dxl.ledOn(ID_2);
  dxl.ledOn(ID_3);
  dxl.ledOn(ID_4);
  dxl.ledOn(ID_5);
  dxl.ledOn(ID_6);
  dxl.ledOn(ID_7);
  delay(1000);
  
  dxl.ledOff(ID_1);
  dxl.ledOff(ID_2);
  dxl.ledOff(ID_3);
  dxl.ledOff(ID_4);
  dxl.ledOff(ID_5);
  dxl.ledOff(ID_6);
  dxl.ledOff(ID_7);
  delay(1000);
  
  soft_serial.begin(115200);
  soft_serial.println("Set up complete");
  
  i=0;
  
  dxl.torqueOn(ID_2);
  dxl.torqueOn(ID_3);
  dxl.torqueOn(ID_5);
  dxl.torqueOn(ID_7);
}

void loop() {
  get_angle();
  
  vel_1 = dxl.getPresentVelocity(ID_1);
  vel_0 = dxl.getPresentVelocity(ID_4);
  
  /* 이 부분 로봇의 모터 위치에 맞게 수정 필요 */
  dxl.setGoalPosition(ID_2, theta2, UNIT_DEGREE);
  dxl.setGoalPosition(ID_3, theta3, UNIT_DEGREE);
  dxl.setGoalPosition(ID_5, rot_ang, UNIT_DEGREE);
  dxl.setGoalPosition(ID_7, rot_ang, UNIT_DEGREE);

  if ((dxl.getPresentVelocity(ID_2) == 0 || dxl.getPresentVelocity(ID_3) == 0) && (vel_1 * dxl.getPresentVelocity(ID_1) > 0 || vel_0 * dxl.getPresentVelocity(ID_4))){
    // 로봇이 못 움직이는 경우
    while(1){
      dxl.torqueOn(ID_1);
      dxl.torqueOn(ID_4);
      dxl.setGoalPosition(ID_1, dxl.getPresentPosition(ID_2));

      vel1_1 = dxl.getPresentVelocity(ID_1);
      vel0_1 = dxl.getPresentVelocity(ID_4);
      
      if (vel1_1 == 0 && vel0_1 == 0){
        dxl.torqueOff(ID_1);
        dxl.torqueOff(ID_4);
      }

      if (-vel_1 * vel1_1 || -vel_0 * vel0_1){
        dxl.torqueOff(ID_1);
        dxl.torqueOff(ID_4);
        break;
      }
    }
  }

  // if (dxl.getPresentVelocity(ID_5)>0) dxl.setGoalPosition(ID_6, dxl.getPresentPosition(ID_5));

  i++;
  if (DEBUG && i==99){
    i=0;
    soft_serial.print("Theta2: ");
    soft_serial.println(theta2);

    soft_serial.print("Theta5: ");
    soft_serial.println(theta5);


    soft_serial.print("Theta3: ");
    soft_serial.println(theta3);

    soft_serial.print("Rotation Angle: ");
    soft_serial.println(rot_ang);
    soft_serial.println("");
  }
}

void get_angle(){ 
  theta2 = dxl.getPresentPosition(ID_1, UNIT_DEGREE);
  theta2 = theta2 - 270.0;
  theta5 = dxl.getPresentPosition(ID_4, UNIT_DEGREE);
  theta5 = theta5 + 45.0;
  A = b/a *(sin(M_PI/180.0*theta2) - sin(M_PI/180.0*theta5));
  B = b/a *(cos(M_PI/180.0*theta2) - cos(M_PI/180.0*theta5));
  E = pow(b,2)/pow(a,2)*(cos(theta2*M_PI/180.0 - theta5*M_PI/180.0)-1)-4*pow(c,2)/pow(a,2)-4*c/a*B;
  F = sqrt(pow(A,2) + pow(B+4*c/a,2));
  alpha = asin((B+4*c/a)/F)*180.0/M_PI;
  theta3 = asin(E/F) - theta2 - alpha;
  theta3 = theta3 + 180.0;
  rot_ang = dxl.getPresentPosition(ID_5, UNIT_DEGREE);
}
