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
const uint8_t ID_5 = 5; //CONTROLLER ROTATION
const uint8_t ID_6 = 6; //ROBOT ROTATION
const int32_t BAUDRATE = 57600;

float a=68.73, b=46.82;
double A,B;
double theta2, theta3, theta5;
double vel_1, vel_0, pos, vel1_1, vel0_1;
int i;

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
  
  dxl.setOperatingMode(ID_1, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_2, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_3, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_4, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_5, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_6, OP_CURRENT_BASED_POSITION);

  dxl.torqueOn(ID_2);
  dxl.torqueOn(ID_3);
  
  get_angle();  

  while(theta2 == dxl.getPresentPosition(ID_2, UNIT_DEGREE) && theta3 == dxl.getPresentPosition(ID_3)){
    dxl.setGoalPosition(ID_2, theta2, UNIT_DEGREE);
    dxl.setGoalPosition(ID_3, theta3, UNIT_DEGREE);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  vel_1 = dxl.getPresentVelocity(ID_1);
  vel_0 = dxl.getPresentVelocity(ID_4);
  
  get_angle();
  dxl.setGoalPosition(ID_2, theta2, UNIT_DEGREE);
  dxl.setGoalPosition(ID_3, theta3, UNIT_DEGREE);

  if ((dxl.getPresentVelocity(ID_2) == 0 || dxl.getPresentVelocity(ID_3) == 0) && (vel_1 * dxl.getPresentVelocity(ID_1) > 0 || vel_0 * dxl.getPresentVelocity(ID_4))){
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

  if (dxl.getPresentVelocity(ID_5)>0) dxl.setGoalPosition(ID_6, dxl.getPresentPosition(ID_5));

  i++;
  if (i==99){
    i=0;
    soft_serial.print("Theta2: ");
    soft_serial.println(theta2);

    soft_serial.print("Theta3: ");
    soft_serial.println(theta3);
  }
}

uint32_t multiply(uint32_t multiplicand, float multiplier){
  const uint32_t TwoToThe24th = 0x01000000;
  uint64_t intMultiplier = multiplier * TwoToThe24th;

  intMultiplier *= multiplicand;

  uint64_t result = intMultiplier / TwoToThe24th;
  return result;
}

void get_angle(){ 
  theta2 = dxl.getPresentPosition(ID_1, UNIT_DEGREE);
  theta5 = dxl.getPresentPosition(ID_4, UNIT_DEGREE);
  A = b/a *(sin(theta2) + sin(theta5));
  B = b/a *(cos(theta2) + cos(theta5));
  theta3 = atan(A/B) - theta2 * M_PI/180.0;
  theta3 = 90.0 + theta3 * 180.0/M_PI;
}
