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

const uint8_t ID_1 = 1; //Controller Joint1
const uint8_t ID_2 = 2; //ROBOT Joint 1
const uint8_t ID_3 = 3; //ROBOT Joint 2
const uint8_t ID_4 = 4; //Controller Joint2
const uint8_t ID_5 = 5;
const uint8_t ID_6 = 6;
const uint8_t ID_7 = 7;
const int32_t BAUDRATE = 57600;

double a = 68.73, b = 46.82, c = 14.25;
double A,B,E,F, X,Y;
double theta2, theta3, theta5, alpha, beta, gamma, rot_ang,rob_ang_1, rob_ang_2;
double root;
double vel_1, vel_0, pos, vel1_1, vel0_1;
int i;

using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxl.ping(ID_1);
  dxl.ping(ID_4);
  dxl.ping(ID_6);
  dxl.ping(ID_2);
  dxl.ping(ID_3);

  dxl.setOperatingMode(ID_1, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_4, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_6, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_2, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(ID_3, OP_CURRENT_BASED_POSITION);

  soft_serial.begin(115200);
  soft_serial.println("Debug Robot Angle");
  i=0;
}

void loop() {
  // put your main code here, to run repeatedly:
  get_angle();
  rot_ang = dxl.getPresentPosition(ID_6, UNIT_DEGREE);
  rob_ang_1 = dxl.getPresentPosition(ID_2, UNIT_DEGREE);
  rob_ang_2 = dxl.getPresentPosition(ID_3, UNIT_DEGREE);

  

  i++;
  if (i==100){
    i=0;
    //Contoller Degree : Theta2 = middle joint, Theta3 = upper motor, Theta5 = lower motor
    soft_serial.print("Theta2: ");
    soft_serial.println(theta2);
    soft_serial.print("Theta5: ");
    soft_serial.println(theta5);
    soft_serial.print("Theta3: ");
    soft_serial.println(theta3);
//    soft_serial.print("A,B : ");
//    soft_serial.println(A);
//    soft_serial.println(B);
//    soft_serial.print("E,F : ");
//    soft_serial.println(E);
//    soft_serial.println(F);
//    soft_serial.print("alpha : ");
//    soft_serial.println(alpha);
    
//    get_theta5();
//    soft_serial.print("Solved theta5: ");
//    soft_serial.println(theta5);

    //Robot Degree : Robot angle 1 = forward motor, Robot angle2 = middle motor
//    soft_serial.print("Robot angle 1: ");
//    soft_serial.println(rob_ang_1);
//    soft_serial.print("Robot_angle 2: ");
//    soft_serial.println(rob_ang_2);

//    soft_serial.print("Controller 1 Velocity : ");
//    soft_serial.println(dxl.getPresentVelocity(ID_1));
//    soft_serial.print("Contoller 2 Velocity : ");
//    soft_serial.prin tln(dxl.getPresentVelocity(ID_4));
//    soft_serial.print("Robot angle 1 velocity : ");
//    soft_serial.println(dxl.getPresentVelocity(ID_2));
//    soft_serial.print("Robot angle 2 velocity : ");
//    soft_serial.println(dxl.getPresentVelocity(ID_3));
    
  }
}

void get_angle(){ 
  theta2 = dxl.getPresentPosition(ID_1, UNIT_DEGREE);
  theta2 = theta2 - 270.0;
  theta5 = dxl.getPresentPosition(ID_4, UNIT_DEGREE);
  theta5 = theta5 + 45.0;
  A = b/a *(sin(PI/180.0*theta2) - sin(PI/180.0*theta5));
  B = b/a *(cos(PI/180.0*theta2) - cos(PI/180.0*theta5));
  E = (pow(A,2) + pow(B,2) + 4*pow(c,2)/pow(a,2)+4*c/a*B)/(-2);
  F = sqrt(pow(A,2) + pow(B+2*c/a,2));
  if (asin((B+2*c/a)/F)*180.0/PI) alpha = 180-asin((B+2*c/a)/F)*180.0/PI;
  else  alpha = asin((B+2*c/a)/F)*180.0/PI;
  if (asin(E/F)*180.0/PI < 0) theta3 = 180-asin(E/F)*180.0/PI - theta2 - alpha;   
  else  theta3 = 180 + asin(E/F)*180.0/PI - theta2 - alpha;
}

//void get_theta5(){
//  theta2 = 370.0 - dxl.getPresentPosition(ID_3, UNIT_DEGREE);
//  theta2 = theta2 * M_PI / 180.0;
//  theta3 = dxl.getPresentPosition(ID_2, UNIT_DEGREE) - 30.0;
//  theta3 = theta3 *M_PI / 180.0;
//  X = a*sin(theta2 + theta3) + b*sin(theta2);
//  Y = a*cos(theta2 + theta3) + b*cos(theta2) + 2*c;
//  root = sqrt(pow(X,2)+pow(Y,2));
//  gamma = acos(X/root)*180.0/M_PI;
//  theta5 = asin((root-pow(a,2)+pow(b,2))/(2*b*sqrt(root)))-gamma;
//  theta5 = theta5 *180.0/M_PI;
//}
