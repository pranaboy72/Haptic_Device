#include <DynamixelShield.h>

 

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

 

const uint8_t ID_1 = 1;

const uint8_t ID_2 = 4;

const int32_t BAUDRATE = 57600;

float pos;

float re_pos;

float vel_1;

float vel_2;

int i;
 

using namespace ControlTableItem;

 

void setup() {

//  DEBUG_SERIAL.begin(115200);

 

  dxl.begin(BAUDRATE);

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

 

  dxl.ping(ID_1);

  dxl.ping(ID_2);

 
  dxl.torqueOff(ID_1);

  dxl.torqueOff(ID_2);

  dxl.setOperatingMode(ID_1, OP_CURRENT_BASED_POSITION);

  dxl.setOperatingMode(ID_2, OP_CURRENT_BASED_POSITION);

  dxl.torqueOn(ID_2);

 

  

  // Initialize

  pos = dxl.getPresentPosition(ID_1, UNIT_DEGREE);

  while( pos == dxl.getPresentPosition(ID_2, UNIT_DEGREE)){

    dxl.setGoalPosition(ID_2, UNIT_DEGREE);

  }

  

  // Que

  dxl.ledOn(ID_1);

  dxl.ledOn(ID_2);

  delay(500);

  dxl.ledOff(ID_1);

  dxl.ledOff(ID_2);

  delay(500);

 

  soft_serial.begin(115200);

  

  i=0;

}

 

void loop() {

  // put your main code here, to run repeatedly:

  vel_1 = dxl.getPresentVelocity(ID_1);

  pos = dxl.getPresentPosition(ID_1, UNIT_DEGREE);

  dxl.setGoalPosition(ID_2, pos, UNIT_DEGREE);

  //delay(10);

 

  if (dxl.getPresentVelocity(ID_2) == 0 && vel_1 * dxl.getPresentVelocity(ID_1) > 0){     // if the robot link stops and the controller link still wants to rotate,

    while(1){                     

      dxl.setGoalVelocity(ID_2, 0);
      
      dxl.torqueOn(ID_1);                                                                 // give current to controller link to stop

      dxl.setGoalPosition(ID_1, dxl.getPresentPosition(ID_2));                            // and set the position with the robot link

      //delay(10);

      

      if (dxl.getPresentVelocity(ID_1) == 0){                                             // if the torque applied to controller is removed, allow it to move 

        dxl.torqueOff(ID_1);

      }

      

      if (- vel_1 * dxl.getPresentVelocity(ID_1)){                                        // if controller turns to the opposite of blocked direction, break

        dxl.torqueOff(ID_1);

        break;

      }

    }

  }

  

  i++;

  if (i==99){

    i=0;

    soft_serial.print("Motor 1 Position: ");

    soft_serial.println(dxl.getPresentPosition(ID_1, UNIT_DEGREE));

    

    soft_serial.print("Motor 2 Position: ");

    soft_serial.println(dxl.getPresentPosition(ID_2, UNIT_DEGREE));

 

    soft_serial.print("Motor 1 Velocity: ");

    soft_serial.println(dxl.getPresentVelocity(ID_1));

  }

}
