#include <DynamixelShield.h>

const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

const uint8_t ID_1 = 1;
const uint8_t ID_2 = 4;
const int32_t BAUDRATE = 57600;

void setup() {
  // put your setup code here, to run once:  
  // Use UART port of DYNAMIXEL Shield to debug.
  Serial.begin(115200);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(BAUDRATE);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxl.ping(ID_1);
  dxl.ping(ID_2);

  dxl.torqueOff(ID_1);
  dxl.torqueOff(ID_2);
  dxl.setOperatingMode(ID_1, OP_POSITION); //pos limit: 0~4095
  dxl.setOperatingMode(ID_2, OP_POSITION);
  dxl.torqueOn(ID_1);
  dxl.torqueOn(ID_2);
}

void loop(){
  dxl.setGoalPosition(ID_1, 1000);
  dxl.setGoalPosition(ID_2, 3000);
  delay(1000);

}
