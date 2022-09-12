#include <DynamixelShield.h>

const float DXL_PROTOCOl_VERSION = 2.0;
const uint8_t id_1 = 1;
const uint8_t id_2 = 4;

DynamixelShield dxl;



using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  dxl.begin(57600);
  if(dxl.ping(id_1) == true){
    dxl.ledOn(id_1);
    delay(500);
    dxl.ledOff(id_1);
    delay(500);
  }
  else{
    for (int i = 0;i<3;i++){
      dxl.ledOn(id_2);
      delay(500);
      dxl.ledOff(id_2);
      delay(500);
    }
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
