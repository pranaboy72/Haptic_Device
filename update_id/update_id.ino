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

DynamixelShield dxl;

using namespace ControlTableItem;

void setup() {
  const uint8_t OLD_ID = 1;
  const uint8_t NEW_ID = 1;
  const int32_t BAUDRATE = 57600;

  // Use UART port of Dynamixel shield to debug
  DEBUG_SERIAL.begin(115200);

  // Set port baudrate to 57600bps
  dxl.begin(BAUDRATE);

  // Set Port Protocol Version
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.print("PROTOCOl ");
  DEBUG_SERIAL.print(DXL_PROTOCOL_VERSION, 1);
  DEBUG_SERIAL.print(", ID");
  DEBUG_SERIAL.print(OLD_ID);
  DEBUG_SERIAL.print(": ");
  if(dxl.ping(OLD_ID) == true){
    DEBUG_SERIAL.print("ping succeeded");
    DEBUG_SERIAL.print(", Model Number: ");
    DEBUG_SERIAL.println(dxl.getModelNumber(OLD_ID));

    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(OLD_ID);

    //set a new ID for Dynamixel. 
    dxl.setID(OLD_ID, NEW_ID);
    DEBUG_SERIAL.println("ID has been successfully changed to: " + (String)NEW_ID);

    dxl.ledOn(NEW_ID);
    delay(500);
    dxl.ledOff(NEW_ID);
    delay(500);
    dxl.ledOn(NEW_ID);
    delay(500);
    dxl.ledOff(NEW_ID);
  }
  else{
    DEBUG_SERIAL.println("ping failed");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
