#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(13, 12); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

DynamixelShield dxl;
const float DXL_PROTOCOL_VERSION = 1.0;
const uint8_t id[]   = { 2, 3 };
const uint8_t LEFT   =   0;
const uint8_t RIGHT  =   1;
unsigned int pos     = 200;   // 200 ~ 350
unsigned int last_ch = '0';

void initDXL(unsigned int id) {
  dxl.ping(id);
  dxl.torqueOff(id);  // Turn off torque when configuring items in EEPROM area
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
  //DEBUG_SERIAL.println(dxl.getPresentPosition(id));
}

void setup() {
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  for(int i = 0; i < 2; i++) initDXL(id[i]);
  if(dxl.getPresentPosition(id[LEFT]) < 200){
    for(pos = 350; pos >= 200; pos -= 10) {
      dxl.setGoalPosition(id[LEFT],  512 - pos);  delay(5);
      dxl.setGoalPosition(id[RIGHT], 512 + pos);  delay(5);
    }
  }
}

void loop() {
  if(DEBUG_SERIAL.available() > 0) {
    char ch = DEBUG_SERIAL.read();
    if(ch != last_ch) {
      if     (ch == '1') {
        DEBUG_SERIAL.println("close grip");
        for(pos = 200; pos <= 349; pos += 10) {
          DEBUG_SERIAL.print(pos);  DEBUG_SERIAL.print(" ");
          dxl.setGoalPosition(id[LEFT],  512 - pos);  delay(10);
          dxl.setGoalPosition(id[RIGHT], 512 + pos);  delay(10);
        } DEBUG_SERIAL.println();
      }
      else if(ch == '0') {
        DEBUG_SERIAL.println("open grip" );
        for(pos = 349; pos >= 200; pos -= 10) {
          DEBUG_SERIAL.print(pos);  DEBUG_SERIAL.print(" ");
          dxl.setGoalPosition(id[LEFT],  512 - pos);  delay(10);
          dxl.setGoalPosition(id[RIGHT], 512 + pos);  delay(10);
        } DEBUG_SERIAL.println();
      }
      else;
    }
    last_ch = ch;
  }
}
