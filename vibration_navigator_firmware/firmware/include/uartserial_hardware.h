#ifndef VIBRATION_NAVIGATOR_FIRMWARE_UARTSERIAL_HARDWARE_H__
#define VIBRATION_NAVIGATOR_FIRMWARE_UARTSERIAL_HARDWARE_H__

// M5Stack and ESP32 headers
#include <M5Stack.h>

class UARTSerialHardware {
public:

  UARTSerialHardware() {};

  void init() {
  }

  int read() {
    return Serial.read();
  }

  void write( uint8_t* data, int length ) {
    Serial.write( data, length );
  }

  unsigned long time() {
    return millis();
  }
};

#endif
