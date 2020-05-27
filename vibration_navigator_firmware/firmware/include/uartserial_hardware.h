#ifndef VIBRATION_NAVIGATOR_FIRMWARE_UARTSERIAL_HARDWARE_H__
#define VIBRATION_NAVIGATOR_FIRMWARE_UARTSERIAL_HARDWARE_H__

// M5Stack and ESP32 headers
#if TYPE_M5STACK == 0
#define M5STACK_MPU6886
#include <M5Stack.h>
#elif TYPE_M5STACK == 1
#define M5STACK_200Q
#include <M5Stack.h>
#elif TYPE_M5STACK == 2
#include <M5StickC.h>
#endif

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
