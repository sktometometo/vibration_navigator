#ifndef VIBRATION_NAVIGATOR_FIRMWARE_BLUETOOTH_HARDWARE_H__
#define VIBRATION_NAVIGATOR_FIRMWARE_BLUETOOTH_HARDWARE_H__

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

#include "BluetoothSerial.h"

class BluetoothHardware {
public:
  BluetoothSerial btserial_;

  BluetoothHardware() {};

  void init(char *bluetoothName) {
    this->btserial_.begin( bluetoothName );
  }

  int read() {
    return this->btserial_.read();
  }

  void write( uint8_t* data, int length ) {
    for ( int i=0; i<length; i++ ) {
      this->btserial_.write( data[i] );
    }
  }

  unsigned long time() {
    return millis();
  }
};

#endif
