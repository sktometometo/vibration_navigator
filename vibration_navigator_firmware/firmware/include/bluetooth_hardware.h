#ifndef VIBRATION_NAVIGATOR_FIRMWARE_BLUETOOTH_HARDWARE_H__
#define VIBRATION_NAVIGATOR_FIRMWARE_BLUETOOTH_HARDWARE_H__

// M5Stack and ESP32 headers
#include <M5Stack.h>
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
