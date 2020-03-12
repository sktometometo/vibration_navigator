// M5Stack and ESP32 headers
#include <M5Stack.h>
#include "BluetoothSerial.h"
// ROS related headers
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>
// USER
#include "bluetooth_hardware.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// function definitions
void callbackVibrationCommands( const std_msgs::UInt16MultiArray& );

// parameters
char BluetoothName[] = "M5Stack Vibration Navigator";

// ROSresources
ros::NodeHandle_<BluetoothHardware> nh;
std_msgs::UInt16MultiArray msg_vibration_commands;
ros::Subscriber<std_msgs::UInt16MultiArray> subscriber_vibration_commands( "~commands", &callbackVibrationCommands);

// Vibrator parameter
int frequency = 1000;
int resolution = 10;
int num_port = 1; // max 16
int port2pin[] = { 21 }; // port number and channel number should be the same

void callbackVibrationCommands( const std_msgs::UInt16MultiArray& msg ) {
  M5.Lcd.setCursor(5, 5);
  M5.Lcd.setTextSize(5);
  M5.Lcd.printf("The Vibration Navigator");
  M5.Lcd.setCursor(5, 20);
  M5.Lcd.setTextSize(1.5);
  for ( int i=0; i<min(num_port,msg.data_length); i++ ) {
    ledcWrite( i, msg.data[i] );
    M5.Lcd.printf("%d th cmd: %d\n", i, msg.data[i]);
  }
}

void setup() {
  // M5 Stack initialization
  M5.begin();
  M5.Power.begin();
  for ( int i=0; i<num_port; i++ ) {
    ledcSetup( i, frequency, resolution );
    ledcAttachPin( port2pin[i], i );
  }

  // Lcd display
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.setCursor(5, 5);
  M5.Lcd.setTextSize(2);
  M5.Lcd.printf("The Vibration Navigator");
  M5.Lcd.setCursor(5, 20);
  M5.Lcd.setTextSize(1.5);
  M5.Lcd.printf("Please pair \"M5Stack Vibration Navigator\"");

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("The device started, now you can pair it with bluetooth!");

  // ROS
  nh.initNode( BluetoothName );
  nh.subscribe( subscriber_vibration_commands );
}

void loop() {
  nh.spinOnce();
  delay(1);
}
