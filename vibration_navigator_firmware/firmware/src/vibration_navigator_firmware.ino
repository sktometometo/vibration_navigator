//
#define M5STACK_200Q
// M5Stack and ESP32 headers
#include <M5Stack.h>
#include "BluetoothSerial.h"
// ROS related headers
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>
// USER
#include "bluetooth_hardware.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define BUFSIZE 128

const float g_acc = 9.80665;

// function definitions
void callbackVibrationCommands( const std_msgs::UInt16MultiArray& );

// parameters
char BluetoothName[] = "M5Stack Vibration Navigator";
int duration_loop = 5; // [ms]

// ROSresources
ros::NodeHandle_<BluetoothHardware> nh;
std_msgs::UInt16MultiArray msg_vibration_commands;
sensor_msgs::Imu msg_imu;
ros::Subscriber<std_msgs::UInt16MultiArray> subscriber_vibration_commands( "~commands", &callbackVibrationCommands);
ros::Publisher publisher_imu( "~imu", &msg_imu );
char frame_id[BUFSIZE];

// task handler
TaskHandle_t xHandle = NULL;
SemaphoreHandle_t xBinarySemaphore = NULL;

// Vibrator parameter
int frequency = 1000;
int resolution = 10;
int num_port = 1; // max 16
int port2pin[] = { 21 }; // port number and channel number should be the same

/**
 * Values for sensor data
 */
float imu_acc_x = 0.0F;
float imu_acc_y = 0.0F;
float imu_acc_z = 0.0F;
float imu_gyro_x = 0.0F;
float imu_gyro_y = 0.0F;
float imu_gyro_z = 0.0F;
float imu_pitch = 0.0F;
float imu_roll  = 0.0F;
float imu_yaw   = 0.0F;
float imu_temp = 0.0F;
float quaternion_x = 1.0F;
float quaternion_y = 0.0F;
float quaternion_z = 0.0F;
float quaternion_w = 0.0F;

/**
 * for debug
 */
int counterIMU = 0;
int counterROS = 0;
unsigned long startTime = 0;

/**
 * Utility
 */
float rad2deg( float rad ) { return rad * 180 / M_PI; }
float deg2rad( float deg ) { return deg * M_PI / 180; }

/**
 * ROS callback
 */
void callbackVibrationCommands( const std_msgs::UInt16MultiArray& msg )
{
    for ( int i=0; i<min(num_port,msg.data_length); i++ ) {
        ledcWrite( i, msg.data[i] );
    }
}

/**
 * Task for IMU
 */
void taskIMUUpdater( void* pvParameters )
{
    while ( true ) {
        M5.IMU.getGyroData(&imu_gyro_x,&imu_gyro_y,&imu_gyro_z);
        M5.IMU.getAccelData(&imu_acc_x,&imu_acc_y,&imu_acc_z);
        M5.IMU.getAhrsData(&imu_pitch,&imu_roll,&imu_yaw);
        M5.IMU.getTempData(&imu_temp);

        // https://www.kazetest.com/vcmemo/quaternion/quaternion.htm
        quaternion_w = cos( imu_roll / 2 )*cos( imu_pitch / 2 )*cos( imu_yaw / 2 ) + sin( imu_roll / 2 )*sin( imu_pitch / 2 )*sin( imu_yaw / 2 );
        quaternion_x = sin( imu_roll / 2 )*cos( imu_pitch / 2 )*cos( imu_yaw / 2 ) - cos( imu_roll / 2 )*sin( imu_pitch / 2 )*sin( imu_yaw / 2 );
        quaternion_y = cos( imu_roll / 2 )*sin( imu_pitch / 2 )*cos( imu_yaw / 2 ) + sin( imu_roll / 2 )*cos( imu_pitch / 2 )*sin( imu_yaw / 2 );
        quaternion_z = cos( imu_roll / 2 )*cos( imu_pitch / 2 )*sin( imu_yaw / 2 ) - sin( imu_roll / 2 )*sin( imu_pitch / 2 )*cos( imu_yaw / 2 );

        if ( xSemaphoreTake( xBinarySemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
            msg_imu.header.stamp = nh.now();
            msg_imu.orientation.x = quaternion_x;
            msg_imu.orientation.y = quaternion_y;
            msg_imu.orientation.z = quaternion_z;
            msg_imu.orientation.w = quaternion_w;
            msg_imu.angular_velocity.x = deg2rad( imu_gyro_x );
            msg_imu.angular_velocity.y = deg2rad( imu_gyro_y );
            msg_imu.angular_velocity.z = deg2rad( imu_gyro_z );
            msg_imu.linear_acceleration.x = imu_acc_x * g_acc;
            msg_imu.linear_acceleration.y = imu_acc_y * g_acc;
            msg_imu.linear_acceleration.z = imu_acc_z * g_acc;
            xSemaphoreGive( xBinarySemaphore );
        }

        // M5.Lcd.printf("imu update\n");
        ++counterIMU;
    }
}

void setup()
{
  /**
   * M5 Stack initialization
   */
  M5.begin();
  M5.Power.begin();
  M5.IMU.Init();

  /**
   *
   */
  for ( int i=0; i<num_port; i++ ) {
    ledcSetup( i, frequency, resolution );
    ledcAttachPin( port2pin[i], i );
  }

  /**
   * Lcd display
   */
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.setCursor(5, 5);
  M5.Lcd.setTextSize(2);
  M5.Lcd.printf("The Vibration Navigator\n");
  M5.Lcd.setTextSize(1);

  /**
   * ROS Initialization
   */
  nh.initNode( BluetoothName );
  if ( not nh.getParam("~imu_frame_id", (char**)&frame_id) ) {
      strcpy( frame_id, "imu_frame" );
  }
  msg_imu.header.frame_id = frame_id;
  nh.subscribe( subscriber_vibration_commands );
  nh.advertise( publisher_imu );

  /**
   * x
   */
  xBinarySemaphore = xSemaphoreCreateBinary();

  /**
   * Task Initialization
   */
  xTaskCreatePinnedToCore(
          taskIMUUpdater,
          "IMU task",
          4096,
          NULL,
          2,
          &xHandle,
          1 );

  M5.Lcd.printf("Initialized\n");
  M5.Lcd.printf("Please pair with \"%s\"\n", BluetoothName);
  M5.Lcd.printf("Main loop started\n");

  startTime = millis();
}

void loop()
{
    if ( counterROS % 100 == 0 ) {
        M5.Lcd.fillRect(5, 50, 2000, 10, WHITE);
        M5.Lcd.setCursor(5, 50);
        M5.Lcd.printf("[%d s] ROS loop: %d, IMU loop: %d\n", (millis() - startTime)/1000, counterROS, counterIMU);
    }
    if ( xBinarySemaphore != NULL ) {
        if ( xSemaphoreTake( xBinarySemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
            publisher_imu.publish( &msg_imu );
            xSemaphoreGive( xBinarySemaphore );
        }
    }
    nh.spinOnce();
    delay(duration_loop);
    ++counterROS;
}
