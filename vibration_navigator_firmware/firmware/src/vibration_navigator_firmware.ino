//
// ROS related headers
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>

#define BUFSIZE 1024

// #define USE_BLUETOOTH
// #define USE_WIFI

#define USE_M5STACK
// #define USE_M5STACK_FIRE
// #define USE_M5STICK_C

//// M5Stack and ESP32 headers
#ifdef USE_M5STACK
#define M5STACK_MPU6886
#include <M5Stack.h>
#elif USE_M5STACK_FIRE
#define M5STACK_200Q
#include <M5Stack.h>
#elif USE_M5STICK_C
#include <M5StickC.h>
#endif

#ifdef USE_BLUETOOTH
#include "bluetooth_hardware.h"
#endif
#ifndef USE_WIFI
#include "uartserial_hardware.h"
#endif

//
const float const_g = 9.8;

// function definitions
void callbackVibrationCommands( const std_msgs::UInt16MultiArray& );

// parameters
#ifdef USE_WIFI
const char* ssid = "";
const char* password = "";
IPAddress server(,,,);
#elif defined(USE_BLUETOOTH)
char* BluetoothName = "VibrationNavigator";
#endif
int duration_loop = 10; // [ms]
int duration_imu = 10;
int duration_mutex = 5;

// ROSresources
#ifdef USE_WIFI
ros::NodeHandle nh;
#elif defined(USE_BLUETOOTH)
ros::NodeHandle_<BluetoothHardware> nh;
#else
ros::NodeHandle_<UARTSerialHardware, 25, 25, 4096, 4096> nh;
#endif
std_msgs::UInt16MultiArray msg_vibration_commands;
sensor_msgs::Imu msg_imu;
ros::Subscriber<std_msgs::UInt16MultiArray> subscriber_vibration_commands( "~commands", &callbackVibrationCommands);
ros::Publisher publisher_imu( "~imu", &msg_imu );
//char frame_id[BUFSIZE];
char* frame_id;

// task handler
TaskHandle_t xHandle = NULL;
SemaphoreHandle_t xMutex = NULL;

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
int counterPub = 0;
int counterCB = 0;
unsigned long startTime = 0;
int indexRow = 5;

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
    counterCB++;
}

/**
 * Task for IMU
 */
void taskIMUUpdater( void* pvParameters )
{
    portTickType xLastWakeTime = xTaskGetTickCount();
    const portTickType xFrequency = (portTickType)duration_imu;

    while ( true ) {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );

        M5.IMU.getGyroData(&imu_gyro_x,&imu_gyro_y,&imu_gyro_z);
        M5.IMU.getAccelData(&imu_acc_x,&imu_acc_y,&imu_acc_z);
        M5.IMU.getAhrsData(&imu_pitch,&imu_roll,&imu_yaw);
        M5.IMU.getTempData(&imu_temp);

        imu_gyro_x *= DEG_TO_RAD;
        imu_gyro_y *= DEG_TO_RAD;
        imu_gyro_z *= DEG_TO_RAD;
        imu_acc_x *= const_g;
        imu_acc_y *= const_g;
        imu_acc_z *= const_g;

        // https://www.kazetest.com/vcmemo/quaternion/quaternion.htm
        quaternion_w = cos( imu_roll / 2 )*cos( imu_pitch / 2 )*cos( imu_yaw / 2 ) + sin( imu_roll / 2 )*sin( imu_pitch / 2 )*sin( imu_yaw / 2 );
        quaternion_x = sin( imu_roll / 2 )*cos( imu_pitch / 2 )*cos( imu_yaw / 2 ) - cos( imu_roll / 2 )*sin( imu_pitch / 2 )*sin( imu_yaw / 2 );
        quaternion_y = cos( imu_roll / 2 )*sin( imu_pitch / 2 )*cos( imu_yaw / 2 ) + sin( imu_roll / 2 )*cos( imu_pitch / 2 )*sin( imu_yaw / 2 );
        quaternion_z = cos( imu_roll / 2 )*cos( imu_pitch / 2 )*sin( imu_yaw / 2 ) - sin( imu_roll / 2 )*sin( imu_pitch / 2 )*cos( imu_yaw / 2 );

        if ( xSemaphoreTake( xMutex, ( TickType_t ) duration_mutex ) == pdTRUE ) {
            msg_imu.header.stamp = nh.now();
            msg_imu.orientation.x = quaternion_x;
            msg_imu.orientation.y = quaternion_y;
            msg_imu.orientation.z = quaternion_z;
            msg_imu.orientation.w = quaternion_w;
            msg_imu.angular_velocity.x = deg2rad( imu_gyro_x );
            msg_imu.angular_velocity.y = deg2rad( imu_gyro_y );
            msg_imu.angular_velocity.z = deg2rad( imu_gyro_z );
            msg_imu.linear_acceleration.x = imu_acc_x;
            msg_imu.linear_acceleration.y = imu_acc_y;
            msg_imu.linear_acceleration.z = imu_acc_z;
            xSemaphoreGive( xMutex );
        }

        ++counterIMU;
    }
}

void setup()
{
  /**
   * M5 Stack initialization
   */
  M5.begin();
#ifdef defined(USE_M5STACK) || defined(USE_M5STACK_FIRE)
  M5.Power.begin();
#endif
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
#ifdef USE_M5STICK_C
  M5.Lcd.setRotation(3);
#endif
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.setCursor(1, indexRow);
  M5.Lcd.setTextSize(1);
  M5.Lcd.printf("The Vibration Navigator\n");
  M5.Lcd.setTextSize(1);
  indexRow += 10;

  /**
   * Print battery level
   */
  M5.Lcd.fillRect(1, indexRow, 2000, 10, WHITE);
  M5.Lcd.setCursor(1, indexRow);
#ifdef USE_M5STICK_C
  M5.Lcd.printf("Bat. Vol.: %.1f V", M5.Axp.GetBatVoltage());
#elif defined(USE_M5STACK) || defined(USE_M5STACK_FIRE)
  M5.Lcd.printf("Battery Level: %d \%", M5.Power.getBatteryLevel());
#endif
  indexRow += 10;
  delay(1000);

  /**
   * Deep sleeping...
   */
  /*
  if ( M5.Power.getBatteryLevel() < 10 ) {
      M5.Lcd.fillRect(1, indexRow, 2000, 10, WHITE);
      M5.Lcd.setCursor(1, indexRow);
      M5.Lcd.printf("Deep sleeping....");
      indexRow += 10;
      delay(5000);
      while ( true ) {
          M5.Power.deepSleep( 10000 );
      }
  }
  */

  /**
   * ROS Initialization
   */
#ifdef USE_WIFI
  int status = WiFi.begin(ssid,password);
  while ( status  != WL_CONNECTED ) {
      M5.Lcd.fillRect(1, indexRow, 2000, 10, WHITE);
      M5.Lcd.setCursor(1, indexRow);
#ifdef USE_M5STICK_C
      M5.Lcd.printf("WiFiCon. waiting...");
      for ( int i=0; i<10; i++ ) {
          delay(100);
          M5.Lcd.printf(".");
      }
#elif defined(USE_M5STACK) || defined(USE_M5STACK_FIRE)
      M5.Lcd.printf("WiFi connecting to %s .", ssid);
      for ( int i=0; i<10; i++ ) {
          delay(100);
          M5.Lcd.printf(".");
      }
#endif
      status = WiFi.begin(ssid,password);
  }
  M5.Lcd.fillRect(1, indexRow, 2000, 10, WHITE);
  M5.Lcd.setCursor(1, indexRow);
  M5.Lcd.printf("WiFi connected.");
  delay(1000);
  indexRow += 10;

  nh.getHardware()->setConnection(server);
  nh.initNode();
  nh.subscribe( subscriber_vibration_commands );
  nh.advertise( publisher_imu );
  while ( true ) {
      nh.spinOnce();
      M5.Lcd.fillRect(1, indexRow, 2000, 10, WHITE);
      M5.Lcd.setCursor(1, indexRow);
      M5.Lcd.printf("Server wainti.");
      if ( nh.getHardware()->connected() ) {
          M5.Lcd.fillRect(1, indexRow, 2000, 10, WHITE);
          M5.Lcd.setCursor(1, indexRow);
          M5.Lcd.printf("Server connected.");
          delay(1000);
          indexRow += 10;
          break;
      } else {
          for ( int i=0; i<10; i++ ) {
              delay(100);
              M5.Lcd.printf(".");
          }
      }
  }
#elif defined(USE_BLUETOOTH)
  nh.initNode( BluetoothName );
  nh.subscribe( subscriber_vibration_commands );
  nh.advertise( publisher_imu );
  while ( not nh.connected() ) {
      nh.spinOnce();
      M5.Lcd.fillRect(1, indexRow, 2000, 10, WHITE);
      M5.Lcd.setCursor(1, indexRow);
      M5.Lcd.printf("Serial conn. waitin.");
      for ( int i=0; i<10; i++ ) {
          delay(100);
          M5.Lcd.printf(".");
      }
  }
  M5.Lcd.fillRect(1, indexRow, 2000, 10, WHITE);
  M5.Lcd.setCursor(1, indexRow);
  M5.Lcd.printf("Serial connected.");
  indexRow += 10;
  delay(1000);
#else
  Serial.begin(115200);
  nh.initNode();
  nh.subscribe( subscriber_vibration_commands );
  nh.advertise( publisher_imu );
  while ( not nh.connected() ) {
      nh.spinOnce();
      M5.Lcd.fillRect(1, indexRow, 2000, 10, WHITE);
      M5.Lcd.setCursor(1, indexRow);
      M5.Lcd.printf("Serial conn. waitin.");
      for ( int i=0; i<10; i++ ) {
          delay(100);
          M5.Lcd.printf(".");
      }
  }
  M5.Lcd.fillRect(1, indexRow, 2000, 10, WHITE);
  M5.Lcd.setCursor(1, indexRow);
  M5.Lcd.printf("Serial connected.");
  indexRow += 10;
  delay(1000);
#endif

  frame_id = (char*)malloc(BUFSIZE);
  char **hoge = &frame_id;

  M5.Lcd.setCursor(1, indexRow);
  M5.Lcd.printf("Requesting parameters.");
  nh.spinOnce();
  if ( not nh.getParam("~imu_frame_id", hoge, 1, 10000 ) ) {
  /*
  if ( true ) {
  */
      strcpy( frame_id, "imu_frame" );
      M5.Lcd.fillRect(1, indexRow, 2000, 10, WHITE);
      M5.Lcd.setCursor(1, indexRow);
      M5.Lcd.printf("Parameter not found.");
      indexRow += 10;
  } else {
      M5.Lcd.fillRect(1, indexRow, 2000, 10, WHITE);
      M5.Lcd.setCursor(1, indexRow);
      M5.Lcd.printf("Parameter found.");
      indexRow += 10;
  }

  msg_imu.header.frame_id = frame_id;


  /**
   * mutex initialization
   */
  xMutex = xSemaphoreCreateMutex();

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

  M5.Lcd.setCursor(1, indexRow);
  M5.Lcd.printf("Init fin.");
  delay(1000);
  indexRow += 10;

  startTime = millis();
}

void loop()
{
    portTickType xLastWakeTime = xTaskGetTickCount();
    const portTickType xFrequency = (portTickType)duration_loop;
    int tempCounterROS, tempCounterIMU, tempCounterPub, tempCounterCB;

    while ( true ) {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );

        if ( counterROS % 100 == 0 ) {
            M5.Lcd.fillRect(1, indexRow, 2000, 10, WHITE);
            M5.Lcd.setCursor(1, indexRow);
            tempCounterROS = counterROS;
            tempCounterIMU = counterIMU;
            tempCounterPub = counterPub;
            tempCounterCB  = counterCB;
            counterROS = 0;
            counterIMU = 0;
            counterPub = 0;
            counterCB = 0;
            M5.Lcd.printf("[%5.1f s] loop counter: %d", (millis() - startTime)*1.0/1000, tempCounterROS );
        }
        if ( xSemaphoreTake( xMutex, ( TickType_t ) duration_mutex ) == pdTRUE ) {
            counterPub++;
            publisher_imu.publish( &msg_imu );
            xSemaphoreGive( xMutex );
        }
        nh.spinOnce();
        ++counterROS;
    }
}
