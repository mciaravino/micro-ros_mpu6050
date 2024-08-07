//As written this file gets about 200 messges per second (Hz). We expect 200, and our goal is 100.

//--Start Includes--//

// ↓ Needed for micro_ros
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
// ↑ Needed for micro_ros

// ↓ Needed for the IMU
#include "MPU6050_6Axis_MotionApps20.h"
// ↑ Needed for the IMU

//This is needed for the multiplexor
#include <Wire.h>

// This is the message format we will use. There are other options though
#include <sensor_msgs/msg/imu.h> //http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html


//--End Includes--//

//In this segment we create the objects required for micro_ros
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer; //If we want to run sensor updates at different intervals, we create more than one timer

//Now we created the publisher, mpu, and msg objects, numbered 0 to n-1, where n is the number of IMUs we have connected
rcl_publisher_t publisher0;
rcl_publisher_t publisher1;
rcl_publisher_t publisher2;
rcl_publisher_t publisher3;
rcl_publisher_t publisher4;

sensor_msgs__msg__Imu msg0;
sensor_msgs__msg__Imu msg1; 
sensor_msgs__msg__Imu msg2; 
sensor_msgs__msg__Imu msg3; 
sensor_msgs__msg__Imu msg4;

MPU6050 imu0;
MPU6050 imu1;
MPU6050 imu2;
MPU6050 imu3;
MPU6050 imu4;

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z] //the ros2 quat message expects xyzw        quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 gg;      // [x, y, z]            gyro sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorInt16 ggWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#define LED_PIN 13 //This is for our error loop

//These are part of the error loop
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//This is required for the multiplexor
#define TCAADDR 0x70

//These states are used to help start the robot without physically disconnecting it
bool micro_ros_init_successful;

//These states are used to help start the robot without physically disconnecting it
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

/* Begin ODrive section 
#include <Arduino.h> //Needed for the ODrive Pro
#include "ODriveCAN.h"  //Needed for hte ODrive Pro

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000
// ODrive node_id for odrv0
#define ODRV0_NODE_ID 0

// Uncomment below the line that corresponds to your hardware.

// See also "Board-specific settings" to adapt the details for your hardware setup.

#define IS_TEENSY_BUILTIN // Teensy boards with built-in CAN interface (e.g. Teensy 4.1). See below to select which interface to use.
// #define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)
#define IS_MCP2515 // Any board with external MCP2515 based extension module. See below to configure the module.



/* Board-specific includes ---------------------------------------------------


#if defined(IS_TEENSY_BUILTIN) + defined(IS_ARDUINO_BUILTIN) + defined(IS_MCP2515) != 1
  #warning "Select exactly one hardware option at the top of this file."
  #if CAN_HOWMANY > 0 || CANFD_HOWMANY > 0
    #define IS_ARDUINO_BUILTIN
    #warning "guessing that this uses HardwareCAN"
  #else
    #error "cannot guess hardware version"
  #endif
#endif


#ifdef IS_ARDUINO_BUILTIN
// See https://github.com/arduino/ArduinoCore-API/blob/master/api/HardwareCAN.h
// and https://github.com/arduino/ArduinoCore-renesas/tree/main/libraries/Arduino_CAN
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
#endif // IS_ARDUINO_BUILTIN

#ifdef IS_MCP2515
  // See https://github.com/sandeepmistry/arduino-CAN/
  #include "MCP2515.h"
  #include "ODriveMCPCAN.hpp"
#endif // IS_MCP2515

#ifdef IS_TEENSY_BUILTIN
  // See https://github.com/tonton81/FlexCAN_T4
  // clone https://github.com/tonton81/FlexCAN_T4.git into /src
  #include <FlexCAN_T4.h>
  #include "ODriveFlexCAN.hpp"
  struct ODriveStatus; // hack to prevent teensy compile error
#endif // IS_TEENSY_BUILTIN

/* Teensy 
#ifdef IS_TEENSY_BUILTIN
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
  bool setupCan() {
    can_intf.begin();
    can_intf.setBaudRate(CAN_BAUDRATE);
    can_intf.setMaxMB(16);
    can_intf.enableFIFO();
    can_intf.enableFIFOInterrupt();
    can_intf.onReceive(onCanMessage);
    return true;
  }
#endif // IS_TEENSY_BUILTIN

/* MCP2515-based extension modules -
#ifdef IS_MCP2515
MCP2515Class& can_intf = CAN;
// chip select pin used for the MCP2515
#define MCP2515_CS 10
// interrupt pin used for the MCP2515
// NOTE: not all Arduino pins are interruptable, check the documentation for your board!
#define MCP2515_INT 2


// freqeuncy of the crystal oscillator on the MCP2515 breakout board. 

// common values are: 16 MHz, 12 MHz, 8 MHz

#define MCP2515_CLK_HZ 8000000



static inline void receiveCallback(int packet_size) {

  if (packet_size > 8) {

    return; // not supported

  }

  CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};

  CAN.readBytes(msg.buffer, packet_size);

  onCanMessage(msg);

}


bool setupCan() {

  // configure and initialize the CAN bus interface

  CAN.setPins(MCP2515_CS, MCP2515_INT);

  CAN.setClockFrequency(MCP2515_CLK_HZ);

  if (!CAN.begin(CAN_BAUDRATE)) {

    return false;

  }


  CAN.onReceive(receiveCallback);

  return true;

}


#endif // IS_MCP2515



/* Arduinos with built-in CAN 


#ifdef IS_ARDUINO_BUILTIN


HardwareCAN& can_intf = CAN;


bool setupCan() {

  return can_intf.begin((CanBitRate)CAN_BAUDRATE);

}


#endif

//End 
*/

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
//This is essentially our loop function
{  
  if (state == AGENT_CONNECTED)
  {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
    /*This is effectively our loop() function*/
    loopIMU();
    /*End is effectively loop*/
    }
  }
}

void setup() 
  {
  state = WAITING_AGENT;
  Wire.begin();
  
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  setupIMU();
  
}



void loop() {
  switch (state)
  {
  case WAITING_AGENT:
    if (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
    {
      state = AGENT_AVAILABLE;
    }
    break;
  case AGENT_AVAILABLE:
    if (create_entities())
    {
      state = AGENT_CONNECTED;
    }
    else
    {
      state = WAITING_AGENT;
    }
    break;
  case AGENT_CONNECTED:
    if (RMW_RET_OK != rmw_uros_ping_agent(100, 1))
    {
      state = AGENT_DISCONNECTED;
    }
    break;
  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;
  }
  if (state == AGENT_CONNECTED)
  {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, 1));
  }
}
