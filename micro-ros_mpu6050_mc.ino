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

//-- Input parameters:

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
bool dmpReady = false;  // set true when DMP init is successful

//These states are used to help start the robot without physically disconnecting it
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  if (state == AGENT_CONNECTED)
  {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
    tcaselect(0);  
    imu0.dmpGetCurrentFIFOPacket(fifoBuffer);
    imu0.dmpGetQuaternion(&q, fifoBuffer);
    imu0.dmpGetGravity(&gravity, &q);
    imu0.dmpGetAccel(&aa, fifoBuffer);
    imu0.dmpGetGyro(&gg, fifoBuffer);
    msg0.orientation.w = q.w;
    msg0.orientation.x = q.x;
    msg0.orientation.y = q.y;
    msg0.orientation.z = q.z;
    msg0.angular_velocity.x = (float)aa.x; // these might be gg not aa
    msg0.angular_velocity.y = (float)aa.y;
    msg0.angular_velocity.z = (float)aa.z;
    msg0.linear_acceleration.x = (float)gg.x; // these might be aa not gg
    msg0.linear_acceleration.y = (float)gg.y;
    msg0.linear_acceleration.z = (float)gg.z;
    msg0.header.stamp.sec = millis() / 1000;
    RCSOFTCHECK(rcl_publish(&publisher0, &msg0, NULL)); //We just do a temperature check once per loop
    //rcl_publish(&publisher0, &msg0, NULL);

    tcaselect(1);  
    imu1.dmpGetCurrentFIFOPacket(fifoBuffer);
    imu1.dmpGetQuaternion(&q, fifoBuffer);
    imu1.dmpGetGravity(&gravity, &q);
    imu1.dmpGetAccel(&aa, fifoBuffer);
    imu1.dmpGetGyro(&gg, fifoBuffer);
    msg1.orientation.w = q.w;
    msg1.orientation.x = q.x;
    msg1.orientation.y = q.y;
    msg1.orientation.z = q.z;
    msg1.angular_velocity.x = (float)aa.x; // these might be gg not aa
    msg1.angular_velocity.y = (float)aa.y;
    msg1.angular_velocity.z = (float)aa.z;
    msg1.linear_acceleration.x = (float)gg.x; // these might be aa not gg
    msg1.linear_acceleration.y = (float)gg.y;
    msg1.linear_acceleration.z = (float)gg.z;
    msg1.header.stamp.sec = millis() / 1000;
    rcl_publish(&publisher1, &msg1, NULL);

    tcaselect(2);  
    imu2.dmpGetCurrentFIFOPacket(fifoBuffer);
    imu2.dmpGetQuaternion(&q, fifoBuffer);
    imu2.dmpGetGravity(&gravity, &q);
    imu2.dmpGetAccel(&aa, fifoBuffer);
    imu2.dmpGetGyro(&gg, fifoBuffer);
    msg2.orientation.w = q.w;
    msg2.orientation.x = q.x;
    msg2.orientation.y = q.y;
    msg2.orientation.z = q.z;
    msg2.angular_velocity.x = (float)aa.x; // these might be gg not aa
    msg2.angular_velocity.y = (float)aa.y;
    msg2.angular_velocity.z = (float)aa.z;
    msg2.linear_acceleration.x = (float)gg.x; // these might be aa not gg
    msg2.linear_acceleration.y = (float)gg.y;
    msg2.linear_acceleration.z = (float)gg.z;
    msg2.header.stamp.sec = millis() / 1000;
    rcl_publish(&publisher2, &msg2, NULL);

    tcaselect(3);  
    imu3.dmpGetCurrentFIFOPacket(fifoBuffer);
    imu3.dmpGetQuaternion(&q, fifoBuffer);
    imu3.dmpGetGravity(&gravity, &q);
    imu3.dmpGetAccel(&aa, fifoBuffer);
    imu3.dmpGetGyro(&gg, fifoBuffer);
    msg3.orientation.w = q.w;
    msg3.orientation.x = q.x;
    msg3.orientation.y = q.y;
    msg3.orientation.z = q.z;
    msg3.angular_velocity.x = (float)aa.x; // these might be gg not aa
    msg3.angular_velocity.y = (float)aa.y;
    msg3.angular_velocity.z = (float)aa.z;
    msg3.linear_acceleration.x = (float)gg.x; // these might be aa not gg
    msg3.linear_acceleration.y = (float)gg.y;
    msg3.linear_acceleration.z = (float)gg.z;
    msg3.header.stamp.sec = millis() / 1000;
    rcl_publish(&publisher3, &msg3, NULL);

    tcaselect(4);  
    imu4.dmpGetCurrentFIFOPacket(fifoBuffer);
    imu4.dmpGetQuaternion(&q, fifoBuffer);
    imu4.dmpGetGravity(&gravity, &q);
    imu4.dmpGetAccel(&aa, fifoBuffer);
    imu4.dmpGetGyro(&gg, fifoBuffer);
    msg4.orientation.w = q.w;
    msg4.orientation.x = q.x;
    msg4.orientation.y = q.y;
    msg4.orientation.z = q.z;
    msg4.angular_velocity.x = (float)aa.x; // these might be gg not aa
    msg4.angular_velocity.y = (float)aa.y;
    msg4.angular_velocity.z = (float)aa.z;
    msg4.linear_acceleration.x = (float)gg.x; // these might be aa not gg
    msg4.linear_acceleration.y = (float)gg.y;
    msg4.linear_acceleration.z = (float)gg.z;
    msg4.header.stamp.sec = millis() / 1000;
    rcl_publish(&publisher4, &msg4, NULL);
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

  //Set all msg headers
  msg0.header.frame_id.data = "imu0_header";
  msg1.header.frame_id.data = "imu1_header";
  msg2.header.frame_id.data = "imu2_header";
  msg3.header.frame_id.data = "imu3_header";
  msg4.header.frame_id.data = "imu4_header";

  //Start all IMUs
  tcaselect(0);
  imu0.initialize();
  imu0.dmpInitialize();
  imu0.setXGyroOffset(-156); // supply your own gyro offsets here, scaled for min sensitivity
  imu0.setYGyroOffset(-11);
  imu0.setZGyroOffset(-14);
  imu0.setXAccelOffset(-3699);
  imu0.setYAccelOffset(-2519);
  imu0.setZAccelOffset(1391);

  tcaselect(1);
  imu1.initialize();
  imu1.dmpInitialize();
  imu1.setXGyroOffset(-156); // supply your own gyro offsets here, scaled for min sensitivity
  imu1.setYGyroOffset(-11);
  imu1.setZGyroOffset(-14);
  imu1.setXAccelOffset(-3699);
  imu1.setYAccelOffset(-2519);
  imu1.setZAccelOffset(1391);

  tcaselect(2);
  imu2.initialize();
  imu2.dmpInitialize();
  imu2.setXGyroOffset(-156); // supply your own gyro offsets here, scaled for min sensitivity
  imu2.setYGyroOffset(-11);
  imu2.setZGyroOffset(-14);
  imu2.setXAccelOffset(-3699);
  imu2.setYAccelOffset(-2519);
  imu2.setZAccelOffset(1391);

  tcaselect(3);
  imu3.initialize();
  imu3.dmpInitialize();
  imu3.setXGyroOffset(-156); // supply your own gyro offsets here, scaled for min sensitivity
  imu3.setYGyroOffset(-11);
  imu3.setZGyroOffset(-14);
  imu3.setXAccelOffset(-3699);
  imu3.setYAccelOffset(-2519);
  imu3.setZAccelOffset(1391);

  tcaselect(4);
  imu4.initialize();
  imu4.dmpInitialize();
  imu4.setXGyroOffset(-156); // supply your own gyro offsets here, scaled for min sensitivity
  imu4.setYGyroOffset(-11);
  imu4.setZGyroOffset(-14);
  imu4.setXAccelOffset(-3699);
  imu4.setYAccelOffset(-2519);
  imu4.setZAccelOffset(1391);

  dmpReady = true;
}

bool create_entities()
{
  // Code to initialize ROS entities
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_imu_node", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher0,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "micro_ros_arduino_imu0_publisher"));

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher1,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "micro_ros_arduino_imu1_publisher"));

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher2,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "micro_ros_arduino_imu2_publisher"));

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher3,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "micro_ros_arduino_imu3_publisher")); 

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher4,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "micro_ros_arduino_imu4_publisher"));
       
  const unsigned int timer_timeout = 5;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  //Make sure to destroy each publisher
  rcl_publisher_fini(&publisher0, &node);
  rcl_publisher_fini(&publisher1, &node);
  rcl_publisher_fini(&publisher2, &node);
  rcl_publisher_fini(&publisher3, &node);
  rcl_publisher_fini(&publisher4, &node);
  
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void loop() {
  
  if (!dmpReady)
    return;
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
