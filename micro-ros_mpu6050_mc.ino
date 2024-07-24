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
#include <Adafruit_MPU6050.h> //https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>  //https://github.com/adafruit/Adafruit_Sensor

// ↑ Needed for mthe IMU

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
//

//Now we created the publisher, mpu, and msg objects, numbered 0 to n-1, where n is the number of IMUs we have connected
rcl_publisher_t publisher0;
rcl_publisher_t publisher1;
rcl_publisher_t publisher2;
rcl_publisher_t publisher3;
rcl_publisher_t publisher4;

Adafruit_MPU6050 mpu0;
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;
Adafruit_MPU6050 mpu3;
Adafruit_MPU6050 mpu4;

sensor_msgs__msg__Imu msg0;
sensor_msgs__msg__Imu msg1; //We want one message per imu so we can set the header for each
sensor_msgs__msg__Imu msg2; //We want one message per imu so we can set the header for each
sensor_msgs__msg__Imu msg3; //We want one message per imu so we can set the header for each
sensor_msgs__msg__Imu msg4; //We want one message per imu so we can set the header for each


//We also create the sensor event objects. We only need one object of each type
sensors_event_t accel, gyro, temp;  

#define LED_PIN 13 //This is for our error loop

//These are part of the error loop
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//This is required for the multiplexor
#define TCAADDR 0x70

//These states are used to help start the robot without physically disconnecting it
bool micro_ros_init_successful;
bool dmpReady = false;  // set true if DMP init was successful

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
    mpu0.getEvent(&accel, &gyro, &temp);
    msg0.header.stamp.sec = (int)millis()/1000;
    msg0.angular_velocity.x = gyro.gyro.x; 
    msg0.angular_velocity.y = gyro.gyro.y;
    msg0.angular_velocity.z = gyro.gyro.z;
    msg0.linear_acceleration.x = accel.acceleration.x; 
    msg0.linear_acceleration.y = accel.acceleration.y;
    msg0.linear_acceleration.z = accel.acceleration.z;
    RCSOFTCHECK(rcl_publish(&publisher0, &msg0, NULL));

    tcaselect(1);
    mpu1.getEvent(&accel, &gyro, &temp);
    msg1.header.stamp.sec = (int)millis()/1000;
    msg1.angular_velocity.x = gyro.gyro.x; 
    msg1.angular_velocity.y = gyro.gyro.y;
    msg1.angular_velocity.z = gyro.gyro.z;
    msg1.linear_acceleration.x = accel.acceleration.x; 
    msg1.linear_acceleration.y = accel.acceleration.y;
    msg1.linear_acceleration.z = accel.acceleration.z;
    RCSOFTCHECK(rcl_publish(&publisher1, &msg1, NULL));

    tcaselect(2);  
    mpu2.getEvent(&accel, &gyro, &temp);
    msg2.header.stamp.sec = (int)millis()/1000;
    msg2.angular_velocity.x = gyro.gyro.x; 
    msg2.angular_velocity.y = gyro.gyro.y;
    msg2.angular_velocity.z = gyro.gyro.z;
    msg2.linear_acceleration.x = accel.acceleration.x; 
    msg2.linear_acceleration.y = accel.acceleration.y;
    msg2.linear_acceleration.z = accel.acceleration.z;
    RCSOFTCHECK(rcl_publish(&publisher2, &msg2, NULL));

    tcaselect(3);  
    mpu3.getEvent(&accel, &gyro, &temp);
    msg3.header.stamp.sec = (int)millis()/1000;
    msg3.angular_velocity.x = gyro.gyro.x; 
    msg3.angular_velocity.y = gyro.gyro.y;
    msg3.angular_velocity.z = gyro.gyro.z;
    msg3.linear_acceleration.x = accel.acceleration.x; 
    msg3.linear_acceleration.y = accel.acceleration.y;
    msg3.linear_acceleration.z = accel.acceleration.z;
    RCSOFTCHECK(rcl_publish(&publisher3, &msg3, NULL));

    tcaselect(4);  
    mpu4.getEvent(&accel, &gyro, &temp);
    msg4.header.stamp.sec = (int)millis()/1000;
    msg4.angular_velocity.x = gyro.gyro.x; 
    msg4.angular_velocity.y = gyro.gyro.y;
    msg4.angular_velocity.z = gyro.gyro.z;
    msg4.linear_acceleration.x = accel.acceleration.x; 
    msg4.linear_acceleration.y = accel.acceleration.y;
    msg4.linear_acceleration.z = accel.acceleration.z;
    RCSOFTCHECK(rcl_publish(&publisher4, &msg4, NULL));
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
  mpu0.begin();
  
  tcaselect(1);
  mpu1.begin();

  tcaselect(2);
  mpu2.begin();
  
  tcaselect(3);
  mpu3.begin();

  tcaselect(4);
  mpu4.begin();
  
  /*
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  */

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
       

  const unsigned int timer_timeout = 10;
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
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  }
}
