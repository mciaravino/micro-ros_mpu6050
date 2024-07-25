//As written this file gets about  messges per second (Hz). We expect 200, and our goal is 100.

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
#include <basicMPU6050.h> //https://github.com/RCmags/basicMPU6050/blob/main/examples/parameters/parameters.ino
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

//Now we created the publisher, mpu, and msg objects, numbered 0 to n-1, where n is the number of IMUs we have connected
rcl_publisher_t publisher0;
rcl_publisher_t publisher1;
rcl_publisher_t publisher2;
rcl_publisher_t publisher3;
rcl_publisher_t publisher4;

//-- Input parameters:

// Gyro settings:
#define         LP_FILTER   3           // Low pass filter.                    Value from 0 to 6
#define         GYRO_SENS   0           // Gyro sensitivity.                   Value from 0 to 3
#define         ACCEL_SENS  0           // Accelerometer sensitivity.          Value from 0 to 3
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.   A0 -> GND : ADDRESS_A0 = LOW
                                        //                                     A0 -> 5v  : ADDRESS_A0 = HIGH
// Accelerometer offset:
constexpr int   AX_OFFSET =  552;       // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = -241;       // These values are unlikely to be zero.
constexpr int   AZ_OFFSET = -3185;

// Output scale: 
constexpr float AX_SCALE = 1.00457;     // Multiplier for accelerometer outputs. Use this to calibrate the sensor. If unknown set to 1.
constexpr float AY_SCALE = 0.99170;
constexpr float AZ_SCALE = 0.98317;

constexpr float GX_SCALE = 0.99764;     // Multiplier to gyro outputs. Use this to calibrate the sensor. If unknown set to 1.
constexpr float GY_SCALE = 1.0;
constexpr float GZ_SCALE = 1.01037;

// Bias estimate:
#define         GYRO_BAND   35          // Standard deviation of the gyro signal. Gyro signals within this band (relative to the mean) are suppresed.   
#define         BIAS_COUNT  5000        // Samples of the mean of the gyro signal. Larger values provide better calibration but delay suppression response. 

//-- Set the template parameters:

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET, 
             &AX_SCALE,  &AY_SCALE,  &AZ_SCALE,
             &GX_SCALE,  &GY_SCALE,  &GZ_SCALE,
             GYRO_BAND,  BIAS_COUNT 
            >imu0;

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET, 
             &AX_SCALE,  &AY_SCALE,  &AZ_SCALE,
             &GX_SCALE,  &GY_SCALE,  &GZ_SCALE,
             GYRO_BAND,  BIAS_COUNT 
            >imu1;

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET, 
             &AX_SCALE,  &AY_SCALE,  &AZ_SCALE,
             &GX_SCALE,  &GY_SCALE,  &GZ_SCALE,
             GYRO_BAND,  BIAS_COUNT 
            >imu2;

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET, 
             &AX_SCALE,  &AY_SCALE,  &AZ_SCALE,
             &GX_SCALE,  &GY_SCALE,  &GZ_SCALE,
             GYRO_BAND,  BIAS_COUNT 
            >imu3;

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET, 
             &AX_SCALE,  &AY_SCALE,  &AZ_SCALE,
             &GX_SCALE,  &GY_SCALE,  &GZ_SCALE,
             GYRO_BAND,  BIAS_COUNT 
            >imu4;

sensor_msgs__msg__Imu msg0;
sensor_msgs__msg__Imu msg1; 
sensor_msgs__msg__Imu msg2; 
sensor_msgs__msg__Imu msg3; 
sensor_msgs__msg__Imu msg4; 


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
    imu0.updateBias();
    msg0.header.stamp.sec = (int)millis()/1000;
    msg0.angular_velocity.x = imu0.gx(); 
    msg0.angular_velocity.y = imu0.gy();
    msg0.angular_velocity.z = imu0.gz();
    msg0.linear_acceleration.x = imu0.ax(); 
    msg0.linear_acceleration.y = imu0.ay();
    msg0.linear_acceleration.z = imu0.az();
    RCSOFTCHECK(rcl_publish(&publisher0, &msg0, NULL)); //We just do a temperature check once per loop
    //rcl_publish(&publisher0, &msg0, NULL);

    tcaselect(1);  
    imu1.updateBias();
    msg1.header.stamp.sec = (int)millis()/1000;
    msg1.angular_velocity.x = imu1.gx(); 
    msg1.angular_velocity.y = imu1.gy();
    msg1.angular_velocity.z = imu1.gz();
    msg1.linear_acceleration.x = imu1.ax(); 
    msg1.linear_acceleration.y = imu1.ay();
    msg1.linear_acceleration.z = imu1.az();
    rcl_publish(&publisher1, &msg1, NULL);

    tcaselect(2);  
    imu2.updateBias();
    msg2.header.stamp.sec = (int)millis()/1000;
    msg2.angular_velocity.x = imu2.gx(); 
    msg2.angular_velocity.y = imu2.gy();
    msg2.angular_velocity.z = imu2.gz();
    msg2.linear_acceleration.x = imu2.ax(); 
    msg2.linear_acceleration.y = imu2.ay();
    msg2.linear_acceleration.z = imu2.az();
    rcl_publish(&publisher2, &msg2, NULL);

    tcaselect(3);  
    imu3.updateBias();
    msg3.header.stamp.sec = (int)millis()/1000;
    msg3.angular_velocity.x = imu3.gx(); 
    msg3.angular_velocity.y = imu3.gy();
    msg3.angular_velocity.z = imu3.gz();
    msg3.linear_acceleration.x = imu3.ax(); 
    msg3.linear_acceleration.y = imu3.ay();
    msg3.linear_acceleration.z = imu3.az();
    rcl_publish(&publisher3, &msg3, NULL);

    tcaselect(4);  
    imu4.updateBias();
    msg4.header.stamp.sec = (int)millis()/1000;
    msg4.angular_velocity.x = imu4.gx(); 
    msg4.angular_velocity.y = imu4.gy();
    msg4.angular_velocity.z = imu4.gz();
    msg4.linear_acceleration.x = imu4.ax(); 
    msg4.linear_acceleration.y = imu4.ay();
    msg4.linear_acceleration.z = imu4.az();
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
  imu0.setup();
  imu0.setBias();

  tcaselect(1);
  imu1.setup();
  imu1.setBias();

  tcaselect(2);
  imu2.setup();
  imu2.setBias();

  tcaselect(3);
  imu3.setup();
  imu3.setBias();

  tcaselect(4);
  imu4.setup();
  imu4.setBias();


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
