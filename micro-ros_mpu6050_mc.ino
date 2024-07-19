#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//#include <std_msgs/msg/int32.h>

#include <Adafruit_MPU6050.h> //https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>  //https://github.com/adafruit/Adafruit_Sensor
//#include <Wire.h>

#include <sensor_msgs/msg/imu.h> //http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html


rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

Adafruit_MPU6050 mpu;

sensor_msgs__msg__Imu msg;

sensors_event_t accel, gyro, temp;  //Declares an object "event"

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
  mpu.getEvent(&accel, &gyro, &temp);

  /*msg.orientation.w = 0;
  msg.orientation.x = event.orientation.x;
  msg.orientation.y = event.orientation.y;
  msg.orientation.z = event.orientation.z;*/
  
  msg.angular_velocity.x = gyro.gyro.x; 
  msg.angular_velocity.y = gyro.gyro.y;
  msg.angular_velocity.z = gyro.gyro.z;
  msg.linear_acceleration.x = accel.acceleration.x; 
  msg.linear_acceleration.y = accel.acceleration.y;
  msg.linear_acceleration.z = accel.acceleration.z;

  
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  //msg.data++;

  Serial.print(msg.angular_velocity.x);
  Serial.println(" m/s");

  Serial.print(gyro.gyro.x);
  Serial.println(" rad/s");

    
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  Serial.begin(115200);
  Serial.println("Successfully got into setup script");

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  Serial.println("Initial options successfully created");


  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "micro_ros_arduino_imu_publisher"));

  Serial.println("Publisher sucessfully created");


  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  //msg.data = 0;

  Serial.println("Executor succesfully created");
  //Serial.println(msg.data);
  
  if (!mpu.begin()) {
    while (1) {
      delay(10);
      Serial.println("Failed to find MPU6050 chip");
     ;
    }
  }
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
  
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
