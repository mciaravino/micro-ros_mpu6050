bool create_entities()
{
  
  /*Creates all ROS Entities*/
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_imu_node", "", &support));
  const unsigned int timer_timeout = 5;//This timer determines how often messges are published, in milliseconds. A value of 5 means we expect 200 messages per second
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  /*End ROS Enttities*/

  /*Creates all IMU publishers*/
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
  /*End IMU Publishers*/
  
  return true;
}
