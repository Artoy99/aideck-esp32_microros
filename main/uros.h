#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>

#include "com.h"
#include "esp_transport.h"
#include "wifi.h"

#include <sensor_msgs/msg/image.h>
#define CAMERA_WIDTH 324
#define CAMERA_HEIGHT 244
#define CAMERA_IS_BIGENDIAN 0
#define CAMERA_IMAGE_SIZE 79056

#define CONFIG_MICRO_ROS_APP_STACK 10000
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 send_msg;
std_msgs__msg__Int32 recv_msg;

rcl_publisher_t img_publisher;
sensor_msgs__msg__Image send_img;

esp_routable_packet_t packet;

void uros_init();
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void subscription_callback(const void * msgin);
void micro_ros_task(void * arg);