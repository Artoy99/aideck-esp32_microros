#include <rclc/rclc.h>

void uros_init();
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void subscription_callback(const void * msgin);
void micro_ros_task(void * arg);