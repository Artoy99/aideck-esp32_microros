/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP deck firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include "discovery.h"

#include "esp_transport.h"
#include "spi_transport.h"
#include "uart_transport.h"
#include "router.h"
#include "com.h"
#include "test.h"
#include "wifi.h"
#include "system.h"

#include <string.h>
#include <unistd.h>

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>

#define CONFIG_MICRO_ROS_APP_STACK 10000
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 send_msg;
std_msgs__msg__Int32 recv_msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		printf("Publishing: %d\n", send_msg.data);
		RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
		send_msg.data++;
	}
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	// printf("Received: %d\n", msg->data);
    ESP_LOGI("ROS", "Message received");
}

void micro_ros_task(void * arg)
{
    // ESP_LOGI("ROS", "uROS task started");
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
    // ESP_LOGI("ROS", "uROS checkpoint 0");

    vTaskDelay(1000);
    
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
    // ESP_LOGI("ROS", "uROS checkpoint 1");

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	// RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
    // ESP_LOGI("ROS", "uROS checkpoint 2");

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    // rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    // ESP_LOGI("ROS", "uROS checkpoint 3");

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "esp32_int32_publisher", "", &support));
    // ESP_LOGI("ROS", "uROS checkpoint 4");

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"freertos_int32_publisher")
    );
    // ESP_LOGI("ROS", "uROS checkpoint 4");

    RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"int32_subscriber")
    );

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback)
    );
    // ESP_LOGI("ROS", "uROS checkpoint 5");

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));
    // ESP_LOGI("ROS", "uROS checkpoint 6");

	send_msg.data = 0;
    // ESP_LOGI("ROS", "uROS checkpoint 7");

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

/* The LED is connected on GPIO */
#define BLINK_GPIO 4

static esp_routable_packet_t txp;
int cpx_and_uart_vprintf(const char * fmt, va_list ap) {
    int len = vprintf(fmt, ap);

    cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_CONSOLE, &txp.route);
    txp.dataLength = vsprintf((char*)txp.data, fmt, ap) + 1;
    espAppSendToRouterBlocking(&txp);

    return len;
}



#define DEBUG_TXD_PIN (GPIO_NUM_0) // Nina 27 /SYSBOOT) => 0

int a = 1;

void app_main(void)
{
    // Menuconfig option set to DEBUG
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("SPI", ESP_LOG_INFO);
    esp_log_level_set("UART", ESP_LOG_INFO);
    esp_log_level_set("SYS", ESP_LOG_INFO);
    esp_log_level_set("ROUTER", ESP_LOG_INFO);
    esp_log_level_set("COM", ESP_LOG_INFO);
    esp_log_level_set("TEST", ESP_LOG_INFO);
    esp_log_level_set("WIFI", ESP_LOG_INFO);
    esp_log_level_set("ROS", ESP_LOG_INFO);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_log_level_set("DISCOVERY", ESP_LOG_INFO);
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 1);

    // Intalling GPIO ISR service so that other parts of the code can
    // setup individual GPIO interrupt routines
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);

    spi_transport_init();

    const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, 1000, 1000, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 0, 25, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI("SYS", "\n\n -- Starting up --\n");
    ESP_LOGI("SYS", "Minimum free heap size: %d bytes", esp_get_minimum_free_heap_size());

    espTransportInit();
    uart_transport_init();
    com_init();

    // TODO krri remove test
    test_init();

    wifi_init();
    router_init();

    esp_log_set_vprintf(cpx_and_uart_vprintf);

    system_init();

    discovery_init();

    // ESP_ERROR_CHECK(uros_network_interface_initialize());
    xTaskCreate(micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL
    );

    ESP_LOGI("ROS", "Task created");

    while(1) {
        vTaskDelay(10);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(10);
        gpio_set_level(BLINK_GPIO, 0);
    }
    esp_restart();
}
