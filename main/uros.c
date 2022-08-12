#include "uros.h"

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
    ESP_LOGI("ROS", "Message received: %d", msg->data);
    if(msg->data)
    {
        // Send connection info to GAP8
        esp_routable_packet_t connection;

        cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_WIFI_CTRL, &connection.route);
        connection.data[0] = 0x32; // WIFI_CTRL_STATUS_CLIENT_CONNECTED;
        connection.data[1] = 1;    // connected
        connection.dataLength = 2;
        espAppSendToRouterBlocking(&connection);
    }else{
        // Send deconnection info to GAP8, not working
        esp_routable_packet_t connection;

        cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_WIFI_CTRL, &connection.route);
        connection.data[0] = 0x32; // WIFI_CTRL_STATUS_CLIENT_CONNECTED;
        connection.data[1] = 0;    // disconnected
        connection.dataLength = 2;
        espAppSendToRouterBlocking(&connection);
    }
}

void micro_ros_task(void * arg)
{
    ESP_LOGI("ROS", "MAIN task started");

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
	RCCHECK(rclc_node_init_default(&node, "crazyflie_esp32_node", "", &support));
    // ESP_LOGI("ROS", "uROS checkpoint 4");

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"freertos_int32_publisher")
    );

    RCCHECK(rclc_publisher_init_default(
		&img_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
		"image_streaming")
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

    send_img.width = CAMERA_WIDTH;
    send_img.height = CAMERA_HEIGHT;
    send_img.is_bigendian = CAMERA_IS_BIGENDIAN;
    send_img.encoding.data = "jpeg";
    send_img.data.capacity = CAMERA_IMAGE_SIZE;
    // send_img.data.data = (uint8_t*) malloc(CAMERA_IMAGE_SIZE*sizeof(uint8_t));
    send_img.data.size = 0;
    // uint64_t matrix_size = CAMERA_IMAGE_SIZE// send_img.step * send_img.height;
    // uint8_t data_image [CAMERA_IMAGE_SIZE];
    // ESP_LOGI("ROS", "img data size: %d", sizeof(send_img.data.data));
    // ESP_LOGI("ROS", "data size: %d", sizeof(data_image));
    // memcpy(&send_img.data.data, &data_image, CAMERA_IMAGE_SIZE);
    // send_img.data.data = data_image;

    RCSOFTCHECK(rcl_publish(&img_publisher, &send_img, NULL));

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_node_fini(&node));

    free(send_img.data.data);

  	vTaskDelete(NULL);
}

void micro_image_task()
{
    ESP_LOGI("ROS", "IMAGE task started");
    // static uint8_t data_image [CAMERA_IMAGE_SIZE];
    static uint32_t count = 0;
    // ESP_LOGI("ROS", "image task variable created");
    while(1)
    {
        com_receive_image_blocking(&packet);
        // Image header from python script: bytearray(b'\xbcD\x01\xf4\x00\x01\x00\xd04\x01\x00')
        // Magic: 0xBC
        // Width: 324
        // Height: 244
        // Depth: 1
        // Format: 0
        // Size: 79056
        if(packet.data[0] == 0xbc){
            ESP_LOGI("ROS", "magic is good");
            // ESP_LOGI("ROS", "data lenght: %d", packet.dataLength);
            ESP_LOGI("ROS", "count: %d", count);
            // ESP_LOGI("ROS", "img data size: %d", sizeof(send_img.data.data));
            // ESP_LOGI("ROS", "data size: %d", sizeof(data_image));
            
            // send_img.data.data = data_image;
            
            count = 0;

            // RCSOFTCHECK(rcl_publish(&img_publisher, &send_img, NULL));
        }else{
            ESP_LOGI("ROS", "data lenght: %d", packet.dataLength);
            
            // for(int i = 0; i < packet.dataLength; i++){
            //     data_image[(count*1020)+i] = packet.data[i];
            // }
            count++;
        }
        // ESP_LOGI("ROS", "%d", packet.data[0]);
        vTaskDelay(1);
    }
}

void uros_init()
{
    xTaskCreate(micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL
    );

    xTaskCreate(micro_image_task,
        "uros_image_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL
    );

    ESP_LOGI("ROS", "Tasks created");
}