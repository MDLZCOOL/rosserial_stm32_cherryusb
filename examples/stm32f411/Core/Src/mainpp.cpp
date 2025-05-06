#include <ros.h>
#include <std_msgs/String.h>
#include "mainpp.h"
#include "main.h"
#include "stm32f4xx_hal.h"

void messageCallback(const std_msgs::String &msg) {
    if (strcmp("led_on", msg.data) == 0) {
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
    } else if (strcmp("led_off", msg.data) == 0)
    {
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    }
    else
    {
        ;
    }
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::String> sub("/stm32_ros_bridge", &messageCallback);

void setup(void) {
    nh.initNode();
    nh.subscribe(sub);
}

void loop(void) {
    nh.spinOnce();
}