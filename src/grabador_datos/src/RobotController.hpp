#pragma once

#include "ImageCapturer.hpp"
#include "DatasetGenerator.hpp"
#include "KeyboardAction.hpp"
#include "OdomCapturer.hpp"

#include <ros/ros.h>
#include <cstring>
#include <cinttypes>

class RobotController
{
    private:
        constexpr static uint8_t RATE = 10;
        constexpr static uint8_t QUEUE_SIZE = 10;
        constexpr static float V_0 = 0.1;
        constexpr static float FORWARD_SPEED = 0.90;
        constexpr static float TURNING_SPEED = 0.6;
        constexpr static float FORWARD_SPEED_TURNING = 0.6;

        static const std::string TWIST_TOPIC;

        void init_SDL() const;

        void apply_keyboard_action(KeyboardAction action);

        KeyboardAction read_keyboard_input();

        float forward_speed = V_0;
        float rotation_speed = 0;
        bool turning_left = false;
        bool turning_right = false;

        ros::NodeHandle node_handle;
        ros::Publisher pub;
        ImageCapturer img_capturer;
        OdomCapturer odom_capturer;
        DatasetGenerator dataset_gen;
    public:
        RobotController(ros::NodeHandle& node_handle);
        void main_loop();
};