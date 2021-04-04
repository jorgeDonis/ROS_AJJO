#include "RobotController.hpp"
#include "NeuralNet.hpp"
#include "RobotNeuralNet.hpp"

#include <geometry_msgs/Twist.h>
#include <cmath>

const std::string RobotController::TWIST_TOPIC = "/robot1/mobile_base/commands/velocity";

RobotController::RobotController(ros::NodeHandle& nh) : 
img_capturer(nh), odom_capturer(nh), rgb_img_capturer(nh)
{
    node_handle = nh;
    pub = nh.advertise<geometry_msgs::Twist>(TWIST_TOPIC, QUEUE_SIZE);
}

void RobotController::apply_keyboard_action(KeyboardAction action)
{
    switch (action)
    {
        case KeyboardAction::STEER_LEFT:
            rotation_speed = TURNING_SPEED;
            forward_speed = FORWARD_SPEED_TURNING;
            break;
        case KeyboardAction::STEER_RIGHT:
            rotation_speed = -TURNING_SPEED;
            forward_speed = FORWARD_SPEED_TURNING;
            break;
        case KeyboardAction::NO_ACTION:
            forward_speed = FORWARD_SPEED;
            rotation_speed = 0;
            break;
    }
}

void RobotController::main_loop()
{
    ros::Rate rate(RobotController::RATE);
    while (ros::ok())
    {
        const cv::Mat img = img_capturer.get_img();
        KeyboardAction action = NeuralNet::predict(img, odom_capturer.get_info());
        apply_keyboard_action(action);

        geometry_msgs::Twist msg;

        msg.linear.x = forward_speed;
        msg.angular.z = rotation_speed;
        // pub.publish(msg);

        // printf("FORWARD_SPEED = %.2f\n", forward_speed);
        // printf("ROTATION_SPEED = %.2f\n\n", rotation_speed);

        if (RobotNeuralNet::is_robot_present(rgb_img_capturer.get_img()))
            printf("ROBOT IS PRESENT\n");
        else
            printf("ROBOT IS NOT PRESENT\n");
        
        cls();

        ros::spinOnce();
        rate.sleep();
    }
}