#include "RobotController.hpp"
#include "DatasetGenerator.hpp"

#include <geometry_msgs/Twist.h>
#include <SDL/SDL.h>
#include <cmath>

const std::string RobotController::TWIST_TOPIC = "/robot1/mobile_base/commands/velocity";

RobotController::RobotController(ros::NodeHandle& nh) : img_capturer(nh), odom_capturer(nh)
{
    node_handle = nh;
    pub = nh.advertise<geometry_msgs::Twist>(TWIST_TOPIC, QUEUE_SIZE);
}

void RobotController::init_SDL() const
{
    SDL_Init(SDL_INIT_VIDEO);
    SDL_SetVideoMode(100, 100, 32, SDL_DOUBLEBUF | SDL_OPENGL);
    SDL_EnableUNICODE(1);
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

KeyboardAction RobotController::read_keyboard_input()
{
    KeyboardAction action = KeyboardAction::NO_ACTION;
    if (turning_left)
        action = KeyboardAction::STEER_LEFT;
    else if (turning_right)
        action = KeyboardAction::STEER_RIGHT;
    SDL_Event event;
    bool read_keys = true;
    while (SDL_PollEvent(&event))
    {
        if (event.type == SDL_KEYDOWN && read_keys)
        {
            switch (event.key.keysym.sym)
            {
                case SDLK_LEFT:
                    turning_left = true;
                    action = KeyboardAction::STEER_LEFT;
                    read_keys = false;
                    break;
                case SDLK_RIGHT:
                    turning_right = true;
                    action = KeyboardAction::STEER_RIGHT;
                    read_keys = false;
                    break;
            }
        }
        else if (event.type == SDL_KEYUP)
        {
            switch (event.key.keysym.sym)
            {
                case SDLK_LEFT:
                    turning_left = false;
                    break;
                case SDLK_RIGHT:
                    turning_right = false;
                    break;
            }
        }
    }
    return action;
}

void RobotController::main_loop()
{
    init_SDL();
    ros::Rate rate(RobotController::RATE);
    while (ros::ok())
    {
        KeyboardAction action = read_keyboard_input();
        apply_keyboard_action(action);

        geometry_msgs::Twist msg;

        msg.linear.x = forward_speed;
        msg.angular.z = rotation_speed;
        pub.publish(msg);

        printf("FORWARD_SPEED = %.2f\n", forward_speed);
        printf("ROTATION_SPEED = %.2f\n\n", rotation_speed);
        const OdomInfo odom_info = odom_capturer.get_info();

        printf("YAW = %.2f\n\n", odom_info.yaw);
        const auto img = img_capturer.get_img();

        if (img.rows != 0)
            dataset_gen.add_to_dataset(action, img, odom_info);

        ros::spinOnce();
        rate.sleep();
    }
}