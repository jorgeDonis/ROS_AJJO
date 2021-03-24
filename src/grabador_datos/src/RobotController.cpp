#include "RobotController.hpp"
#include "DatasetGenerator.hpp"

#include <geometry_msgs/Twist.h>
#include <SDL/SDL.h>
#include <cmath>

const std::string RobotController::TWIST_TOPIC = "/robot1/mobile_base/commands/velocity";

RobotController::RobotController(ros::NodeHandle& nh) : img_capturer(nh)
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
            break;
        case KeyboardAction::STEER_RIGHT:
            rotation_speed = -TURNING_SPEED;
            break;
        case KeyboardAction::BRAKE:
            forward_speed = BREAK_SPEED;
            break;
        case KeyboardAction::NO_ACTION:
            forward_speed = FORWARD_SPEED;
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
    while (SDL_PollEvent(&event))
    {
        if (event.type == SDL_KEYDOWN)
        {
            switch (event.key.keysym.sym)
            {
            case SDLK_SPACE:
                action = KeyboardAction::BRAKE;
                break;
            case SDLK_LEFT:
                turning_left = true;
                action = KeyboardAction::STEER_LEFT;
                break;
            case SDLK_RIGHT:
                turning_right = true;
                action = KeyboardAction::STEER_RIGHT;
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

        dataset_gen.add_to_dataset(action, img_capturer.get_img());

        ros::spinOnce();
        rate.sleep();
    }
}