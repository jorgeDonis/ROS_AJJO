#include "RobotController.hpp"

#include <geometry_msgs/Twist.h>
#include <SDL/SDL.h>
#include <cmath>

const std::string RobotController::TWIST_TOPIC = "/mobile_base/commands/velocity";

RobotController::RobotController(ros::NodeHandle& nh)
{
    node_handle = nh;
    pub = nh.advertise<geometry_msgs::Twist>(TWIST_TOPIC, QUEUE_SIZE);
    pressed_keys.reserve(3);
}

void RobotController::init_SDL() const
{
    SDL_Init(SDL_INIT_VIDEO);
    SDL_SetVideoMode(100, 100, 32, SDL_DOUBLEBUF | SDL_OPENGL);
    SDL_EnableUNICODE(1);
}

void RobotController::apply_speeds()
{
    switch (state)
    {
        case State::IDLE:
            forward_speed = 0;
            rotation_speed = 0;
            break;
        case State::MOVING_FORWARD:
            forward_speed = FORWARD_SPEED;
            rotation_speed = 0;
            break;
        case State::STEERING_LEFT:
            forward_speed = 0;
            rotation_speed = TURNING_SPEED;
            break;
        case State::STEERING_RIGHT:
            forward_speed = 0;
            rotation_speed = -TURNING_SPEED;
    }
}

void RobotController::update_state()
{
    if (!pressed_keys.empty())
    {
        const Key& last_pressed_key = pressed_keys.back();
        switch (last_pressed_key)
        {
            case UP:
                state = State::MOVING_FORWARD;
                break;
            case LEFT:
                state = State::STEERING_LEFT;
                break;
            case RIGHT:
                state = State::STEERING_RIGHT;
                break;
            }
    }
    else
        state = State::IDLE;
}

void RobotController::delete_key(Key to_delete)
{
    for (uint8_t i = 0; i < pressed_keys.size(); ++i)
    {
        if (pressed_keys[i] == to_delete) 
        {
            pressed_keys.erase(pressed_keys.begin() + i);
            break;
        }
    }
}

void RobotController::update_pressed_keys()
{
    SDL_Event event;
    bool read_keys = true;
    while (SDL_PollEvent(&event))
    {
        if (event.type == SDL_KEYDOWN && read_keys)
        {
            switch (event.key.keysym.sym)
            {
                case SDLK_LEFT:
                    pressed_keys.push_back(Key::LEFT);
                    break;
                case SDLK_RIGHT:
                    pressed_keys.push_back(Key::RIGHT);
                    break;
                case SDLK_UP:
                    pressed_keys.push_back(Key::UP);
                    break;
            }
        }
        else if (event.type == SDL_KEYUP)
        {
            switch (event.key.keysym.sym)
            {
                case SDLK_LEFT:
                    delete_key(Key::LEFT);
                    break;
                case SDLK_RIGHT:
                    delete_key(Key::RIGHT);
                    break;
                case SDLK_UP:
                    delete_key(Key::UP);
                    break;
            }
        }
    }
}

void RobotController::print_state() const
{
    using namespace std;
    switch (state)
    {
        case State::IDLE:
            cout << "IDLE" << endl;
            break;
        case State::MOVING_FORWARD:
            cout << "MOVING_FORWARD" << endl;
            break;
        case State::STEERING_LEFT:
            cout << "STEERING_LEFT" << endl;
            break;
        case State::STEERING_RIGHT:
            cout << "STEERING_RIGHT" << endl;
            break;
        }
}

void RobotController::main_loop()
{
    init_SDL();
    ros::Rate rate(RobotController::RATE);
    while (ros::ok())
    {
        using namespace std;
        update_pressed_keys();
        update_state();
        apply_speeds();

        // print_state();

        geometry_msgs::Twist msg;

        msg.linear.x = forward_speed;
        msg.angular.z = rotation_speed;
        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }
}