#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <SDL/SDL.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cinttypes>
#include <vector>

enum class State
{
    IDLE,
    MOVING_FORWARD,
    STEERING_RIGHT,
    STEERING_LEFT,
};

enum Key
{
    UP,
    LEFT,
    RIGHT
};

class RobotController
{
    private:
        constexpr static uint8_t RATE = 10;
        constexpr static uint8_t QUEUE_SIZE = 10;
        constexpr static float FORWARD_SPEED = 0.3;
        constexpr static float TURNING_SPEED = 0.3;

        static const std::string TWIST_TOPIC;
        static const std::string PC_TOPIC;
        static const std::string BAG_FILENAME;
        static const std::string BAG_TOPIC;

        void init_SDL() const;

        void print_state() const;
        void delete_key(Key to_delete);

        std::vector<Key> pressed_keys;
        State state = State::IDLE;

        void update_pressed_keys();
        void update_state();
        void apply_speeds();

        float forward_speed = 0;
        float rotation_speed = 0;

        void pc_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) const;

        ros::NodeHandle node_handle;
        ros::Publisher pub;
        ros::Subscriber pc_sub;
    public:
        RobotController(ros::NodeHandle& node_handle);
        void main_loop();
};