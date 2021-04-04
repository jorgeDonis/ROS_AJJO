#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "KeyboardAction.hpp"
#include "OdomCapturer.hpp"

namespace RobotNeuralNet
{
    bool is_robot_present(cv::Mat const& img);
}