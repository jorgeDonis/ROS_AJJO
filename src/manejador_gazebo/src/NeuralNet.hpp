#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "KeyboardAction.hpp"

namespace NeuralNet
{
    KeyboardAction predict(cv::Mat img);
}