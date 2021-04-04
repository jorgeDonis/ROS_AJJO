#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "KeyboardAction.hpp"
#include "OdomCapturer.hpp"

namespace NeuralNet
{
    KeyboardAction predict(cv::Mat const& img, OdomInfo const& info);
}