#pragma once

#include "KeyboardAction.hpp"
#include "OdomCapturer.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <string>

class DatasetGenerator
{
    private:
        static const std::string DATASET_PATH;
        std::string to_str(KeyboardAction action) const;
    public:
        void add_to_dataset(KeyboardAction user_input, cv::Mat const &img, OdomInfo const& info);
};