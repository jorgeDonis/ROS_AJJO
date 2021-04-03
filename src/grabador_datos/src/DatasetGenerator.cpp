#include "DatasetGenerator.hpp"

#include <chrono>

const std::string DatasetGenerator::DATASET_PATH = "./dataset";

std::string DatasetGenerator::to_str(KeyboardAction action) const
{
    switch (action)
    {
        case KeyboardAction::NO_ACTION:
            return "NO_ACTION";
            break;
        case KeyboardAction::STEER_LEFT:
            return "STEER_LEFT";
            break;
        case KeyboardAction::STEER_RIGHT:
            return "STEER_RIGHT";
            break;
    }
}

void DatasetGenerator::add_to_dataset(KeyboardAction user_input, cv::Mat const& img, OdomInfo const &odom_info)
{
    using namespace std::chrono;
    const std::string timestamp = std::to_string(
        duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()
    );
    std::string filename = to_str(user_input) + "_" + odom_info.to_string() + "_" 
                           +  timestamp + ".jpg";
    std::cout << timestamp << std::endl;
    cv::imwrite(DATASET_PATH + "/" + filename, img);
}