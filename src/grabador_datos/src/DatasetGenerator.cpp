#include "DatasetGenerator.hpp"

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

void DatasetGenerator::add_to_dataset(KeyboardAction user_input, cv::Mat const &img, OdomInfo const &odom_info)
{
    std::string filename = to_str(user_input) + "_" + std::to_string(num_images) + ".jpg";
    printf("adding image %s to dataset\n", filename.c_str());
    cv::imwrite(DATASET_PATH + "/" + filename, img);
    ++num_images;
}

DatasetGenerator::DatasetGenerator()
{
    std::string s = "rm " + DATASET_PATH + "/*";
    system(s.c_str());
    num_images = 0;
}