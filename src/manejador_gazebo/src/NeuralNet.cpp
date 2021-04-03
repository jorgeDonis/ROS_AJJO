#include "NeuralNet.hpp"

#include <fdeep/fdeep.hpp>
#include <vector>
#include <cmath>
#include <cinttypes>

namespace NeuralNet
{
    const static uint16_t IMG_WIDTH = 180;
    const static uint16_t IMG_HEIGHT = 128;
    
    const static auto model 
    = fdeep::load_model("tf_model_driving_general.json");

    KeyboardAction get_action(const size_t best_index)
    {
        switch (best_index)
        {
            case 0:
                return KeyboardAction::NO_ACTION;
            case 1:
                return KeyboardAction::STEER_LEFT;
            case 2:
                return KeyboardAction::STEER_RIGHT;
        }
    }

    const auto get_odom_tensor(OdomInfo const& odom_info)
    {
        return fdeep::tensor
        (
            fdeep::tensor_shape(static_cast<std::size_t>(3)),
            std::vector<float>
            {
                odom_info.x   / 20  ,
                odom_info.y   / 20  ,
                odom_info.yaw / 3.141592f 
            }
        );
    }

    cv::Mat preprocess_img(cv::Mat img)
    {
        // cv::Rect rect(0, 0, 640, 430);
        // cv::Mat cropped_img = img(rect);
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(IMG_WIDTH, IMG_HEIGHT), 0, 0, CV_INTER_LINEAR);
        return resized_img;
    }

    KeyboardAction predict(cv::Mat const& img, OdomInfo const& odom_info)
    {
        if (img.rows == 0)
            return KeyboardAction::NO_ACTION;
        const auto img_processed = preprocess_img(img);
        // cv::Mat processed_img = preprocess_img(img);
        // printf("rows: %d, cols: %d\n", resized_img.rows, resized_img.cols);
        const auto input = fdeep::tensor_from_bytes
        (
            img_processed.ptr(),
            static_cast<std::size_t>(IMG_HEIGHT),  //IMG_HEIGHT
            static_cast<std::size_t>(IMG_WIDTH),   //IMG_WIDTH
            static_cast<std::size_t>(1)
        );
        const auto result = model.predict_class( {input, get_odom_tensor(odom_info)} );
        return get_action(result);
    }
}