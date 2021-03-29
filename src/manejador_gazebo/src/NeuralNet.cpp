#include "NeuralNet.hpp"

#include <fdeep/fdeep.hpp>
#include <vector>
#include <cmath>
#include <cinttypes>

namespace NeuralNet
{
    const static auto model 
    = fdeep::load_model("/home/jorge/catkin_ws/src/manejador_gazebo/tf_model_driving_frugal.json");

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

    cv::Mat preprocess_img(cv::Mat img)
    {
        cv::Rect rect(0, 0, 640, 430);
        cv::Mat cropped_img = img(rect);
        cv::Mat resized_img;
        cv::resize(cropped_img, resized_img, cv::Size(256, 128), 0, 0, CV_INTER_LANCZOS4);
        return resized_img;
    }

    KeyboardAction predict(cv::Mat img)
    {
        if (img.rows == 0)
            return KeyboardAction::NO_ACTION;
        cv::Mat processed_img = preprocess_img(img);
        // printf("rows: %d, cols: %d\n", resized_img.rows, resized_img.cols);
        const auto input = fdeep::tensor_from_bytes
        (
            processed_img.ptr(),
            static_cast<std::size_t>(128),
            static_cast<std::size_t>(256),
            static_cast<std::size_t>(1)
        );
        const auto result = model.predict_class( {input} );
        std::cout << result << std::endl;
        return get_action(result);
    }
}