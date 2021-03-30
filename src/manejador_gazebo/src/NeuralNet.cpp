#include "NeuralNet.hpp"

#include <fdeep/fdeep.hpp>
#include <vector>
#include <cmath>
#include <cinttypes>

namespace NeuralNet
{
    const static auto model 
    = fdeep::load_model("tf_model_driving_frugal.json");

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
        // cv::Rect rect(0, 0, 640, 430);
        // cv::Mat cropped_img = img(rect);
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(64, 128), 0, 0, CV_INTER_LANCZOS4);
        return resized_img;
    }

    KeyboardAction predict(cv::Mat img)
    {
        if (img.rows == 0)
            return KeyboardAction::NO_ACTION;
        const auto img_processed = preprocess_img(img);
        // cv::Mat processed_img = preprocess_img(img);
        // printf("rows: %d, cols: %d\n", resized_img.rows, resized_img.cols);
        const auto input = fdeep::tensor_from_bytes
        (
            img_processed.ptr(),
            static_cast<std::size_t>(128),  //IMG_HEIGHT
            static_cast<std::size_t>(64),   //IMG_WIDTH
            static_cast<std::size_t>(1)
        );
        const auto result = model.predict_class( {input} );
        std::cout << result << std::endl;
        return get_action(result);
    }
}