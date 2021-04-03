#include "RobotNeuralNet.hpp"

#include <fdeep/fdeep.hpp>
#include <vector>
#include <cmath>
#include <cinttypes>

namespace RobotNeuralNet
{
    constexpr static uint16_t CROPPED_IMG_WIDTH = 64;
    constexpr static uint16_t CROPPED_IMG_HEIGHT = 128;
    constexpr static uint16_t ORIGINAL_IMG_WIDTH = 640;
    constexpr static uint16_t ORIGINAL_IMG_HEIGHT = 460;
    constexpr static uint16_t TOP_CROP_ROWS = 100;
    constexpr static uint16_t BOTTOM_CROP_ROWS = 70;

    const static auto model = fdeep::load_model("modelo_clasificador_robot.json");


    cv::Mat preprocess_img(cv::Mat img)
    {
        cv::Rect rect(0, BOTTOM_CROP_ROWS, ORIGINAL_IMG_WIDTH, ORIGINAL_IMG_HEIGHT - TOP_CROP_ROWS);
        cv::Mat cropped_img = img(rect);
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(CROPPED_IMG_WIDTH, CROPPED_IMG_HEIGHT), 0, 0, CV_INTER_LANCZOS4);
        return resized_img;
    }

    bool is_robot_present(cv::Mat const& img)
    {
        if (img.rows == 0)
            return false;
        const auto img_processed = preprocess_img(img);
        cv::imwrite("imagen_pre_red.jpg", img_processed);
        // printf("rows: %d, cols: %d\n", img_processed.rows, img_processed.cols);
        const auto input = fdeep::tensor_from_bytes
        (
            img_processed.ptr(),
            static_cast<std::size_t>(CROPPED_IMG_WIDTH),   //IMG_WIDTH
            static_cast<std::size_t>(CROPPED_IMG_HEIGHT),  //IMG_HEIGHT
            static_cast<std::size_t>(1)
        );
        const auto result = model.predict_class( {input} );
        return result;
    }
}