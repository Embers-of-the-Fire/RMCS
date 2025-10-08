#pragma once

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <optional>
#include <rclcpp/node.hpp>
#include <utility>
#include <vector>

namespace rmcs_core::identifier {

class Identifier {
    using Rect = cv::RotatedRect;
    using Point = cv::Point;

private:
    std::optional<cv::Mat> image_;

    std::vector<std::vector<Point>> contours_;
    std::vector<Rect> lights_;
    std::vector<std::tuple<Rect, Rect>> armors_;

    std::optional<rclcpp::Logger> logger_;

public:
    cv::Scalar lower;
    cv::Scalar upper;

    Identifier() = default;

    explicit Identifier(
        cv::Scalar lower, cv::Scalar upper, std::optional<rclcpp::Logger> logger = std::nullopt)
        : logger_(std::move(logger))
        , lower(std::move(lower))
        , upper(std::move(upper)) {}

    void update_image(const cv::Mat& image) {
        image_ = image;
        lights_.clear();
        armors_.clear();
    }
    std::optional<cv::Mat> get_image() const { return image_; }

    void find_targets() {
        cv::Mat hsv, mask;
        contours_.clear();
        cv::cvtColor(image_.value(), hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, lower, upper, mask);

        cv::findContours(mask, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::sort(
            contours_.begin(), contours_.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) > cv::contourArea(b);
            });

        for (const auto& contour : contours_) {
            const auto rect = cv::minAreaRect(contour);
            lights_.emplace_back(rect);
        }

        std::sort(lights_.begin(), lights_.end(), [](const Rect& a, const Rect& b) {
            return a.center.x < b.center.x;
        });

        if (lights_.size() % 2 != 0)
            lights_.pop_back();

        for (size_t i = 0; i < lights_.size(); i += 2)
            armors_.emplace_back(lights_[i], lights_[i + 1]);

        // if (logger_) {
        //     const auto pixel = hsv.at<cv::Vec3b>(400, 800);
        //     RCLCPP_INFO(logger_.value(), "Pixel: %d, %d, %d", pixel[0], pixel[1], pixel[2]);
        // }

        // cv::imshow("rmcs", hsv);
        // cv::waitKey(1);
    };

    std::vector<std::tuple<Rect, Rect>> get_armors() const { return armors_; }
    std::vector<Rect> get_lights() const { return lights_; }
    auto get_contours() const { return contours_; }
};

}; // namespace rmcs_core::identifier
