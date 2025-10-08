#include "identifier/identifier.hpp"
#include <atomic>
#include <chrono>
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>

namespace rmcs_core::identifier {

class IdentifierController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    IdentifierController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        // camera_profile_.invert_image = false;
        camera_profile_.exposure_time =
            std::chrono::microseconds(get_parameter("exposure").as_int());
        camera_profile_.gain = static_cast<float>(get_parameter("gain").as_double());
        image_capture_ = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);

        register_output("/ident/camera/camera", camera_image_);
        register_output("/ident/camera/display", display_image_);

        const auto lower = get_parameter("hsv_lower").as_double_array();
        const auto upper = get_parameter("hsv_upper").as_double_array();
        identifier_ = Identifier(
            cv::Scalar(lower[0], lower[1], lower[2]), cv::Scalar(upper[0], upper[1], upper[2]),
            logger_);

        camera_thread_ = std::thread(&IdentifierController::camera_runtime, this);
    }

    void update() override {}

private:
    std::thread camera_thread_;
    std::atomic<bool> camera_enable_flag_ = true;

    Identifier identifier_;

    rclcpp::Logger logger_;

    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;

    OutputInterface<cv::Mat> camera_image_;
    OutputInterface<cv::Mat> display_image_;

    void camera_runtime() {
        while (camera_enable_flag_) {
            cv::Mat read = image_capture_->read();

            *camera_image_ = read;
            identifier_.update_image(read);
            identifier_.find_targets();

            cv::Mat display;
            // cv::cvtColor(read, display, cv::COLOR_BGR2HSV);
            display = read.clone();

            std::vector<std::vector<cv::Point>> rect_points;
            for (const auto& light : identifier_.get_lights()) {
                std::array<cv::Point2f, 4> points;
                light.points(points.begin());
                std::vector<cv::Point> int_points;
                int_points.reserve(points.size());
                for (const auto& point : points)
                    int_points.emplace_back(point);
                rect_points.emplace_back(int_points);
            }
            cv::drawContours(display, rect_points, -1, cv::Scalar(0, 255, 0), 2);

            for (const auto& [left, right] : identifier_.get_armors()) {
                std::array<cv::Point2f, 4> left_points;
                left.points(left_points.begin());
                std::array<cv::Point2f, 4> right_points;
                right.points(right_points.begin());
                // cross-line for each armor,
                // left light top right to right light bottom left
                cv::line(display, left_points[2], right_points[0], cv::Scalar(255, 0, 0), 2);
                // left light bottom right to right light top left
                cv::line(display, left_points[3], right_points[1], cv::Scalar(255, 0, 0), 2);
            }
            cv::drawContours(display, identifier_.get_contours(), -1, cv::Scalar(0, 0, 255), 1);

            cv::putText(
                display, std::format("Lights: {}", identifier_.get_lights().size()),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

            cv::imshow("rmcs", display);
            cv::waitKey(1);

            *display_image_ = display;
        }
    }
};

} // namespace rmcs_core::identifier

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::identifier::IdentifierController, rmcs_executor::Component)
