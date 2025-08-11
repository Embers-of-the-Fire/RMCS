#include <cmath>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::component_connection {

class Emit
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Emit()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , omega_(get_parameter("omega").as_double())
        , counter_(0) {

        register_output(get_parameter("sin_output").as_string(), sin_);
        register_output(get_parameter("cos_output").as_string(), cos_);
    }

    void update() override {
        const double time = counter_ * time_per_frame;
        *sin_ = std::sin(omega_ * time);
        *cos_ = std::cos(omega_ * time);

        counter_++;
    }

private:
    static constexpr double time_per_frame = 1.0 / 1000;

    const double omega_;

    mutable uint32_t counter_;
    OutputInterface<double> sin_;
    OutputInterface<double> cos_;
};

} // namespace rmcs_core::controller::component_connection

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::component_connection::Emit, rmcs_executor::Component)
