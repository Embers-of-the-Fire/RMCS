#include <cmath>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::component_connection {

class Process
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Process()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input(get_parameter("sin").as_string(), sin_);
        register_input(get_parameter("cos").as_string(), cos_);
        register_output(get_parameter("out").as_string(), sum_);
    }

    void update() override { *sum_ = *sin_ + *cos_; }

private:
    InputInterface<double> sin_;
    InputInterface<double> cos_;

    OutputInterface<double> sum_;
};

} // namespace rmcs_core::controller::component_connection

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::component_connection::Process, rmcs_executor::Component)
