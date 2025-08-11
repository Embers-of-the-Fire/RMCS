#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::rmcs_simulator_bridge::executor {

class RExecutor
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    RExecutor()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , target_(get_parameter("target_velocity").as_double()) {
        register_output(get_parameter("output").as_string(), velocity_target_);
    }

    void update() override { *velocity_target_ = target_; }

private:
    const double target_;
    OutputInterface<double> velocity_target_;
};

} // namespace rmcs_core::rmcs_simulator_bridge::executor

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::rmcs_simulator_bridge::executor::RExecutor, rmcs_executor::Component)
