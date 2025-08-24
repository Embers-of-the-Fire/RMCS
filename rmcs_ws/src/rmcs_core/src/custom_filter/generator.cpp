#include <random>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::custom_filter::generator {

class Generator
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Generator()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , amp_(get_parameter("amplitude").as_double())
        , gen(std::random_device()()) // hacky... but works well
        , dis(0.0, amp_ / 10.0) {
        register_output(get_parameter("output").as_string(), value_);
    }

    void update() override {
        tick_++;

        const auto std_sin = std::sin(static_cast<double>(tick_) / 1000.0);
        const auto rand = dis(gen);

        *value_ = std_sin + rand;
    }

private:
    const double amp_;

    mutable size_t tick_ = 0;

    mutable std::default_random_engine gen;
    mutable std::normal_distribution<double> dis;

    OutputInterface<double> value_;
};

} // namespace rmcs_core::custom_filter::generator

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::custom_filter::generator::Generator, rmcs_executor::Component)
