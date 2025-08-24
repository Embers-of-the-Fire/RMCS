#include <librmcs/utility/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "custom_filter/filters.hpp"

namespace rmcs_core::custom_filter::filter {

class Filter
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Filter()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input(get_parameter("input").as_string(), source_);
        register_output(get_parameter("output").as_string(), value_);

        const auto variant = get_parameter("filter").as_string();
        if (variant == "median") {
            filter_ = std::make_unique<MedianFilter<double>>(
                static_cast<size_t>(get_parameter("window_size").as_int()));
        } else if (variant == "low-pass") {
            filter_ = std::make_unique<LowPassFilter<double>>(get_parameter("alpha").as_double());
        } else {
            throw std::runtime_error("Unknown filter variant, got " + variant);
        }
    }

    void update() override {
        filter_->update(*source_);
        *value_ = filter_->get();
    }

private:
    std::unique_ptr<rmcs_core::custom_filter::Filter<double>> filter_;

    InputInterface<double> source_;

    OutputInterface<double> value_;
};

} // namespace rmcs_core::custom_filter::filter

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::custom_filter::filter::Filter, rmcs_executor::Component)
