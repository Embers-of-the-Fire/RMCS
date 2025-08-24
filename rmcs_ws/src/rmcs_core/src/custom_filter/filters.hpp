#pragma once

#include <algorithm>
#include <concepts>
#include <deque>
#include <stdexcept>
#include <vector>

namespace rmcs_core::custom_filter {

template <typename N>
concept Number = std::integral<N> || std::floating_point<N>;

template <typename D>
requires(Number<D>) class Filter {
public:
    virtual void update(const D& next) = 0;
    virtual D get() const = 0;
    virtual ~Filter() = default;
};

template <typename D = double>
requires(Number<D>) class MedianFilter : public Filter<D> {
public:
    explicit MedianFilter(size_t window_size = 5)
        : window_size_(window_size)
        , buffer_()
        , sorted_buffer_() {
        if (window_size % 2 == 0) {
            throw std::runtime_error("Window size must be an odd number.");
        }

        sorted_buffer_.reserve(window_size);
    }

    void update(const D& next) override {
        buffer_.push_back(next);
        if (buffer_.size() > window_size_) {
            buffer_.pop_front();
        }
    }

    D get() const override { return calculate_median(); }

private:
    const size_t window_size_;

    mutable std::deque<D> buffer_;
    mutable std::vector<D> sorted_buffer_;

    D calculate_median() const {
        if (buffer_.empty())
            return D{};

        sorted_buffer_.clear();
        sorted_buffer_.reserve(buffer_.size());

        for (const auto& val : buffer_) {
            sorted_buffer_.push_back(val);
        }
        size_t n = sorted_buffer_.size();
        std::nth_element(
            sorted_buffer_.begin(), sorted_buffer_.begin() + n / 2, sorted_buffer_.end());
        return sorted_buffer_[n / 2];
    }
};

// RC-low pass
// Cannot understand parameters' meaning of Butterworth filter :sad:
template <typename D = double>
requires(Number<D>) class LowPassFilter : public Filter<D> {
public:
    explicit LowPassFilter(D alpha)
        : last_()
        , alpha_(alpha) {
        if (alpha > static_cast<D>(1) || alpha < static_cast<D>(0)) {
            throw std::runtime_error("Low pass filter's alpha factor must be between 0.0 and 1.0.");
        }
    }

    void update(const D& next) override {
        last_ = current_;
        current_ = next;
    }

    D get() const override { return alpha_ * current_ + (static_cast<D>(1.0) - alpha_) * last_; }

private:
    mutable D last_;
    mutable D current_;

    const D alpha_;
};

} // namespace rmcs_core::custom_filter
