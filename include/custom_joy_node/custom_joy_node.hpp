#ifndef CUSTOM_JOY_NODE__CUSTOM_JOY_NODE_HPP_
#define CUSTOM_JOY_NODE__CUSTOM_JOY_NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#ifdef __linux__
#include <linux/input.h>
#include <linux/joystick.h>
#endif

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class CustomJoyNode : public rclcpp::Node
{
public:
  CustomJoyNode();
  ~CustomJoyNode() override;

private:
  enum class Backend
  {
    none,
    evdev,
    joystick
  };

#ifdef __linux__
  struct EvdevAxis
  {
    int code;
    int minimum;
    int maximum;
    int flat;
  };

  static bool test_bit(const unsigned long * bits, int bit);
#endif

  void publish_joy();
  void zero_outputs();
  void try_connect();
  void log_missing_once();

#ifdef __linux__
  bool scan_evdev_devices();
  bool scan_joystick_devices();
  bool open_evdev_device(const std::string & path);
  bool configure_evdev_device(int fd);
  bool open_joystick_device(const std::string & path);
  void read_device_events();
  void read_evdev_events();
  void read_joystick_events();
  void handle_evdev_event(const input_event & event);
  void handle_joystick_event(const js_event & event);
  float normalize_evdev_axis(const EvdevAxis & axis, int value) const;
  static bool is_unidirectional_axis(int code);
  void handle_disconnect();
  void close_device();
  static const std::vector<int> & preferred_axis_codes();
  static const std::vector<int> & preferred_button_codes();
#else
  void read_device_events();
  void close_device();
#endif

  std::string device_path_;
  std::string active_device_path_;
  double publish_rate_hz_;
  int scan_period_ms_;
  int default_axes_count_;
  int default_buttons_count_;
  double deadzone_;
  bool prefer_evdev_;
  bool connected_{false};
  bool missing_logged_{false};
  Backend backend_{Backend::none};

#ifdef __linux__
  int fd_{-1};
  std::vector<EvdevAxis> evdev_axes_;
  std::vector<int> evdev_button_codes_;
#endif

  std::vector<float> axes_;
  std::vector<int32_t> buttons_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
};

#endif  // CUSTOM_JOY_NODE__CUSTOM_JOY_NODE_HPP_
