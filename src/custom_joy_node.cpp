#include "custom_joy_node/custom_joy_node.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>

#ifdef __linux__
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

CustomJoyNode::CustomJoyNode()
: Node("custom_joy_node")
{
  topic_name_ = declare_parameter<std::string>("topic_name", "joy");
  device_path_ = declare_parameter<std::string>("device_path", "");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
  scan_period_ms_ = declare_parameter<int>("scan_period_ms", 1000);
  default_axes_count_ = declare_parameter<int>("default_axes_count", 8);
  default_buttons_count_ = declare_parameter<int>("default_buttons_count", 16);
  deadzone_ = declare_parameter<double>("deadzone", 0.0);
  prefer_evdev_ = declare_parameter<bool>("prefer_evdev", true);

  default_axes_count_ = std::max(default_axes_count_, 0);
  default_buttons_count_ = std::max(default_buttons_count_, 0);
  publish_rate_hz_ = std::max(publish_rate_hz_, 1.0);
  scan_period_ms_ = std::max(scan_period_ms_, 100);
  deadzone_ = std::clamp(deadzone_, 0.0, 1.0);

  axes_.assign(static_cast<size_t>(default_axes_count_), 0.0F);
  buttons_.assign(static_cast<size_t>(default_buttons_count_), 0);

  joy_pub_ = create_publisher<sensor_msgs::msg::Joy>(topic_name_, 10);

  const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
    std::bind(&CustomJoyNode::publish_joy, this));
  scan_timer_ = create_wall_timer(
    std::chrono::milliseconds(scan_period_ms_),
    std::bind(&CustomJoyNode::try_connect, this));

  try_connect();
  RCLCPP_INFO(get_logger(), "custom_joy_node started. Publishing '%s'.", topic_name_.c_str());
}

CustomJoyNode::~CustomJoyNode()
{
  close_device();
}

#ifdef __linux__
bool CustomJoyNode::test_bit(const unsigned long * bits, int bit)
{
  constexpr int bits_per_word = static_cast<int>(sizeof(unsigned long) * 8);
  return (bits[bit / bits_per_word] & (1UL << (bit % bits_per_word))) != 0;
}
#endif

void CustomJoyNode::publish_joy()
{
  read_device_events();

  sensor_msgs::msg::Joy msg;
  msg.header.stamp = now();
  msg.header.frame_id = "joy";
  msg.axes = axes_;
  msg.buttons = buttons_;
  joy_pub_->publish(msg);
}

void CustomJoyNode::zero_outputs()
{
  std::fill(axes_.begin(), axes_.end(), 0.0F);
  std::fill(buttons_.begin(), buttons_.end(), 0);
}

void CustomJoyNode::try_connect()
{
  if (connected_) {
    return;
  }

#ifdef __linux__
  if (!device_path_.empty()) {
    if (device_path_.find("/event") != std::string::npos) {
      open_evdev_device(device_path_);
    } else if (device_path_.find("/js") != std::string::npos) {
      open_joystick_device(device_path_);
    } else if (prefer_evdev_) {
      open_evdev_device(device_path_) || open_joystick_device(device_path_);
    } else {
      open_joystick_device(device_path_) || open_evdev_device(device_path_);
    }
    log_missing_once();
    return;
  }

  if (prefer_evdev_ && scan_evdev_devices()) {
    return;
  }
  if (scan_joystick_devices()) {
    return;
  }
  if (!prefer_evdev_ && scan_evdev_devices()) {
    return;
  }

  log_missing_once();
#else
  if (!missing_logged_) {
    RCLCPP_WARN(
      get_logger(),
      "Linux input devices are not available on this platform. Publishing zero Joy messages.");
    missing_logged_ = true;
  }
#endif
}

void CustomJoyNode::log_missing_once()
{
  if (!connected_ && !missing_logged_) {
    RCLCPP_WARN(
      get_logger(),
      "No joystick controller found. Publishing zero Joy messages until one is connected.");
    missing_logged_ = true;
  }
}

#ifdef __linux__
bool CustomJoyNode::scan_evdev_devices()
{
  for (int i = 0; i < 64; ++i) {
    const std::string candidate = "/dev/input/event" + std::to_string(i);
    if (open_evdev_device(candidate)) {
      return true;
    }
  }
  return false;
}

bool CustomJoyNode::scan_joystick_devices()
{
  for (int i = 0; i < 32; ++i) {
    const std::string candidate = "/dev/input/js" + std::to_string(i);
    if (open_joystick_device(candidate)) {
      return true;
    }
  }
  return false;
}

bool CustomJoyNode::open_evdev_device(const std::string & path)
{
  const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd < 0) {
    return false;
  }

  if (!configure_evdev_device(fd)) {
    close(fd);
    return false;
  }

  close_device();
  fd_ = fd;
  backend_ = Backend::evdev;
  active_device_path_ = path;
  connected_ = true;
  missing_logged_ = false;

  char name[256] = "unknown";
  if (ioctl(fd_, EVIOCGNAME(sizeof(name)), name) < 0) {
    std::strncpy(name, "unknown", sizeof(name));
    name[sizeof(name) - 1] = '\0';
  }

  RCLCPP_INFO(
    get_logger(),
    "Joystick connected via evdev: %s (%s), axes=%zu buttons=%zu",
    name,
    active_device_path_.c_str(),
    evdev_axes_.size(),
    evdev_button_codes_.size());
  return true;
}

bool CustomJoyNode::configure_evdev_device(int fd)
{
  std::array<unsigned long, (EV_MAX + 64) / 64> event_bits{};
  std::array<unsigned long, (ABS_MAX + 64) / 64> abs_bits{};
  std::array<unsigned long, (KEY_MAX + 64) / 64> key_bits{};

  if (ioctl(fd, EVIOCGBIT(0, sizeof(event_bits)), event_bits.data()) < 0) {
    return false;
  }
  if (!test_bit(event_bits.data(), EV_ABS) || !test_bit(event_bits.data(), EV_KEY)) {
    return false;
  }
  if (ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits.data()) < 0 ||
    ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(key_bits)), key_bits.data()) < 0)
  {
    return false;
  }

  std::vector<EvdevAxis> axes;
  for (const int code : preferred_axis_codes()) {
    if (!test_bit(abs_bits.data(), code)) {
      continue;
    }

    input_absinfo info{};
    if (ioctl(fd, EVIOCGABS(code), &info) < 0 || info.maximum <= info.minimum) {
      continue;
    }
    axes.push_back({code, info.minimum, info.maximum, info.flat});
  }

  std::vector<int> buttons;
  for (const int code : preferred_button_codes()) {
    if (test_bit(key_bits.data(), code)) {
      buttons.push_back(code);
    }
  }

  if (axes.empty() || buttons.empty()) {
    return false;
  }

  evdev_axes_ = axes;
  evdev_button_codes_ = buttons;
  axes_.assign(std::max(default_axes_count_, static_cast<int>(evdev_axes_.size())), 0.0F);
  buttons_.assign(std::max(default_buttons_count_, static_cast<int>(evdev_button_codes_.size())), 0);

  for (size_t i = 0; i < evdev_axes_.size() && i < axes_.size(); ++i) {
    input_absinfo info{};
    if (ioctl(fd, EVIOCGABS(evdev_axes_[i].code), &info) == 0) {
      axes_[i] = normalize_evdev_axis(evdev_axes_[i], info.value);
    }
  }

  return true;
}

bool CustomJoyNode::open_joystick_device(const std::string & path)
{
  const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd < 0) {
    return false;
  }

  uint8_t axes_count = 0;
  uint8_t buttons_count = 0;
  char name[128] = "unknown";

  if (ioctl(fd, JSIOCGAXES, &axes_count) < 0 ||
    ioctl(fd, JSIOCGBUTTONS, &buttons_count) < 0)
  {
    close(fd);
    return false;
  }

  if (ioctl(fd, JSIOCGNAME(sizeof(name)), name) < 0) {
    std::strncpy(name, "unknown", sizeof(name));
    name[sizeof(name) - 1] = '\0';
  }

  close_device();
  fd_ = fd;
  backend_ = Backend::joystick;
  active_device_path_ = path;
  connected_ = true;
  missing_logged_ = false;
  axes_.assign(std::max<int>(axes_count, default_axes_count_), 0.0F);
  buttons_.assign(std::max<int>(buttons_count, default_buttons_count_), 0);

  RCLCPP_INFO(
    get_logger(),
    "Joystick connected via js: %s (%s), axes=%u buttons=%u",
    name,
    active_device_path_.c_str(),
    axes_count,
    buttons_count);
  return true;
}

void CustomJoyNode::read_device_events()
{
  if (!connected_) {
    return;
  }

  if (backend_ == Backend::evdev) {
    read_evdev_events();
  } else if (backend_ == Backend::joystick) {
    read_joystick_events();
  }
}

void CustomJoyNode::read_evdev_events()
{
  input_event event;
  while (true) {
    const ssize_t bytes = read(fd_, &event, sizeof(event));
    if (bytes == static_cast<ssize_t>(sizeof(event))) {
      handle_evdev_event(event);
      continue;
    }
    if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return;
    }
    handle_disconnect();
    return;
  }
}

void CustomJoyNode::read_joystick_events()
{
  js_event event;
  while (true) {
    const ssize_t bytes = read(fd_, &event, sizeof(event));
    if (bytes == static_cast<ssize_t>(sizeof(event))) {
      handle_joystick_event(event);
      continue;
    }
    if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return;
    }
    handle_disconnect();
    return;
  }
}

void CustomJoyNode::handle_evdev_event(const input_event & event)
{
  if (event.type == EV_ABS) {
    for (size_t i = 0; i < evdev_axes_.size() && i < axes_.size(); ++i) {
      if (evdev_axes_[i].code == event.code) {
        axes_[i] = normalize_evdev_axis(evdev_axes_[i], event.value);
        return;
      }
    }
  } else if (event.type == EV_KEY) {
    for (size_t i = 0; i < evdev_button_codes_.size() && i < buttons_.size(); ++i) {
      if (evdev_button_codes_[i] == event.code) {
        buttons_[i] = event.value ? 1 : 0;
        return;
      }
    }
  }
}

void CustomJoyNode::handle_joystick_event(const js_event & event)
{
  const uint8_t type = event.type & ~JS_EVENT_INIT;
  const size_t number = static_cast<size_t>(event.number);

  if (type == JS_EVENT_AXIS && number < axes_.size()) {
    float value = static_cast<float>(event.value) / 32767.0F;
    value = std::clamp(value, -1.0F, 1.0F);
    if (std::fabs(value) < deadzone_) {
      value = 0.0F;
    }
    axes_[number] = value;
  } else if (type == JS_EVENT_BUTTON && number < buttons_.size()) {
    buttons_[number] = event.value ? 1 : 0;
  }
}

float CustomJoyNode::normalize_evdev_axis(const EvdevAxis & axis, int value) const
{
  if (axis.maximum <= axis.minimum) {
    return 0.0F;
  }

  const double range = static_cast<double>(axis.maximum - axis.minimum);
  double normalized = 0.0;
  if (is_unidirectional_axis(axis.code)) {
    normalized = static_cast<double>(value - axis.minimum) / range;
    normalized = std::clamp(normalized, 0.0, 1.0);
  } else {
    normalized = (2.0 * static_cast<double>(value - axis.minimum) / range) - 1.0;
    normalized = std::clamp(normalized, -1.0, 1.0);
  }

  const double flat = range > 0.0 ? static_cast<double>(axis.flat) / range : 0.0;
  if (std::fabs(normalized) < std::max(deadzone_, flat)) {
    normalized = 0.0;
  }
  return static_cast<float>(normalized);
}

bool CustomJoyNode::is_unidirectional_axis(int code)
{
  switch (code) {
    case ABS_Z:
    case ABS_RZ:
#ifdef ABS_GAS
    case ABS_GAS:
#endif
#ifdef ABS_BRAKE
    case ABS_BRAKE:
#endif
#ifdef ABS_THROTTLE
    case ABS_THROTTLE:
#endif
      return true;
    default:
      return false;
  }
}

void CustomJoyNode::handle_disconnect()
{
  RCLCPP_WARN(
    get_logger(),
    "Joystick disconnected: %s. Publishing zero Joy messages and waiting for reconnection.",
    active_device_path_.c_str());
  close_device();
  zero_outputs();
}

void CustomJoyNode::close_device()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  backend_ = Backend::none;
  connected_ = false;
  active_device_path_.clear();
  evdev_axes_.clear();
  evdev_button_codes_.clear();
}

const std::vector<int> & CustomJoyNode::preferred_axis_codes()
{
  static const std::vector<int> codes = [] {
    std::vector<int> values{
      ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ,
      ABS_HAT0X, ABS_HAT0Y, ABS_HAT1X, ABS_HAT1Y,
      ABS_HAT2X, ABS_HAT2Y, ABS_HAT3X, ABS_HAT3Y};
#ifdef ABS_GAS
    values.push_back(ABS_GAS);
#endif
#ifdef ABS_BRAKE
    values.push_back(ABS_BRAKE);
#endif
    return values;
  }();
  return codes;
}

const std::vector<int> & CustomJoyNode::preferred_button_codes()
{
  static const std::vector<int> codes{
    BTN_SOUTH, BTN_EAST, BTN_NORTH, BTN_WEST,
    BTN_TL, BTN_TR, BTN_TL2, BTN_TR2,
    BTN_SELECT, BTN_START, BTN_MODE,
    BTN_THUMBL, BTN_THUMBR,
    BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_DPAD_LEFT, BTN_DPAD_RIGHT,
    BTN_TRIGGER, BTN_THUMB, BTN_THUMB2, BTN_TOP, BTN_TOP2, BTN_PINKIE, BTN_BASE};
  return codes;
}
#else
void CustomJoyNode::read_device_events() {}

void CustomJoyNode::close_device()
{
  backend_ = Backend::none;
  connected_ = false;
  active_device_path_.clear();
}
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomJoyNode>());
  rclcpp::shutdown();
  return 0;
}
