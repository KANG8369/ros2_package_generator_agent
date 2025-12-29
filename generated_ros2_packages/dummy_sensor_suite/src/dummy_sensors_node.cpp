#include "dummy_sensor_suite/dummy_sensors_node.hpp"

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace dummy_sensor_suite
{

DummySensorsNode::DummySensorsNode(const rclcpp::NodeOptions & options)
: Node("dummy_sensors", options)
{
  // Seed RNG with non-deterministic random device
  std::random_device rd;
  rng_ = std::mt19937(rd());

  // Declare parameters with sensible defaults
  // IMU
  imu_enable_ = this->declare_parameter<bool>("imu.enable", true);
  imu_frame_id_ = this->declare_parameter<std::string>("imu.frame_id", "imu_link");
  imu_rate_hz_ = this->declare_parameter<double>("imu.rate_hz", 50.0);
  imu_accel_mean_ = this->declare_parameter<double>("imu.accel_mean", 0.0);
  imu_accel_stddev_ = this->declare_parameter<double>("imu.accel_stddev", 0.02);
  imu_gyro_mean_ = this->declare_parameter<double>("imu.gyro_mean", 0.0);
  imu_gyro_stddev_ = this->declare_parameter<double>("imu.gyro_stddev", 0.01);

  // Laser scan
  laser_enable_ = this->declare_parameter<bool>("laser.enable", true);
  laser_frame_id_ = this->declare_parameter<std::string>("laser.frame_id", "laser_frame");
  laser_rate_hz_ = this->declare_parameter<double>("laser.rate_hz", 10.0);
  laser_angle_min_ = this->declare_parameter<double>("laser.angle_min", -1.57);
  laser_angle_max_ = this->declare_parameter<double>("laser.angle_max", 1.57);
  laser_angle_increment_ = this->declare_parameter<double>("laser.angle_increment", 0.01);
  laser_range_min_ = this->declare_parameter<double>("laser.range_min", 0.1);
  laser_range_max_ = this->declare_parameter<double>("laser.range_max", 10.0);
  laser_range_mean_ = this->declare_parameter<double>("laser.range_mean", 5.0);
  laser_range_stddev_ = this->declare_parameter<double>("laser.range_stddev", 0.1);

  // Range sensor
  range_enable_ = this->declare_parameter<bool>("range.enable", true);
  range_frame_id_ = this->declare_parameter<std::string>("range.frame_id", "range_sensor");
  range_rate_hz_ = this->declare_parameter<double>("range.rate_hz", 20.0);
  range_min_ = this->declare_parameter<double>("range.min", 0.1);
  range_max_ = this->declare_parameter<double>("range.max", 4.0);
  range_mean_ = this->declare_parameter<double>("range.mean", 2.0);
  range_stddev_ = this->declare_parameter<double>("range.stddev", 0.05);

  // Temperature
  temperature_enable_ = this->declare_parameter<bool>("temperature.enable", true);
  temperature_frame_id_ = this->declare_parameter<std::string>("temperature.frame_id", "temp_sensor");
  temperature_rate_hz_ = this->declare_parameter<double>("temperature.rate_hz", 1.0);
  temperature_mean_ = this->declare_parameter<double>("temperature.mean", 293.15);  // ~20 C in K
  temperature_stddev_ = this->declare_parameter<double>("temperature.stddev", 0.5);

  // Image
  image_enable_ = this->declare_parameter<bool>("image.enable", true);
  image_frame_id_ = this->declare_parameter<std::string>("image.frame_id", "camera_link");
  image_rate_hz_ = this->declare_parameter<double>("image.rate_hz", 5.0);
  image_width_ = this->declare_parameter<int>("image.width", 320);
  image_height_ = this->declare_parameter<int>("image.height", 240);
  image_encoding_ = this->declare_parameter<std::string>("image.encoding", "mono8");
  image_base_intensity_ = static_cast<uint8_t>(
    this->declare_parameter<int>("image.base_intensity", 128));
  image_noise_stddev_ = this->declare_parameter<double>("image.noise_stddev", 10.0);

  // Publishers and timers
  if (imu_enable_) {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/dummy/imu", 10);
    if (imu_rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / imu_rate_hz_);
      imu_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&DummySensorsNode::imu_timer_callback, this));
    }
  }

  if (laser_enable_) {
    laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/dummy/scan", 10);
    if (laser_rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / laser_rate_hz_);
      laser_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&DummySensorsNode::laser_timer_callback, this));
    }
  }

  if (range_enable_) {
    range_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/dummy/range", 10);
    if (range_rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / range_rate_hz_);
      range_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&DummySensorsNode::range_timer_callback, this));
    }
  }

  if (temperature_enable_) {
    temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/dummy/temperature", 10);
    if (temperature_rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / temperature_rate_hz_);
      temperature_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&DummySensorsNode::temperature_timer_callback, this));
    }
  }

  if (image_enable_) {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/dummy/image", 10);
    if (image_rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / image_rate_hz_);
      image_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&DummySensorsNode::image_timer_callback, this));
    }
  }

  RCLCPP_INFO(this->get_logger(), "DummySensorsNode initialized.");
}

// Utility: Gaussian noise using Box-Muller transform over mt19937

double DummySensorsNode::gaussian_noise(double mean, double stddev)
{
  if (stddev <= 0.0) {
    return mean;
  }
  std::normal_distribution<double> dist(mean, stddev);
  return dist(rng_);
}

rclcpp::Time DummySensorsNode::now_time()
{
  return this->get_clock()->now();
}

// IMU callback

void DummySensorsNode::imu_timer_callback()
{
  auto msg = sensor_msgs::msg::Imu();
  msg.header.stamp = now_time();
  msg.header.frame_id = imu_frame_id_;

  // Simple model: zero orientation, noisy accel and gyro
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 1.0;

  // Simulate stationary sensor with gravity on Z axis and some noise
  msg.linear_acceleration.x = gaussian_noise(imu_accel_mean_, imu_accel_stddev_);
  msg.linear_acceleration.y = gaussian_noise(imu_accel_mean_, imu_accel_stddev_);
  msg.linear_acceleration.z = 9.81 + gaussian_noise(imu_accel_mean_, imu_accel_stddev_);

  msg.angular_velocity.x = gaussian_noise(imu_gyro_mean_, imu_gyro_stddev_);
  msg.angular_velocity.y = gaussian_noise(imu_gyro_mean_, imu_gyro_stddev_);
  msg.angular_velocity.z = gaussian_noise(imu_gyro_mean_, imu_gyro_stddev_);

  // Simple diagonal covariance
  for (size_t i = 0; i < 9; ++i) {
    msg.orientation_covariance[i] = 0.0;
    msg.angular_velocity_covariance[i] = 0.0;
    msg.linear_acceleration_covariance[i] = 0.0;
  }
  msg.angular_velocity_covariance[0] = imu_gyro_stddev_ * imu_gyro_stddev_;
  msg.angular_velocity_covariance[4] = imu_gyro_stddev_ * imu_gyro_stddev_;
  msg.angular_velocity_covariance[8] = imu_gyro_stddev_ * imu_gyro_stddev_;
  msg.linear_acceleration_covariance[0] = imu_accel_stddev_ * imu_accel_stddev_;
  msg.linear_acceleration_covariance[4] = imu_accel_stddev_ * imu_accel_stddev_;
  msg.linear_acceleration_covariance[8] = imu_accel_stddev_ * imu_accel_stddev_;

  imu_pub_->publish(msg);
}

// Laser scan callback

void DummySensorsNode::laser_timer_callback()
{
  auto msg = sensor_msgs::msg::LaserScan();
  msg.header.stamp = now_time();
  msg.header.frame_id = laser_frame_id_;

  msg.angle_min = static_cast<float>(laser_angle_min_);
  msg.angle_max = static_cast<float>(laser_angle_max_);
  msg.angle_increment = static_cast<float>(laser_angle_increment_);
  msg.range_min = static_cast<float>(laser_range_min_);
  msg.range_max = static_cast<float>(laser_range_max_);

  double fov = laser_angle_max_ - laser_angle_min_;
  size_t num_readings = 0;
  if (laser_angle_increment_ > 0.0 && fov > 0.0) {
    num_readings = static_cast<size_t>(std::floor(fov / laser_angle_increment_)) + 1;
  }
  msg.time_increment = 0.0f;
  msg.scan_time = static_cast<float>((laser_rate_hz_ > 0.0) ? (1.0 / laser_rate_hz_) : 0.0);

  msg.ranges.resize(num_readings);
  msg.intensities.resize(num_readings);

  for (size_t i = 0; i < num_readings; ++i) {
    double noisy = gaussian_noise(laser_range_mean_, laser_range_stddev_);
    if (noisy < laser_range_min_) noisy = laser_range_min_;
    if (noisy > laser_range_max_) noisy = laser_range_max_;
    msg.ranges[i] = static_cast<float>(noisy);
    // Simple intensity pattern: center beams higher intensity
    double angle = laser_angle_min_ + static_cast<double>(i) * laser_angle_increment_;
    double center_factor = std::cos(angle);
    double intensity = 100.0 * std::max(0.0, center_factor);
    msg.intensities[i] = static_cast<float>(intensity);
  }

  laser_pub_->publish(msg);
}

// Range callback

void DummySensorsNode::range_timer_callback()
{
  auto msg = sensor_msgs::msg::Range();
  msg.header.stamp = now_time();
  msg.header.frame_id = range_frame_id_;
  msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  msg.field_of_view = 0.5f;  // rad
  msg.min_range = static_cast<float>(range_min_);
  msg.max_range = static_cast<float>(range_max_);

  double noisy = gaussian_noise(range_mean_, range_stddev_);
  if (noisy < range_min_) noisy = range_min_;
  if (noisy > range_max_) noisy = range_max_;
  msg.range = static_cast<float>(noisy);

  range_pub_->publish(msg);
}

// Temperature callback

void DummySensorsNode::temperature_timer_callback()
{
  auto msg = sensor_msgs::msg::Temperature();
  msg.header.stamp = now_time();
  msg.header.frame_id = temperature_frame_id_;

  msg.temperature = gaussian_noise(temperature_mean_, temperature_stddev_);
  msg.variance = static_cast<double>(temperature_stddev_ * temperature_stddev_);

  temperature_pub_->publish(msg);
}

// Image callback

void DummySensorsNode::image_timer_callback()
{
  auto msg = sensor_msgs::msg::Image();
  msg.header.stamp = now_time();
  msg.header.frame_id = image_frame_id_;

  msg.width = static_cast<uint32_t>(image_width_);
  msg.height = static_cast<uint32_t>(image_height_);
  msg.encoding = image_encoding_;
  msg.is_bigendian = 0;

  uint32_t channels = 1;
  if (image_encoding_ == "rgb8" || image_encoding_ == "bgr8") {
    channels = 3;
  } else if (image_encoding_ == "rgba8" || image_encoding_ == "bgra8") {
    channels = 4;
  }

  msg.step = msg.width * channels;
  size_t size = static_cast<size_t>(msg.height) * msg.step;
  msg.data.resize(size);

  std::normal_distribution<double> dist(0.0, image_noise_stddev_);

  for (size_t y = 0; y < msg.height; ++y) {
    for (size_t x = 0; x < msg.width; ++x) {
      uint8_t base = image_base_intensity_;
      // Add simple gradient pattern horizontally
      double gradient = static_cast<double>(x) / static_cast<double>(msg.width);
      double value = static_cast<double>(base) + 50.0 * (gradient - 0.5) + dist(rng_);
      if (value < 0.0) value = 0.0;
      if (value > 255.0) value = 255.0;
      uint8_t v = static_cast<uint8_t>(value);

      size_t idx = y * msg.step + x * channels;
      for (uint32_t c = 0; c < channels; ++c) {
        msg.data[idx + c] = v;
      }
    }
  }

  image_pub_->publish(msg);
}

}  // namespace dummy_sensor_suite

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dummy_sensor_suite::DummySensorsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
