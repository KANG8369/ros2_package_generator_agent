#ifndef DUMMY_SENSOR_SUITE__DUMMY_SENSORS_NODE_HPP_
#define DUMMY_SENSOR_SUITE__DUMMY_SENSORS_NODE_HPP_

#include <random>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dummy_sensor_suite
{

class DummySensorsNode : public rclcpp::Node
{
public:
  explicit DummySensorsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Utility
  double gaussian_noise(double mean, double stddev);

  // Common time handling
  rclcpp::Time now_time();

  // IMU
  void imu_timer_callback();
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  std::string imu_frame_id_;
  double imu_rate_hz_;
  double imu_accel_mean_;
  double imu_accel_stddev_;
  double imu_gyro_mean_;
  double imu_gyro_stddev_;
  bool imu_enable_;

  // Laser scan
  void laser_timer_callback();
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::TimerBase::SharedPtr laser_timer_;
  std::string laser_frame_id_;
  double laser_rate_hz_;
  double laser_angle_min_;
  double laser_angle_max_;
  double laser_angle_increment_;
  double laser_range_min_;
  double laser_range_max_;
  double laser_range_mean_;
  double laser_range_stddev_;
  bool laser_enable_;

  // Range (single beam)
  void range_timer_callback();
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::TimerBase::SharedPtr range_timer_;
  std::string range_frame_id_;
  double range_rate_hz_;
  double range_min_;
  double range_max_;
  double range_mean_;
  double range_stddev_;
  bool range_enable_;

  // Temperature
  void temperature_timer_callback();
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::TimerBase::SharedPtr temperature_timer_;
  std::string temperature_frame_id_;
  double temperature_rate_hz_;
  double temperature_mean_;
  double temperature_stddev_;
  bool temperature_enable_;

  // Dummy image
  void image_timer_callback();
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr image_timer_;
  std::string image_frame_id_;
  double image_rate_hz_;
  int image_width_;
  int image_height_;
  std::string image_encoding_;
  uint8_t image_base_intensity_;
  double image_noise_stddev_;
  bool image_enable_;

  // Random engine for all noise
  std::mt19937 rng_;
};

}  // namespace dummy_sensor_suite

#endif  // DUMMY_SENSOR_SUITE__DUMMY_SENSORS_NODE_HPP_
