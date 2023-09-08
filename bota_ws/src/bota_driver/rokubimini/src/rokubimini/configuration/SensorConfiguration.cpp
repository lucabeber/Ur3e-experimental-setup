#include <rokubimini/configuration/SensorConfiguration.hpp>

namespace rokubimini
{
namespace configuration
{
SensorConfiguration::SensorConfiguration(const uint8_t calibrationMatrixActive,
                                         const uint8_t temperatureCompensationActive, const uint8_t imuActive,
                                         const uint8_t coordinateSystemConfigurationActive,
                                         const uint8_t inertiaCompensationActive,
                                         const uint8_t orientationEstimationActive)
  : calibrationMatrixActive_(calibrationMatrixActive)
  , temperatureCompensationActive_(temperatureCompensationActive)
  , imuActive_(imuActive)
  , coordinateSystemConfigurationActive_(coordinateSystemConfigurationActive)
  , inertiaCompensationActive_(inertiaCompensationActive)
  , orientationEstimationActive_(orientationEstimationActive)
{
}

bool SensorConfiguration::load(const std::string& key, const NodeHandlePtr& nh)
{
  bool success = true;
  std::string local_key;
  local_key = key + "/calibration_matrix_active";
  bool calibration_matrix_active;
  nh->declare_parameter<bool>(local_key, true);
  if (nh->get_parameter(local_key, calibration_matrix_active))
  {
    calibrationMatrixActive_ = static_cast<uint8_t>(calibration_matrix_active);
    success = success && true;
  }
  local_key = key + "/temperature_compensation_active";
  bool temperature_compensation_active;
  nh->declare_parameter<bool>(local_key, false);
  if (nh->get_parameter(local_key, temperature_compensation_active))
  {
    temperatureCompensationActive_ = static_cast<uint8_t>(temperature_compensation_active);
    success = success && true;
  }
  local_key = key + "/imu_active";
  int imu_active;
  nh->declare_parameter<int>(local_key, false);
  if (nh->get_parameter(local_key, imu_active))
  {
    imuActive_ = static_cast<uint8_t>(imu_active);
    success = success && true;
  }
  local_key = key + "/coordinate_system_active";
  bool coordinate_system_active;
  nh->declare_parameter<bool>(local_key, false);
  if (nh->get_parameter(local_key, coordinate_system_active))
  {
    coordinateSystemConfigurationActive_ = static_cast<uint8_t>(coordinate_system_active);
    success = success && true;
  }
  local_key = key + "/inertia_compensation_active";
  int inertia_compensation_active;
  nh->declare_parameter<int>(local_key, false);
  if (nh->get_parameter(local_key, inertia_compensation_active))
  {
    inertiaCompensationActive_ = static_cast<uint8_t>(inertia_compensation_active);
    success = success && true;
  }
  local_key = key + "/orientation_estimation_active";
  int orientation_estimation_active;
  nh->declare_parameter<int>(local_key, false);
  if (nh->get_parameter(local_key, orientation_estimation_active))
  {
    orientationEstimationActive_ = static_cast<uint8_t>(orientation_estimation_active);
    success = success && true;
  }
  return success;
}

void SensorConfiguration::print() const
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("SensorConfiguration"),
                     "calibrationMatrixActive_: " << static_cast<unsigned int>(calibrationMatrixActive_));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("SensorConfiguration"),
                     "temperatureCompensationActive_: " << static_cast<unsigned int>(temperatureCompensationActive_));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("SensorConfiguration"),
                     "imuActive_: " << static_cast<unsigned int>(imuActive_));
  RCLCPP_INFO_STREAM(
      rclcpp::get_logger("SensorConfiguration"),
      "coordinateSystemConfigurationActive_: " << static_cast<unsigned int>(coordinateSystemConfigurationActive_));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("SensorConfiguration"),
                     "inertiaCompensationActive_: " << static_cast<unsigned int>(inertiaCompensationActive_));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("SensorConfiguration"),
                     "orientationEstimationActive_: " << static_cast<unsigned int>(orientationEstimationActive_));
}

}  // namespace configuration
}  // namespace rokubimini
