#include <rokubimini/configuration/Configuration.hpp>

namespace rokubimini
{
namespace configuration
{
void Configuration::load(const std::string& rokubiminiName, const NodeHandlePtr& nh)
{
  // blabla load the shit from file
  // Clear the configuration first.
  *this = Configuration();

  {
    // lock the mutex_ for accessing all the internal variables.
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::string key_string;
    key_string = "set_reading_to_nan_on_disconnect";
    bool set_reading_to_nan_on_disconnect;
    nh->declare_parameter<bool>(key_string, true);
    if (nh->get_parameter(key_string, set_reading_to_nan_on_disconnect))
    {
      setSetReadingToNanOnDisconnect(set_reading_to_nan_on_disconnect);
      hasSetReadingToNanOnDisconnect_ = true;
    }
    else
    {
      hasSetReadingToNanOnDisconnect_ = false;
    }
    key_string = "imu_acceleration_range";
    int imu_acceleration_range;
    nh->declare_parameter<int>(key_string, 0);
    if (nh->get_parameter(key_string, imu_acceleration_range))
    {
      setImuAccelerationRange(static_cast<unsigned int>(imu_acceleration_range));
      hasImuAccelerationRange_ = true;
    }
    else
    {
      hasImuAccelerationRange_ = false;
    }
    key_string = "imu_angular_rate_range";
    int imu_angular_rate_range;
    nh->declare_parameter<int>(key_string, 0);
    if (nh->get_parameter(key_string, imu_angular_rate_range))
    {
      setImuAngularRateRange(static_cast<unsigned int>(imu_angular_rate_range));
      hasImuAngularRateRange_ = true;
    }
    else
    {
      hasImuAngularRateRange_ = false;
    }
    key_string = "force_torque_filter";
    ForceTorqueFilter filter;
    if (filter.load(key_string, nh))
    {
      setForceTorqueFilter(filter);
      hasForceTorqueFilter_ = true;
    }
    else
    {
      hasForceTorqueFilter_ = false;
    }
    key_string = "imu_acceleration_filter";
    int imu_acceleration_filter;
    nh->declare_parameter<int>(key_string, 0);
    if (nh->get_parameter(key_string, imu_acceleration_filter))
    {
      setImuAccelerationFilter(static_cast<unsigned int>(imu_acceleration_filter));
      hasImuAccelerationFilter_ = true;
    }
    else
    {
      hasImuAccelerationFilter_ = false;
    }

    key_string = "imu_angular_rate_filter";
    int imu_angular_rate_filter;
    nh->declare_parameter<int>(key_string, 0);
    if (nh->get_parameter(key_string, imu_angular_rate_filter))
    {
      setImuAngularRateFilter(static_cast<unsigned int>(imu_angular_rate_filter));
      hasImuAngularRateFilter_ = true;
    }
    else
    {
      hasImuAngularRateFilter_ = false;
    }

    key_string = "force_torque_offset";
    Eigen::Matrix<double, 6, 1> offset;
    bool success = true;
    std::string force_torque_key;

    force_torque_key = key_string + "/Fx";
    nh->declare_parameter<double>(force_torque_key, 0.0);
    if (nh->get_parameter(force_torque_key, offset(0, 0)))
    {
      success = success && true;
    }
    force_torque_key = key_string + "/Fy";
    nh->declare_parameter<double>(force_torque_key, 0.0);
    if (nh->get_parameter(force_torque_key, offset(1, 0)))
    {
      success = success && true;
    }
    force_torque_key = key_string + "/Fz";
    nh->declare_parameter<double>(force_torque_key, 0.0);
    if (nh->get_parameter(force_torque_key, offset(2, 0)))
    {
      success = success && true;
    }
    force_torque_key = key_string + "/Tx";
    nh->declare_parameter<double>(force_torque_key, 0.0);
    if (nh->get_parameter(force_torque_key, offset(3, 0)))
    {
      success = success && true;
    }
    force_torque_key = key_string + "/Ty";
    nh->declare_parameter<double>(force_torque_key, 0.0);
    if (nh->get_parameter(force_torque_key, offset(4, 0)))
    {
      success = success && true;
    }
    force_torque_key = key_string + "/Tz";
    nh->declare_parameter<double>(force_torque_key, 0.0);
    if (nh->get_parameter(force_torque_key, offset(5, 0)))
    {
      success = success && true;
    }
    if (success)
    {
      setForceTorqueOffset(offset);
      hasForceTorqueOffset_ = true;
    }
    else
    {
      hasForceTorqueOffset_ = false;
    }

    key_string = "sensor_configuration";
    SensorConfiguration configuration;
    if (configuration.load(key_string, nh))
    {
      setSensorConfiguration(configuration);
      hasSensorConfiguration_ = true;
    }
    else
    {
      hasSensorConfiguration_ = false;
    }

    key_string = "use_custom_calibration";
    bool use_custom_calibration;
    nh->declare_parameter<bool>(key_string, false);
    if (nh->get_parameter(key_string, use_custom_calibration))
    {
      setUseCustomCalibration(use_custom_calibration);
      hasUseCustomCalibration_ = true;
    }
    else
    {
      hasUseCustomCalibration_ = false;
    }
    key_string = "sensor_calibration";
    calibration::SensorCalibration calibration;
    if (calibration.load(key_string, nh))
    {
      setSensorCalibration(calibration);
      hasSensorCalibration_ = true;
    }
    else
    {
      hasSensorCalibration_ = false;
    }
    key_string = "save_configuration";
    bool save_configuration;
    nh->declare_parameter<bool>(key_string, false);
    if (nh->get_parameter(key_string, save_configuration))
    {
      setSaveConfiguration(save_configuration);
      hasSaveConfiguration_ = true;
    }
    else
    {
      hasSaveConfiguration_ = false;
    }
  }
}

void Configuration::printConfiguration() const
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("Configuration"),
                     "setReadingToNanOnDisconnect_: " << setReadingToNanOnDisconnect_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("Configuration"), "useCustomCalibration_: " << useCustomCalibration_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("Configuration"), "saveConfiguration_: " << saveConfiguration_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("Configuration"), "imuAccelerationRange_: " << imuAccelerationRange_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("Configuration"), "imuAngularRateRange_: " << imuAngularRateRange_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("Configuration"), "imuAccelerationFilter_: " << imuAccelerationFilter_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("Configuration"), "imuAngularRateFilter_: " << imuAngularRateFilter_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("Configuration"), "forceTorqueOffset_:\n" << forceTorqueOffset_);
  forceTorqueFilter_.print();
  sensorConfiguration_.print();
}

Configuration& Configuration::operator=(const Configuration& other)
{
  setReadingToNanOnDisconnect_ = other.getSetReadingToNanOnDisconnect();
  sensorConfiguration_ = other.getSensorConfiguration();
  forceTorqueFilter_ = other.getForceTorqueFilter();
  forceTorqueOffset_ = other.getForceTorqueOffset();
  useCustomCalibration_ = other.getUseCustomCalibration();
  saveConfiguration_ = other.getSaveConfiguration();
  sensorCalibration_ = other.getSensorCalibration();
  imuAccelerationRange_ = other.imuAccelerationRange_;
  imuAngularRateRange_ = other.imuAngularRateRange_;
  imuAccelerationFilter_ = other.imuAccelerationFilter_;
  imuAngularRateFilter_ = other.imuAngularRateFilter_;
  hasSetReadingToNanOnDisconnect_ = other.hasSetReadingToNanOnDisconnect();
  hasSensorConfiguration_ = other.hasSensorConfiguration();
  hasForceTorqueFilter_ = other.hasForceTorqueFilter();
  hasForceTorqueOffset_ = other.hasForceTorqueOffset();
  hasUseCustomCalibration_ = other.hasUseCustomCalibration();
  hasSaveConfiguration_ = other.hasSaveConfiguration();
  hasSensorCalibration_ = other.hasSensorCalibration();
  hasImuAccelerationRange_ = other.hasImuAccelerationRange();
  hasImuAngularRateRange_ = other.hasImuAngularRateRange();
  hasImuAccelerationFilter_ = other.hasImuAccelerationFilter();
  hasImuAngularRateFilter_ = other.hasImuAngularRateFilter();
  return *this;
}

void Configuration::setForceTorqueFilter(const ForceTorqueFilter& forceTorqueFilter)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  forceTorqueFilter_ = forceTorqueFilter;
}

const ForceTorqueFilter& Configuration::getForceTorqueFilter() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return forceTorqueFilter_;
}

void Configuration::setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  forceTorqueOffset_ = forceTorqueOffset;
}

const Eigen::Matrix<double, 6, 1>& Configuration::getForceTorqueOffset() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return forceTorqueOffset_;
}

void Configuration::setUseCustomCalibration(const bool useCustomCalibration)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  useCustomCalibration_ = useCustomCalibration;
}

bool Configuration::getUseCustomCalibration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return useCustomCalibration_;
}

void Configuration::setSaveConfiguration(const bool saveConfiguration)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  saveConfiguration_ = saveConfiguration;
}

bool Configuration::getSaveConfiguration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return saveConfiguration_;
}

void Configuration::setSetReadingToNanOnDisconnect(const bool setReadingToNanOnDisconnect)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  setReadingToNanOnDisconnect_ = setReadingToNanOnDisconnect;
}

bool Configuration::getSetReadingToNanOnDisconnect() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return setReadingToNanOnDisconnect_;
}

void Configuration::setSensorConfiguration(const SensorConfiguration& sensorConfiguration)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  sensorConfiguration_ = sensorConfiguration;
}

const SensorConfiguration& Configuration::getSensorConfiguration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return sensorConfiguration_;
}

void Configuration::setSensorCalibration(const calibration::SensorCalibration& sensorCalibration)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  sensorCalibration_ = sensorCalibration;
}

const calibration::SensorCalibration& Configuration::getSensorCalibration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return sensorCalibration_;
}

void Configuration::setImuAccelerationFilter(const unsigned int imuAccelerationFilter)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuAccelerationFilter_ = imuAccelerationFilter;
}

unsigned int Configuration::getImuAccelerationFilter() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuAccelerationFilter_;
}

void Configuration::setImuAngularRateFilter(const unsigned int imuAngularRateFilter)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuAngularRateFilter_ = imuAngularRateFilter;
}

unsigned int Configuration::getImuAngularRateFilter() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuAngularRateFilter_;
}

void Configuration::setImuAccelerationRange(const uint8_t imuAccelerationRange)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuAccelerationRange_ = imuAccelerationRange;
}

uint8_t Configuration::getImuAccelerationRange() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuAccelerationRange_;
}

void Configuration::setImuAngularRateRange(const uint8_t imuAngularRateRange)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuAngularRateRange_ = imuAngularRateRange;
}

uint8_t Configuration::getImuAngularRateRange() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuAngularRateRange_;
}

bool Configuration::hasImuAngularRateRange() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasImuAngularRateRange_;
}

bool Configuration::hasSetReadingToNanOnDisconnect() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasSetReadingToNanOnDisconnect_;
}

bool Configuration::hasSensorConfiguration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasSensorConfiguration_;
}

bool Configuration::hasForceTorqueFilter() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasForceTorqueFilter_;
}

bool Configuration::hasForceTorqueOffset() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasForceTorqueOffset_;
}

bool Configuration::hasUseCustomCalibration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasUseCustomCalibration_;
}

bool Configuration::hasSaveConfiguration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasSaveConfiguration_;
}

bool Configuration::hasSensorCalibration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasSensorCalibration_;
}

bool Configuration::hasImuAccelerationRange() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasImuAccelerationRange_;
}

bool Configuration::hasImuAccelerationFilter() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasImuAccelerationFilter_;
}

bool Configuration::hasImuAngularRateFilter() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasImuAngularRateFilter_;
}
}  // namespace configuration
}  // namespace rokubimini
