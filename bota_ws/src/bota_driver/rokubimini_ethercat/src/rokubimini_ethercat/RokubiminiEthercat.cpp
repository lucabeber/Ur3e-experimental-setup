#include <rokubimini_ethercat/RokubiminiEthercat.hpp>
#include <csignal>
#include <thread>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace rokubimini
{
namespace ethercat
{
void RokubiminiEthercat::postSetupConfiguration()
{
}
void RokubiminiEthercat::preSetupConfiguration()
{
  slavePtr_->preSetupConfiguration();
}

void RokubiminiEthercat::updateProcessReading()
{
  {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    slavePtr_->getReading(reading_);

    // Update statusword.
    auto statusword(reading_.getStatusword());
    setStatusword(statusword);
    statuswordRequested_ = false;
  }

  if (deviceIsMissing())
  {
    Statusword statusword;
    setStatusword(statusword);
  }
}

void RokubiminiEthercat::shutdownWithCommunication()
{
  slavePtr_->shutdown();
  // publish the last diagnostics before shutdown
  connectionStatusUpdater_->force_update();
}

bool RokubiminiEthercat::deviceIsMissing() const
{
  return false;
}

bool RokubiminiEthercat::getSerialNumber(unsigned int& serialNumber)
{
  return slavePtr_->getSerialNumber(serialNumber);
}

bool RokubiminiEthercat::getForceTorqueSamplingRate(int& samplingRate)
{
  return slavePtr_->getForceTorqueSamplingRate(samplingRate);
}

bool RokubiminiEthercat::setForceTorqueFilter(const configuration::ForceTorqueFilter& filter)
{
  return slavePtr_->setForceTorqueFilter(filter);
}

bool RokubiminiEthercat::setAccelerationFilter(const unsigned int filter)
{
  return slavePtr_->setAccelerationFilter(filter);
}

bool RokubiminiEthercat::setAngularRateFilter(const unsigned int filter)
{
  return slavePtr_->setAngularRateFilter(filter);
}

bool RokubiminiEthercat::setAccelerationRange(const unsigned int range)
{
  return slavePtr_->setAccelerationRange(range);
}

bool RokubiminiEthercat::setAngularRateRange(const unsigned int range)
{
  return slavePtr_->setAngularRateRange(range);
}

bool RokubiminiEthercat::setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset)
{
  return slavePtr_->setForceTorqueOffset(forceTorqueOffset);
}

bool RokubiminiEthercat::setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration)
{
  if (!slavePtr_->setSensorConfiguration(sensorConfiguration))
  {
    return false;
  }
  getConfiguration().setSensorConfiguration(sensorConfiguration);
  return true;
}

bool RokubiminiEthercat::setSensorCalibration(const calibration::SensorCalibration& sensorCalibration)
{
  if (!slavePtr_->setSensorCalibration(sensorCalibration))
  {
    return false;
  }
  getConfiguration().setSensorCalibration(sensorCalibration);
  return true;
}

bool RokubiminiEthercat::setConfigMode()
{
  return slavePtr_->setConfigMode();
}

bool RokubiminiEthercat::setRunMode()
{
  startCount_ = 0;
  return slavePtr_->setRunMode();
}

bool RokubiminiEthercat::saveConfigParameter()
{
  return slavePtr_->saveConfigParameter();
}

using RokubiminiReadingRos = rokubimini_msgs::msg::Reading;
using RokubiminiWrenchRos = geometry_msgs::msg::WrenchStamped;
using RokubiminiImuRos = sensor_msgs::msg::Imu;
using RokubiminiTemperatureRos = sensor_msgs::msg::Temperature;
using DiagnosticArrayRos = diagnostic_msgs::msg::DiagnosticArray;
void RokubiminiEthercat::createRosPublishers()
{
  readingPublisher_ = nh_->create_publisher<RokubiminiReadingRos>(
      std::string(nh_->get_name()) + "/" + getName() + "/ft_sensor_readings/reading", 1);

  wrenchPublisher_ = nh_->create_publisher<RokubiminiWrenchRos>(
      std::string(nh_->get_name()) + "/" + getName() + "/ft_sensor_readings/wrench", 1);

  imuPublisher_ = nh_->create_publisher<RokubiminiImuRos>(
      std::string(nh_->get_name()) + "/" + getName() + "/ft_sensor_readings/imu", 1);

  temperaturePublisher_ = nh_->create_publisher<RokubiminiTemperatureRos>(
      std::string(nh_->get_name()) + "/" + getName() + "/ft_sensor_readings/temperature", 1);

  dataFlagsDiagnosticsPublisher_ = nh_->create_publisher<DiagnosticArrayRos>("/diagnostics", 1);
}

void RokubiminiEthercat::setRunsAsync(bool runsAsync)
{
  slavePtr_->setRunsAsync(runsAsync);
}
void RokubiminiEthercat::publishRosMessages()
{
  auto reading = getReading();
  rokubimini_msgs::msg::Reading reading_msg;
  reading_msg.header.stamp = reading.getWrench().header.stamp;
  // reading_msg.header.frame_id = reading.getWrench().header.frame_id;
  reading_msg.statusword = reading.getStatusword().getData();
  reading_msg.imu = reading.getImu();
  reading_msg.wrench = reading.getWrench();
  reading_msg.external_imu = reading.getExternalImu();
  reading_msg.is_force_torque_saturated = reading.isForceTorqueSaturated();
  reading_msg.temperature = reading.getTemperature();

  if (startCount_ < 3)
  {
    startCount_++;
    return;
  }
  // if reset wrench is triggered take the mean of the measurements
  if (computeMeanWrenchFlag_)
  {
    std::lock_guard<std::recursive_mutex> lock(meanWrenchOffsetMutex_);
    std::uint32_t lpos = wrenchMessageCount_;
    wrenchMessageCount_++;
    // increase the rows of the dynamic array
    resetServiceWrenchSamples_.conservativeResize(6, wrenchMessageCount_);
    // assign new values to the array
    resetServiceWrenchSamples_(0, lpos) = reading.getWrench().wrench.force.x;
    resetServiceWrenchSamples_(1, lpos) = reading.getWrench().wrench.force.y;
    resetServiceWrenchSamples_(2, lpos) = reading.getWrench().wrench.force.z;
    resetServiceWrenchSamples_(3, lpos) = reading.getWrench().wrench.torque.x;
    resetServiceWrenchSamples_(4, lpos) = reading.getWrench().wrench.torque.y;
    resetServiceWrenchSamples_(5, lpos) = reading.getWrench().wrench.torque.z;
  }
  geometry_msgs::msg::WrenchStamped tmp;
  tmp = reading.getWrench();
  // RCLCPP_INFO_STREAM(nh_->get_logger(), "Data:  " << tmp.wrench.force.x << std::endl);
  readingPublisher_->publish(reading_msg);
  //  RCLCPP_INFO_STREAM(nh_->get_logger(), "[RokubiminiEthercat::publishRosMessages] reading " <<
  //  reading_msg.wrench.wrench.force.x);
  //  RCLCPP_INFO_STREAM(nh_->get_logger(), "[RokubiminiEthercat::publishRosMessages] Fx " <<
  //  reading.getWrench().wrench.force.x); RCLCPP_INFO_STREAM(nh_->get_logger(),
  //  "[RokubiminiEthercat::publishRosMessages] Fy " << reading.getWrench().wrench.force.y);
  //  RCLCPP_INFO_STREAM(nh_->get_logger(),
  //  "[RokubiminiEthercat::publishRosMessages] Fz " << reading.getWrench().wrench.force.z);
  wrenchPublisher_->publish(tmp);
  imuPublisher_->publish(reading.getImu());
  temperaturePublisher_->publish(reading.getTemperature());
  // update the data flags to be used by diagnostics
  slavePtr_->updateDataFlags();
}

void RokubiminiEthercat::signalShutdown()
{
  // wait a small amount of time so that the callback can return the result to the user.
  std::this_thread::sleep_for(std::chrono::microseconds(500));
  kill(getpid(), SIGINT);
}

void RokubiminiEthercat::updateConnectionStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  slavePtr_->updateConnectionStatus(stat);
}

void RokubiminiEthercat::createRosDiagnostics()
{
  connectionStatusUpdater_ = std::make_shared<diagnostic_updater::Updater>(nh_, 0.1);
  unsigned int serial_number;
  getSerialNumber(serial_number);
  connectionStatusUpdater_->setHardwareID(std::to_string(serial_number));
  connectionStatusUpdater_->add(getName() + ": Status", this, &RokubiminiEthercat::updateConnectionStatus);
  // create the timer for the data flags diagnostics
  dataFlagsDiagnosticsTimer_ = rclcpp::create_timer(nh_, nh_->get_clock(), rclcpp::Duration(0, 100000000),
                                                    std::bind(&RokubiminiEthercat::publishDataFlagsDiagnostics, this));
}

void RokubiminiEthercat::publishDataFlagsDiagnostics()
{
  // publish the Data Status Diagnostics
  diagnostic_updater::DiagnosticStatusWrapper stat;
  slavePtr_->createDataFlagsDiagnostics(stat);
  // reset the data flags
  slavePtr_->resetDataFlags();
  // set the namespace of the diagnostics status
  stat.name = std::string(nh_->get_name()) + ": " + getName() + ": Data Flags";
  // set the hardware ID
  unsigned int serial_number;
  getSerialNumber(serial_number);
  stat.hardware_id = std::to_string(serial_number);
  // create the diagnostics array and fill it with the Diagnostics Status message
  diagnostic_msgs::msg::DiagnosticArray diagnostics_array;
  diagnostics_array.status.emplace_back(stat);
  // fill the Header
  diagnostics_array.header.stamp = nh_->get_clock()->now();
  dataFlagsDiagnosticsPublisher_->publish(diagnostics_array);
}

bool RokubiminiEthercat::firmwareUpdateCallback(
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat::Request> request,
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat::Response> response)
{
  response->result = slavePtr_->firmwareUpdate(request->file_path, request->file_name, request->password);
  if (!slavePtr_->isRunning())
  {
    // time to shut down the ROS node.
    std::thread shutdown_thread(&RokubiminiEthercat::signalShutdown, this);
    shutdown_thread.detach();
  }
  return true;
}

bool RokubiminiEthercat::resetWrenchCallback(std::shared_ptr<rokubimini_msgs::srv::ResetWrench::Request> request,
                                             std::shared_ptr<rokubimini_msgs::srv::ResetWrench::Response> response)
{
  RCLCPP_INFO(nh_->get_logger(), "Reseting sensor measurements...");
  bool success = false;
  unsigned int count = 0;
  Eigen::Matrix<double, 6, 1> new_offset;

  while (!success and count < 8)
  {
    // initialize all variables to zero
    wrenchMessageCount_ = 0;
    // enable the computation of the mean wrench
    computeMeanWrenchFlag_ = true;
    // wait for computing the mean of wrench measurements
    while (wrenchMessageCount_ != TOTAL_NUMBER_OF_WRENCH_MESSAGES)
      ;
    // disable the computation of the mean wrench
    computeMeanWrenchFlag_ = false;
    if (!setConfigMode())
    {
      RCLCPP_ERROR(nh_->get_logger(), "Device could not switch to config mode");
      response->success = false;
      return true;
    }
    geometry_msgs::msg::Wrench wrench;
    Eigen::Matrix<double, 6, 1> mean_wrench;
    // lock to get the mean wrench offset value
    {
      std::lock_guard<std::recursive_mutex> lock(meanWrenchOffsetMutex_);
      mean_wrench = resetServiceWrenchSamples_.rowwise().mean();
      wrench.force.x = mean_wrench(0, 0);
      wrench.force.y = mean_wrench(1, 0);
      wrench.force.z = mean_wrench(2, 0);
      wrench.torque.x = mean_wrench(3, 0);
      wrench.torque.y = mean_wrench(4, 0);
      wrench.torque.z = mean_wrench(5, 0);
    }
    geometry_msgs::msg::Wrench desired_wrench = request->desired_wrench;
    auto current_offset = configuration_.getForceTorqueOffset();
    // new offset = current wrench measurements + current offset - desired offset
    new_offset(0, 0) = desired_wrench.force.x - wrench.force.x + current_offset(0, 0);
    new_offset(1, 0) = desired_wrench.force.y - wrench.force.y + current_offset(1, 0);
    new_offset(2, 0) = desired_wrench.force.z - wrench.force.z + current_offset(2, 0);
    new_offset(3, 0) = desired_wrench.torque.x - wrench.torque.x + current_offset(3, 0);
    new_offset(4, 0) = desired_wrench.torque.y - wrench.torque.y + current_offset(4, 0);
    new_offset(5, 0) = desired_wrench.torque.z - wrench.torque.z + current_offset(5, 0);
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << getName() << "] "
                                               << "New offset is: " << new_offset);
    if (!setForceTorqueOffset(new_offset))
    {
      RCLCPP_ERROR(nh_->get_logger(), "Could not write new offset to device");
      response->success = false;
      return true;
    }
    if (!setRunMode())
    {
      RCLCPP_ERROR(nh_->get_logger(), "Device could not switch to run mode");
      response->success = false;
      return true;
    }
    response->success = true;
    configuration_.setForceTorqueOffset(new_offset);

    // initialize all variables to zero
    wrenchMessageCount_ = 0;
    // enable the computation of the mean wrench
    computeMeanWrenchFlag_ = true;
    // wait for computing the mean of wrench measurements
    while (wrenchMessageCount_ != TOTAL_NUMBER_OF_WRENCH_MESSAGES)
      ;
    // disable the computation of the mean wrench
    computeMeanWrenchFlag_ = false;

    // lock to get the mean wrench offset value
    {
      std::lock_guard<std::recursive_mutex> lock(meanWrenchOffsetMutex_);
      mean_wrench = resetServiceWrenchSamples_.rowwise().mean();
    }
    success = true;
    Eigen::Vector3d forceError;
    forceError << mean_wrench(0, 0) - desired_wrench.force.x, mean_wrench(1, 0) - desired_wrench.force.y,
        mean_wrench(2, 0) - desired_wrench.force.z;
    if (forceError.norm() > 0.2)
    {
      RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Could not reset wrench. Force error" << forceError);
      success = false;
    }

    Eigen::Vector3d torqueError;
    torqueError << mean_wrench(3, 0) - desired_wrench.torque.x, mean_wrench(4, 0) - desired_wrench.torque.y,
        mean_wrench(5, 0) - desired_wrench.torque.z;
    if (torqueError.norm() > 0.01)
    {
      RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Could not reset wrench. Torque error" << torqueError);
      success = false;
    }
    count++;
  }

  response->success = success;
  RCLCPP_INFO(nh_->get_logger(), "Sensor measurements are reset successfully");
  return true;
}
void RokubiminiEthercat::createRosServices()
{
  firmwareUpdateService_ = nh_->create_service<rokubimini_msgs::srv::FirmwareUpdateEthercat>(
      std::string(nh_->get_name()) + "/" + getName() + "/firmware_update",
      std::bind(&RokubiminiEthercat::firmwareUpdateCallback, this, std::placeholders::_1, std::placeholders::_2));
  resetWrenchService_ = nh_->create_service<rokubimini_msgs::srv::ResetWrench>(
      std::string(nh_->get_name()) + "/" + getName() + "/reset_wrench",
      std::bind(&RokubiminiEthercat::resetWrenchCallback, this, std::placeholders::_1, std::placeholders::_2));
}
bool RokubiminiEthercat::sendSdoReadGeneric(const std::string& indexString, const std::string& subindexString,
                                            const std::string& valueTypeString, std::string& valueString)
{
  return slavePtr_->sendSdoReadGeneric(indexString, subindexString, valueTypeString, valueString);
}

bool RokubiminiEthercat::sendSdoWriteGeneric(const std::string& indexString, const std::string& subindexString,
                                             const std::string& valueTypeString, const std::string& valueString)
{
  return slavePtr_->sendSdoWriteGeneric(indexString, subindexString, valueTypeString, valueString);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int8_t& value)
{
  return slavePtr_->sendSdoReadInt8(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int16_t& value)
{
  return slavePtr_->sendSdoReadInt16(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int32_t& value)
{
  return slavePtr_->sendSdoReadInt32(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int64_t& value)
{
  return slavePtr_->sendSdoReadInt64(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint8_t& value)
{
  return slavePtr_->sendSdoReadUInt8(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint16_t& value)
{
  return slavePtr_->sendSdoReadUInt16(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint32_t& value)
{
  return slavePtr_->sendSdoReadUInt32(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint64_t& value)
{
  return slavePtr_->sendSdoReadUInt64(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     float& value)
{
  return slavePtr_->sendSdoReadFloat(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     double& value)
{
  return slavePtr_->sendSdoReadDouble(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int8_t value)
{
  return slavePtr_->sendSdoWriteInt8(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int16_t value)
{
  return slavePtr_->sendSdoWriteInt16(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int32_t value)
{
  return slavePtr_->sendSdoWriteInt32(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int64_t value)
{
  return slavePtr_->sendSdoWriteInt64(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint8_t value)
{
  return slavePtr_->sendSdoWriteUInt8(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint16_t value)
{
  return slavePtr_->sendSdoWriteUInt16(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint32_t value)
{
  return slavePtr_->sendSdoWriteUInt32(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint64_t value)
{
  return slavePtr_->sendSdoWriteUInt64(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const float value)
{
  return slavePtr_->sendSdoWriteFloat(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const double value)
{
  return slavePtr_->sendSdoWriteDouble(index, subindex, completeAccess, value);
}

}  // namespace ethercat
}  // namespace rokubimini
