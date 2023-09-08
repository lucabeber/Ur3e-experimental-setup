#include <rokubimini_serial/RokubiminiSerial.hpp>
#include <csignal>
#include <thread>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace rokubimini
{
namespace serial
{
void RokubiminiSerial::postSetupConfiguration()
{
  /*
  ** Print configurations of the sensor
  */
  // configuration_.printConfiguration();
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Calibration Matrix of the sensor: "
                                             << configuration_.getSensorCalibration().getCalibrationMatrix()
                                             << std::endl);

  if (implPtr_->runsAsync())
  {
    // start publishing thread
    if (!publishingThread_.joinable())
    {
      RCLCPP_INFO(nh_->get_logger(), "Launching publishing thread.");
      publishingThread_ = std::thread{ &RokubiminiSerial::update, this };
    }
  }
  implPtr_->startup();
}

void RokubiminiSerial::preSetupConfiguration()
{
  parseCommunicationMsgs();
}

void RokubiminiSerial::parseCommunicationMsgs()
{
  if (!implPtr_->parseCommunicationMsgs())
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to parse communication messages");
  }
  if (productName_ != implPtr_->getProductName())
  {
    RCLCPP_WARN(nh_->get_logger(),
                "[%s] Invalid product name '%s' given, didn't match the actual product name of the device: '%s'",
                name_.c_str(), productName_.c_str(), implPtr_->getProductName().c_str());
    productName_ = implPtr_->getProductName();
  }
}

bool RokubiminiSerial::setPublishMode(double timeStep)
{
  // if the timeStep variable is 0, we set the flag 'runsAsync' to true. By default the 'runsAsync' flag is false.
  if (timeStep == 0)
  {
    implPtr_->setRunsAsync(true);
  }
  else
  {
    implPtr_->setPollingTimeStep(timeStep);
  }
  return true;
}

bool RokubiminiSerial::init()
{
  return implPtr_->init();
}

void RokubiminiSerial::update()
{
  while (implPtr_->isRunning())
  {
    updateProcessReading();
    publishRosMessages();
  }
}
void RokubiminiSerial::updateProcessReading()
{
  // if the polling is async and the thread reaching this method is from application, return immediately.
  {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    implPtr_->getReading(reading_);

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

void RokubiminiSerial::shutdownWithCommunication()
{
  implPtr_->shutdown();
  // publish the last diagnostics before shutdown
  connectionStatusUpdater_->force_update();
  if (implPtr_->runsAsync())
  {
    // Shutdown the publishing thread if running
    if (publishingThread_.joinable())
    {
      publishingThread_.join();
    }
  }
}

bool RokubiminiSerial::deviceIsMissing() const
{
  return false;
}

bool RokubiminiSerial::getSerialNumber(unsigned int& serialNumber)
{
  return implPtr_->getSerialNumber(serialNumber);
}

bool RokubiminiSerial::getForceTorqueSamplingRate(int& samplingRate)
{
  return implPtr_->getForceTorqueSamplingRate(samplingRate);
}

bool RokubiminiSerial::setForceTorqueFilter(const configuration::ForceTorqueFilter& filter)
{
  return implPtr_->setForceTorqueFilter(filter);
}

bool RokubiminiSerial::setAccelerationFilter(const unsigned int filter)
{
  return implPtr_->setAccelerationFilter(filter);
}

bool RokubiminiSerial::setAngularRateFilter(const unsigned int filter)
{
  return implPtr_->setAngularRateFilter(filter);
}

bool RokubiminiSerial::setAccelerationRange(const unsigned int range)
{
  return implPtr_->setAccelerationRange(range);
}

bool RokubiminiSerial::setAngularRateRange(const unsigned int range)
{
  return implPtr_->setAngularRateRange(range);
}

bool RokubiminiSerial::setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset)
{
  return implPtr_->setForceTorqueOffset(forceTorqueOffset);
}

bool RokubiminiSerial::setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration)
{
  if (!implPtr_->setSensorConfiguration(sensorConfiguration))
  {
    return false;
  }
  getConfiguration().setSensorConfiguration(sensorConfiguration);
  return true;
}

bool RokubiminiSerial::setSensorCalibration(const calibration::SensorCalibration& sensorCalibration)
{
  if (!implPtr_->setSensorCalibration(sensorCalibration))
  {
    return false;
  }
  getConfiguration().setSensorCalibration(sensorCalibration);
  return true;
}

bool RokubiminiSerial::setConfigMode()
{
  return implPtr_->setConfigMode();
}

bool RokubiminiSerial::setRunMode()
{
  return implPtr_->setRunMode();
}

bool RokubiminiSerial::saveConfigParameter()
{
  return implPtr_->saveConfigParameter();
}

bool RokubiminiSerial::loadConfig()
{
  return implPtr_->loadConfig();
}

bool RokubiminiSerial::printUserConfig()
{
  return implPtr_->printUserConfig();
}

bool RokubiminiSerial::setHardwareReset()
{
  return implPtr_->setHardwareReset();
}

bool RokubiminiSerial::setInitMode()
{
  return implPtr_->setInitMode();
}

using RokubiminiReadingRos = rokubimini_msgs::msg::Reading;
using RokubiminiWrenchRos = geometry_msgs::msg::WrenchStamped;
using RokubiminiTemperatureRos = sensor_msgs::msg::Temperature;
using DiagnosticArrayRos = diagnostic_msgs::msg::DiagnosticArray;
void RokubiminiSerial::createRosPublishers()
{
  readingPublisher_ = nh_->create_publisher<RokubiminiReadingRos>(
      std::string(nh_->get_name()) + "/" + getName() + "/ft_sensor_readings/reading", 1);

  wrenchPublisher_ = nh_->create_publisher<RokubiminiWrenchRos>(
      std::string(nh_->get_name()) + "/" + getName() + "/ft_sensor_readings/wrench", 1);

  temperaturePublisher_ = nh_->create_publisher<RokubiminiTemperatureRos>(
      std::string(nh_->get_name()) + "/" + getName() + "/ft_sensor_readings/temperature", 1);

  dataFlagsDiagnosticsPublisher_ = nh_->create_publisher<DiagnosticArrayRos>("/diagnostics", 1);
  publishFlag_ = true;
}

void RokubiminiSerial::publishRosMessages()
{
  if (publishFlag_ && implPtr_->hasFrameSync() && implPtr_->isRunning())
  {
    auto reading = getReading();
    rokubimini_msgs::msg::Reading reading_msg;
    reading_msg.header.stamp = reading.getWrench().header.stamp;
    reading_msg.header.frame_id = reading.getWrench().header.frame_id;
    reading_msg.statusword = reading.getStatusword().getData();
    reading_msg.wrench = reading.getWrench();
    reading_msg.is_force_torque_saturated = reading.isForceTorqueSaturated();
    reading_msg.temperature = reading.getTemperature();

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
    readingPublisher_->publish(reading_msg);
    wrenchPublisher_->publish(reading.getWrench());
    temperaturePublisher_->publish(reading.getTemperature());
    // update the data flags to be used by diagnostics
    implPtr_->updateDataFlags();
    // reset the "no-frame" counter
    noFrameSyncCounter_ = 0;
  }
  else
  {
    noFrameSyncCounter_++;
  }
  // check if there is no synced frame over 100 times
  if (noFrameSyncCounter_ > 100)
  {
    RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 3, "Driver failed to synchronize with the device");
  }
}

void RokubiminiSerial::signalShutdown()
{
  // wait a small amount of time so that the callback can return the result to the user.
  std::this_thread::sleep_for(std::chrono::microseconds(500));
  kill(getpid(), SIGINT);
}

bool RokubiminiSerial::firmwareUpdateCallback(
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial::Request> request,
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial::Response> response)
{
  response->result = implPtr_->firmwareUpdate(request->file_path);
  if (!implPtr_->isRunning())
  {
    // time to shut down the ROS node.
    std::thread shutdown_thread(&RokubiminiSerial::signalShutdown, this);
    shutdown_thread.detach();
  }
  return true;
}

bool RokubiminiSerial::resetWrenchCallback(std::shared_ptr<rokubimini_msgs::srv::ResetWrench::Request> request,
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
void RokubiminiSerial::createRosServices()
{
  firmwareUpdateService_ = nh_->create_service<rokubimini_msgs::srv::FirmwareUpdateSerial>(
      std::string(nh_->get_name()) + "/" + getName() + "/firmware_update",
      std::bind(&RokubiminiSerial::firmwareUpdateCallback, this, std::placeholders::_1, std::placeholders::_2));
  resetWrenchService_ = nh_->create_service<rokubimini_msgs::srv::ResetWrench>(
      std::string(nh_->get_name()) + "/" + getName() + "/reset_wrench",
      std::bind(&RokubiminiSerial::resetWrenchCallback, this, std::placeholders::_1, std::placeholders::_2));
}
void RokubiminiSerial::updateConnectionStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  implPtr_->updateConnectionStatus(stat);
}

void RokubiminiSerial::createRosDiagnostics()
{
  connectionStatusUpdater_ = std::make_shared<diagnostic_updater::Updater>(nh_, 0.1);
  unsigned int serial_number;
  implPtr_->getSerialNumber(serial_number);
  connectionStatusUpdater_->setHardwareID(std::to_string(serial_number));
  connectionStatusUpdater_->add(getName() + ": Device Connection Status", this,
                                &RokubiminiSerial::updateConnectionStatus);
  // create the timer for the data flags diagnostics
  dataFlagsDiagnosticsTimer_ = rclcpp::create_timer(nh_, nh_->get_clock(), rclcpp::Duration(0, 100000000),
                                                    std::bind(&RokubiminiSerial::publishDataFlagsDiagnostics, this));
}

void RokubiminiSerial::publishDataFlagsDiagnostics()
{
  // publish the Data Flags Diagnostics
  diagnostic_updater::DiagnosticStatusWrapper stat;
  implPtr_->createDataFlagsDiagnostics(stat);
  // reset the data flags
  implPtr_->resetDataFlags();
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
}  // namespace serial
}  // namespace rokubimini
