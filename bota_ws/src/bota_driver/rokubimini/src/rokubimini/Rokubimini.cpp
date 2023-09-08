// std
#include <chrono>

// rokubimini
#include <rokubimini/Rokubimini.hpp>
namespace rokubimini
{
void Rokubimini::load()
{
  std::string prefix = "rokubiminis/" + name_;
  std::string product_name_string = prefix + "/product_name";
  nh_->declare_parameter<std::string>(product_name_string, "BFT-xxx-xx-xx-xx");
  if (!nh_->get_parameter(product_name_string, productName_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Could not find product name parameter: %s", product_name_string.c_str());
  }

  configuration_.load(prefix, nh_);
}

void Rokubimini::startupWithCommunication()
{
  preSetupConfiguration();
  if (configuration_.hasForceTorqueFilter())
  {
    setForceTorqueFilter(configuration_.getForceTorqueFilter());
  }
  if (configuration_.hasImuAccelerationFilter())
  {
    setAccelerationFilter(configuration_.getImuAccelerationFilter());
  }
  if (configuration_.hasImuAngularRateFilter())
  {
    setAngularRateFilter(configuration_.getImuAngularRateFilter());
  }
  if (configuration_.hasImuAccelerationRange())
  {
    setAccelerationRange(configuration_.getImuAccelerationRange());
  }
  if (configuration_.hasImuAngularRateRange())
  {
    setAngularRateRange(configuration_.getImuAngularRateRange());
  }
  if (configuration_.hasSensorConfiguration())
  {
    setSensorConfiguration(configuration_.getSensorConfiguration());
  }
  if (configuration_.hasForceTorqueOffset())
  {
    setForceTorqueOffset(configuration_.getForceTorqueOffset());
  }

  if (configuration_.getUseCustomCalibration() && configuration_.hasSensorCalibration())
  {
    setSensorCalibration(configuration_.getSensorCalibration());
  }

  if (configuration_.hasSaveConfiguration())
  {
    if (configuration_.getSaveConfiguration())
    {
      saveConfigParameter();
    }
  }
  postSetupConfiguration();
}

void Rokubimini::startupWithoutCommunication()
{
}

void Rokubimini::clearGoalStateEnum()
{
  // stateMachine_.clearGoalStateEnum();
}

void Rokubimini::errorCb()
{
  for (const auto& error_cb : errorCbs_)
  {
    error_cb.second(getName());
  }
}

void Rokubimini::errorRecoveredCb()
{
  clearGoalStateEnum();
  for (const auto& error_recovered_cb : errorRecoveredCbs_)
  {
    error_recovered_cb.second(getName());
  }
}

bool Rokubimini::deviceIsInErrorState()
{
  return true;
  // return getStatusword().getStateEnum() == fsm::StateEnum::Error;
}

void Rokubimini::fatalCb()
{
  for (const auto& fatal_cb : fatalCbs_)
  {
    fatal_cb.second(getName());
  }
}

void Rokubimini::fatalRecoveredCb()
{
  clearGoalStateEnum();
  for (const auto& fatal_recovered_cb : fatalRecoveredCbs_)
  {
    fatal_recovered_cb.second(getName());
  }
}

bool Rokubimini::deviceIsInFatalState()
{
  return true;
  // return getStatusword().getStateEnum() == fsm::StateEnum::Fatal;
}

void Rokubimini::deviceDisconnectedCb()
{
  statuswordRequested_ = false;
  clearGoalStateEnum();
  for (const auto& device_disconnected_cb : deviceDisconnectedCbs_)
  {
    device_disconnected_cb.second(getName());
  }

  // Set statusword and reading accordingly.
  //  statusword_.resetData();
  Reading reading;
  const auto& stamp = nh_->now();
  {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.getWrench().header.stamp = stamp;
    reading_.getImu().header.stamp = stamp;
    reading_.setStatusword(statusword_);
    if (getConfiguration().getSetReadingToNanOnDisconnect())
    {
      reading_.getWrench().wrench.force.x = NAN_VALUE;
      reading_.getWrench().wrench.force.y = NAN_VALUE;
      reading_.getWrench().wrench.force.z = NAN_VALUE;
      reading_.getWrench().wrench.torque.x = NAN_VALUE;
      reading_.getWrench().wrench.torque.y = NAN_VALUE;
      reading_.getWrench().wrench.torque.z = NAN_VALUE;

      reading_.getImu().angular_velocity.x = NAN_VALUE;
      reading_.getImu().angular_velocity.y = NAN_VALUE;
      reading_.getImu().angular_velocity.z = NAN_VALUE;
      reading_.getImu().linear_acceleration.x = NAN_VALUE;
      reading_.getImu().linear_acceleration.y = NAN_VALUE;
      reading_.getImu().linear_acceleration.z = NAN_VALUE;
    }
    reading = reading_;
  }

  // External reading callbacks.
  for (const auto& reading_cb : readingCbs_)
  {
    reading_cb.second(getName(), reading);
  }
}

void Rokubimini::deviceReconnectedCb()
{
  // setGoalStateEnum(getConfiguration().getGoalStateEnumStartup());
  for (const auto& device_reconnected_cb : deviceReconnectedCbs_)
  {
    device_reconnected_cb.second(getName());
  }
}

Reading Rokubimini::getReading() const
{
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  return reading_;
}

void Rokubimini::getReading(Reading& reading) const
{
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  reading = reading_;
}

void Rokubimini::setStatusword(Statusword& statusword)
{
  // If the stamp has not changed, we assume it is again the same statusword.
  if (statusword.getStamp() == statusword_.getStamp())
  {
    return;
  }

  // Check if statusword contains new data.
  if (statusword_.isEmpty() || statusword.getData() != statusword_.getData())
  {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Received new statusword (" << statusword << ").");
    std::vector<std::string> infos;
    std::vector<std::string> warnings;
    std::vector<std::string> errors;
    std::vector<std::string> fatals;
    statusword.getMessagesDiff(statusword_, infos, warnings, errors, fatals);
    for (const std::string& info : infos)
    {
      RCLCPP_INFO_STREAM(nh_->get_logger(), "[" << name_.c_str() << "] " << info);
    }
    for (const std::string& warning : warnings)
    {
      RCLCPP_WARN_STREAM(nh_->get_logger(), "[" << name_.c_str() << "] " << warning);
    }
    for (const std::string& error : errors)
    {
      RCLCPP_ERROR_STREAM(nh_->get_logger(), "[" << name_.c_str() << "] " << error);
    }
    for (const std::string& fatal : fatals)
    {
      RCLCPP_ERROR_STREAM(nh_->get_logger(), "[" << name_.c_str() << "] " << fatal);
    }
  }

  // Always update statusword to set new time stamp.
  statusword_ = statusword;
}

}  // namespace rokubimini
