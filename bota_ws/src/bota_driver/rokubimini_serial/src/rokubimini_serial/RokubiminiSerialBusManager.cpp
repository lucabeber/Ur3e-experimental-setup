
#include <rokubimini_serial/RokubiminiSerialBusManager.hpp>

namespace rokubimini
{
namespace serial
{
double RokubiminiSerialBusManager::loadTimeStep()
{
  return timeStep_;
}
void RokubiminiSerialBusManager::fetchTimeStep()
{
  nh_->declare_parameter<double>("time_step", 0.0);
  if (!nh_->get_parameter("time_step", timeStep_))
  {
    RCLCPP_INFO(nh_->get_logger(),
                "Could not find the 'time_step' parameter in Parameter Server. Running asynchronously");
    timeStep_ = 0;
  }
}

bool RokubiminiSerialBusManager::createRokubimini(const std::string& rokubiminiName)
{
  auto rokubimini = std::make_shared<RokubiminiSerial>(rokubiminiName, nh_);
  rokubimini->load();
  rokubiminis_.emplace_back(rokubimini);
  if (!addRokubiminiToBus(rokubimini))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Could not add rokubimini to bus");
    return false;
  }
  return true;
}

bool RokubiminiSerialBusManager::loadBusParameters()
{
  std::string port_string = "port";
  nh_->declare_parameter<std::string>(port_string, "/dev/ttyUSB0");
  if (!nh_->get_parameter(port_string, serialPort_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Could not find serial port in Parameter Server: %s", port_string.c_str());
    return false;
  }
  return true;
}

bool RokubiminiSerialBusManager::addRokubiminiToBus(const std::shared_ptr<RokubiminiSerial>& rokubimini) const
{
  auto impl_ptr = std::make_shared<RokubiminiSerialImpl>(rokubimini->getName(), serialPort_);

  rokubimini->setImplPointer(impl_ptr);
  return true;
}

bool RokubiminiSerialBusManager::startupCommunication()
{
  for (auto& rokubimini : rokubiminis_)
  {
    auto rokubimini_serial = std::dynamic_pointer_cast<RokubiminiSerial>(rokubimini);
    fetchTimeStep();
    if (!rokubimini_serial->setPublishMode(timeStep_))
    {
      RCLCPP_ERROR(nh_->get_logger(), "[%s] Failed to set publish mode (sync vs async) to the serial device",
                   rokubimini_serial->getName().c_str());
      return false;
    }
    if (!rokubimini_serial->init())
    {
      RCLCPP_ERROR(nh_->get_logger(), "[%s] Failed to initialize the serial device",
                   rokubimini_serial->getName().c_str());
      return false;
    }
  }
  return true;
}

void RokubiminiSerialBusManager::setConfigMode()
{
  for (const auto& rokubimini : rokubiminis_)
  {
    auto rokubimini_serial = std::dynamic_pointer_cast<RokubiminiSerial>(rokubimini);
    if (!rokubimini_serial->setConfigMode())
    {
      RCLCPP_ERROR(nh_->get_logger(), "[%s] The Serial device could not switch to configuration mode",
                   rokubimini_serial->getName().c_str());
    }
  }
}
void RokubiminiSerialBusManager::setRunMode()
{
  for (const auto& rokubimini : rokubiminis_)
  {
    auto rokubimini_serial = std::dynamic_pointer_cast<RokubiminiSerial>(rokubimini);
    if (!rokubimini_serial->setRunMode())
    {
      RCLCPP_ERROR(nh_->get_logger(), "[%s] The Serial device could not switch to run mode",
                   rokubimini_serial->getName().c_str());
    }
  }
}
}  // namespace serial
}  // namespace rokubimini