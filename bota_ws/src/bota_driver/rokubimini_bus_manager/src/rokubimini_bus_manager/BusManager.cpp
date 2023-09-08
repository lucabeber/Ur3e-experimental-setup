
#include <rokubimini_bus_manager/BusManager.hpp>

namespace rokubimini
{
bool RokubiminiBusManager::init()
{
  std::string setup_file;

  name_ = nh_->get_name();
  if (!load())
  {
    RCLCPP_ERROR(nh_->get_logger(), "[%s] Could not load bus configuration", name_.c_str());
    return false;
  }

  if (!startup())
  {
    RCLCPP_WARN(nh_->get_logger(), "[%s] Bus Manager could not startup", name_.c_str());
    return false;
  }
  createRokubiminiRosPublishers();
  createRokubiminiRosServices();
  createRokubiminiRosDiagnostics();
  std::vector<std::shared_ptr<Rokubimini>> rokubiminis;
  if (getRokubiminis().empty())
  {
    RCLCPP_ERROR(nh_->get_logger(), "[%s] No rokubimini available", name_.c_str());
    return false;
  }
  bota_worker::WorkerOptions options;
  options.callback_ = std::bind(&RokubiminiBusManager::update, this, std::placeholders::_1);
  options.defaultPriority_ = 10;  // this has high priority
  options.name_ = "RokubiminiBusManager::updateWorker";
  options.timeStep_ = loadTimeStep();
  if (options.timeStep_ != 0)
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(),
                       "Starting Worker at " << 1 / options.timeStep_ << " Hz, based on force/torque sampling rate.");
    if (!this->addWorker(options))
    {
      RCLCPP_ERROR_STREAM(nh_->get_logger(),
                          "[RokubiminiBusManager] Worker " << options.name_ << "could not be added!");
      return false;
    }
  }
  return true;
}

bool RokubiminiBusManager::startup()
{
  if (isRunning_)
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "Cannot start up, Rokubimini Bus Manager is already running.");
    return false;
  }
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Starting up Rokubimini Bus Manager ...");
  startupWithoutCommunication();
  if (!startupCommunication())
  {
    shutdownWithoutCommunication();
    return false;
  }
  setConfigMode();
  startupWithCommunication();
  setRunMode();
  isRunning_ = true;
  return true;
}

bool RokubiminiBusManager::update(const bota_worker::WorkerEvent& event)
{
  RCLCPP_DEBUG(nh_->get_logger(), "[RokubiminiBusManager]: update called");
  readBus();
  updateProcessReading();
  writeToBus();
  publishRosMessages();
  return true;
}

void RokubiminiBusManager::cleanup()
{
  shutdown();
}

void RokubiminiBusManager::shutdown()
{
  if (!isRunning_)
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "Cannot shut down, Rokubimini Bus Manager is not running.");
    return;
  }
  shutdownWithCommunication();
  shutdownBus();
  shutdownWithoutCommunication();
  isRunning_ = false;
}

bool RokubiminiBusManager::addRokubiminiToBus(const std::shared_ptr<Rokubimini>& rokubimini) const
{
  return true;
};
bool RokubiminiBusManager::load()
{
  if (!loadBusParameters())
  {
    RCLCPP_ERROR(nh_->get_logger(), "Could not load bus parameters");
    return false;
  }
  if (!createRokubiminisFromParamServer())
  {
    RCLCPP_ERROR(nh_->get_logger(), "Could not create rokubiminis from parameters.");
    return false;
  }
  return true;
}

bool RokubiminiBusManager::createRokubiminisFromParamServer()
{
  bool success = false;
  nh_->declare_parameter<std::vector<std::string>>("rokubiminis", std::vector<std::string>());
  std::vector<std::string> rokubiminis;
  nh_->get_parameter("rokubiminis", rokubiminis);
  for (const auto& rokubimini_name : rokubiminis)
  {
    if (!createRokubimini(rokubimini_name))
    {
      RCLCPP_ERROR(nh_->get_logger(), "Could not create rokubimini");
      return false;
    };
    RCLCPP_INFO(nh_->get_logger(), "Successfully created rokubimini with name: %s", rokubimini_name.c_str());
    success = true;
  }

  if (!success)
  {
    RCLCPP_ERROR_STREAM(nh_->get_logger(), "Could not find parameters to create a rokubimini.");
  }
  return success;
}
}  // namespace rokubimini
