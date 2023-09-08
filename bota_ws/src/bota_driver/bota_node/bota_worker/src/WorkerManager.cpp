/*!
 * @file	WorkerManager.cpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#include "bota_worker/WorkerManager.hpp"
#include <rclcpp/rclcpp.hpp>
namespace bota_worker
{
WorkerManager::WorkerManager() : workers_(), mutexWorkers_()
{
}

WorkerManager::~WorkerManager()
{
  cancelWorkers();
}

bool WorkerManager::addWorker(const WorkerOptions& options, const bool autostart)
{
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto inserted_element = workers_.emplace(options.name_, Worker(options));
  if (!inserted_element.second)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Bota Worker"), "Failed to create worker [%s]", options.name_.c_str());
    return false;
  }
  if (autostart)
  {
    return inserted_element.first->second.start();
  }
  return true;
}

// bool WorkerManager::addWorker(Worker&& worker) {
//    std::lock_guard<std::mutex> lock(mutexWorkers_);
//    auto insertedElement = workers_.emplace( worker.getName(), std::move(worker) );
//    if(!insertedElement.second) {
//        ROS_ERROR("Failed to move worker [%s]", worker.getName().c_str());
//        return false;
//    }
//    return true;
//}

void WorkerManager::startWorker(const std::string& name, const int priority)
{
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto worker = workers_.find(name);
  if (worker == workers_.end())
  {
    RCLCPP_ERROR(rclcpp::get_logger("Bota Worker"), "Cannot start worker [%s], worker not found", name.c_str());
    return;
  }
  worker->second.start(priority);
}

void WorkerManager::startWorkers()
{
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  for (auto& worker : workers_)
  {
    worker.second.start();
  }
}

void WorkerManager::stopWorker(const std::string& name, const bool wait)
{
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto worker = workers_.find(name);
  if (worker == workers_.end())
  {
    RCLCPP_ERROR(rclcpp::get_logger("Bota Worker"), "Cannot stop worker [%s], worker not found", name.c_str());
    return;
  }
  worker->second.stop(wait);
}

void WorkerManager::stopWorkers(const bool wait)
{
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  for (auto& worker : workers_)
  {
    worker.second.stop(wait);
  }
}

bool WorkerManager::hasWorker(const std::string& name)
{
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  return (workers_.find(name) != workers_.end());
}

void WorkerManager::cancelWorker(const std::string& name, const bool wait)
{
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto worker = workers_.find(name);
  if (worker == workers_.end())
  {
    RCLCPP_ERROR(rclcpp::get_logger("Bota Worker"), "Cannot stop worker [%s], worker not found", name.c_str());
    return;
  }
  worker->second.stop(wait);
  workers_.erase(worker);
}

void WorkerManager::cancelWorkers(const bool wait)
{
  std::lock_guard<std::mutex> lock(mutexWorkers_);

  // signal all workers to stop
  for (auto& worker : workers_)
  {
    worker.second.stop(wait);
  }

  // call destructors of all workers, which will join the underlying thread
  workers_.clear();
}

void WorkerManager::setWorkerTimestep(const std::string& name, const double timeStep)
{
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto worker = workers_.find(name);
  if (worker == workers_.end())
  {
    RCLCPP_ERROR(rclcpp::get_logger("Bota Worker"), "Cannot change timestep of worker [%s], worker not found",
                 name.c_str());
    return;
  }
  worker->second.setTimestep(timeStep);
}

void WorkerManager::cleanDestructibleWorkers()
{
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  for (auto it = workers_.begin(); it != workers_.end();)
  {
    if (it->second.isDestructible())
    {
      it = workers_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

}  // namespace bota_worker
