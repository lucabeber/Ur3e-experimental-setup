#include <rokubimini/configuration/ForceTorqueFilter.hpp>

namespace rokubimini
{
namespace configuration
{
ForceTorqueFilter::ForceTorqueFilter(const uint16_t sincFilterSize, const uint8_t chopEnable, const uint8_t skipEnable,
                                     const uint8_t fastEnable)
  : sincFilterSize_(sincFilterSize), chopEnable_(chopEnable), skipEnable_(skipEnable), fastEnable_(fastEnable)
{
}

bool ForceTorqueFilter::load(const std::string& key, const NodeHandlePtr& nh)
{
  bool success = true;
  std::string local_key;
  local_key = key + "/sinc_filter_size";
  int sinc_filter_size;
  nh->declare_parameter<int>(local_key, 256);
  if (nh->get_parameter(local_key, sinc_filter_size))
  {
    sincFilterSize_ = static_cast<uint16_t>(sinc_filter_size);
    success = success && true;
  }
  local_key = key + "/chop_enable";
  bool chop_enable;
  nh->declare_parameter<bool>(local_key, false);
  if (nh->get_parameter(local_key, chop_enable))
  {
    chopEnable_ = static_cast<uint8_t>(chop_enable);
    success = success && true;
  }
  local_key = key + "/fir_disable";
  bool skip_enable;
  nh->declare_parameter<bool>(local_key, true);
  if (nh->get_parameter(local_key, skip_enable))
  {
    skipEnable_ = static_cast<uint8_t>(skip_enable);
    success = success && true;
  }
  local_key = key + "/fast_enable";
  bool fast_enable;
  nh->declare_parameter<bool>(local_key, false);
  if (nh->get_parameter(local_key, fast_enable))
  {
    fastEnable_ = static_cast<uint8_t>(fast_enable);
    success = success && true;
  }
  return success;
}

void ForceTorqueFilter::print() const
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ForceTorqueFilter"),
                     "sinc_filter_size_: " << static_cast<unsigned int>(sincFilterSize_));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ForceTorqueFilter"),
                     "chopEnable_: " << static_cast<unsigned int>(chopEnable_));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ForceTorqueFilter"),
                     "skipEnable_: " << static_cast<unsigned int>(skipEnable_));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ForceTorqueFilter"),
                     "fastEnable_: " << static_cast<unsigned int>(fastEnable_));
}

}  // namespace configuration
}  // namespace rokubimini
