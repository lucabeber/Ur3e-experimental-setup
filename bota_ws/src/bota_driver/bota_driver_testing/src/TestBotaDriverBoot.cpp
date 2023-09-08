/**
 * @authors     Martin Wermelinger, Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Tests Command
 */

#include <gtest/gtest.h>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bota_driver_testing
{
class BotaDriverTestBoot : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr sub_;
  std::vector<std::string> nodeNames_;
  int testDuration_;

  BotaDriverTestBoot()
  {
    nh_ = std::make_shared<rclcpp::Node>("bota_driver_test_boot");
  }

  ~BotaDriverTestBoot() override
  {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override
  {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  void TearDown() override
  {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

public:
  void rosoutCallback(const rcl_interfaces::msg::Log::SharedPtr msg)
  {
    //    RCLCPP_INFO(nh_->get_logger(), "rosoutCallback");
    // Check that the message has not type ERROR || FATAL and the sender node is not rokubimini.
    for (const auto node : nodeNames_)
    {
      if (msg->name == node)
      {
        ASSERT_EQ((msg->level == rcl_interfaces::msg::Log::ERROR || msg->level == rcl_interfaces::msg::Log::FATAL),
                  false);
      }
    }
  }
};

TEST_F(BotaDriverTestBoot, NoErrorsInBoot)
{
  SCOPED_TRACE("NoErrorsInBoot");
  nh_->declare_parameter<std::vector<std::string>>("nodes");
  ASSERT_EQ(nh_->get_parameter<std::vector<std::string>>(std::string("nodes"), nodeNames_), true);
  sub_ = nh_->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", 5, std::bind(&BotaDriverTestBoot::rosoutCallback, this, std::placeholders::_1));
  nh_->declare_parameter<int>("test_duration");
  ASSERT_EQ(nh_->get_parameter<int>("test_duration", testDuration_), true);
  rclcpp::Time time_after_boot = nh_->get_clock()->now() + rclcpp::Duration(testDuration_, 0);
  while (rclcpp::ok() && nh_->get_clock()->now() < time_after_boot)
  {
    rclcpp::spin_some(nh_);
    if (HasFatalFailure())
    {
      FAIL() << "There were errors in the boot process.\nReceived message from rokubimini node with type ERROR or "
                "FATAL.";
      return;
    }
  }
}

}  // namespace bota_driver_testing
