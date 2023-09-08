/**
 * @authors     Martin Wermelinger, Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Tests Command
 */

#include <gtest/gtest.h>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
namespace bota_driver_testing
{
class BotaDriverTestFTOffsets : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;
  std::uint32_t msgCount_;
  geometry_msgs::msg::Wrench meanWrenchOffset_;
  std::string topicName_;
  VectorXd fxSamples_;
  VectorXd fySamples_;
  VectorXd fzSamples_;
  VectorXd txSamples_;
  VectorXd tySamples_;
  VectorXd tzSamples_;
  int testDuration_;

  BotaDriverTestFTOffsets() : msgCount_(0)
  {
    nh_ = std::make_shared<rclcpp::Node>("bota_driver_test_ft_offsets");
    meanWrenchOffset_.force.x = 0;
    meanWrenchOffset_.force.y = 0;
    meanWrenchOffset_.force.z = 0;
    meanWrenchOffset_.torque.x = 0;
    meanWrenchOffset_.torque.y = 0;
    meanWrenchOffset_.torque.z = 0;
  }

  ~BotaDriverTestFTOffsets() override
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
  void meanwrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::uint32_t lpos = msgCount_;
    msgCount_++;
    fxSamples_.conservativeResize(msgCount_);
    fySamples_.conservativeResize(msgCount_);
    fzSamples_.conservativeResize(msgCount_);
    txSamples_.conservativeResize(msgCount_);
    tySamples_.conservativeResize(msgCount_);
    tzSamples_.conservativeResize(msgCount_);

    fxSamples_(lpos) = msg->wrench.force.x;
    fySamples_(lpos) = msg->wrench.force.y;
    fzSamples_(lpos) = msg->wrench.force.z;
    txSamples_(lpos) = msg->wrench.torque.x;
    tySamples_(lpos) = msg->wrench.torque.y;
    tzSamples_(lpos) = msg->wrench.torque.z;
  }
};

TEST_F(BotaDriverTestFTOffsets, SetFTOffsets)
{
  SCOPED_TRACE("SetFTOffsets");
  rclcpp::Time time_offset;
  nh_->declare_parameter<std::string>("topic_name");
  ASSERT_EQ(nh_->get_parameter<std::string>("topic_name", topicName_), true);

  sub_ = nh_->create_subscription<geometry_msgs::msg::WrenchStamped>(
      topicName_, 10, std::bind(&BotaDriverTestFTOffsets::meanwrenchCallback, this, std::placeholders::_1));

  nh_->declare_parameter<int>("test_duration");
  ASSERT_EQ(nh_->get_parameter<int>("test_duration", testDuration_), true);
  time_offset = nh_->get_clock()->now() + rclcpp::Duration(testDuration_, 0);
  while (rclcpp::ok() && nh_->get_clock()->now() < time_offset)
  {
    rclcpp::spin_some(nh_);
    if (HasFatalFailure())
    {
      FAIL() << "Fatal errors occurred.\n";
      return;
    }
  }
  EXPECT_GT(msgCount_, 0U);
  meanWrenchOffset_.force.x = fxSamples_.mean();
  meanWrenchOffset_.force.y = fySamples_.mean();
  meanWrenchOffset_.force.z = fzSamples_.mean();
  meanWrenchOffset_.torque.x = txSamples_.mean();
  meanWrenchOffset_.torque.y = tySamples_.mean();
  meanWrenchOffset_.torque.z = tzSamples_.mean();
  ASSERT_NEAR(meanWrenchOffset_.force.x, 10000.0, 1000);
  ASSERT_NEAR(meanWrenchOffset_.force.y, -10000.0, 1000);
  ASSERT_NEAR(meanWrenchOffset_.force.z, 20000.0, 1000);
  ASSERT_NEAR(meanWrenchOffset_.torque.x, -20000.0, 100);
  ASSERT_NEAR(meanWrenchOffset_.torque.y, 30000.0, 100);
  ASSERT_NEAR(meanWrenchOffset_.torque.z, -30000.0, 100);
}

}  // namespace bota_driver_testing