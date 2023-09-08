/**
 * @authors     Martin Wermelinger, Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Tests Command
 */

#include <gtest/gtest.h>
#include <chrono>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rokubimini_msgs/srv/reset_wrench.hpp>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
namespace bota_driver_testing
{
using namespace std::chrono_literals;

class BotaDriverTestResetService : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;
  rclcpp::Client<rokubimini_msgs::srv::ResetWrench>::SharedPtr client_;
  std::uint32_t msgCount_;
  geometry_msgs::msg::Wrench meanWrenchOffset_;
  std::string topicName_;
  std::string serviceName_;
  VectorXd fxSamples_;
  VectorXd fySamples_;
  VectorXd fzSamples_;
  VectorXd txSamples_;
  VectorXd tySamples_;
  VectorXd tzSamples_;
  double fx_, fy_, fz_, tx_, ty_, tz_;

  BotaDriverTestResetService() : msgCount_(0)
  {
    nh_ = std::make_shared<rclcpp::Node>("bota_driver_test_reset_service");
    meanWrenchOffset_.force.x = 0.0;
    meanWrenchOffset_.force.y = 0.0;
    meanWrenchOffset_.force.z = 0.0;
    meanWrenchOffset_.torque.x = 0.0;
    meanWrenchOffset_.torque.y = 0.0;
    meanWrenchOffset_.torque.z = 0.0;
  }

  ~BotaDriverTestResetService() override
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
  void resetWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
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

TEST_F(BotaDriverTestResetService, CustomWrench)
{
  SCOPED_TRACE("TestResetServiceCustomWrench");
  rclcpp::Time time_offset;
  // add a small time offset for the serial to start
  int startup_duration;
  // callback duration
  int callback_duration;
  nh_->declare_parameter<std::string>("topic_name");
  nh_->declare_parameter<std::string>("service_name");
  nh_->declare_parameter<double>("fx");
  nh_->declare_parameter<double>("fy");
  nh_->declare_parameter<double>("fz");
  nh_->declare_parameter<double>("tx");
  nh_->declare_parameter<double>("ty");
  nh_->declare_parameter<double>("tz");
  nh_->declare_parameter<int>("startup_duration");
  nh_->declare_parameter<int>("callback_duration");
  ASSERT_EQ(nh_->get_parameter<std::string>("topic_name", topicName_), true);
  ASSERT_EQ(nh_->get_parameter<std::string>("service_name", serviceName_), true);
  ASSERT_EQ(nh_->get_parameter<double>("fx", fx_), true);
  ASSERT_EQ(nh_->get_parameter<double>("fy", fy_), true);
  ASSERT_EQ(nh_->get_parameter<double>("fz", fz_), true);
  ASSERT_EQ(nh_->get_parameter<double>("tx", tx_), true);
  ASSERT_EQ(nh_->get_parameter<double>("ty", ty_), true);
  ASSERT_EQ(nh_->get_parameter<double>("tz", tz_), true);
  ASSERT_EQ(nh_->get_parameter<int>("callback_duration", callback_duration), true);
  ASSERT_EQ(nh_->get_parameter<int>("startup_duration", startup_duration), true);

  /*
   * Step 1
   * Create the previous offset of wrench.
   *
   */
  time_offset = nh_->get_clock()->now() + rclcpp::Duration(startup_duration, 0);
  while (rclcpp::ok() && nh_->get_clock()->now() < time_offset)
  {
    rclcpp::spin_some(nh_);
    if (HasFatalFailure())
    {
      FAIL() << "Fatal errors occurred.\n";
      return;
    }
  }

  /*
   * Step 2
   * Call successfully the ros service (reset_wrench).
   *
   */
  client_ = nh_->create_client<rokubimini_msgs::srv::ResetWrench>(serviceName_);
  auto request = std::make_shared<rokubimini_msgs::srv::ResetWrench::Request>();

  ASSERT_TRUE(client_->wait_for_service(1s));

  //  ASSERT_TRUE(ros::service::waitForService(serviceName_));
  request->desired_wrench = geometry_msgs::msg::Wrench();
  request->desired_wrench.force.x = fx_;
  request->desired_wrench.force.y = fy_;
  request->desired_wrench.force.z = fz_;
  request->desired_wrench.torque.x = tx_;
  request->desired_wrench.torque.y = ty_;
  request->desired_wrench.torque.z = tz_;

  auto result = client_->async_send_request(request);
  // Wait for the result.
  ASSERT_TRUE(rclcpp::spin_until_future_complete(nh_, result) == rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result.get()->success, true);

  /*
   * Step 3
   * Gather measurements to form the new mean offset of wrench.
   *
   */
  // Sleep to allow the sensor to send msgs of the reset wrench.
  rclcpp::Rate sleepRate(2.0);
  sleepRate.sleep();

  sub_ = nh_->create_subscription<geometry_msgs::msg::WrenchStamped>(
      topicName_, 1, std::bind(&BotaDriverTestResetService::resetWrenchCallback, this, std::placeholders::_1));
  time_offset = nh_->get_clock()->now() + rclcpp::Duration(callback_duration, 0);
  while (rclcpp::ok() && nh_->get_clock()->now() < time_offset)
  {
    rclcpp::spin_some(nh_);
    if (HasFatalFailure())
    {
      FAIL() << "Fatal errors occurred.\n";
      return;
    }
  }

  meanWrenchOffset_.force.x = fxSamples_.mean();
  meanWrenchOffset_.force.y = fySamples_.mean();
  meanWrenchOffset_.force.z = fzSamples_.mean();
  meanWrenchOffset_.torque.x = txSamples_.mean();
  meanWrenchOffset_.torque.y = tySamples_.mean();
  meanWrenchOffset_.torque.z = tzSamples_.mean();
  ASSERT_GT(msgCount_, 0U);
  EXPECT_NEAR(meanWrenchOffset_.force.x, fx_, 0.2);
  EXPECT_NEAR(meanWrenchOffset_.force.y, fy_, 0.2);
  EXPECT_NEAR(meanWrenchOffset_.force.z, fz_, 0.2);
  EXPECT_NEAR(meanWrenchOffset_.torque.x, tx_, 0.01);
  EXPECT_NEAR(meanWrenchOffset_.torque.y, ty_, 0.01);
  EXPECT_NEAR(meanWrenchOffset_.torque.z, tz_, 0.01);
}
}  // namespace bota_driver_testing