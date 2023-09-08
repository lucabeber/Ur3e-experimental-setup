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

#define ASSERT_DOUBLE_NOT_NEAR(val1, val2, abs_error)                                                                  \
  ASSERT_PRED_FORMAT3(!::testing::internal::DoubleNearPredFormat, val1, val2, abs_error)

using namespace Eigen;

namespace bota_driver_testing
{
class BotaDriverSignalQuality : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;
  std::uint32_t msgCount_;
  std::uint32_t maxCount_;
  std::string topicName_;
  VectorXd fxSamples_;
  VectorXd fySamples_;
  VectorXd fzSamples_;
  VectorXd txSamples_;
  VectorXd tySamples_;
  VectorXd tzSamples_;
  int testDuration_;
  bool skip_;

  BotaDriverSignalQuality() : msgCount_(0), maxCount_(1000), skip_(true)
  {
    nh_ = std::make_shared<rclcpp::Node>("bota_driver_test_signal_quality");
  }

  ~BotaDriverSignalQuality() override
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
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    if (msg->header.stamp.sec <= 0)
      return;
    if (skip_)
    {
      skip_ = false;
      return;
    }
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

TEST_F(BotaDriverSignalQuality, SignalQuality)
{
  SCOPED_TRACE("SignalQualityFT");
  rclcpp::Time time_offset;
  nh_->declare_parameter<std::string>("topic_name");
  ASSERT_EQ(nh_->get_parameter<std::string>("topic_name", topicName_), true);

  skip_ = true;
  sub_ = nh_->create_subscription<geometry_msgs::msg::WrenchStamped>(
      topicName_, 1, std::bind(&BotaDriverSignalQuality::wrenchCallback, this, std::placeholders::_1));

  nh_->declare_parameter<int>("test_duration");
  ASSERT_EQ(nh_->get_parameter<int>("test_duration", testDuration_), true);
  time_offset = nh_->get_clock()->now() + rclcpp::Duration(testDuration_, 0);
  while (rclcpp::ok() && nh_->get_clock()->now() < time_offset && msgCount_ < maxCount_)
  {
    rclcpp::spin_some(nh_);
    if (HasFatalFailure())
    {
      FAIL() << "Fatal errors occurred.\n";
      return;
    }
  }

  EXPECT_GT(msgCount_, 0U);
  fxSamples_ = fxSamples_.array() - fxSamples_.mean();
  double sigma_fx = std::sqrt(fxSamples_.squaredNorm() / msgCount_);
  fySamples_ = fySamples_.array() - fySamples_.mean();
  double sigma_fy = std::sqrt(fySamples_.squaredNorm() / msgCount_);
  fzSamples_ = fzSamples_.array() - fzSamples_.mean();
  double sigma_fz = std::sqrt(fzSamples_.squaredNorm() / msgCount_);
  txSamples_ = txSamples_.array() - txSamples_.mean();
  double sigma_tx = std::sqrt(txSamples_.squaredNorm() / msgCount_);
  tySamples_ = tySamples_.array() - tySamples_.mean();
  double sigma_ty = std::sqrt(tySamples_.squaredNorm() / msgCount_);
  tzSamples_ = tzSamples_.array() - tzSamples_.mean();
  double sigma_tz = std::sqrt(tzSamples_.squaredNorm() / msgCount_);

  double max_sigma_fxy, max_sigma_fz, max_sigma_txy, max_sigma_tz;
  nh_->declare_parameter<double>("max_sigma_fxy");
  nh_->declare_parameter<double>("max_sigma_fz");
  nh_->declare_parameter<double>("max_sigma_txy");
  nh_->declare_parameter<double>("max_sigma_tz");
  ASSERT_EQ(nh_->get_parameter<double>("max_sigma_fxy", max_sigma_fxy), true);
  ASSERT_EQ(nh_->get_parameter<double>("max_sigma_fz", max_sigma_fz), true);
  ASSERT_EQ(nh_->get_parameter<double>("max_sigma_txy", max_sigma_txy), true);
  ASSERT_EQ(nh_->get_parameter<double>("max_sigma_tz", max_sigma_tz), true);
  // Compute standard deviation of the signal.
  ASSERT_GT(sigma_fx, 1e-7);
  ASSERT_GT(sigma_fy, 1e-7);
  ASSERT_GT(sigma_fz, 1e-7);
  ASSERT_GT(sigma_tx, 1e-7);
  ASSERT_GT(sigma_ty, 1e-7);
  ASSERT_GT(sigma_tz, 1e-7);
  ASSERT_LT(sigma_fx, max_sigma_fxy);
  ASSERT_LT(sigma_fy, max_sigma_fxy);
  ASSERT_LT(sigma_fz, max_sigma_fz);
  ASSERT_LT(sigma_tx, max_sigma_txy);
  ASSERT_LT(sigma_ty, max_sigma_txy);
  ASSERT_LT(sigma_tz, max_sigma_tz);
}

}  // namespace bota_driver_testing