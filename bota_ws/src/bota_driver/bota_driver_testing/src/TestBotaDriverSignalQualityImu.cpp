/**
 * @authors     Martin Wermelinger, Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Tests Command
 */

#include <gtest/gtest.h>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>

#define G_TO_METERS_PER_SECOND_SQUARED 9.80665
using namespace Eigen;
namespace bota_driver_testing
{
class BotaDriverSignalQualityImu : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  std::uint32_t msgCount_;
  std::uint32_t maxCount_;
  std::string topicName_;
  VectorXd linAccXSamples_;
  VectorXd linAccYSamples_;
  VectorXd linAccZSamples_;
  VectorXd angVelXSamples_;
  VectorXd angVelYSamples_;
  VectorXd angVelZSamples_;
  int testDuration_;

  BotaDriverSignalQualityImu() : msgCount_(0), maxCount_(1000)
  {
    nh_ = std::make_shared<rclcpp::Node>("bota_driver_test_signal_quality_imu");
  }

  ~BotaDriverSignalQualityImu() override
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
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (msg->header.stamp.sec <= 0)
      return;
    std::uint32_t lpos = msgCount_;
    msgCount_++;
    linAccXSamples_.conservativeResize(msgCount_);
    linAccYSamples_.conservativeResize(msgCount_);
    linAccZSamples_.conservativeResize(msgCount_);
    angVelXSamples_.conservativeResize(msgCount_);
    angVelYSamples_.conservativeResize(msgCount_);
    angVelZSamples_.conservativeResize(msgCount_);

    linAccXSamples_(lpos) = msg->linear_acceleration.x;
    linAccYSamples_(lpos) = msg->linear_acceleration.y;
    linAccZSamples_(lpos) = msg->linear_acceleration.z;
    angVelXSamples_(lpos) = msg->angular_velocity.x;
    angVelYSamples_(lpos) = msg->angular_velocity.y;
    angVelZSamples_(lpos) = msg->angular_velocity.z;
  }
};

TEST_F(BotaDriverSignalQualityImu, SignalQuality)
{
  SCOPED_TRACE("SignalQualityImu");
  rclcpp::Time time_offset;
  nh_->declare_parameter<std::string>("topic_name");
  ASSERT_EQ(nh_->get_parameter<std::string>("topic_name", topicName_), true);

  sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>(
      topicName_, 10, std::bind(&BotaDriverSignalQualityImu::imuCallback, this, std::placeholders::_1));

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
  double linacc_xsamples_mean, linacc_ysamples_mean, linacc_zsamples_mean, angvel_xsamples_mean, angvel_ysamples_mean,
      angvel_zsamples_mean, linacc_mean, angvel_mean;
  linacc_xsamples_mean = linAccXSamples_.mean();
  linacc_ysamples_mean = linAccYSamples_.mean();
  linacc_zsamples_mean = linAccZSamples_.mean();
  // compute the mean of linear acceleration.
  linacc_mean = std::sqrt(std::pow(linacc_xsamples_mean, 2) + std::pow(linacc_ysamples_mean, 2) +
                          std::pow(linacc_zsamples_mean, 2));
  angvel_xsamples_mean = angVelXSamples_.mean();
  angvel_ysamples_mean = angVelYSamples_.mean();
  angvel_zsamples_mean = angVelZSamples_.mean();
  // compute the mean of the angular velocities.
  angvel_mean = std::sqrt(std::pow(angvel_xsamples_mean, 2) + std::pow(angvel_ysamples_mean, 2) +
                          std::pow(angvel_zsamples_mean, 2));
  linAccXSamples_ = linAccXSamples_.array() - linAccXSamples_.mean();
  double sigma_ax = std::sqrt(linAccXSamples_.squaredNorm() / msgCount_);
  linAccYSamples_ = linAccYSamples_.array() - linAccYSamples_.mean();
  double sigma_ay = std::sqrt(linAccYSamples_.squaredNorm() / msgCount_);
  linAccZSamples_ = linAccZSamples_.array() - linAccZSamples_.mean();
  double sigma_az = std::sqrt(linAccZSamples_.squaredNorm() / msgCount_);
  angVelXSamples_ = angVelXSamples_.array() - angVelXSamples_.mean();
  double sigma_vx = std::sqrt(angVelXSamples_.squaredNorm() / msgCount_);
  angVelYSamples_ = angVelYSamples_.array() - angVelYSamples_.mean();
  double sigma_vy = std::sqrt(angVelYSamples_.squaredNorm() / msgCount_);
  angVelZSamples_ = angVelZSamples_.array() - angVelZSamples_.mean();
  double sigma_vz = std::sqrt(angVelZSamples_.squaredNorm() / msgCount_);

  double max_sigma_axy, max_sigma_az, max_sigma_vxy, max_sigma_vz, mean_dev_lin_acc, mean_dev_ang_vel;
  nh_->declare_parameter<double>("max_sigma_axy");
  nh_->declare_parameter<double>("max_sigma_az");
  nh_->declare_parameter<double>("max_sigma_vxy");
  nh_->declare_parameter<double>("max_sigma_vz");
  nh_->declare_parameter<double>("mean_dev_lin_acc");
  nh_->declare_parameter<double>("mean_dev_ang_vel");
  ASSERT_EQ(nh_->get_parameter<double>("max_sigma_axy", max_sigma_axy), true);
  ASSERT_EQ(nh_->get_parameter<double>("max_sigma_az", max_sigma_az), true);
  ASSERT_EQ(nh_->get_parameter<double>("max_sigma_vxy", max_sigma_vxy), true);
  ASSERT_EQ(nh_->get_parameter<double>("max_sigma_vz", max_sigma_vz), true);
  ASSERT_EQ(nh_->get_parameter<double>("mean_dev_lin_acc", mean_dev_lin_acc), true);
  ASSERT_EQ(nh_->get_parameter<double>("mean_dev_ang_vel", mean_dev_ang_vel), true);

  // Assert mean linear acceleration is close to 1g and mean angular velocity close to 0.
  ASSERT_NEAR(linacc_mean, 1.0 * G_TO_METERS_PER_SECOND_SQUARED, mean_dev_lin_acc);
  ASSERT_NEAR(angvel_mean, 0.0, mean_dev_ang_vel);

  // Compute standard deviation of the signal.
  ASSERT_GT(sigma_ax, 1e-7);
  ASSERT_GT(sigma_ay, 1e-7);
  ASSERT_GT(sigma_az, 1e-7);
  ASSERT_GT(sigma_vx, 1e-7);
  ASSERT_GT(sigma_vy, 1e-7);
  ASSERT_GT(sigma_vz, 1e-7);
  ASSERT_LT(sigma_ax, max_sigma_axy);
  ASSERT_LT(sigma_ay, max_sigma_axy);
  ASSERT_LT(sigma_az, max_sigma_az);
  ASSERT_LT(sigma_vx, max_sigma_vxy);
  ASSERT_LT(sigma_vy, max_sigma_vxy);
  ASSERT_LT(sigma_vz, max_sigma_vz);
}

}  // namespace bota_driver_testing