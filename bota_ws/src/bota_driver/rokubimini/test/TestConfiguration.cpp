/**
 * @authors     Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Tests Configuration
 */

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include <rokubimini/configuration/Configuration.hpp>

namespace rokubimini
{
namespace configuration
{
class ConfigurationTest : public ::testing::Test
{
protected:
  Configuration* configuration;
  ConfigurationTest()
  {
    configuration = new Configuration();
  }

  ~ConfigurationTest() override
  {
    delete configuration;
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
};

TEST_F(ConfigurationTest, forceTorqueFilterWorksCorrectly)
{
  configuration->setForceTorqueFilter(ForceTorqueFilter(64, true, true, false));
  ForceTorqueFilter filter = configuration->getForceTorqueFilter();
  EXPECT_DOUBLE_EQ(filter.getSincFilterSize(), 64);
  EXPECT_EQ(filter.getChopEnable(), true);
  EXPECT_EQ(filter.getSkipEnable(), true);
  EXPECT_EQ(filter.getFastEnable(), false);
}

TEST_F(ConfigurationTest, forceTorqueOffsetWorksCorrectly)
{
  configuration->setForceTorqueOffset(Eigen::Matrix<double, 6, 1>::Ones());
  Eigen::Matrix<double, 6, 1> matrix = configuration->getForceTorqueOffset();
  EXPECT_DOUBLE_EQ(matrix(0, 0), 1);
  EXPECT_DOUBLE_EQ(matrix(1, 0), 1);
  EXPECT_DOUBLE_EQ(matrix(2, 0), 1);
  EXPECT_DOUBLE_EQ(matrix(3, 0), 1);
  EXPECT_DOUBLE_EQ(matrix(4, 0), 1);
  EXPECT_DOUBLE_EQ(matrix(5, 0), 1);
}

TEST_F(ConfigurationTest, useCustomCalibrationWorksCorrectly)
{
  configuration->setUseCustomCalibration(true);
  auto use_custom_calibration = configuration->getUseCustomCalibration();
  EXPECT_EQ(use_custom_calibration, true);
}

TEST_F(ConfigurationTest, setReadingToNanOnDisconnectWorksCorrectly)
{
  configuration->setSetReadingToNanOnDisconnect(true);
  auto set_reading_to_nan_on_disconnect = configuration->getSetReadingToNanOnDisconnect();
  EXPECT_EQ(set_reading_to_nan_on_disconnect, true);
}

TEST_F(ConfigurationTest, sensorConfigurationWorksCorrectly)
{
  configuration->setSensorConfiguration(SensorConfiguration(true, true, false, false, false, true));
  auto sensor_configuration = configuration->getSensorConfiguration();
  EXPECT_EQ(sensor_configuration.getCalibrationMatrixActive(), true);
  EXPECT_EQ(sensor_configuration.getTemperatureCompensationActive(), true);
  EXPECT_EQ(sensor_configuration.getImuActive(), false);
  EXPECT_EQ(sensor_configuration.getCoordinateSystemConfigurationActive(), false);
  EXPECT_EQ(sensor_configuration.getInertiaCompensationActive(), false);
  EXPECT_EQ(sensor_configuration.getOrientationEstimationActive(), true);
}

TEST_F(ConfigurationTest, sensorCalibrationWorksCorrectly)
{
  Eigen::Matrix<double, 6, 6> calibration_matrix = Eigen::Matrix<double, 6, 6>::Identity();
  auto sensor_calibration = rokubimini::calibration::SensorCalibration();
  sensor_calibration.setCalibrationMatrix(calibration_matrix);
  configuration->setSensorCalibration(sensor_calibration);
  auto calibration_expected = configuration->getSensorCalibration();
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(0, 0), 1);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(0, 1), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(0, 2), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(0, 3), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(0, 4), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(0, 5), 0);

  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(1, 0), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(1, 1), 1);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(1, 2), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(1, 3), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(1, 4), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(1, 5), 0);

  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(2, 0), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(2, 1), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(2, 2), 1);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(2, 3), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(2, 4), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(2, 5), 0);

  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(3, 0), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(3, 1), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(3, 2), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(3, 3), 1);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(3, 4), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(3, 5), 0);

  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(4, 0), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(4, 1), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(4, 2), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(4, 3), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(4, 4), 1);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(4, 5), 0);

  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(5, 0), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(5, 1), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(5, 2), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(5, 3), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(5, 4), 0);
  EXPECT_DOUBLE_EQ(calibration_expected.getCalibrationMatrix()(5, 5), 1);
}

TEST_F(ConfigurationTest, imuAccelerationFilterWorksCorrectly)
{
  configuration->setImuAccelerationFilter((unsigned int)128);
  auto imu_acceleration_filter = configuration->getImuAccelerationFilter();
  EXPECT_EQ(imu_acceleration_filter, (unsigned int)128);
}

TEST_F(ConfigurationTest, imuAngularRateFilterWorksCorrectly)
{
  configuration->setImuAngularRateFilter((unsigned int)128);
  auto imu_angular_rate_filter = configuration->getImuAngularRateFilter();
  EXPECT_EQ(imu_angular_rate_filter, (unsigned int)128);
}

TEST_F(ConfigurationTest, imuAccelerationRangeWorksCorrectly)
{
  configuration->setImuAccelerationRange((uint8_t)15);
  auto imu_acceleration_range = configuration->getImuAccelerationRange();
  EXPECT_EQ(imu_acceleration_range, 15);
}

TEST_F(ConfigurationTest, imuAngularRateRangeWorksCorrectly)
{
  configuration->setImuAngularRateRange((uint8_t)15);
  auto imu_angular_rate_range = configuration->getImuAngularRateRange();
  EXPECT_EQ(imu_angular_rate_range, 15);
}

}  // namespace configuration
}  // namespace rokubimini
