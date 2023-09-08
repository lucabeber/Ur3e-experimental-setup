/**
 * @authors     Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Tests Reading
 */

#include <gtest/gtest.h>

#include <rokubimini/Reading.hpp>

namespace rokubimini
{
class ReadingTest : public ::testing::Test
{
protected:
  Reading* reading;
  ReadingTest()
  {
    reading = new Reading();
  }

  ~ReadingTest() override
  {
    delete reading;
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

// TEST_F(ReadingTest, ImuWorksCorrectly)
// {
//   ImuType imu;
//   reading->setImu(imu);
//   auto imu_expected = reading->getImu();
//   EXPECT_EQ(imu, imu_expected);
// }

// TEST_F(ReadingTest, WrenchWorksCorrectly)
// {
//   WrenchType wrench;
//   reading->setWrench(wrench);
//   auto wrench_expected = reading->getWrench();
//   EXPECT_EQ(wrench, wrench_expected);
// }

// TEST_F(ReadingTest, ExternalImuWorksCorrectly)
// {
//   ImuType imu;
//   reading->setExternalImu(imu);
//   auto imu_expected = reading->getExternalImu();
//   EXPECT_EQ(imu, imu_expected);
// }

TEST_F(ReadingTest, StatusWordWorksCorrectly)
{
  Statusword word = Statusword();
  reading->setStatusword(word);
  auto word_expected = reading->getStatusword();
  EXPECT_EQ(word.getData(), word_expected.getData());
  EXPECT_EQ(word.getStamp(), word_expected.getStamp());
  EXPECT_EQ(word.hasErrorAdcSaturated(), word_expected.hasErrorAdcSaturated());
  EXPECT_EQ(word.hasErrorAccSaturated(), word_expected.hasErrorAccSaturated());
  EXPECT_EQ(word.hasErrorGyroSaturated(), word_expected.hasErrorGyroSaturated());
  EXPECT_EQ(word.hasErrorAdcOutOfSync(), word_expected.hasErrorAdcOutOfSync());
  EXPECT_EQ(word.hasErrorSensingRangeExceeded(), word_expected.hasErrorSensingRangeExceeded());
  EXPECT_EQ(word.hasWarningOvertemperature(), word_expected.hasWarningOvertemperature());
  EXPECT_EQ(word.hasFatalSupplyVoltage(), word_expected.hasFatalSupplyVoltage());
}

TEST_F(ReadingTest, ForceTorqueSaturatedWorksCorrectly)
{
  reading->setForceTorqueSaturated(true);
  auto force_torque_saturated = reading->isForceTorqueSaturated();
  EXPECT_EQ(force_torque_saturated, true);
}

TEST_F(ReadingTest, TemperatureWorksCorrectly)
{
  auto temperature = sensor_msgs::msg::Temperature();
  temperature.temperature = 15.46329113;
  reading->setTemperature(temperature);
  auto temp = reading->getTemperature();
  EXPECT_FLOAT_EQ(temp.temperature, 15.46329113);
}

}  // namespace rokubimini
