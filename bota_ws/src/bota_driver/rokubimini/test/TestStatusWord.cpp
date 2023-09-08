/**
 * @authors     Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Tests Reading
 */

#include <gtest/gtest.h>

#include <rokubimini/Statusword.hpp>

namespace rokubimini
{
class StatusWordTest : public ::testing::Test
{
protected:
  Statusword* statusword;
  StatusWordTest()
  {
    statusword = new Statusword();
  }

  ~StatusWordTest() override
  {
    delete statusword;
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

TEST_F(StatusWordTest, CopyConstructorWorksCorrectly)
{
  auto word_expected = Statusword(*statusword);
  EXPECT_EQ(statusword->getData(), word_expected.getData());
  EXPECT_EQ(statusword->getStamp(), word_expected.getStamp());
  EXPECT_EQ(statusword->hasErrorAdcSaturated(), word_expected.hasErrorAdcSaturated());
  EXPECT_EQ(statusword->hasErrorAccSaturated(), word_expected.hasErrorAccSaturated());
  EXPECT_EQ(statusword->hasErrorGyroSaturated(), word_expected.hasErrorGyroSaturated());
  EXPECT_EQ(statusword->hasErrorAdcOutOfSync(), word_expected.hasErrorAdcOutOfSync());
  EXPECT_EQ(statusword->hasErrorSensingRangeExceeded(), word_expected.hasErrorSensingRangeExceeded());
  EXPECT_EQ(statusword->hasWarningOvertemperature(), word_expected.hasWarningOvertemperature());
  EXPECT_EQ(statusword->hasFatalSupplyVoltage(), word_expected.hasFatalSupplyVoltage());
}

TEST_F(StatusWordTest, assignmentOperatorWorksCorrectly)
{
  auto new_statusword = Statusword();

  new_statusword = *statusword;

  EXPECT_EQ(statusword->getData(), new_statusword.getData());
  EXPECT_EQ(statusword->getStamp(), new_statusword.getStamp());
  EXPECT_EQ(statusword->hasErrorAdcSaturated(), new_statusword.hasErrorAdcSaturated());
  EXPECT_EQ(statusword->hasErrorAccSaturated(), new_statusword.hasErrorAccSaturated());
  EXPECT_EQ(statusword->hasErrorGyroSaturated(), new_statusword.hasErrorGyroSaturated());
  EXPECT_EQ(statusword->hasErrorAdcOutOfSync(), new_statusword.hasErrorAdcOutOfSync());
  EXPECT_EQ(statusword->hasErrorSensingRangeExceeded(), new_statusword.hasErrorSensingRangeExceeded());
  EXPECT_EQ(statusword->hasWarningOvertemperature(), new_statusword.hasWarningOvertemperature());
  EXPECT_EQ(statusword->hasFatalSupplyVoltage(), new_statusword.hasFatalSupplyVoltage());
}

TEST_F(StatusWordTest, ExplicitConstructorWorksCorrectly)
{
  Statusword::Data data = Statusword::Data(0xff000000);
  Statusword::Data data2 = Statusword::Data(0xfe000000);
  Statusword::Data data3 = Statusword::Data();
  data3.all_ = 0xff000000;
  EXPECT_TRUE(data != data2);
  EXPECT_TRUE(data == data3);
  EXPECT_EQ(statusword->isEmpty(), true);
  Statusword newword = Statusword(data.all_);
  statusword->setData(data.all_);
  EXPECT_EQ(statusword->getData(), newword.getData());
  EXPECT_EQ(statusword->hasErrorAdcSaturated(), newword.hasErrorAdcSaturated());
  EXPECT_EQ(statusword->hasErrorAccSaturated(), newword.hasErrorAccSaturated());
  EXPECT_EQ(statusword->hasErrorGyroSaturated(), newword.hasErrorGyroSaturated());
  EXPECT_EQ(statusword->hasErrorAdcOutOfSync(), newword.hasErrorAdcOutOfSync());
  EXPECT_EQ(statusword->hasErrorSensingRangeExceeded(), newword.hasErrorSensingRangeExceeded());
  EXPECT_EQ(statusword->hasWarningOvertemperature(), newword.hasWarningOvertemperature());
  EXPECT_EQ(statusword->hasFatalSupplyVoltage(), newword.hasFatalSupplyVoltage());
}

TEST_F(StatusWordTest, ProblemMessagesWorksCorrectly)
{
  Statusword::DataBits databits{ .errorAdcSaturated_ = 0,
                                 .errorAccSaturated_ = 0,
                                 .errorGyroSaturated_ = 0,
                                 .errorAdcOutOfSync_ = 0,
                                 .errorSensingRangeExceeded_ = 0,
                                 .warningOvertemperature_ = 0,
                                 .fatalSupplyVoltage_ = 0 };
  Statusword::Data data = Statusword::Data();
  data.bits_ = databits;

  Statusword::DataBits databits2{ .errorAdcSaturated_ = 1,
                                  .errorAccSaturated_ = 1,
                                  .errorGyroSaturated_ = 1,
                                  .errorAdcOutOfSync_ = 1,
                                  .errorSensingRangeExceeded_ = 1,
                                  .warningOvertemperature_ = 1,
                                  .fatalSupplyVoltage_ = 1 };
  Statusword::Data data2 = Statusword::Data();
  data2.bits_ = databits2;

  std::vector<std::string> infos, warnings, errors, fatals;
  Statusword newword = Statusword(data.all_);
  statusword->setData(data2.all_);
  statusword->getMessagesDiff(newword, infos, warnings, errors, fatals);
  EXPECT_EQ(infos.size(), (uint32_t)0);
  ASSERT_EQ(warnings.size(), (uint32_t)1);
  ASSERT_EQ(errors.size(), (uint32_t)5);
  ASSERT_EQ(fatals.size(), (uint32_t)1);
  EXPECT_STREQ(warnings[0].c_str(), "High temperature in Rokubimini Sensor");
  EXPECT_STREQ(errors[0].c_str(), "Force/Torque is invalid. ADC saturation");
  EXPECT_STREQ(errors[1].c_str(), "Acceleration has saturated.");
  EXPECT_STREQ(errors[2].c_str(), "Angular rates have saturated.");
  EXPECT_STREQ(errors[3].c_str(), "Force/Torque is invalid. ADCs are not synced");
  EXPECT_STREQ(errors[4].c_str(), "Sensing range exceeded.");
  EXPECT_STREQ(fatals[0].c_str(), "Supply voltage exceeds limits.");
}

TEST_F(StatusWordTest, InfoMessagesWorksCorrectly)
{
  Statusword::DataBits databits{ .errorAdcSaturated_ = 0,
                                 .errorAccSaturated_ = 0,
                                 .errorGyroSaturated_ = 0,
                                 .errorAdcOutOfSync_ = 0,
                                 .errorSensingRangeExceeded_ = 0,
                                 .warningOvertemperature_ = 0,
                                 .fatalSupplyVoltage_ = 0 };
  Statusword::Data data = Statusword::Data();
  data.bits_ = databits;

  Statusword::DataBits databits2{ .errorAdcSaturated_ = 1,
                                  .errorAccSaturated_ = 1,
                                  .errorGyroSaturated_ = 1,
                                  .errorAdcOutOfSync_ = 1,
                                  .errorSensingRangeExceeded_ = 1,
                                  .warningOvertemperature_ = 1,
                                  .fatalSupplyVoltage_ = 1 };
  Statusword::Data data2 = Statusword::Data();
  data2.bits_ = databits2;
  std::vector<std::string> infos, warnings, errors, fatals;
  Statusword newword = Statusword(data2.all_);
  statusword->setData(data.all_);
  statusword->getMessagesDiff(newword, infos, warnings, errors, fatals);
  EXPECT_EQ(warnings.size(), (uint32_t)0);
  EXPECT_EQ(errors.size(), (uint32_t)0);
  EXPECT_EQ(fatals.size(), (uint32_t)0);
  ASSERT_EQ(infos.size(), (uint32_t)7);
  EXPECT_STREQ(infos[0].c_str(), "Temperature in Rokubimini Sensor is normal again");
  EXPECT_STREQ(infos[1].c_str(), "Force/Torque is valid again. ADC is not saturated");
  EXPECT_STREQ(infos[2].c_str(), "Acceleration is not saturated anymore.");
  EXPECT_STREQ(infos[3].c_str(), "Angular rates are not saturated anymore.");
  EXPECT_STREQ(infos[4].c_str(), "Force/Torque is valid again. ADCs are synced");
  EXPECT_STREQ(infos[5].c_str(), "Sensing range is not exceeded.");
  EXPECT_STREQ(infos[6].c_str(), "Supply voltage is normal.");
}
}  // namespace rokubimini
