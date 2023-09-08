/**
 * @authors     Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Tests Reading
 */

#include <gtest/gtest.h>

#include <rokubimini/fsm/StateEnum.hpp>

namespace rokubimini
{
namespace fsm
{
TEST(StateEnumTest, StateEnumToIdWorksCorrectly)
{
  EXPECT_EQ(ROKUBIMINI_STATE_ID_CALIBRATE, stateEnumToId(StateEnum::CALIBRATE));
  EXPECT_EQ(ROKUBIMINI_STATE_ID_DEVICE_MISSING, stateEnumToId(StateEnum::DEVICE_MISSING));
  EXPECT_EQ(ROKUBIMINI_STATE_ID_ERROR, stateEnumToId(StateEnum::ERROR));
  EXPECT_EQ(ROKUBIMINI_STATE_ID_FATAL, stateEnumToId(StateEnum::FATAL));
  EXPECT_EQ(ROKUBIMINI_STATE_ID_OPERATIONAL, stateEnumToId(StateEnum::OPERATIONAL));
  EXPECT_EQ(ROKUBIMINI_STATE_ID_STANDBY, stateEnumToId(StateEnum::STANDBY));
  EXPECT_EQ(ROKUBIMINI_STATE_ID_NA, stateEnumToId(StateEnum::NA));
}

TEST(StateEnumTest, StateIdToEnumWorksCorrectly)
{
  EXPECT_EQ(StateEnum::CALIBRATE, stateIdToEnum(ROKUBIMINI_STATE_ID_CALIBRATE));
  EXPECT_EQ(StateEnum::DEVICE_MISSING, stateIdToEnum(ROKUBIMINI_STATE_ID_DEVICE_MISSING));
  EXPECT_EQ(StateEnum::ERROR, stateIdToEnum(ROKUBIMINI_STATE_ID_ERROR));
  EXPECT_EQ(StateEnum::FATAL, stateIdToEnum(ROKUBIMINI_STATE_ID_FATAL));
  EXPECT_EQ(StateEnum::OPERATIONAL, stateIdToEnum(ROKUBIMINI_STATE_ID_OPERATIONAL));
  EXPECT_EQ(StateEnum::STANDBY, stateIdToEnum(ROKUBIMINI_STATE_ID_STANDBY));
  EXPECT_EQ(StateEnum::NA, stateIdToEnum(15));
}

TEST(StateEnumTest, StateEnumToNameWorksCorrectly)
{
  EXPECT_STREQ(ROKUBIMINI_STATE_NAME_CALIBRATE, stateEnumToName(StateEnum::CALIBRATE).c_str());
  EXPECT_STREQ(ROKUBIMINI_STATE_NAME_DEVICE_MISSING, stateEnumToName(StateEnum::DEVICE_MISSING).c_str());
  EXPECT_STREQ(ROKUBIMINI_STATE_NAME_ERROR, stateEnumToName(StateEnum::ERROR).c_str());
  EXPECT_STREQ(ROKUBIMINI_STATE_NAME_FATAL, stateEnumToName(StateEnum::FATAL).c_str());
  EXPECT_STREQ(ROKUBIMINI_STATE_NAME_OPERATIONAL, stateEnumToName(StateEnum::OPERATIONAL).c_str());
  EXPECT_STREQ(ROKUBIMINI_STATE_NAME_STANDBY, stateEnumToName(StateEnum::STANDBY).c_str());
  EXPECT_STREQ(ROKUBIMINI_STATE_NAME_NA, stateEnumToName(StateEnum::NA).c_str());
}
TEST(StateEnumTest, StateNameToEnumWorksCorrectly)
{
  EXPECT_EQ(StateEnum::CALIBRATE, stateNameToEnum(ROKUBIMINI_STATE_NAME_CALIBRATE));
  EXPECT_EQ(StateEnum::DEVICE_MISSING, stateNameToEnum(ROKUBIMINI_STATE_NAME_DEVICE_MISSING));
  EXPECT_EQ(StateEnum::ERROR, stateNameToEnum(ROKUBIMINI_STATE_NAME_ERROR));
  EXPECT_EQ(StateEnum::FATAL, stateNameToEnum(ROKUBIMINI_STATE_NAME_FATAL));
  EXPECT_EQ(StateEnum::OPERATIONAL, stateNameToEnum(ROKUBIMINI_STATE_NAME_OPERATIONAL));
  EXPECT_EQ(StateEnum::STANDBY, stateNameToEnum(ROKUBIMINI_STATE_NAME_STANDBY));
  EXPECT_EQ(StateEnum::NA, stateNameToEnum("ABBAB"));
}

}  // namespace fsm
}  // namespace rokubimini
