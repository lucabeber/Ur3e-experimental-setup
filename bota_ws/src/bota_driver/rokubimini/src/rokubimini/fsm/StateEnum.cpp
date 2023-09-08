// rokubimini
#include "rokubimini/fsm/StateEnum.hpp"

namespace rokubimini
{
namespace fsm
{
#define ROKUBIMINI_STATE_ID_CALIBRATE (0)
#define ROKUBIMINI_STATE_ID_DEVICE_MISSING (1)
#define ROKUBIMINI_STATE_ID_ERROR (2)
#define ROKUBIMINI_STATE_ID_FATAL (3)
#define ROKUBIMINI_STATE_ID_OPERATIONAL (4)
#define ROKUBIMINI_STATE_ID_NA (5)
#define ROKUBIMINI_STATE_ID_STANDBY (6)

uint16_t stateEnumToId(const StateEnum stateEnum)
{
  if (stateEnum == StateEnum::CALIBRATE)
  {
    return ROKUBIMINI_STATE_ID_CALIBRATE;
  }
  if (stateEnum == StateEnum::DEVICE_MISSING)
  {
    return ROKUBIMINI_STATE_ID_DEVICE_MISSING;
  }
  if (stateEnum == StateEnum::ERROR)
  {
    return ROKUBIMINI_STATE_ID_ERROR;
  }
  if (stateEnum == StateEnum::FATAL)
  {
    return ROKUBIMINI_STATE_ID_FATAL;
  }
  if (stateEnum == StateEnum::OPERATIONAL)
  {
    return ROKUBIMINI_STATE_ID_OPERATIONAL;
  }
  if (stateEnum == StateEnum::STANDBY)
  {
    return ROKUBIMINI_STATE_ID_STANDBY;
  }
  return ROKUBIMINI_STATE_ID_NA;
}

StateEnum stateIdToEnum(uint16_t stateId)
{
  if (stateId == ROKUBIMINI_STATE_ID_CALIBRATE)
  {
    return StateEnum::CALIBRATE;
  }
  if (stateId == ROKUBIMINI_STATE_ID_DEVICE_MISSING)
  {
    return StateEnum::DEVICE_MISSING;
  }
  if (stateId == ROKUBIMINI_STATE_ID_ERROR)
  {
    return StateEnum::ERROR;
  }
  if (stateId == ROKUBIMINI_STATE_ID_FATAL)
  {
    return StateEnum::FATAL;
  }
  if (stateId == ROKUBIMINI_STATE_ID_OPERATIONAL)
  {
    return StateEnum::OPERATIONAL;
  }
  if (stateId == ROKUBIMINI_STATE_ID_STANDBY)
  {
    return StateEnum::STANDBY;
  }
  return StateEnum::NA;
}

std::string stateEnumToName(const StateEnum stateEnum)
{
  if (stateEnum == StateEnum::CALIBRATE)
  {
    return ROKUBIMINI_STATE_NAME_CALIBRATE;
  }
  if (stateEnum == StateEnum::DEVICE_MISSING)
  {
    return ROKUBIMINI_STATE_NAME_DEVICE_MISSING;
  }
  if (stateEnum == StateEnum::ERROR)
  {
    return ROKUBIMINI_STATE_NAME_ERROR;
  }
  if (stateEnum == StateEnum::FATAL)
  {
    return ROKUBIMINI_STATE_NAME_FATAL;
  }
  if (stateEnum == StateEnum::OPERATIONAL)
  {
    return ROKUBIMINI_STATE_NAME_OPERATIONAL;
  }
  if (stateEnum == StateEnum::STANDBY)
  {
    return ROKUBIMINI_STATE_NAME_STANDBY;
  }
  return ROKUBIMINI_STATE_NAME_NA;
}

StateEnum stateNameToEnum(const std::string& string)
{
  if (string == ROKUBIMINI_STATE_NAME_CALIBRATE)
  {
    return StateEnum::CALIBRATE;
  }
  if (string == ROKUBIMINI_STATE_NAME_DEVICE_MISSING)
  {
    return StateEnum::DEVICE_MISSING;
  }
  if (string == ROKUBIMINI_STATE_NAME_ERROR)
  {
    return StateEnum::ERROR;
  }
  if (string == ROKUBIMINI_STATE_NAME_FATAL)
  {
    return StateEnum::FATAL;
  }
  if (string == ROKUBIMINI_STATE_NAME_OPERATIONAL)
  {
    return StateEnum::OPERATIONAL;
  }
  if (string == ROKUBIMINI_STATE_NAME_STANDBY)
  {
    return StateEnum::STANDBY;
  }
  return StateEnum::NA;
}

std::ostream& operator<<(std::ostream& out, const StateEnum stateEnum)
{
  return out << stateEnumToName(stateEnum);
}

}  // namespace fsm
}  // namespace rokubimini
