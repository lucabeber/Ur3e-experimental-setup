// rokubimini
#include "rokubimini/Statusword.hpp"

namespace rokubimini
{
Statusword::Data::Data(const uint32_t data) : all_(data)
{
}

bool Statusword::Data::operator==(const Data& other)
{
  return all_ == other.all_;
}

bool Statusword::Data::operator!=(const Data& other)
{
  return !(*this == other);
}

Statusword::Statusword(const Statusword& statusword) : stamp_(statusword.getStamp()), data_(statusword.getData())
{
}

Statusword::Statusword(uint32_t data)
{
  setData(data);
}

Statusword& Statusword::operator=(const Statusword& statusword)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  stamp_ = statusword.getStamp();
  data_ = Data(statusword.getData());
  return *this;
}

bool Statusword::isEmpty() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return stamp_ == TimePoint();
}

double Statusword::getAge() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const Duration age = std::chrono::system_clock::now() - stamp_;
  return age.count();
}

Statusword::TimePoint Statusword::getStamp() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return stamp_;
}

void Statusword::setData(const uint32_t data)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  stamp_ = std::chrono::system_clock::now();
  data_ = Data(data);
}

uint32_t Statusword::getData() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.all_;
}

// fsm::StateEnum Statusword::getStateEnum() const
// {
//   std::lock_guard<std::recursive_mutex> lock(mutex_);
//   return fsm::stateIdToEnum(data_.bits_.warningIsBusy_);
// }

// void Statusword::setStateEnum(const fsm::StateEnum stateEnum)
// {
//   std::lock_guard<std::recursive_mutex> lock(mutex_);
//   stamp_ = std::chrono::system_clock::now();
//   data_.bits_.warningIsBusy_ = fsm::stateEnumToId(stateEnum);
// }

void Statusword::getMessages(std::vector<std::string>& infos, std::vector<std::string>& warnings,
                             std::vector<std::string>& errors, std::vector<std::string>& fatals) const
{
  Statusword previous_statusword;
  return getMessagesDiff(previous_statusword, infos, warnings, errors, fatals);
}

void Statusword::getMessagesDiff(Statusword& previousStatusword, std::vector<std::string>& infos,
                                 std::vector<std::string>& warnings, std::vector<std::string>& errors,
                                 std::vector<std::string>& fatals) const
{
  // Warnings.
  if (!previousStatusword.hasWarningOvertemperature() && hasWarningOvertemperature())
  {
    warnings.emplace_back("High temperature in Rokubimini Sensor");
  }
  else if (previousStatusword.hasWarningOvertemperature() && !hasWarningOvertemperature())
  {
    infos.emplace_back("Temperature in Rokubimini Sensor is normal again");
  }

  // Errors.
  if (!previousStatusword.hasErrorAdcSaturated() && hasErrorAdcSaturated())
  {
    errors.emplace_back("Force/Torque is invalid. ADC saturation");
  }
  else if (previousStatusword.hasErrorAdcSaturated() && !hasErrorAdcSaturated())
  {
    infos.emplace_back("Force/Torque is valid again. ADC is not saturated");
  }
  if (!previousStatusword.hasErrorAccSaturated() && hasErrorAccSaturated())
  {
    errors.emplace_back("Acceleration has saturated.");
  }
  else if (previousStatusword.hasErrorAccSaturated() && !hasErrorAccSaturated())
  {
    infos.emplace_back("Acceleration is not saturated anymore.");
  }
  if (!previousStatusword.hasErrorGyroSaturated() && hasErrorGyroSaturated())
  {
    errors.emplace_back("Angular rates have saturated.");
  }
  else if (previousStatusword.hasErrorGyroSaturated() && !hasErrorGyroSaturated())
  {
    infos.emplace_back("Angular rates are not saturated anymore.");
  }
  if (!previousStatusword.hasErrorAdcOutOfSync() && hasErrorAdcOutOfSync())
  {
    errors.emplace_back("Force/Torque is invalid. ADCs are not synced");
  }
  else if (previousStatusword.hasErrorAdcOutOfSync() && !hasErrorAdcOutOfSync())
  {
    infos.emplace_back("Force/Torque is valid again. ADCs are synced");
  }
  if (!previousStatusword.hasErrorSensingRangeExceeded() && hasErrorSensingRangeExceeded())
  {
    errors.emplace_back("Sensing range exceeded.");
  }
  else if (previousStatusword.hasErrorSensingRangeExceeded() && !hasErrorSensingRangeExceeded())
  {
    infos.emplace_back("Sensing range is not exceeded.");
  }

  // Fatals.
  if (!previousStatusword.hasFatalSupplyVoltage() && hasFatalSupplyVoltage())
  {
    fatals.emplace_back("Supply voltage exceeds limits.");
  }
  else if (previousStatusword.hasFatalSupplyVoltage() && !hasFatalSupplyVoltage())
  {
    infos.emplace_back("Supply voltage is normal.");
  }
}

bool Statusword::hasErrorAdcSaturated() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return static_cast<bool>(data_.bits_.errorAdcSaturated_);
}

bool Statusword::hasErrorAccSaturated() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return static_cast<bool>(data_.bits_.errorAccSaturated_);
}

bool Statusword::hasErrorGyroSaturated() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return static_cast<bool>(data_.bits_.errorGyroSaturated_);
}

bool Statusword::hasErrorAdcOutOfSync() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return static_cast<bool>(data_.bits_.errorAdcOutOfSync_);
}

bool Statusword::hasErrorSensingRangeExceeded() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return static_cast<bool>(data_.bits_.errorSensingRangeExceeded_);
}

bool Statusword::hasWarningOvertemperature() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return static_cast<bool>(data_.bits_.warningOvertemperature_);
}

bool Statusword::hasFatalSupplyVoltage() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return static_cast<bool>(data_.bits_.fatalSupplyVoltage_);
}

std::ostream& operator<<(std::ostream& os, const Statusword& statusword)
{
  for (uint32_t i = 8 * sizeof(uint32_t); i > uint32_t(0); i--)
  {
    os << ((statusword.getData() & (uint32_t(1) << (i - uint32_t(1)))) ? "1" : "0");
  }
  return os;
}

}  // namespace rokubimini
