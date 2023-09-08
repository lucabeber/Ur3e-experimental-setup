// soem_interface
#include <rokubimini_ethercat/soem_interface/EthercatBusBase.hpp>
#include <rokubimini_ethercat/soem_interface/EthercatSlaveBase.hpp>

namespace rokubimini
{
namespace soem_interface
{
EthercatSlaveBase::EthercatSlaveBase(EthercatBusBase* bus, const uint32_t address) : bus_(bus), address_(address)
{
}

// This definition must not be in the header, because of the forward declaration of EthercatBus
template <typename Value>
bool EthercatSlaveBase::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     const Value value)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return bus_->sendSdoWrite(address_, index, subindex, completeAccess, value);
}

// This definition must not be in the header, because of the forward declaration of EthercatBus
template <typename Value>
bool EthercatSlaveBase::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                    Value& value)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return bus_->sendSdoRead(address_, index, subindex, completeAccess, value);
}

// These definitions must not be in the header, because of the forward declaration of EthercatBus
template bool EthercatSlaveBase::sendSdoWrite<int8_t>(const uint16_t index, const uint8_t subindex,
                                                      const bool completeAccess, const int8_t value);
template bool EthercatSlaveBase::sendSdoWrite<int16_t>(const uint16_t index, const uint8_t subindex,
                                                       const bool completeAccess, const int16_t value);
template bool EthercatSlaveBase::sendSdoWrite<int32_t>(const uint16_t index, const uint8_t subindex,
                                                       const bool completeAccess, const int32_t value);
template bool EthercatSlaveBase::sendSdoWrite<int64_t>(const uint16_t index, const uint8_t subindex,
                                                       const bool completeAccess, const int64_t value);
template bool EthercatSlaveBase::sendSdoWrite<uint8_t>(const uint16_t index, const uint8_t subindex,
                                                       const bool completeAccess, const uint8_t value);
template bool EthercatSlaveBase::sendSdoWrite<uint16_t>(const uint16_t index, const uint8_t subindex,
                                                        const bool completeAccess, const uint16_t value);
template bool EthercatSlaveBase::sendSdoWrite<uint32_t>(const uint16_t index, const uint8_t subindex,
                                                        const bool completeAccess, const uint32_t value);
template bool EthercatSlaveBase::sendSdoWrite<uint64_t>(const uint16_t index, const uint8_t subindex,
                                                        const bool completeAccess, const uint64_t value);
template bool EthercatSlaveBase::sendSdoWrite<float>(const uint16_t index, const uint8_t subindex,
                                                     const bool completeAccess, const float value);
template bool EthercatSlaveBase::sendSdoWrite<double>(const uint16_t index, const uint8_t subindex,
                                                      const bool completeAccess, const double value);

template bool EthercatSlaveBase::sendSdoRead<int8_t>(const uint16_t index, const uint8_t subindex,
                                                     const bool completeAccess, int8_t& value);
template bool EthercatSlaveBase::sendSdoRead<int16_t>(const uint16_t index, const uint8_t subindex,
                                                      const bool completeAccess, int16_t& value);
template bool EthercatSlaveBase::sendSdoRead<int32_t>(const uint16_t index, const uint8_t subindex,
                                                      const bool completeAccess, int32_t& value);
template bool EthercatSlaveBase::sendSdoRead<int64_t>(const uint16_t index, const uint8_t subindex,
                                                      const bool completeAccess, int64_t& value);
template bool EthercatSlaveBase::sendSdoRead<uint8_t>(const uint16_t index, const uint8_t subindex,
                                                      const bool completeAccess, uint8_t& value);
template bool EthercatSlaveBase::sendSdoRead<uint16_t>(const uint16_t index, const uint8_t subindex,
                                                       const bool completeAccess, uint16_t& value);
template bool EthercatSlaveBase::sendSdoRead<uint32_t>(const uint16_t index, const uint8_t subindex,
                                                       const bool completeAccess, uint32_t& value);
template bool EthercatSlaveBase::sendSdoRead<uint64_t>(const uint16_t index, const uint8_t subindex,
                                                       const bool completeAccess, uint64_t& value);
template bool EthercatSlaveBase::sendSdoRead<float>(const uint16_t index, const uint8_t subindex,
                                                    const bool completeAccess, float& value);
template bool EthercatSlaveBase::sendSdoRead<double>(const uint16_t index, const uint8_t subindex,
                                                     const bool completeAccess, double& value);

bool EthercatSlaveBase::sendSdoReadGeneric(const std::string& indexString, const std::string& subindexString,
                                           const std::string& valueTypeString, std::string& valueString)
{
  printWarnNotImplemented();
  return false;
}

bool EthercatSlaveBase::sendSdoWriteGeneric(const std::string& indexString, const std::string& subindexString,
                                            const std::string& valueTypeString, const std::string& valueString)
{
  printWarnNotImplemented();
  return false;
}

}  // namespace soem_interface
}  // namespace rokubimini