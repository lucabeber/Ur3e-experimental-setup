#include <rokubimini_ethercat/soem_interface/EthercatBusBase.hpp>
#include <thread>
namespace rokubimini
{
namespace soem_interface
{
EthercatBusBase::EthercatBusBase(const std::string& name) : name_(name), wkc_(0), isRunning_{ true }
{
  // Initialize all SOEM context data pointers that are not used with null.
  ecatContext_.elist->head = 0;
  ecatContext_.elist->tail = 0;
  ecatContext_.port->stack.sock = nullptr;
  ecatContext_.port->stack.txbuf = nullptr;
  ecatContext_.port->stack.txbuflength = nullptr;
  ecatContext_.port->stack.tempbuf = nullptr;
  ecatContext_.port->stack.rxbuf = nullptr;
  ecatContext_.port->stack.rxbufstat = nullptr;
  ecatContext_.port->stack.rxsa = nullptr;
  ecatContext_.port->redport = nullptr;
  //  ecatContext_.idxstack->data = nullptr; // This does not compile since SOEM uses a fixed size array of void
  //  pointers.
  ecatContext_.FOEhook = nullptr;
}

bool EthercatBusBase::busIsAvailable(const std::string& name)
{
  ec_adaptert* adapter = ec_find_adapters();
  while (adapter != nullptr)
  {
    if (name == std::string(adapter->name))
    {
      return true;
    }
    adapter = adapter->next;
  }
  return false;
}

void EthercatBusBase::printAvailableBusses()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("EtherCAT bus"), "Available adapters:");
  ec_adaptert* adapter = ec_find_adapters();
  while (adapter != nullptr)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("EtherCAT bus"),
                       "- Name: '" << adapter->name << "', description: '" << adapter->desc << "'");
    adapter = adapter->next;
  }
}

bool EthercatBusBase::busIsAvailable() const
{
  return busIsAvailable(name_);
}

int EthercatBusBase::getNumberOfSlaves() const
{
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  return *ecatContext_.slavecount;
}

bool EthercatBusBase::addSlave(const EthercatSlaveBasePtr& slave)
{
  for (const auto& existing_slave : slaves_)
  {
    if (slave->getAddress() == existing_slave->getAddress())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"),
                          "[" << getName() << "] "
                              << "Slave '" << existing_slave->getName() << "' and slave '" << slave->getName()
                              << "' have identical addresses (" << slave->getAddress() << ").");
      return false;
    }
  }

  slaves_.push_back(slave);
  return true;
}

bool EthercatBusBase::startup(const bool sizeCheck)
{
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  /*
   * Followed by start of the application we need to set up the NIC to be used as
   * EtherCAT Ethernet interface. In a simple setup we call ec_init(ifname) and if
   * SOEM comes with support for cable redundancy we call ec_init_redundant that
   * will open a second port as backup. You can send NULL as ifname if you have a
   * dedicated NIC selected in the nicdrv.c. It returns >0 if succeeded.
   */
  if (!busIsAvailable())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"), "[" << getName() << "] "
                                                                << "Bus is not available.");
    printAvailableBusses();
    return false;
  }
  if (ecx_init(&ecatContext_, name_.c_str()) <= 0)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"), "[" << getName() << "] "
                                                                << "No socket connection. Execute as root.");
    return false;
  }

  // Initialize SOEM.
  // Note: ecx_config_init(..) requests the slaves to go to PRE-OP.
  for (unsigned int retry = 0; retry <= ecatConfigMaxRetries_; retry++)
  {
    if (ecx_config_init(&ecatContext_, FALSE) > 0)
    {
      // Successful initialization.
      break;
    }
    else if (retry == ecatConfigMaxRetries_)
    {
      // Too many failed attempts.
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"), "[" << getName() << "] "
                                                                  << "No slaves have been found.");
      return false;
    }
    // Sleep and retry.
    std::this_thread::sleep_for(std::chrono::duration<double>(ecatConfigRetrySleep_));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("EtherCAT bus"), "[" << getName() << "] "
                                                               << "No slaves have been found, retrying " << retry + 1
                                                               << "/" << ecatConfigMaxRetries_ << " ...");
  }

  // Check if the given slave addresses are valid.
  bool slave_addresses_are_ok = true;
  for (const auto& slave : slaves_)
  {
    auto address = static_cast<int>(slave->getAddress());
    if (address == 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"), "[" << getName() << "] "
                                                                  << "Slave '" << slave->getName()
                                                                  << "': Invalid address " << address << ".");
      slave_addresses_are_ok = false;
    }
    if (address > getNumberOfSlaves())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"),
                          "[" << getName() << "] "
                              << "Slave '" << slave->getName() << "': Invalid address " << address << ", "
                              << "only " << getNumberOfSlaves() << " slave(s) found.");
      slave_addresses_are_ok = false;
    }
  }
  if (!slave_addresses_are_ok)
  {
    return false;
  }

  // Check if the given slave names are valid.
  for (const auto& slave : slaves_)
  {
    auto product_name = slave->getProductName();
    if (!isCorrectDeviceName(slave->getAddress(), product_name))
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("EtherCAT bus"),
                         "[" << getName() << "] "
                             << "Slave '" << slave->getName() << "': Invalid product name '" << product_name
                             << "' given, didn't match the actual product name of the device: '"
                             << getDeviceName(slave->getAddress()) << "'");
      slave->setProductName(getDeviceName(slave->getAddress()));
    }
  }
  // Disable symmetrical transfers.
  ecatContext_.grouplist[0].blockLRW = 1;

  // Initialize the communication interfaces of all slaves.
  for (auto& slave : slaves_)
  {
    if (!slave->startup())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"), "[" << getName() << "] "
                                                                  << "Slave '" << slave->getName()
                                                                  << "' was not initialized successfully.");
      return false;
    }
  }
  // wait for the slaves to reach PREOP state
  waitForState(EC_STATE_PRE_OP);
  // Print how many slaves have been detected.
  RCLCPP_INFO_STREAM(rclcpp::get_logger("EtherCAT bus"), "[" << getName() << "] "
                                                             << "The following " << getNumberOfSlaves()
                                                             << " slaves have been found and configured:");
  unsigned int serial_number;
  for (auto& slave : slaves_)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("EtherCAT bus"), "[" << getName() << "] "
                                                               << "Address: " << slave->getAddress());
    RCLCPP_INFO_STREAM(rclcpp::get_logger("EtherCAT bus"), "[" << getName() << "] "
                                                               << "Name: '" << slave->getProductName() << "'");
    slave->getSerialNumber(serial_number);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("EtherCAT bus"), "[" << getName() << "] "
                                                               << "S/N: " << serial_number);
    slave->setSerialNumber(serial_number);
  }
  // Set up the communication IO mapping.
  // Note: ecx_config_map_group(..) requests the slaves to go to SAFE-OP.
  ecx_config_map_group(&ecatContext_, &ioMap_, 0);

  // Check if the size of the IO mapping fits our slaves.
  bool io_map_is_ok = true;
  // do this check only if 'sizeCheck' is true
  if (sizeCheck)
  {
    for (const auto& slave : slaves_)
    {
      const EthercatSlaveBase::PdoInfo pdo_info = slave->getCurrentPdoInfo();
      if (pdo_info.rxPdoSize_ != ecatContext_.slavelist[slave->getAddress()].Obytes)
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"),
                            "[" << getName() << "] "
                                << "RxPDO size mismatch: The slave '" << slave->getName() << "' expects a size of "
                                << pdo_info.rxPdoSize_ << " bytes but the slave found at its address "
                                << slave->getAddress() << " requests "
                                << ecatContext_.slavelist[slave->getAddress()].Obytes << " bytes).");
        io_map_is_ok = false;
      }
      if (pdo_info.txPdoSize_ != ecatContext_.slavelist[slave->getAddress()].Ibytes)
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"),
                            "[" << getName() << "] "
                                << "TxPDO size mismatch: The slave '" << slave->getName() << "' expects a size of "
                                << pdo_info.txPdoSize_ << " bytes but the slave found at its address "
                                << slave->getAddress() << " requests "
                                << ecatContext_.slavelist[slave->getAddress()].Ibytes << " bytes).");
        io_map_is_ok = false;
      }
    }
  }
  if (!io_map_is_ok)
  {
    return false;
  }

  // Initialize the memory with zeroes.
  for (int slave = 1; slave <= getNumberOfSlaves(); slave++)
  {
    memset(ecatContext_.slavelist[slave].inputs, 0, ecatContext_.slavelist[slave].Ibytes);
    memset(ecatContext_.slavelist[slave].outputs, 0, ecatContext_.slavelist[slave].Obytes);
  }

  return true;
}

void EthercatBusBase::updateRead()
{
  if (!sentProcessData_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("EtherCAT bus"), "[" << getName() << "] "
                                                                << "No process data to read.");
    return;
  }

  //! Receive the EtherCAT data.
  updateReadStamp_ = clock_.now();
  {
    std::lock_guard<std::recursive_mutex> guard(contextMutex_);
    wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
  }
  sentProcessData_ = false;

  //! Check the working counter.
  if (!workingCounterIsOk())
  {
    RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("EtherCAT bus"), clock_, 1.0,
                                "[" << getName() << "] "
                                    << "Update Read:" << this);
    RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("EtherCAT bus"), clock_, 1.0,
                                "[" << getName() << "] "
                                    << "Working counter is too low: " << wkc_.load() << " < "
                                    << getExpectedWorkingCounter());
    return;
  }

  //! Each slave attached to this bus reads its data to the buffer.
  for (auto& slave : slaves_)
  {
    slave->updateRead();
  }
}

void EthercatBusBase::updateWrite()
{
  if (sentProcessData_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("EtherCAT bus"),
                        "[" << getName() << "] "
                            << "Sending new process data without reading the previous one.");
  }

  //! Each slave attached to this bus write its data to the buffer.
  for (auto& slave : slaves_)
  {
    slave->updateWrite();
  }

  //! Send the EtherCAT data.
  updateWriteStamp_ = clock_.now();
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  ecx_send_processdata(&ecatContext_);
  sentProcessData_ = true;
}

void EthercatBusBase::shutdown()
{
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  // Set the slaves to state Init.
  if (getNumberOfSlaves() > 0)
  {
    setState(EC_STATE_INIT);
    waitForState(EC_STATE_INIT);
  }

  // Close the port.
  if (ecatContext_.port != nullptr)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("EtherCAT bus"), "[" << getName() << "] "
                                                               << "Closing socket ...");
    ecx_close(&ecatContext_);
    // Sleep to make sure the socket is closed, because ecx_close is non-blocking.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void EthercatBusBase::setState(const uint16_t state, const uint16_t slave)
{
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  ecatContext_.slavelist[slave].state = state;
  ecx_writestate(&ecatContext_, slave);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("EtherCAT bus"), "Slave " << slave << ": State " << state << " has been set.");
}

bool EthercatBusBase::waitForState(const uint16_t state, const uint16_t slave, const unsigned int maxRetries,
                                   const double retrySleep)
{
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  for (unsigned int retry = 0; retry <= maxRetries; retry++)
  {
    if (ecx_statecheck(&ecatContext_, slave, state, static_cast<int>(1e6 * retrySleep)) == state)
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("EtherCAT bus"),
                          "Slave " << slave << ": State " << state << " has been reached.");
      return true;
    }
    // TODO: Do this for all states?
    ecx_send_processdata(&ecatContext_);
    wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
  }

  RCLCPP_WARN_STREAM(rclcpp::get_logger("EtherCAT bus"),
                     "Slave " << slave << ": State " << state << " has not been reached.");
  return false;
}

int EthercatBusBase::getExpectedWorkingCounter(const uint16_t slave) const
{
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  return ecatContext_.grouplist[slave].outputsWKC * 2 + ecatContext_.grouplist[slave].inputsWKC;
}

std::string EthercatBusBase::getErrorString(ec_errort error)
{
  std::stringstream stream;
  stream << "Time: " << (static_cast<double>(error.Time.sec) + (static_cast<double>(error.Time.usec) / 1000000.0));

  switch (error.Etype)
  {
    case EC_ERR_TYPE_SDO_ERROR:
      stream << " SDO slave: " << error.Slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex
             << error.Index << "." << std::setfill('0') << std::setw(2) << std::hex
             << static_cast<uint16_t>(error.SubIdx) << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex
             << static_cast<unsigned>(error.AbortCode) << " " << ec_sdoerror2string(error.AbortCode);
      break;
    case EC_ERR_TYPE_EMERGENCY:
      stream << " EMERGENCY slave: " << error.Slave << " error: 0x" << std::setfill('0') << std::setw(4) << std::hex
             << error.ErrorCode;
      break;
    case EC_ERR_TYPE_PACKET_ERROR:
      stream << " PACKET slave: " << error.Slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex
             << error.Index << "." << std::setfill('0') << std::setw(2) << std::hex
             << static_cast<uint16_t>(error.SubIdx) << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex
             << error.ErrorCode;
      break;
    case EC_ERR_TYPE_SDOINFO_ERROR:
      stream << " SDO slave: " << error.Slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex
             << error.Index << "." << std::setfill('0') << std::setw(2) << std::hex
             << static_cast<uint16_t>(error.SubIdx) << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex
             << static_cast<unsigned>(error.AbortCode) << " " << ec_sdoerror2string(error.AbortCode);
      break;
    case EC_ERR_TYPE_SOE_ERROR:
      stream << " SoE slave: " << error.Slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex
             << error.Index << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex
             << static_cast<unsigned>(error.AbortCode) << " " << ec_soeerror2string(error.ErrorCode);
      break;
    case EC_ERR_TYPE_MBX_ERROR:
      stream << " MBX slave: " << error.Slave << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex
             << error.ErrorCode << " " << ec_mbxerror2string(error.ErrorCode);
      break;
    default:
      stream << " MBX slave: " << error.Slave << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex
             << static_cast<unsigned>(error.AbortCode);
      break;
  }
  return stream.str();
}

void EthercatBusBase::printALStatus(const uint16_t slave)
{
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  assert(static_cast<int>(slave) <= getNumberOfSlaves());

  RCLCPP_INFO_STREAM(rclcpp::get_logger("EtherCAT bus"),
                     " slave: " << slave << " alStatusCode: 0x" << std::setfill('0') << std::setw(8) << std::hex
                                << ecatContext_.slavelist[slave].ALstatuscode << " "
                                << ec_ALstatuscode2string(ecatContext_.slavelist[slave].ALstatuscode));
}

bool EthercatBusBase::checkForSdoErrors(const uint16_t slave, const uint16_t index)
{
  while (ecx_iserror(&ecatContext_))
  {
    ec_errort error;
    if (ecx_poperror(&ecatContext_, &error))
    {
      std::string error_str = getErrorString(error);
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"), error_str);
      if (error.Slave == slave && error.Index == index)
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"), "Error in index " << index << slave << "of slave "
                                                                                  << slave
                                                                                  << ". Error string: " << error_str);
        return true;
      }
    }
  }
  return false;
}

bool EthercatBusBase::workingCounterIsOk() const
{
  return wkc_ >= getExpectedWorkingCounter();
}

void EthercatBusBase::syncDistributedClock0(const uint16_t slave, const bool activate, const double cycleTime,
                                            const double cycleShift)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("EtherCAT bus"), "Bus '" << name_ << "', slave " << slave << ":  "
                                                                 << (activate ? "Activating" : "Deactivating")
                                                                 << " distributed clock synchronization...");

  ecx_dcsync0(&ecatContext_, slave, static_cast<uint8_t>(activate), static_cast<uint32_t>(cycleTime * 1e9),
              static_cast<int32_t>(1e9 * cycleShift));

  RCLCPP_INFO_STREAM(rclcpp::get_logger("EtherCAT bus"), "Bus '" << name_ << "', slave " << slave << ":  "
                                                                 << (activate ? "Activated" : "Deactivated")
                                                                 << " distributed clock synchronization.");
}

EthercatBusBase::PdoSizeMap EthercatBusBase::getHardwarePdoSizes()
{
  PdoSizeMap pdo_map;

  for (const auto& slave : slaves_)
  {
    pdo_map.insert(std::make_pair(slave->getName(), getHardwarePdoSizes(slave->getAddress())));
  }

  return pdo_map;
}

uint8_t EthercatBusBase::getSlaveState(uint16_t slave)
{
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  ecx_readstate(&ecatContext_);
  return ecatSlavelist_[slave].state;
}

uint16_t EthercatBusBase::getSlaveALStatusCode(uint16_t slave)
{
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  return ecatContext_.slavelist[slave].ALstatuscode;
}

EthercatBusBase::PdoSizePair EthercatBusBase::getHardwarePdoSizes(const uint16_t slave)
{
  return std::make_pair(ecatContext_.slavelist[slave].Obytes, ecatContext_.slavelist[slave].Ibytes);
}

int EthercatBusBase::writeFile(const uint16_t slave, const std::string& fileName, const uint32_t& password,
                               const int fileSize, char* fileBuffer, int timeout)
{
  return ecx_FOEwrite(&ecatContext_, slave, const_cast<char*>(fileName.c_str()), password, fileSize, fileBuffer,
                      timeout);
}

bool EthercatBusBase::writeFirmware(const uint16_t slave, const std::string& fileName, const uint32_t& password,
                                    const int fileSize, char* fileBuffer)
{
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  // Set slave to EC_STATE_INIT state.
  ecatSlavelist_[slave].state = EC_STATE_INIT;
  ecx_writestate(&ecatContext_, slave);
  if (ecx_statecheck(&ecatContext_, slave, EC_STATE_INIT, EC_TIMEOUTSTATE * 4) != EC_STATE_INIT)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"), "Could not set slave '" << slave << "' to INIT state.");
    printALStatus(slave);
    isRunning_ = false;
    return false;
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("EtherCAT bus"), "Set slave '" << slave << "' to INIT state.");

  /* read BOOT mailbox data, master -> slave */
  uint32_t data = ecx_readeeprom(&ecatContext_, slave, ECT_SII_BOOTRXMBX, EC_TIMEOUTEEP);
  ecatSlavelist_[slave].SM[0].StartAddr = (uint16)LO_WORD(data);
  ecatSlavelist_[slave].SM[0].SMlength = (uint16)HI_WORD(data);
  /* store boot write mailbox address */
  ecatSlavelist_[slave].mbx_wo = (uint16)LO_WORD(data);
  /* store boot write mailbox size */
  ecatSlavelist_[slave].mbx_l = (uint16)HI_WORD(data);

  /* read BOOT mailbox data, slave -> master */
  data = ecx_readeeprom(&ecatContext_, slave, ECT_SII_BOOTTXMBX, EC_TIMEOUTEEP);
  ecatSlavelist_[slave].SM[1].StartAddr = (uint16)LO_WORD(data);
  ecatSlavelist_[slave].SM[1].SMlength = (uint16)HI_WORD(data);
  /* store boot read mailbox address */
  ecatSlavelist_[slave].mbx_ro = (uint16)LO_WORD(data);
  /* store boot read mailbox size */
  ecatSlavelist_[slave].mbx_rl = (uint16)HI_WORD(data);

  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT bus"), " SM0 A:%4.4x L:%4d F:%8.8x", ecatSlavelist_[slave].SM[0].StartAddr,
               ecatSlavelist_[slave].SM[0].SMlength, (int)ecatSlavelist_[slave].SM[0].SMflags);
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT bus"), " SM1 A:%4.4x L:%4d F:%8.8x", ecatSlavelist_[slave].SM[1].StartAddr,
               ecatSlavelist_[slave].SM[1].SMlength, (int)ecatSlavelist_[slave].SM[1].SMflags);
  /* program SM0 mailbox in for slave */
  ecx_FPWR(&ecatPort_, ecatSlavelist_[slave].configadr, ECT_REG_SM0, sizeof(ec_smt), &ecatSlavelist_[slave].SM[0],
           EC_TIMEOUTRET);
  /* program SM1 mailbox out for slave */
  ecx_FPWR(&ecatPort_, ecatSlavelist_[slave].configadr, ECT_REG_SM1, sizeof(ec_smt), &ecatSlavelist_[slave].SM[1],
           EC_TIMEOUTRET);

  // Set slave to EC_STATE_BOOT state.
  ecatSlavelist_[slave].state = EC_STATE_BOOT;
  ecx_writestate(&ecatContext_, slave);
  if (ecx_statecheck(&ecatContext_, slave, EC_STATE_BOOT, EC_TIMEOUTSTATE * 10) != EC_STATE_BOOT)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"), "Could not set slave '" << slave << "' to BOOT state.");
    printALStatus(slave);
    isRunning_ = false;
    return false;
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("EtherCAT bus"), "Set slave '" << slave << "' to BOOT state.");

  // Send file over EtherCAT.
  RCLCPP_INFO(rclcpp::get_logger("EtherCAT bus"), "Flashing firmware ...");
  // Write over EtherCAT.
  RCLCPP_INFO(rclcpp::get_logger("EtherCAT bus"), "Writing file over ethercat (FOE)");
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("EtherCAT bus"), "Size of file buffer: " << fileSize);

  int wkc = writeFile(slave, fileName, password, fileSize, fileBuffer);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("EtherCAT bus"), "Result wkc: " << wkc);
  // Check work counter.
  if (wkc != 1)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"), "Writing file over EtherCAT failed (wkc = " << wkc << ").");
    isRunning_ = false;
    return false;
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("EtherCAT bus"), "Request init state for slave " << slave);
  // Set slave to EC_STATE_INIT state.
  ecatSlavelist_[slave].state = EC_STATE_INIT;
  ecx_writestate(&ecatContext_, slave);

  RCLCPP_INFO(rclcpp::get_logger("EtherCAT bus"), "Flashing over EtherCAT was successful");

  isRunning_ = false;
  return true;
}
}  // namespace soem_interface
}  // namespace rokubimini
