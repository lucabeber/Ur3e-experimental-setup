
#include <rokubimini_ethercat/ObjectDictionary.hpp>
#include <rokubimini_ethercat/RokubiminiEthercatSlave.hpp>
#include <rokubimini_ethercat/TxPdoA.hpp>
#include <rokubimini_ethercat/TxPdoB.hpp>
#include <rokubimini_ethercat/TxPdoC.hpp>
#include <rokubimini_ethercat/TxPdoZ.hpp>
#include <rokubimini_ethercat/sdo/RxSdoCalibration.hpp>

#include <fstream>
#include <bitset>
#include <thread>
namespace rokubimini
{
namespace ethercat
{
RokubiminiEthercatSlave::RokubiminiEthercatSlave(const std::string& name,
                                                 rokubimini::soem_interface::EthercatBusBase* bus,
                                                 const uint32_t address, const PdoTypeEnum pdoTypeEnum)
  : rokubimini::soem_interface::EthercatSlaveBase(bus, address)
  , name_(name)
  , pdoTypeEnum_(pdoTypeEnum)
  , currentPdoTypeEnum_(PdoTypeEnum::NA)
  , isRunning_{ true }
  , runsAsync_{ false }
{
  PdoInfo pdo_info;

  pdo_info.rxPdoId_ = OD_RX_PDO_ID_VAL_A;
  pdo_info.txPdoId_ = OD_TX_PDO_ID_VAL_A;
  pdo_info.rxPdoSize_ = sizeof(RxPdo);
  pdo_info.txPdoSize_ = sizeof(TxPdoA);
  pdo_info.moduleId_ = 0x00119800;
  pdoInfos_.insert({ PdoTypeEnum::A, pdo_info });
}

bool RokubiminiEthercatSlave::startup()
{
  // Configure PDO setup
  return configurePdo(pdoTypeEnum_);
}

void RokubiminiEthercatSlave::preSetupConfiguration()
{
}
void RokubiminiEthercatSlave::updateRead()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const auto& update_stamp = bus_->getUpdateReadStamp();

  switch (getCurrentPdoTypeEnum())
  {
    case PdoTypeEnum::A:
      TxPdoA tx_pdo_a;
      bus_->readTxPdo(address_, tx_pdo_a);
      {
        reading_.getWrench().header.stamp = update_stamp;
        reading_.getWrench().header.frame_id = name_ + "_wrench";
        reading_.getWrench().wrench.force.x = tx_pdo_a.forceX_;
        reading_.getWrench().wrench.force.y = tx_pdo_a.forceY_;
        reading_.getWrench().wrench.force.z = tx_pdo_a.forceZ_;
        reading_.getWrench().wrench.torque.x = tx_pdo_a.torqueX_;
        reading_.getWrench().wrench.torque.y = tx_pdo_a.torqueY_;
        reading_.getWrench().wrench.torque.z = tx_pdo_a.torqueZ_;

        reading_.getImu().header.stamp = update_stamp;
        reading_.getImu().header.frame_id = name_ + "_imu";
        reading_.getImu().angular_velocity.x = tx_pdo_a.angularRateX_ * DEG_TO_RAD;
        reading_.getImu().angular_velocity.y = tx_pdo_a.angularRateY_ * DEG_TO_RAD;
        reading_.getImu().angular_velocity.z = tx_pdo_a.angularRateZ_ * DEG_TO_RAD;
        reading_.getImu().linear_acceleration.x = tx_pdo_a.accelerationX_ * G_TO_METERS_PER_SECOND_SQUARED;
        reading_.getImu().linear_acceleration.y = tx_pdo_a.accelerationY_ * G_TO_METERS_PER_SECOND_SQUARED;
        reading_.getImu().linear_acceleration.z = tx_pdo_a.accelerationZ_ * G_TO_METERS_PER_SECOND_SQUARED;
        reading_.getImu().orientation.x = tx_pdo_a.estimatedOrientationX_;
        reading_.getImu().orientation.y = tx_pdo_a.estimatedOrientationY_;
        reading_.getImu().orientation.z = tx_pdo_a.estimatedOrientationZ_;
        reading_.getImu().orientation.w = tx_pdo_a.estimatedOrientationW_;

        reading_.setStatusword(Statusword(tx_pdo_a.warningsErrorsFatals_));
        reading_.setForceTorqueSaturated(tx_pdo_a.forceTorqueSaturated_ != 0);

        reading_.getTemperature().header.stamp = update_stamp;
        reading_.getTemperature().header.frame_id = name_ + "_temp";
        reading_.getTemperature().temperature = tx_pdo_a.temperature_;
        reading_.getTemperature().variance = 0;  // unknown variance
      }
      break;
    default:
      break;
  }
}

bool RokubiminiEthercatSlave::getSerialNumber(unsigned int& serialNumber)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  bool success = true;
  uint32_t serial_number;
  success &= sendSdoRead(OD_IDENTITY_ID, OD_IDENTITY_SID_SERIAL_NUMBER, false, serial_number);
  serialNumber = static_cast<unsigned int>(serial_number);
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT slave"), "[%s] Reading serial number: %u", name_.c_str(), serialNumber);
  return success;
}

bool RokubiminiEthercatSlave::getForceTorqueSamplingRate(int& samplingRate)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  bool success = true;
  int16_t sampling_rate;
  success &= sendSdoRead(OD_SAMPLING_RATE_ID, 0x00, false, sampling_rate);
  samplingRate = static_cast<int>(sampling_rate);
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT slave"), "[%s] Force/Torque sampling rate: %d Hz", name_.c_str(),
               samplingRate);
  return success;
}

bool RokubiminiEthercatSlave::setForceTorqueFilter(const configuration::ForceTorqueFilter& filter)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT slave"), "[%s] Setting force/torque filter", name_.c_str());
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT slave"), "[%s] \tchop: %u", name_.c_str(), filter.getChopEnable());
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT slave"), "[%s] \tfast: %u", name_.c_str(), filter.getFastEnable());
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT slave"), "[%s] \tskip: %u", name_.c_str(), filter.getSkipEnable());
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT slave"), "[%s] \tsize: %u", name_.c_str(), filter.getSincFilterSize());
  bool success = true;
  /* the following order is mandatory as the sinc filter size range is depending on the other values.
   * Thats why has to be the last one to be changed */
  success &=
      sendSdoWrite(OD_FORCE_TORQUE_FILTER_ID, OD_FORCE_TORQUE_FILTER_SID_CHOP_ENABLE, false, filter.getChopEnable());
  success &=
      sendSdoWrite(OD_FORCE_TORQUE_FILTER_ID, OD_FORCE_TORQUE_FILTER_SID_FAST_ENABLE, false, filter.getFastEnable());
  success &=
      sendSdoWrite(OD_FORCE_TORQUE_FILTER_ID, OD_FORCE_TORQUE_FILTER_SID_FIR_DISABLE, false, filter.getSkipEnable());
  success &=
      sendSdoWrite(OD_FORCE_TORQUE_FILTER_ID, OD_FORCE_TORQUE_FILTER_SID_SINC_SIZE, false, filter.getSincFilterSize());
  return success;
}

bool RokubiminiEthercatSlave::setAccelerationFilter(const unsigned int filter)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT slave"), "[%s] Setting acceleration filter: %u", name_.c_str(), filter);
  bool success = true;
  success &= sendSdoWrite(OD_ACCELERATION_FILTER_ID, 0x00, false, static_cast<uint8_t>(filter));
  return success;
}

bool RokubiminiEthercatSlave::setAngularRateFilter(const unsigned int filter)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT slave"), "[%s] Setting angular rate filter: %u", name_.c_str(), filter);
  bool success = true;
  success &= sendSdoWrite(OD_ANGULAR_RATE_FILTER_ID, 0x00, false, static_cast<uint8_t>(filter));
  return success;
}

bool RokubiminiEthercatSlave::setAccelerationRange(const unsigned int range)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT slave"), "[%s] Setting acceleration range: %u", name_.c_str(), range);
  bool success = true;
  success &= sendSdoWrite(OD_ACCELERATION_RANGE_ID, 0x00, false, static_cast<uint8_t>(range));
  return success;
}

bool RokubiminiEthercatSlave::setAngularRateRange(const unsigned int range)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT slave"), "[%s] Setting angular rate range: %u", name_.c_str(), range);
  bool success = true;
  success &= sendSdoWrite(OD_ANGULAR_RATE_RANGE_ID, 0x00, false, static_cast<uint8_t>(range));
  return success;
}

bool RokubiminiEthercatSlave::setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("EtherCAT slave"),
                      "[" << name_.c_str() << "] Setting Force/Torque offset: " << forceTorqueOffset << std::endl);
  bool success = true;
  success &= sendSdoWrite(OD_SENSOR_FORCE_TORQUE_OFFSET_ID, OD_SENSOR_FORCE_TORQUE_OFFSET_SID_1, false,
                          static_cast<float>(forceTorqueOffset(0, 0)));
  success &= sendSdoWrite(OD_SENSOR_FORCE_TORQUE_OFFSET_ID, OD_SENSOR_FORCE_TORQUE_OFFSET_SID_2, false,
                          static_cast<float>(forceTorqueOffset(1, 0)));
  success &= sendSdoWrite(OD_SENSOR_FORCE_TORQUE_OFFSET_ID, OD_SENSOR_FORCE_TORQUE_OFFSET_SID_3, false,
                          static_cast<float>(forceTorqueOffset(2, 0)));
  success &= sendSdoWrite(OD_SENSOR_FORCE_TORQUE_OFFSET_ID, OD_SENSOR_FORCE_TORQUE_OFFSET_SID_4, false,
                          static_cast<float>(forceTorqueOffset(3, 0)));
  success &= sendSdoWrite(OD_SENSOR_FORCE_TORQUE_OFFSET_ID, OD_SENSOR_FORCE_TORQUE_OFFSET_SID_5, false,
                          static_cast<float>(forceTorqueOffset(4, 0)));
  success &= sendSdoWrite(OD_SENSOR_FORCE_TORQUE_OFFSET_ID, OD_SENSOR_FORCE_TORQUE_OFFSET_SID_6, false,
                          static_cast<float>(forceTorqueOffset(5, 0)));
  return success;
}

bool RokubiminiEthercatSlave::setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT slave"), "[%s] Setting sensor configuration", name_.c_str());
  bool success = true;

  success &= sendSdoWrite(OD_SENSOR_CONFIGURATION_ID, OD_SENSOR_CONFIGURATION_SID_CALIBRATION_MATRIX_ACTIVE, false,
                          sensorConfiguration.getCalibrationMatrixActive());
  success &= sendSdoWrite(OD_SENSOR_CONFIGURATION_ID, OD_SENSOR_CONFIGURATION_SID_TEMPERATURE_COMPENSATION, false,
                          sensorConfiguration.getTemperatureCompensationActive());
  success &= sendSdoWrite(OD_SENSOR_CONFIGURATION_ID, OD_SENSOR_CONFIGURATION_SID_IMU_ACTIVE, false,
                          sensorConfiguration.getImuActive());
  success &= sendSdoWrite(OD_SENSOR_CONFIGURATION_ID, OD_SENSOR_CONFIGURATION_SID_COORD_SYSTEM_CONFIGURATION, false,
                          sensorConfiguration.getCoordinateSystemConfigurationActive());
  success &= sendSdoWrite(OD_SENSOR_CONFIGURATION_ID, OD_SENSOR_CONFIGURATION_SID_INERTIA_COMPENSATION, false,
                          sensorConfiguration.getInertiaCompensationActive());
  success &= sendSdoWrite(OD_SENSOR_CONFIGURATION_ID, OD_SENSOR_CONFIGURATION_SID_ORIENTATION_ESTIMATION, false,
                          sensorConfiguration.getOrientationEstimationActive());

  return success;
}

bool RokubiminiEthercatSlave::sendCalibrationMatrixEntry(const uint8_t subId, const double entry)
{
  return sendSdoWrite(OD_SENSOR_CALIBRATION_ID, subId, false, static_cast<float>(entry));
}

bool RokubiminiEthercatSlave::setSensorCalibration(const calibration::SensorCalibration& sensorCalibration)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  bool success = true;

  success &= sendCalibrationMatrixEntry(0x02, sensorCalibration.getCalibrationMatrix()(0, 0));
  success &= sendCalibrationMatrixEntry(0x03, sensorCalibration.getCalibrationMatrix()(0, 1));
  success &= sendCalibrationMatrixEntry(0x04, sensorCalibration.getCalibrationMatrix()(0, 2));
  success &= sendCalibrationMatrixEntry(0x05, sensorCalibration.getCalibrationMatrix()(0, 3));
  success &= sendCalibrationMatrixEntry(0x06, sensorCalibration.getCalibrationMatrix()(0, 4));
  success &= sendCalibrationMatrixEntry(0x07, sensorCalibration.getCalibrationMatrix()(0, 5));
  success &= sendCalibrationMatrixEntry(0x08, sensorCalibration.getCalibrationMatrix()(1, 0));
  success &= sendCalibrationMatrixEntry(0x09, sensorCalibration.getCalibrationMatrix()(1, 1));
  success &= sendCalibrationMatrixEntry(0x0A, sensorCalibration.getCalibrationMatrix()(1, 2));
  success &= sendCalibrationMatrixEntry(0x0B, sensorCalibration.getCalibrationMatrix()(1, 3));
  success &= sendCalibrationMatrixEntry(0x0C, sensorCalibration.getCalibrationMatrix()(1, 4));
  success &= sendCalibrationMatrixEntry(0x0D, sensorCalibration.getCalibrationMatrix()(1, 5));
  success &= sendCalibrationMatrixEntry(0x0E, sensorCalibration.getCalibrationMatrix()(2, 0));
  success &= sendCalibrationMatrixEntry(0x0F, sensorCalibration.getCalibrationMatrix()(2, 1));
  success &= sendCalibrationMatrixEntry(0x10, sensorCalibration.getCalibrationMatrix()(2, 2));
  success &= sendCalibrationMatrixEntry(0x11, sensorCalibration.getCalibrationMatrix()(2, 3));
  success &= sendCalibrationMatrixEntry(0x12, sensorCalibration.getCalibrationMatrix()(2, 4));
  success &= sendCalibrationMatrixEntry(0x13, sensorCalibration.getCalibrationMatrix()(2, 5));
  success &= sendCalibrationMatrixEntry(0x14, sensorCalibration.getCalibrationMatrix()(3, 0));
  success &= sendCalibrationMatrixEntry(0x15, sensorCalibration.getCalibrationMatrix()(3, 1));
  success &= sendCalibrationMatrixEntry(0x16, sensorCalibration.getCalibrationMatrix()(3, 2));
  success &= sendCalibrationMatrixEntry(0x17, sensorCalibration.getCalibrationMatrix()(3, 3));
  success &= sendCalibrationMatrixEntry(0x18, sensorCalibration.getCalibrationMatrix()(3, 4));
  success &= sendCalibrationMatrixEntry(0x19, sensorCalibration.getCalibrationMatrix()(3, 5));
  success &= sendCalibrationMatrixEntry(0x1A, sensorCalibration.getCalibrationMatrix()(4, 0));
  success &= sendCalibrationMatrixEntry(0x1B, sensorCalibration.getCalibrationMatrix()(4, 1));
  success &= sendCalibrationMatrixEntry(0x1C, sensorCalibration.getCalibrationMatrix()(4, 2));
  success &= sendCalibrationMatrixEntry(0x1D, sensorCalibration.getCalibrationMatrix()(4, 3));
  success &= sendCalibrationMatrixEntry(0x1E, sensorCalibration.getCalibrationMatrix()(4, 4));
  success &= sendCalibrationMatrixEntry(0x1F, sensorCalibration.getCalibrationMatrix()(4, 5));
  success &= sendCalibrationMatrixEntry(0x20, sensorCalibration.getCalibrationMatrix()(5, 0));
  success &= sendCalibrationMatrixEntry(0x21, sensorCalibration.getCalibrationMatrix()(5, 1));
  success &= sendCalibrationMatrixEntry(0x22, sensorCalibration.getCalibrationMatrix()(5, 2));
  success &= sendCalibrationMatrixEntry(0x23, sensorCalibration.getCalibrationMatrix()(5, 3));
  success &= sendCalibrationMatrixEntry(0x24, sensorCalibration.getCalibrationMatrix()(5, 4));
  success &= sendCalibrationMatrixEntry(0x25, sensorCalibration.getCalibrationMatrix()(5, 5));
  success &= sendSdoWrite(OD_SENSOR_CALIBRATION_ID, 0x01, false, sensorCalibration.getPassPhrase());

  return success;
}
bool RokubiminiEthercatSlave::setConfigMode()
{
  setState(EC_STATE_PRE_OP);
  // Sleep for some time to give the slave time to execute the pre-op cb
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  if (!waitForState(EC_STATE_PRE_OP))
  {
    RCLCPP_ERROR(rclcpp::get_logger("EtherCAT slave"), "[%s] Slave failed to switch to PREOP state", name_.c_str());
    return false;
  }
  return true;
}

uint8_t RokubiminiEthercatSlave::getState()
{
  return bus_->getSlaveState(address_);
}

uint16_t RokubiminiEthercatSlave::getALStatusCode()
{
  return bus_->getSlaveALStatusCode(address_);
}

std::string RokubiminiEthercatSlave::getSlaveStateString(uint8_t state)
{
  return STATE_TO_STRING.find(state)->second;
}

void RokubiminiEthercatSlave::updateConnectionStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  // although SOEM names it state, it's actually status since it has also the error.
  uint8_t status = getState();
  std::bitset<8> status_bits(status);
  // get the error from the 4th bit.
  bool has_error = status_bits[4];
  // get the state of the slave, without the errors
  uint8_t state = status & 0x0F;
  // get the AL Status Code (indicating if there are any errors)
  uint16_t al_status_code = getALStatusCode();
  if (state == EC_STATE_INIT || has_error)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Slave is in " + getSlaveStateString(state));
  }
  else if (state == EC_STATE_PRE_OP || !bus_->workingCounterIsOk())
  {
    if (state == EC_STATE_PRE_OP)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Slave is in " + getSlaveStateString(state));
    }
    else if (!bus_->workingCounterIsOk())
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                   "Working counter is too low, expected working counter is " +
                       std::to_string(bus_->getExpectedWorkingCounter()));
    }
  }
  else if (state == EC_STATE_SAFE_OP || state == EC_STATE_OPERATIONAL)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Slave is in " + getSlaveStateString(state));
  }
  stat.add("EtherCAT Slave State", getSlaveStateString(state));
  stat.add("Product Name", getProductName());
  stat.add("Serial Number (S/N)", std::to_string(fetchSerialNumber()));
  stat.add("EtherCAT Interface", bus_->getName());
  stat.add("EtherCAT Address", std::to_string(address_));
  stat.add("Runs Async", runsAsync() ? "Yes" : "No");
  stat.add("Has Error", has_error ? "Yes" : "No");
  std::stringstream al_status_code_stream;
  al_status_code_stream << "0x" << std::setfill('0') << std::setw(8) << std::hex << al_status_code;
  stat.add("Error Code", al_status_code_stream.str());
}

void RokubiminiEthercatSlave::updateDataFlags()
{
  Reading reading;
  getReading(reading);
  Statusword status_word = reading.getStatusword();
  std::lock_guard<std::recursive_mutex> lock(dataFlagsMutex_);
  status_word.setData(status_word.getData() | currentDataFlagsDiagnostics_.getData());
  currentDataFlagsDiagnostics_ = status_word;
}

void RokubiminiEthercatSlave::resetDataFlags()
{
  std::lock_guard<std::recursive_mutex> lock(dataFlagsMutex_);
  currentDataFlagsDiagnostics_ = Statusword();
}

void RokubiminiEthercatSlave::createDataFlagsDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  std::lock_guard<std::recursive_mutex> lock(dataFlagsMutex_);
  Statusword status_word = currentDataFlagsDiagnostics_;
  std::bitset<32> bits(status_word.getData());
  if (status_word.hasErrorAdcSaturated() || status_word.hasErrorAccSaturated() || status_word.hasErrorGyroSaturated() ||
      status_word.hasErrorAdcOutOfSync() || status_word.hasErrorSensingRangeExceeded() ||
      status_word.hasFatalSupplyVoltage())
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Errors in Data Flags");
  }
  else if (status_word.hasWarningOvertemperature())
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Errors in Data Flags");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No errors in Data Flags");
  }
  stat.add("ADC Saturated", bits[0] ? "Yes" : "No");
  stat.add("ACC Saturated", bits[1] ? "Yes" : "No");
  stat.add("Gyroscope Saturated", bits[2] ? "Yes" : "No");
  stat.add("ADC is out of sync", bits[3] ? "Yes" : "No");
  stat.add("Sensing Range Exceeded", bits[4] ? "Yes" : "No");
  stat.add("High temperature in Rokubimini Sensor", bits[5] ? "Yes" : "No");
  stat.add("Supply voltage exceeds limits", bits[6] ? "Yes" : "No");
  stat.add("Reserved 1", bits[7] ? "Yes" : "No");
  stat.add("Reserved 2", bits[8] ? "Yes" : "No");
  stat.add("Reserved 3", bits[9] ? "Yes" : "No");
  stat.add("Reserved 4", bits[10] ? "Yes" : "No");
  stat.add("Reserved 5", bits[11] ? "Yes" : "No");
  stat.add("Reserved 6", bits[12] ? "Yes" : "No");
  stat.add("Reserved 7", bits[13] ? "Yes" : "No");
  stat.add("Reserved 8", bits[14] ? "Yes" : "No");
  stat.add("Reserved 9", bits[15] ? "Yes" : "No");
  stat.add("Reserved 10", bits[16] ? "Yes" : "No");
  stat.add("Reserved 11", bits[17] ? "Yes" : "No");
  stat.add("Reserved 12", bits[18] ? "Yes" : "No");
  stat.add("Reserved 13", bits[19] ? "Yes" : "No");
  stat.add("Reserved 14", bits[20] ? "Yes" : "No");
  stat.add("Reserved 15", bits[21] ? "Yes" : "No");
  stat.add("Reserved 16", bits[22] ? "Yes" : "No");
  stat.add("Reserved 17", bits[23] ? "Yes" : "No");
  stat.add("Reserved 18", bits[24] ? "Yes" : "No");
  stat.add("Reserved 19", bits[25] ? "Yes" : "No");
  stat.add("Reserved 20", bits[26] ? "Yes" : "No");
  stat.add("Reserved 21", bits[27] ? "Yes" : "No");
  stat.add("Reserved 22", bits[28] ? "Yes" : "No");
  stat.add("Reserved 23", bits[29] ? "Yes" : "No");
  stat.add("Reserved 24", bits[30] ? "Yes" : "No");
  stat.add("Reserved 25", bits[31] ? "Yes" : "No");
}
bool RokubiminiEthercatSlave::setRunMode()
{
  setState(EC_STATE_SAFE_OP);
  setState(EC_STATE_OPERATIONAL);
  return true;
}

bool RokubiminiEthercatSlave::saveConfigParameter()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_DEBUG(rclcpp::get_logger("EtherCAT slave"), "[%s] Saving configuration parameters", name_.c_str());
  bool success = true;
  uint8_t retain_configuration = 0x01;
  success &= sendSdoWrite(OD_CONTROL_ID, OD_CONTROL_SID_COMMAND, false, retain_configuration);
  uint8_t status;
  success &= sendSdoRead(OD_CONTROL_ID, OD_CONTROL_SID_STATUS, false, status);
  if (status != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("EtherCAT slave"),
                 "[%s] Could not save configuration parameters on device. Status value is: %u", name_.c_str(), status);
    return false;
  }
  return success;
}

void RokubiminiEthercatSlave::getReading(rokubimini::Reading& reading) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  reading = reading_;
}

void RokubiminiEthercatSlave::updateWrite()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  RxPdo rx_pdo;
  rx_pdo.digitalOutput_ = static_cast<uint8_t>(0);
  bus_->writeRxPdo(address_, rx_pdo);
}

void RokubiminiEthercatSlave::shutdown()
{
  setState(EC_STATE_INIT);
  waitForState(EC_STATE_INIT);
}

void RokubiminiEthercatSlave::setState(const uint16_t state)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  bus_->setState(state, address_);
}

bool RokubiminiEthercatSlave::waitForState(const uint16_t state)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return bus_->waitForState(state, address_);
}

RokubiminiEthercatSlave::PdoInfo RokubiminiEthercatSlave::getCurrentPdoInfo() const
{
  // const auto a = pdoInfos_[PdoTypeEnum::A];
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return pdoInfos_.at(currentPdoTypeEnum_);
}

bool RokubiminiEthercatSlave::configurePdo(const PdoTypeEnum pdoTypeEnum)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (pdoTypeEnum == PdoTypeEnum::NA)
  {
    RCLCPP_ERROR(rclcpp::get_logger("EtherCAT slave"), "[%s] Invalid EtherCAT PDO Type.", name_.c_str());
    return false;
  }

  // If PDO setup is already active, return.
  if (pdoTypeEnum == getCurrentPdoTypeEnum())
  {
    return true;
  }

  // {
  //   std::lock_guard<std::recursive_mutex> lock(busMutex_);
  //   if (!sendSdoWrite(OD_SWITCH_PDO_ID, OD_SWITCH_PDO_SID, false, pdoInfos_.at(pdoTypeEnum).moduleId_)) {
  //     return false;
  //   }
  // }

  currentPdoTypeEnum_ = pdoTypeEnum;
  return true;
}

bool RokubiminiEthercatSlave::readFileToBuffer(const std::string& filePath)
{
  // Read the file into the file buffer
  std::ifstream file(filePath, std::ios::binary);
  std::string file_buffer;
  if (file.is_open())
  {
    int ch;
    while ((ch = file.get()) != std::istream::traits_type::eof())
    {
      file_buffer.push_back(static_cast<char>(ch));
    }
    fileBuffer_ = const_cast<char*>(file_buffer.c_str());
    fileSize_ = static_cast<int>(file_buffer.size());
    file.close();
    if (fileSize_ > MAX_FILE_SIZE_FIRMWARE)
    {
      RCLCPP_ERROR(rclcpp::get_logger("EtherCAT slave"), "[%s] File is too big.", name_.c_str());
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT slave"), "[" << getName() << "] "
                                                                  << "Failed to open file " << filePath);
    return false;
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("EtherCAT slave"),
                      "[" << getName() << "] "
                          << "The firmware was read successfully. Size of file buffer: " << fileSize_);
  return true;
}

bool RokubiminiEthercatSlave::firmwareUpdate(const std::string& filePath, const std::string& fileName,
                                             const uint32_t& password)
{
  bool success;
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!readFileToBuffer(filePath))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT slave"), "[" << getName() << "] "
                                                                  << "Could not read file in path " << filePath << ".");
    return false;
  }
  success = bus_->writeFirmware(address_, fileName, password, fileSize_, fileBuffer_);
  if (!bus_->isRunning())
  {
    isRunning_ = false;
  }
  if (!success)
  {
    RCLCPP_ERROR(rclcpp::get_logger("EtherCAT slave"), "[%s] Flashing was not successful.", name_.c_str());
  }
  return success;
}
}  // namespace ethercat
}  // namespace rokubimini
