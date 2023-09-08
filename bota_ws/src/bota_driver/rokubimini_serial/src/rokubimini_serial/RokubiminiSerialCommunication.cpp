
#include <rokubimini_serial/RokubiminiSerialCommunication.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rokubimini
{
namespace serial
{
RokubiminiSerialCommandCommSetup::RokubiminiSerialCommandCommSetup(
    const configuration::SensorConfiguration& sensorConfiguration, const uint8_t& dataFormat, const uint8_t& baudRate)
  : sensorConfiguration_(sensorConfiguration), dataFormat_(dataFormat), baudRate_(baudRate)
{
}

bool RokubiminiSerialCommandCommSetup::formatCommand(std::string& formattedString)
{
  char buffer[100];
  int bytes_count = sprintf(buffer, formatString_.c_str(), sensorConfiguration_.getTemperatureCompensationActive(),
                            sensorConfiguration_.getCalibrationMatrixActive(), dataFormat_, baudRate_);

  if (bytes_count < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Serial Communication"), "Failed to format communication setup command");
    return false;
  }
  formattedString.clear();
  formattedString.assign(buffer, bytes_count);
  return true;
}

RokubiminiSerialCommandFilter::RokubiminiSerialCommandFilter(const configuration::ForceTorqueFilter& filter)
  : filter_(filter)
{
}

bool RokubiminiSerialCommandFilter::formatCommand(std::string& formattedString)
{
  char buffer[100];
  int bytes_count = sprintf(buffer, formatString_.c_str(), filter_.getSincFilterSize(), filter_.getChopEnable(),
                            filter_.getFastEnable(), filter_.getSkipEnable());

  if (bytes_count < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Serial Communication"), "Failed to format filters command");
    return false;
  }
  formattedString.clear();
  formattedString.assign(buffer, bytes_count);
  return true;
}

RokubiminiSerialCommandOffset::RokubiminiSerialCommandOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset)
  : forceTorqueOffset_(forceTorqueOffset)
{
}

bool RokubiminiSerialCommandOffset::formatCommand(std::string& formattedString)
{
  char buffer[100];
  int bytes_count =
      sprintf(buffer, formatString_.c_str(), forceTorqueOffset_(0, 0), forceTorqueOffset_(1, 0),
              forceTorqueOffset_(2, 0), forceTorqueOffset_(3, 0), forceTorqueOffset_(4, 0), forceTorqueOffset_(5, 0));

  if (bytes_count < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Serial Communication"), "Failed to format offset command");
    return false;
  }
  formattedString.clear();
  formattedString.assign(buffer, bytes_count);
  return true;
}

RokubiminiSerialCommandSensorCalibrationRow::RokubiminiSerialCommandSensorCalibrationRow(
    const double& sensorCalibration1, const double& sensorCalibration2, const double& sensorCalibration3,
    const double& sensorCalibration4, const double& sensorCalibration5, const double& sensorCalibration6,
    const uint32_t& row)
  : sensorCalibration1_(sensorCalibration1)
  , sensorCalibration2_(sensorCalibration2)
  , sensorCalibration3_(sensorCalibration3)
  , sensorCalibration4_(sensorCalibration4)
  , sensorCalibration5_(sensorCalibration5)
  , sensorCalibration6_(sensorCalibration6)
  , row_(row)
{
}

bool RokubiminiSerialCommandSensorCalibrationRow::formatCommand(std::string& formattedString)
{
  char buffer[100];
  int bytes_count = sprintf(buffer, formatString_.c_str(), row_, sensorCalibration1_, sensorCalibration2_,
                            sensorCalibration3_, sensorCalibration4_, sensorCalibration5_, sensorCalibration6_);

  if (bytes_count < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Serial Communication"), "Failed to format offset command");
    return false;
  }
  formattedString.clear();
  formattedString.assign(buffer, bytes_count);
  return true;
}

bool RokubiminiSerialCommandHardReset::formatCommand(std::string& formattedString)
{
  formattedString.clear();
  formattedString.assign(formatString_);
  return true;
}

bool RokubiminiSerialCommandSoftReset::formatCommand(std::string& formattedString)
{
  formattedString.clear();
  formattedString.assign(formatString_);
  return true;
}

bool RokubiminiSerialCommandConfig::formatCommand(std::string& formattedString)
{
  formattedString.clear();
  formattedString.assign(formatString_);
  return true;
}

bool RokubiminiSerialCommandRun::formatCommand(std::string& formattedString)
{
  formattedString.clear();
  formattedString.assign(formatString_);
  return true;
}

bool RokubiminiSerialCommandSave::formatCommand(std::string& formattedString)
{
  formattedString.clear();
  formattedString.assign(formatString_);
  return true;
}

bool RokubiminiSerialCommandLoad::formatCommand(std::string& formattedString)
{
  formattedString.clear();
  formattedString.assign(formatString_);
  return true;
}

bool RokubiminiSerialCommandPrint::formatCommand(std::string& formattedString)
{
  formattedString.clear();
  formattedString.assign(formatString_);
  return true;
}

bool RokubiminiSerialResponseRegexCommSetup::matchInString(const std::string& str)
{
  bool success = false;
  std::smatch matches;
  success = regex_search(str, matches, stringRegex_);
  if (matches.empty() || !matches.ready())
  {
    return false;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("Serial Communication"), "Match size is: %zu", matches.size());
  for (unsigned i = 0; i < matches.size(); ++i)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"), "match " << i << ": " << matches[i]);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"),
                        " (with a length of " << matches[i].length() << ")");
  }
  std::string s = matches[0];
  char temp_comp, calibration, data_format, baud_rate;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"), "String is: " << s << " and format is: " << format_);
  int bytes_count = sscanf(s.c_str(), format_.c_str(), &temp_comp, &calibration, &data_format, &baud_rate);
  if (bytes_count < 4)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Serial Communication"), "Failed to format communication setup fields");
    return false;
  }
  // use character arithmetic for extracting integer from ASCII codes.
  tempComp_ = temp_comp - '0';
  calibration_ = calibration - '0';
  dataFormat_ = data_format - '0';
  baudRate_ = baud_rate - '0';
  RCLCPP_DEBUG(rclcpp::get_logger("Serial Communication"),
               "Baud rate is: %u, data format is: %u, temp comp: %u, calibration: %u", getBaudRate(), getDataFormat(),
               getTempComp(), getCalibration());
  return success;
}

bool RokubiminiSerialResponseRegexAck::matchInString(const std::string& str)
{
  bool success = false;
  std::smatch matches;
  success = regex_search(str, matches, stringRegex_);
  if (matches.empty() || !matches.ready())
  {
    return false;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("Serial Communication"), "Match size is: %zu", matches.size());
  for (unsigned i = 0; i < matches.size(); ++i)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"), "match " << i << ": " << matches[i]);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"),
                        " (with a length of " << matches[i].length() << ")");
  }
  std::string s = matches[0];
  char first_number, return_code;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"),
                      "String is: " << s << " and format is: " << format_.c_str());
  int bytes_count = sscanf(s.c_str(), format_.c_str(), &first_number, &command_, &return_code);
  if (bytes_count < 3)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Serial Communication"), "Failed to format ACK fields");
    return false;
  }
  // use character arithmetic for extracting integer from ASCII codes.
  firstNumber_ = first_number - '0';
  returnCode_ = return_code - '0';
  RCLCPP_DEBUG(rclcpp::get_logger("Serial Communication"), "First number is: %u, command is: %c, return code: %u",
               getFirstNumber(), getCommand(), getReturnCode());
  return success;
}

bool RokubiminiSerialResponseRegexProductName::matchInString(const std::string& str)
{
  bool success = false;
  std::smatch matches;
  success = regex_search(str, matches, stringRegex_);
  if (matches.empty() || !matches.ready())
  {
    return false;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("Serial Communication"), "Match size is: %zu", matches.size());
  for (unsigned i = 0; i < matches.size(); ++i)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"), "match " << i << ": " << matches[i]);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"),
                        " (with a length of " << matches[i].length() << ")");
  }
  std::string s = matches[0];
  char product_name[20];
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"), "String is: " << s << " and format is: " << format_);
  int ret = sscanf(s.c_str(), format_.c_str(), &product_name);
  if (ret < 1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Serial Communication"), "Failed to format product name");
    return false;
  }
  productName_.assign(product_name);
  RCLCPP_DEBUG(rclcpp::get_logger("Serial Communication"), "Product name is: %s", getProductName().c_str());
  return success;
}

bool RokubiminiSerialResponseRegexSerialNumber::matchInString(const std::string& str)
{
  bool success = false;
  std::smatch matches;
  success = regex_search(str, matches, stringRegex_);
  if (matches.empty() || !matches.ready())
  {
    return false;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("Serial Communication"), "Match size is: %zu", matches.size());
  for (unsigned i = 0; i < matches.size(); ++i)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"), "match " << i << ": " << matches[i]);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"),
                        " (with a length of " << matches[i].length() << ")");
  }
  std::string s = matches[0];
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"), "String is: " << s << " and format is: " << format_);
  int ret = sscanf(s.c_str(), format_.c_str(), &serialNumber_);
  if (ret < 1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Serial Communication"), "Failed to format serial number");
    return false;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("Serial Communication"), "Serial number is: %u", getSerialNumber());
  return success;
}

bool RokubiminiSerialResponseRegexFirmwareVersion::matchInString(const std::string& str)
{
  bool success = false;
  std::smatch matches;
  success = regex_search(str, matches, stringRegex_);
  if (matches.empty() || !matches.ready())
  {
    return false;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("Serial Communication"), "Match size is: %zu", matches.size());
  for (unsigned i = 0; i < matches.size(); ++i)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"), "match " << i << ": " << matches[i]);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"),
                        " (with a length of " << matches[i].length() << ")");
  }
  std::string s = matches[0];
  char firmware_version[25];
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"), "String is: " << s << " and format is: " << format_);
  int ret = sscanf(s.c_str(), format_.c_str(), &firmware_version);
  if (ret < 1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Serial Communication"), "Failed to format firmware version");
    return false;
  }
  firmwareVersion_.assign(firmware_version);
  RCLCPP_DEBUG(rclcpp::get_logger("Serial Communication"), "Firmware version is: %s", getFirmwareVersion().c_str());
  return success;
}

bool RokubiminiSerialResponseRegexBoot::matchInString(const std::string& str)
{
  bool success = false;
  std::smatch matches;
  success = regex_search(str, matches, stringRegex_);
  if (matches.empty() || !matches.ready())
  {
    return false;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("Serial Communication"), "Match size is: %zu", matches.size());
  for (unsigned i = 0; i < matches.size(); ++i)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"), "match " << i << ": " << matches[i]);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"),
                        " (with a length of " << matches[i].length() << ")");
  }
  std::string s = matches[0];
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Serial Communication"), "String is: " << s);

  return success;
}
}  // namespace serial
}  // namespace rokubimini
