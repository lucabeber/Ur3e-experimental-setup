#pragma once

// std
#include <cstdint>

/*!
 * Object dictionary:
 *   ID = index (16 bit)
 *   SID = sub-index (8 bit)
 *   BIT = bit index
 *   VAL = pre-defined values
 */

#define OD_TX_PDO_ID_VAL_A (uint16_t(0x6000))
#define OD_TX_PDO_ID_VAL_B (uint16_t(0x6001))
#define OD_TX_PDO_ID_VAL_C (uint16_t(0x6002))

#define OD_RX_PDO_ID_VAL_A (uint16_t(0x7000))
#define OD_RX_PDO_ID_VAL_B (uint16_t(0x7001))
#define OD_RX_PDO_ID_VAL_C (uint16_t(0x7002))

#define OD_IDENTITY_ID (uint16_t(0x1018))
#define OD_IDENTITY_SID_VENDOR_ID (uint16_t(0x01))
#define OD_IDENTITY_SID_PRODUCT_CODE (uint16_t(0x02))
#define OD_IDENTITY_SID_REVISION_NUMBER (uint16_t(0x03))
#define OD_IDENTITY_SID_SERIAL_NUMBER (uint16_t(0x04))

#define OD_FORCE_TORQUE_OFFSET_ID (uint16_t(0x8001))
#define OD_ACCELERATION_OFFSET_ID (uint16_t(0x8002))
#define OD_ANGULAR_RATE_OFFSET_ID (uint16_t(0x8003))
#define OD_ACCELERATION_RANGE_ID (uint16_t(0x8004))
#define OD_ANGULAR_RATE_RANGE_ID (uint16_t(0x8005))

#define OD_FORCE_TORQUE_FILTER_ID (uint16_t(0x8006))
#define OD_FORCE_TORQUE_FILTER_SID_SINC_SIZE (uint8_t(0x01))
#define OD_FORCE_TORQUE_FILTER_SID_FIR_DISABLE (uint8_t(0x02))
#define OD_FORCE_TORQUE_FILTER_SID_FAST_ENABLE (uint8_t(0x03))
#define OD_FORCE_TORQUE_FILTER_SID_CHOP_ENABLE (uint8_t(0x04))

#define OD_ACCELERATION_FILTER_ID (uint16_t(0x8007))
#define OD_ANGULAR_RATE_FILTER_ID (uint16_t(0x8008))
#define OD_SAMPLING_RATE_ID (uint16_t(0x8011))
#define OD_COORDINATE_SYSTEM_CONFIGURATION_ID (uint16_t(0x8012))
#define OD_INERTIA_CONFIGURATION_ID (uint16_t(0x8013))
#define OD_RESET_TO_FACTORY_CONFIG_ID (uint16_t(0x802F))

#define OD_SENSOR_FORCE_TORQUE_OFFSET_ID (uint16_t(0x8000))
#define OD_SENSOR_FORCE_TORQUE_OFFSET_SID_1 (uint8_t(0x01))
#define OD_SENSOR_FORCE_TORQUE_OFFSET_SID_2 (uint8_t(0x02))
#define OD_SENSOR_FORCE_TORQUE_OFFSET_SID_3 (uint8_t(0x03))
#define OD_SENSOR_FORCE_TORQUE_OFFSET_SID_4 (uint8_t(0x04))
#define OD_SENSOR_FORCE_TORQUE_OFFSET_SID_5 (uint8_t(0x05))
#define OD_SENSOR_FORCE_TORQUE_OFFSET_SID_6 (uint8_t(0x06))

#define OD_SENSOR_CONFIGURATION_ID (uint16_t(0x8010))
#define OD_SENSOR_CONFIGURATION_SID_CALIBRATION_MATRIX_ACTIVE (uint8_t(0x01))
#define OD_SENSOR_CONFIGURATION_SID_TEMPERATURE_COMPENSATION (uint8_t(0x02))
#define OD_SENSOR_CONFIGURATION_SID_IMU_ACTIVE (uint8_t(0x03))
#define OD_SENSOR_CONFIGURATION_SID_COORD_SYSTEM_CONFIGURATION (uint8_t(0x04))
#define OD_SENSOR_CONFIGURATION_SID_INERTIA_COMPENSATION (uint8_t(0x05))
#define OD_SENSOR_CONFIGURATION_SID_ORIENTATION_ESTIMATION (uint8_t(0x06))

#define OD_CONTROL_ID (uint16_t(0x8030))
#define OD_CONTROL_SID_COMMAND (uint8_t(0x01))
#define OD_CONTROL_SID_STATUS (uint8_t(0x02))

#define OD_SENSOR_CALIBRATION_ID (uint16_t(0x2000))
#define OD_SENSOR_CALIBRATION_SID_PASSPHRASE (uint8_t(0x01))
#define OD_SENSOR_CALIBRATION_SID_CALIBRATION_MATRIX (uint8_t(0x02))
#define OD_SENSOR_CALIBRATION_SID_TEMPERATURE_CALIBRATION (uint8_t(0x26))
#define OD_SENSOR_CALIBRATION_SID_CALIBRATION_OFFSET (uint8_t(0x27))
#define OD_SENSOR_CALIBRATION_SID_ADC_RANGE (uint8_t(0x2D))
#define OD_SENSOR_CALIBRATION_SID_THERMISTOR_CALIBRATION (uint8_t(0x2E))
#define OD_SENSOR_CALIBRATION_SID_TEMPERATURE_GAIN (uint8_t(0x30))
#define OD_SENSOR_CALIBRATION_SID_ACCELERATION_OFFSET (uint8_t(0x36))
#define OD_SENSOR_CALIBRATION_SID_ANGULAR_RATE_OFFSET (uint8_t(0x3C))
#define OD_SENSOR_CALIBRATION_SID_SENSOR_INTERTIA (uint8_t(0x42))
#define OD_SENSOR_CALIBRATION_SID_CALIBRATION_GAIN (uint8_t(0x4C))