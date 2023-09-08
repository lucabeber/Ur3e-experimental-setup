#include <rokubimini/calibration/ForceTorqueCalibration.hpp>
#include <utility>

namespace rokubimini
{
namespace calibration
{
ForceTorqueCalibration::ForceTorqueCalibration()
{
  RCLCPP_INFO(rclcpp::get_logger("ForceTorqueCalibration"), "[rokubimini][ForceTorqueCalibration][constructor]");
}

ForceTorqueCalibration::~ForceTorqueCalibration()
{
}

void ForceTorqueCalibration::addMeasurement(const Eigen::Vector3d& acc, const Eigen::Vector3d& gravity,
                                            Eigen::Vector3d angVel, Eigen::Vector3d angAcc,
                                            const Eigen::VectorXd& ftRaw)
{
  numMeasurements_++;

  // From measurements, create measurement matrix as data point for LS
  Eigen::MatrixXd h = createMeasurementMat(acc, gravity, std::move(angVel), std::move(angAcc));

  // Add data point
  if (numMeasurements_ == 1)
  {
    measurementMat_ = h;
    ftReadings_ = ftRaw;
  }
  else
  {
    Eigen::MatrixXd h_temp = measurementMat_;
    Eigen::VectorXd z_temp = ftReadings_;

    measurementMat_.resize(numMeasurements_ * 6, 10);
    ftReadings_.resize(numMeasurements_ * 6);

    measurementMat_.topRows((numMeasurements_ - 1) * 6) = h_temp;
    ftReadings_.topRows((numMeasurements_ - 1) * 6) = z_temp;

    measurementMat_.bottomRows(6) = h;
    ftReadings_.bottomRows(6) = ftRaw;
  }
}

Eigen::MatrixXd ForceTorqueCalibration::createMeasurementMat(const Eigen::Vector3d& acc, const Eigen::Vector3d& gravity,
                                                             Eigen::Vector3d angVel, Eigen::Vector3d angAcc)
{
  Eigen::MatrixXd h = Eigen::Matrix<double, 6, 10>::Zero();

  // Initialize skew symmetric matrices
  Eigen::Matrix3d ang_vel_skew = skewMatrix(std::move(angVel));
  Eigen::Matrix3d ang_acc_skew = skewMatrix(std::move(angAcc));
  Eigen::Matrix3d acc_skew = skewMatrix(gravity - acc);
  Eigen::Matrix3d ang_pow2 = ang_vel_skew * ang_vel_skew;

  // Create identity matrix for the offsets
  for (int i = 0; i < 6; i++)
    h(i, i + 4) = 1.0;

  // Look at paper "RIGID BODY LOAD IDENTIFICATION FOR MANIPULATORS" for an explanation
  h.col(0).head(3) << (acc - gravity);

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      h(i, j + 1) = ang_pow2(i, j) + ang_acc_skew(i, j);
      h(i + 3, j + 1) = acc_skew(i, j);
    }
  }

  return h;
}

Eigen::Matrix3d ForceTorqueCalibration::skewMatrix(Eigen::Vector3d inVec)
{
  Eigen::Matrix3d skew_mat = Eigen::Matrix<double, 3, 3>::Zero();

  skew_mat(0, 1) = -inVec[2];
  skew_mat(0, 2) = inVec[1];
  skew_mat(1, 2) = -inVec[0];

  skew_mat(1, 0) = inVec[2];
  skew_mat(2, 0) = -inVec[1];
  skew_mat(2, 1) = inVec[0];

  return skew_mat;
}

Eigen::VectorXd ForceTorqueCalibration::getCalibParams()
{
  RCLCPP_INFO(rclcpp::get_logger("ForceTorqueCalibration"),
              "[rokubimini::ForceTorqueCalibration][getCalibParams] solve LS problem");
  Eigen::VectorXd calib_params = Eigen::VectorXd::Zero(6);

  calib_params = measurementMat_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(ftReadings_);

  return calib_params;
}

void ForceTorqueCalibration::resetCalibration()
{
  RCLCPP_INFO(rclcpp::get_logger("ForceTorqueCalibration"), "[rokubimini::ForceTorqueCalibration][resetCalibration]");
  numMeasurements_ = 0;
}
}  // namespace calibration
}  // namespace rokubimini
