#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include <cartesian_compliance_controller/cartesian_compliance_controller.h>
#include <iostream>

using namespace std;
namespace cartesian_compliance_controller
{

  CartesianComplianceController::CartesianComplianceController()
      // Base constructor won't be called in diamond inheritance, so call that
      // explicitly
      : Base::CartesianControllerBase(),
        MotionBase::CartesianMotionController(),
        ForceBase::CartesianForceController()
  {
  }

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE || true
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianComplianceController::on_init()
  {
    using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    if (MotionBase::on_init() != TYPE::SUCCESS || ForceBase::on_init() != TYPE::SUCCESS)
    {
      return TYPE::ERROR;
    }

    auto_declare<std::string>("compliance_ref_link", "");

    constexpr double default_lin_stiff = 500.0;
    constexpr double default_rot_stiff = 50.0;
    auto_declare<double>("stiffness.trans_x", default_lin_stiff);
    auto_declare<double>("stiffness.trans_y", default_lin_stiff);
    auto_declare<double>("stiffness.trans_z", default_lin_stiff);
    auto_declare<double>("stiffness.rot_x", default_rot_stiff);
    auto_declare<double>("stiffness.rot_y", default_rot_stiff);
    auto_declare<double>("stiffness.rot_z", default_rot_stiff);

    return TYPE::SUCCESS;
  }
#elif defined CARTESIAN_CONTROLLERS_FOXY
  controller_interface::return_type CartesianComplianceController::init(const std::string &controller_name)
  {
    using TYPE = controller_interface::return_type;
    if (MotionBase::init(controller_name) != TYPE::OK || ForceBase::init(controller_name) != TYPE::OK)
    {
      return TYPE::ERROR;
    }

    auto_declare<std::string>("compliance_ref_link", "");

    return TYPE::OK;
  }
#endif

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianComplianceController::on_configure(
      const rclcpp_lifecycle::State &previous_state)
  {
    using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    if (MotionBase::on_configure(previous_state) != TYPE::SUCCESS || ForceBase::on_configure(previous_state) != TYPE::SUCCESS)
    {
      return TYPE::ERROR;
    }

    // Make sure compliance link is part of the robot chain
    m_compliance_ref_link = get_node()->get_parameter("compliance_ref_link").as_string();
    if (!Base::robotChainContains(m_compliance_ref_link))
    {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          m_compliance_ref_link << " is not part of the kinematic chain from "
                                                << Base::m_robot_base_link << " to "
                                                << Base::m_end_effector_link);
      return TYPE::ERROR;
    }

    // Make sure sensor wrenches are interpreted correctly
    ForceBase::setFtSensorReferenceFrame(m_compliance_ref_link);
    
    return TYPE::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianComplianceController::on_activate(
      const rclcpp_lifecycle::State &previous_state)
  {
    // Base::on_activation(..) will get called twice,
    // but that's fine.
    using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    if (MotionBase::on_activate(previous_state) != TYPE::SUCCESS || ForceBase::on_activate(previous_state) != TYPE::SUCCESS)
    {
      return TYPE::ERROR;
    }
    // Subscriber
    m_ft_sensor_wrench_subscriber = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
        get_node()->get_name() + std::string("/ft_sensor_wrench"),
        10,
        std::bind(&CartesianComplianceController::ftSensorWrenchCallback, this, std::placeholders::_1));
    // Publisher
    m_data_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    std::string("/stiffness_data"), 10);
  // Publishers
    m_target_pose_publisher = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
    get_node()->get_name() + std::string("/target_frame"), 10);

    m_starting_pose(0) = MotionBase::m_current_frame.p.x();
    m_starting_pose(1) = MotionBase::m_current_frame.p.y();
    m_starting_pose(2) = MotionBase::m_current_frame.p.z();
    cout << "Starting position: "<< m_starting_pose(0) << " " << m_starting_pose(1) << " " << m_starting_pose(2) << endl;

    old_time = current_time = start_time = get_node()->get_clock()->now();
    Xt = 1.0;
    dXt = 0.0;
    tank_energy = old_tank_energy = 0.5 * Xt * Xt;
    tank_energy_threshold = 0.1;

    USING_NAMESPACE_QPOASES
    Options options;
    options.printLevel = PL_LOW;
    // redeclare solver with options
    min_problem = QProblem(3, 4);

    m_ft_sensor_wrench = ctrl::Vector3D::Zero();

    x_d_old << m_starting_pose(0), m_starting_pose(1), m_starting_pose(2);
    return TYPE::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianComplianceController::on_deactivate(
      const rclcpp_lifecycle::State &previous_state)
  {
    using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    if (MotionBase::on_deactivate(previous_state) != TYPE::SUCCESS || ForceBase::on_deactivate(previous_state) != TYPE::SUCCESS)
    {
      return TYPE::ERROR;
    }
    return TYPE::SUCCESS;
  }

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE
  controller_interface::return_type CartesianComplianceController::update(const rclcpp::Time &time,
                                                                          const rclcpp::Duration &period)
#elif defined CARTESIAN_CONTROLLERS_FOXY
  controller_interface::return_type CartesianComplianceController::update()
#endif
  {
    current_time = get_node()->get_clock()->now();
    if (abs((current_time - old_time).nanoseconds()) < rclcpp::Duration::from_seconds(0.0001).nanoseconds())
    {
      return controller_interface::return_type::OK;
    }
    // Synchronize the internal model and the real robot
    Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

    // Publish target_frame

    // Control the robot motion in such a way that the resulting net force
    // vanishes. This internal control needs some simulation time steps.
    for (int i = 0; i < Base::m_iterations; ++i)
    {
      // The internal 'simulation time' is deliberately independent of the outer
      // control cycle.
      auto internal_period = rclcpp::Duration::from_seconds(0.02);

      // Compute the net force
      ctrl::Vector6D error = computeComplianceError();
      // Turn Cartesian error into joint motion
      Base::computeJointControlCmds(error, internal_period);
    }

    // publish target frame
    // publishTargetFrame();

    // Write final commands to the hardware interface
    Base::writeJointControlCmds();
    old_time = current_time;
    x_d_old << MotionBase::m_target_frame.p.x(), MotionBase::m_target_frame.p.y(), MotionBase::m_target_frame.p.z();
    return controller_interface::return_type::OK;
  }

  void CartesianComplianceController::publishTargetFrame(){
    double timeFromStart = (current_time - start_time).nanoseconds() * 1e-9 ;
    int step = timeFromStart / step_seconds;

    geometry_msgs::msg::PoseStamped target_pose;
    if (step < 2){
      target_pose.pose.position.z = 0.122;
    }else{
      target_pose.pose.position.z = 0.122 - (step-2) * z_step;
    }

    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp = current_time;
    target_pose.pose.position.x = m_starting_pose(0);
    target_pose.pose.position.y = m_starting_pose(1);
    target_pose.pose.orientation.x = 1.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 0.0;

    m_target_pose_publisher->publish(target_pose);
  }

  ctrl::Vector6D CartesianComplianceController::computeComplianceError()
  {

    ctrl::Vector6D tmp = CartesianComplianceController::computeStiffness();
    tmp[3] = get_node()->get_parameter("stiffness.rot_x").as_double();
    tmp[4] = get_node()->get_parameter("stiffness.rot_y").as_double();
    tmp[5] = get_node()->get_parameter("stiffness.rot_z").as_double();

    m_stiffness = tmp.asDiagonal();

    ctrl::Vector6D net_force =

        // Spring force in base orientation
        Base::displayInBaseLink(m_stiffness, m_compliance_ref_link) * MotionBase::computeMotionError()

        // Sensor and target force in base orientation
        + ForceBase::computeForceError();

    return net_force;
  }

  void CartesianComplianceController::ftSensorWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
  {
    KDL::Wrench tmp;
    tmp[0] = wrench->wrench.force.x;
    tmp[1] = wrench->wrench.force.y;
    tmp[2] = wrench->wrench.force.z;

    m_ft_sensor_wrench(0) = tmp[0];
    m_ft_sensor_wrench(1) = tmp[1];
    m_ft_sensor_wrench(2) = tmp[2];
  }
  ctrl::Vector6D CartesianComplianceController::computeStiffness()
  {
    USING_NAMESPACE_QPOASES
    
    rclcpp::Duration deltaT_ros = current_time - old_time;
    double deltaT = deltaT_ros.nanoseconds() * 1e-9;
    // retrieve target position
    ctrl::Vector3D x;
    x(0) = MotionBase::m_current_frame.p.x();
    x(1) = MotionBase::m_current_frame.p.y();
    x(2) = MotionBase::m_current_frame.p.z();
    // retrieve current position
    ctrl::Vector3D x_d;
    x_d(0) = MotionBase::m_target_frame.p.x();
    x_d(1) = MotionBase::m_target_frame.p.y();
    x_d(2) = MotionBase::m_target_frame.p.z();

    // x_d and x are inverted in order to consider the force in the same direction to the material's force

    // retrieve current velocity
    ctrl::Vector6D xdot = Base::m_ik_solver->getEndEffectorVel();

    // retrieve material stiffness
    ctrl::Vector3D kl = {kl_, kl_, kl_};

    // define material steady-state position
    ctrl::Vector3D sl = {0.0, 0.0, 0.12};
    // define material current position
    ctrl::Vector3D l = {0.0, 0.0, 0.12};

    // retrieve material damping
    ctrl::Vector3D dl = {dl_, dl_, dl_};

    // F_ref
    ctrl::Vector3D F_ref = {0.0, 0.0, 0.0};

    if (x(2) < sl(2))
    {
      // penetrating material
      l(2) = x(2);
      kl = {kl_, kl_, kl_};
      dl = {dl_, dl_, dl_};
      F_ref(2) = - 15.0;
    }
    else
    {
      // free motion
      l(2) = sl(2);
      kl = {kl_, kl_, kl_};
      dl = {dl_, dl_, dl_};
      F_ref(2) = 0.0;
    }
    // Update energy
    // compute w_transpose * x_tilde_dot -> EQUATION 14-15
    ctrl::Vector3D velocity_error;
    velocity_error << (x_d(0)-x_d_old(0))/deltaT - xdot(0), 
                      (x_d(1)-x_d_old(1))/deltaT - xdot(1), 
                      (x_d(2)-x_d_old(2))/deltaT - xdot(2);


    real_t H[3 * 3] =
        {
            R(0) + Q(0) * pow(x(0) - x_d(0), 2), 0, 0,
            0, R(1) + Q(1) * pow(x(1) - x_d(1), 2), 0,
            0, 0, R(2) + Q(2) * pow(x(2) - x_d(2), 2)
        };


    // -Kmin1 R1 - Fdx Q1 x1 + kd1 (R1 + Q1 x1^2)
    real_t g[3] = {
      -kd_min(0) *  R(0) - F_ref(0) * (x_d(0) - x(0)) * Q(0), // + kd(0) * (R(0) + Q(0) * pow(x_d(0) - x(0),2)),  
      -kd_min(1) *  R(1) - F_ref(1) * (x_d(1) - x(1)) * Q(1), // + kd(1) * (R(1) + Q(1) * pow(x_d(1) - x(1),2)),
      -kd_min(2) *  R(2) - F_ref(2) * (x_d(2) - x(2)) * Q(2) // + kd(2) * (R(2) + Q(2) * pow(x_d(2) - x(2),2))
    };
      // (R(0) + Q(0) * pow(x_d(0) - x(0), 2)),
      // (R(1) + Q(1) * pow(x_d(1) - x(1), 2)),
      // (R(2) + Q(2) * pow(x_d(2) - x(2), 2))

    // Constraints on  K2
    real_t lb[3] = {kd_min(0), kd_min(1), kd_min(2)};
    real_t ub[3] = {kd_max(0), kd_max(1), kd_max(2)};

    // Tank equation
    //  T =
    //  Xt^2/2 + x_tilde_x*x_tilde_dot_x*(kd_x - kd_x_min) + x_tilde_y*x_tilde_dot_y*(kd_y - kd_y_min) + x_tilde_z*x_tilde_dot_z*(kd_z - kd_z_min)

    // extract x depent on Kd from T -> this goes in A as {x(0) * xdot(0) , x(1) * xdot(1) , x(2) * xdot(2)}
    // real_t tank_A = x(0) * xdot(0) + x(1) * xdot(1) + x(2) * xdot(2);

    // Constraint equation is:
    // A * x = b + epsilon -> I need to compute b + epsilon

    // Rewriting it as tank_A * Kd will left out the following term which must be sum in the left part of the equation (so I changed sign)
    // should be  = - Xt^2/2 + kd_x_min*x_tilde_x*x_tilde_dot_x + kd_y_min*x_tilde_y*x_tilde_dot_y + kd_z_min*x_tilde_z*x_tilde_dot_z + epsilon

    // Xt^2/2 is for the first step -> then became the old tank value

    real_t T_constr_min; 

    if (old_tank_energy < tank_energy_threshold)
    {
      // empty tank
      stiffness << kd_min(0), kd_min(1), kd_min(2), 50.0,50.0,50.0;
      energy_var_stiff = (x_d-x).transpose() * (kd - kd_min).asDiagonal() * velocity_error;
      tank_energy = tank_energy_threshold; // + (energy_var_stiff)*deltaT;
      return stiffness;
    }
    else
    {
      T_constr_min = kd_min(0) * x(0) * velocity_error(0) + kd_min(1) * x(1) * velocity_error(1) + kd_min(2) * x(2) * velocity_error(2) + (tank_energy_threshold-old_tank_energy)/deltaT;
    }
    
    real_t A[4 * 3] = {
      x_d(0) - x(0), 0, 0,
      0, x_d(1) - x(1), 0,
      0, 0, x_d(2) - x(2),
      (x_d(0) - x(0)) * velocity_error(0), (x_d(1) - x(1)) * velocity_error(1), (x_d(2) - x(2)) * velocity_error(2)
    };    
    
    real_t ubA[4] = {F_max(0), F_max(1), F_max(2), 9999999999999};
    real_t lbA[4] = {F_min(0), F_min(1), F_min(2), T_constr_min};

    F_min(2) = kl(2) * (-0.1) + dl(2) * xdot(2);



    int_t nWSR = 10;
    Options options;
    options.printLevel = PL_NONE;
    // redeclare solver with options
    min_problem.setOptions(options);
    min_problem.init(H, g, A, lb, ub, lbA, ubA, nWSR);

    real_t xOpt[3];
    int_t ret_val;

    ret_val = getSimpleStatus(min_problem.getPrimalSolution(xOpt));
    if (ret_val != SUCCESSFUL_RETURN){
      stiffness << kd_min(0), kd_min(1), kd_min(2), 50.0,50.0,50.0;
      energy_var_stiff = (x_d-x).transpose() * (kd - kd_min).asDiagonal() * velocity_error;
      tank_energy = tank_energy_threshold; // + (energy_var_stiff)*deltaT;
      return stiffness;
    }
    // cout << "ret_val: " << ret_val << endl;
    stiffness << xOpt[0], xOpt[1], xOpt[2], 50.0,50.0,50.0;
    kd << stiffness(0), stiffness(1), stiffness(2);

    


    // compute energy tank (previous + derivative of current*delta_T) -> EQUATION 16
    energy_var_stiff = (x_d - x).transpose() * ((kd - kd_min).asDiagonal()) * velocity_error;
    tank_energy = old_tank_energy + energy_var_stiff * deltaT;

    print_index++;
    if (print_index % 20 == 0)
    {
      cout << "#########################################################" << endl;
      cout << " stiffness : "<<xOpt[2];
      cout<<" KD-KMIN: "<<endl<< (kd-kd_min)<<endl;
      cout << "deltaX_ext: " << (x_d(2) - x(2)) << "  | deltaX_mat: " << -(sl(2) - l(2)) << endl;
      cout << "Kd: " << kd(0) << " " << kd(1) << " " << kd(2) << endl;
      cout << "Tank: " << tank_energy << " | Tank_dot: " << energy_var_stiff <<" |  threshold: " << T_constr_min << endl;
      cout << "F_ext: " << kd(2) * (x_d(2) - x(2)) << "|  F_des: " << F_ref(2) << "|  F_min: " <<F_min(2) << endl;
      // cout<< "X: "<< x(2) <<endl;
    }

    std_msgs::msg::Float64MultiArray m_data_msg;
    m_data_msg.data = {
      (current_time.nanoseconds() * 1e-9), // Time
      (x_d(2) - x(2)),  // deltaX_ext
      (sl(2) - l(2)), // deltaX_mat
      kd(2) * (x_d(2) - x(2)), // F_ext
      // (kl(2) * (sl(2) - l(2)) - dl(2) * xdot(2)), // F_mat
      F_ref(2), // F_ref
      m_ft_sensor_wrench(2),// F/T sensor
      kd(2), // Kd_z
      kd_min(2) // Kd_z min

    };
    m_data_publisher->publish(m_data_msg);
    
    old_tank_energy = tank_energy;
    return stiffness;
  }
} // namespace
// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_compliance_controller::CartesianComplianceController, controller_interface::ControllerInterface)
