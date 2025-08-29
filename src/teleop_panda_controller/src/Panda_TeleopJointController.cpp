#include <joint_wall.h>
#include <Panda_TeleopJointController.h>

#include <hardware_interface/hardware_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include <pseudo_inversion.h>

#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <fstream>
#include <iostream>
#include <filesystem>

// initiate joint vector
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector3d = Eigen::Matrix<double, 3, 1>;

Vector3d position;
Eigen::Quaterniond orientation;

// initiate controller name
const std::string kControllerName = "Panda_TeleopJointController";

namespace franka_example_controllers
{

  int demo_num;

  size_t timestamp_for_data_recording = 0; // for manually timestamp recording
  // function for initiating the state of panda roboter
  bool Panda_TeleopJointController::init(hardware_interface::RobotHW *robot_hw,
                                         ros::NodeHandle &node_handle)
  {
    // for data recording

    node_handle.getParam("/demo_num", demo_num);

    // std::cout<< node_handle.getParam("/demo_num", demo_num) << std::endl;
    // std::cout<< "demo number is: "<< demo_num << std::endl;

    std::string follower_arm_id;
    std::vector<std::string> follower_joint_names;
    // find parameters
    try
    {
      k_p_follower_ = get7dParam("follower/p_gains", node_handle);
      k_d_follower_ = get7dParam("follower/d_gains", node_handle);
      k_dq_ = get7dParam("follower/drift_comp_gains", node_handle);
      dq_max_lower_ = get7dParam("follower/dq_max_lower", node_handle);
      dq_max_upper_ = get7dParam("follower/dq_max_upper", node_handle);
      ddq_max_lower_ = get7dParam("follower/ddq_max_lower", node_handle);
      ddq_max_upper_ = get7dParam("follower/ddq_max_upper", node_handle);

      // Init for publishers
      // auto init_pub_follower = [&node_handle](auto &publisher, const auto &topic)
      // {
      //   publisher.init(node_handle, topic, 1);
      //   publisher.lock();
      //   publisher.msg_.name.resize(6);
      //   publisher.msg_.position.resize(6); // follower_data_.q
      //   publisher.msg_.velocity.resize(6); // k_p_follower_
      //   publisher.msg_.effort.resize(6);   // follower_tau_ext_hat
      //   publisher.unlock();
      // };

      // init_pub_follower(follower_data_pub_, "/follower_data");
      // follower_contact_pub_.init(node_handle, "/follower_contact", 1);

      auto init_pub_follower = [&node_handle](auto &publisher, const auto &topic)
      {
        publisher.init(node_handle, topic, 1);
        publisher.lock();

        publisher.unlock();
      };

      init_pub_follower(arm_pose_pub_, "/arm_pose"); // publish the pose of the arm

      // subscribe the topic from leader
      sub_leader_data = node_handle.subscribe("/leader_data", 1, &Panda_TeleopJointController::leaderDataCallback, this);
      sub_model_data = node_handle.subscribe("/model_prediction", 1, &Panda_TeleopJointController::ModelDataCallback, this);

      follower_data_.contact_force_threshold =
          get1dParam<double>("follower/contact_force_threshold", node_handle);

      follower_arm_id = get1dParam<std::string>("follower/arm_id", node_handle);

      follower_joint_names = getJointParams<std::string>("follower/joint_names", node_handle);

      if (!node_handle.getParam("debug", debug_))
      {
        ROS_INFO_STREAM_NAMED(kControllerName, "Could not find parameter debug. Defaulting to "
                                                   << std::boolalpha << debug_);
      }

      // Init for each arm
      // initArm(robot_hw, node_handle, leader_data_, leader_arm_id, leader_joint_names);
      initArm(robot_hw, node_handle, follower_data_, follower_arm_id, follower_joint_names);
    }
    catch (const std::invalid_argument &ex)
    {
      ROS_ERROR_NAMED(kControllerName, "%s", ex.what());
      return false;
    }

    if (debug_)
    {
      // Init for dynamic reconfigure
      dynamic_reconfigure_teleop_param_node_ = ros::NodeHandle("dyn_reconf_teleop_param_node");
      dynamic_server_teleop_param_ = std::make_unique<
          dynamic_reconfigure::Server<franka_example_controllers::teleop_paramConfig>>(
          dynamic_reconfigure_teleop_param_node_);
      dynamic_server_teleop_param_->setCallback(
          boost::bind(&Panda_TeleopJointController::teleopParamCallback, this, _1, _2));

      // // Init for publishers
      // auto init_pub = [&node_handle](auto &publisher, const auto &topic)
      // {
      //   publisher.init(node_handle, topic, 1);
      //   publisher.lock();
      //   publisher.msg_.name.resize(7);
      //   publisher.msg_.position.resize(7);
      //   publisher.msg_.velocity.resize(7);
      //   publisher.msg_.effort.resize(7);
      //   publisher.unlock();
      // };

      // init_pub(follower_data_pub_, "follower_data");

      follower_contact_pub_.init(node_handle, "follower_contact", 1);
      marker_pub_.init(node_handle, "marker_labels", 1, true);

      auto get_marker = [](const std::string &arm_id, int32_t id, const std::string &text)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = arm_id + "_link0";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.text = text;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.1;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        return marker;
      };

      {
        std::lock_guard<realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray>> lock(
            marker_pub_);
        // marker_pub_.msg_.markers.push_back(get_marker(leader_arm_id, 1, "leader"));
        marker_pub_.msg_.markers.push_back(get_marker(follower_arm_id, 2, "follower"));
      }
      publishMarkers();
    }

    position_d_in_cartesian.setZero();
    orientation_d_in_cartesian.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_in_cartesian.setZero();
    orientation_d_target_in_cartesian.coeffs() << 0.0, 0.0, 0.0, 1.0;

    return true;
  }

  // function for initiating the arm of panda roboter
  void Panda_TeleopJointController::initArm(hardware_interface::RobotHW *robot_hw,
                                            ros::NodeHandle &node_handle,
                                            FrankaDataContainer &arm_data,
                                            const std::string &arm_id,
                                            const std::vector<std::string> &joint_names)
  {

    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (not effort_joint_interface)
    {
      throw std::invalid_argument(kControllerName +
                                  ": Error getting effort joint interface from hardware of " +
                                  arm_id + ".");
    }

    arm_data.joint_handles.clear();
    for (const auto &name : joint_names)
    {
      try
      {
        arm_data.joint_handles.push_back(effort_joint_interface->getHandle(name));
      }
      catch (const hardware_interface::HardwareInterfaceException &e)
      {
        throw std::invalid_argument(kControllerName +
                                    ": Exception getting joint handle: " + std::string(e.what()));
      }
    }

    // Get state interface.
    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (not state_interface)
    {
      throw std::invalid_argument(kControllerName + ": Error getting state interface from hardware.");
    }

    try
    {
      arm_data.state_handle = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      throw std::invalid_argument(
          kControllerName +
          ": Exception getting state handle from interface: " + std::string(ex.what()));
    }

    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Error getting model interface from hardware");
    }
    try
    {
      arm_data.model_handle = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting model handle from interface: "
          << ex.what());
    }

    // Setup joint walls
    // Virtual joint position wall parameters
    const std::array<double, 7> kPDZoneWidth = {{0.12, 0.09, 0.09, 0.09, 0.0349, 0.0349, 0.0349}};
    const std::array<double, 7> kDZoneWidth = {{0.12, 0.09, 0.09, 0.09, 0.0349, 0.0349, 0.0349}};
    const std::array<double, 7> kPDZoneStiffness = {
        {2000.0, 2000.0, 1000.0, 1000.0, 500.0, 200.0, 200.0}};
    const std::array<double, 7> kPDZoneDamping = {{30.0, 30.0, 30.0, 10.0, 5.0, 5.0, 5.0}};
    const std::array<double, 7> kDZoneDamping = {{30.0, 30.0, 30.0, 10.0, 5.0, 5.0, 5.0}};

    std::array<double, 7> upper_joint_soft_limit;
    std::array<double, 7> lower_joint_soft_limit;
    getJointLimits(node_handle, joint_names, upper_joint_soft_limit, lower_joint_soft_limit);

    arm_data.virtual_joint_wall = std::make_unique<JointWallContainer<7>>(
        upper_joint_soft_limit, lower_joint_soft_limit, kPDZoneWidth, kDZoneWidth, kPDZoneStiffness,
        kPDZoneDamping, kDZoneDamping);
  }

  // function for starting the state of panda roboter
  void Panda_TeleopJointController::starting(const ros::Time & /*time*/)
  {

    start_time_ = ros::Time::now(); // this should be placed to the line that start the robot by pressing enter

    // Reset joint walls to start from the current q, dq
    follower_data_.virtual_joint_wall->reset();

    // Reset stored states to the current states.
    franka::RobotState initial_follower_robot_state = follower_data_.state_handle->getRobotState();
    q_initial = Eigen::Map<Vector7d>(initial_follower_robot_state.q.data());

    std::array<double, 42> jacobian_array = follower_data_.model_handle->getZeroJacobian(franka::Frame::kEndEffector);

    // Transformation matrix introduced due to the offset between robot EE and allegrohand
    T_offset << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::Affine3d T_matrix_follower_initial(Eigen::Matrix4d::Map(initial_follower_robot_state.O_T_EE.data()));

    // save initial position for circle
    initial_position_ = T_matrix_follower_initial.translation();

    T_matrix_follower_initial = T_matrix_follower_initial * T_offset;

    position_d_in_cartesian = T_matrix_follower_initial.translation();
    orientation_d_in_cartesian = Eigen::Quaterniond(T_matrix_follower_initial.rotation());
    position_d_target_in_cartesian = T_matrix_follower_initial.translation();
    orientation_d_target_in_cartesian = Eigen::Quaterniond(T_matrix_follower_initial.rotation());

    q_d_nullspace = q_initial;

    std::cout << T_offset << std::endl;
    // Store alignment position from leader
    current_state_ = TeleopStateMachine::ALIGN;
    if (debug_)
    {
      publishMarkers();
    }
  }

  // function for updating the state of panda roboter
  void Panda_TeleopJointController::update(const ros::Time & /*time*/,
                                           const ros::Duration &period)
  {
    // Determine whether the seconds of the timestamp have changed
    topic_received = (timestamp != last_timestamp);

    // std::cout<< "timestamp: " << std::endl;

    if (topic_received)
    {
      last_timestamp = timestamp;
    }

    // get all current roboter states
    franka::RobotState follower_robot_state = follower_data_.state_handle->getRobotState();
    follower_data_.q = Eigen::Map<Vector7d>(follower_robot_state.q.data());
    follower_data_.dq = Eigen::Map<Vector7d>(follower_robot_state.dq.data());

    std::array<double, 7> coriolis_array = follower_data_.model_handle->getCoriolis(); // this two lines may have error
    std::array<double, 42> jacobian_array = follower_data_.model_handle->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

    Eigen::Affine3d T_matrix_follower(Eigen::Matrix4d::Map(follower_robot_state.O_T_EE.data()));

    T_matrix_follower = T_matrix_follower * T_offset; // this offset for position difference between flange
    // Cartesian position
    Eigen::Vector3d position_follower(T_matrix_follower.translation());

    Eigen::Quaterniond orientation_follower(T_matrix_follower.rotation());

    position = position_follower; // assign position to global variable for ros communication
    orientation = orientation_follower;

    // orientation_follower.normalize();
    //  // Cartesian orientation (from rotation matrix to roll, pitch, yaw)
    Eigen::Matrix3d rotation_follower = T_matrix_follower.rotation();
    Eigen::Vector3d euler_angles_follower = rotation_follower.eulerAngles(0, 1, 2); // Roll, pitch, yaw

    // Cartesian velocity
    follower_EE_velocity = jacobian * follower_data_.dq;

    if ((ros::Time::now() - start_time_).toSec() < 8 || !is_model_prediction_)
    {
      // leader_data_in_cartesian= leader_data_in_cartesian_temp;
      position_d_target_in_cartesian = position_d_target_in_cartesian + period.toSec() * leader_data_in_cartesian.head(3);
      // std::cout << position_d_target_in_cartesian.transpose() << std::endl;
      // std::cout << " Teleoperation   " << std::endl;
    }
    else
    {
      position_d_target_in_cartesian = leader_data_in_cartesian.head(3);
      // std::cout << position_d_target_in_cartesian.transpose() << std::endl;
      // std::cout << "    " << std::endl;
    }


    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position_d_in_cartesian - position_follower;

    Eigen::Quaterniond error_quaternion(orientation_follower.inverse() * orientation_d_in_cartesian);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << T_matrix_follower.rotation() * error.tail(3);

    external_force_torque = Eigen::Map<Vector6d>(follower_robot_state.O_F_ext_hat_K.data());

    // Data recording
    std::stringstream ss_demo_num;
    ss_demo_num << demo_num;

    std::string directory = "/home/msrm/Sitong/test_Follower_TeleopJointController/src/teleop_panda_controller/Dataset_Follower_Arm/";
    std::string filename = "Follower_Arm_data_";
    std::string fullPath = directory + filename + ss_demo_num.str() + ".txt";

    outfile.open(fullPath, std::ios::out | std::ios::app);

    if (outfile.tellp() == 0)
    {
      // File is empty, write the header
      outfile << "Time, pos_x, pos_y, pos_z, Ori_w, Ori_x, Ori_y, Ori_z, vx, vy, vz, wx, wy, wz, Roll, Pitch, Yaw, d_pos_x, d_pos_y, d_pos_z, d_Ori_w, d_Ori_x, d_Ori_y, d_Ori_z, F_x, F_y, F_z, Torque_x, Torque_y, Torque_z" << std::endl;
    }

    outfile << timestamp_for_data_recording << ", "
            << position_follower[0] << ", " << position_follower[1] << ", " << position_follower[2] << ", "
            << orientation_follower.w() << ", " << orientation_follower.x() << ", "
            << orientation_follower.y() << ", " << orientation_follower.z() << ", "

            << follower_EE_velocity[0] << ", " << follower_EE_velocity[1] << ", "
            << follower_EE_velocity[2] << ", " << follower_EE_velocity[3] << ", "
            << follower_EE_velocity[4] << ", " << follower_EE_velocity[5] << ", "

            << euler_angles_follower[0] << ", " << euler_angles_follower[1] << ", "
            << euler_angles_follower[2] << ", "
            << position_d_in_cartesian[0] << ", " << position_d_in_cartesian[1] << ", " << position_d_in_cartesian[2] << ", "
            << orientation_d_in_cartesian.w() << ", " << orientation_d_in_cartesian.x() << ", "
            << orientation_d_in_cartesian.y() << ", " << orientation_d_in_cartesian.z() << ","
            << external_force_torque[0] << "," << external_force_torque[1] << "," << external_force_torque[2] << ","
            << external_force_torque[3] << "," << external_force_torque[4] << "," << external_force_torque[5] << ","

            << std::endl;

    outfile.close();

    if (topic_received)
    {
      // set current state to TRACK if the errror is smaller than tolerance
      if (current_state_ == TeleopStateMachine::ALIGN)
      {
        // Check coefficient-wise if the two robots are aligned
        const auto kNorm = (leader_data_q - follower_data_.q).cwiseAbs().array();
        if ((kNorm < kAlignmentTolerance_).all())
        {
          current_state_ = TeleopStateMachine::TRACK;
          ROS_INFO_STREAM_NAMED(kControllerName, "Leader and follower are aligned");
        }
      }

      Vector6d follower_f_ext_hat = Eigen::Map<Vector6d>(follower_robot_state.K_F_ext_hat_K.data());
      follower_data_.f_ext_norm = follower_f_ext_hat.head(3).norm();
      follower_data_.contact =
          rampParameter(follower_data_.f_ext_norm, 1.0, 0.0, follower_data_.contact_force_threshold,
                        follower_data_.contact_ramp_increase);

      // Determine max velocities and accelerations of follower arm depending on tracking errors to
      // avoid jumps and high velocities when starting example.
      Vector7d q_deviation = (q_target_last_ - leader_data_q).cwiseAbs();
      Vector7d dq_max;
      Vector7d ddq_max;

      if (current_state_ == TeleopStateMachine::ALIGN)
      {
        dq_max = dq_max_align_;
        ddq_max = ddq_max_align_;
        prev_alignment_error_ = alignment_error_;
        alignment_error_ = (init_leader_q_ - follower_data_.q);
        Vector7d dalignment_error = (alignment_error_ - prev_alignment_error_) / period.toSec(); // rate of alignment errors

        dq_unsaturated_ = k_p_follower_align_.asDiagonal() * alignment_error_ +
                          k_d_follower_align_.asDiagonal() * dalignment_error;
      }
      else
      {
        for (size_t i = 0; i < 7; ++i)
        {
          dq_max[i] = rampParameter(q_deviation[i], dq_max_lower_[i], dq_max_upper_[i],
                                    velocity_ramp_shift_, velocity_ramp_increase_);
          ddq_max[i] = rampParameter(q_deviation[i], ddq_max_lower_[i], ddq_max_upper_[i],
                                     velocity_ramp_shift_, velocity_ramp_increase_);
        }
        dq_unsaturated_ = k_dq_.asDiagonal() * (leader_data_q - q_target_last_) + leader_data_dq;
      }

      // in cartesian space
      dq_target_ = saturateAndLimit(dq_unsaturated_, dq_target_last_, dq_max, ddq_max, period.toSec());
      dq_target_last_ = dq_target_;

      if (!follower_robot_state.current_errors)
      {
        // Compute force-feedback for the leader arm to render the haptic interaction of the follower
        // robot. Add a slight damping to reduce vibrations.
        // The force feedback is applied when the external forces on the follower arm exceed a
        // threshold. While the leader arm is unguided (not in contact), the force-feedback is
        // reduced. When the leader robot exceeds the soft limit velocities dq_max_leader_lower
        // damping is increased gradually until it saturates when reaching dq_max_leader_upper to
        // maximum damping.

        follower_tau_ext_hat =
            Eigen::Map<Vector7d>(follower_robot_state.tau_ext_hat_filtered.data());
        // Compute PD control for the follower arm to track the leader's motions.
        k_p_follower_cartesian << 900.0, 900.0, 300.0, 50.0, 50.0, 25.0; // origin is 50 50 25  for testing, use kp=9 and kd=6,  kp_position=900, kp_orientation a too aggressive
        k_d_follower_cartesian << 60.0, 60.0, 40.0, 15.0, 15.0, 10.0;    // origin is 15 15 10
        // change the name to tau desired: k_p_follower need to change to 6D
        Eigen::MatrixXd jacobian_transpose_pinv;
        pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

        tau_in_cartesian = jacobian.transpose() * (follower_stiffness_scaling_ * k_p_follower_cartesian.asDiagonal() * error +
                                                   sqrt(follower_stiffness_scaling_) * k_d_follower_cartesian.asDiagonal() *
                                                       (-follower_EE_velocity)); // 0- velocity

        tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                          jacobian.transpose() * jacobian_transpose_pinv) *
                             (nullspace_stiffness_ * (q_d_nullspace - follower_data_.q) -
                              (2.0 * sqrt(nullspace_stiffness_)) * follower_data_.dq);

        follower_data_.tau_target = tau_in_cartesian + tau_nullspace + coriolis;

        position_d_in_cartesian = filter_params_ * position_d_target_in_cartesian + (1.0 - filter_params_) * position_d_in_cartesian;
        orientation_d_in_cartesian = orientation_d_in_cartesian.slerp(filter_params_, orientation_d_target_in_cartesian);
      }
      else
      {
        // Control target torques to zero if any arm is in error state.
        follower_data_.tau_target = decrease_factor_ * follower_data_.tau_target_last;
      }

      // Add torques from joint walls to the torque commands.
      auto to_eigen = [](const std::array<double, 7> &data)
      { return Vector7d(data.data()); };
      auto from_eigen = [](const Vector7d &data)
      {
        return std::array<double, 7>{data(0), data(1), data(2), data(3), data(4), data(5), data(6)};
      };

      std::array<double, 7> virtual_wall_tau_follower =
          follower_data_.virtual_joint_wall->computeTorque(from_eigen(follower_data_.q),
                                                           from_eigen(follower_data_.dq));

      // leader_data_.tau_target += to_eigen(virtual_wall_tau_leader);
      follower_data_.tau_target += to_eigen(virtual_wall_tau_follower);

      // Store torques for next time step
      follower_data_.tau_target_last = follower_data_.tau_target;

      // reset
      topic_received = false;
    }
    else
    {
      follower_data_.tau_target << 0, 0, 0, 0, 0, 0, 0;
    }
    updateArm(follower_data_);

    if (true)
    {
      publishFollowerData();
      publishFollowerContact();
    }

    timestamp_for_data_recording = timestamp_for_data_recording + 1;
    counter = counter + period;
  }

  void Panda_TeleopJointController::updateArm(FrankaDataContainer &arm_data)
  {
    for (size_t i = 0; i < 7; ++i)
    {
      arm_data.joint_handles[i].setCommand(arm_data.tau_target[i]);
    }
  }

  Eigen::Matrix<double, 7, 1> Panda_TeleopJointController::saturateAndLimit(const Vector7d &x_calc,
                                                                            const Vector7d &x_last,
                                                                            const Vector7d &x_max,
                                                                            const Vector7d &dx_max,
                                                                            const double delta_t)
  {
    Vector7d x_limited;
    for (size_t i = 0; i < 7; i++)
    {
      double delta_x_max = dx_max[i] * delta_t;
      double diff = x_calc[i] - x_last[i];
      double x_saturated = x_last[i] + std::max(std::min(diff, delta_x_max), -delta_x_max);
      x_limited[i] = std::max(std::min(x_saturated, x_max[i]), -x_max[i]);
    }
    return x_limited;
  }

  double Panda_TeleopJointController::rampParameter(const double x,
                                                    const double neg_x_asymptote,
                                                    const double pos_x_asymptote,
                                                    const double shift_along_x,
                                                    const double increase_factor)
  {
    double ramp =
        0.5 * (pos_x_asymptote + neg_x_asymptote -
               (pos_x_asymptote - neg_x_asymptote) * tanh(increase_factor * (x - shift_along_x)));
    return ramp;
  }

  void Panda_TeleopJointController::teleopParamCallback(
      franka_example_controllers::teleop_paramConfig &config,
      uint32_t /*level*/)
  {
    if (dynamic_reconfigure_mutex_.try_lock())
    {
      follower_stiffness_scaling_ = config.follower_stiffness_scaling;
      force_feedback_guiding_ = config.force_feedback_guiding;
      force_feedback_idle_ = config.force_feedback_idle;
      follower_data_.contact_force_threshold = config.follower_contact_force_threshold;

      dq_max_lower_[0] = config.dq_l_1;
      dq_max_lower_[1] = config.dq_l_2;
      dq_max_lower_[2] = config.dq_l_3;
      dq_max_lower_[3] = config.dq_l_4;
      dq_max_lower_[4] = config.dq_l_5;
      dq_max_lower_[5] = config.dq_l_6;
      dq_max_lower_[6] = config.dq_l_7;

      dq_max_upper_[0] = config.dq_u_1;
      dq_max_upper_[1] = config.dq_u_2;
      dq_max_upper_[2] = config.dq_u_3;
      dq_max_upper_[3] = config.dq_u_4;
      dq_max_upper_[4] = config.dq_u_5;
      dq_max_upper_[5] = config.dq_u_6;
      dq_max_upper_[6] = config.dq_u_7;

      ddq_max_lower_[0] = config.ddq_l_1;
      ddq_max_lower_[1] = config.ddq_l_2;
      ddq_max_lower_[2] = config.ddq_l_3;
      ddq_max_lower_[3] = config.ddq_l_4;
      ddq_max_lower_[4] = config.ddq_l_5;
      ddq_max_lower_[5] = config.ddq_l_6;
      ddq_max_lower_[6] = config.ddq_l_7;

      ddq_max_upper_[0] = config.ddq_u_1;
      ddq_max_upper_[1] = config.ddq_u_2;
      ddq_max_upper_[2] = config.ddq_u_3;
      ddq_max_upper_[3] = config.ddq_u_4;
      ddq_max_upper_[4] = config.ddq_u_5;
      ddq_max_upper_[5] = config.ddq_u_6;
      ddq_max_upper_[6] = config.ddq_u_7;

      ROS_INFO_NAMED(kControllerName, "Dynamic reconfigure: Controller params set.");
    }
    dynamic_reconfigure_mutex_.unlock();
  }

  void Panda_TeleopJointController::getJointLimits(ros::NodeHandle &nh,
                                                   const std::vector<std::string> &joint_names,
                                                   std::array<double, 7> &upper_joint_soft_limit,
                                                   std::array<double, 7> &lower_joint_soft_limit)
  {
    const std::string &node_namespace = nh.getNamespace();
    std::size_t found = node_namespace.find_last_of('/');
    std::string parent_namespace = node_namespace.substr(0, found);

    if (!nh.hasParam(parent_namespace + "/robot_description"))
    {
      throw std::invalid_argument(kControllerName + ": No parameter robot_description (namespace: " +
                                  parent_namespace + ")found to set joint limits!");
    }

    urdf::Model urdf_model;
    if (!urdf_model.initParamWithNodeHandle(parent_namespace + "/robot_description", nh))
    {
      throw std::invalid_argument(kControllerName +
                                  ": Could not initialize urdf model from robot_description "
                                  "(namespace: " +
                                  parent_namespace + ").");
    }

    joint_limits_interface::SoftJointLimits soft_limits;
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      const std::string &joint_name = joint_names.at(i);
      auto urdf_joint = urdf_model.getJoint(joint_name);
      if (!urdf_joint)
      {
        ROS_ERROR_STREAM_NAMED(kControllerName,
                               ": Could not get joint " << joint_name << " from urdf");
      }
      if (!urdf_joint->safety)
      {
        ROS_ERROR_STREAM_NAMED(kControllerName, ": Joint " << joint_name << " has no limits");
      }
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
      {
        upper_joint_soft_limit[i] = soft_limits.max_position;
        lower_joint_soft_limit[i] = soft_limits.min_position;
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(kControllerName, ": Could not parse joint limit for joint "
                                                    << joint_name << " for joint limit interfaces");
      }
    }
  }

  void Panda_TeleopJointController::leaderDataCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {

    Vector7d leader_data_in_cartesian_temp;

    teleoperation_time_ = ros::Time::now();

    if ((teleoperation_time_ - start_time_).toSec() < 8) // after 5 seconds, teleoperation is cut off

    {
      leader_data_in_cartesian[0] = msg->pose.position.x; // it is velocity data
      leader_data_in_cartesian[1] = msg->pose.position.y;
      leader_data_in_cartesian[2] = msg->pose.position.z;
      leader_data_in_cartesian[3] = msg->pose.orientation.x;
      leader_data_in_cartesian[4] = msg->pose.orientation.y;
      leader_data_in_cartesian[5] = msg->pose.orientation.z;
      leader_data_in_cartesian[6] = msg->pose.orientation.w;
      is_teleoperation_ = true;
      // std::cout << "Teleoperation Mode: " << std::endl;
    }
    // else
    // {
    //   leader_data_in_cartesian_temp[0] = 0.0; // if the teleoperation time is over, set the data to zero
    //   leader_data_in_cartesian_temp[1] = 0.0;
    //   leader_data_in_cartesian_temp[2] = 0.0;
    //   leader_data_in_cartesian_temp[3] = 0.0;
    //   leader_data_in_cartesian_temp[4] = 0.0;
    //   leader_data_in_cartesian_temp[5] = 0.0;
    //   leader_data_in_cartesian_temp[6] = 1.0;
    // }

    quaternion_delta.coeffs() << leader_data_in_cartesian[3], leader_data_in_cartesian[4],
        leader_data_in_cartesian[5], leader_data_in_cartesian[6];

    quaternion_delta.normalize();

    Eigen::Quaterniond last_orientation_d_target_in_cartesian(orientation_d_target_in_cartesian); // save the quaternion for the last timestamp

    // quaternion_result = quaternion_delta * orientation_d_target_in_cartesian * quaternion_delta.conjugate();
    quaternion_result = quaternion_delta * orientation_d_target_in_cartesian;
    quaternion_result.normalize();

    orientation_d_target_in_cartesian.coeffs() << quaternion_result.x(), quaternion_result.y(),
        quaternion_result.z(), quaternion_result.w();

    if (last_orientation_d_target_in_cartesian.coeffs().dot(orientation_d_target_in_cartesian.coeffs()) < 0.0)
    {
      orientation_d_target_in_cartesian.coeffs() << -orientation_d_target_in_cartesian.coeffs();
    }

    // std::cout<<"position x: "<<leader_data_in_cartesian[0]<<std::endl;
    // std::cout<<"position y: "<<leader_data_in_cartesian[1]<<std::endl;
    // std::cout<<" "<<std::endl;

    timestamp = msg->header.stamp;
  }

  void Panda_TeleopJointController::ModelDataCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {

    teleoperation_time_ = ros::Time::now();

    Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaterniond orientation(msg->pose.orientation.w, msg->pose.orientation.x,
                                   msg->pose.orientation.y, msg->pose.orientation.z);

    if ((teleoperation_time_ - start_time_).toSec() >= 8) // after 5 seconds, teleoperation is cut off

    {
      leader_data_in_cartesian[0] = 0.05 * position.x() + initial_position_.x(); // it is velocity data
      leader_data_in_cartesian[1] = 0.02 * position.y() + initial_position_.y();
      leader_data_in_cartesian[2] = 0.47;

      // leader_data_in_cartesian[0] = 0.05 * position.x(); // it is velocity data
      // leader_data_in_cartesian[1] = 0.02 * position.y();
      // leader_data_in_cartesian[2] = 0;
      leader_data_in_cartesian[3] = msg->pose.orientation.x;
      leader_data_in_cartesian[4] = msg->pose.orientation.y;
      leader_data_in_cartesian[5] = msg->pose.orientation.z;
      leader_data_in_cartesian[6] = msg->pose.orientation.w;

      is_model_prediction_ = true;

      // std::cout << "Model Prediction Mode: " << std::endl;
    }
    // else
    // {
    //   model_data_in_cartesian_temp[0] = 0.0; // if the teleoperation time is over, set the data to zero
    //   model_data_in_cartesian_temp[1] = 0.0;
    //   model_data_in_cartesian_temp[2] = 0.0;
    //   model_data_in_cartesian_temp[3] = 0.0;
    //   model_data_in_cartesian_temp[4] = 0.0;
    //   model_data_in_cartesian_temp[5] = 0.0;
    //   model_data_in_cartesian_temp[6] = 1.0;
    // }

    // {
    //   static ros::Time last_time = ros::Time::now();
    //   // ros::Duration min_interval(1.0 / 1000.0);

    //   // if ((ros::Time::now() - last_time) < min_interval) {
    //   //     return;
    //   // }
    //   last_time = ros::Time::now();

    //   Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    //   Eigen::Quaterniond orientation(msg->pose.orientation.w, msg->pose.orientation.x,
    //                                  msg->pose.orientation.y, msg->pose.orientation.z);

    //   position_d_target_.x() = 0.05 * position.x() + initial_position_.x();
    //   position_d_target_.y() = 0.02 * position.y() + initial_position_.y();
    //   position_d_target_.z() = 0.48;

    // std::lock_guard<std::mutex> position_d_target_mutex_lock(
    // position_and_orientation_d_target_mutex_);

    // std::cout<<"data received"<<std::endl;
    // timestamp = msg->header.stamp;
  }

  void Panda_TeleopJointController::publishFollowerData()
  {

    static int publish_counter = 0;
    int publish_rate = int(1000 / 80); // training data is 80 Hz

    // if (publish_counter % publish_rate == 0 && point_reached == true)
    if (publish_counter % publish_rate == 0)
    {

      if (arm_pose_pub_.trylock())
      {
        arm_pose_pub_.msg_.pose.position.x = position[0] - initial_position_[0];
        arm_pose_pub_.msg_.pose.position.y = position[1] - initial_position_[1];
        arm_pose_pub_.msg_.pose.position.z = position[2];
        arm_pose_pub_.msg_.pose.orientation.x = orientation.x();
        arm_pose_pub_.msg_.pose.orientation.y = orientation.y();
        arm_pose_pub_.msg_.pose.orientation.z = orientation.z();
        arm_pose_pub_.msg_.pose.orientation.w = orientation.w();

        // std::cout << "x_position: " << position[0] << std::endl;
        // std::cout << "y_position: " << position[1] << std::endl;

        // std::cout << "initial_x_position: " << initial_position_[0] << std::endl;
        // std::cout << "initial_y_position: " << initial_position_[1] << std::endl;

        // std::cout << arm_pose_pub_.msg_.pose.position.x << std::endl;
        // std::cout << arm_pose_pub_.msg_.pose.position.y << std::endl;
        // std::cout << "    " << std::endl;

        // arm_pose_pub_.msg_.header.stamp = counter;
        arm_pose_pub_.unlockAndPublish();
      }
    }
    publish_counter++;
  }

  void Panda_TeleopJointController::publishFollowerContact()
  {
    if (follower_contact_pub_.trylock())
    {
      follower_contact_pub_.msg_.data = follower_data_.contact;
      follower_contact_pub_.unlockAndPublish();
    }
  }

  Vector7d Panda_TeleopJointController::get7dParam(const std::string &param_name,
                                                   ros::NodeHandle &nh)
  {
    auto buffer = getJointParams<double>(param_name, nh);
    return Vector7d(Eigen::Map<Vector7d>(buffer.data()));
  }

  void Panda_TeleopJointController::publishMarkers()
  {
    marker_pub_.lock();
    marker_pub_.unlockAndPublish();
  }

}

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::Panda_TeleopJointController,
                       controller_interface::ControllerBase)