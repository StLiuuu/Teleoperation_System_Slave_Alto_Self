#pragma once

#include <joint_wall.h>
#include <teleop_panda_controller/teleop_paramConfig.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <franka_hw/trigger_rate.h>
#include <geometry_msgs/PoseStamped.h>

#include <control_msgs/GripperCommandAction.h>
#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <vector>
#include <string>

#include <fstream>
#include <iostream>
#include <filesystem>

namespace franka_example_controllers
{
  /**
   * Finite state machine that defines the states of the teleoperation phases.
   * ALIGN is the initial phase, when the leader and follower align. During this phase the leader
   * cannot be moved.
   * TRACK is the tracking phase.
   */
  enum TeleopStateMachine
  {
    ALIGN,
    TRACK,
  };

  /**
   * Controller class for ros_control that allows force-feedback teleoperation of a follower arm from
   * a leader arm. Smooth tracking is implemented by integrating a velocity signal, which is
   * calculated by limiting and saturating the velocity of the leader arm and a drift compensation.
   * The torque control of the follower arm is implemented by a simple PD-controller.
   * The leader arm is slightly damped to reduce vibrations.
   * Force-feedback is applied to the leader arm when the external forces on the follower arm exceed a
   * configured threshold.
   * While the leader arm is unguided (not in contact), the applied force-feedback will be reduced.
   */
  class Panda_TeleopJointController : public controller_interface::MultiInterfaceController<
                                          hardware_interface::EffortJointInterface, franka_hw::FrankaModelInterface,
                                          franka_hw::FrankaStateInterface>
  {

  private:
    std::ofstream outfile; // Declare the ofstream as a class member
    std::string fullPath;  // Optional: Keep track of file path as well

  public:
    /**
     * Initializes the controller class to be ready to run.
     *
     * @param[in] robot_hw Pointer to a RobotHW class to get interfaces and resource handles.
     * @param[in] node_handle Nodehandle that allows getting parameterizations from the server and
     * starting publisher
     */

    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;

    /**
     * Prepares the controller for the real-time execution. This method is executed once every time
     * the controller is started and runs in real-time
     */
    void starting(const ros::Time & /*time*/) override;

    /**
     * Computes the control-law and commands the resulting joint torques to the robots.
     *
     * @param[in] period The control period (here 0.001s)
     */
    void update(const ros::Time & /*time*/, const ros::Duration &period) override;

  private:
  ros::Time start_time_;  
  ros::Time teleoperation_time_; // Time when the teleoperation started


    using Vector3d = Eigen::Matrix<double, 3, 1>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    using Matrix7d = Eigen::Matrix<double, 7, 7>;
    using Matrix4d = Eigen::Matrix<double, 4, 4>;

    struct FrankaDataContainer
    {
      std::unique_ptr<franka_hw::FrankaStateHandle> state_handle;
      std::unique_ptr<franka_hw::FrankaModelHandle> model_handle;
      std::vector<hardware_interface::JointHandle> joint_handles;

      // A virtual wall to avoid joint limits.
      std::unique_ptr<JointWallContainer<7>> virtual_joint_wall;

      Vector7d tau_target;      // Target effort of each joint [Nm, Nm, Nm, Nm, Nm, Nm, Nm]
      Vector7d tau_target_last; // Last target effort of each joint [Nm, ...]
      Vector7d q;               // Measured position of each joint [rad, ...]
      Vector7d dq;              // Measured velocity of each joint [rad/s, ...]

      double f_ext_norm;                 // Norm of the external (cartesian) forces vector at the EE [N]
      double contact;                    // Contact scaling factor (values between 0 and 1)
      double contact_ramp_increase{0.3}; // Parameter for contact scaling factor
      double contact_force_threshold;    // Parameter for contact scaling factor [N]
    };

    FrankaDataContainer leader_data_;   // Container for data of the leader arm
    FrankaDataContainer follower_data_; // Container for data of the follower arm

    double filter_params_{0.005};
    double nullspace_stiffness_{20.0};
    double nullspace_stiffness_target_{20.0};

    Eigen::Quaterniond quaternion_delta;
    Eigen::Quaterniond quaternion_result;

    Vector7d leader_data_in_cartesian;
    Vector7d follower_data_in_cartesian;


    Vector7d model_data_in_cartesian_temp;
    Vector7d leader_data_in_cartesian_temp;

    Vector6d error_in_cartesian;

    Vector6d k_p_follower_cartesian;
    Vector6d k_d_follower_cartesian;

    Vector6d follower_EE_velocity;
    Vector7d tau_nullspace;
    Vector7d tau_in_cartesian;
    Vector7d q_d_nullspace;
    Vector7d q_initial;

    Vector3d position_d_in_cartesian;
    Eigen::Quaterniond orientation_d_in_cartesian;
    Vector3d position_d_target_in_cartesian;
    Eigen::Quaterniond orientation_d_target_in_cartesian;

    Eigen::Matrix4d T_offset;

    Eigen::Vector3d initial_position_;
    Eigen::Quaterniond initial_orientation_;

    Vector3d position_target_last_in_cartesian;
    Vector3d euler_angle_last_in_cartesian;

    Vector3d euler_angle_d_target_in_cartesian;
    Vector3d euler_angle_d_in_cartesian;

    Vector7d q_target_;      // Target positions of the follower arm [rad, rad, rad, rad, rad, rad, rad]
    Vector7d q_target_last_; // Last target positions of the follower arm [rad, ...]
    Vector6d q_target_in_cartesian;
    Vector6d q_target_last_in_cartesian;

    Vector7d dq_unsaturated_; // Unsaturated target velocities of the follower arm [rad/s, ...]
    Vector7d dq_target_;      // Target velocities of the follower arm [rad/s, ...]
    Vector6d dq_target_last_in_cartesian;
    Vector7d dq_target_last_; // Last target velocities of the follower arm [rad/s, ...]

    Vector7d init_leader_q_; // Measured q of leader arm during alignment [rad]  // ----------------------------------------
    Vector7d leader_data_q;
    Vector7d leader_data_dq;
    Vector7d follower_tau_ext_hat; // The feedback is in cartesian space, only the v/w of end effector worth reading and sending back

    Vector6d follower_f_ext_hat;

    Vector6d external_force_torque;
    Vector7d dq_max_lower_;             // Lower max velocities of the follower arm [rad/s, ...]
    Vector7d dq_max_upper_;             // Upper max velocities of the follower arm [rad/s, ...]
    Vector7d ddq_max_lower_;            // Lower max accelerations of the follower arm [rad/s², ...]
    Vector7d ddq_max_upper_;            // Upper max accelerations of the follower arm [rad/s², ...]
    double velocity_ramp_shift_{0.25};  // parameter for ramping dq_max and ddq_max [rad]
    double velocity_ramp_increase_{20}; // parameter for ramping dq_max and ddq_max

    // Max velocities of the follower arm during alignment [rad/s, ...]
    Vector7d dq_max_align_{(Vector7d() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished()};
    // Max accelerations of the follower arm during alignment [rad/s², ...]
    Vector7d ddq_max_align_{(Vector7d() << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5).finished()};

    Vector7d alignment_error_;      // Diff between follower and leader q during alignment [rad/s, ...]
    Vector7d prev_alignment_error_; // alignment_error_ in previous control loop [rad/s, ...]

    Vector7d k_p_follower_; // p-gain for follower arm
    Vector7d k_d_follower_; // d-gain for follower arm

    // p-gain for follower arm during alignment
    Vector7d k_p_follower_align_{(Vector7d() << 45.0, 45.0, 45.0, 45.0, 18.0, 11.0, 5.0).finished()};
    // d-gain for follower arm during alignment
    Vector7d k_d_follower_align_{(Vector7d() << 4.5, 4.5, 4.5, 4.5, 1.5, 1.5, 1.0).finished()};

    Vector7d dq_max_leader_lower_; // Soft max velocities of the leader arm [rad/s, ...]
    Vector7d dq_max_leader_upper_; // Hard max velocities of the leader arm [rad/s, ...]
    Vector7d k_d_leader_lower_;    // d-gain for leader arm when under soft-limit
    Vector7d k_d_leader_upper_;    // d-gain for leader arm when hard limit is reached

    Vector7d k_dq_; // gain for drift compensation in follower arm

    double force_feedback_idle_{0.5};     // Applied force-feedback, when leader arm is not guided
    double force_feedback_guiding_{0.95}; // Applied force-feeback, when leader arm is guided

    double decrease_factor_{0.95}; // Param, used when (in error state) controlling torques to zero

    TeleopStateMachine current_state_{TeleopStateMachine::ALIGN}; // Current state in teleoperation

    const double kAlignmentTolerance_{1e-2}; // Tolerance to consider a joint aligned [rad]

    void initArm(hardware_interface::RobotHW *robot_hw,
                 ros::NodeHandle &node_handle,
                 FrankaDataContainer &arm_data,
                 const std::string &arm_id,
                 const std::vector<std::string> &joint_names);

    void updateArm(FrankaDataContainer &arm_data);

    Vector7d saturateAndLimit(const Vector7d &x_calc,
                              const Vector7d &x_last,
                              const Vector7d &x_max,
                              const Vector7d &dx_max,
                              double delta_t);

    double rampParameter(double x,
                         double neg_x_asymptote,
                         double pos_x_asymptote,
                         double shift_along_x,
                         double increase_factor);

    template <typename T>
    std::vector<T> getJointParams(const std::string &param_name, ros::NodeHandle &nh)
    {
      std::vector<T> vec;
      if (!nh.getParam(param_name, vec) || vec.size() != 7)
      {
        throw std::invalid_argument("Panda_TeleopJointController: Invalid or no parameter " +
                                    nh.getNamespace() + "/" + param_name +
                                    " provided, aborting controller init!");
      }
      return vec;
    }

    Vector7d get7dParam(const std::string &param_name, ros::NodeHandle &nh);

    template <typename T>
    T get1dParam(const std::string &param_name, ros::NodeHandle &nh)
    {
      T out;
      if (!nh.getParam(param_name, out))
      {
        throw std::invalid_argument("Panda_TeleopJointController: Invalid or no parameter " +
                                    nh.getNamespace() + "/" + param_name +
                                    " provided, "
                                    "aborting controller init!");
      }
      return out;
    }

    static void getJointLimits(ros::NodeHandle &nh,
                               const std::vector<std::string> &joint_names,
                               std::array<double, 7> &upper_joint_soft_limit,
                               std::array<double, 7> &lower_joint_soft_limit);

    Vector7d leaderDamping(const Vector7d &dq);

    // Debug tool
    bool debug_{false};
    std::mutex dynamic_reconfigure_mutex_;
    double leader_damping_scaling_{1.0};
    double follower_stiffness_scaling_{1.0};

    ros::NodeHandle dynamic_reconfigure_teleop_param_node_;
    std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::teleop_paramConfig>>
        dynamic_server_teleop_param_;
    void teleopParamCallback(franka_example_controllers::teleop_paramConfig &config, uint32_t level);

    franka_hw::TriggerRate publish_rate_{60.0};
    realtime_tools::RealtimePublisher<std_msgs::Float64> follower_contact_pub_;
    realtime_tools::RealtimePublisher<sensor_msgs::JointState> follower_data_pub_;
    realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray> marker_pub_;

    ros::Subscriber sub_leader_data;
    ros::Subscriber sub_model_data;

    // declare parameters that follower uses
    ros::Time counter;
    ros::Time timestamp;
    ros::Time last_timestamp{};
    bool topic_received = false;


    bool is_teleoperation_{false}; // Flag to check if teleoperation mode is active
    bool is_model_prediction_{false}; // Flag to check if model prediction mode is active


    void leaderDataCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void ModelDataCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    ros::Subscriber sub_equilibrium_pose_;
    void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> arm_pose_pub_;

    void publishFollowerContact();
    void publishFollowerData();
    void publishMarkers();
  };
}