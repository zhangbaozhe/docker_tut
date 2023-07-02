#include "control_demo/MpcBase.h"

#include <cstddef>
#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Eigen>
#include <gflags/gflags.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/AttitudeTarget.h>

DEFINE_double(r, 1.0, "Desired circle radius [m]");
DEFINE_double(w, 0.5, "Desired circle rotation angular speed [rad/s]");

using Estimate_t = acado_mpc_common::QuadRelativeEstimate;
using TrajectoryPoint_t = acado_mpc_common::RelativeTrajectoryPoint;
using Trajectory_t = acado_mpc_common::RelativeTrajectory;
using ControlCommand_t = acado_mpc_common::ControlCommand;

const size_t SAMPLE_NUM = 10000;

/* ===================== Function Prototypes ========================== */
Trajectory_t generateCircleTrajectory(size_t sample_num, double R, double omega, 
    double duration, double start_z);

nav_msgs::Odometry genOdomFromGZ(const gazebo_msgs::ModelStates::ConstPtr &gz_msg_ptr, 
    const std::string &model_name);

std::ofstream& info2csv(std::ofstream &csv_stream, const Estimate_t &est, const Trajectory_t &traj, 
    const ControlCommand_t &cmd);

Trajectory_t extractReference(size_t window_num, double time, double duration, size_t sample_num, 
    const Trajectory_t &trajectory);

Trajectory_t interpolatePoints(size_t window_num, const TrajectoryPoint_t &start, 
    const TrajectoryPoint_t &end);
/* ===================== Function Prototypes ========================== */

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 4) {
  std::ostringstream out;
  out.precision(n);
  out << std::fixed << a_value;
  return out.str();
}

int main(int argc, char **argv)
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "mpc_node");
  std::ofstream CSV_STREAM;

  auto time = std::chrono::system_clock::now();
  auto sys_time = std::chrono::system_clock::to_time_t(time);
  std::string r_str, w_str;
  r_str = to_string_with_precision(FLAGS_r);
  w_str = to_string_with_precision(FLAGS_w);
  char buffer[25];
  buffer[24] = 0;
  std::strncpy(buffer, std::ctime(&sys_time), 24);
  CSV_STREAM.open(ros::package::getPath("control_demo") + "/csv/" + ros::this_node::getName() + " " +
      std::string(buffer) + " r_w " + r_str + "_" + w_str + ".csv");
  CSV_STREAM << "time, px, py, pz, vx, vy, vz, qw, qx, qy, qz, px_r, py_r, pz_r, vx_r, vy_r, vz_r, qw_r, qx_r, qy_r, qz_r, T, wx, wy, wz\n";

  ros::NodeHandle nh, pnh("~");
  ros::AsyncSpinner spinner(1);
  auto attitude_target_pub = nh.advertise<mavros_msgs::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", 1);

  // shared variable
  Estimate_t CURRENT_EST;
  CURRENT_EST.a_imu = Eigen::Vector3d(0, 0, 9.8);

  const std::string MODEL_NAME = "iris";
  const std::string GAZEBO_MODELS_TOPIC = "/gazebo/model_states";
  
  auto gazebo_model_states_sub = nh.subscribe<gazebo_msgs::ModelStates>(GAZEBO_MODELS_TOPIC, 1, 
      [&CURRENT_EST, MODEL_NAME](const gazebo_msgs::ModelStates::ConstPtr &gz_msg_ptr)
      {
        nav_msgs::Odometry QUAD_ODOM = 
            genOdomFromGZ(gz_msg_ptr, MODEL_NAME);
        CURRENT_EST.position.x() = QUAD_ODOM.pose.pose.position.x;
        CURRENT_EST.position.y() = QUAD_ODOM.pose.pose.position.y;
        CURRENT_EST.position.z() = QUAD_ODOM.pose.pose.position.z;
        CURRENT_EST.velocity.x() = QUAD_ODOM.twist.twist.linear.x;
        CURRENT_EST.velocity.y() = QUAD_ODOM.twist.twist.linear.y;
        CURRENT_EST.velocity.z() = QUAD_ODOM.twist.twist.linear.z;
        CURRENT_EST.orientation.w() = QUAD_ODOM.pose.pose.orientation.w;
        CURRENT_EST.orientation.x() = QUAD_ODOM.pose.pose.orientation.x;
        CURRENT_EST.orientation.y() = QUAD_ODOM.pose.pose.orientation.y;
        CURRENT_EST.orientation.z() = QUAD_ODOM.pose.pose.orientation.z;
      });
  while (!gazebo_model_states_sub.getNumPublishers()) {
    ROS_WARN("Waiting for connection");
  }

  spinner.start();

  ros::Rate rate(100);

  mpc::MpcBase controller(nh, pnh);

  const Trajectory_t CIRCLE_TRAJ = generateCircleTrajectory(SAMPLE_NUM, FLAGS_r, FLAGS_w, 2 * M_PI / FLAGS_w, 
      1.0);
  
  const TrajectoryPoint_t START_POINT = CIRCLE_TRAJ.points.front();

  using Time = std::chrono::high_resolution_clock;
  using dsec = std::chrono::duration<double>;


  bool is_begin = false; 
  while (ros::ok()) {
    rate.sleep();
    controller.setStateEstimate(CURRENT_EST);

    if (!is_begin && (CURRENT_EST.position - START_POINT.position).norm() < 0.10) {
      // begin
      is_begin = true;
    } 

    if (!is_begin) {
      TrajectoryPoint_t start{CURRENT_EST.position, CURRENT_EST.velocity, CURRENT_EST.orientation};
      Trajectory_t traj = interpolatePoints(acado_mpc::kSamples+1, start, START_POINT);
      controller.setReferenceWindow(traj);
    } else {
      static auto start_time = Time::now();
      dsec time = (Time::now() - start_time);
      controller.setReferenceWindow(extractReference(acado_mpc::kSamples+1, time.count(), 
          2 * M_PI / FLAGS_w, SAMPLE_NUM, CIRCLE_TRAJ));
      if (time.count() > 2 * M_PI / FLAGS_w + 0.1) 
        break;
    }

    auto cmd = controller.run();
    attitude_target_pub.publish(cmd.toRosAttitudeTarget(0.7));
    if (is_begin)
      info2csv(CSV_STREAM, CURRENT_EST, CIRCLE_TRAJ, cmd);
  }
  CSV_STREAM.close();
  return 0;
}

/* ===================== Function Implementations ========================== */

Trajectory_t generateCircleTrajectory(size_t sample_num, double R, double omega, 
    double duration, double start_z)
{
  Trajectory_t result;
  Eigen::Matrix3d temp_rotation_WB;
  Eigen::Quaterniond temp_q_WB;
  const double G = 9.81;
  double t = sqrt(pow(G, 2) + pow(omega * omega * R, 2));
  for (size_t i = 0; i < sample_num; i++) {
    temp_rotation_WB << G / t * cos(omega * (i * duration / sample_num)),
        -sin(omega * (i * duration / sample_num)),
        -omega * omega * R / t * cos(omega * (i * duration / sample_num)),
        G / t * sin(omega * (i * duration / sample_num)),
        cos(omega * (i * duration / sample_num)),
        -omega * omega * R /t * sin(omega * (i * duration / sample_num)),
        omega * omega * R / t,
        0,
        G / t;
    temp_q_WB = Eigen::Quaterniond(temp_rotation_WB);
    temp_q_WB.normalize();
    TrajectoryPoint_t point;
    point.position(0) = R * cos(omega * (i * duration / sample_num));
    point.position(1) = R * sin(omega * (i * duration / sample_num));
    point.position(2) = start_z;
    point.orientation = temp_q_WB;
    point.velocity(0) = -omega * R * sin(omega * (i * duration / sample_num));
    point.velocity(1) = omega * R * cos(omega * (i * duration / sample_num));
    point.velocity(2) = 0;
    result.points.push_back(point);
  }
  return result;
}

nav_msgs::Odometry genOdomFromGZ(const gazebo_msgs::ModelStates::ConstPtr &gz_msg_ptr, 
    const std::string &model_name)
{
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  auto it = std::find(gz_msg_ptr->name.begin(), gz_msg_ptr->name.end(), model_name);
  if (it == gz_msg_ptr->name.end()) {
    ROS_ERROR("[generateOdomFromGZ] Cannot find the target: %s", model_name.c_str());
    ros::shutdown();
    exit(1);
  }
  auto index = std::distance(gz_msg_ptr->name.begin(), it);
  odom.pose.pose = gz_msg_ptr->pose[index];
  odom.twist.twist = gz_msg_ptr->twist[index];
  return odom;
}

std::ofstream& info2csv(std::ofstream &csv_stream, const Estimate_t &est, const Trajectory_t &traj, 
    const ControlCommand_t &cmd)
{
  static auto START_TIME = ros::Time::now().toSec();
  // time, est, traj, control
  csv_stream << ros::Time::now().toSec() - START_TIME << ", "
            << est.position.x() << ", "
            << est.position.y() << ", "
            << est.position.z() << ", "
            << est.velocity.x() << ", "
            << est.velocity.y() << ", "
            << est.velocity.z() << ", "
            << est.orientation.w() << ", "
            << est.orientation.x() << ", "
            << est.orientation.y() << ", "
            << est.orientation.z() << ", "
            << traj.points.front().position.x() << ", "
            << traj.points.front().position.y() << ", "
            << traj.points.front().position.z() << ", "
            << traj.points.front().velocity.x() << ", "
            << traj.points.front().velocity.y() << ", "
            << traj.points.front().velocity.z() << ", "
            << traj.points.front().orientation.w() << ", "
            << traj.points.front().orientation.x() << ", "
            << traj.points.front().orientation.y() << ", "
            << traj.points.front().orientation.z() << ", "
            << cmd.collective_thrust << ", "
            << cmd.bodyrates.x() << ", "
            << cmd.bodyrates.y() << ", "
            << cmd.bodyrates.z() << ", "
            << "\n";
  return csv_stream;
}

Trajectory_t extractReference(size_t window_num, double time, double duration, size_t sample_num, 
    const Trajectory_t &trajectory)
{
  // ceil(time % T / T) * (N - 1)
  size_t target_index = (size_t)(std::fmod(time, duration) / duration * (sample_num - 1));
  ROS_WARN("time: %.3f, duration: %.3f", time, duration);
  ROS_WARN("Target index: %d", target_index);
  Trajectory_t result;
  if (target_index > sample_num - window_num) {
    size_t tail_num = sample_num - target_index;
    size_t head_num = window_num - tail_num;
    for (auto i = 0; i < tail_num; i++)
      result.points.push_back(trajectory.points.at(i + target_index));
    for (auto i = 0; i < head_num; i++)
      result.points.push_back(trajectory.points.at(i));
  } else {
    for (auto i = 0; i < window_num; i++) 
      result.points.push_back(trajectory.points.at(i + target_index));
  }
  return result;
}

Trajectory_t interpolatePoints(size_t window_num, const TrajectoryPoint_t &start, 
    const TrajectoryPoint_t &end)
{
  Trajectory_t result;
  result.points.push_back(start);
  for (size_t i = 1; i < window_num - 1; i++) {
    double ratio = static_cast<double>(i) / (window_num - 1);
    TrajectoryPoint_t p;
    p.position = ratio * start.position + (1 - ratio) * end.position;
    p.velocity = ratio * start.velocity + (1 - ratio) * end.velocity;
    p.orientation = start.orientation.slerp(ratio, end.orientation);
    result.points.push_back(p);
  }
  result.points.push_back(end);
  return result;
}

/* ===================== Function Implementations ========================== */