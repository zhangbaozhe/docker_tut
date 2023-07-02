// Created by Baozhe Zhang on Oct 13, 2022

#pragma once

#include <vector>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/AttitudeTarget.h>

namespace acado_mpc_common
{

static constexpr const char *RELATIVE_FRAME_ID = "base_link";
static constexpr const char *WORLD_FRAME_ID    = "map";


/**
 * @brief Relative estimate of the quadrotor
 * 
 */
struct QuadRelativeEstimate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QuadRelativeEstimate();
  QuadRelativeEstimate(const nav_msgs::Odometry &relative_estimate_msg,
      const Eigen::Vector3d &a_imu,
      const Eigen::Vector3d &omega_non,
      const Eigen::Vector3d &beta_non);
  QuadRelativeEstimate(const Eigen::Vector3d &p,
      const Eigen::Vector3d &v,
      const Eigen::Quaterniond &q,
      const Eigen::Vector3d &a_imu,
      const Eigen::Vector3d &omega_non,
      const Eigen::Vector3d &beta_non);
  virtual ~QuadRelativeEstimate();

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d a_imu;
  Eigen::Vector3d omega_non;
  Eigen::Vector3d beta_non;
};

/**
 * @brief Relative trajectory point
 * 
 */
struct RelativeTrajectoryPoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RelativeTrajectoryPoint();
  RelativeTrajectoryPoint(const Eigen::Vector3d &p,
      const Eigen::Vector3d &v,
      const Eigen::Quaterniond &q); 
  virtual ~RelativeTrajectoryPoint();

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  // Eigen::Vector3d a_imu_ref;
  // Eigen::Vector3d omega_non_ref;
  // Eigen::Vector3d beta_non_ref;
};


/**
 * @brief Relative trajectory
 * 
 */
struct RelativeTrajectory
{
  RelativeTrajectory();
  RelativeTrajectory(const acado_mpc_common::RelativeTrajectoryPoint &point);
  virtual ~RelativeTrajectory();

  nav_msgs::Path toRosPath() const;

  std::vector<acado_mpc_common::RelativeTrajectoryPoint> points;
};

/**
 * @brief Control command
 * 
 */
struct ControlCommand
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ControlCommand();
  ControlCommand(const Eigen::Vector4d &u);
  ControlCommand(double collective_thrust, const Eigen::Vector3d &bodyrates);

  /**
   * @brief Convert to mavros_msgs::AttitudeTarget
   * 
   * @param thrust_param thrust that makes the quadrotor hover in (0, 1)
   * @return mavros_msgs::AttitudeTarget 
   */
  mavros_msgs::AttitudeTarget toRosAttitudeTarget(double thrust_param) const;

  virtual ~ControlCommand();

  double collective_thrust;
  Eigen::Vector3d bodyrates;
};


/**
 * @brief Get the Param object
 * 
 * @tparam T parameter type
 * @param name parameter name
 * @param parameter parameter value
 * @param pnh private node handler
 * @return true 
 * @return false 
 */
template<typename T>
bool getParam(const std::string& name, T& parameter,
              const ros::NodeHandle& pnh = ros::NodeHandle("~"))
{
  if (pnh.getParam(name, parameter))
  {
    ROS_INFO_STREAM(
        "[" << pnh.getNamespace() << "] " << name << " = " << parameter);
    return true;
  }
  ROS_ERROR_STREAM(
      "[" << pnh.getNamespace() << "] Could not load parameter "
      << pnh.getNamespace() << "/"<< name);
  return false;
}

/**
 * @brief Get the Param object (overloaded)
 * 
 * @tparam T 
 * @param name 
 * @param parameter 
 * @param defaultValue 
 * @param pnh 
 * @return true 
 * @return false 
 */
template<typename T>
bool getParam(const std::string& name, T& parameter, const T& defaultValue,
              const ros::NodeHandle& pnh = ros::NodeHandle("~"))
{
  if (pnh.getParam(name, parameter))
    ROS_INFO_STREAM(
        "[" << pnh.getNamespace() << "] " << name << " = " << parameter);
  else
  {
    parameter = defaultValue;
    ROS_WARN_STREAM(
        "[" << pnh.getNamespace() << "] " << name << " = " << parameter
        << " set to default value");
  }
  return true;
}



}// namespace acado_mpc_common