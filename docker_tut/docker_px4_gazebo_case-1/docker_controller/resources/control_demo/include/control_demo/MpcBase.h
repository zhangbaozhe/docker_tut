#ifndef MPC_BASE_H
#define MPC_BASE_H

#include "acado_mpc/mpc_common.h"
#include "acado_mpc/mpc_params.h"
#include "acado_mpc/mpc_controller.h"

#include <ros/ros.h>
#include <Eigen/Eigen>

namespace mpc
{

class MpcBase
{
 public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcBase(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  virtual ~MpcBase() = default;
  MpcBase(const MpcBase &) = delete;
  MpcBase &operator=(const MpcBase &) = delete;
  MpcBase(MpcBase &&) = delete;
  MpcBase &operator=(MpcBase &&) = delete;


  acado_mpc_common::RelativeTrajectory 
  getReferenceTrajectory() const { return reference_trajectory_; }

  acado_mpc_common::RelativeTrajectory 
  getReferenceWindow() const { return reference_window_; }

  acado_mpc_common::QuadRelativeEstimate 
  getStateEstimate() const { return state_estimate_; }


  void setReferenceTrajectory(const acado_mpc_common::RelativeTrajectory &reference_trajectory) 
  { reference_trajectory_ = reference_trajectory; }
  void setReferenceWindow(const acado_mpc_common::RelativeTrajectory &reference_window) 
  { reference_window_ = reference_window; }
  void setStateEstimate(const acado_mpc_common::QuadRelativeEstimate &state_estimate) 
  { state_estimate_ = state_estimate; }


  virtual acado_mpc_common::ControlCommand run();

  const size_t WINDOW_NUM = acado_mpc::kSamples + 1;
 private: 
  acado_mpc_common::RelativeTrajectory reference_trajectory_;
  acado_mpc_common::RelativeTrajectory reference_window_;
  acado_mpc_common::QuadRelativeEstimate state_estimate_;
  

  acado_mpc::MpcParams<double> mpc_params_;
  acado_mpc::MpcController<double> mpc_controller_;
  
};

} // namespace mpc


#endif