#include "control_demo/MpcBase.h"

namespace mpc
{

MpcBase::MpcBase(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : 
    reference_trajectory_(), 
    reference_window_(),
    state_estimate_(),
    mpc_params_(),
    mpc_controller_(nh, pnh)
{
}

acado_mpc_common::ControlCommand MpcBase::run()
{
  if (reference_window_.points.size() < WINDOW_NUM) {
    return acado_mpc_common::ControlCommand();
  }
  return mpc_controller_.run(state_estimate_, reference_window_, mpc_params_);
}

} // namespace mpc