#ifndef FSMSTATE_RL_H
#define FSMSTATE_RL_H

#include <thread>
#include "FSMState.h"
#include "tensorrt_cuda/tensor_cuda_test.hpp"
/**
 *
 */

struct ModelParams
{
  float damping;
  float stiffness;
  float action_scale;
  float hip_scale_reduction;
  float num_of_dofs;
  float lin_vel_scale;
  float ang_vel_scale;
  float dof_pos_scale;
  float dof_vel_scale;
  float clip_obs;
  float clip_actions;
  float torque_limits[8];
  float d_gains[8];
  float p_gains[8];
  float commands_scale[3];
  float default_dof_pos[8];
};

struct Observations
{
  float lin_vel[3];
  float ang_vel[3];
  float gravity_vec[3];
  float forward_vec[3];
  float commands[3];
  float base_quat[4];
  float dof_pos[8];
  float dof_vel[8];
  float actions[8];
};

class FSMState_RL : public FSMState
{
public:
  FSMState_RL(std::shared_ptr<ControlFSMData> data);
  virtual ~FSMState_RL() {}

  // Behavior to be carried out when entering a state
  void enter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSMStateName checkTransition();

  // Manages state specific transitions
  //   TransitionData transition();

  // Behavior to be carried out when exiting a state
  void exit();

  //   TransitionData testTransition();

private:
  // Keep track of the control iterations
  float wheel_init_pos_abs_[4];
  float x_vel_cmd_;
  float yaw_vel_cmd_;

private:
  ModelParams params_;
  Observations obs_;

  void _Forward();
  void _Run_Forward();

  std::shared_ptr<CudaTest> cuda_test_;

  std::thread forward_thread;
  bool threadRunning;
  float desired_pos[8] = {0, 0.75, -1.5, 0, 0, 0.75, -1.5, 0};
  // AttitudeData* attitude_ = nullptr;
  // LegData* legdata_ = nullptr;
  // LinearKFPositionVelocityEstimator<float>* posvelest_ = nullptr;

  std::shared_ptr<float[]> input_0;
  std::shared_ptr<float[]> input_1;
  std::shared_ptr<float[]> output;

  std::shared_ptr<float[]> output_last;
  std::shared_ptr<float[]> input_1_temp;

  int history_length = 10;

  void _GetObs();
  Vec3<double> a_l;

  float action[8];

  bool stop_update_ = false;
  bool thread_first_ = true;
};

#endif // FSMSTATE_RL_H
