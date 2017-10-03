#include "RAI_machine.h"

#include <utility>
#include <vector>

#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/robotlocomotion/robot_plan_t.hpp"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"

namespace drake {

robotlocomotion::robot_plan_t MakeDefaultIiwaPlan() {
  robotlocomotion::robot_plan_t default_plan{};
  default_plan.utime = 0;
  default_plan.num_states = 0;
  return default_plan;
}

lcmt_schunk_wsg_command MakeDefaultWsgCommand() {
  lcmt_schunk_wsg_command default_command{};
  default_command.utime = 0;
  default_command.target_position_mm = 110;  // maximum aperture
  default_command.force = 0;
  return default_command;
}

struct RAI_Machine::InternalState {
  //inputs:
  bot_core::robot_state_t iiwa_state;
  lcmt_schunk_wsg_status wsg_state;
  bot_core::robot_state_t obj_state;

  InternalState(){}
  ~InternalState(){}
};

RAI_Machine::RAI_Machine(const double period_sec) {

    grip = 100.;

  input_port_iiwa_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_box_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_wsg_status_ = this->DeclareAbstractInputPort().get_index();

  output_port_iiwa_plan_ =
      this->DeclareAbstractOutputPort(
              MakeDefaultIiwaPlan(),
              &RAI_Machine::CalcIiwaPlan)
          .get_index();

  output_port_wsg_command_ =
      this->DeclareAbstractOutputPort(
              MakeDefaultWsgCommand(),
              &RAI_Machine::CalcWsgCommand)
          .get_index();

  this->DeclarePeriodicUnrestrictedUpdate(period_sec, 0);
}

std::unique_ptr<systems::AbstractValues> RAI_Machine::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  abstract_vals.push_back(std::unique_ptr<systems::AbstractValue>(
                              new systems::Value<InternalState>( InternalState() )
                              ));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

void RAI_Machine::SetDefaultState( const systems::Context<double>&, systems::State<double>* state) const {
  InternalState& internal_state = state->get_mutable_abstract_state<InternalState>(0);
  internal_state = InternalState();
}

void RAI_Machine::CalcIiwaPlan(const systems::Context<double>& context, robotlocomotion::robot_plan_t* iiwa_plan) const {
  /* Call actions based on state machine logic */
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
#if 1
  robotlocomotion::robot_plan_t plan;
  plan.utime = 0;
  plan.num_states = 0;
  plan.num_bytes = 0;
  plan.num_grasp_transitions = 0;
  bot_core::robot_state_t state = internal_state.iiwa_state;
  std::fill(state.joint_velocity.begin(), state.joint_velocity.end(), 0.f);
  std::fill(state.joint_effort.begin(), state.joint_effort.end(), 0.f);
  if(state.num_joints>0 && path.d0>0){
      plan.num_states = path.d0;
      for(uint i=0;i<path.d0;i++){
          state.utime = i*100000;
          floatA x;
          copy(x,path[i]);
          state.joint_position = conv_arr2stdvec(x);
          plan.plan.push_back(state);
          plan.plan_info.push_back(1);
      }
  }
#endif
  *iiwa_plan = plan; //internal_state.last_iiwa_plan;
}


void RAI_Machine::CalcWsgCommand(const systems::Context<double>& context, lcmt_schunk_wsg_command* wsg_command) const {
  lcmt_schunk_wsg_command wsgcommand = {0, grip, 40.};
  *wsg_command = wsgcommand;
}

void RAI_Machine::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {

  // Extract Internal state.
  InternalState& internal_state = state->get_mutable_abstract_state<InternalState>(0);

  /* Update world state from inputs. */
  internal_state.iiwa_state =
      this->EvalAbstractInput(context, input_port_iiwa_state_)-> GetValue<bot_core::robot_state_t>();
  internal_state.obj_state =
      this->EvalAbstractInput(context, input_port_box_state_ )-> GetValue<bot_core::robot_state_t>();
  internal_state.wsg_state =
      this->EvalAbstractInput(context, input_port_wsg_status_)-> GetValue<lcmt_schunk_wsg_status>();
}

}  // namespace drake
