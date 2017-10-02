#include "RAI_machine.h"

#include <utility>
#include <vector>

#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"
#include "lcmtypes/robotlocomotion/robot_plan_t.hpp"

#include <Algo/spline.h>

using bot_core::robot_state_t;

namespace drake {

/* index of iiwastate */
const int kStateIndex = 0;

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

using manipulation::planner::ConstraintRelaxingIk;
using examples::kuka_iiwa_arm::pick_and_place::PickAndPlaceStateMachine;

arr rndSpline(uint T, uint n){
  rnd.seed(0);
  arr P(10,n);

  //a random spline
  //a set of random via points with zero start and end:
  rndUniform(P,-1.,1.,false); P[0]=0.; P[P.d0-1]=0.;

  P *= 2.;

  //convert into a smooth spline (1/0.03 points per via point):
  return mlr::Spline(T,P).eval();
}

struct RAI_Machine::InternalState {
  InternalState(const std::string& iiwa_model_path,
                const std::string& end_effector_name,
                const std::vector<Isometry3<double>>& place_locations)
      : world_state(iiwa_model_path, end_effector_name),
        state_machine(place_locations, false),
        last_iiwa_plan(MakeDefaultIiwaPlan()),
        last_wsg_command(MakeDefaultWsgCommand()) {

      X = rndSpline(30, 7);

  }

  ~InternalState() {}

  arr X;
  examples::kuka_iiwa_arm::pick_and_place::WorldState world_state;
  examples::kuka_iiwa_arm::pick_and_place::PickAndPlaceStateMachine state_machine;
  robotlocomotion::robot_plan_t last_iiwa_plan;
  lcmt_schunk_wsg_command last_wsg_command;
};

RAI_Machine::RAI_Machine(
    const std::string& iiwa_model_path,
    const std::string& end_effector_name,
    const Isometry3<double>& iiwa_base,
    const std::vector<Isometry3<double>>& place_locations,
    const double period_sec)
    : iiwa_model_path_(iiwa_model_path),
      end_effector_name_(end_effector_name),
      iiwa_base_(iiwa_base),
      planner_(std::make_unique<ConstraintRelaxingIk>(
          iiwa_model_path_, end_effector_name_, iiwa_base_)),
      place_locations_(place_locations) {
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

std::unique_ptr<systems::AbstractValues>
RAI_Machine::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  abstract_vals.push_back(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<InternalState>(
          InternalState(iiwa_model_path_, end_effector_name_,
                        place_locations_))));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

void RAI_Machine::SetDefaultState(
    const systems::Context<double>&,
    systems::State<double>* state) const {
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);
  internal_state = InternalState(iiwa_model_path_, end_effector_name_,
                                 place_locations_);
}

void RAI_Machine::CalcIiwaPlan(
    const systems::Context<double>& context,
    robotlocomotion::robot_plan_t* iiwa_plan) const {
  /* Call actions based on state machine logic */
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
#if 1
  if(internal_state.last_iiwa_plan.plan.size()>0){
      bot_core::robot_state_t state0 = internal_state.last_iiwa_plan.plan[0];
      robotlocomotion::robot_plan_t* plan = (robotlocomotion::robot_plan_t*)&internal_state.last_iiwa_plan;
      arr X = internal_state.X;
      plan->num_states=X.d0;
      plan->plan.clear();
      plan->plan_info.clear();
      for(uint i=0;i<X.d0;i++){
          bot_core::robot_state_t state = state0;
          state.utime = i*100000;
          floatA x;
          copy(x,X[i]);
          state.joint_position = conv_arr2stdvec(x);
          plan->plan.push_back(state);
          plan->plan_info.push_back(1);
      }
  }
#endif
  *iiwa_plan = internal_state.last_iiwa_plan;
}


void RAI_Machine::CalcWsgCommand(
    const systems::Context<double>& context,
    lcmt_schunk_wsg_command* wsg_command) const {
  /* Call actions based on state machine logic */
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  lcmt_schunk_wsg_command wsgcommand = {0, 100., 40.};
  *wsg_command = wsgcommand; //internal_state.last_wsg_command;
}

void RAI_Machine::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  // Extract Internal state.
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);

  /* Update world state from inputs. */
  const robot_state_t& iiwa_state =
      this->EvalAbstractInput(context, input_port_iiwa_state_)
          ->GetValue<robot_state_t>();
  const robot_state_t& box_state =
      this->EvalAbstractInput(context, input_port_box_state_)
          ->GetValue<robot_state_t>();
  const lcmt_schunk_wsg_status& wsg_status =
      this->EvalAbstractInput(context, input_port_wsg_status_)
          ->GetValue<lcmt_schunk_wsg_status>();

  internal_state.world_state.HandleIiwaStatus(iiwa_state);
  internal_state.world_state.HandleWsgStatus(wsg_status);
  internal_state.world_state.HandleObjectStatus(box_state);

  PickAndPlaceStateMachine::IiwaPublishCallback iiwa_callback =
      ([&](const robotlocomotion::robot_plan_t* plan) {
        internal_state.last_iiwa_plan = *plan;
      });

  PickAndPlaceStateMachine::WsgPublishCallback wsg_callback =
      ([&](const lcmt_schunk_wsg_command* msg) {
        internal_state.last_wsg_command = *msg;
      });
  internal_state.state_machine.Update(
      internal_state.world_state, iiwa_callback, wsg_callback, planner_.get());
}

examples::kuka_iiwa_arm::pick_and_place::PickAndPlaceState RAI_Machine::state(
    const systems::Context<double>& context) const {
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  return internal_state.state_machine.state();
}

const examples::kuka_iiwa_arm::pick_and_place::WorldState& RAI_Machine::world_state(
    const systems::Context<double>& context) const {
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  return internal_state.world_state;
}

}  // namespace drake
