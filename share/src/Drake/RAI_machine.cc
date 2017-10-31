#include "RAI_machine.h"
#include "../../projects/17-RAI/filter.h"
#include <Msg/MotionReference.h>

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

  Access<Msg_MotionReference> motionReference;

  Access<arr> currentQ;
  Access<mlr::Transformation> robotBase;
  Access<PerceptSimpleL> percepts_input;
  Access<double> timeToGo;

  Msg_MotionReference ref;
  double refGrip;

  InternalState()
    : motionReference("MotionReference"),
      currentQ("currentQ"),
      robotBase("robotBase"),
      percepts_input("percepts_input"),
      timeToGo("timeToGo"),
      refGrip(100.){}
  ~InternalState(){}
};

RAI_Machine::RAI_Machine(const double period_sec)
   {

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

  robotlocomotion::robot_plan_t plan;
  plan.utime = 0;
  plan.num_states = 0;
  plan.num_bytes = 0;
  plan.num_grasp_transitions = 0;
  bot_core::robot_state_t state = internal_state.iiwa_state;
  std::fill(state.joint_velocity.begin(), state.joint_velocity.end(), 0.f);
  std::fill(state.joint_effort.begin(), state.joint_effort.end(), 0.f);

  arr path = internal_state.ref.path;

  if(state.num_joints>0 && path.d0>0){
    if(path.d1 >(uint)state.num_joints) path.delColumns(-1);
    CHECK_EQ(path.d1, (uint)state.num_joints, "wrong dim");
    double tau = internal_state.ref.tau.scalar()*1e6;
    plan.num_states = path.d0;
    for(uint i=0;i<path.d0;i++){
      state.utime = i*tau;
      floatA x;
      copy(x,path[i]);
      state.joint_position = conv_arr2stdvec(x);
      plan.plan.push_back(state);
      plan.plan_info.push_back(1);
    }
  }

  *iiwa_plan = plan; //internal_state.last_iiwa_plan;
}


void RAI_Machine::CalcWsgCommand(const systems::Context<double>& context, lcmt_schunk_wsg_command* wsg_command) const {
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);

  double grip = internal_state.refGrip;
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

  // publish current q (with gripper appended)
  arr q;
  copy(q, conv_stdvec2arr( internal_state.iiwa_state.joint_position ) );
  q.append( .5*.001*internal_state.wsg_state.actual_position_mm );
  internal_state.currentQ.set() = q;

  // receive the motion ref command
  if(internal_state.motionReference.hasNewRevision()){
    internal_state.ref = internal_state.motionReference.get();
    internal_state.timeToGo.set() = internal_state.ref.tau.scalar() * (internal_state.ref.path.d0-1);
  }

  // read out gripper pose from current motion ref path
  if(internal_state.ref.path.d1 == q.N){
    double timeToGo = internal_state.timeToGo.get();
    int k = floor( double(internal_state.ref.path.d0-1) - timeToGo/internal_state.ref.tau.scalar() );
    if(k<0) k=0;
    if(k>(int)internal_state.ref.path.d0-1) k=internal_state.ref.path.d0-1;
    internal_state.refGrip = internal_state.ref.path(k,q.N-1) * 2.*1000.;
  }else{
    internal_state.refGrip = internal_state.wsg_state.actual_position_mm;
  }

  // publish the robot base pose
  mlr::Transformation X;
  X.pos.set(&internal_state.iiwa_state.pose.translation.x);
  X.rot.set(&internal_state.iiwa_state.pose.rotation.w);
  internal_state.robotBase.set() = X;

  // publish perception of the object
  PerceptSimpleL P;
  PerceptSimple *p = new PerceptSimple();
  p->pose.pos.set(&internal_state.obj_state.pose.translation.x);
  p->pose.rot.set(&internal_state.obj_state.pose.rotation.w);
  P.append(p);
  if(P.N) internal_state.percepts_input.set()->append(P);

//  cout <<"HERE pathDim=" <<internal_state.ref.path.dim() <<" grip=" << internal_state.refGrip <<endl;

}

}  // namespace drake
