#pragma once

#include <memory>
#include <string>
#include <vector>
#include <Core/array.h>

//#include "bot_core/robot_state_t.hpp"

//#include "pick_and_place_state_machine.h"
//#include "world_state.h"
//#include "drake/manipulation/planner/constraint_relaxing_ik.h"
//#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
//#include "drake/systems/framework/system_symbolic_inspector.h"
//#include "drake/lcmtypes/drake/lcmt_schunk_wsg_status.hpp"
//#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/robotlocomotion/robot_plan_t.hpp"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"

namespace drake {

/**
 * A class that implements the Finite-State-Machine logic for the
 * Pick-And-Place demo. This system should be used by coupling the outputs with
 * the `IiwaMove` and `GripperAction` systems and the inputs are to be
 * connected to the appropriate output ports of the `IiwaStatusSender`,
 * `SchunkWsgStatusSender` and `OracularStateEstimator` systems.
 */
class RAI_Machine : public systems::LeafSystem<double> {
public:

    //outputs:
    arr path;
    double grip;


  /**
   * Constructor for the RAI_Machine
   * @param iiwa_base, The pose of the base of the IIWA robot system.
   * @param period_sec : The update interval of the unrestricted update of
   * this system. This should be bigger than that of the PlanSource components.
   */
  RAI_Machine(const double period_sec = 0.01);

  std::unique_ptr<systems::AbstractValues> AllocateAbstractState() const override;

  // This kind of a system is not a direct feedthrough.
  optional<bool> DoHasDirectFeedthrough(int, int) const final { return false;  }

  void setPath(const arr& X);
  void setGrip(double x);

  void SetDefaultState(const systems::Context<double>& context,
                       systems::State<double>* state) const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
                                systems::State<double>* state) const override;

  /**
   * Getter for the input port corresponding to the abstract input with iiwa
   * state message (LCM `robot_state_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_iiwa_state() const {
    return this->get_input_port(input_port_iiwa_state_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with box
   * state message (LCM `botcore::robot_state_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_box_state() const {
    return this->get_input_port(input_port_box_state_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with the wsg
   * status message (LCM `lcmt_schunk_wsg_status` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_wsg_status() const {
    return this->get_input_port(input_port_wsg_status_);
  }

  const systems::OutputPort<double>& get_output_port_iiwa_plan() const {
    return this->get_output_port(output_port_iiwa_plan_);
  }

  const systems::OutputPort<double>& get_output_port_wsg_command() const {
    return this->get_output_port(output_port_wsg_command_);
  }

 private:
  void CalcIiwaPlan(const systems::Context<double>& context, robotlocomotion::robot_plan_t* iiwa_plan) const;
  void CalcWsgCommand(const systems::Context<double>& context, lcmt_schunk_wsg_command* wsg_command) const;

  struct InternalState;

  // Input ports.
  int input_port_iiwa_state_{-1};
  int input_port_box_state_{-1};
  int input_port_wsg_status_{-1};
  // Output ports.
  int output_port_iiwa_plan_{-1};
  int output_port_wsg_command_{-1};
};

}  // namespace drake
