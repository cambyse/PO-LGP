
#include <Core/module.h>
#include <Motion/feedbackControl.h>
#include <pr2/roscom.h>

extern struct TaskControllerModule *taskControllerModule();

struct TaskControllerModule : Module {
  //protected access points
  ACCESS(CtrlMsg, ctrl_ref) //< the message send to the RTController
  ACCESS(CtrlMsg, ctrl_obs) //< the message received from the RTController
  ACCESS(MT::Array<CtrlTask*>, ctrlTasks)
  ACCESS(MT::String, effects)

  //non-protected members
  Mutex mutex;
  ors::KinematicWorld realWorld;
  ors::KinematicWorld modelWorld;
  FeedbackMotionControl feedbackController;
  arr q_real, qdot_real; //< real state
  arr q_model, qdot_model; //< model state
  const arr q0; //< homing pose
  ofstream fil; //< file for diagnostics
  bool useRos;
  bool syncModelStateWithRos; //< whether the step() should reinit the state from the ros message
  bool verbose;

  TaskControllerModule();
  ~TaskControllerModule();

  /// @name module implementations
  void open();
  void step();
  void close();
};
