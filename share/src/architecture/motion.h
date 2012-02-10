#include <MT/process.h>
#include <MT/robot.h> //TODO: needs removing!


//===========================================================================
//
// fields of some variables
//

namespace ors{ struct Graph; }
/*struct TaskVectorFunction{
  virtual void get_Phi(arr& Phi, arr& J, const arr& q, ors::Graph&) = 0;
  virtual const char* taskName(uint i) = 0;
  virtual uint taskDim(uint i) = 0;
};*/

//===========================================================================
//
// Variables
//

//namespace b{

struct GeometricState:Variable{
  ors::Graph ors;
  
  GeometricState();
};

struct MotionKeyframe:Variable{
  FIELD( arr, q_estimate );
  FIELD( double, duration_estimate );
  FIELD( MotionKeyframe*, previous_keyframe );
  FIELD( MotionKeyframe*, next_keyframe );
  MotionKeyframe():Variable("MotionKeyFrame"), previous_keyframe(NULL), next_keyframe(NULL) {};
};


struct MotionPlan:Variable{
  FIELD( arr, q_plan );
  FIELD( double, tau );

  //problem description
  FIELD( bool, hasGoal ); //if there is no goal (=tasks) given, the planner may sleep
  FIELD( bool, converged );
  FIELD( MotionKeyframe*, final_keyframe );
  //FUTURE:
  //arr W; //diagonal of the control cost matrix
  //arr Phi, rho; //task cost descriptors
  //...for now: do it conventionally: task list or socSystem??

  MotionPlan():Variable("MotionPlan"), hasGoal(false), converged(false) {};
};

struct ControllerTask:Variable{
  enum ControllerMode { noType=0, followTrajectory, feedback, done  };
  //optional: followWithFeedback
  
  FIELD( ControllerMode, mode );
  FIELD( bool, forceColLimTVs );
  FIELD( bool, fixFingers );
  //for followTrajectroy mode:
  FIELD( double, followTrajectoryTimeScale ); //real in [0,1]
  FIELD( double, relativeRealTimeOfController );
  //for feedback mode:
  FIELD( TaskAbstraction*, feedbackControlTask);
  
  ControllerTask():Variable("ControllerTask"), mode(noType) {};
};


struct HardwareReference:Variable{
  FIELD( arr, q_reference );
  FIELD( arr, v_reference );
  FIELD( arr, q_real );
  
  uintA armMotorIndices, handMotorIndices;
  bool readHandFromReal;
  
  HardwareReference():Variable("HardwareReference"), readHandFromReal(true) {};
};


struct Action:Variable{
  enum ActionPredicate { noAction, grasp, place, home };

  FIELD( ActionPredicate, action);
  FIELD( bool, executed );
  FIELD( char*, objectRef1); //arguments to the relational predicates
  FIELD( char*, objectRef2);
  
  
  Action():Variable("Action"), action(noAction), executed(false), objectRef1(NULL), objectRef2(NULL) {};
};


struct ActionPlan:Variable{
  MT::Array<Action> a_plan;

  ActionPlan();
};

//===========================================================================
//
// Processes
//

struct myController:Process{
  struct sMotionControllerProcess *s;

  //links
  ControllerTask *controllerTask;
  MotionPlan *motionPlan;
  HardwareReference *hardwareReference;
  GeometricState *geometricState;
  SkinPressureVar *skinPressure;

  //parameters
  PARAM( double, maxJointStep );

  myController();
  ~myController();
  void open();
  void step();
  void close();
};


struct MotionPlanner_AICO:Process{
  struct sMotionPlanner *s;

  //links
  MotionPlan *motionPlan;
  GeometricState *geometricState;

  MotionPlanner_AICO();
  ~MotionPlanner_AICO();
  void open();
  void step();
  void close();
};


struct MotionPlanner_interpolation:Process{
  struct sMotionPlanner_interpolation *s;
  //links
  MotionPlan *motionPlan;
  GeometricState *geometricState;

  MotionPlanner_interpolation();
  ~MotionPlanner_interpolation();
  void open();
  void step();
  void close();
};

struct KeyframeEstimator:Process{
  struct sKeyframeEstimator *s;

  //links
  Action* action;
  MotionKeyframe *motionKeyframe;
  GeometricState *geometricState;

  KeyframeEstimator();
  ~KeyframeEstimator();
  void open();
  void step();
  void close();
  
};

struct MotionPrimitive:Process{
  struct sMotionPrimitive *s;
  //links
  //ActionPlan *actionPlan; TODO: in future use an action plan instead of just the next action
  Action *action;
  MotionPlan *motionPlan;
  MotionKeyframe *motionKeyframe;
  GeometricState *geometricState;
  
  MotionPrimitive();
  ~MotionPrimitive();
  void open();
  void step();
  void close();
};
//}
