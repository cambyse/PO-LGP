#include <MT/process.h>
#include <MT/robot.h> //TODO: needs removing!


//===========================================================================
//
// Variables
//

struct GeometricState:Variable{
  ors::Graph ors;
  
  GeometricState();
};

struct MotionKeyframe:Variable{
  FIELD( arr, q_estimate );
  FIELD( double, duration_estimate );
  FIELD( bool, converged );

  //problem formulation
  arr q_prev;
  //optional for later:
  //arr q_next; optimize also w.r.t. future keyframe
  arr W; //diagonal of the control cost matrix
  arr Phi, rho; //task cost descriptors
  
  MotionKeyframe();
};


struct MotionPlan:Variable{
  FIELD( arr, q_plan );
  FIELD( double, tau );
  FIELD( bool, converged );

  //problem description
  FIELD( bool, hasGoal ); //if there is no goal (=tasks) given, the planner may sleep
  //FUTURE:
  //arr W; //diagonal of the control cost matrix
  //arr Phi, rho; //task cost descriptors
  //...for now: do it conventionally: task list or socSystem??

  MotionPlan();
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
  
  ControllerTask();
};


struct ControllerReference:Variable{

  FIELD( arr, q_reference );
  FIELD( arr, v_reference );
  FIELD( arr, q_real );
  
  uintA armMotorIndices, handMotorIndices;
  bool readHandFromReal;
  
  ControllerReference();
};


struct Action{
  enum ActionPredicate { pick, place, home };
  uint objectRef1, objectRef2; //arguments to the relational predicates
  
  Action();
};


struct ActionPlan:Variable{
  MT::Array<Action> a_plan;

  ActionPlan();
};

//===========================================================================
//
// Processes
//

struct Controller:Process{
  struct sMotionControllerProcess *s;

  //links
  ControllerTask *controllerMode;
  MotionPlan *motionPlan;
  ControllerReference *controllerReference;
  GeometricState *geometricState;
  SkinPressureVar *skinPressure;

  //parameters
  PARAM( double, maxJointStep );

  Controller();
  ~Controller();
  void open();
  void step();
  void close();
};


struct MotionPlanner:Process{
  struct sMotionPlanner *s;

  //links
  MotionPlan *motionPlan;
  GeometricState *geometricState;

  MotionPlanner();
  void open();
  void step();
  void close();
};


struct MotionPrimitive:Process{
  //links
  ActionPlan *actionPlan;
  GeometricState *geometricState;
  
  MotionPrimitive();
  void open();
  void step();
  void close();
};

