
//===========================================================================
//
// Variables
//

struct MotionKeyframe:Variable{
  MotionKeyframe();

  bool converged;

  //outputs
  arr q_estimate;
  double duration_estimate;

  //problem formulation
  arr q_prev;
  //optional for later:
  //arr q_next; optimize also w.r.t. future keyframe
  arr W; //diagonal of the control cost matrix
  arr Phi, rho; //task cost descriptors
};


struct MotionPlan:Variable{
  MotionPlan();
  
  //output
  bool converged;
  arr q_plan;
  double tau;

  //problem description
  bool hasGoal; //if there is no goal (=tasks) given, the planner may sleep
  //FUTURE:
  //arr W; //diagonal of the control cost matrix
  //arr Phi, rho; //task cost descriptors
  //...for now: do it conventionally: task list or socSystem??
};


struct MotionControllerMode:Variable{
  MotionControllerMode();

  enum ControllerMode { noType=0, followTrajectory, feedback, done  };
  //optional: followWithFeedback

  ControllerMode mode;
};


struct MotionControllerFollowTrajectryConfig:Variable{
  MotionControllerFollowTrajectry();

  //followTrajectroy mode:
  double followTrajectoryTimeScale; //real in [0,1]
  double relativeRealTimeOfController;
};


struct MotionControllerFeedbackConfig:Variable{
  MotionControllerFeedbackConfig();

  //feedback mode:
  FeedbackTaskAbstraction *feedbackControlTask;
};


struct MotionReference:Variable{
  MotionReference();

  arr q_reference, v_reference;
  arr q_real;
  
  uintA armMotorIndices, handMotorIndices;
  bool readHandFromReal;
};


struct MotionPlan:Variable{
  MotionPlan();
  
  //output
  bool converged;
  arr q_plan;
  double tau;

  //problem description
  arr W; //diagonal of the control cost matrix
  arr Phi, rho; //task cost descriptors
};

struct Action{
  enum ActionPredicate { pick, place, home };
  uint objectRef1, objectRef2; //arguments to the relational predicates
};

struct ActionPlan:Variable{
  ActionPlan();

  MT::Array<Action> a_plan;
};

//===========================================================================
//
// Processes
//

struct MotionControllerProcess:Process{
  MotionControllerProcess();
 
  MotionControllerMode *varControllerMode;
  MotionControllerFollowTrajectryConfig *varFollowTrajectryConfig;
  MotionControllerFeedbackConfig *varFeedbackConfig;
  MotionPlan *varMotionPlan;
  MotionReference *varMotionReference;
  GeometricState *varGeometricState;
  SkinPressureVar *skinPressureVar;

  struct sMotionControllerProcess *s;

  void open();
  void step();
  void close();
};

struct MotionPlanner:Process{
  MotionPlan *varPlan;
  GeometricState *varGeometricState;

  struct sMotionPlanner *s;
  
  MotionPlanner();
  void open();
  void step();
  void close();
};

struct MotorPrimitive:Process{
  ActionPlan *varPlan;
  GeometricState *varGeometricState;
  
};

