#ifndef MT_motion_h
#define MT_motion_h

#include <biros/biros.h>
#include <MT/ors.h>
#include <MT/opengl.h>
#include "FeedbackControlTasks.h"

struct SkinPressure;
struct JoystickState;



//===========================================================================
//
// Variables
//

struct GeometricState:Variable {
  ors::Graph ors;
  
  GeometricState();
};


struct MotionKeyframe:Variable {
  FIELD(arr, x_estimate);
  FIELD(double, duration_estimate);
  FIELD(MotionKeyframe*, previous_keyframe);
  FIELD(MotionKeyframe*, next_keyframe);
  FIELD(bool, converged);
  
  MotionKeyframe():Variable("MotionKeyFrame"), previous_keyframe(NULL), next_keyframe(NULL), converged(false) {};
  void get_poseView(arr& x) { x=x_estimate; }
};


struct MotionPlan:Variable {
  FIELD(arr, q_plan);
  FIELD(double, tau);
  FIELD(uint, steps);
  
  //problem description
  FIELD(bool, hasGoal);   //if there is no goal (=tasks) given, the planner may sleep
  FIELD(bool, converged);
  FIELD(MotionKeyframe*, final_keyframe);
  //FUTURE:
  //arr W; //diagonal of the control cost matrix
  //arr Phi, rho; //task cost descriptors
  //...for now: do it conventionally: task list or socSystem??
  TaskVariableList TVs;
  
  MotionPlan():Variable("MotionPlan"), hasGoal(false), converged(false), final_keyframe(NULL) { };
  void get_poseView(arr& q) { q=q_plan; }
};

struct ControllerTask:Variable {
  enum ControllerMode { noType=0, followTrajectory, feedback, done  };
  //optional: followWithFeedback
  
  FIELD(ControllerMode, mode);
  FIELD(bool, fixFingers);
  //for followTrajectroy mode:
  FIELD(double, followTrajectoryTimeScale);   //real in [0,1]
  FIELD(double, relativeRealTimeOfController);
  //for feedback mode:
  FIELD(bool, forceColLimTVs);
  FIELD(FeedbackControlTaskAbstraction*, feedbackControlTask);
  
  ControllerTask():Variable("ControllerTask"),
      mode(noType),
      followTrajectoryTimeScale(1.), relativeRealTimeOfController(0.),
      forceColLimTVs(true), feedbackControlTask(NULL) {};
};


struct HardwareReference:Variable {
  FIELD(arr, q_reference);
  FIELD(arr, v_reference);
  FIELD(arr, q_real);
  FIELD(double, hardwareRealTime);
  
  uintA armMotorIndices, handMotorIndices;
  bool readHandFromReal;
  
  HardwareReference():Variable("HardwareReference"), hardwareRealTime(0.), readHandFromReal(true) {};
  void get_poseView(arr& q) { q=q_reference; }
};


struct Action:Variable {
  enum ActionPredicate { noAction, grasp, place, home };
  
  FIELD(ActionPredicate, action);
  FIELD(bool, executed);
  FIELD(char*, objectRef1);  //arguments to the relational predicates
  FIELD(char*, objectRef2);
  
  Action():Variable("Action"), action(noAction), executed(false), objectRef1(NULL), objectRef2(NULL) {};
};


struct ActionPlan:Variable {
  MT::Array<Action> a_plan;
  
  ActionPlan();
};


//===========================================================================
//
// Processes
//

struct Controller:Process {
  struct sController *s;
  
  //links
  ControllerTask *controllerTask;
  MotionPlan *motionPlan;
  HardwareReference *hardwareReference;
  GeometricState *geo;
  SkinPressure *skinPressure;
  JoystickState *joystickState;
  
  Controller();
  ~Controller();
  void open();
  void step();
  void close();
};


struct MotionPrimitive:Process {
  struct sMotionPrimitive *s;
  //links
  //ActionPlan *actionPlan; TODO: in future use an action plan instead of just the next action
  Action *action;
  MotionKeyframe *frame0,*frame1;
  MotionPlan *plan;
  GeometricState *geo;
  
  MotionPrimitive(Action&, MotionKeyframe&, MotionKeyframe&, MotionPlan&, GeometricState&);
  ~MotionPrimitive();
  void open();
  void step();
  void close();
};


struct MotionPlanner:Process {
  struct sMotionPlanner_interpolation *s;
  //links
  MotionPlan *plan;
  GeometricState *geo;
  
  enum MotionPlannerAlgo { interpolation=0, AICO_noinit } algo;
  
  MotionPlanner();
  ~MotionPlanner();
  void open();
  void step();
  void close();
};


//===========================================================================
//
// Viewers
//

template<class T>
struct PoseViewer:Process {
  T *var;
  GeometricState *geo;
  OpenGL gl;
  ors::Graph *ors;
  
  PoseViewer(T& v, GeometricState& g):Process("MotionPoseViewer"), var(&v), geo(&g), gl(v.name), ors(NULL) {
  }
  void open() {
    geo->writeAccess(this);
    ors = geo->ors.newClone();
    geo->deAccess(this);
    gl.add(glStandardScene);
    gl.add(ors::glDrawGraph, ors);
    gl.camera.setPosition(5, -10, 10);
    gl.camera.focus(0, 0, 1);
    gl.camera.upright();
    gl.update();
  }
  void close() {}
  void step() {
    arr q;
    var->readAccess(this);
    var->get_poseView(q);
    var->deAccess(this);
    if (q.nd==1) {
      if (q.N==2*ors->getJointStateDimension()) q = q.sub(0,q.N/2-1); //check dynamic state
      ors->setJointState(q);
      ors->calcBodyFramesFromJoints();
      gl.text.clr() <<"pose view";
      gl.update();
    } else {
      for (uint t=0; t<q.d0; t++) {
        ors->setJointState(q[t]);
        ors->calcBodyFramesFromJoints();
        gl.text.clr() <<"pose view at step " <<t <<"/" <<q.d0-1;
        gl.update();
      }
    }
  }
};



#include "MotionPrimitive.h"

#endif
