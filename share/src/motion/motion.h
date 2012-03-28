#ifndef MT_motion_h
#define MT_motion_h

#include <biros/biros.h>
#include <MT/ors.h>
#include <MT/opengl.h>

struct SkinPressure;
struct JoystickState;
struct FeedbackControlTaskAbstraction;

//===========================================================================
//
// Variables
//

struct GeometricState:Variable {
  FIELD(ors::Graph, ors);
  
  GeometricState();
  ors::Graph& get_ors() { return ors; }
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
  FIELD(TaskVariableList, TVs);
  
  MotionPlan():Variable("MotionPlan"), hasGoal(false), converged(false), final_keyframe(NULL) { };
  void get_poseView(arr& q) { q=q_plan; }
};

struct ControllerTask:Variable {
  enum ControllerMode { stop=0, followPlan, feedback, done  };
  //optional: followWithFeedback
  
  FIELD(ControllerMode, mode);
  FIELD(bool, fixFingers);
  //for followTrajectroy mode:
  FIELD(double, relativeRealTimeOfController);
  //for feedback mode:
  FIELD(bool, forceColLimTVs);
  FIELD(FeedbackControlTaskAbstraction*, feedbackControlTask);
  
  ControllerTask():Variable("ControllerTask"),
      mode(stop),
      relativeRealTimeOfController(0.),
      forceColLimTVs(true), feedbackControlTask(NULL) {};
};


struct HardwareReference:Variable {
  FIELD(arr, q_reference);
  FIELD(arr, v_reference);
  FIELD(arr, q_real);
  FIELD(double, hardwareRealTime);
  
  FIELD(bool, readHandFromReal);
  
  HardwareReference():Variable("HardwareReference"), hardwareRealTime(0.), readHandFromReal(true) {};
  void get_poseView(arr& q) { q=q_reference; }
};


struct Action:Variable {
  enum ActionPredicate { noAction, grasp, place, home };
  
  FIELD(ActionPredicate, action);
  FIELD(bool, executed);
  FIELD(char*, objectRef1);  //arguments to the relational predicates
  FIELD(char*, objectRef2);
  //FIELD(char*, objectRef3);
  
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

PROCESS(Controller)

PROCESS(MotionPlanner)

struct MotionPrimitive:Process {
  struct sMotionPrimitive *s;
  
  //ActionPlan *actionPlan; TODO: in future use an action plan instead of just the next action
  Action *action;
  MotionKeyframe *frame0,*frame1;
  MotionPlan *plan;
  
  MotionPrimitive(Action&, MotionKeyframe&, MotionKeyframe&, MotionPlan&);
  ~MotionPrimitive();
  void open();
  void step();
  void close();
};




//===========================================================================
//
// Viewer (will be redundant with a more generic GUI)
//

template<class T>
struct PoseViewer:Process {
  T *var;
  WorkingCopy<GeometricState> geo;
  OpenGL *gl;
  
  PoseViewer(T& v):Process("PoseViewer"), var(&v), gl(NULL) {
    geo.init("GeometricState", this);
    threadListenTo(var);
  }
  void open() {
    geo.pull();
//     geo->writeAccess(this);
//     ors = geo->ors.newClone();
//     geo->deAccess(this);
    gl = new OpenGL(var->name);
    gl->add(glStandardScene);
    gl->add(ors::glDrawGraph, &geo().ors);
    gl->camera.setPosition(5, -10, 10);
    gl->camera.focus(0, 0, 1);
    gl->camera.upright();
    gl->update();
  }
  void close() {
    delete gl;
    gl = NULL;
  }
  void step() {
    geo.pull();
    arr q;
    var->readAccess(this);
    var->get_poseView(q);
    var->deAccess(this);
    if (q.nd==1) {
      if (q.N==2*geo().ors.getJointStateDimension())
        q = q.sub(0,q.N/2-1); //check dynamic state
      geo().ors.setJointState(q);
      geo().ors.calcBodyFramesFromJoints();
      gl->text.clear() <<"pose view";
      gl->update();
    } else {
      for (uint t=0; t<q.d0; t++) {
        geo().ors.setJointState(q[t]);
        geo().ors.calcBodyFramesFromJoints();
        gl->text.clear() <<"pose view at step " <<t <<"/" <<q.d0-1;
        gl->update();
      }
    }
  }
};

template<class T>
struct OrsViewer:Process {
  T *var;
  WorkingCopy<GeometricState> geo;
  OpenGL *gl;
  
  OrsViewer(T& v, GeometricState& g):Process("OrsViewer"), var(&v), gl(NULL) {
    geo.init("GeometricState", this);
    threadListenTo(var);
  }
  void open() {
    geo.pull();
//     geo->writeAccess(this);
//     ors = geo->ors.newClone();
//     geo->deAccess(this);
    gl = new OpenGL(var->name);
    gl->add(glStandardScene);
    gl->add(ors::glDrawGraph, &geo().ors);
    gl->camera.setPosition(5, -10, 10);
    gl->camera.focus(0, 0, 1);
    gl->camera.upright();
    gl->update();
  }
  void close() {
    delete gl;
    gl = NULL;
  }
  void step() {
    arr q;
    geo.pull();
    gl->text.clear() <<"ors view of Variable " <<var->name;
    gl->update();
  }
};

#include "MotionPrimitive.h"

#endif
