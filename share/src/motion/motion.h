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


struct Action:Variable {
  enum ActionPredicate { noAction, grasp, place, home };
  
  FIELD(ActionPredicate, action);
  FIELD(bool, executed);
  FIELD(char*, objectRef1);  //arguments to the relational predicates
  FIELD(char*, objectRef2);
  
  Action():Variable("Action"), action(noAction), executed(false), objectRef1(NULL), objectRef2(NULL) {};
  
  void setNewAction(const ActionPredicate _action, const char *ref1, const char *ref2, Process *p);
};


struct MotionKeyframe:Variable {
  FIELD(arr, x_estimate);
  FIELD(double, duration_estimate);
  FIELD(bool, converged);
  
  MotionKeyframe():Variable("MotionKeyFrame"), converged(true) {};
  void get_poseView(arr& x) { x=x_estimate; }
};


struct MotionPrimitive:Variable {
  enum MotionMode{ stop=0, followPlan, feedback, done  };

  FIELD(MotionMode, mode);
  
  //in case of followPlan
  FIELD(MotionKeyframe*, frame0);
  FIELD(MotionKeyframe*, frame1);
  FIELD(arr, q_plan);
  FIELD(double, tau);
  FIELD(bool, planConverged);

  //in case of feedback
  FIELD(FeedbackControlTaskAbstraction*, feedbackControlTask);

  //controller options
  FIELD(bool, fixFingers);
  FIELD(bool, forceColLimTVs);
  FIELD(double, relativeRealTimeOfController);

  //only for info - to enable a view
  //FIELD(TaskVariableList, TVs);
  
  MotionPrimitive():Variable("MotionPrimitive"),
      mode(stop),
      frame0(NULL), frame1(NULL), planConverged(false),
      feedbackControlTask(NULL),
      forceColLimTVs(true), relativeRealTimeOfController(0.) { };
  
  void get_poseView(arr& q) { q=q_plan; }
  void setClearPlanTask(const arr& frame0_pose, Process *p);
  void setFeedbackTask(FeedbackControlTaskAbstraction& task, bool _forceColLimTVs, bool _fixFingers, Process *p);
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


//===========================================================================
//
// Processes
//

PROCESS(Controller)

//PROCESS(MotionPlanner)

struct ActionToMotionPrimitive:Process {
  struct sActionToMotionPrimitive *s;
  
  //ActionPlan *actionPlan; TODO: in future use an action plan instead of just the next action
  Action *action;
  MotionPrimitive *motionPrimitive;
  
  ActionToMotionPrimitive(Action&, MotionKeyframe&, MotionKeyframe&, MotionPrimitive&);
  ~ActionToMotionPrimitive();
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
    uint n=geo().ors.getJointStateDimension();
    arr q;
    var->readAccess(this);
    var->get_poseView(q);
    var->deAccess(this);
    if (q.nd==1) {
      if (q.N==2*n) q = q.sub(0,q.N/2-1); //check dynamic state
      if (q.N!=n){ MT_MSG("pose view on wrong dimension");  return; }
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
  
  OrsViewer(T& v):Process("OrsViewer"), var(&v), gl(NULL) {
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

#include "ActionToMotionPrimitive.h"

#endif
