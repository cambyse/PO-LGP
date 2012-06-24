#ifndef MT_motion_h
#define MT_motion_h

#include <biros/biros.h>
#include <MT/ors.h>
#include <MT/opengl.h>

struct SkinPressure;
struct JoystickState;
struct FeedbackControlTaskAbstraction;
struct ActionToMotionPrimitive;

//===========================================================================
//
// Variables
//

struct GeometricState:Variable {
  FIELD(ors::Graph, ors);
  GeometricState();
  ors::Graph& get_ors() { return ors; }
};


/** \brief Represents single symbolic action. Is associated one-to-one with a MotionPrimitive. */
struct Action:Variable {
  //grasp: goto object, close hand, attach shape to hand
  //reach: goto object or move to location, but do not attach shape
  enum ActionPredicate { noAction, reach, grasp, place, openHand, closeHand, home };
  
  FIELD(uint, frameCount);
  FIELD(ActionPredicate, action);
  FIELD(bool, executed);
  FIELD(char*, objectRef1);  //arguments to the relational predicates
  FIELD(char*, objectRef2);
  
  Action():Variable("Action"), frameCount(0), action(noAction), executed(false), objectRef1(NULL), objectRef2(NULL) {};
  
  void setNewAction(const ActionPredicate _action, const char *ref1, const char *ref2, Process *p);
};


/** \brief A keyframe represents a pose at the beginning and end of a motion primitive, that is, in between two symbolic actions. */
struct MotionKeyframe:Variable {
  FIELD(uint, frameCount);
  FIELD(arr, x_estimate);
  FIELD(double, duration_estimate);
  FIELD(bool, converged);
  
  MotionKeyframe():Variable("MotionKeyFrame"), frameCount(0), converged(false) {};
  void get_poseView(arr& x) { x=x_estimate; }
};


/** \brief A motion primitive is the motion-grounding of a symbolic action. It can be a feedback control task, or a planned motion.
 In the first case, the MotionPrimitive is given a FeedbackControlTaskAbstraction, which implements the necessary task variable updates for a feedback controller.
 In the planned case, a motion planner first generates a trajectroy (q_plan), then this is followed by the controller */
struct MotionPrimitive:Variable {
  enum MotionMode{ stop=0, followPlan, feedback, done  };

  FIELD(uint, frameCount);
  FIELD(MotionMode, mode);
  
  //in case of followPlan
  FIELD(MotionKeyframe*, frame0);
  FIELD(MotionKeyframe*, frame1);
  FIELD(arr, q_plan);
  FIELD(double, tau);
  FIELD(bool, planConverged);
  FIELD(uint, iterations_till_convergence);
  FIELD(double, cost);

  //in case of feedback
  FIELD(FeedbackControlTaskAbstraction*, feedbackControlTask);

  //controller options
  FIELD(bool, fixFingers);
  FIELD(bool, forceColLimTVs);
  FIELD(double, relativeRealTimeOfController);

  //only for info - to enable a view
  //FIELD(TaskVariableList, TVs);
  
  MotionPrimitive():Variable("MotionPrimitive"),
      frameCount(0), 
      mode(stop),
      frame0(NULL), frame1(NULL), planConverged(false),
      feedbackControlTask(NULL),
      forceColLimTVs(true), relativeRealTimeOfController(0.) { };
  
  void get_poseView(arr& q) { q=q_plan; }
  void setClearPlanTask(const arr& frame0_pose, Process *p);
  void setFeedbackTask(FeedbackControlTaskAbstraction& task, bool _forceColLimTVs, bool _fixFingers, Process *p);
};


/** \brief The HardwareReference is the interface to motors, containing the reference pose for the motor controllers and
 * their return values (q_real). */
struct HardwareReference:Variable {
  FIELD(arr, q_reference);
  FIELD(arr, v_reference);
  FIELD(arr, q_real);
  FIELD(double, hardwareRealTime);
  
  FIELD(bool, readHandFromReal);
  
  HardwareReference():Variable("HardwareReference"), hardwareRealTime(0.), readHandFromReal(true) {};
  void get_poseView(arr& q) { q=q_reference; }
};


/** \brief The MotionFuture contains a whole queue of future actions, motion primitives and keyframes. This allows parallel
 * planning of the motion primitives even when the action will only be in the future. The ActionProgressor takes care to point
 * the controller to the next motion primitive when the previous one was executed */
struct MotionFuture:Variable {
  FIELD(uint, currentFrame);
  FIELD(bool, done);
  FIELD(MT::Array<Action*>, actions)
  FIELD(MT::Array<MotionPrimitive*>, motions);
  FIELD(MT::Array<MotionKeyframe*>, frames);
  FIELD(MT::Array<ActionToMotionPrimitive*>, planners);
  
  MotionFuture():Variable("MotionFuture"), currentFrame(0), done(true) {};
  
  void appendNewAction(const Action::ActionPredicate _action, const char *ref1, const char *ref2, Process *p);
  void incrementFrame(Process *p){ writeAccess(p); currentFrame++; deAccess(p); }
  uint getTodoFrames(Process *p){ readAccess(p); uint n=motions.N-currentFrame; deAccess(p); return n; }
  MotionPrimitive *getCurrentMotionPrimitive(Process *p){ readAccess(p); MotionPrimitive *m=motions(currentFrame); deAccess(p); return m; }
  Action *getCurrentAction(Process *p){ readAccess(p); Action *a=actions(currentFrame); deAccess(p); return a; }
};

//===========================================================================
//
// Processes
//

PROCESS(Controller)

PROCESS(ActionProgressor)

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
