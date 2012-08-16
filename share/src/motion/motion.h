#ifndef MT_motion_h
#define MT_motion_h

#include <biros/biros.h>
#include <biros/biros_views.h>
#include <MT/ors.h>
#include <MT/opengl.h>

struct SkinPressure;
struct JoystickState;
struct FeedbackControlTaskAbstraction;
struct MotionPlanner;

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
  FIELD(charp, objectRef1);  //arguments to the relational predicates
  FIELD(charp, objectRef2);
  
  Action():Variable("Action"), frameCount(0), action(noAction), executed(false), objectRef1(""), objectRef2("") {
    reg_frameCount(); reg_action(); reg_executed(); reg_objectRef1(); reg_objectRef2();
  };
  
  void setNewAction(const ActionPredicate _action, const char *ref1, const char *ref2, Process *p);
};


/** \brief A keyframe represents a pose at the beginning and end of a motion primitive, that is, in between two symbolic actions. */
struct MotionKeyframe:Variable {
  FIELD(uint, frameCount);
  FIELD(arr, x_estimate);
  FIELD(double, duration_estimate);
  FIELD(bool, converged);
  
  MotionKeyframe():Variable("MotionKeyFrame"), frameCount(0), converged(false) {
    reg_frameCount(); reg_x_estimate(); reg_duration_estimate(); reg_converged();
  };
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

  //only for info - to enable a view
  //FIELD(TaskVariableList, TVs);
  
  MotionPrimitive():Variable("MotionPrimitive"),
      frameCount(0), 
      mode(stop),
      frame0(NULL), frame1(NULL), planConverged(false),
      feedbackControlTask(NULL),
      forceColLimTVs(true) {
	reg_frameCount(); reg_mode(); reg_frame0(); reg_frame1(); reg_q_plan(); reg_tau(); reg_planConverged();
	reg_iterations_till_convergence(); reg_cost();
	reg_feedbackControlTask(); reg_fixFingers(); reg_forceColLimTVs();
      };
  
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
  FIELD(double, motionPrimitiveRelativeTime);
  
  FIELD(bool, readHandFromReal);
  
  HardwareReference():Variable("HardwareReference"), hardwareRealTime(0.), motionPrimitiveRelativeTime(0.), readHandFromReal(true) {
    reg_q_reference(); reg_q_real(); reg_hardwareRealTime(); reg_readHandFromReal(); reg_motionPrimitiveRelativeTime();
  };
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
  FIELD(MT::Array<MotionPlanner*>, planners);
  
  MotionFuture():Variable("MotionFuture"), currentFrame(0), done(true) {
    reg_currentFrame(); reg_actions(); reg_motions(); reg_frames(); reg_planners();
  };
  
  void appendNewAction(const Action::ActionPredicate _action, const char *ref1, const char *ref2, Process *p);
  void incrementFrame(Process *p){ writeAccess(p); currentFrame++; deAccess(p); }
  uint getTodoFrames(Process *p){ readAccess(p); uint n=motions.N-currentFrame; deAccess(p); return n; }
  MotionPrimitive *getCurrentMotionPrimitive(Process *p){ if(!getTodoFrames(p)) return NULL; readAccess(p); MotionPrimitive *m=motions(currentFrame); deAccess(p); return m; }
  Action *getCurrentAction(Process *p){ readAccess(p); Action *a=actions(currentFrame); deAccess(p); return a; }
};


//===========================================================================
//
// Processes
//

//either MotionFuture or MotionPrimitive must be set, one can be zero;
Process* newMotionController(HardwareReference*, MotionPrimitive*, MotionFuture*);

Process* newMotionPlanner(Action&, MotionKeyframe&, MotionKeyframe&, MotionPrimitive&);

Process* newActionProgressor(MotionFuture&);




//===========================================================================
//
// Views
//

struct PoseView:View{
  WorkingCopy<GeometricState> geo;
  uint t;
  PoseView();
  PoseView(struct FieldInfo* field);
  void glInit();
  void glDraw();
  void gtkNew(GtkWidget *container){ if(!container) container=gtkTopWindow("PoseView"); gtkNewGl(container); }
};


#include "MotionPlanner.h"

#endif
