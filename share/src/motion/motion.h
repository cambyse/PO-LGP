#ifndef MT_motion_h
#define MT_motion_h

#include <biros/biros.h>

//===========================================================================
//
// fwd declarations
//

//-- Variables
struct GeometricState;
struct MotionPrimitive;
struct HardwareReference;
struct MotionFuture;
struct SkinPressure;
struct JoystickState;

//-- Process creators
//either MotionFuture or MotionPrimitive must be set, one can be zero;
Process* newMotionController(HardwareReference*, MotionPrimitive*, MotionFuture*);
Process* newMotionPlanner(MotionPrimitive&);
Process* newActionProgressor(MotionFuture&);

struct MotionPlanner;
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


/** \brief A motion primitive is the motion-grounding of a symbolic action. It can be a feedback control task, or a planned motion.
 In the first case, the MotionPrimitive is given a FeedbackControlTaskAbstraction, which implements the necessary task variable updates for a feedback controller.
 In the planned case, a motion planner first generates a trajectroy (q_plan), then this is followed by the controller */
struct MotionPrimitive:Variable {
  enum MotionMode{ none=0, planned, feedback, done  };
  enum ActionPredicate { noAction, reach, grasp, place, place_location, openHand, closeHand, homing };

  FIELD(uint, count);
  FIELD(MotionMode, mode);

  //-- symbolic action
  FIELD(ActionPredicate, action);
  FIELD(charp, objectRef1);  //arguments to the relational predicates
  FIELD(charp, objectRef2);
  FIELD(arr, locationRef);

  //-- motion
  //in case of planned
  FIELD(arr, frame0);
  FIELD(arr, frame1);
  FIELD(arr, q_plan);
  FIELD(double, tau);
  FIELD(double, duration);
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
  
  MotionPrimitive()
    :Variable("MotionPrimitive"),
      count(0),
      mode(none),
      action(noAction), objectRef1(""), objectRef2(""),
      planConverged(false),
      feedbackControlTask(NULL),
      forceColLimTVs(true) {
    reg_count(); reg_mode();
    reg_action(); reg_objectRef1(); reg_objectRef2(); reg_locationRef();
    reg_frame0(); reg_frame1(); reg_q_plan(); reg_tau(); reg_duration(); reg_planConverged();
    reg_iterations_till_convergence(); reg_cost();
    reg_feedbackControlTask(); reg_fixFingers(); reg_forceColLimTVs();
  };
  
  void setClearPlanTask(const arr& frame0_pose, Process *p);
  void setFeedbackTask(FeedbackControlTaskAbstraction& task, bool _forceColLimTVs, bool _fixFingers, Process *p);
  void setNewAction(const ActionPredicate _action, const char *ref1, const char *ref2, const arr& locref, Process *p);
};


/** \brief The HardwareReference is the interface to motors, containing the reference pose for the motor controllers and
 * their return values (q_real). */
struct HardwareReference:Variable {
  FIELD(arr, q_reference);
  FIELD(arr, v_reference);
  FIELD(arr, q_real);
  FIELD(arr, v_real);
  FIELD(double, hardwareRealTime);
  FIELD(double, motionPrimitiveRelativeTime);
  
  FIELD(bool, readHandFromReal);
  
  HardwareReference():Variable("HardwareReference"), hardwareRealTime(0.), motionPrimitiveRelativeTime(0.), readHandFromReal(true) {
    reg_q_reference(); reg_q_real(); reg_v_real(); reg_hardwareRealTime(); reg_readHandFromReal(); reg_motionPrimitiveRelativeTime();
  };
};


/** \brief The MotionFuture contains a whole queue of future actions, motion primitives and keyframes. This allows parallel
 * planning of the motion primitives even when the action will only be in the future. The ActionProgressor takes care to point
 * the controller to the next motion primitive when the previous one was executed */
struct MotionFuture:Variable {
  FIELD(uint, currentFrame);
  FIELD(bool, done);
  FIELD(MT::Array<MotionPrimitive*>, motions);
  FIELD(MT::Array<MotionPlanner*>, planners);
  
  MotionFuture():Variable("MotionFuture"), currentFrame(0), done(true) {
    reg_currentFrame(); reg_motions(); reg_planners();
  };
  
  void appendNewAction(const MotionPrimitive::ActionPredicate _action, const char *ref1, const char *ref2, const arr& locref, Process *p);
  void incrementFrame(Process *p){ writeAccess(p); currentFrame++; deAccess(p); }
  uint getTodoFrames(Process *p){ readAccess(p); uint n=motions.N-currentFrame; deAccess(p); return n; }
  MotionPrimitive *getCurrentMotionPrimitive(Process *p){ if(!getTodoFrames(p)) return NULL; readAccess(p); MotionPrimitive *m=motions(currentFrame); deAccess(p); return m; }
};


//===========================================================================
//
// Views
//

struct PoseView:View{
  WorkingCopy<GeometricState> geo;
  uint t;
  PoseView();
  PoseView(arr& q, RWLock *lock=NULL, GtkWidget *container=NULL);
  void glInit();
  void glDraw();
  virtual void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};

#endif
