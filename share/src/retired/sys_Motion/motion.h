#ifndef MLR_motion_h
#define MLR_motion_h

#include <System/biros.h>

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
struct GamepadState;

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
  ors::KinematicWorld ors;
  GeometricState();
  ors::KinematicWorld& get_ors() { return ors; }
};


/** @brief A motion primitive is the motion-grounding of a symbolic action. It can be a feedback control task, or a planned motion.
 In the first case, the MotionPrimitive is given a FeedbackControlTaskAbstraction, which implements the necessary task variable updates for a feedback controller.
 In the planned case, a motion planner first generates a trajectroy (q_plan), then this is followed by the controller */
struct MotionPrimitive:Variable {
  enum MotionMode{ none=0, planned, feedback, done  };
  enum ActionPredicate { toBeAssigned, reach, grasp, place, place_location, openHand, closeHand, homing };

  uint count;
  MotionMode mode;

  //-- symbolic action
  ActionPredicate action;
  charp objectRef1;  //arguments to the relational predicates
  charp objectRef2;
  arr locationRef;

  //-- motion
  //in case of planned
  arr frame0;
  arr frame1;
  arr q_plan;
  double tau;
  double duration;
  bool planConverged;
  uint iterations_till_convergence;
  double cost;

  //in case of feedback
  FeedbackControlTaskAbstraction* feedbackControlTask;

  //controller options
  bool fixFingers;
  bool forceColLimTVs;

  //only for info - to enable a view
  //TaskVariableList TVs;
  
  MotionPrimitive()
    :Variable("MotionPrimitive"),
      count(0),
      mode(none),
      action(toBeAssigned), objectRef1(""), objectRef2(""),
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


/** @brief The HardwareReference is the interface to motors, containing the reference pose for the motor controllers and
 * their return values (q_real). */
struct HardwareReference:Variable {
  arr q_reference;
  arr v_reference;
  arr q_real;
  arr v_real;
  double hardwareRealTime;
  double motionPrimitiveRelativeTime;
  
  bool readHandFromReal;
  
  HardwareReference():Variable("HardwareReference"), hardwareRealTime(0.), motionPrimitiveRelativeTime(0.), readHandFromReal(true) {
    reg_q_reference(); reg_q_real(); reg_v_real(); reg_hardwareRealTime(); reg_readHandFromReal(); reg_motionPrimitiveRelativeTime();
  };
};


/** @brief The MotionFuture contains a whole queue of future actions, motion primitives and keyframes. This allows parallel
 * planning of the motion primitives even when the action will only be in the future. The ActionProgressor takes care to point
 * the controller to the next motion primitive when the previous one was executed */
struct MotionFuture:Variable {
  uint currentPrimitive;
  uint nextFreePrimitive;
  bool done;
  mlr::Array<MotionPrimitive*> motions;
  mlr::Array<MotionPlanner*> planners;
  
  MotionFuture():Variable("MotionFuture"), currentPrimitive(0), nextFreePrimitive(0), done(true) {
    reg_currentPrimitive();  reg_nextFreePrimitive(); reg_motions();
#if 1
    motions.append(new MotionPrimitive); //append a new motion primitive
    motions.append(new MotionPrimitive); //append a new motion primitive
    planners.append((MotionPlanner*)newMotionPlanner(*motions(0)));
    planners.append((MotionPlanner*)newMotionPlanner(*motions(1)));
#endif
  };
  
  void appendNewAction(const MotionPrimitive::ActionPredicate _action, const char *ref1, const char *ref2, const arr& locref, Process *p);
  void incrementFrame(Process *p){ writeAccess(p); currentPrimitive++; if(currentPrimitive>=motions.N) currentPrimitive=0; deAccess(p); }
  uint getTodoFrames(Process *p){ readAccess(p); uint n=motions.N-currentPrimitive; deAccess(p); return n; }
  MotionPrimitive *getCurrentMotionPrimitive(Process *p){ if(!getTodoFrames(p)) return NULL; readAccess(p); MotionPrimitive *m=motions(currentPrimitive); deAccess(p); return m; }
  MotionPrimitive *getNextMotionPrimitive(Process *p){
    //if(!(getTodoFrames(p)>=2)) return NULL;
    readAccess(p); MotionPrimitive *m=motions((currentPrimitive+1)%motions.N); deAccess(p); return m; }
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
