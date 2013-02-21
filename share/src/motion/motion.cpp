#include "motion.h"

void reattachShape(const char* objShape, const char* toBody);

struct MDummyForceGlobal{
  MDummyForceGlobal(){
    cout <<"MOTION!" <<endl;
  }
} Mdummy;


void MotionPrimitive::setNewAction(const ActionPredicate _action, const char *ref1, const char *ref2, const arr& locref, Process *p){
  writeAccess(p);
  action = _action;
  mode = none;
  objectRef1 = (char*)ref1;
  objectRef2 = (char*)ref2;
  if(&locref) locationRef = locref;  
  deAccess(p);
}

void MotionPrimitive::setClearPlanTask(const arr& frame0_pose, Process *p){
  writeAccess(p);
  mode = MotionPrimitive::planned;
  frame0 = frame0_pose;
  frame1.clear();
  planConverged = false;
  deAccess(p);
}

void MotionPrimitive::setFeedbackTask(FeedbackControlTaskAbstraction& task, bool _forceColLimTVs, bool _fixFingers, Process* p){
  writeAccess(p);
  mode = MotionPrimitive::feedback;
  feedbackControlTask = &task;
  forceColLimTVs=_forceColLimTVs;
  fixFingers=_fixFingers;
  deAccess(p);
}


void MotionFuture::appendNewAction(const MotionPrimitive::ActionPredicate _action, const char *ref1, const char *ref2, const arr& locref, Process *p){
  

#if 0
  writeAccess(p);

  MotionPrimitive *m = motions.append(new MotionPrimitive); //append a new motion primitive

  m->writeAccess(p);
  m->count = motions.N-1;
  if(!m->frame0.N){
      //check if at least 1 keyframe exists
    VAR(HardwareReference);
    arr x0 =  _HardwareReference->get_q_reference(NULL);
    x0.append(_HardwareReference->get_v_reference(NULL));
    m->frame0 = x0;
  }
  m->deAccess(p);

  m->setNewAction(_action, ref1, ref2, locref, p);
  
  Process *planner = newMotionPlanner(*m);
  planners.append((MotionPlanner*)planner);
  planner->threadStep();

  deAccess(p);
#else
  MotionPrimitive *m = get_motions(p)(nextFreePrimitive);
  while(m->get_action(p)!=MotionPrimitive::toBeAssigned)
    m->waitForNextWriteAccess();

  writeAccess(p);
  m->writeAccess(p);
  m->count = nextFreePrimitive;
  if(!m->frame0.N){
      //check if at least 1 keyframe exists
    VAR(HardwareReference);
    arr x0 =  _HardwareReference->get_q_reference(NULL);
    x0.append(_HardwareReference->get_v_reference(NULL));
    m->frame0 = x0;
  }
  m->frame1.clear();
  m->planConverged = false;
  m->deAccess(p);

  m->setNewAction(_action, ref1, ref2, locref, p);

  /*Process *planner = newMotionPlanner(*m);
  planners.append((MotionPlanner*)planner);
  planner->threadStep();*/

  nextFreePrimitive++;
  if(nextFreePrimitive>=motions.N) nextFreePrimitive=0;

  deAccess(p);
#endif
}
