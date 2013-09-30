#include "motion.h"

void reattachShape(const char* objShape, const char* toBody);

struct ActionProgressor:Process {
  MotionFuture *motionFuture;
  
  ActionProgressor(MotionFuture&);
  ~ActionProgressor(){};
  void open(){};
  void step();
  void close(){};
};

Process* newActionProgressor(MotionFuture& a){
  return new ActionProgressor(a);
}

ActionProgressor::ActionProgressor(MotionFuture& a)
:Process("ActionProgressor"), motionFuture(&a) {
  if(!motionFuture) biros().getVariable(motionFuture, "MotionFuture", this);
  listenTo(motionFuture);
  MotionPrimitive *mp = motionFuture->getCurrentMotionPrimitive(this);
  if(mp) listenTo(mp);
}

void ActionProgressor::step(){
  //TODO: the progressor shouldn't wait itself - it should be triggered listening to the primitive and future
  
  //if(motionFuture->getTodoFrames(this) <= 1) return; //no future to progress to
  MotionPrimitive *mp0 = motionFuture->getCurrentMotionPrimitive(this);
  MotionPrimitive *mp1 = motionFuture->getNextMotionPrimitive(this);
  listenTo(mp0);
  if(!mp1) return;
  listenTo(mp1);
  if(mp0->get_mode(this) != MotionPrimitive::done) return;
  if(mp1->get_action(this) == MotionPrimitive::toBeAssigned) return;

  //-- YES, progress to new primitive

  switch(mp0->get_action(this)){ //wait, depending on current action
    case MotionPrimitive::grasp:
      reattachShape(mp0->get_objectRef1(this), "m9");
      break;
    case MotionPrimitive::place_location:
    case MotionPrimitive::place:
      reattachShape(mp0->get_objectRef1(this), "OBJECTS");
      break;
    default: //don't do anything
      break;
  }
  mp0->set_action(MotionPrimitive::toBeAssigned, this);

  //reset the frame0 of the motion primitive to the real hardware pose! -> triggers the MotionPlanner to refine!
  VAR(HardwareReference);
  arr x0 =  _HardwareReference->get_q_reference(this);
  x0.append(_HardwareReference->get_v_reference(this));
  x0.subRange(x0.N/2,-1) = 0.;
  //cout <<"0-state! in motion progressor\n" <<x0 <<"\n ...frame=" <<f->currentFrame <<endl;
  mp1->writeAccess(this);
  mp1->frame0 = x0;
  mp1->planConverged = false;
  _HardwareReference->set_motionPrimitiveRelativeTime(0., this);
  mp1->deAccess(this);

  motionFuture->incrementFrame(this);
}

void reattachShape(ors::Graph& ors, SwiftInterface *swift, const char* objShape, const char* toBody){
  ors::Shape *obj  = ors.getShapeByName(objShape);
  obj->body->shapes.removeValue(obj);
  obj->body = ors.getBodyByName(toBody);
  obj->ibody = obj->body->index;
  obj->body->shapes.append(obj);
  obj->rel.setDifference(obj->body->X, obj->X);
  if(swift) swift->initActivations(ors);
}

void reattachShape(const char* objShape, const char* toBody){
  VAR(GeometricState);
  VAR(HardwareReference);
  _GeometricState->writeAccess(NULL);
  arr q=_HardwareReference->get_q_reference(NULL);
  arr v=_HardwareReference->get_v_reference(NULL);
  _GeometricState->ors.setJointState(q,v);
  _GeometricState->ors.calcBodyFramesFromJoints();
  reattachShape(_GeometricState->ors, NULL, objShape, toBody);
  _GeometricState->deAccess(NULL);
}
