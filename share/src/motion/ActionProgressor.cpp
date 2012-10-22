#include "motion_internal.h"

#define VAR(Type) \
  Type *_##Type;  birosInfo().getVariable<Type>(_##Type, #Type, NULL);

void reattachShape(const char* objShape, const char* toBody);

Process* newActionProgressor(MotionFuture& a){
  return new ActionProgressor(a);
}

ActionProgressor::ActionProgressor(MotionFuture& a)
:Process("ActionProgressor"), motionFuture(&a) {
  if(!motionFuture) birosInfo().getVariable(motionFuture, "MotionFuture", this);
  threadListenTo(motionFuture);
  MotionPrimitive *mp = motionFuture->getCurrentMotionPrimitive(this);
  if(mp) threadListenTo(mp);
}

void ActionProgressor::step(){
  //TODO: the progressor shouldn't wait itself - it should be triggered listening to the primitive and future
  
  if(motionFuture->getTodoFrames(this) <= 1) return; //no future to progress to
  MotionPrimitive *mp = motionFuture->getCurrentMotionPrimitive(this);
  threadListenTo(mp);
  
  if(mp->get_mode(this) != MotionPrimitive::done) return;
  switch(mp->get_action(this)){ //wait, depending on current action
    case MotionPrimitive::grasp:
      reattachShape(mp->get_objectRef1(this), "m9");
      break;
    case MotionPrimitive::place_location:
    case MotionPrimitive::place:
      reattachShape(mp->get_objectRef1(this), "OBJECTS");
      break;
    default: //don't do anything
      break;
  }

  motionFuture->incrementFrame(this);
  mp = motionFuture->getCurrentMotionPrimitive(this);
  threadListenTo(mp);
  
  //reset the frame0 of the motion primitive to the real hardware pose! -> triggers the MotionPlanner to refine!
  VAR(HardwareReference);
  arr x0 =  _HardwareReference->get_q_reference(this);
  x0.append(_HardwareReference->get_v_reference(this));
  x0.subRange(x0.N/2,-1) = 0.;
  //cout <<"0-state! in motion progressor\n" <<x0 <<"\n ...frame=" <<f->currentFrame <<endl;
  mp->writeAccess(this);
  mp->frame0 = x0;
  mp->planConverged = false;
  _HardwareReference->set_motionPrimitiveRelativeTime(0., this);
  mp->deAccess(this);
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

#define VAR(Type) \
  Type *_##Type;  birosInfo().getVariable<Type>(_##Type, #Type, NULL);

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
