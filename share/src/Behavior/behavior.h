#pragma once

#include <Core/module.h>

struct ControlRefs{
  arr u0;                    ///< bias
  arr Kp, q_ref;             ///< P-term $K_p(q^*-q)$
  arr Kd, qd_ref;            ///< D-term $K_p(q^*-q)$
  double ki, gamma_p, cap_p; ///< I-term $k_i K_p e, \dot e=q^*-q -\g e, e\le c$
  arr f_ref, Jfinv;          ///< f-Term:
  double kf, gamma_f, cap_f; ///< $k_f e, \dot e=[f^* - J_f^\dag f]_{-\sgn(f^*)} - \g e, e\le c$
};

struct ControlState{
  arr q, qd, u, f;
};

struct Robot{
  //-- access
  Access_typed<ControlRefs> controlRefs = Access_typed<ControlRefs>("controlRefs");
  Access_typed<ControlState> controlState = Access_typed<ControlState>("controlState");

  //-- dynamic info
  void getState(arr& q, arr& qd, arr& u, arr& f);

  //-- simple control modes
  void setPositionControl(double kp=1., double kd=1.);
  void setPositionRef(const arr& q_ref);

  //--

  void disablePosControl();
  void enablePosControl();

  void enableTotalTorqueMode();
  void disableTotalTorqueMode();
  void publishTorque(const arr& u, const char* prefix="right_");

  
  uint reportPerceptionObjects();
  double setTestJointState(const arr& q);
  void getEquationOfMotion(arr& M, arr& F);


  //-- get position closest cluster
  ors::Vector closestCluster();
  ors::Vector arPose();

  void disablePosControl();
  void enablePosControl();

  void enableTotalTorqueMode();
  void disableTotalTorqueMode();
  void publishTorque(const arr& u, const char* prefix="right_");

  const ors::KinematicWorld& getKinematicWorld();

  double getCollisionScalar();

  arr getHoming();
  arr getLimits();

};

struct Handle;
struct ControlHandle;

struct Behavior{

  //-- generic
  Handle* start(const Graph& specs);
  Handle* modify(Handle* t, const Graph& specs);
  void stop(const HandleL& activities);
  void testCondition(const Handle* act, const char* condition, double conditionValue);
  void waitForCondition(const Handle* act, const char* condition, double conditionValue);

  //-- low-level motion contol
  ControlHandle* control(const Graph& specs);
  ControlHandle* modifyTarget(ControlHandle* t, const arr& target);
  arr getState(ControlHandle* t);
  void waitForConditionConv(const ControlHandleL& tasks);


  //-- low-level motion contol




};

