#ifndef MT_soc_inverseKinematics_h
#define MT_soc_inverseKinematics_h

#include "soc.h"

namespace soc{
  
//===========================================================================
// @}
///@name     inverse kinematics control
// @{

void bayesianIKControl(SocSystemAbstraction& soci, arr& dq, uint t);
void pseudoIKControl(SocSystemAbstraction& soci, arr& dq, uint t,double regularization=1e-8);
void hierarchicalIKControl(SocSystemAbstraction& soci, arr& dq, uint t,double regularization=1e-8);
void bayesianIterateIKControl(SocSystemAbstraction& soci,
                              arr& qt,const arr& qt_1,uint t,double eps,uint maxIter);
void bayesianIKTrajectory  (SocSystemAbstraction& soci, arr& q, double eps=-1);
void bayesianDynamicControl(SocSystemAbstraction& soci, arr& qv, const arr& qv_1, uint t, arr *v=NULL, arr *Vinv=NULL);
void bayesianIKControl2    (SocSystemAbstraction& soci, arr& q , const arr& q_1 , uint t, arr *v=NULL, arr *Vinv=NULL);

}

#endif