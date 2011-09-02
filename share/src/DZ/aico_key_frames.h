#include <MT/soc.h>

/** Returns mean and variance of end-position without dynamic constraints   */
void OneStepKinematic(arr& b,arr& Binv, soc::SocSystemAbstraction& sys,uint T,double alpha);

/** Returns mean and variance of end-position with dynamic constraints   */
void OneStepDynamic(arr& b,arr& Binv, soc::SocSystemAbstraction& sys,uint T,double alpha);

/** Returns mean and variance of end-position without dynamic constraints 
and minimal time duration needed for the execution.  */
void OneStepKinematicT(arr& b,arr& Binv,double& duration, soc::SocSystemAbstraction& sys,uint T,double alpha);

// Full dynamic versions
/** Returns mean and variance of end-position belief with dynamic constraints  */
void OneStepDynamicFull(arr& b,arr& Binv, soc::SocSystemAbstraction& sys,double time,double alpha, bool verbose);

/** Returns derivative of likelihood for being in state B with dynamic constraints and costs R within time T  */
void OneStepDynamicGradientFull(double& grad,soc::SocSystemAbstraction& sys,arr& R,arr& r,double time);


/** Returns optimal Time */
void GetOptimalDynamicTime(double& time,arr& b,arr& Binv, soc::SocSystemAbstraction& sys,double alpha,double step);
