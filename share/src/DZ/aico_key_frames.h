#include <MT/soc.h>

/** Returns mean and variance of end-position without dynamic constraints   */
double OneStepKinematic(arr& b,arr& Binv, soc::SocSystemAbstraction& sys,uint T,double alpha);

/** Returns mean and variance of end-position with dynamic constraints   */
void OneStepDynamic(arr& b,arr& Binv, soc::SocSystemAbstraction& sys,uint T,double alpha);

/** Returns mean and variance of end-position without dynamic constraints 
and minimal time duration needed for the execution.  */
void OneStepKinematicT(arr& b,arr& Binv,double& duration, soc::SocSystemAbstraction& sys,uint T,double alpha);

// Full dynamic versions
/** Returns mean and variance of end-position belief with dynamic constraints  */
double OneStepDynamicFull(arr& b,arr& Binv, soc::SocSystemAbstraction& sys,double time,double alpha, double task_eps, uint verbose, bool b_is_initialized=false);

/** Returns derivative of likelihood for being in state B with dynamic constraints and costs R within time T  */
void OneStepDynamicGradientFull(double& grad,double& likelihood,soc::SocSystemAbstraction& sys,arr& R,arr& r,double time);


/** Returns optimal Time */
void GetOptimalDynamicTime(double& time,arr& b,arr& Binv, soc::SocSystemAbstraction& sys,double alpha,double step,double min_step, bool verbose);
