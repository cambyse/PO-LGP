#include <MT/soc.h>

/** Returns mean and variance of end-position without dynamic constraints   */
double OneStepKinematic(arr& b, arr& Binv, uint& counter, soc::SocSystemAbstraction& sys, double stopTolerance, bool q_is_initialized);

// Full dynamic versions
/** Returns mean and variance of end-position belief with dynamic constraints  */
double OneStepDynamicFull(arr& b,arr& Binv, uint& counter,
                          soc::SocSystemAbstraction& sys,
                          double time,double alpha,double task_eps,double eps_alpha,
                          uint verbose, bool b_is_initialized);

/** Returns gradient and likelihood for being in state B with dynamic constraints and costs R within time T  */
void OneStepDynamicGradientFull(double& grad,double& likelihood,soc::SocSystemAbstraction& sys,arr& R,arr& r,double time);

/** Returns optimal Time and posterior posture */
void GetOptimalDynamicTime(double& time, uint& counter,
                           arr& b,arr& Binv,soc::SocSystemAbstraction& sys,
                           double alpha, double task_eps, double eps_alpha, double step,
                           double min_step, uint verbose);
