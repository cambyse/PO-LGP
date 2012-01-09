/*  Copyright 2009 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/> */

/** \file soc.h
    \brief Stochastic Optimal Control library */

#ifndef MT_soc_h
#define MT_soc_h

#include "array.h"
#include "util.h"

//-- fwd declarations
class OpenGL;
struct SwiftInterface;
namespace ors{ struct Graph; }
struct TaskVariable;
typedef MT::Array<TaskVariable*> TaskVariableList;
extern uint countMsg, countSetq;


namespace soc{

/// gradient method options
//enum SocSolverType{ ConjGrad=0, LevMar=1, Rprop=2, RpropConjGrad=3, SQP=4, Attractor=5 };
enum { ConjGrad=0, LevMar=1, Rprop=2, RpropConjGrad=3, SQP=4, Attractor=5 };


//===========================================================================
//
// SocSystemAbstraction
//

/** \brief defines an abstraction of stochastic optimal control
    problems which interfaces between solution methods and system simulators
    -- see section 3.2 of the <a href="../guide.pdf">guide</a> */
struct SocSystemAbstraction{

  ///@name data fields
  std::ostream *os; ///< if non-NULL, some routines might give output
  OpenGL *gl;       ///< if non-NULL, some routines might give output
  bool dynamic;     ///< determines whether this problem is dynamic or not
  uint scalePower;  ///< if non-zero, all routines assume an horizon T=T/2^scalePower
  
  uintA stepScale;    ///< the scale of each step (time interval between i-th and (i+1)-th step=2^scale)

  ///@name initialization
  SocSystemAbstraction();
  virtual ~SocSystemAbstraction();
  virtual SocSystemAbstraction *newClone() const; ///< creates a new clone of this SocAbstraction (deep copy of simulators etc)
    
  ///@name low level access routines: need to be implemented by the simulator

  // access general problem information
  virtual uint nTime() = 0;            ///< total time steps of the trajectory
  virtual uint nTasks() = 0;           ///< number of task variables
  virtual uint qDim() = 0;             ///< dimensionality of q-space
  virtual uint uDim();                 ///< dimensionality of control
  virtual uint yDim(uint i) = 0;       ///< dimensionality of the i-th task
  virtual void getq0 (arr& q) = 0;     ///< start joint configuration
  virtual void getv0 (arr& v) = 0;     ///< start joint velocity
  virtual void getqv0(arr& x);        ///< start joint configuration and velocity
  virtual void getqv0(arr& q, arr& qd); ///< start joint configuration and velocity
  virtual double getTau(bool scaled=true);    ///< time step size (for dynamic problems)
  void getx0(arr& x){ if(dynamic) getqv0(x); else getq0(x); }

  // set x-state (following calls to getPhi and getJ are w.r.t. this x)
  virtual void setq  (const arr& q, uint t=0) = 0;
  virtual void setx (const arr& x, uint t=0);
  virtual void setqv (const arr& q, const arr& qd, uint t=0);
  void setx(const arr& x){ if(dynamic) setx(x); else setq(x); }
  virtual void setq0AsCurrent() = 0;
  virtual void setToq0(){ arr q; getq0(q); setq(q); }

  //motion prior, or control cost  [t indicates the step]
  virtual void getW   (arr& W, uint t) = 0;       ///< kinematic step cost metric: cost = dq^T W dq
  virtual void getWinv(arr& Winv, uint t){ throw("NIY"); } ///< kinematic step cost metric: cost = dq^T W dq
  virtual void getH   (arr& H, uint t);           ///< dynamic control cost metric: cost = u^T H u
  virtual void getHinv(arr& H, uint t);           ///< dynamic control cost metric: cost = u^T H u
  virtual void getQ   (arr& Q, uint t);           ///< process stochasticity or integration noise Q (e.g., setDiag(1e-10, qDim()) )

  // dynamic model
  virtual void getMF(arr& M, arr& F, uint t);
  virtual void getMinvF(arr& Minv, arr& F, uint t);
  
  // task coupling
  virtual bool isConditioned(uint i, uint t) = 0;
  virtual bool isConstrained(uint i, uint t);
  virtual const char* taskName(uint i){ return NULL; };
  virtual void getPhi(arr& phiq_i, uint i){ throw("NIY"); }
  virtual void getJJt(arr& J_i, arr& Jt_i, uint i){ throw("NIY"); }
  virtual void getJqd(arr& Jqd_i, uint i);
  virtual void getHessian(arr& H_i, uint i);
  virtual void getTarget (arr& y_i, double& prec, uint i, uint t){ throw("NIY"); }
  virtual void getTargetV(arr& v_i, double& prec, uint i, uint t);
  //virtual void getLinearConstraint(arr& c, double& coff, uint i, uint t); ///< defines a cost 1 iff [[c^T y_i + coff>0]]



  ///@name high level methods: they are being accessed by the solvers

  // abstract SOC interface
  virtual void getTaskCostTerms(arr& Phi, arr& PhiJ, const arr& xt, uint t); ///< the general (`big') task vector and its Jacobian
  virtual void getTransitionCostTerms(arr& Psi, arr& PsiI, arr& PsiJ, const arr& xt_1, const arr& xt, uint t);
  virtual void getProcess(arr& A, arr& a, arr& B, uint t);
  virtual void getProcess(arr& A, arr& tA, arr& Ainv, arr& invtA, arr& a, arr& B, arr& tB, uint t);
  virtual double getTaskCosts(arr& R, arr& r, const arr& qt, uint t);
  virtual void getConstraints(arr& c, arr& coff, const arr& qt, uint t);

  // cost info
  double taskCost (arr* grad, int t, int i);
  double totalCost(arr *grad, const arr& q, bool plot=false);
  
  virtual void displayState(const arr& q, const arr *Qinv, const char *text);
  virtual void displayTrajectory(const arr& q, const arr *Qinv, int steps, const char *tag);

  //-- convenience (prelim...)
  void costChecks(const arr& x); //computes the costs in many different ways - check if they're equal - code is instructive...
  double analyzeTrajectory(const arr& q, bool plot);
  void constantTrajectory(arr& q);
  void passiveDynamicTrajectory(arr& q);
  void controlledDynamicTrajectory(arr& q, const arr& u);
  void getControlFromTrajectory(arr& u, const arr& q);
};


//===========================================================================
//
// high level SOC solver - for standard problems...
//

struct SocSolver{
  //parameters
  enum SocSolverType{ AICO=0, AICO_ms, LQG_straightInit, LQG_IKinit, LQG_ms, gradient, SQPopt };
  int method, scalePowers, iterations, gradientMethod, splinePoints, splineDegree, display, seed;
  double convergenceRate, repeatThreshold, recomputeTaskThreshold, tolerance;
  MT::String filename;
  std::ostream *os;
  arr q, b, v, Vinv;

  void init();
  void go(soc::SocSystemAbstraction &sys);
};


//===========================================================================
//
///@name     trivial helpers
// @{

void getVelocity(arr& vt, const arr& q, uint t, double tau);
void getPhaseTrajectory(arr& _q, const arr& q, double tau);
void getPositionTrajectory(arr& q, const arr& _q);
void interpolateTrajectory(arr& qNew, const arr& _q, double step);

//only for the first task so far!
void getJointFromTaskTrajectory(SocSystemAbstraction& soci, arr& q, const arr& x);
void partialJointFromTaskTrajectory(SocSystemAbstraction& soci, arr& dx, const arr& dq, const arr& q, const arr& x);
void straightTaskTrajectory(SocSystemAbstraction& soci, arr& q, uint taskid);


//===========================================================================
// @}
///@name     inverse kinematics control
// @{

void bayesianIKControl(SocSystemAbstraction& soci, arr& dq, uint t);
void pseudoIKControl(SocSystemAbstraction& soci, arr& dq, uint t, double regularization=1e-8);
void hierarchicalIKControl(SocSystemAbstraction& soci, arr& dq, uint t, double regularization=1e-8);
void bayesianIterateIKControl(SocSystemAbstraction& soci,
                              arr& qt, const arr& qt_1, uint t, double eps, uint maxIter);
void bayesianIKTrajectory  (SocSystemAbstraction& soci, arr& q, double eps=-1);
void bayesianDynamicControl(SocSystemAbstraction& soci, arr& x, const arr& x_1, uint t, arr *v=NULL, arr *Vinv=NULL);
void bayesianIKControl2    (SocSystemAbstraction& soci, arr& q , const arr& q_1 , uint t, arr *v=NULL, arr *Vinv=NULL);



//===========================================================================
// @}
///@name     gradient optimization
// @{

void gradientOptimization(SocSystemAbstraction& soci,
                                arr& q,
//                                int gradient_method,
                                uint maxIterations,
                                uint spline_points,
                                uint spline_degree,
                                double stoppingTolerance,
                                bool checkGradient,
                                uint display);

void gradientTaskOptimization(SocSystemAbstraction& soci,
                                arr& q,
                                uint spline_points,
                                uint spline_degree,
                                int gradient_method,
                                uint maxIterations,
                                double stoppingTolerance,
                                bool checkGradient,
                                uint display);

void gradientAttractorTaskOptimization(SocSystemAbstraction& soci,
                                arr& q,
                                uint spline_points,
                                uint spline_degree,
                                int gradient_method,
                                uint maxIterations,
                                double stoppingTolerance,
                                bool checkGradient,
                                uint display);

void SQPOptimization(SocSystemAbstraction& soci,
                           arr& q, uint iterations,
                           uint spline_points,
                           uint spline_degree,
                           uint display);

//===========================================================================
// @}
///@name     LQG methods
// @{

struct LQG{
  //parameters
  SocSystemAbstraction *sys;
  double convergenceRate;
  uint display;
  //messages & state info
  arr q, q_phase;                  //!< current trajectory in position and phase space
  arr vbar, Vbar, r, R, Vinv, v;       //!< bwd, and task messages
  double cost;                      //!< cost of MAP trajectory
  uint sweep;                     //!< #sweeps so far
  uint scale;                     //!< scale of this LQG in a multi-scale approach
  
  LQG(){ sweep=0; scale=0; }
  
  void init(SocSystemAbstraction& _sys,
            double _convergenceRate, uint _display,
            uint _scale);
  void initMessages();
  void shiftSolution(int offset);

  double stepGeneral();
  double stepKinematic();
};

LQG* LQG_solve(SocSystemAbstraction& sys,
                 arr& q, double tolerance,
                 double convergenceRate,
                 uint display);

LQG* LQG_multiScaleSolver(SocSystemAbstraction& sys,
                            arr& q,
                            double tolerance,
                            double convergenceRate,
                            uint display,
                            uint scalePowers);

//===========================================================================
// @}
///@name     bayesian inference methods
// @{

/** \brief Apprioximate Inference Control */
struct AICO{
  //parameters (INPUT)
  SocSystemAbstraction *sys;
  double convergenceRate, repeatThreshold, recomputeTaskThreshold, maxStep;
  double damping;
  uint display;
  bool useBwdMsg;
  arr bwdMsg_v, bwdMsg_Vinv;

  //messages (OUTPUT)
  arr s, Sinv, v, Vinv, r, R;          //!< fwd, bwd, and task messages
  MT::Array<arr> phiBar, JBar;     //!< all task cost terms
  arr Psi;                        //!< all transition cost terms
  arr b, Binv;                     //!< beliefs
  arr q, qhat;                     //!< trajectory (MAP), and point of linearization
  arr dampingReference;
  double cost;                      //!< cost of MAP trajectory

  // INTERNAL
  bool useFwdMessageAsQhat;
  arr A, tA, Ainv, invtA, a, B, tB, Winv, Hinv, Q; //!< processes...
  uint sweep;                     //!< #sweeps so far
  uint scale;                     //!< scale of this AICO in a multi-scale approach
  
  AICO(){ sweep=0; scale=0; maxStep=.1; }
  
  void init(SocSystemAbstraction& _sys,
            double _convergenceRate, double _repeatThreshold, double _recomputeTaskThreshold,
            uint _display, uint _scale);
  void initMessages();
  void shiftSolution(int offset);
  double stepDynamic  ();
  double stepKinematic();
  double stepDynamicIlqg();
  double stepGaussNewton();
  double stepMinSum();

  //internal helpers
  void initMessagesWithReferenceQ(arr& qref);
  void initMessagesFromScaleParent(AICO *parent);
  void updateFwdMessage(uint t);
  void updateBwdMessage(uint t);
};

soc::AICO* AICO_solver(SocSystemAbstraction& soci,
                       arr& q, double tolerance,
                       double convergenceRate, double repeatThreshold, double recomputeTaskThreshold,
                       uint display);

void AICO_multiScaleSolver(SocSystemAbstraction& sys,
                           arr& q,
                           double tolerance,
                           double convergenceRate, double repeatThreshold, double recomputeTaskThreshold,
                           uint display,
                           uint scalePowers);

inline void getController(arr& G, arr& g, const soc::AICO& aico){
  //we can only compute a controller for time steps 0 to T-1 (based on V_{t+1})
  uint T=aico.s.d0-1;
  uint n=aico.s.d1;
  if(!aico.sys->dynamic){
    G.resize(T, n, n);
    g.resize(T, n);
  }else{
    G.resize(T, n/2, n);
    g.resize(T, n/2);
  }
  arr H;
  for(uint t=0;t<T;t++){
    arr Vstar, barv, VstarH;
    aico.sys->getH(H, t);
    if(!aico.sys->dynamic){
      //controller model u_mean = G*x+g
      Vstar = aico.Vinv[t+1] + aico.R[t+1];
      lapack_Ainv_b_sym(barv, Vstar, aico.Vinv[t+1]*aico.v[t+1] + aico.r[t+1]);
      inverse_SymPosDef(VstarH, Vstar + H);
      G[t] = - VstarH * Vstar; // * aico.A[t];
      g[t] = VstarH * Vstar * (barv); // - aico.a[t]);
    }else{
      Vstar = aico.Vinv[t+1] + aico.R[t+1];
      lapack_Ainv_b_sym(barv, Vstar, aico.Vinv[t+1]*aico.v[t+1] + aico.r[t+1]);
      inverse_SymPosDef(VstarH, aico.tB[t]*Vstar*aico.B[t] + H);
      G[t] = - VstarH * aico.tB[t] * Vstar * aico.A[t];
      g[t] = VstarH * aico.tB[t] * Vstar * (barv - aico.a[t]);
    }
  }
}

inline void forwardSimulateTrajectory(arr& q, const arr& G, const arr& g, soc::SocSystemAbstraction& sys, const soc::AICO& aico){
  uint t, T=sys.nTime(), n=sys.qDim();
  if(!aico.sys->dynamic){
    q.resize(T+1, n);
    sys.getq0(q[0]());
    for(t=0;t<T;t++) q[t+1]() = q[t] + (G[t]*q[t] + g[t]); //A=1, B=1
  }else{
    q.resize(T+1, 2*n);
    sys.getqv0(q[0]());
    for(t=0;t<T;t++) q[t+1]() = aico.A[t]*q[t] + aico.B[t]*(G[t]*q[t] + g[t]) + aico.a[t];
    arr q_sub;
    getPositionTrajectory(q_sub, q);
    q=q_sub;
  }
}

inline void getControlledTrajectory(arr& q, const soc::AICO& aico){
  arr G, g;
  getController(G, g, aico);
  forwardSimulateTrajectory(q, G, g, *aico.sys, aico);
}

//===========================================================================
// @}
///@name preliminary or obsolete
// @{

#if 1
class SocSystem_Ors;
class SocSystem_Toy;

double getDynamicCostMeassure_obsolete(SocSystemAbstraction& soci, arr& q, double& cost1, double& cost2, std::ostream *os=0);
double getFilterCostMeassure_obsolete(SocSystemAbstraction& soci, arr& q, double& cost1, double& cost2, std::ostream *os=0);
void setupOpenGL_obsolete(SocSystem_Ors &soci);

void createDynamicProblem(SocSystem_Ors &soci,
                          const char *ors_file,
                          double trajectory_time,
                          uint trajectory_steps);

void setupOpenGL(SocSystem_Toy &soci);

void createEndeffectorReachProblem(SocSystem_Toy &soci,
                                   const char *ors_file,
                                   uint trajectory_length,
                                   int rand_seed);

/*void createNikolayReachProblem(SocSystem_Toy &soci,
                               slGraph &ors,
                               SwiftInterface& swift,
                               uint trajectory_length,
                               const arr& endeffector_target,
                               const char* endeffector_name,
                               const arr& W);*/

void createDynamicProblem(SocSystem_Toy &soci,
                          const char *ors_file,
                          double trajectory_time,
                          uint trajectory_steps);
#endif

//===========================================================================
// @}
// ORS simulator implementation of the SocAbstration
//

struct SocSystem_Ors_Workspace;

/** \brief an implementation of the SocSystemAbstraction using the \ref ors
    simulator */
struct SocSystem_Ors: public virtual SocSystemAbstraction{
  ors::Graph *ors;
  SwiftInterface *swift;
  MT::Array<TaskVariable*> vars;
  SocSystem_Ors_Workspace *s;

  SocSystem_Ors();
  virtual ~SocSystem_Ors();
  SocSystem_Ors* newClone(bool deep) const;
  
  //initialization methods
  void initBasics(ors::Graph *_ors, SwiftInterface *_swift, OpenGL *_gl,
		  uint trajectory_steps, double trajectory_time, bool _dynamic, arr *W);
  void setTimeInterval(double trajectory_time, uint trajectory_steps);
  void setTaskVariables(const TaskVariableList& CVlist);

  //--exemplary problem setups: read specifications from MT.cfg
  void initStandardReachProblem(uint rand_seed=0, uint T=0, bool _dynamic=false);
  void initStandardBenchmark(uint rand_seed=0);
  
  //info
  void reportOnState(std::ostream& os);
  void displayState(const arr& q, const arr *Q, const char *text=NULL);

  //implementations of virtual methods
  uint nTime();
  uint nTasks();
  uint qDim();
  uint uDim();
  uint yDim(uint i);
  void getq0 (arr& q);
  void getv0 (arr& v);
  void getqv0(arr& x);
  void getqv0(arr& q, arr& qd);
  bool isDynamic();
  void setq  (const arr& q, uint t=0);
  void setx (const arr& x, uint t=0);
  void setqv (const arr& q, const arr& qd, uint t=0);
  void setq0AsCurrent();
  //void geth  (arr& h);
  void getW   (arr& W, uint t);
  void getWinv(arr& Winv, uint t);
  void getH   (arr& H, uint t);
  void getHinv(arr& Hinv, uint t);
  void getQ   (arr& Q, uint t);
  bool isConditioned(uint i, uint t);
  bool isConstrained(uint i, uint t);
  const char* taskName(uint i);
  void getPhi(arr& phiq_i, uint i);
  void getJJt(arr& J_i, arr& Jt_i, uint i);
  void getHessian(arr& H_i, uint i);
  void getJqd(arr& Jqd_i, uint i);
  void getTarget (arr& y_i, double& prec, uint i, uint t);
  void getTargetV(arr& v_i, double& prec, uint i, uint t);
  //void getC   (arr& C_i, uint i, uint t);
  //void getCV  (arr& D_i, uint i, uint t);
  double getTau(bool scaled=true);
  void getMF(arr& M, arr& F, uint t);
  void getMinvF(arr& Minv, arr& F, uint t);

};

//========= untidy stuff ...



//===========================================================================
//
// toy implementation of the SocAbstration
//

struct SocSystem_Toy_Workspace;

/** \brief an implementation of the SocSystemAbstraction that simulates a
    single 1D point mass on a spring */
struct SocSystem_Toy: public virtual SocSystemAbstraction{
  SocSystem_Toy_Workspace *s;

  SocSystem_Toy();
  virtual ~SocSystem_Toy();

  //implementations of virtual methods
  uint nTime();
  uint nTasks();
  uint qDim();
  uint uDim();
  uint yDim(uint i);
  void getq0 (arr& q);
  void getv0 (arr& v){throw("NIY");}
  void getqv0(arr& x);
  void getqv0(arr& q, arr& qd);
  bool isDynamic();
  void setq  (const arr& q, uint t=0);
  void setx (const arr& x, uint t=0);
  void setqv (const arr& q, const arr& qd, uint t=0);
  void setq0AsCurrent();
  void geth  (arr& h);
  void getW  (arr& W);
  void getWinv(arr& Winv){ throw("NIY"); };
  void getH  (arr& H);
  void getQ  (arr& Q);
  bool isConditioned(uint i, uint t);
  void getPhi(arr& phiq_i, uint i);
  void getJJt(arr& J_i, arr& Jt_i, uint i);
  void getJqd(arr& Jqd_i, uint i);
  void getTarget (arr& y_i, uint i, uint t);
  void getTargetV(arr& v_i, uint i, uint t);
  void getC   (arr& C_i, uint i, uint t);
  void getCV  (arr& D_i, uint i, uint t);
  void getPrecision (double& prec, uint i, uint t);
  void getPrecisionV(double& prec, uint i, uint t);
  double getTau(bool scaled=true);
  void getMF(arr& M, arr& F);
  void getMinvF(arr& Minv, arr& F);
};

}


//===========================================================================
//
// implementations
//

#ifdef  MT_IMPLEMENTATION
#  include "soc.cpp"
#  include "soc_method_AICO.cpp"
#  include "soc_method_LQG.cpp"
#  include "soc_method_gradient.cpp"
//#  include "soc_method_attractor.cpp"
#  include "soc_system_ors.cpp"
#  include "soc_system_analytical.cpp"
#  include "soc_system_toy.cpp"
#endif

#endif
