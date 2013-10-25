// Read DOCSTRING to get an idea of motionpy!
%define DOCSTRING_MOTIONPY
"
This is a simple SWIG wrapper to be able to use the motion framework
within python.


author: Johannes kulick
"
%enddef
%module(docstring=DOCSTRING_MOTIONPY) motionpy

%feature("autodoc", "1");

%include "Core/array.i"
%import "Core/core.i"
%import "Ors/ors.i"

//===========================================================================
%module motionpy
%{
  #include "motion.h"
  #include <Optim/optimization.h>
  #include <sstream>
%}


//===========================================================================
// we need to map uint. Otherwise python complains about the uint type
%inline %{
  typedef unsigned int uint;
%}

//===========================================================================

enum DefaultTaskMapType {
  noneTMT,     ///< undefined
  posTMT,      ///< 3D position of reference, can have 2nd reference, no param
  zoriTMT,     ///< 3D z-axis orientation, no 2nd reference, no param
  zalignTMT,   ///< 1D z-axis alignment, can have 2nd reference, param (optional) determins alternative reference world vector
  qItselfTMT,  ///< q itself as task variable, no param
  qLinearTMT,  ///< k-dim variable linear in q, no references, param: k-times-n matrix
  qSingleTMT,  ///< 1D entry of q, reference-integer=index, no param
  qSquaredTMT, ///< 1D square norm of q, no references, param: n-times-n matrix
  qLimitsTMT,  ///< 1D meassure for joint limit violation, no references, param: n-times-2 matrix with lower and upper limits for each joint
  collTMT,     ///< 1D meassure for collision violation, no references, param: 1D number defining the distance margin
  colConTMT,   ///< 1D meassure collision CONSTRAINT meassure, no references, param: 1D number defining the distance margin
  comTMT,      ///< 2D vector of the horizontal center of mass, no refs, no param
  skinTMT      ///< vector of skin pressures...
};


//===========================================================================
//
// defines only a map (task space), not yet the costs in this space
//

struct TaskMap {
  bool constraint;
  virtual void phi(arr& y, arr& J, const ors::Graph& G) = 0;
  virtual uint dim_phi(const ors::Graph& G) = 0; //the dimensionality of $y$

  TaskMap():constraint(false) {}
};


//===========================================================================
//
// the costs in a task space
//

struct TaskCost {
  TaskMap& map;
  MT::String name;
  bool active;
  arr y_target, y_prec;  ///< target & precision over a whole trajectory
  arr v_target, v_prec;  ///< velocity target & precision over a whole trajectory
  
  TaskCost(TaskMap* m):map(*m), active(true) {}
};


//===========================================================================
//
// a motion problem description
//

/// This class allows you to DESCRIBE a motion problem, nothing more
struct MotionProblem {
  //engines to compute things
  ors::Graph *ors;
  SwiftInterface *swift;
  
  //task cost descriptions
  enum TaskCostInterpolationType { constFinalMid, constEarlyMid, linearInterpolation };
  MT::Array<TaskCost*> taskCosts;
  
  //transition cost descriptions
  enum TransitionType { kinematic=0, pseudoDynamic=1, realDynamic=2 };
  TransitionType transitionType;
  arr H_rate_diag; ///< cost rate
  uint T; ///< number of time steps
  double tau; ///< duration of single step
  
  //start constraints
  arr x0,v0; ///< fixed start state and velocity
  arr x_current, v_current; ///< memory for which state was set (which state ors is in)
  
  //evaluation outcomes
  arr costMatrix;
  
  MotionProblem(ors::Graph *_ors=NULL, SwiftInterface *_swift=NULL);
  
  void loadTransitionParameters(); ///< loads transition parameters from cfgFile
  
  //-- methods for defining the task
  void setx0(const arr&);
  void setx0v0(const arr&, const arr&);
  
  //adding add task spaces
  TaskCost* addDefaultTaskMap(const char* name, DefaultTaskMapType type,
                              int iBody=-1, const ors::Transformation& irel=NoTransformation,
                              int jBody=-1, const ors::Transformation& jrel=NoTransformation,
                              const arr& params=NoArr);
                              
  TaskCost* addDefaultTaskMap_Bodies(const char* name, DefaultTaskMapType type,
                                     const char *iBodyName=NULL, const ors::Transformation& irel=NoTransformation,
                                     const char *jBodyName=NULL, const ors::Transformation& jrel=NoTransformation,
                                     const arr& params=NoArr);
                              
  TaskCost* addDefaultTaskMap_Shapes(const char* name, DefaultTaskMapType type,
                                     const char *iShapeName=NULL, const ors::Transformation& irel=NoTransformation,
                                     const char *jShapeName=NULL, const ors::Transformation& jrel=NoTransformation,
                                     const arr& param=NoArr);

  TaskCost* addCustomTaskMap(const char* name, TaskMap *map);

  //setting costs in a task space
  void setInterpolatingCosts(TaskCost *c,
                             TaskCostInterpolationType inType,
                             const arr& y_finalTarget, double y_finalPrec, const arr& y_midTarget=NoArr, double y_midPrec=-1., double earlyFraction=-1.);
                             
  void setInterpolatingVelCosts(TaskCost *c,
                                TaskCostInterpolationType inType,
                                const arr& v_finalTarget, double v_finalPrec, const arr& v_midTarget, double v_midPrec);
                                
  //-- cost infos
  uint dim_phi(uint t);
  uint dim_g(uint t);
  uint dim_psi();
  void getTaskCosts(arr& phi, arr& J_x, arr& J_v, uint t); // the general (big) task vector and its Jacobian
  void costReport();
  
  void setState(const arr& x, const arr& v);
  void activateAllTaskCosts(bool activate=true);
};


//===========================================================================
//
// transforming a motion problem description into an optimization problem
//

struct MotionProblemFunction:KOrderMarkovFunction {
  MotionProblem& P;
  bool makeConstrainedProblem;

  MotionProblemFunction(MotionProblem& _P):P(_P), makeConstrainedProblem(false) {};
  
  //KOrderMarkovFunction definitions
  virtual void phi_t(arr& phi, arr& J, uint t, const arr& x_bar);
  //functions to get the parameters $T$, $k$ and $n$ of the $k$-order Markov Process
  virtual uint get_T() { return P.T; }
  virtual uint get_k() { if(P.transitionType==MotionProblem::kinematic) return 1;  return 2; }
  virtual uint dim_x() { return P.x0.N; }
  virtual uint dim_phi(uint t){ return dim_x() + P.dim_phi(t); }
  virtual uint dim_g(uint t){ return P.dim_g(t); }
  virtual arr get_prefix(); //the history states x(-k),..,x(-1)
};

//===========================================================================
