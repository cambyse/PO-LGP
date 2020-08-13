#pragma once

#include <Optimization/decentralized_lagrangian.h>

#include <Optim/newton.h>
#include <Optim/constrained.h>

#include <functional>
#include <memory>

using CallBackType = std::function<void()>;

enum Mode
{
  SEQUENTIAL = 0,
  PARALLEL,
  FIRST_ITERATION_SEQUENTIAL_THEN_PARALLEL // doesn't seem to perform very well on komo problems
};

struct DecOptConfig
{
  DecOptConfig(const Mode& scheduling, bool compressed, OptOptions opt=NOOPT, bool checkGradients=false, CallBackType callback = CallBackType(), ostream *logFile=nullptr)
    : scheduling(scheduling)
    , compressed(compressed)
    , opt(opt)
    , checkGradients(checkGradients)
    , callback(callback)
    , logFile(logFile)
  {
  }

  Mode scheduling;

  bool compressed; // wether xs.d0 == x.d0, if true subproblem optimizers act on smaller (local) x
  OptOptions opt;  // for newton and aula

  bool checkGradients;
  CallBackType callback; // called after each step() (for debugging)
  ostream *logFile;
};

template< typename T>
struct LagrangianTypes
{
};

template<>
struct LagrangianTypes<ConstrainedProblem>
{
  typedef LagrangianProblem Lagrangian;
  typedef DecLagrangianProblem<Lagrangian> DecLagrangian;
};


template< typename T>
struct DecOptConstrained
{
  typedef typename LagrangianTypes<T>::Lagrangian LagrangianType;
  typedef typename LagrangianTypes<T>::DecLagrangian DecLagrangianType;

  arr&z_final; ///< init and solution
  const uint N; ///< number of subproblems
  std::vector<std::unique_ptr<LagrangianType>> Ls;
  std::vector<std::unique_ptr<OptNewton>> newtons;
  std::vector<std::unique_ptr<DecLagrangianType>> DLs;
  std::vector<intA> vars; ///< how local xs maps to global opt variable
  std::vector<intA> admmVars; ///< where each subproblem needs admm lagrange terms
  arr contribs; ///< amount of contribution on x (on each index) - caching (deduced directly from masks)
  uint m; ///< number of z indices where problems overlap
  arr z_prev, z; ///< internal admm reference variable (z_prev is the one of the previous step)
  std::vector<arr> xs; ///< subproblems solutions
  std::vector<arr> duals;
  bool subProblemsSolved{false}; ///< indicates whether the stopping criterion of each sub-problem is met (necessary for overall stopping condition)
  uint its{0}; ///< number of ADMM iterations

  DecOptConfig config;

public:
  DecOptConstrained(arr&_z, std::vector<std::shared_ptr<T>> & Ps, const std::vector<arr> & masks, DecOptConfig _config); //bool compressed = false, int verbose=-1, OptOptions _opt=NOOPT, ostream* _logFile=0);

  std::vector<uint> run();

private:
  void initVars(const std::vector<arr> & xmasks);
  void initXs();  // init xs based on z (typicaly called once at the start)
  void initLagrangians(const std::vector<std::shared_ptr<T>> & Ps);

  bool step(); // outer step
  bool stepSequential(); // step each subproblem and update Z in sequence
  bool stepParallel();   // step each subproblem and update Z in //

  bool step(DecLagrangianType& DL, OptNewton& newton, arr& dual, uint i) const; // inner step

  //void updateZSequential(uint i); // updating z usinmg last result of subproblem i
  void updateZ(); // update z given the xs

  void updateADMM();
  bool stoppingCriterion() const;

  double primalResidual() const;
  double dualResidual() const;

  bool primalFeasibility(double r) const;
  bool dualFeasibility(double s) const;

  void checkGradients() const;
};

#include <Optimization/decentralized_optimizer.tpp>
