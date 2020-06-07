#pragma once

#include <decentralized_lagrangian.h>

#include <Optim/newton.h>
#include <Optim/constrained.h>

#include <memory>

enum Mode
{
  SEQUENTIAL = 0,
  PARALLEL,
  FIRST_ITERATION_SEQUENTIAL_THEN_PARALLEL // doesn't seem to perform very well on komo problems
};

double sparsity(arr & H);

struct DecOptConfig
{
  DecOptConfig(const Mode& scheduling, bool compressed, OptOptions opt=NOOPT, ostream *logFile=nullptr, int verbose=-1)
    : scheduling(scheduling)
    , compressed(compressed)
    , opt(opt)
    , logFile(logFile)
    , verbose(verbose)
  {
  }

  Mode scheduling;

  bool compressed; // wether xs.d0 == x.d0, if true subproblem optimizers act on smaller (local) x
  OptOptions opt;  // for newton and aula
  ostream *logFile;

  int verbose;
};

struct DecOptConstrained
{
  arr&z_final; ///< init and solution
  const uint N; ///< number of subproblems
  std::vector<std::unique_ptr<LagrangianProblem>> Ls;
  std::vector<std::unique_ptr<OptNewton>> newtons;
  std::vector<std::unique_ptr<DecLagrangianProblem>> DLs;
  std::vector<intA> vars; ///< how local xs maps to global opt variable
  arr contribs; ///< amount of contribution on x (on each index) - caching (deduced directly from masks)
  uint m; ///< number of z indices where problems overlap
  arr z_prev, z; ///< internal admm reference variable (z_prev is the one of the previous step)
  std::vector<arr> xs; ///< subproblems solutions
  std::vector<arr> duals;
  bool subProblemsSolved{false}; ///< indicates whether the stopping criterion of each sub-problem is met (necessary for overall stopping condition)
  uint its{0}; ///< number of ADMM iterations

  DecOptConfig config;

public:
  DecOptConstrained(arr&_z, std::vector<std::shared_ptr<ConstrainedProblem>> & Ps, const std::vector<arr> & masks, DecOptConfig _config); //bool compressed = false, int verbose=-1, OptOptions _opt=NOOPT, ostream* _logFile=0);

  std::vector<uint> run();

private:
  void initVars(const std::vector<arr> & xmasks);
  void initXs();  // init xs based on z (typicaly called once at the start)
  void initLagrangians(const std::vector<std::shared_ptr<ConstrainedProblem>> & Ps);

  bool step(); // outer step
  bool stepSequential(); // step each subproblem and update Z in sequence
  bool stepParallel();   // step each subproblem and update Z in //

  bool step(DecLagrangianProblem& DL, OptNewton& newton, arr& dual, uint i) const; // inner step

  //void updateZSequential(uint i); // updating z usinmg last result of subproblem i
  void updateZ(); // update z given the xs

  void updateADMM();
  bool stoppingCriterion() const;

  double primalResidual() const;
  bool primalFeasibility(double r) const;
  bool dualFeasibility(double s) const;
};
