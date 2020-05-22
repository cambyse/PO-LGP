#pragma once

#include <decentralized_lagrangian.h>

#include <Optim/newton.h>
#include <Optim/constrained.h>

#include <memory>

struct DecOptConstrained
{
  arr&z_final; ///< init and solution
  const uint N; ///< number of subproblems
  std::vector<std::unique_ptr<LagrangianProblem>> Ls;
  std::vector<std::unique_ptr<OptNewton>> newtons;
  std::vector<std::unique_ptr<DecLagrangianProblem>> DLs;
  std::vector<intA> vars; ///< how local xs maps to global opt variable
  arr contribs; ///< amount of contribution on x (on each index) - caching (deduced directly from masks)
  arr z; ///< internal admm reference variable
  std::vector<arr> xs; ///< subproblems solutions
  std::vector<arr> duals;

  bool compressed; // wether xs.d0 == x.d0, if true subproblem optimizers act on smaller (local) x
  OptOptions opt;

  uint its=0;
  ostream *logFile=NULL;

public:
  DecOptConstrained(arr&_z, std::vector<std::shared_ptr<ConstrainedProblem>> & Ps, const std::vector<arr> & masks = {}, bool compressed = false, int verbose=-1, OptOptions _opt=NOOPT, ostream* _logFile=0);

  std::vector<uint> run();

private:
  void initVars(const std::vector<arr> & xmasks);
  void initXs();  // init xs based on z (typicaly called once at the start)
  void initLagrangians(const std::vector<std::shared_ptr<ConstrainedProblem>> & Ps);

  bool step(); // outer step
  bool step(DecLagrangianProblem& DL, OptNewton& newton, arr& dual, uint i) const; // inner step

  void updateZ(); // update z given the xs

  double primalResidual() const;
  bool primalFeasibility(double r) const;
  bool dualFeasibility(double s) const;
};
