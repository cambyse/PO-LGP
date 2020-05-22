#pragma once

#include <decentralized_lagrangian.h>

#include <Optim/newton.h>
#include <Optim/constrained.h>

#include <memory>

struct DecOptConstrained
{
  std::vector<std::unique_ptr<LagrangianProblem>> Ls;
  std::vector<std::unique_ptr<OptNewton>> newtons;
  std::vector<std::unique_ptr<DecLagrangianProblem>> DLs;
  std::vector<arr> masks;
  arr contribs; // number of contribution per portion of subproblem
  arr&x; ///< last opt result
  arr z;
  std::vector<arr> xs;
  std::vector<arr> duals;
  OptOptions opt;
  uint its=0;
  ostream *logFile=NULL;

  DecOptConstrained(arr&x, std::vector<std::shared_ptr<ConstrainedProblem>> & Ps, const std::vector<arr> & _masks = {}, int verbose=-1, OptOptions _opt=NOOPT, ostream* _logFile=0);

  double updateZ();
  bool step(); // outer step
  bool step(DecLagrangianProblem& DL, OptNewton& newton, arr& dual, uint i) const; // inner step
  std::vector<uint> run();

  bool primalFeasibility(double r) const;
  bool dualFeasibility(double s) const;
};
