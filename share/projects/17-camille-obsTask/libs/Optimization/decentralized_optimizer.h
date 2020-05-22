#pragma once

#include <decentralized_lagrangian.h>

#include <Optim/newton.h>
#include <Optim/constrained.h>

#include <memory>
#include <future>
#include <thread>

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

  DecOptConstrained(arr&x, std::vector<std::shared_ptr<ConstrainedProblem>> & Ps, const std::vector<arr> & _masks = {}, int verbose=-1, OptOptions _opt=NOOPT, ostream* _logFile=0)
    : x(x)
    , z(x.copy())
    , contribs(zeros(x.d0))
    , opt(_opt)
    , logFile(_logFile)
  {
    Ls.reserve(Ps.size());
    newtons.reserve(Ps.size());
    xs.reserve(Ps.size());
    masks.reserve(Ps.size());

    // maybe preferable to have the same pace for ADMM and AULA terms -> breaks convergence is set to 2.0, strange!
    opt.aulaMuInc = 1.2;

    /// TO BE EQUIVALENT TO PYTHON
    //opt.damping = 0.1;
    //opt.maxStep = 10.0;
    ///
    for(auto i = 0; i < Ps.size(); ++i)
    {
      masks.push_back( (i < _masks.size() && _masks[i].d0 == x.d0 ? _masks[i] : ones(x.d0) ));
      contribs += masks.back();
    }

    for(auto i = 0; i < Ps.size(); ++i)
    {
      auto& P = Ps[i];
      auto& mask = masks[i];

      xs.push_back(z.copy());
      duals.push_back(arr());

      arr& x = xs.back();
      arr& _dual = duals.back();

      Ls.push_back(std::unique_ptr<LagrangianProblem>(new LagrangianProblem(*P, opt, _dual)));
      LagrangianProblem& L = *Ls.back();
      DLs.push_back(std::unique_ptr<DecLagrangianProblem>(new DecLagrangianProblem(L, z, mask, opt)));
      DecLagrangianProblem& DL = *DLs.back();
      newtons.push_back(std::unique_ptr<OptNewton>(new OptNewton(x, DL, opt, 0)));
    }
  }

  double updateZ()
  {
    /// Z
    z = zeros(xs.front().d0);
    for(auto i = 0; i < xs.size(); ++i)
    {
      const auto& x = xs[i];
      const auto& lambda = DLs[i]->lambda;
      const auto& mask = masks[i];

      arr zinc = x;
      if(DLs[i]->mu > 0.0) // add term based on admm lagrange term always except in the first step
        zinc += lambda / DLs[i]->mu;

      // apply mask
      for(uint i=0;i<zinc.N;i++) zinc.elem(i) *= mask.elem(i);

      // add increment
      z += zinc;
      // sanity check // lagrange admm sum = 0 after one iteration
    }

    // correct for correct average
    for(uint i=0;i<z.d0;i++) z.elem(i) /= contribs.elem(i);

    ///Primal residual
    double r = 0;

    for(auto i = 0; i < xs.size(); ++i)
    {
      arr es = fabs(xs[i] - z);
      const auto& mask = masks[i];
      for(uint i=0;i<es.N;i++) es.elem(i) *= mask.elem(i);

      r += length(es);
    }
    r /= xs.size();

    return r;
  }

  bool primalFeasibility(double r) const
  {
    const double eps = 1e-3 * sqrt(x.d0) + 1e-3 * max(fabs(z));
    return r < eps;
  }

  bool dualFeasibility(double s) const
  {
    double ymax = 0;
    for(const auto& dl: DLs)
    {
      ymax = std::max(ymax, max(fabs(dl->lambda)));
    }
    const double eps = 1e-3 * sqrt(x.d0) + 1e-3 * ymax;
    return s < eps;
  }

  bool step()
  {
    std::vector<std::future<bool>> futures;
    for(auto i = 0; i < DLs.size(); ++i)
    {
      DecLagrangianProblem& DL = *DLs[i];
      OptNewton& newton = *newtons[i];
      arr& dual = duals[i];

      // spawn // threads
      DL.z = z;
      futures.push_back(std::async(std::launch::async,
      [&, i]{
        return step(DL, newton, dual, i);
      }
      ));
    }

    // synchro
    bool subProblemsSolved = true;
    for(auto i = 0; i < DLs.size(); ++i)
    {
      subProblemsSolved = futures[i].get() && subProblemsSolved;
    }

    // stop criterion
    auto z_old = z;
    double r = updateZ(); // primal residual
    double s = DLs.front()->mu * length(z - z_old); // dual residual

    // update
    for(auto i = 0; i < DLs.size(); ++i)
    {
      OptNewton& newton = *newtons[i];
      DLs[i]->updateADMM(xs[i], z);
      // update newton cache / state (necessary because we updated the underlying problem!)
      newton.fx = DLs[i]->decLagrangian(newton.gx, newton.Hx, xs[i]); // this is important!
    }

    if(opt.verbose>0) {
      cout <<"** DecOptConstr.[x] ADMM UPDATE";
      cout <<"\t |x-z|=" << r;
      if(z.d0 < 10) cout << '\t' << "z=" << z;
      cout <<endl<<endl;
    }

    its++;

    // nominal exit condition
    if(subProblemsSolved && primalFeasibility(r) && dualFeasibility(s))
    {
      if(opt.verbose>0) cout <<"** ADMM StoppingCriterion Primal Dual convergence" << std::endl;
      return true;
    }
    // other exit conditions
    if(subProblemsSolved && absMax(z_old-z) < opt.stopTolerance) { // stalled ADMM
      if(opt.verbose>0) cout <<"** ADMM StoppingCriterion Delta<" <<opt.stopTolerance <<endl;
      if(opt.stopGTolerance<0. || r < opt.stopGTolerance)
        return true;
    }

    return false;
  }

  bool step(DecLagrangianProblem& DL, OptNewton& newton, arr& dual, uint i) const
  {
    auto& L = DL.L;

    if(opt.verbose>0) {
      cout <<"** DecOptConstr.[" << i << "] it=" <<its
           <<" mu=" <<L.mu <<" nu=" <<L.nu <<" muLB=" <<L.muLB;
      if(newton.x.N<5) cout <<" \tlambda=" <<L.lambda;
      cout <<endl;
    }

    arr x_old = newton.x;

    //check for no constraints
    bool newtonOnce=false;
    if(L.get_dimOfType(OT_ineq)==0 && L.get_dimOfType(OT_eq)==0) {
      if(opt.verbose>0) cout <<"** [" << i << "] optConstr. NO CONSTRAINTS -> run Newton once and stop" <<endl;
      newtonOnce=true;
    }

    //run newton on the Lagrangian problem
    if(newtonOnce || opt.constrainedMethod==squaredPenaltyFixed) {
      newton.run();
    } else {
      double stopTol = newton.o.stopTolerance;
      if(opt.constrainedMethod==anyTimeAula)  newton.run(20);
      else                                    newton.run();
      newton.o.stopTolerance = stopTol;
    }

    if(opt.verbose>0) {
      cout <<"** DecOptConstr.[" << i << "] it=" <<its
           <<" evals=" <<newton.evals
           <<" f(x)=" <<L.get_costs()
           <<" \tg_compl=" <<L.get_sumOfGviolations()
           <<" \th_compl=" <<L.get_sumOfHviolations()
           <<" \t|x-x'|=" <<absMax(x_old-newton.x);
      if(newton.x.N<5) cout <<" \tx=" <<newton.x;
      cout <<endl;
    }

    //check for squaredPenaltyFixed method
    if(opt.constrainedMethod==squaredPenaltyFixed) {
      if(opt.verbose>0) cout <<"** optConstr.[" << i << "] squaredPenaltyFixed stops after one outer iteration" <<endl;
      return true;
    }

    //check for newtonOnce
    if(newtonOnce) {
      return true;
    }

    //stopping criterons
    if(its>=2 && absMax(x_old-newton.x) < opt.stopTolerance) {
      if(opt.verbose>0) cout <<"** optConstr.[" << i << "] StoppingCriterion Delta<" <<opt.stopTolerance <<endl;
      if(opt.stopGTolerance<0.
         || L.get_sumOfGviolations() + L.get_sumOfHviolations() < opt.stopGTolerance)
        return true;
    }

    if(newton.evals>=opt.stopEvals) {
      if(opt.verbose>0) cout <<"** optConstr.[" << i << "] StoppingCriterion MAX EVALS" <<endl;
      return true;
    }
    if(newton.its>=opt.stopIters) {
      if(opt.verbose>0) cout <<"** optConstr.[" << i << "] StoppingCriterion MAX ITERS" <<endl;
      return true;
    }
    if(its>=opt.stopOuters) {
      if(opt.verbose>0) cout <<"** optConstr.[" << i << "] StoppingCriterion MAX OUTERS" <<endl;
      return true;
    }

    double L_x_before = newton.fx;

    if(opt.verbose>0) {
      cout <<"** DecOptConstr.[" << i << "] AULA UPDATE";
      cout <<endl;
    }

    //upate Lagrange parameters
    switch(opt.constrainedMethod) {
  //  case squaredPenalty: UCP.mu *= opt.aulaMuInc;  break;
      case squaredPenalty: L.aulaUpdate(false, -1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
      case augmentedLag:   L.aulaUpdate(false, 1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
      case anyTimeAula:    L.aulaUpdate(true,  1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
      case logBarrier:     L.muLB /= 2.;  break;
      case squaredPenaltyFixed: HALT("you should not be here"); break;
      case noMethod: HALT("need to set method before");  break;
    }

    if(!!dual) dual=L.lambda;

    if(logFile){
      (*logFile) <<"{ optConstraint: " <<its <<", mu: " <<L.mu <<", nu: " <<L.nu <<", L_x_beforeUpdate: " <<L_x_before <<", L_x_afterUpdate: " <<newton.fx <<", errors: ["<<L.get_costs() <<", " <<L.get_sumOfGviolations() <<", " <<L.get_sumOfHviolations() <<"], lambda: " <<L.lambda <<" }," <<endl;
    }

    return false;
  }

  std::vector<uint> run()
  {
    // loop
    while(!step());

    // last step
    for(auto& newton: newtons)
      newton->beta *= 1e-3;

    step();

    // get solution
    x = z;

    // number of evaluations
    std::vector<uint> evals;
    evals.reserve(Ls.size());
    for(const auto& newton: newtons)
    {
      evals.push_back(newton->evals);
    }
    return evals;
  }
};
