#pragma once

#include <Optim/newton.h>
#include <Optim/constrained.h>

#include <memory>
#include <future>
#include <thread>

struct DecLagrangianProblem : ScalarFunction {
  LagrangianProblem& L;

  //-- parameters of the ADMM
  double mu;         ///< ADMM square penalty
  arr lambda;        ///< ADMM lagrange multiplier
  arr z;             ///< external ADMM reference
  const arr& xmask;  ///< where on x does this subproblem contribute

  ostream *logFile=NULL;  ///< file for logging

  DecLagrangianProblem(LagrangianProblem&L, const arr & z, const arr & xmask, OptOptions opt=NOOPT)
    : L(L)
    , mu(0.0) // first step done with 0 (to avoid fitting to a unset reference)
    , lambda(zeros(z.d0))
    , z(z)
    , xmask(xmask)
  {
    ScalarFunction::operator=([this](arr& dL, arr& HL, const arr& x) -> double {
      return this->decLagrangian(dL, HL, x);
    });
  }

  double decLagrangian(arr& dL, arr& HL, const arr& x) {
    double l = L(dL, HL, x);

    arr delta = x - z;
    for(uint i=0;i<delta.N;i++) delta.elem(i) *= xmask.elem(i);

    // value
    l += scalarProduct(lambda, delta) + 0.5 * mu * scalarProduct(delta, delta);

    // jacobian
    dL += lambda + mu * delta;

    // hessian
    if(isSparseMatrix(HL))
    {
      auto Hs = dynamic_cast<rai::SparseMatrix*>(HL.special);
      for(auto i = 0; i < x.d0; ++i)
      {
        if(xmask(i)) Hs->elem(i, i) += mu;
      }
    }
    else
    {
      for(auto i = 0; i < x.d0; ++i)
      {
        if(xmask(i)) HL(i, i) += mu;
      }
    }

    return l;
  }

  void updateADMM(const arr& x, const arr& z)
  {
      this->z = z;
      lambda += mu * (x - z);

      if(mu==0.0) mu=1.0;
      //else mu *= 2.0;
  }
};

struct DecOptConstrained
{
  std::vector<std::unique_ptr<LagrangianProblem>> Ls;
  std::vector<std::unique_ptr<OptNewton>> newtons;
  std::vector<std::unique_ptr<DecLagrangianProblem>> DLs;
  std::vector<arr> masks;
  arr&x; ///< last opt result
  arr z;
  std::vector<arr> xs;
  std::vector<arr> duals;
  OptOptions opt;
  const double stopTol;
  uint its=0;
  ostream *logFile=NULL;

  DecOptConstrained(arr&x, arr & dual, std::vector<std::shared_ptr<ConstrainedProblem>> & Ps, const std::vector<arr> & _masks = {}, int verbose=-1, OptOptions _opt=NOOPT, ostream* _logFile=0)
    : x(x)
    , z(x.copy())
    , opt(_opt)
    , stopTol(0.01)
    , logFile(_logFile)
  {
    Ls.reserve(Ps.size());
    duals.reserve(Ps.size());
    newtons.reserve(Ps.size());
    xs.reserve(Ps.size());
    masks.reserve(Ps.size());

    // maybe preferable to have the same pace for ADMM and AULA terms
    opt.aulaMuInc = 2.0;

    /// TO BE EQUIVALENT TO PYTHON
    //opt.damping = 0.1;
    //opt.maxStep = 10.0;
    ///

    for(auto i = 0; i < Ps.size(); ++i)
    {
      auto& P = Ps[i];

      xs.push_back(z.copy());
      duals.push_back(dual);

      arr& x = xs.back();
      arr& _dual = duals.back();

      masks.push_back( (i < _masks.size() && _masks[i].d0 == x.d0 ? _masks[i] : ones(x.d0) ));
      arr& mask = masks.back();
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
    arr contribs = zeros(xs.front().d0);
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
      contribs += mask;
    }

    // correct for correct average
    for(uint i=0;i<z.d0;i++) z.elem(i) /= contribs.elem(i);

    ///E
    double e = 0;

    for(auto i = 0; i < xs.size(); ++i)
    {
      arr es = fabs(xs[i] - z);
      uint m = 0; double me = 0;
      for(auto j = 0; j < es.d0; ++j)
      {
        if(fabs(es(j) - z(j)) > me)
        {
          me = fabs(es(j) - z(j));
          m = j;
        }
      }

      const auto& mask = masks[i];
      for(uint i=0;i<es.N;i++) es.elem(i) *= mask.elem(i);

      e = std::max(e, max(fabs(es)));
    }

    return e;
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
    bool stop = true;
    for(auto i = 0; i < DLs.size(); ++i)
    {
      stop = futures[i].get() && stop;
    }

    // stop criterion
    auto z_old = z;
    double e = updateZ();

    // update
    for(auto i = 0; i < DLs.size(); ++i)
    {
      OptNewton& newton = *newtons[i];
      DLs[i]->updateADMM(xs[i], z);
      // update newton cache / state (necessary because we updated the underlying problem!)
      newton.fx = DLs[i]->decLagrangian(newton.gx, newton.Hx, xs[i]); // this is important!
    }

    stop = stop && (e < 0.01);// || its > 5);

    if(opt.verbose>0) {
      cout <<"** DecOptConstr.[x] ADMM UPDATE";
      cout <<"\t |x-z|=" << e;
      if(z.d0 < 10) cout << '\t' << "z=" << z;
      cout <<endl<<endl;
    }

    if(its>=2 && absMax(z_old-z) < opt.stopTolerance) {
      if(opt.verbose>0) cout <<"** ADMM] StoppingCriterion Delta<" <<opt.stopTolerance <<endl;
      if(opt.stopGTolerance<0.
         || e < opt.stopGTolerance)
        return true;
    }

    its++;

    return stop;
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
