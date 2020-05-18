#include <Optim/newton.h>
#include <Optim/constrained.h>
#include <gtest/gtest.h>
#include "functions.cpp"
#include <memory>

constexpr double eps = 0.0001;
constexpr double eps_s = 0.01;

struct DecLagrangianProblem : ScalarFunction {
  LagrangianProblem& L;

  //-- parameters of the ADMM
  double mu;         ///< square penalty of ADMM term
  arr lambda;        ///< lagrange multiplier of ADMM term
  arr z;             ///< external ADMM reference

  const double admmMuInc;
  const arr Hb;            ///< constant hessian of barrier

  ostream *logFile=NULL;  ///< file for logging

  DecLagrangianProblem(LagrangianProblem&L, const arr & z, OptOptions opt=NOOPT)
    : L(L)
    , mu(1.0)
    , lambda(zeros(z.d0))
    , z(z)
    , admmMuInc(1.1)
    , Hb(eye(z.d0, z.d0))
  {
    ScalarFunction::operator=([this](arr& dL, arr& HL, const arr& x) -> double {
      return this->decLagrangian(dL, HL, x);
    });
  }

  double decLagrangian(arr& dL, arr& HL, const arr& x) {
    double l = L(dL, HL, x);

    arr delta = x - z;

    // value
    l += scalarProduct(lambda, delta) + 0.5 * mu * scalarProduct(delta, delta);

    // jacobian
    dL += lambda + mu * delta;

    // hessian
    HL += mu * Hb;

    return l;
  }

  void updateADMM(const arr& x, const arr& z)
  {
    lambda += mu * (x - z);
    mu *= admmMuInc;
  }
};

struct DecOptConstrained
{
  std::vector<std::unique_ptr<LagrangianProblem>> Ls;
  std::vector<std::unique_ptr<OptNewton>> newtons;
  std::vector<std::unique_ptr<DecLagrangianProblem>> DLs;
  arr&x; ///< last opt result
  std::vector<arr> xs;
  std::vector<arr> duals;
  OptOptions opt;
  uint its=0;
  ostream *logFile=NULL;

  DecOptConstrained(arr&x, arr & dual, std::vector<ConstrainedProblem*> & Ps, int verbose=-1, OptOptions opt=NOOPT, ostream* _logFile=0)
    : x(x)
    , opt(opt)
    , logFile(_logFile)
  {
    opt.nonStrictSteps = -1;
    Ls.reserve(Ps.size());
    duals.reserve(Ps.size());
    newtons.reserve(Ps.size());
    xs.reserve(Ps.size());

    for(auto & P: Ps)
    {
      xs.push_back(x.copy());
      duals.push_back(dual);

      arr& _x = xs.back();
      arr& _dual = duals.back();

      Ls.push_back(std::unique_ptr<LagrangianProblem>(new LagrangianProblem(*P, opt, _dual)));
      LagrangianProblem& L = *Ls.back();
      DLs.push_back(std::unique_ptr<DecLagrangianProblem>(new DecLagrangianProblem(L, x, opt)));
      DecLagrangianProblem& DL = *DLs.back();
      newtons.push_back(std::unique_ptr<OptNewton>(new OptNewton(_x, DL, opt, logFile)));
    }
  }

  arr Z() const
  {
    arr z(xs.front().d0);

    for(auto i = 0; i < xs.size(); ++i)
    {
      auto& x = xs[i];
      auto& lambda = DLs[i]->lambda;
      z += x + lambda;
    }

    z *= 1.0 / xs.size();

    return z;
  }

  bool step()
  {
    bool stop = true;

    auto z = Z();
    for(auto i = 0; i < DLs.size(); ++i)
    {
      DecLagrangianProblem& DL = *DLs[i];
      OptNewton& newton = *newtons[i];
      arr& dual = duals[i];

      DL.z = z;
      stop = step(DL, newton, dual) && stop;
    }

    z = Z();
    for(auto i = 0; i < DLs.size(); ++i)
    {
      DLs[i]->updateADMM(xs[i], z);
    }

    double e = std::fabs(max(xs.back() - z));

    stop = stop && e < 0.01;

    return stop;
  }

  bool step(DecLagrangianProblem& DL, OptNewton& newton, arr& dual)
  {
    auto& L = DL.L;

    if(opt.verbose>0) {
      cout <<"** DecOptConstr. it=" <<its
           <<" mu=" <<L.mu <<" nu=" <<L.nu <<" muLB=" <<L.muLB;
      if(newton.x.N<5) cout <<" \tlambda=" <<L.lambda;
      cout <<endl;
    }

    arr x_old = newton.x;

    //check for no constraints
    bool newtonOnce=false;
    if(L.get_dimOfType(OT_ineq)==0 && L.get_dimOfType(OT_eq)==0) {
      if(opt.verbose>0) cout <<"** optConstr. NO CONSTRAINTS -> run Newton once and stop" <<endl;
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
      cout <<"** DecOptConstr. it=" <<its
           <<' ' <<newton.evals
           <<" f(x)=" <<L.get_costs()
           <<" \tg_compl=" <<L.get_sumOfGviolations()
           <<" \th_compl=" <<L.get_sumOfHviolations()
           <<" \t|x-x'|=" <<absMax(x_old-newton.x);
      if(newton.x.N<5) cout <<" \tx=" <<newton.x;
      cout <<endl;
    }

    //check for squaredPenaltyFixed method
    if(opt.constrainedMethod==squaredPenaltyFixed) {
      if(opt.verbose>0) cout <<"** optConstr. squaredPenaltyFixed stops after one outer iteration" <<endl;
      return true;
    }

    //check for newtonOnce
    if(newtonOnce) {
      return true;
    }

    //stopping criterons
    if(its>=2 && absMax(x_old-newton.x) < opt.stopTolerance) {
      if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion Delta<" <<opt.stopTolerance <<endl;
      if(opt.stopGTolerance<0.
         || L.get_sumOfGviolations() + L.get_sumOfHviolations() < opt.stopGTolerance)
        return true;
    }

    if(newton.evals>=opt.stopEvals) {
      if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion MAX EVALS" <<endl;
      return true;
    }
    if(newton.its>=opt.stopIters) {
      if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion MAX ITERS" <<endl;
      return true;
    }
    if(its>=opt.stopOuters) {
      if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion MAX OUTERS" <<endl;
      return true;
    }

    double L_x_before = newton.fx;

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

    its++;

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
    x = xs.back();

    // number of evaluation
    std::vector<uint> evals;
    evals.reserve(Ls.size());
    for(const auto& newton: newtons)
    {
      evals.push_back(newton->evals);
    }
    return evals;
  }
};



TEST(DecentralizedAugmentedLagrangian, DecAulaWithDecomposedProblem) {
  arr x{0.0, 0.0, 0.0};
  arr dual;

  Distance3DDecompXY pb0(arr{1.0, 1.0, 1.0});
  Distance3DDecompXZ pb1(arr{1.0, 1.0, 1.0});
  std::vector<ConstrainedProblem*> pbs;
  pbs.push_back(&pb0);
  pbs.push_back(&pb1);

  DecOptConstrained opt(x, dual, pbs);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, DecAulaWithOneProblem) {
  arr x{0.0, 0.0, 0.0};
  arr dual;

  Distance3DDecompXY pb(arr{1.0, 1.0, 1.0});
  std::vector<ConstrainedProblem*> pbs;
  pbs.push_back(&pb);

  DecOptConstrained opt(x, dual, pbs);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(0.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, Distance3DTestHG) {
  arr x{0.0, 0.0, 0.0};
  arr dual;

  Distance3D pb(arr{1.0, 1.0, 1.0});

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, Distance3DDecompXYTestHG) {
  arr x{0.0, 0.0, 0.0};
  arr dual;

  Distance3DDecompXY pb(arr{1.0, 1.0, 1.0});

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(0.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, Distance3DDecompXZTestHG) {
  arr x{0.0, 0.0, 0.0};
  arr dual;

  Distance3DDecompXZ pb(arr{1.0, 1.0, 1.0});

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(0.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
}


//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
