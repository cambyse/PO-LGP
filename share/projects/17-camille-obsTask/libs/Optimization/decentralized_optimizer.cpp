#include <decentralized_optimizer.h>

#include <future>
#include <thread>


DecOptConstrained::DecOptConstrained(arr& _z, std::vector<std::shared_ptr<ConstrainedProblem>> & Ps, const std::vector<arr> & masks, bool compressed, int verbose, OptOptions _opt, ostream* _logFile)
  : z_final(_z)
  , N(Ps.size())
  , compressed(compressed)
  , contribs(zeros(z_final.d0))
  , z(z_final.copy())
  , opt(_opt)
  , logFile(_logFile)
{
  // maybe preferable to have the same pace for ADMM and AULA terms -> breaks convergence is set to 2.0, strange!
  opt.aulaMuInc = 1.2;

  /// TO BE EQUIVALENT TO PYTHON
  //opt.damping = 0.1;
  //opt.maxStep = 10.0;
  ///

  initVars(masks);
  initXs();
  initLagrangians(Ps);
}

void DecOptConstrained::initVars(const std::vector<arr> & xmasks)
{
  // fill masks with default values if not provided
  std::vector<arr> masks;
  masks.reserve(N);
  if(compressed) CHECK(xmasks.size() > 0, "In compressed mode, the masks should be provided!");
  for(auto i = 0; i < N; ++i)
  {
    masks.push_back( (i < xmasks.size() && xmasks[i].d0 == z.d0 ? xmasks[i] : ones(z.d0) ) );
    contribs += masks.back();
  }

  // fill vars with default values if not provided
  vars.reserve(N);
  for(auto i = 0; i < N; ++i)
  {
    intA var;
    var.reserve(z.d0);

    for(auto k = 0; k < masks[i].d0; ++k)
    {
      if(masks[i](k))
      {
        var.append(k);
      }
      else
      {
        if(!compressed)
          var.append(-1);
      }
    }

    vars.push_back(var);
  }
}

void DecOptConstrained::initXs()
{
  xs.reserve(N);

  for(auto i = 0; i < N; ++i)
  {
    if(compressed)
    {
      const auto& var = vars[i];
      auto x = arr(var.d0);
      for(auto j = 0; j < x.d0; ++j)
      {
        x(j) = z(var(j));
      }
      xs.push_back(x);
    }
    else
    {
      xs.push_back(z.copy());
    }
  }
}

void DecOptConstrained::initLagrangians(const std::vector<std::shared_ptr<ConstrainedProblem>> & Ps)
{
  duals.reserve(N);
  Ls.reserve(N);
  newtons.reserve(N);
  for(auto i = 0; i < N; ++i)
  {
    auto& P = Ps[i];
    auto& var = vars[i];
    arr& x = xs[i];

    duals.push_back(arr());
    arr& _dual = duals.back();
    Ls.push_back(std::unique_ptr<LagrangianProblem>(new LagrangianProblem(*P, opt, _dual)));
    LagrangianProblem& L = *Ls.back();
    DLs.push_back(std::unique_ptr<DecLagrangianProblem>(new DecLagrangianProblem(L, z, var, opt)));
    DecLagrangianProblem& DL = *DLs.back();
    newtons.push_back(std::unique_ptr<OptNewton>(new OptNewton(x, DL, opt, 0)));
  }
}

std::vector<uint> DecOptConstrained::run()
{
  // loop
  while(!step());

  // last step
  for(auto& newton: newtons)
    newton->beta *= 1e-3;

  step();

  // get solution
  z_final = z;

  // number of evaluations
  std::vector<uint> evals;
  evals.reserve(Ls.size());
  for(const auto& newton: newtons)
  {
    evals.push_back(newton->evals);
  }
  return evals;
}

bool DecOptConstrained::step()
{
  std::vector<std::future<bool>> futures;
  for(auto i = 0; i < N; ++i)
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
  for(auto i = 0; i < N; ++i)
  {
    subProblemsSolved = futures[i].get() && subProblemsSolved;
  }

  // stop criterion
  auto z_old = z; // copy
  updateZ();

  // update
  for(auto i = 0; i < N; ++i)
  {
    OptNewton& newton = *newtons[i];
    DLs[i]->updateADMM(xs[i], z);
    // update newton cache / state (necessary because we updated the underlying problem!)
    newton.fx = DLs[i]->decLagrangian(newton.gx, newton.Hx, xs[i]); // this is important!
  }

  double r = primalResidual();
  double s = DLs.front()->mu * length(z - z_old); // dual residual

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


bool DecOptConstrained::step(DecLagrangianProblem& DL, OptNewton& newton, arr& dual, uint i) const
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

void DecOptConstrained::updateZ()
{
  /// Z
  z = zeros(z.d0);
  for(auto i = 0; i < N; ++i)
  {
    const auto& x = xs[i];
    const auto& lambda = DLs[i]->lambda;
    const auto& var = vars[i];

    arr zinc = x;
    if(DLs[i]->mu > 0.0) // add term based on admm lagrange term always except in the first step
      zinc += lambda / DLs[i]->mu;

    // add increment
    for(auto i = 0; i < var.d0; ++i)
    {
      const auto& I = var(i);
      if(I!=-1)
        z(I) += x(i);
    }

    // sanity check // lagrange admm sum = 0 after one iteration
  }

  // contrib scaling
  // correct for correct average
  for(uint i=0;i<z.d0;i++) z.elem(i) /= contribs.elem(i);
}

double DecOptConstrained::primalResidual() const
{
  ///Primal residual
  double r = 0;

  for(auto i = 0; i < N; ++i)
  {
    arr delta = DLs[i]->deltaZ(xs[i]);
    r += length(delta);
  }

  r /= xs.size();

  return r;
}

bool DecOptConstrained::primalFeasibility(double r) const
{
  const double eps = 1e-3 * sqrt(z.d0) + 1e-3 * max(fabs(z));
  return r < eps;
}

bool DecOptConstrained::dualFeasibility(double s) const
{
  double ymax = 0;
  for(const auto& dl: DLs)
  {
    ymax = std::max(ymax, max(fabs(dl->lambda)));
  }
  const double eps = 1e-3 * sqrt(z.d0) + 1e-3 * ymax;
  return s < eps;
}

