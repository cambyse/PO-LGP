#include <Optimization/decentralized_optimizer.h>
#include <Optimization/utils.h>

#include <future>
#include <thread>

namespace
{
  arr getSub(const arr& zz, const intA& var) // extract sub-variable global zz
  {
    arr x(var.d0);

    for(auto i = 0; i < var.d0; ++i)
    {
      const auto& I = var(i);
      if(I!=-1)
        x(i) = zz(I);
    }

    return x;
  }

  void setFromSub(const arr& x, const intA& var, arr & zz) // set global variable zz from sub x
  {
    for(auto i = 0; i < var.d0; ++i)
    {
      const auto& I = var(i);
      if(I!=-1)
        zz(I) = x(i);
    }
  }
}

DecOptConstrained::DecOptConstrained(arr& _z, std::vector<std::shared_ptr<ConstrainedProblem>> & Ps, const std::vector<arr> & masks, DecOptConfig _config)//bool compressed, int verbose, OptOptions _opt, ostream* _logFile)
  : z_final(_z)
  , N(Ps.size())
  , contribs(zeros(z_final.d0))
  , z(z_final.copy())
  , config(_config)
{
  // maybe preferable to have the same pace for ADMM and AULA terms -> breaks convergence is set to 2.0, strange!
  config.opt.aulaMuInc = 1.2;

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
  if(config.compressed) CHECK(xmasks.size() > 0, "In compressed mode, the masks should be provided!");
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
        if(!config.compressed)
          var.append(-1);
      }
    }

    vars.push_back(var);
  }

  // count where admm comes into play (for averaging contributions and computing primal residual)
  for(auto i = 0; i < contribs.d0; ++i)
  {
    if(contribs(i)>1) m++;
  }

  // fill admm vars
  admmVars.reserve(N);
  for(auto i = 0; i < N; ++i)
  {
    const auto & var = vars[i];
    intA admmVar;

    for(auto i = 0; i < var.d0; ++i)
    {
      if(contribs(var(i))>1) // another subproblem contributes here, we activate the ADMM term
      {
        admmVar.append(i);
      }
    }

    admmVars.push_back(admmVar);
  }
}

void DecOptConstrained::initXs()
{
  xs.reserve(N);

  for(auto i = 0; i < N; ++i)
  {
    if(config.compressed)
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
    auto& admmVar = admmVars[i];
    arr& x = xs[i];

    duals.push_back(arr());
    arr& _dual = duals.back();
    Ls.push_back(std::unique_ptr<LagrangianProblem>(new LagrangianProblem(*P, config.opt, _dual)));
    LagrangianProblem& L = *Ls.back();
    DLs.push_back(std::unique_ptr<DecLagrangianProblem>(new DecLagrangianProblem(L, z, var, admmVar, config.opt)));
    DecLagrangianProblem& DL = *DLs.back();
    newtons.push_back(std::unique_ptr<OptNewton>(new OptNewton(x, DL, config.opt, 0)));
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
  if(config.scheduling == SEQUENTIAL || (config.scheduling == FIRST_ITERATION_SEQUENTIAL_THEN_PARALLEL && its == 0))
  {
    stepSequential();
  }
  else if(config.scheduling == PARALLEL || (config.scheduling == FIRST_ITERATION_SEQUENTIAL_THEN_PARALLEL && its > 0))
  {
    stepParallel();
  }
  else NIY;

  updateADMM(); // update lagrange parameters of ADMM terms based on updated Z (make all problem converge)

  return stoppingCriterion();
}

bool DecOptConstrained::stepSequential()
{
  subProblemsSolved = true;
  arr zz = z; // local z modified by each subproblem in sequence (we can't use the z here, which has to be consistent among all subproblems iterations)
  for(auto i = 0; i < N; ++i)
  {
    DecLagrangianProblem& DL = *DLs[i];
    OptNewton& newton = *newtons[i];
    arr& dual = duals[i];
    const auto& var = vars[i];

    DL.z = z; // set global reference

    auto x_start = getSub(zz, var);
    newton.reinit(x_start);
    subProblemsSolved = step(DL, newton, dual, i) && subProblemsSolved;
    setFromSub(xs[i], var, zz);
  }

  // update
  z_prev = z;
  updateZ();
}

bool DecOptConstrained::stepParallel()
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
  subProblemsSolved = true;
  for(auto i = 0; i < N; ++i)
  {
    subProblemsSolved = futures[i].get() && subProblemsSolved;
  }

  // update
  z_prev = z;
  updateZ();
}

bool DecOptConstrained::step(DecLagrangianProblem& DL, OptNewton& newton, arr& dual, uint i) const
{
  auto& L = DL.L;

  if(config.opt.verbose>0) {
    cout <<"** DecOptConstr.[" << i << "] it=" <<its
         <<" mu=" <<L.mu <<" nu=" <<L.nu <<" muLB=" <<L.muLB;
    if(newton.x.N<5) cout <<" \tlambda=" <<L.lambda;
    cout <<endl;
  }

  arr x_old = newton.x;

  //check for no constraints
  bool newtonOnce=false;
  if(L.get_dimOfType(OT_ineq)==0 && L.get_dimOfType(OT_eq)==0) {
    if(config.opt.verbose>0) cout <<"** [" << i << "] optConstr. NO CONSTRAINTS -> run Newton once and stop" <<endl;
    newtonOnce=true;
  }

  //run newton on the Lagrangian problem
  if(newtonOnce || config.opt.constrainedMethod==squaredPenaltyFixed) {
    newton.run();
  } else {
    double stopTol = newton.o.stopTolerance;
    if(config.opt.constrainedMethod==anyTimeAula)  newton.run(20);
    else                                    newton.run();
    newton.o.stopTolerance = stopTol;
  }

  if(config.opt.verbose>0) {
    cout <<"** Hessian size.[" << newton.Hx.d0 << "] sparsity=" << sparsity(newton.Hx);
    cout <<endl;
  }

  if(config.opt.verbose>0) {
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
  if(config.opt.constrainedMethod==squaredPenaltyFixed) {
    if(config.opt.verbose>0) cout <<"** optConstr.[" << i << "] squaredPenaltyFixed stops after one outer iteration" <<endl;
    return true;
  }

  //check for newtonOnce
  if(newtonOnce) {
    return true;
  }

  //stopping criterons
  const auto & opt = config.opt;
  if(its>=2 && absMax(x_old-newton.x) < config.opt.stopTolerance) {
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

  if(config.logFile){
    (*config.logFile) <<"{ optConstraint: " <<its <<", mu: " <<L.mu <<", nu: " <<L.nu <<", L_x_beforeUpdate: " <<L_x_before <<", L_x_afterUpdate: " <<newton.fx <<", errors: ["<<L.get_costs() <<", " <<L.get_sumOfGviolations() <<", " <<L.get_sumOfHviolations() <<"], lambda: " <<L.lambda <<" }," <<endl;
  }

  return false;
}

void DecOptConstrained::updateZ()
{
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
        z(I) += zinc(i);
    }

    // TODO: sanity check // lagrange admm sum = 0 after one iteration
  }

  // contrib scaling
  for(uint i=0;i<z.d0;i++) z.elem(i) /= contribs.elem(i);
}

void DecOptConstrained::updateADMM()
{
  its++;

  for(auto i = 0; i < N; ++i)
  {
    OptNewton& newton = *newtons[i];
    DLs[i]->updateADMM(xs[i], z);
    // update newton cache / state (necessary because we updated the underlying problem by updating the ADMM params!!)
    newton.fx = DLs[i]->decLagrangian(newton.gx, newton.Hx, xs[i]); // this is important!
  }
}

bool DecOptConstrained::stoppingCriterion() const
{
  double r = primalResidual();
  double s = dualResidual();

  // stop criterion
  const auto& opt = config.opt;
  if(opt.verbose>0) {
    cout <<"** DecOptConstr.[x] ADMM UPDATE";
    cout <<"\t |x-z|=" << r;
    if(z.d0 < 10) cout << '\t' << "z=" << z;
    cout <<endl<<endl;
  }

  // nominal exit condition
  if(subProblemsSolved && primalFeasibility(r) && dualFeasibility(s))
  {
    if(opt.verbose>0) cout <<"** ADMM StoppingCriterion Primal Dual convergence" << std::endl;
    return true;
  }
  // other exit conditions
  if(subProblemsSolved && absMax(z - z_prev) < opt.stopTolerance) { // stalled ADMM
    if(opt.verbose>0) cout <<"** ADMM StoppingCriterion Delta<" <<opt.stopTolerance <<endl;
    if(opt.stopGTolerance<0. || r < opt.stopGTolerance)
      return true;
  }

  return false;
}

double DecOptConstrained::primalResidual() const
{
  double r = 0;

  for(auto i = 0; i < N; ++i)
  {
    arr delta = DLs[i]->deltaZ(xs[i]);
    r += length(delta);
  }

  r /= xs.size();

  return r;
}

double DecOptConstrained::dualResidual() const
{
  return DLs.front()->mu * length(z - z_prev);
}

bool DecOptConstrained::primalFeasibility(double r) const
{
  const double eps = 1e-2 * sqrt(m) + 1e-3 * max(fabs(z));
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

