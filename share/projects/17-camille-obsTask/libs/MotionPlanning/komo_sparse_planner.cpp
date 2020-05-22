#include <komo_sparse_planner.h>
#include <tree_builder.h>
#include <komo_wrapper.h>
#include <trajectory_tree_visualizer.h>
#include <decentralized_optimizer.h>

#include <Kin/kin.h>
#include <Kin/switch.h>
#include <Kin/TM_transition.h>
#include <Kin/TM_FlagConstraints.h>
#include <Kin/TM_FixSwitchedObjects.h>

namespace mp
{
/// COMMON: KOMOSparsePlanner
TreeBuilder KOMOSparsePlanner::buildTree( Policy & policy ) const
{
  TreeBuilder treeBuilder;

  std::list< Policy::GraphNodeTypePtr > fifo;
  fifo.push_back( policy.root() );

  while( ! fifo.empty() )
  {
    auto b = fifo.back();
    fifo.pop_back();

    const auto& a = b->parent();

    if(a)
    {
      const auto& p = b->data().p;
      treeBuilder.add_edge(a->id(), b->id(), p);
    }

    for(const auto&c : b->children())
    {
      fifo.push_back(c);
    }
  }

  return treeBuilder;
}
std::shared_ptr< ExtensibleKOMO > KOMOSparsePlanner::intializeKOMO( const TreeBuilder & tree, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics ) const
{
  auto komo = komoFactory_.createKomo();
  komo->setModel(*startKinematics.front());
  komo->sparseOptimization = true;
  komo->groundInit(tree);

  const auto nPhases = tree.n_nodes() - 1;
  komo->setTiming(nPhases, config_.microSteps_, config_.secPerPhase_, 2);

  return komo;
}
std::vector<Vars> KOMOSparsePlanner::getSubProblems( const TreeBuilder & tree, Policy & policy ) const
{
  std::vector<Vars> allVars;
  allVars.reserve(policy.leaves().size());

  auto leaves = policy.leaves();

  leaves.sort([](Policy::GraphNodeTypePtr a, Policy::GraphNodeTypePtr b)->bool
  {return a->id() < b->id();});

  for(const auto& l: leaves)
  {
    auto vars0 = tree.get_vars({0, 1.0 * l->depth()}, l->id(), 0, config_.microSteps_);
    auto vars1 = tree.get_vars({0, 1.0 * l->depth()}, l->id(), 1, config_.microSteps_);
    auto vars2 = tree.get_vars({0, 1.0 * l->depth()}, l->id(), 2, config_.microSteps_);
    Vars branch{vars0, vars1, vars2, config_.microSteps_};
    allVars.push_back(branch);
  }

  return allVars;
}
std::vector<intA> KOMOSparsePlanner::getSubProblemMasks( const std::vector<Vars> & allVars, uint T ) const
{
  std::vector<intA> masks(allVars.size());
  for(auto w = 0; w < allVars.size(); ++w)
  {
    auto & mask = masks[w];
    auto & vars = allVars[w][0];

    mask = intA(T);

    for(auto i: vars)
    {
      mask(i) = 1;
    }
  }

  return masks;
}

void KOMOSparsePlanner::groundPolicyActionsJoint( const TreeBuilder & tree,
                               Policy & policy,
                               const std::shared_ptr< ExtensibleKOMO > & komo ) const
{
  using W = KomoWrapper;

  auto leaves = policy.leaves();

  leaves.sort([](Policy::GraphNodeTypePtr a, Policy::GraphNodeTypePtr b)->bool
  {return a->id() < b->id();});

  std::vector<uint> visited(tree.n_nodes(), 0);
  for(const auto& l: leaves)
  {
    auto q = l;
    auto p = q->parent();

    while(p)
    {
      if(!visited[q->id()])
      {
        double start = p->depth();
        double end = q->depth();

        Interval interval;
        interval.time = {start, start + 1.0};
        interval.edge = {p->id(), q->id()};

        // square acc
        W(komo.get()).addObjective(interval, tree, new TM_Transition(komo->world), OT_sos, NoArr, 1.0, 2);

        // ground other tasks
        komo->groundTasks(interval, tree, q->data().leadingKomoArgs, 1);

        visited[q->id()] = 1;
      }
      q = p;
      p = q->parent();
    }
  }
}

void KOMOSparsePlanner::watch( const std::shared_ptr< ExtensibleKOMO > & komo ) const
{
  //komo->displayTrajectory(0.1, true, false);
  Var<WorldL> configs;
  auto v = std::make_shared<KinPathViewer>(configs,  0.2, -0 );
  v->setConfigurations(komo->configurations);
  rai::wait();
}

/// JOINT
void JointPlanner::optimize( Policy & policy, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics ) const
{
  using W = KomoWrapper;

  // build tree
  auto treeBuilder = buildTree(policy);

  // prepare komo
  auto komo = intializeKOMO(treeBuilder, startKinematics);

  // ground policy actions
  auto allVars = getSubProblems(treeBuilder, policy);
  groundPolicyActionsJoint(treeBuilder, policy, komo);

  // run optimization
  komo->verbose = 3;
  W(komo.get()).reset(allVars);
  komo->run();

  //komo->getReport(true);
  //for(auto c: komo->configurations) std::cout << c->q.N << std::endl;

  watch(komo);
}

/// ADMM SPARSE
void ADMMSParsePlanner::optimize( Policy & policy, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics ) const
{
  using W = KomoWrapper;

  // build tree
  auto tree = buildTree(policy);

  // prepare komos
  std::vector< std::shared_ptr< ExtensibleKOMO > > komos;
  for(auto w = 0; w < policy.leaves().size(); ++w)
  {
    auto komo = intializeKOMO(tree, startKinematics);
    komos.push_back(komo);
  }

  // get subproblems
  auto allVars = getSubProblems(tree, policy);
  auto allMasks = getSubProblemMasks(allVars, komos.front()->T);

  // ground each komo
  for(uint w = 0; w < policy.leaves().size(); ++w)
  {
    groundPolicyActionsJoint(tree, policy, komos[w]);
  }

  // reset each komo
  for(auto & komo: komos)
  {
    W(komo.get()).reset(allVars, 0);
  }

  // SEQUENTIAL ADMM
  auto x = komos.front()->x;

  std::vector<std::shared_ptr<GraphProblem>> converters;
  std::vector<std::shared_ptr<ConstrainedProblem>> constrained_problems;
  std::vector<arr> xmasks;
  converters.reserve(policy.leaves().size());
  constrained_problems.reserve(policy.leaves().size());
  xmasks.reserve(policy.leaves().size());

  for(auto w = 0; w < policy.leaves().size(); ++w)
  {
    auto& komo = *komos[w];
    auto& mask = allMasks[w];

    auto gp = std::make_shared<ADMM_MotionProblem_GraphProblem>(komo);
    gp->setSubProblem(mask);
    arr xmask;
    gp->getXMask(xmask);

    auto pb = std::make_shared<Conv_Graph_ConstrainedProblem>(*gp, komo.logFile);

    converters.emplace_back(gp);
    constrained_problems.push_back(pb);
    xmasks.push_back(xmask);
  }

  // RUN
  double timeZero = rai::timerStart();

  DecOptConstrained opt(x, constrained_problems, xmasks);
  opt.run();

  double optimizationTime = rai::timerRead(true, timeZero);

  // LOGS
  if(true) {
    cout <<"** optimization time=" << optimizationTime
         <<" setJointStateCount=" << rai::KinematicWorld::setJointStateCount <<endl;
  }
  //

  auto & komo = komos.front();
  komo->set_x(x);
  watch(komos.front());
}

/// ADMM COMPRESSED
void ADMMCompressedPlanner::optimize( Policy & policy, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics ) const
{
  using W = KomoWrapper;

  // build tree
  auto tree = buildTree(policy);

  // prepare komos
  std::vector< std::shared_ptr< ExtensibleKOMO > > komos;
  for(auto w = 0; w < policy.leaves().size(); ++w)
  {
    auto komo = intializeKOMO(tree, startKinematics);
    komos.push_back(komo);
  }

}
}
