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

#include <unordered_set>

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

std::shared_ptr< ExtensibleKOMO > KOMOSparsePlanner::intializeKOMO( const TreeBuilder & tree, const std::shared_ptr< const rai::KinematicWorld > & startKinematic ) const
{
  auto komo = komoFactory_.createKomo();
  komo->setModel(*startKinematic);
  komo->sparseOptimization = true;

  const auto nPhases = tree.n_nodes() - 1;
  komo->setTiming(nPhases, config_.microSteps_, config_.secPerPhase_, 2);

  return komo;
}

std::vector<Vars> KOMOSparsePlanner::getSubProblems( const TreeBuilder & tree, Policy & policy ) const
{
  std::vector<Vars> allVars;
  allVars.reserve(tree.get_leaves().size());

  for(const auto& l: policy.sleaves())
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
  // traverse tree and ground symbols
  std::unordered_set<uint> visited;
  for(const auto& l: policy.sleaves())
  {
    auto q = l;
    auto p = q->parent();

    while(p)
    {
      if(visited.find(q->id()) == visited.end())
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

        visited.insert(q->id());
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
  auto tree = buildTree(policy);

  // prepare komo
  auto komo = intializeKOMO(tree, startKinematics.front());

  // ground policy actions
  komo->groundInit(tree);
  auto allVars = getSubProblems(tree, policy);
  groundPolicyActionsJoint(tree, policy, komo);

  // run optimization
  komo->verbose = 3;
  W(komo.get()).reset(allVars);

  auto start = std::chrono::high_resolution_clock::now();

  komo->run();

  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  double optimizationTime=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;

  // LOGS
  if(true) {
    cout <<"** optimization time=" << optimizationTime
         <<" setJointStateCount=" << rai::KinematicWorld::setJointStateCount <<endl;
  }
  //
  //komo->getReport(true);
  //for(auto c: komo->configurations) std::cout << c->q.N << std::endl;

  watch(komo);
}

/// ADMM SPARSE
void ADMMSParsePlanner::optimize( Policy & policy, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics ) const
{
  // build tree
  auto tree = buildTree(policy);

  // prepare komos
  std::vector< std::shared_ptr< ExtensibleKOMO > > komos;
  for(auto w = 0; w < policy.leaves().size(); ++w)
  {
    auto komo = intializeKOMO(tree, startKinematics.front());
    komos.push_back(komo);
  }

  // ground each komo
  for(uint w = 0; w < policy.leaves().size(); ++w)
  {
    groundPolicyActionsJoint(tree, policy, komos[w]);
  }

  // reset each komo
  auto allVars = getSubProblems(tree, policy);   // get subproblems
  auto allTMasks = getSubProblemMasks(allVars, komos.front()->T);

  for(auto & komo: komos)
  {
    W(komo.get()).reset(allVars, 0);
  }

  // ADMM
  std::vector<std::shared_ptr<GraphProblem>> converters;
  std::vector<std::shared_ptr<ConstrainedProblem>> constrained_problems;
  std::vector<arr> xmasks;
  converters.reserve(policy.leaves().size());
  constrained_problems.reserve(policy.leaves().size());
  xmasks.reserve(policy.leaves().size());

  for(auto w = 0; w < policy.leaves().size(); ++w)
  {
    auto& komo = *komos[w];
    auto& tmask = allTMasks[w];

    auto gp = std::make_shared<ADMM_MotionProblem_GraphProblem>(komo);
    gp->setSubProblem(tmask);
    arr xmask;
    gp->getXMask(xmask);

    auto pb = std::make_shared<Conv_Graph_ConstrainedProblem>(*gp, komo.logFile);

    converters.emplace_back(gp);
    constrained_problems.push_back(pb);
    xmasks.push_back(xmask);
  }

  // RUN
  auto start = std::chrono::high_resolution_clock::now();

  auto x = komos.front()->x;
  DecOptConstrained opt(x, constrained_problems, xmasks);
  opt.run();

  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  double optimizationTime=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;


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
void ADMMCompressedPlanner::groundPolicyActionsCompressed( const TreeBuilder & policyTree,
                                                           const TreeBuilder & komoTree,
                                                           const Mapping & mapping,
                                                           Policy & policy,
                                                           const std::shared_ptr< ExtensibleKOMO > & komo ) const
{
  // policy_tree and komo_tree differ in general (komo_tree could be a subtree or branch only)

  // traverse tree and ground symbols
  std::unordered_set<uint> visited;
  for(const auto& l: policy.sleaves())
  {
    auto q = l;
    auto p = q->parent();

    while(p)
    {
      if(visited.find(q->id()) == visited.end() && policyTree.has_node(q->id()))
      {
        double start = p->depth();
        double end = q->depth();

        auto pid = mapping.orig_to_compressed(p->id());
        auto qid = mapping.orig_to_compressed(q->id());

        Interval interval;
        interval.time = {start, start + 1.0};
        interval.edge = {pid, qid};

        // square acc
        W(komo.get()).addObjective(interval, komoTree, new TM_Transition(komo->world), OT_sos, NoArr, 1.0, 2);

        // ground other tasks
        komo->groundTasks(interval, komoTree, q->data().leadingKomoArgs, 1);

        visited.insert(q->id());
      }
      q = p;
      p = q->parent();
    }
  }
}

void ADMMCompressedPlanner::optimize( Policy & policy, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics ) const
{
  using W = KomoWrapper;

  // build tree
  auto tree = buildTree(policy);

  // prepare komos
  std::vector< std::tuple< TreeBuilder, TreeBuilder, Mapping > > branches;
  std::vector< std::shared_ptr< ExtensibleKOMO > > komos;
  auto witness = intializeKOMO(tree, startKinematics.front());
  uint w = 0;

  for(auto l: policy.sleaves())
  {
    Mapping mapping;
    auto policyBranch = tree.get_branch(l->id());
    auto komoBranch = policyBranch.compressed(mapping);
    auto komo = intializeKOMO(komoBranch, startKinematics(w));

    std::cout << komoBranch << std::endl;

    komos.push_back(komo);
    branches.push_back(std::tuple< TreeBuilder, TreeBuilder, Mapping >{policyBranch, komoBranch, mapping});

    ++w;
  }

  // ground each komo
  groundPolicyActionsJoint(tree, policy, witness);
  for(auto w = 0; w < policy.leaves().size(); ++w)
  {
    auto branchInfo = branches[w];
    const auto& policyBranch = std::get<0>(branchInfo);
    const auto& komoBranch = std::get<1>(branchInfo);
    const auto& mapping = std::get<2>(branchInfo);
    groundPolicyActionsCompressed(policyBranch, komoBranch, mapping, policy, komos[w]);
  }

  // get mask of subproblems using witness
  auto allVars = getSubProblems(tree, policy);   // get subproblems
  auto allTMasks = getSubProblemMasks(allVars, witness->T);

  // reset
  W(witness.get()).reset(allVars, 0);
  for(auto w = 0; w < policy.leaves().size(); ++w)
  {
    komos[w]->reset();
  }

  std::vector<arr> xmasks;
  xmasks.reserve(policy.leaves().size());
  auto gp = std::make_shared<ADMM_MotionProblem_GraphProblem>(*witness);
  for(auto w = 0; w < policy.leaves().size(); ++w)
  {
    auto& tmask = allTMasks[w];
    gp->setSubProblem(tmask);
    arr xmask;
    gp->getXMask(xmask);
    xmasks.push_back(xmask);
  }

  // create subproblems
  std::vector<std::shared_ptr<GraphProblem>> converters;
  std::vector<std::shared_ptr<ConstrainedProblem>> constrained_problems;
  converters.reserve(policy.leaves().size());
  constrained_problems.reserve(policy.leaves().size());
  for(auto w = 0; w < policy.leaves().size(); ++w)
  {
    auto& komo = *komos[w];
    auto gp = std::make_shared<KOMO::Conv_MotionProblem_GraphProblem>(komo);
    auto pb = std::make_shared<Conv_Graph_ConstrainedProblem>(*gp, komo.logFile);

    converters.push_back(gp);
    constrained_problems.push_back(pb);
  }

  // RUN
  auto start = std::chrono::high_resolution_clock::now();

  auto x = witness->x;
  DecOptConstrained opt(x, constrained_problems, xmasks, true);
  opt.run();

  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  double optimizationTime=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;

  // LOGS
  if(true) {
    cout <<"** optimization time=" << optimizationTime
         <<" setJointStateCount=" << rai::KinematicWorld::setJointStateCount <<endl;
  }
  //
  //witness->set_x(x);
  //watch(witness);

  watch(komos.front());
}

}
