#include <komo_sub_problems_finder.h>
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
/// SUBPROBLEMS ANALYSER COMPRESSED
void KOMOSubProblemsFinder::analyse(Policy & policy, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics)
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
  komo->opt->newton.Hx;
}

}
