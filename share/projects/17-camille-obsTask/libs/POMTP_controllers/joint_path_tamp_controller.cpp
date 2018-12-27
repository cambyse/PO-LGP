#include <joint_path_tamp_controller.h>

#include <set>

Skeleton JointPathTAMPController::plan( uint maxIt, bool saveInformed, bool saveFinal, bool show, int secs )
{
  auto skeletonComp = []( const Skeleton & a, const Skeleton & b ) { return  a.value() > b.value(); };

  /// LOOP
  std::vector< Skeleton > markovPolicies;
  std::vector< Skeleton > jointPolicies;
  Skeleton policy, previousPolicy;
  tp_.solve();
  policy = tp_.getPolicy();

  uint nIt = 0;

  while( policy != previousPolicy && nIt != maxIt )
  {
    nIt++;

    /// MOTION PLANNING
    auto po     = MotionPlanningParameters( policy.id() );
    po.setParam( "type", "markovJointPath" );
    mp_.solveAndInform( po, policy );

    if( saveInformed ) policy.saveAll( "-informed" );

    /// TASK PLANNING
    tp_.integrate( policy );
    tp_.solve();

    markovPolicies.push_back( policy );
    previousPolicy = policy;
    policy = tp_.getPolicy();
  }

  /// JOINT PATH
  std::sort( markovPolicies.begin(), markovPolicies.end(), skeletonComp );
  uint nJointReplanMax = 5;
  uint nJointReplans = 0;
  for( auto policy : markovPolicies )
  {
    auto po     = MotionPlanningParameters( policy.id() );
    po.setParam( "type", "jointPath" );
    mp_.solveAndInform( po, policy );
    jointPolicies.push_back( policy );

    nJointReplans++;
    if( nJointReplans >= nJointReplanMax )
      break;
  }

  std::sort( jointPolicies.begin(), jointPolicies.end(), skeletonComp );
  policy = *jointPolicies.begin();

  /// DIPSLAY
  if( saveFinal ) policy.saveAll( "-final" );

  if( show )
  {
    mp_.display( policy, 3000 );
    mlr::wait( secs, true );
  }

  return policy;
}
