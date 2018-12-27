#include <markovian_tamp_controller.h>

Skeleton MarkovianTAMPController::plan( uint maxIt, bool saveInformed, bool saveFinal, bool show, int secs )
{
  /// LOOP
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

    previousPolicy = policy;
    policy = tp_.getPolicy();
  }

  if( saveFinal ) policy.saveAll( "-final" );

  if( show )
  {
    mp_.display( policy, 3000 );
    mlr::wait( secs, true );
  }

  return policy;
}
