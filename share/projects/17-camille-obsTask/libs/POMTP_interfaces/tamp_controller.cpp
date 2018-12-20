#include <tamp_controller.h>

Skeleton TAMPController::plan( uint maxIt, bool saveInformed, bool saveFinal, bool show, int secs )
{
  /// LOOP
  Skeleton policy, lastPolicy;
  tp_.solve();
  policy = tp_.getPolicy();

  uint nIt = 0;
  do
  {
    nIt++;

    lastPolicy = policy;

    /// MOTION PLANNING
    auto po     = MotionPlanningParameters( policy.id() );
    po.setParam( "type", "markovJointPath" );
    mp_.solveAndInform( po, policy );

    if( saveInformed ) policy.saveAll( "-informed" );

    /// TASK PLANNING
    tp_.integrate( policy );
    tp_.solve();

    policy = tp_.getPolicy();
  }
  while( lastPolicy != policy && nIt != maxIt );

  if( saveFinal ) policy.saveAll( "-final" );

  if( show )
  {
    mp_.display( policy, 3000 );
    mlr::wait( secs, true );
  }

  return policy;
}
