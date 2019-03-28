#pragma once

#include <string>

#include <skeleton.h>
#include <motion_planner.h>

#include <komo_factory.h>
#include <path_evaluator.h>

namespace mp
{

class KOMOPlanner : public MotionPlanner
{
  using PolicyNodePtr = Skeleton::GraphNodeTypePtr;

public:
  // modifiers
  void setKin( const std::string & kinDescription ) override;
  std::vector< double > drawRandomVector( const std::vector< double > & override = std::vector< double >() );

  // informers
  void solveAndInform( const MotionPlanningParameters &, Skeleton & ) override;

  // display
  void display( const Skeleton & policy, double sec ) override;
  std::pair< double, double > evaluateLastSolution();

  // ground symbols
  void registerInit( const InitGrounder & grounder );
  void registerTask( const std::string & type, const SymbolGrounder & grounder );

  void setSecsPerPhase( double s ) { secPerPhase_ = s; }
  void setNSteps( uint n ) { microSteps_ = n; }
  void setMinMarkovianCost( double m ) { minMarkovianCost_ = m; }

private:
  void computeQMask();

  /// MARKOVIAN
  // poses
  void optimizePoses( Skeleton & );
  void optimizePosesFrom( const PolicyNodePtr & );

  // markovian path
  void optimizeMarkovianPath( Skeleton & );
  void optimizeMarkovianPathFrom( const PolicyNodePtr & );

  /// NON MARKOVIAN
  void clearLastNonMarkovianResults();
  // path
  void optimizePath( Skeleton & );
  void optimizePathTo( const PolicyNodePtr & );
  void computePathQResult( const Skeleton& policy );

  // joint path
  void optimizeJointPath( Skeleton & );
  void optimizeJointPathTo( const PolicyNodePtr & );
  void computeJointPathQResult( const Skeleton& policy );

private:
  // state
  rai::Array< std::shared_ptr< const rai::KinematicWorld > > startKinematics_;
  arr qmask_; //1 for agent, 0 for non agent joint
  KOMOFactory komoFactory_;

  std::vector< double > randomVec_; // used to randomize the initial configuration

  // pose
  std::map< uint, rai::Array< rai::KinematicWorld > > effKinematics_;
  std::map< uint, arr > poseCosts_;       // node id -> costs for each world
  std::map< uint, arr > poseConstraints_; // node id -> constraints for each world

  // markovian path
  std::map< uint, rai::Array< rai::KinematicWorld > > effMarkovianPathKinematics_;
  std::map< uint, double > markovianPathCosts_; // node id -> averaged cost
  std::map< uint, double > markovianPathConstraints_; // node id -> averaged constraints

  // path
  std::map< PolicyNodePtr, rai::Array< rai::Array< rai::KinematicWorld > > > pathKinFrames_; // node(leaf) -> trajectory for each world
  std::map< PolicyNodePtr, rai::Array< arr > > pathXSolution_; // node(leaf) -> x for each world
  std::map< PolicyNodePtr, rai::Array< arr > > pathCostsPerPhase_;
  QResult pathQResult_;

  // joint path
  std::map< PolicyNodePtr, rai::Array< rai::Array< rai::KinematicWorld > > > jointPathKinFrames_; // maps each leaf to its path // memory leak?
  std::map< PolicyNodePtr, rai::Array< arr > > jointPathCostsPerPhase_;
  rai::Array< PolicyNodePtr > bsToLeafs_; //indicates the leaf terminating for a given state
  QResult jointPathQResult_;

  // params
  const rai::String beliefStateTag_  = "BELIEF_START_STATE";

  double kinEqualityWeight_  = 1e4;
  double fixEffJointsWeight_ = 1e3;
  double secPerPhase_        = 10.;

  double maxConstraint_      = 10 * 0.5;

  uint phase_start_offset_ = 1; // the first task should be grounded starting from this time
  uint phase_end_offset_   = 1;

  double minMarkovianCost_   = 0;

  uint microSteps_     = 20; // per phase
};

void freeKomo( ExtensibleKOMO::ptr komo );

}
