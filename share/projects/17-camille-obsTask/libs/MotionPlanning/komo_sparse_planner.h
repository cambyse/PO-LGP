#pragma once

#include <string>
#include <unordered_map>
#include <future>

#include <skeleton.h>

#include <komo_factory.h>
#include <komo_planner_config.h>

namespace mp
{

class KOMOSparsePlanner
{
public:
  KOMOSparsePlanner(const KOMOPlannerConfig& config, const KOMOFactory& factory)
    : config_(config)
    , komoFactory_(factory)
  {}

  // common parts between all strategies
  TreeBuilder buildTree( Policy & ) const;
  std::shared_ptr< ExtensibleKOMO > intializeKOMO( const TreeBuilder & tree, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & ) const;
  std::vector<Vars> getSubProblems( const TreeBuilder & tree, Policy & policy ) const;
  std::vector<intA> getSubProblemMasks( const std::vector<Vars> & allVars, uint T ) const;
  void groundPolicyActionsJoint( const TreeBuilder & tree,
                                 Policy & policy,
                                 const std::shared_ptr< ExtensibleKOMO > & komo ) const;
  void watch( const std::shared_ptr< ExtensibleKOMO > & komo ) const;

  virtual void optimize( Policy &, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & ) const = 0;

protected:
  const KOMOPlannerConfig& config_;
  const KOMOFactory & komoFactory_;
};

class JointPlanner : KOMOSparsePlanner
{
public:
  JointPlanner(const KOMOPlannerConfig& config, const KOMOFactory& factory)
    : KOMOSparsePlanner(config, factory)
  {};
  void optimize( Policy &, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & ) const override;
};

class ADMMSParsePlanner : KOMOSparsePlanner
{
public:
  ADMMSParsePlanner(const KOMOPlannerConfig& config, const KOMOFactory& factory)
    : KOMOSparsePlanner(config, factory)
  {};
  void optimize( Policy &, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & ) const override;
};

}
