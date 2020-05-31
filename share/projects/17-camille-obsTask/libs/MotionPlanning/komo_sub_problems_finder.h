#pragma once

#include <string>
#include <unordered_map>
#include <future>

#include <skeleton.h>

#include <komo_factory.h>
#include <komo_planner_config.h>
#include <komo_wrapper.h>
#include <komo_sparse_planner.h>

namespace mp
{

class KOMOSubProblemsFinder : KOMOSparsePlanner
{
public:
  KOMOSubProblemsFinder(const KOMOPlannerConfig& config, const KOMOFactory& factory)
    : KOMOSparsePlanner(config, factory) {};

  void analyse(Policy &, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > &);
  void optimize( Policy &, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & ) const override{};
};

}
