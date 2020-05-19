#pragma once

#include <KOMO/komo-ext.h>
#include <list>
#include <tree_builder.h>

namespace mp
{

//komo joint wrapper
struct KomoJoint
{
  KomoJoint(KOMO_ext * komo)
    : komo_(komo)
    , world_(komo->world)
  {

  }
  void reset(const std::vector<Vars>& branches, double initNoise=0.01);
  void setupConfigurations(const std::vector<Vars>& branches);
  void addObjective(const Interval& it, const TreeBuilder& tb, Feature* map, ObjectiveType type=OT_sos, const arr& target=NoArr, double scale=-1., int order=-1, int deltaFromStep=0, int deltaToStep=0);
  void addSwitch(const Interval& it, const TreeBuilder& tb, rai::KinematicSwitch * sw);

  KOMO_ext * komo_;
  const rai::KinematicWorld & world_;
};

struct ADMM_MotionProblem_GraphProblem : KOMO::Conv_MotionProblem_GraphProblem
{
  ADMM_MotionProblem_GraphProblem(KOMO& _komo) : Conv_MotionProblem_GraphProblem(_komo) {}

  void setSubProblem(const intA & _mask) { mask = _mask; }

  virtual void getStructure(uintA& variableDimensions, intAA& featureVariables, ObjectiveTypeA& featureTypes) override;
  virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x) override;

  intA mask;
};

//struct KomoADMM
//{
//  KomoADMM(KOMO_ext * komo)
//    : komo_(komo)
//    , admm_graph_problem(*komo)
//  {

//  }

//  void run(const intA mask, const arr& x_ref, const arr& y, double rho);

//  KOMO_ext * komo_;
//  //ADMM_MotionProblem_GraphProblem admm_graph_problem;
//};
}
