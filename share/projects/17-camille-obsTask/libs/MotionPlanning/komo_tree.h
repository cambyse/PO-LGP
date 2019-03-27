/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#pragma once

#include <memory>
#include <map>

#include <KOMO/komo.h>

namespace mp
{

//=====TreeTask==============================================

struct Branch
{
    std::vector< int > local_to_global;
    std::vector< int > global_to_local;
    double p; // probability to reach the leaf
    uint leaf_id;

    static Branch computeMicroStepBranch(const Branch& a, int stepsPerPhase);
    static Branch linearTrajectory(int T);
};

bool operator==(const Branch& a, const Branch& b);
bool operator<(const Branch& a, const Branch& b);

struct TreeTask : public Task {
  Branch branch;           ///< way to traverse x allow traj tree opt : link from branch index to x index

  TreeTask(TaskMap *m, const ObjectiveType& type) : Task(m, type){}
  virtual ~TreeTask(){}

  void setCostSpecs(int fromTime, int toTime, int T,
                    const arr& _target=ARR(0.),
                    double _prec=1.);
  void setCostSpecs(double fromTime, double toTime,
                    int stepsPerPhase, uint T,
                    const arr& _target,
                    double _prec, const Branch& branch_time_spec=Branch());

  int to_local_t(int global_t) const { CHECK(global_t < branch.global_to_local.size(), "wrong dimensions!");return branch.global_to_local[global_t]; }
  int to_global_t(int local_t) const { CHECK(local_t < branch.local_to_global.size(), "wrong dimensions!"); return branch.local_to_global[local_t];/*local_t < branch.local_to_global.size() ? branch.local_to_global[local_t] : branch.local_to_global.back();*/ }
};

//=====KOMOTree==============================================

class KOMOTree : public KOMO
{
public:
  KOMOTree();
  virtual void run() override;
  virtual bool checkGradients() override;
  virtual struct Task* setTask(double startTime, double endTime, TaskMap* map, ObjectiveType type=OT_sumOfSqr, const arr& target=NoArr, double prec=1e2, uint order=0) override;
  TreeTask* addTreeTask(const char* name, TaskMap *map, const ObjectiveType& termType); ///< manually add a task
  struct TreeTask* setTreeTask(double startTime, double endTime, const Branch& branch, TaskMap* map, ObjectiveType type=OT_sumOfSqr, const arr& target=NoArr, double prec=1e2, uint order=0);
  virtual bool displayTrajectory(double delay=0.01, bool watch=false) override; ///< display th

private:
  struct Conv_Tree_KOMO_Problem : KOMO_Problem{
    KOMO& komo;
    uint dimPhi;

    Conv_Tree_KOMO_Problem(KOMO& _komo) : komo(_komo){}

    WorldL getConfigurations(TreeTask* task, uint t) const;
    virtual uint get_k(){ return komo.k_order; }
    virtual void getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes);
    virtual void phi(arr& phi, arrA& J, arrA& H, ObjectiveTypeA& tt, const arr& x);
  } komo_tree_problem;
};

//-- converters
struct Conv_KOMO_Tree_ConstrainedProblem : ConstrainedProblem{
  KOMO_Problem& KOMO;
  uintA variableDimensions, varDimIntegral;
  uintA featureTimes;
  ObjectiveTypeA featureTypes;
  arrA J_KOMO, H_KOMO;

  Conv_KOMO_Tree_ConstrainedProblem(KOMO_Problem& P);

  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x);
};

}
