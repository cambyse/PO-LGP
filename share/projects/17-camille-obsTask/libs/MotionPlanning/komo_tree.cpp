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

#include "komo_tree.h"

#include <set>
#include <iomanip>
#include <Gui/opengl.h>
#include <Kin/frame.h>
#include <Core/graph.h>

namespace mp
{

Branch Branch::computeMicroStepBranch(const Branch& a, int stepsPerPhase)
{
    Branch b;

    b.p = a.p;
    b.leaf_id = a.leaf_id;

    // local
    int n_phases = a.local_to_global.size() - 1;

    b.local_to_global = std::vector< int >(n_phases * stepsPerPhase); // 2 == prefix

    for(int phaseEndIndex = 1; phaseEndIndex <= n_phases; ++phaseEndIndex)
    {
        const uint phaseEnd = a.local_to_global[phaseEndIndex];

        for(int s = 0; s < stepsPerPhase; ++s)
        {
            b.local_to_global[phaseEndIndex * stepsPerPhase - s - 1] = phaseEnd * stepsPerPhase - s -1;
        }
    }

    // global
    int n_global_phases = a.global_to_local.size() - 1;
    b.global_to_local = std::vector< int >(n_global_phases * stepsPerPhase, -1);

    for(int local = 0; local < n_phases * stepsPerPhase; ++local)
    {
        const auto global =b.local_to_global[local];
        b.global_to_local[global] = local;
    }

    // hack
    if( b.local_to_global.size() < 3 )
    {
        b.local_to_global.push_back(2);
        b.global_to_local.push_back(2);
    }

    return b;
}

Branch Branch::linearTrajectory(int T)
{
    Branch b;
    b.p = 1.0;

    b.global_to_local = std::vector< int >(T);
    b.local_to_global = std::vector< int >(T);

    for(auto t = 0; t < T; ++t)
    {
        b.global_to_local[t] = t;
        b.local_to_global[t] = t;
    }

    return b;
}

bool operator==(const Branch& a, const Branch& b)
{
    return (a.p == b.p) && (a.local_to_global == b.local_to_global) && (a.global_to_local == b.global_to_local) && (a.leaf_id == b.leaf_id);
}

bool operator<(const Branch& a, const Branch& b)
{
    return a.leaf_id < b.leaf_id;
}

//===========================================================================

//#define STEP(p) (floor(branch.pathFromRoot[p]*double(stepsPerPhase) + .500001))-1

double phaseToStep(double p, double stepsPerPhase)
{
    return floor(p*double(stepsPerPhase));//(floor(p*double(stepsPerPhase) + .500001))-1;
}

void TreeTask::setCostSpecs(int fromTime,
                        int toTime, int T,
                        const arr& _target,
                        double _prec){
    if(&_target) target = _target; else target = {0.};
    if(fromTime<0) fromTime=0;
    CHECK(toTime>=fromTime,"");
    prec.resize(T).setZero();
    for(int local = 0; local < int(branch.local_to_global.size())-2; ++local) // clarify magic number
    {
        if(local >= fromTime && local < toTime)
        {
            auto global = branch.local_to_global[local];
            prec(global) = _prec;
        }
    }
}

void TreeTask::setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T, const arr& _target, double _prec, const Branch& branch_time_spec){
    if(stepsPerPhase<0) stepsPerPhase=T;
    uint maxStepOnBranch = *std::max_element(branch_time_spec.local_to_global.begin(), branch_time_spec.local_to_global.end()) * stepsPerPhase;
    if(phaseToStep(toTime, stepsPerPhase)>maxStepOnBranch){
        LOG(-1) <<"beyond the time!: endTime=" <<toTime <<" phases=" <<double(T)/stepsPerPhase;
    }

    CHECK(&branch_time_spec, "case without branch not handled yet!");
    branch = Branch::computeMicroStepBranch(branch_time_spec, stepsPerPhase);

    int tFrom = (fromTime<0.?0              :phaseToStep(fromTime, stepsPerPhase));
    int tTo =   (toTime  <0.?maxStepOnBranch:phaseToStep(toTime, stepsPerPhase));
    if(tTo<0) tTo=0;
    if(tFrom>tTo && tFrom-tTo<=(int)map->order) tFrom=tTo;

    setCostSpecs(tFrom, tTo, T, _target, _prec);
}

//==============KOMOTree==============================================

KOMOTree::KOMOTree()
  : KOMO()
  , komo_tree_problem(*this)
{

}

void KOMOTree::run(){
  mlr::KinematicWorld::setJointStateCount=0;
  mlr::timerStart();
  CHECK(T,"");
  if(opt) delete opt;

  auto cp = Conv_KOMO_Tree_ConstrainedProblem(komo_tree_problem);
  opt = new OptConstrained(x, dual, cp);
  opt->run();

  if(verbose>0){
    cout <<"** optimization time=" <<mlr::timerRead()
        <<" setJointStateCount=" <<mlr::KinematicWorld::setJointStateCount <<endl;
  }
  if(verbose>1) cout <<getReport(false);
}

bool KOMOTree::checkGradients(){
  CHECK(T,"");

  return checkJacobianCP(Convert(komo_problem), x, 1e-4);
}

Task *KOMOTree::setTask(double startTime, double endTime, TaskMap *map, ObjectiveType type, const arr& target, double prec, uint order){
  return setTreeTask(startTime, endTime, Branch::linearTrajectory(T/stepsPerPhase+1), map, type, target, prec, order);
}

TreeTask* KOMOTree::addTreeTask(const char* name, TaskMap *m, const ObjectiveType& termType){
  TreeTask *t = new TreeTask(m, termType);
  t->name=name;
  tasks.append(t);
  return t;
}

TreeTask *KOMOTree::setTreeTask(double startTime, double endTime, const Branch& branch, TaskMap *map, ObjectiveType type, const arr& target, double prec, uint order){
  CHECK(k_order>=order,"");
  map->order = order;
  TreeTask *task = addTreeTask(map->shortTag(world), map, type);
  task->setCostSpecs(startTime, endTime, stepsPerPhase, T, target, prec, branch);
  return task;
}

bool KOMOTree::displayTrajectory(double delay, bool watch){
  const char* tag = "KOMO planned trajectory";

  // retrieve branches
  std::set<mp::Branch> branches;
  for(auto t: this->tasks)
  {
    auto tree_task = dynamic_cast< TreeTask* >(t); // hack ?
    branches.insert(tree_task->branch);
  }

  // name viewer
  auto name = [](uint m, uint leaf_id)
  {
    std::stringstream ss;
    ss << "KOMO display - branch " << m << " - leaf " << leaf_id;
    return ss.str();
  };

  // create viewer for each branch consecutively
  uint branch_number = 0;
  for(const auto branch : branches)
  {
    auto gl = std::make_shared< OpenGL >(name(branch_number, branch.leaf_id).c_str());
    gl->camera.setDefault();
    auto T = branch.local_to_global.size();
    for(uint local = k_order; local < T; ++local)
    {
      auto global = branch.local_to_global[local];
      gl->clear();
      gl->add(glStandardScene, 0);
      gl->addDrawer(configurations(global));
      if(delay<0.){
        if(delay<-10.) FILE("z.graph") <<*configurations(global);
        gl->watch(STRING(tag <<" (time " <<std::setw(3) << local <<'/' <<T <<')').p);
      }else{
        gl->update(STRING(tag <<" (time " <<std::setw(3) << local <<'/' <<T <<')').p);
        if(delay) mlr::wait(delay);
      }
    }
    ++branch_number;
  }

  return true;
}

//-- komo problem

void KOMOTree::Conv_Tree_KOMO_Problem::getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes){
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet: use komo.reset()");
  variableDimensions.resize(komo.T);
  for(uint t=0;t<komo.T;t++) variableDimensions(t) = komo.configurations(t+komo.k_order)->getJointStateDimension();

  featureTimes.clear();
  featureTypes.clear();
  for(uint t=0;t<komo.T;t++){
    for(Task *task: komo.tasks) if(task->prec.N>t && task->prec(t)){
      //      CHECK(task->prec.N<=MP.T,"");
      auto tree_task = dynamic_cast< TreeTask* >(task); // hack ?
      const auto local_t = tree_task->to_local_t(t);
      WorldL configurations = getConfigurations(tree_task, local_t);
      uint m = task->map->dim_phi(configurations, t); //dimensionality of this task
      featureTimes.append(consts<uint>(tree_task->to_global_t(local_t), m));
      featureTypes.append(consts<ObjectiveType>(tree_task->type, m));
    }
  }
  dimPhi = featureTimes.N;
}

WorldL KOMOTree::Conv_Tree_KOMO_Problem::getConfigurations(TreeTask* task, uint local_t) const
{
  WorldL configurations(komo.k_order+1);
  for(uint k = 0; k<=komo.k_order; ++k)
  {
    configurations(k) = komo.configurations(task->to_global_t(local_t+k));
  }
  return configurations;
}

void KOMOTree::Conv_Tree_KOMO_Problem::phi(arr& phi, arrA& J, arrA& H, ObjectiveTypeA& tt, const arr& x){
  //-- set the trajectory
  komo.set_x(x);

  CHECK(dimPhi,"getStructure must be called first");
  phi.resize(dimPhi);
  if(&tt) tt.resize(dimPhi);
  if(&J)
  {
    J.resize(dimPhi);
    for(uint i=0;i<dimPhi;i++) J(i) = arr(x.N);
  }

  arr y, Jy;
  uint M=0;
  for(uint t=0;t<komo.T;t++){
    for(Task *task: komo.tasks) if(task->prec.N>t && task->prec(t)){
      //TODO: sightly more efficient: pass only the configurations that correspond to the map->order
      auto tree_task = dynamic_cast< TreeTask* >(task); // hack ?
      const auto local_t = tree_task->to_local_t(t);
      WorldL configurations = getConfigurations(tree_task, local_t);
      tree_task->map->phi(y, (&J?Jy:NoArr), configurations, komo.tau, t);
      if(!y.N) continue;
      if(absMax(y)>1e10) MLR_MSG("WARNING y=" <<y);

      //linear transform (target shift)
      if(task->target.N==1) y -= task->target.elem(0);
      else if(task->target.nd==1) y -= task->target;
      else if(task->target.nd==2) y -= task->target[t];
      y *= tree_task->branch.p * sqrt(task->prec(t));

      //write into phi and J
      const auto & qN = komo.configurations(0)->q.N;
      phi.setVectorBlock(y, M);
      if(&J){
        Jy *= tree_task->branch.p * sqrt(task->prec(t));
        for(uint i=0;i<y.N;i++)
        {
          for(uint k=0; k<=komo.k_order; ++k)
          {
            for(uint j=0; j<qN; ++j)
            {
              auto col_in_jacobian = qN * tree_task->to_global_t(local_t+k)+j;
              col_in_jacobian -= get_k() * qN; // shift back to compensate for the prefix

              if(col_in_jacobian < x.N) // case col < 0 implicitely handled by overflow
              {
                J(M+i)(col_in_jacobian)=Jy(i,qN * k + j);
              }
            }
          }
        }
      }
      if(&tt) for(uint i=0;i<y.N;i++) tt(M+i) = task->type;

      //counter for features phi
      M += y.N;
    }
  }

  CHECK_EQ(M, dimPhi, "");
  komo.featureValues = ARRAY<arr>(phi);
  if(&tt) komo.featureTypes = ARRAY<ObjectiveTypeA>(tt);
}

//-- converters
Conv_KOMO_Tree_ConstrainedProblem::Conv_KOMO_Tree_ConstrainedProblem(KOMO_Problem& P) : KOMO(P){
  KOMO.getStructure(variableDimensions, featureTimes, featureTypes);
  varDimIntegral = integral(variableDimensions);
}

void Conv_KOMO_Tree_ConstrainedProblem::phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x){
  KOMO.phi(phi, (&J?J_KOMO:NoArrA), (&H?H_KOMO:NoArrA), tt, x);

  //-- construct J
  if(&J){
    //J = mlr::Array<double>(phi.N, x.N);
    J = zeros(phi.N, x.N);

    //loop over features
    for(uint i=0; i<phi.N; i++) {
      arr& Ji = J_KOMO(i);
      CHECK(Ji.N<=J.d1,"");
      memmove(&J(i,0), Ji.p, Ji.sizeT*Ji.N);
    }
  }

  if(&H){
    bool hasFterm = false;
    if(&tt) hasFterm = (tt.findValue(OT_f) != -1);
    if(hasFterm){
      CHECK(H_KOMO.N, "this problem has f-terms -- I need a Hessian!");
      NIY
    }else{
      H.clear();
    }
  }
}

}
