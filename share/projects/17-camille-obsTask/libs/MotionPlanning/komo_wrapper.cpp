#include <komo_wrapper.h>
#include <Kin/switch.h>
#include <Kin/TM_angVel.h>
#include <Kin/TM_default.h>
#include <Kin/TM_gravity.h>
#include <Kin/TM_InsideBox.h>
#include <Kin/TM_AboveBox.h>

using namespace rai;

double shapeSize(const KinematicWorld& K, const char* name, uint i=2);

namespace mp
{
void W::reset(const std::list<Vars>& branches, double initNoise)
{
  if(!komo_->configurations.N) setupConfigurations(branches);
  komo_->x = komo_->getPath_decisionVariable();
  komo_->dual.clear();
  komo_->featureValues.clear();
  komo_->featureTypes.clear();
  komo_->komo_problem.clear();
  komo_->dense_problem.clear();
  if(initNoise>0.)
    rndGauss(komo_->x, initNoise, true); //don't initialize at a singular config
  if(komo_->splineB.N) {
    komo_->z = pseudoInverse(komo_->splineB) * komo_->x;
  }
}

void W::setupConfigurations(const std::list<Vars>& branches)
{
  //IMPORTANT: The configurations need to include the k prefix configurations!
  //Therefore configurations(0) is for time=-k and configurations(k+t) is for time=t
  CHECK(!komo_->configurations.N,"why setup again?");
//    listDelete(configurations);

  computeMeshNormals(komo_->world.frames, true);

  komo_->configurations.resize(komo_->T + komo_->k_order);
  for(auto s = 0; s < komo_->configurations.d0; ++s) komo_->configurations(s) = new KinematicWorld();

  //komo_->configurations.append(new KinematicWorld())->copy(komo_->world, true);
  komo_->configurations(0)->copy(komo_->world, true);
  komo_->configurations(0)->setTimes(komo_->tau); //(-tau*k_order);
  komo_->configurations(0)->calc_q();
  komo_->configurations(0)->checkConsistency();
  for(KinematicSwitch *sw:komo_->switches) {
    if(sw->timeOfApplication+(int)komo_->k_order<=0) {
      sw->apply(*komo_->configurations(0));
    }
  }

  komo_->configurations(1)->copy(*komo_->configurations(0), true);
  komo_->configurations(2)->copy(*komo_->configurations(1), true);
  std::vector<int> visited(komo_->T+komo_->k_order, 0);
  //visited[0] = 1; visited[1] = 1; visited[2] = 1;

  //int i = 0;
  for(const auto& branch: branches)
  {
    //std::cout << "branch " << i << std::endl;
    //int ns = 0;
    for(uint s=1; s<branch.order0.d0; s++) {
      //for(uint s=1; s<komo_->k_order+komo_->T; s++) {
      auto s_global = branch.order0(s,0);
      auto s_m_1_global = branch.order0(s-1, 0);

      if(visited[s_global + komo_->k_order]) continue;
      visited[s_global + komo_->k_order] = 1;

      komo_->configurations(s_global + komo_->k_order)->copy(*komo_->configurations(s_m_1_global + komo_->k_order), true);
      rai::KinematicWorld& K = *komo_->configurations(s_global + komo_->k_order);
      CHECK(K.frames.d0>0, "Copied wrong element");
      K.setTimes(komo_->tau); //(tau*(int(s)-int(k_order)));
      K.checkConsistency();
      //apply potential graph switches
      for(KinematicSwitch *sw:komo_->switches) {
        if(sw->timeOfApplication == s_global) {
          sw->apply(K);
          //ns++;
          //std::cout << "  apply switch at" << s_global << std::endl;
        }
      }
      //apply potential PERSISTENT flags
      for(Flag *fl:komo_->flags) {
        if(fl->persist && fl->stepOfApplication == s_global) {
          fl->apply(K);
        }
      }
      K.calc_q();
      K.checkConsistency();
      //    {
      //      cout <<"CONFIGURATION s-k_order=" <<int(s)-k_order <<endl;
      //      K.glAnimate();
      //      rai::wait();
      ////      K.glClose();
      //    }
    }
    //++i;
  }

  //now apply NON-PERSISTENT flags
  //for(uint s=1; s<komo_->k_order+komo_->T; s++) {
  for(const auto& branch: branches)
  for(uint s=1; s<branch.order0.d0; s++) {
    auto s_global = branch.order0(s,0);
    for(Flag *fl:komo_->flags) {
      if(!fl->persist && fl->stepOfApplication==s_global) {
        fl->apply(*komo_->configurations(s_global+komo_->k_order));
      }
    }
  }
}

void W::addObjective(double start, double end, const mp::Vars & branch, Feature* map, ObjectiveType type, const arr& target, double scale, int order, int deltaFromStep, int deltaToStep)
{
  auto obj = komo_->addObjective(-123., -123., map, type, target, scale, order, deltaFromStep, deltaToStep);
  obj->vars = branch.getVars(start, end, order);
}

void W::addSwitch_stable(double time, const mp::Vars & branch, const char* from, const char* to)
{
  auto sw = new KinematicSwitch(SW_effJoint, JT_free, from, to, world_);
  sw->timeOfApplication = branch.getStep(time);
  komo_->switches.append(sw);

  addObjective(time, -1.0, branch, new TM_ZeroQVel(world_, to), OT_eq, NoArr, 3e1, 1, +1, -1);
  addObjective(time, -1.0, branch, new TM_LinAngVel(world_, to), OT_eq, NoArr, 1e1, 2, +0, +1);
}

void W::addSwitch_stableOn(double time, const mp::Vars & branch, const char* from, const char* to)
{
  Transformation rel{0};
  rel.pos.set(0,0, .5*(shapeSize(world_, from) + shapeSize(world_, to)));

  auto sw = new KinematicSwitch(SW_effJoint, JT_transXYPhi, from, to, world_, SWInit_zero, 0, rel);
  sw->timeOfApplication = branch.getStep(time);
  komo_->switches.append(sw);

  addObjective(time, -1.0, branch, new TM_ZeroQVel(world_, to), OT_eq, NoArr, 3e1, 1, +1, -1);
  addObjective(time, -1.0, branch, new TM_LinAngVel(world_, to), OT_eq, NoArr, 1e1, 2, +0, +1);
}
}
