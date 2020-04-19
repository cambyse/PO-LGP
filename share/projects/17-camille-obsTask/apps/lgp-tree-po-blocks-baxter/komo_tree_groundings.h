#pragma one

#include <graph_planner.h>
#include <komo_factory.h>

#include <Kin/switch.h>
#include <Kin/TM_InsideBox.h>
#include <Kin/TM_angVel.h>
#include <Kin/TM_default.h>
#include <Kin/TM_gravity.h>

double shapeSize(const rai::KinematicWorld& K, const char* name, uint i=2);

//komo wrapper
struct W
{
  W(KOMO_ext * komo)
    : komo_(komo)
    , world_(komo->world)
  {

  }

  void addObjective(const mp::Vars & vars, Feature* map, ObjectiveType type=OT_sos, const arr& target=NoArr, double scale=-1., int order=-1, int deltaFromStep=0, int deltaToStep=0)
  {
    auto obj = komo_->addObjective(-123., -123., map, type, target, scale, order, deltaFromStep, deltaToStep);
    obj->vars = vars[order];
  }

  void addSwitch_stable(const mp::Vars & vars, const char* from, const char* to)
  {
    using namespace rai;

    auto sw = new KinematicSwitch(SW_effJoint, JT_free, from, to, world_);
    sw->timeOfApplication = vars.order0(-1, 0);
    komo_->switches.append(sw);

    addObjective(vars, new TM_ZeroQVel(world_, to), OT_eq, NoArr, 3e1, 1, +1, -1);
    addObjective(vars, new TM_LinAngVel(world_, to), OT_eq, NoArr, 1e1, 2, +0, +1);
  }

  void addSwitch_stableOn(const mp::Vars & vars, const char* from, const char* to)
  {
    using namespace rai;
    Transformation rel = 0;
    rel.pos.set(0,0, .5*(shapeSize(world_, from) + shapeSize(world_, to)));
    auto sw = new KinematicSwitch(SW_effJoint, JT_transXYPhi, from, to, world_, SWInit_zero, 0, rel);
    sw->timeOfApplication = vars.order0(-1, 0);
    komo_->switches.append(sw);

    addObjective(vars, new TM_ZeroQVel(world_, to), OT_eq, NoArr, 3e1, 1, +1, -1);
    addObjective(vars, new TM_LinAngVel(world_, to), OT_eq, NoArr, 1e1, 2, +0, +1);
  }

  KOMO_ext * komo_;
  const rai::KinematicWorld & world_;
};

//
void groundTreePickUp( const mp::Vars& vars, const arr& scales, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

//  double duration=1.0;

//  //
//  const double t_start = phase;
//  const double t_end =   phase + duration;
//  //
//  komo->setAlign( t_end - 0.3, t_end, "baxterR", ARR( 1.0, 0.0, 0.0 ), nullptr, ARR( 0.0, 0.0, -1.0 ) );
//  komo->setGrasp( t_end, "baxterR", facts[0].c_str(), 0 );

  if( verbose > 0 )
  {
    std::cout << " pick up " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundTreeUnStack( const mp::Vars& vars, const arr& scales, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose )
{
  using namespace rai;

  groundPrefixIfNeeded( komo, verbose );

  // switch
  const auto& eff = "baxterR";
  const auto& object = facts[0].c_str();
  W(komo).addSwitch_stable(vars, eff, object);

  // costs
  W(komo).addObjective(vars, new TM_InsideBox(komo->world, eff, NoVector, object), OT_ineq, NoArr, 1e1, 0);

  //  double duration=1.0;

//  //
//  const double t_start = phase;
//  const double t_end =   phase + duration;
//  //
//  komo->setAlign( t_end - 0.3, t_end, "baxterR", ARR( 1.0, 0.0, 0.0 ), nullptr, ARR( 0.0, 0.0, -1.0 ) );
//  komo->setGrasp( t_end, "baxterR", facts[0].c_str(), 0 );

  //  // lift after pick
  //komo->setTask(t_end+.10, t_end+.10, new TM_Default(TMT_pos, komo->world,"baxterR"), OT_sos, {0.,0.,+.1}, 1e1, 1); //move up

  if( verbose > 0 )
  {
    std::cout << " unstack " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundTreePutDown( const mp::Vars& vars, const arr& scales, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose )
{
  using namespace rai;

  groundPrefixIfNeeded( komo, verbose );

  const auto& object = facts[0].c_str();
  const auto& place = facts[1].c_str();

  W(komo).addSwitch_stableOn(vars, place, object);

//  komo->setPlace( t_end, "baxterR", facts[0].c_str(), facts[1].c_str(), verbose );

  if( verbose > 0 )
  {
    std::cout << " put down " << facts[0] << " at " << facts[1] << std::endl;
  }
}

void groundTreeCheck( const mp::Vars& vars, const arr& scales, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

//  double duration=1.0;

//  //
//  const double t_start = phase + 0.5;
//  const double t_end =   phase + duration;
//  //
//  komo->addObjective( t_start, t_end, new LimitsConstraint(0.05), OT_ineq, NoArr ); // avoid self collision with baxter
//  //komo->setTask( t_start, t_end, new TM_Transition(komo->world), OT_sos, NoArr, 1e-1, 2);

  auto task1 = komo->addObjective( -123., -123., new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0.01, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_sos, NoArr, 1e2 ); // slight offset (0.01) to break symmetry and avoid quternion normalization problem
  task1->vars = vars.order2;

  if( verbose > 0 )
  {
    std::cout << " check " << facts[0] << std::endl;
  }
}

void groundTreeStack( const mp::Vars& vars, const arr& scales, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

//  double duration=1.0;

//  //
//  const double t_start = phase;
//  const double t_end =   phase + duration;
//  //

//  komo->setPlace( t_end, "baxterR", facts[0].c_str(), facts[1].c_str(), verbose );

  if( verbose > 0 )
  {
    std::cout << " stack " << facts[0] << " on " << facts[1] << std::endl;
  }
}
