#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Motion/komo.h>
#include <Motion/motion.h>
#include <Optim/opt-convert.h>

//===========================================================================

void TEST(Specs){
  const char* specsfile="specs.g";
  const char* outprefix="z";
  if(mlr::argc>1) specsfile=mlr::argv[1];
  if(mlr::argc>2) outprefix=mlr::argv[2];

  Graph specs(specsfile);
  KOMO komo(specs);

  int repeats=specs.get<double>("repeats", -1.);
  for(int r=0;repeats<0. || r<repeats; r++){
    komo.init(Graph(specsfile));
//    komo.checkGradients();
    komo.run();
    FILE(STRING(outprefix<<".costs.g")) <<komo.getReport();
    ors::KinematicWorld pose=komo.MP->world;
    if(komo.MP->T){ //generate all kinematic switches
      pose=*komo.MPF->configurations.last();
      for(ors::KinematicSwitch *sw:komo.MP->switches){
        if(sw->timeOfApplication>komo.MP->T) sw->apply(pose);
      }
    }
    FILE(STRING(outprefix<<".pose.g")) <<"ChDir = '../../../data/pr2_model/'\n\n" <<pose;
    komo.MP->costReport(true);
    komo.displayTrajectory();
  }
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  cout <<mlr::String(FILE("USAGE"));

  testSpecs();

  return 0;
}



