#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Motion/komo.h>
#include <Motion/motion.h>
#include <Optim/convert.h>

//===========================================================================

void TEST(Executable){
  const char* specsfile="specs.g";
  const char* outprefix="z";
  if(mlr::argc>1) specsfile=mlr::argv[1];
  if(mlr::argc>2) outprefix=mlr::argv[2];

  Graph specs(specsfile);
  KOMO komo(specs);
  komo.reset();
  komo.MP->reportFeatures();

  int repeats=specs.get<double>("repeats", -1.);
  for(int r=0;repeats<0. || r<repeats; r++){
    if(r!=0) komo.init(Graph(specsfile)); //reload the specs
//    komo.checkGradients();
    komo.run(); //reoptimize

    //-- output results:
    FILE(STRING(outprefix<<".costs.g")) <<komo.getReport(); //cost details
    mlr::KinematicWorld pose=komo.MP->world;
    if(komo.MP->T){ //generate all kinematic switches
      pose=*komo.MP->configurations.last(); //take the last pose
      for(mlr::KinematicSwitch *sw:komo.MP->switches){ //apply all switches that are 'after last'
        if(sw->timeOfApplication >= komo.MP->T+1) sw->apply(pose);
      }
    }
    FILE(STRING(outprefix<<".pose.g")) <<"ChDir = '../../../data/pr2_model/'\n\n" <<pose;

    komo.displayTrajectory();  //play trajectory
  }
}

//===========================================================================

void TEST(cInterface){
  KOMO komo(Graph("specs2.g"));
//  cout <<komo.specs <<endl;

  komo.setFact("(EqualZero posDiff endeff target)");
  komo.setFact("(LowerEqualZero collisionIneq){ margin=0.05 scale=.1 }");
  komo.reset();
  komo.MP->reportFeatures();
  komo.run(); //reoptimize
  komo.displayTrajectory();  //play trajectory
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  cout <<mlr::String(FILE("USAGE"));

  testExecutable();
//  testcInterface();

  return 0;
}




