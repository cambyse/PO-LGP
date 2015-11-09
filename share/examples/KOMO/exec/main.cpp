#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Motion/komo.h>
#include <Motion/motion.h>
#include <Optim/opt-convert.h>

//===========================================================================

void TEST(Executable){
  const char* specsfile="specs.g";
  const char* outprefix="z";
  if(mlr::argc>1) specsfile=mlr::argv[1];
  if(mlr::argc>2) outprefix=mlr::argv[2];

  Graph specs(specsfile);
  KOMO komo(specs);

  int repeats=specs.get<double>("repeats", -1.);
  for(int r=0;repeats<0. || r<repeats; r++){
    if(r!=0) komo.init(Graph(specsfile)); //reload the specs
//    komo.checkGradients();
    komo.run(); //reoptimize

    //-- output results:
    FILE(STRING(outprefix<<".costs.g")) <<komo.getReport(); //cost details
    ors::KinematicWorld pose=komo.MP->world;
    if(komo.MP->T){ //generate all kinematic switches
      pose=*komo.MPF->configurations.last(); //take the last pose
      for(ors::KinematicSwitch *sw:komo.MP->switches){ //apply all switches that are 'after last'
        if(sw->timeOfApplication >= komo.MP->T+1) sw->apply(pose);
      }
    }
    FILE(STRING(outprefix<<".pose.g")) <<"ChDir = '../../../data/pr2_model/'\n\n" <<pose;

    komo.MP->costReport(true); //show cost plot
    komo.displayTrajectory();  //play trajectory
  }
}

//===========================================================================

void TEST(cInterface){


}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  cout <<mlr::String(FILE("USAGE"));

  testExecutable();

  return 0;
}




