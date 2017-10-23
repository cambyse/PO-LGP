#include <Core/thread.h>
#include <Kin/kin.h>
#include <Algo/spline.h>

//===============================================================================

struct PerceptSimple;
typedef mlr::Array<PerceptSimple*> PerceptSimpleL;

//===============================================================================

struct KinSim : Thread{
  mlr::KinematicWorld K;
  uint pathRev=0, switchesRev=0;
  mlr::Spline reference;
  double phase=0.;
  double dt;
  ofstream log;

  Access<arr> path;
  Access<arr> currentQ;
  Access<mlr::Transformation> robotBase;
  Access<arr> nextQ;
  Access<StringA> switches;
  Access<mlr::KinematicWorld> world;
  Access<double> timeToGo;
  Access<PerceptSimpleL> percepts_input;


  KinSim(double dt=.01);
  ~KinSim(){
    log.close();
  }

  void open(){}
  void close(){}

  void step();

};
