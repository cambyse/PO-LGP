#include <Core/array.h>
#include <Ors/ors.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>
#include <Roopi/roopi.h>
#include <Control/TaskControlThread.h>
#include <Ors/orsviewer.h>
#include <Algo/ann.h>

#include "../src/surfaceModelModule.h"
#include "../src/objectGenerator.h"
#include "../src/surfaceVisualizationModule.h"


void surfaceExploration_1() {
  ors::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors"));

  Object o(world);
  o.generateObject("b", 0.16, 0.16, 0.1, 0.55, -0.1, 0.55); //0.5 for x
  Object ob(world);
  ob.generateObject("trueShape", 0.17, 0.17, 0.12, 0.55, -0.1, 0.55, false);

  Roopi R(world);

  CtrlTask* contactSimulator = R.createCtrlTask("contactSimulator", new TaskMap_Proxy(allPTMT, {0u}, 0.01));
  R.modifyCtrlTaskGains(contactSimulator, 70.0, 10.0);
  R.modifyCtrlTaskReference(contactSimulator, ARR(0.0));
  R.activateCtrlTask(contactSimulator);

  CtrlTask* posEndeff = R.createCtrlTask("posEndeff", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(posEndeff, diag(ARR(10.0,10.0,10.0)), diag(ARR(5.0,5.0,5.0)));
  R.modifyCtrlTaskReference(posEndeff, ARR(0.5,-0.1,0.75));

  CtrlTask* oriEndeff = R.createCtrlTask("oriEndeff", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", ors::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(oriEndeff, 20.0, 5.0);
  R.modifyCtrlTaskReference(oriEndeff, ARR(0.0,0.0,-1.0));

  CtrlTask* limits = R.createCtrlTask("limits", new TaskMap_qLimits);
  R.modifyCtrlTaskGains(limits, 10.0, 5.0);
  R.modifyCtrlTaskReference(limits, ARR(0.0));

  R.activateCtrlTask(posEndeff);
  R.activateCtrlTask(oriEndeff);
  R.activateCtrlTask(limits);
  R.releasePosition();
  mlr::wait(5.0);
  R.holdPosition();
  R.activateCtrlTask(contactSimulator);

  CtrlTask* moveTowardsSurface = R.createCtrlTask("moveTowardsSurface", new TaskMap_Default(pos1DTMT, R.tcm()->modelWorld.get()(), "endeffR", ors::Vector(0.0,0.0,-1.0)));
  R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(15.0));
  R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.1));

  R.modifyCtrlTaskGains(posEndeff, diag(ARR(30.0,30.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  R.activateCtrlTask(posEndeff);
  R.activateCtrlTask(oriEndeff);
  R.activateCtrlTask(moveTowardsSurface);
  R.activateCtrlTask(limits);
  R.releasePosition();

  //world.watch(true);

  SurfaceModelModule surfaceModel(R);

  SurfaceVisualizationModule surfaceVisualization(R);
  surfaceModel.threadLoop();

  PathAndGradientVisualizationModule pgvm(R);
  pgvm.threadLoop();

  mlr::wait(2.0);

  TaskReferenceInterpolAct* th = R.createTaskReferenceInterpolAct("interpol", posEndeff);
  arr pos = R.getTaskValue(posEndeff);
  pos(1) -= 0.1;
  pos(0) += 0.1;
  R.interpolateToReference(th, 5.0, pos);
  R.waitForFinishedTaskReferenceInterpolAct(th);

  R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(5.0));
  R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.05));

  surfaceVisualization.threadLoop();

  Access<uint> maxIt(NULL, "maxIt");
  maxIt.set()() = 1000000;

  TaskMap_Default eOri(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", ors::Vector(1.0,0.0,0.0));

  for(uint i = 0; i < maxIt.get()(); i++) {

    arr actPos = R.getTaskValue(posEndeff);

    arr gradGP, gradV;
    surfaceModel.gpSurface.readAccess();
    gradGP = surfaceModel.gpSurface().gradient(actPos);
    gradV = surfaceModel.gpSurface().gradientVariance(actPos);
    surfaceModel.gpSurface.deAccess();
    gradV.reshapeFlat();

    arr gradVT = gradV - (~gradGP*gradV).first()*gradGP/length(gradGP)/length(gradGP);
    gradVT = gradVT/length(gradVT);

    arr newPos = actPos + 0.08*gradVT;

    if(newPos(2) < 0.55) {
      newPos(2) = 0.55;// + rnd.uni()*0.08;
      if(fabs(scalarProduct(gradVT,ARR(0.0,0.0,-1.0))-1.0) < 0.1) {
        cout << "blub" << endl;
        gradVT(2) = -gradVT(2);
        newPos = actPos + 0.08*gradVT;
      }
    }

    R.tcm()->ctrlTasks.writeAccess();
    dynamic_cast<TaskMap_Default&>(moveTowardsSurface->map).ivec = ors::Vector(-gradGP/length(gradGP));
    R.tcm()->ctrlTasks.deAccess();
    //R.modifyCtrlTaskReference(oriEndeff, -gradGP/length(gradGP));

    ors::Vector n = ors::Vector(-gradGP/length(gradGP));

    arr V = n.generateOrthonormalSystemMatrix();

    arr Lambda = diag(ARR(0.0,10.0,10.0));
    arr A = V*Lambda*~V;
    arr B = V*Lambda/1.0*~V;
    R.modifyCtrlTaskGains(posEndeff, A, B);

    bool mo = true;
    if(surfaceModel.gpSurface.get()->Y.last() > 0.0) {
      mo = false;
    }

    if(mo) {
      R.interpolateToReference(th, 0.3, newPos, actPos); //TODO 0.5, 0.1, 0.1 IDEE: schrittweite und zeit adaptiv wählen?
      R.waitForFinishedTaskReferenceInterpolAct(th);
    } else {
      cout << "out" << endl;
      mlr::wait(0.5);
      //R.holdPosition();
    }
  }
  R.holdPosition();
  surfaceModel.threadStop();
  //surfaceVisualization.threadStop();
  //pgvm.threadStop();
  mlr::wait(1000.0);
}

void changeColorA(void*){  orsDrawColors=false; glColor(.5, 1., .5, .7); }
void changeColor2A(void*){  orsDrawColors=true; orsDrawAlpha=1.; }

void showClip() {
  ors::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors"));

  Object o(world);
  o.generateObject("b", 0.16, 0.16, 0.1, 0.55, -0.1, 0.55); //0.5 for x
  Object ob(world);
  ob.generateObject("trueShape", 0.17, 0.17, 0.12, 0.55, -0.1, 0.55, false);
  world.gl().add(changeColor2A);
  world.gl().add(glDrawPlot, &plotModule);

  arr X;
  double x = 0.4, y = 0.4;
  X.append(~ARR(0.55+x,-0.1+y,0.55));
  X.append(~ARR(0.55-x,-0.1+y,0.55));
  X.append(~ARR(0.55-x,-0.1-y,0.55));
  X.append(~ARR(0.55+x,-0.1-y,0.55));

  ors::Mesh me;
  me.V = X;
  me.T.append(~ARRAY<uint>(0,1,3));
  me.T.append(~ARRAY<uint>(1,2,3));
  world.gl().add(me);

  world.watch(true);
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  //surfaceExploration_1();
  showClip();

  return 0;
}
