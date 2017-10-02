#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>
#include <stdio.h>

#include <Kin/frame.h>
#include <Kin/kin_swift.h>

#include <Geo/pairCollide.h>
#include <Kin/proxy.h>

#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Kin/kinViewer.h>

void illustrate(){

  OpenGL gl("Red Ball Scenes", 1600, 800);

  uint N=11;
  mlr::Array<mlr::KinematicWorld> K(N);
  for(uint i=0;i<N;i++){
    mlr::String str = STRINGF("p%02i.g", i+1);
    K(i).init(str);
    K(i).orsDrawMarkers=false;
    gl.addView(i, glStandardScene, NULL);
    gl.addSubView(i, K(i));
    gl.views(i).camera.setDefault();
    gl.views(i).camera.setPosition(3., 0., 3.);
    gl.views(i).camera.focus(0., 0., 1.);
    gl.views(i).camera.upright();
    gl.views(i).text = str;
  }
  gl.setSubViewTiles(4,3);

  gl.watch();
}

void analyzeSupport(){
  uint N=11;
  mlr::Array<mlr::KinematicWorld> K(N);
  OpenGL gl;

  ofstream fil("z.supportAna");

  StringA contactName = {"vertex", "edge", "plane"};

  for(uint i=0;i<N;i++){
    mlr::String str = STRINGF("p%02i.g", i+1);
    mlr::KinematicWorld K(str);
    fil <<"\n### PROBLEM " <<i <<"  " <<str <<endl;

    for(mlr::Frame *f:K.frames) if(f->shape){
      f->shape->cont=true;
      f->shape->mesh().C.append(.3);
    }
    K.swift().setCutoff(.05);
    K.swift().initActivations(K, 0);

    K.stepSwift();
    K.reportProxies();

    for(mlr::Proxy* p:K.proxies){
      mlr::Frame *a = K.frames(p->a);
      mlr::Frame *b = K.frames(p->b);
      if(a->shape->type()!=mlr::ST_ssBox || b->shape->type()!=mlr::ST_ssBox) continue;
      PairCollision coll(a->shape->sscCore(),
                         b->shape->sscCore(),
                         a->X,
                         b->X);

      coll.marginAnalysis(.001);

      cout <<"PROXY " <<a->name <<"--" <<b->name
             <<"\n  " <<coll;

      if(coll.distance<a->shape->size(3)+b->shape->size(3)+.001){
        fil <<"  " <<a->name <<'-' <<b->name <<" : " <<contactName(coll.eig1.d0) <<'-' <<contactName(coll.eig2.d0) <<endl;
      }

      gl.add(glStandardLight);
      gl.add(coll);
      gl.add(K);
//      gl.watch();
      gl.update();
      gl.clear();

    }

//    K.watch(true);
  }
}

//===========================================================================

#define collisionsOff(x) komo.world.swift().deactivate(komo.world.getFrameByName(x))
#define collisionsOn(x) komo.world.swift().activate(komo.world.getFrameByName(x))

void init(KOMO& komo, uint trial, mlr::KinematicWorld& K, mlr::KinematicWorld& Kfin, double phases=4.){
  K.init(STRINGF("p%02i.g", trial));
  Kfin.init(STRINGF("p%02i.g", trial+1));

  komo.setModel(K, true);
  komo.setPathOpt(phases, 20, 5.);

  komo.displayCamera().setPosition(-5.,-1.,2.);
  komo.displayCamera().focus(0,0,1.);
  komo.displayCamera().upright();

  // explicitly active certain collision computations (by SWIFT)
//  komo.world.swift().deactivate(komo.world.frames); //deactivate all
//  collisionsOff("table");
}

//===========================================================================

void optimize(KOMO& komo){
  komo.reset();
  komo.run();
//  komo.checkGradients();

  cout <<komo.getReport(true);

  while(komo.displayTrajectory(.1, true));

  renderConfigurations(komo.configurations, "vid/z.path.", -2, 600, 600, &komo.gl->camera);
}

mlr::Transformation relPose(const mlr::KinematicWorld& K, const char* obj1, const char* obj2){
  return K.getFrameByName(obj1)->X / K.getFrameByName(obj2)->X;
}

//===========================================================================

void trial3(){
  KOMO komo;
  mlr::KinematicWorld K, Kfin;
  init(komo, 3, K, Kfin);
  collisionsOn("yellow");
  collisionsOn("blue");
  collisionsOn("white");
  collisionsOff("red");
  collisionsOff("black");
  collisionsOff("table");


//  komo.setGrasp(1., "humanR", "red", 0, .8);
  komo.setGrasp(1, "humanL", "blue", 0, .8);
  komo.setDrop(1.2, "yellow", NULL, "table");
  komo.setPlace(2., NULL, "yellow", "table");

  komo.setPlaceFixed(2.5, "humanR", "blue", "white", relPose(Kfin,"blue","white"));

  komo.setTask(1.2, 2., new TaskMap_Proxy(allPTMT, uintA(), .01), OT_sumOfSqr, NoArr, 1e2);

  optimize(komo);
}

//===========================================================================

void trial8(){
  KOMO komo;
  mlr::KinematicWorld K, Kfin;
  init(komo, 8, K, Kfin);
  collisionsOn("yellow");
  collisionsOn("blue");
  collisionsOn("white");
  collisionsOff("red");
  collisionsOff("black");
  collisionsOff("table");


//  komo.setGrasp(1., "humanR", "red", 0, .8);
  komo.setGrasp(1, "humanL", "red", 0, .8);

  komo.setDropEdge(1.2, "yellow", "table");
  komo.setPlaceFixed(2., NULL, "yellow", "table", mlr::Transformation("t(0 .1 .045) d(90 1 0 0)"));

  komo.setDropEdge(1.2, "blue", "table");
  komo.setPlaceFixed(2., NULL, "blue", "table", mlr::Transformation("t(0 -.05 .045) d(-90 1 0 0)"));

  komo.setPlace(2.5, "humanR", "red", "black");


//  komo.setTask(1.2, 2., new TaskMap_Proxy(allPTMT, uintA(), .01), OT_sumOfSqr, NoArr, 1e2);

  optimize(komo);
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc, argv);

//  illustrate();
//  analyzeSupport();
//  trial3();
  trial8();

  return 0;
}


