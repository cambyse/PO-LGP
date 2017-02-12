#include <unistd.h>
#include <Kin/kin.h>
#include <Kin/kin_physx.h>
#include <Kin/kin_swift.h>
#include <Algo/spline.h>
#include <Algo/algos.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>
#include <GL/gl.h>
#include <Core/thread.h>
//#include <System/engine.h>

#include "ros_module.h"

#include <ros/ros.h>

void createScene(mlr::KinematicWorld& ors, OpenGL& gl) {
  ors.clear();

  for(uint k=0; k<3; k++) {
    mlr::Body *b = new mlr::Body(ors);
    b->X.setRandom();
    b->X.pos.z += 1.;
    b->name <<"rndSphere_" <<k;
    mlr::Shape *s = new mlr::Shape(ors, *b);
    s->type=mlr::ST_box;
    s->size[0]=.1; s->size[1]=.1; s->size[2]=.1; s->size[3]=.1;
  }
  for(uint k=0; k<3; k++) {
    mlr::Body *b = new mlr::Body(ors);
    b->X.setRandom();
    b->X.pos.z += 1.;
    b->name <<"thing_" <<k;
    mlr::Shape *s = new mlr::Shape(ors, *b);
    s->type=mlr::ST_sphere;
    s->size[0]=.1; s->size[1]=.1; s->size[2]=.1; s->size[3]=.1;
    //s->mesh.readFile("pin1.off");
  }
  for(uint k=0; k<10; k++) {
    mlr::Body *b = new mlr::Body(ors);
    b->X.pos.setRandom();
    b->X.pos.z += .5;
    b->name <<"thing_" <<k;
    mlr::Shape *s = new mlr::Shape(ors, *b);
    s->type=mlr::ST_mesh;
    s->mesh.readFile("pin1.off");
  }
  //ors.calcShapeFramesFromBodies();
  cout <<ors <<endl;

  gl.clear();
  gl.add(glStandardScene,NULL);
  gl.add(mlr::glDrawGraph,&ors);
  gl.setClearColors(1.,1.,1.,1.);
  gl.camera.setPosition(10.,-15.,8.);
  gl.camera.focus(0,0,1.);
  gl.update();
}

class SetupWorld : public Module {
private:
  int revision = 0;
  mlr::KinematicWorld w;
  OpenGL gl;

public:
  ACCESS(mlr::KinematicWorld, world);
  ACCESS(bool, do_physics)

  virtual void open() {
    createScene(w, gl);
    world.set() = w;
  }

  virtual void step() {
    // if world has been changed externally
    if(revision < world.data->revisionNumber()) {
      w = world.get();
    }
    if(do_physics.get()) {
      w.physx().step();
      world.set() = w;
      revision = world.data->revisionNumber();
    }
  }
};

REGISTER_MODULE(SetupWorld);

void run() {
  System S;

  S.addModule<SetupWorld>("SetupWorld", /*Module::loopWithBeat,*/ .03); // ~30 Hz
  S.addModule<RosTf>("RosTf", /*Module::loopWithBeat,*/ .03);
  S.addModule<PhysicsMenu>("PhysicsMenu", /*Module::loopWithBeat,*/ .03);
  //S.connect();
  cout << S << endl; // get some info

  S.getAccess<bool>("do_physics")->set() = true;

  // run it
  threadOpenModules(true);
  moduleShutdown().waitForStatusGreaterThan(0);

  cout << "bye bye" << endl;
  threadCloseModules();

}

int MAIN(int argc,char **argv){
  ros::init(argc, argv, "ors_ros", ros::init_options::NoSigintHandler);

  run();

  return 0;
}
