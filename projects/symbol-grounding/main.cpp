#include <MT/opengl.h>

#include "naiveBayesClassificator.h"
#include "DataReader.h"
#include "activeLearningProcess.h"

#include <MT/array_t.cpp>
#include <MT/ors.h>

void sample(MT::Array<arr>& sample) {
  DataReader d;

	d.readDataFile("s120116-1029-0.dat", "s120116-1029-0.dat.rel");
  d.readDataFile("s120116-1029-1.dat", "s120116-1029-1.dat.rel");
	d.readDataFile("s120116-1029-2.dat", "s120116-1029-2.dat.rel");
	d.readDataFile("s120116-1029-3.dat", "s120116-1029-3.dat.rel");
	d.readDataFile("s120116-1029-4.dat", "s120116-1029-4.dat.rel");
  d.readDataFile("s120116-1029-5.dat", "s120116-1029-5.dat.rel");
  d.readDataFile("s120116-1029-6.dat", "s120116-1029-6.dat.rel");
  d.readDataFile("s120116-1029-7.dat", "s120116-1029-7.dat.rel");
	d.readDataFile("s120116-1029-8.dat", "s120116-1029-8.dat.rel");
  d.readDataFile("s120116-1029-9.dat", "s120116-1029-9.dat.rel");

  NaiveBayesClassificator cl;
  cl.setTrainingsData(d.getData(), d.getClasses());

  cl.nextSample(sample);

}

void drawEnv(void*){  glStandardLight(NULL); }

void init(ors::Graph& G,OpenGL& gl,const char* orsFile){
  G.init(orsFile);
  //G.makeLinkTree();
  gl.add(drawEnv,0);
  gl.add(ors::glDrawGraph,&G);
  gl.setClearColors(1.,1.,1.,1.);
  gl.camera.setPosition(10.,-15.,8.);
  gl.camera.focus(0,0,1.);
  gl.camera.upright();
  gl.update();
}

void createCylinder(ors::Body& cyl, const ors::Vector& pos, const int color) {
    
  ors::Transformation t;
  t.pos = pos;
  ors::Shape* s = new ors::Shape();
  s->size[0] = 0.1;
  s->size[1] = 0.1;
  s->size[2] = 0.108;
  s->size[3] = 0.0375;

  s->type = ors::cylinderST;
  switch (color) {
    case 0:  
      s->color[0] = 1;
      s->color[1] = 0;
      s->color[2] = 0;
      break;
    case 1:
      s->color[0] = 0;
      s->color[1] = 1;
      s->color[2] = 0;
      break;
  }
  s->body = &cyl;
  cyl.shapes.append(s);
  cyl.X = t;
}

int main(int argn, char** argv) {
  MT::initCmdLine(argn,argv);
  
  LearningDataVariable data;
  ActiveLearningProcess alp;

  alp.data = &data;

  ors::Graph G;
  OpenGL gl;
  init(G,gl,"situation.ors");


  ors::Body fst;
  ors::Body snd;
  createCylinder(fst, ARR(0.,0.,0.), 0);
  createCylinder(snd, ARR(0.,0.,0.), 1);
 
  G.bodies.append(&fst);
  G.bodies.append(&snd);

  alp.threadOpen();
  alp.threadLoop();
  while (true) { 
    data.readAccess(NULL);
    fst.X.pos = data.pos1; 
    snd.X.pos = data.pos2; 
    data.deAccess(NULL);   
    gl.timedupdate(0.01); 
    sleep(0.1);

  }
}
