#include <MT/opengl.h>

#include "naiveBayesClassificator.h"
#include "DataReader.h"

#include <MT/array_t.cpp>
#include <MT/ors.h>

void sample(MT::Array<arr>& sample) {
  DataReader d;

	d.readDataFile("s111230-1356-0.dat", "s111230-1356-0.dat.rel");
  d.readDataFile("s111230-1356-1.dat", "s111230-1356-1.dat.rel");
	d.readDataFile("s111230-1436-1.dat", "s111230-1436-1.dat.rel");
	d.readDataFile("s111230-1439-0.dat", "s111230-1439-0.dat.rel");
	d.readDataFile("s111230-1439-1.dat", "s111230-1439-1.dat.rel");
  d.readDataFile("s111230-1442-0.dat", "s111230-1442-0.dat.rel");
  d.readDataFile("s111230-1442-1.dat", "s111230-1442-1.dat.rel");
  d.readDataFile("s111230-1442-2.dat", "s111230-1442-2.dat.rel");

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


  ors::Graph G;
  OpenGL gl;
  init(G,gl,"schunk_no.ors");

  MT::Array<arr> s;
  sample(s);

  cout << "Sample: " << s << endl;

  ors::Body fst;
  ors::Body snd;
  createCylinder(fst, s(0), 0);
  createCylinder(snd, s(1), 1);
 
  G.bodies.append(&fst);
  G.bodies.append(&snd);

  DataReader d;

	d.readDataFile("s111230-1356-0.dat", "s111230-1356-0.dat.rel");
  d.readDataFile("s111230-1356-1.dat", "s111230-1356-1.dat.rel");
	d.readDataFile("s111230-1436-1.dat", "s111230-1436-1.dat.rel");
	d.readDataFile("s111230-1439-0.dat", "s111230-1439-0.dat.rel");
	d.readDataFile("s111230-1439-1.dat", "s111230-1439-1.dat.rel");
  d.readDataFile("s111230-1442-0.dat", "s111230-1442-0.dat.rel");
  d.readDataFile("s111230-1442-1.dat", "s111230-1442-1.dat.rel");
  d.readDataFile("s111230-1442-2.dat", "s111230-1442-2.dat.rel");


  int steps=0;
  int i = 0;
  while (true) { 
    steps ++;
    if (steps % 200 == 0 && i < d.getData().d0) {
      fst.X.pos = d.getData()(i,0);
      snd.X.pos = d.getData()(i,1);
      i++;
    }
    else if (i > d.getData().d0) {
      fst.X.pos = s(0); 
      snd.X.pos = s(1); 
    }
    gl.timedupdate(0.01); 
  }
}
