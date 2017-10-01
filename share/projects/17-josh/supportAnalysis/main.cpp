#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>
#include <stdio.h>

#include <Kin/frame.h>
#include <Kin/kin_swift.h>

#include <Geo/pairCollide.h>
#include <Kin/proxy.h>

void illustrate(){

  OpenGL gl("Red Ball Scenes", 1600, 800);

  uint N=11;
  mlr::Array<mlr::KinematicWorld> K(N);
  for(uint i=0;i<N;i++){
    mlr::String str;
    str.resize(10, false);
    snprintf(str.p, 10, "%02i", i+1);
    K(i).init(STRING("p"<<str<<".g"));
    K(i).orsDrawMarkers=false;
    gl.addView(i, glStandardScene, NULL);
    gl.addSubView(i, K(i));
    gl.views(i).camera.setDefault();
    gl.views(i).camera.setPosition(3., 0., 3.);
    gl.views(i).camera.focus(0., 0., 1.);
    gl.views(i).camera.upright();
    gl.views(i).text <<"problem " <<i+1;
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
    fil <<"\n### PROBLEM " <<i <<endl;
    mlr::String str;
    str.resize(10, false);
    snprintf(str.p, 10, "%02i", i+1);
    mlr::KinematicWorld K(STRING("p"<<str<<".g"));

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

int main(int argc,char **argv){
  mlr::initCmdLine(argc, argv);

//  illustrate();
  analyzeSupport();

  return 0;
}


