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

#include <GL/gl.h>

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

StringA contactName = {"vertex", "edge", "plane"};

struct SupportGraph : GLDrawer{
  mlr::KinematicWorld &K;

  struct Contact : GLDrawer{
    mlr::Frame *f1, *f2;
    mlr::Transformation X=0;
    arr normal;
    arr m1, m2;
    arr eig1, eig2;
    arr axis;
    arr wrench;
    arr ZMP;

    Contact(mlr::Frame *f1, mlr::Frame *f2, const PairCollision& coll)
      : f1(f1), f2(f2), normal(coll.normal), m1(coll.m1), m2(coll.m2), eig1(coll.eig1), eig2(coll.eig2){

      X.pos = mlr::Vector(.5*(m1+m2));
      if(eig1.d0==1){
        X.pos = m1;
        axis=eig1[0];
      }
      if(eig2.d0==1){
        X.pos = m2;
        axis=eig2[0];
      }
      if(eig1.d0==2 && eig2.d0==2){
        X.pos = .5*(m1+m2);
        axis.clear();
      }


    }

    void write(ostream& os){
      cout <<"Contact " <<f1->name <<"--" <<f2->name <<" : " <<contactName(eig1.d0) <<'-' <<contactName(eig2.d0) <<" w=" <<wrench <<" ZMP=" <<ZMP;
    }

    void glDraw(struct OpenGL&){
      if(axis.N){
        glTransform(X);
        glScalef(.2, .2, .2);
        glColor(1., 1., 0., 1.);
        glDrawAxis();
      }

      glLoadIdentity();
      glTranslated(ZMP(0), ZMP(1), ZMP(2));
      glColor(1., 0., 0., 1.);
      glDrawDiamond(.02, .02, .02);
    }
  };

  mlr::Array<Contact*> contacts;

  SupportGraph(mlr::KinematicWorld &K) : K(K){}

  void addContact(mlr::Frame *f1, mlr::Frame *f2, const PairCollision& coll){
    contacts.append(new Contact(f1, f2, coll));
  }

  void computeLinearEquation(double gc=-9.81){
    uint N=K.frames.N;
    uint n=contacts.N;
    arr g(N,6); g.setZero();
    arr A;
    A.resize({N,6,n,6}); A.setZero();

    for(mlr::Frame *a:K.frames){
      if(a->inertia){
        arr grav = {0., 0., a->inertia->mass * gc};
        g[a->ID] = cat( crossProduct( a->X.pos.getArr(), grav), grav );  //wrench in world! frame
      }
    }

    //give all gravity inversely to the root
    arr g_total = sum(g,0);
    g[0] = -g_total;


    for(uint i=0;i<contacts.N;i++){
      Contact *c=contacts.elem(i);
      arr T = ~c->X.getWrenchTransform();
      for(uint j=0;j<6;j++) for(uint k=0;k<6;k++){
        A.elem({c->f1->ID, j, i, k}) = +T(j,k);
        A.elem({c->f2->ID, j, i, k}) = -T(j,k);
      }
    }

    //    cout <<g <<endl <<A <<endl;

    A.reshape(N*6, n*6);
    g.reshape(N*6);

    //add constraints for line contacts (as hinge joints)
#if 1
    for(uint i=0;i<contacts.N;i++){
      Contact *c=contacts.elem(i);
      arr axis = c->axis;
      if(axis.N==3){
        axis = (c->X.rot/Vector_x).getArr(); //HACK: We know that all axes are (1,0,0) in our domains
        axis /= length(axis);
        arr h = cat(axis, zeros(3));
        arr Arow = zeros(n*6);
        Arow({i*6,i*6+5}) = h;
        A.append(Arow);
        g.append(0.);
      }
    }
#endif

    arr f;
//    lapack_min_Ax_b(f, A, -g);
    f = -pseudoInverse(A)*g;
    cout <<"error=" <<sumOfSqr(g+A*f) <<endl;
    f.reshape(n,6);
    for(uint i=0;i<contacts.N;i++) contacts.elem(i)->wrench = f[i];
  }

  void computeZMPs(){
    for(Contact *c:contacts){
      //the wrench is in contact frame (relative to the origin of c->X, which also is on the plane)
      //we only have to compute the ZMP offset on the plane
      c->ZMP = crossProduct(c->normal, c->wrench({0,2})) / scalarProduct(c->normal, c->wrench({3,5}));
      c->ZMP = (c->X*mlr::Vector(c->ZMP)).getArr();
    }
  }

  void write(ostream& os) const{
    for(Contact *c:contacts){ c->write(os); os <<endl; }
  }

  void glDraw(struct OpenGL& gl){
    for(Contact *c:contacts) c->glDraw(gl);
  }

};
stdOutPipe(SupportGraph)

void computeSupportGraph(){
  uint N=11;
  OpenGL gl;

  ofstream fil("z.supportAna");


  for(uint i=0;i<N;i++){
    mlr::String str = STRINGF("p%02i.g", i+1);
    mlr::KinematicWorld K(str);
    fil <<"\n### PROBLEM " <<i <<"  " <<str <<endl;

    //delete all links!
    for(mlr::Frame *f:K.frames) if(f->parent) f->unLink();
    K.optimizeTree(false);

    for(mlr::Frame *f:K.frames) if(f->shape){
      f->shape->cont=true;
      f->shape->mesh().C.append(.3);
    }
    K.swift().setCutoff(.05);
    K.swift().initActivations(K, 0);

    K.stepSwift();
//    K.reportProxies();

    SupportGraph C(K);
    for(mlr::Proxy* p:K.proxies){
      mlr::Frame *a = K.frames(p->a);
      mlr::Frame *b = K.frames(p->b);
      if(a->shape->type()!=mlr::ST_ssBox || b->shape->type()!=mlr::ST_ssBox) continue;
      PairCollision coll(a->shape->sscCore(),
                         b->shape->sscCore(),
                         a->X,
                         b->X);

      coll.marginAnalysis(.001);

      if(coll.distance<a->shape->size(3)+b->shape->size(3)+.001){
        C.addContact(a, b, coll);

        cout <<"PROXY " <<a->name <<"--" <<b->name <<" : " <<contactName(coll.eig1.d0) <<'-' <<contactName(coll.eig2.d0) <<endl;
        fil <<"  " <<a->name <<'-' <<b->name <<" : " <<contactName(coll.eig1.d0) <<'-' <<contactName(coll.eig2.d0) <<endl;

        if(!b->parent){
          b->linkFrom(a, true);
        }else if(!a->parent){
          a->linkFrom(b, true);
        }else{
          cout <<"LOOP!" <<endl;
        }
      }

      gl.add(glStandardLight);
      gl.add(coll);
      gl.add(K);
//      gl.watch();
      gl.update();
      gl.clear();
    }


    for(mlr::Frame *f:K.frames) if(f->parent){
      mlr::Inertia *m = new mlr::Inertia(*f);
      m->defaultInertiaByShape();
    }

    K.gravityToForces(-10.);

//    K.NewtonEuler_backward();

    C.computeLinearEquation(-10.);
    C.computeZMPs();

    cout <<C <<endl;

    K.proxies.clear(); //don't display proxies
    gl.clear();
    gl.text = str;
    gl.add(glStandardLight);
    gl.add(C);
    gl.add(K);
    gl.watch();
    gl.clear();

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
  computeSupportGraph();
//  trial3();
//  trial8();

  return 0;
}


