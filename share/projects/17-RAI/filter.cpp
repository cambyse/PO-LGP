#include "filter.h"
#include <Gui/opengl.h>
#include <Kin/frame.h>
#include <GL/gl.h>

double precision_transition = 20.;
double precision_threshold = 0.25;
double alpha = .2;

//===============================================================================

void glDrawPercepts(void *P){
  PerceptSimpleL& percepts = *((PerceptSimpleL*)P);
  for(PerceptSimple *p:percepts){
    glPushMatrix();
    glTransform(p->pose);
    glColor3f(0,0,0);
    glDrawText(STRING(p->geomID),0,0,0, true);
    p->glDraw(NoOpenGL);
    glPopMatrix();
  }
}

//===============================================================================

double PerceptSimple::fuse(PerceptSimple *other){
  precision += other->precision;
  pose.pos = (1.-alpha)*pose.pos + alpha*other->pose.pos;
  pose.rot.setInterpolate(alpha, pose.rot, other->pose.rot);
  return 0.;
}

void PerceptSimple::write(ostream& os) const{
  os <<store.get(geomID).type <<'_' <<geomID <<" (" <<precision <<") <" <<pose <<">:";
//  os <<" trans=" <<transform <<" frame=" <<frame <<" type=" <<type;
}

void PerceptSimple::glDraw(OpenGL &gl){
  if(geomID>=0) store.get(geomID).glDraw(gl);
  else glDrawAxes(.2);
}

//===============================================================================

FilterSimple::FilterSimple(double dt)
  : Thread("FilterSimple", dt),
    percepts_input(this, "percepts_input"),
    percepts_filtered(this, "percepts_filtered"),
    currentQ(this, "currentQ"),
    switches(this, "switches"),
    timeToGo(this, "timeToGo"),
    robotBase(this, "robotBase"),
    filterWorld(this, "filterWorld")
{

  K = Access<mlr::KinematicWorld>("world").get();
  K.gl().title = "Filter";
  K.gl().add(glDrawPercepts, &percepts_display);

  for(mlr::Frame *a:K.frames){
    if(a->ats["percept"]) objects.append(a);
  }
}

void FilterSimple::step(){
  //== print info
  time += metronome.ticInterval;
  K.gl().text.printf("%4.2fsec", time);
  K.gl().text <<"\ntimeToGo=" <<timeToGo.get();

  //== robot base location
  mlr::Transformation base = robotBase.get();
//  if(!base.isZero()) K["base"]->X = base;
//  cout <<"BASE = " <<K["base"]->X <<endl;

  //== joint state input
  arr q = currentQ.get();
  StringA joints = Access<StringA>("jointNames").get();
  if(q.N) K.setJointState(q, joints );

  //== kinematic switch inputs
  if(switches.hasNewRevision()){
    StringA cmd = switches.get();
    cout <<"CMD = " <<cmd <<endl;
    if(cmd(0)=="attach"){
      mlr::Frame *a = K.getFrameByName(cmd(1));
      mlr::Frame *b = K.getFrameByName(cmd(2));

      if(b->parent) b->unLink();
      b->linkFrom(a, true);
      (new mlr::Joint(*b)) -> type=mlr::JT_rigid;
      K.calc_q();
    }
  }

  //== perceptual inputs

  //just copy!
  percepts_input.writeAccess();
  percepts_filtered.writeAccess();

  K.gl().text <<"\nPerception: #in=" <<percepts_input().N <<" #fil=" <<percepts_filtered().N <<" #obj=" <<objects.N;

//  if(percepts_input().N){
//    cout <<"PERCEPT INPUT:" <<*percepts_input().elem(0) <<endl;
//  }

  //-- step 1: discount precision of old percepts
  // in forward models, the variance of two Gaussians is ADDED -> precision is 1/variance
  for(PerceptSimple *p:percepts_filtered()) p->precision = 1./(1./p->precision + 1./precision_transition);

  //-- step 2: for each percept input, compute matchID or new (=-1) or discard (=-2)
  intA matchId(percepts_input().N);
  if(matchId.N){
    matchId=-1;
    //PRELIM: TRIVIAL ID
    if(percepts_filtered().N) matchId=0;
//    uint i=0;
//    for(PerceptSimple *p:percepts_input()){
//      if(percepts_filtered().N>i) matchId(i)=i;
//      else matchId(i)=-1; //new
//      i++;
//    }
  }

//  listClone(percepts_display, percepts_input());

  //-- step 3: fuse matched percepts; add new percepts; clean percept inputs
  for(uint i=0;i<percepts_input().N;i++){
    PerceptSimple *p = percepts_input().elem(i);
    if(matchId(i)!=-1){ //fuse
      percepts_filtered().elem(matchId(i))->fuse(p);
      delete p;
      percepts_input().elem(i) = NULL;
    }else{ //new
      percepts_filtered().append(p);
    }
  }
  percepts_input().clear();

  //-- step 4: fuse with kinematic knowledge using inverse kinematics
  //TODO, with K
//  while(objects.N<percepts_filtered().N){
//    mlr::Frame *f = objects.append( new mlr::Frame(K) );
//    f->name = STRING("PERC_" <<objects.N);
//    mlr::Shape *s = new mlr::Shape(*f);
//  }
  for(uint i=0;i<objects.N;i++){
    if(i>=percepts_filtered().N) break;
//    objects(i)->shape->geomID = percepts_filtered()(i)->geomID;
//    if(objects(i)->shape->geomID==-1){
//      objects(i)->shape->geomID = K.getFrameByName("box0")->shape->geomID;
//    }
    objects(i)->X = percepts_filtered()(i)->pose;
  }

  //-- done

  listClone(percepts_display, percepts_filtered());

  percepts_filtered.deAccess();
  percepts_input.deAccess();

  filterWorld.set() = K;
  K.gl().update();
}


