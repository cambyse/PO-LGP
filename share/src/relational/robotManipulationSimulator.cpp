/*  
    Copyright 2011   Tobias Lang
    
    E-mail:    tobias.lang@fu-berlin.de
    
    This file is part of libPRADA.

    libPRADA is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libPRADA is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libPRADA.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <MT/opengl.h>
#include <MT/plot.h>
#include <utilTL.h>

#include "robotManipulationSimulator.h"
#include <sstream>


// SPECIFIC OBJECT KNOWLEDGE
#define BLOCK_SMALL 0.04
#define BLOCK_BIG 0.06
#define BLOCK_VERY_BIG 0.08
#define SMALL_HEIGHT_STEP 0.05
// table + neutralHeightBonus = NEUTRAL_HEIGHT
#define NEUTRAL_HEIGHT_BONUS 0.5
#define HARD_LIMIT_DIST_Y -0.8

// !!!!!!!!!!!!!!!!!!!!!
// ATTENTION: NOISE SPECIFICATIONS (noise of the robot actions) somewhere below.
// !!!!!!!!!!!!!!!!!!!!!










// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================

  // --------------------------------
  // LOW LEVEL CONTROL
  // --------------------------------
  

uint gl_step = 0;
uint revel_step = 0;



void oneStep(const arr &q,ors::Graph *C,OdeInterface *ode,SwiftInterface *swift,OpenGL *gl, RevelInterface *revel,const char* text){
#ifdef MT_ODE
  C->setJointState(q);
  C->calcBodyFramesFromJoints();
  if(ode){
    ode->exportStateToOde(*C);
    ode->step(.01);
    ode->importStateFromOde(*C);
    //ode->importProxiesFromOde(*C);
  //C->getJointState(q);
  }
  if(swift){
    swift->computeProxies(*C);
  }else{
    if(ode) ode->importProxiesFromOde(*C);
  }
  
  if(gl){
    gl_step++;
//     if (gl_step % 1 == 50) {
      gl->text.clr() <<text <<endl;
      gl->update();
//     }
  }
#ifdef MT_REVEL
  if(revel){
    revel_step++;
    if (true) {
      revel->addFrameFromOpengl();
      revel_step = 0;
    }
  }
#endif
#endif
}



void controlledStep(arr &q,arr &W,ors::Graph *C,OdeInterface *ode,SwiftInterface *swift,OpenGL *gl, RevelInterface *revel,TaskVariableList& TVs, const char* text){
#ifdef MT_ODE
  static arr dq;
  updateState(TVs);
  updateChanges(TVs);
  bayesianControl(TVs,dq,W);
  q += dq;
  oneStep(q,C,ode,swift,gl,revel,text);
#endif
}















// How many time-steps until action fails
#define SEC_ACTION_ABORT 500

RobotManipulationSimulator::RobotManipulationSimulator(){
  C=0;
  gl=0;
  ode=0;
  swift=0;
  revel=0;
  Tabort = SEC_ACTION_ABORT;
}

RobotManipulationSimulator::~RobotManipulationSimulator(){
    shutdownAll();
}




// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================

// --------------------------------
// ADMINISTRATION
// --------------------------------


arr q0, W;

void drawEnv(void* horst){
#ifdef MT_FREEGLUT
  glStandardLight(horst);
//   glDrawFloor(4.,1,1,1);
  glDrawFloor(4., 108./255., 123./255., 139./255.);
#endif
}


void RobotManipulationSimulator::calcObjectNumber() {
  // determine number of objects
  numObjects = 0;
  // assuming that all objects start with "o"
  std::stringstream ss;
  uint i;
  for (i=1;;i++) {
    ss.str("");
    ss << "o" << i;
    ors::Body* n = C->getBodyByName(ss.str().c_str());
    if (n==0)
      break;
    numObjects++;
  }
}


void RobotManipulationSimulator::loadConfiguration(const char* ors_filename){
  if(C) delete C;
  C = new ors::Graph();
  MT::load(*C,ors_filename);
  C->calcBodyFramesFromJoints();
  C->getJointState(q0);
  
  uint i;
  arr BM(C->bodies.N);
  BM=1.;
  for(i=BM.N;i--;){
    if(C->bodies(i)->outLinks.N){
      BM(i) += BM(C->bodies(i)->outLinks(0)->to->index);
    }
  }
  arr Wdiag(q0.N);
  for(i=0;i<q0.N;i++) Wdiag(i)=BM(C->joints(i)->to->index);
  W.setDiag(Wdiag);
  
  calcObjectNumber();
  
  // determine table height
  neutralHeight = getPosition(getTableID())[2] + NEUTRAL_HEIGHT_BONUS;
  
  if(gl){
    gl->clear();
    gl->add(drawEnv,0);
    gl->add(ors::glDrawGraph,C);
    return;
  }
#ifdef MT_FREEGLUT
  gl=new OpenGL;
  gl->add(drawEnv,0);
  gl->add(ors::glDrawGraph,C);
  gl->setClearColors(1.,1.,1.,1.);
  orsDrawProxies = false;
//   gl->resize(1024, 768);
  gl->resize(800, 600);
//   gl->resize(1024, 600);
//   gl->resize(600, 400);
//   gl->resize(400, 320);
  gl->camera.setPosition(2.5,-7.5,2.3);  // position of camera
//   gl->camera.setPosition(6.,-4.,4.5);  // position of camera
  gl->camera.focus(-0.25, -0.6, 1.1);  // rotate the frame to focus the point (x,y,z)
  gl->update();
#endif
}


void RobotManipulationSimulator::write(const char* ors_filename){
  MT::save(*C,ors_filename);
}




void RobotManipulationSimulator::startOde(double ode_coll_bounce, double ode_coll_erp, double ode_coll_cfm, double ode_friction) {
#ifdef MT_ODE
  CHECK(C,"load a configuration first");
  if(ode) delete ode;
  ode = new OdeInterface;
  
  // SIMULATOR PARAMETER
  ode->coll_bounce = ode_coll_bounce;
  ode->coll_ERP = ode_coll_erp;
  ode->coll_CFM = ode_coll_cfm;
  ode->friction = ode_friction;
  
  ode->createOde(*C);
#endif
}

void RobotManipulationSimulator::startSwift(){
#ifdef MT_SWIFT
  if(swift) delete swift;
  swift = new SwiftInterface;
  swift->init(*C);
#endif
}

void RobotManipulationSimulator::startIBDS(){
  NIY;
}

void RobotManipulationSimulator::startRevel(const char* filename){
#ifdef MT_REVEL
  if(revel) delete revel;
  revel= new RevelInterface;
  revel->open(gl->width(),gl->height(), filename);
#endif
}



void RobotManipulationSimulator::shutdownAll(){
  if(C) delete C;          C=0;
#ifdef MT_ODE
  if(ode) delete ode;      ode=0;
#endif
#ifdef MT_FREEGLUT
  if(gl) delete gl;        gl=0;
#endif
#ifdef MT_SWIFT
  if(swift) delete swift;  swift=0;
#endif
#ifdef MT_REVEL
  if(revel){ revel->close(); delete revel; } revel=0;
#endif
}



void RobotManipulationSimulator::simulate(uint t, const char* message){
  String msg_string(message);
  arr q;
  TaskVariableList TVs;
  TVs.clear();
  C->getJointState(q);
  bool change = true;
  for(;t--;){
    MT::String send_string;
    if (msg_string.N() == 0) {
      if (t%20==0)
        change = !change;
      if (change)
        send_string << "S";
    }
    else
      send_string << msg_string;
    //     send_string << msg_string << "     \n\n(time " << t << ")";
    controlledStep(q,W,C,ode,swift,gl,revel,TVs,send_string);
  }
}


void RobotManipulationSimulator::watch(){
  gl->text.clr() <<"Watch" <<endl;
#ifdef MT_FREEGLUT
  gl->watch();
#endif
}



void RobotManipulationSimulator::indicateFailure(){
  // drop object
  ors::Joint* e;
  uint i;
  for_list(i,e,C->getBodyByName("fing1c")->outLinks){
    del_edge(e,C->bodies,C->joints,true); //otherwise: no object in hand
  }
  std::cerr << "RobotManipulationSimulator: CONTROL FAILURE" << endl;
  relaxPosition();
}

// if z-value of objects is beneath THRESHOLD
bool RobotManipulationSimulator::onGround(uint id) {
    double THRESHOLD = 0.4;
    ors::Body* obj=C->bodies(id);
    if (obj->X.pos.p[2] < THRESHOLD)
        return true;
    else
        return false;
}








// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================

//   ***********  ACTIONS   **************


// ==============================================================================
// ==============================================================================
// ==============================================================================
//   GRAB

// ******************************************************************
// THIS IS WHERE YOU CAN BRING IN YOUR OWN IDEAS OF "NOISE" ETC. !!!

double GRAB_UNCLEAR_OBJ_FAILURE_PROB = 0.4;

void RobotManipulationSimulator::grab_final(const char *manipulator,const char *obj_grabbed, const char* message){
  ors::Body *obj=C->getBodyByName(obj_grabbed);
  bool isTable = obj->index == getTableID();

  uintA list;
  getObjectsAbove(list, obj_grabbed);
  bool object_is_clear = list.N == 0;
  
  MT::String msg_string(message);
  if (msg_string.N() == 0) {
    msg_string << "grab "<<obj_grabbed;
  }
  
  DefaultTaskVariable x("endeffector", *C, posTVT, manipulator, 0, 0, 0, ARR());
  x.setGainsAsAttractor(20, .2);
  x.y_prec=1000.;
  
  uint t;
  arr q, dq;
  
  // (1) drop object if one is in hand
  //   dropInhandObjectOnTable(message);
  uint id_grabbed = getInhand();
  if (id_grabbed != UINT_MAX) {
    // move a bit towards new object
    for(t=0;t<10;t++){
      x.y_target.setCarray(obj->X.pos.p,3);
      if (isTable) {x.y_target(2) = neutralHeight-0.1;}
      MT::String send_msg;
      send_msg << msg_string /*<< "      \n\n(time " << t << ")"*/;
  //     controlledStep(q,W,send_msg);
      controlledStep(q,W,C,ode,swift,gl,revel,TVs,send_msg);
      // TODO passt das? Ueberall state durch "active" ersetzt
//       if(x.active==1 || C->getContact(x.i,obj->index)) break;
      if(x.active==1 || C->getContact(x.i,obj->index)) break;
    }
    if(C->bodies(id_grabbed)->inLinks.N){
      ors::Joint* e=C->bodies(id_grabbed)->inLinks(0);
      del_edge(e,C->bodies,C->joints,true);
    }
  }
  // MARCs OLD VERSION
//     ors::Joint *e;
//   uint i;
//   for_list(i, e, C->bodies(x.i)->outLinks){
//     NIY;
    //C->del_edge(e);
//   }
  
  // (2) move towards new object
  C->getJointState(q);
  for(t=0; t<Tabort; t++){
    x.y_target.setCarray(obj->X.pos.p, 3);
    if (isTable) {x.y_target(2) = neutralHeight-0.1;}
    MT::String send_msg;
    send_msg << msg_string /*<< "      \n\n(time " << t << ")"*/;
    controlledStep(q,W,C,ode,swift,gl,revel,TVs,send_msg);
//     gl->text.clr() <<"catchObject --  time " <<t <<endl;
//     gl->update();
    if(x.err<.05 || C->getContact(x.i, obj->index)) break;
  }
  if(t==Tabort){ indicateFailure(); return; }
  
    
    
  
  // (3) grasp if not table or world
  if(obj->index!=getTableID()){
    C->glueBodies(C->bodies(x.i), obj);
  }else{
    //indicateFailure()?
  }
  
  // (4) move upwards (to avoid collisions)
  // to be sure: unset contact of that object if grabbed from table
  ors::Shape* s = NULL;
  if (isClear(obj->index)) {
    s = obj->shapes(0);
    s->cont = false;
  }
   
  for(t=0; t<Tabort; t++){
    x.y_target.setCarray(obj->X.pos.p, 3);
//     if (x.y_target(2) < neutralHeight)       // ALTE LOESUNG -- TOBIAS
//       x.y_target(2) += SMALL_HEIGHT_STEP;    // ALTE LOESUNG -- TOBIAS
    x.y_target(2) = 1.2;
    MT::String send_msg;
    send_msg << msg_string /*<< "      \n\n(time " << t << ")"*/;
    controlledStep(q,W,C,ode,swift,gl,revel,TVs,send_msg);
//     gl->text.clr() <<"catchObject --  time " <<t <<endl;
//     gl->update();
    if(x.err<.05) break;
    
    // might drop object
    if (t==50  &&  !object_is_clear  &&  obj->index!=getTableID()) {
      cout<<"I might drop that object!"<<endl;
      if (rnd.uni() < GRAB_UNCLEAR_OBJ_FAILURE_PROB) {
        dropObject(convertObjectName2ID(manipulator));
        cout<<"  >>  Uh, indeed I dropped it!"<<endl;
      }
    }
  }
  if(t==Tabort){ indicateFailure(); return; }
  
    
  // reset contact of grabbed object
  if (s!=NULL) {
    s->cont = true;
#ifdef MT_SWIFT
    swift->initActivations(*C);
#endif
  }
  if(t==Tabort) { indicateFailure(); return; }
}

void RobotManipulationSimulator::grab(uint ID, const char* message) {
  grab(convertObjectID2name(ID), message);
}

void RobotManipulationSimulator::grab(const char* obj, const char* message) {
  grab_final("fing1c", obj, message);
}




// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
//   DROP



void RobotManipulationSimulator::dropObject(uint manipulator_id) {
  if (C->bodies(manipulator_id)->outLinks.N == 0)
    return;
  CHECK(C->bodies(manipulator_id)->outLinks.N == 1, "too many objects in hand");
  del_edge(C->bodies(manipulator_id)->outLinks(0), C->bodies, C->joints, true);
}

void RobotManipulationSimulator::dropInhandObjectOnTable(const char* message) {
  uint id_grabbed = getInhand();
  if (UINT_MAX == id_grabbed)
    return;
  dropObjectAbove(id_grabbed, getTableID(), message);
  relaxPosition(message);
}

void RobotManipulationSimulator::dropObjectAbove_final(const char *obj_dropped, const char *obj_below, const char* message){
  MT::String msg_string(message);
  if (msg_string.N() == 0) {
    msg_string << "puton " << obj_below;
  }
  
  arr I(q0.N,q0.N); I.setId();
  bool obj_is_inhand = strlen(obj_dropped) > 0;
  MT::String obj_dropped1;
  if (obj_is_inhand) {
    obj_dropped1 = obj_dropped;
  }
  else
    obj_dropped1 = "fing1c";
  
  uint obj_dropped1_index=C->getBodyByName(obj_dropped1)->index;
  uint obj_below_id = convertObjectName2ID(obj_below);
  
  if (obj_below_id == obj_dropped1_index) { // if puton itself --> puton table
//     MT_MSG("Trying to put on table");
    obj_below_id = getTableID();
  }
  
  DefaultTaskVariable o("obj",*C,posTVT,obj_dropped1,0,0,0,0);
  DefaultTaskVariable z;
  //
  ors::Quaternion rot;
  rot = C->bodies(obj_dropped1_index)->X.rot;
  ors::Vector upvec; double maxz=-2;
  if((rot*VEC_x)(2)>maxz){ upvec=VEC_x; maxz=(rot*upvec)(2); }
  if((rot*VEC_y)(2)>maxz){ upvec=VEC_y; maxz=(rot*upvec)(2); }
  if((rot*VEC_z)(2)>maxz){ upvec=VEC_z; maxz=(rot*upvec)(2); }
  if((rot*(-VEC_x))(2)>maxz){ upvec=-VEC_x; maxz=(rot*upvec)(2); }
  if((rot*(-VEC_y))(2)>maxz){ upvec=-VEC_y; maxz=(rot*upvec)(2); }
  if((rot*(-VEC_z))(2)>maxz){ upvec=-VEC_z; maxz=(rot*upvec)(2); }
  ors::Transformation tf;
  tf.rot.setDiff(VEC_z, upvec);
  z.set("obj-z-align",*C,zalignTVT,obj_dropped1_index,tf,-1,ors::Transformation(),0);
  //
  DefaultTaskVariable r("full state",*C,qLinearTVT,0,0,0,0,I);
  DefaultTaskVariable c("collision",*C,collTVT,0,0,0,0,ARR(.02));
  
  r.setGainsAsAttractor(50,.1);
  r.y_prec=1.;
  r.y_target=q0;
  r.active=false;
  o.setGainsAsAttractor(20,.2);
  o.y_prec=1000.;
//   o.active_tol=.005;
  // TODO tolerance now?
  z.setGainsAsAttractor(20,.2);
  z.y_prec=1000.;
  z.y_target.resize(1);  z.y_target = 1.;
  // TODO
//   z.state_tol=.005;
  
  c.setGainsAsAttractor(20,.1);
  c.y_prec=10000.;
//   c.state_tol=.005;
  // TODO tolerance now?
  if(!swift) c.active=false;
  
  uint t;
  arr q,dq;
  C->getJointState(q);

  TaskVariableList TVs;
  TVs.append(&o);
  TVs.append(&z);
  TVs.append(&r);
  TVs.append(&c);  
  
  // Calculate (noisy) target position
  double x_target, y_target;
  if (obj_is_inhand)
    calcTargetPositionForDrop(x_target, y_target, obj_dropped1_index, obj_below_id);
  else {
    x_target = C->bodies(obj_below_id)->X.pos.p[0];
    y_target = C->bodies(obj_below_id)->X.pos.p[1];
  }
  
  // Hard limit on y-distance to robot
  if (y_target < HARD_LIMIT_DIST_Y)
    y_target = HARD_LIMIT_DIST_Y;
  
  
  // Phase 1: up
  updateState(TVs);
  o.y_target(2) += .3;
  // TODO
//   o.active_tol=.05;
  for(t=0;t<Tabort;t++){
    if (o.y_target(2) < neutralHeight)
      o.y_target(2) += 0.05;
    MT::String send_string;
    send_string << msg_string /*<< "     \n\n(time " << t << ")"*/;
//     controlledStep(q,W,send_string);
    controlledStep(q,W,C,ode,swift,gl,revel,TVs,send_string);
    if(o.active==1) break;
  }
  if(t==Tabort){ indicateFailure(); return; }
  
  
  
  // Phase 2: above object
  // TODO
//   o.active_tol=.05;
  o.y_target(0) = x_target;
  o.y_target(1) = y_target;
  // WHERE TO GO ABOVE
  o.y_target(2) = highestPosition(o.y_target(0), o.y_target(1), 0.06, obj_dropped1_index) + .2;
  for(t=0;t<Tabort;t++){
    MT::String send_string;
    send_string << msg_string /*<< "     \n\n(time " << t << ")"*/;
//     controlledStep(q,W,send_string);
    controlledStep(q,W,C,ode,swift,gl,revel,TVs,send_string);
    if(o.active==1) break;
  }
  if(t==Tabort){ indicateFailure(); return; }

  //turn off collision avoidance
  c.active=false;

  
  
  // Phase 3: down
  // TODO
//   o.active_tol=.002;
  // IMPORTANT PARAM: set distance to target (relative height-distance in which "hand is opened" / object let loose)
  double Z_ADD_DIST = getSize(obj_dropped1_index)[0]/2 + .05;
  if (getOrsType(obj_below_id) == OBJECT_TYPE__BOX) {
    Z_ADD_DIST += 0.05;
  }
  double z_target = Z_ADD_DIST + highestPosition(o.y_target(0), o.y_target(1), 0.06, obj_dropped1_index);
  if (getTableID() == obj_below_id)
    z_target += 0.05;
  // we slowly approach z_target
  o.y_target(2) = C->bodies(obj_dropped1_index)->X.pos.p[2] - 0.05;
  for(t=0;t<Tabort;t++){
//     cout<<"C->bodies(obj_dropped1_index)->X.pos.p[2] = "<<C->bodies(obj_dropped1_index)->X.pos.p[2]<<endl;
//     cout<<"z_target = "<<z_target<<endl;
    if (C->bodies(obj_dropped1_index)->X.pos.p[2] - o.y_target(2) < 0.05) {
      // slowly go down
      if (o.y_target(2) - z_target > 0.1)
        o.y_target(2) -= 0.02;
      else if (o.y_target(2) - z_target > 0.02)
        o.y_target(2) -= 0.01;
    }
    // WHERE TO GO ABOVE
    MT::String send_string;
    send_string << msg_string /*<< "     \n\n(time " << t << ")"*/;
//     controlledStep(q,W,send_string);
    controlledStep(q,W,C,ode,swift,gl,revel,TVs,send_string);
    if(o.active==1 && z.active==1) break;
  }
  if(t==Tabort){ indicateFailure(); return; }
  
  
  // Phase 4: let loose
  if(C->bodies(o.i)->inLinks.N && obj_is_inhand){
    ors::Joint* e=C->bodies(o.i)->inLinks(0);
    del_edge(e,C->bodies,C->joints,true); //otherwise: no object in hand
  }
}

void RobotManipulationSimulator::dropObjectAbove(uint obj_id, uint obj_below, const char* message) {
  dropObjectAbove_final(convertObjectID2name(obj_id), convertObjectID2name(obj_below), message);
}

void RobotManipulationSimulator::dropObjectAbove(uint obj_below, const char* message) {
  dropObjectAbove(getInhand(), obj_below, message);
}


// ******************************************************************
// THIS IS WHERE YOU CAN BRING IN YOUR OWN IDEAS OF "NOISE" ETC. !!!

double DROP_TARGET_NOISE__BLOCK_ON_SMALL_BLOCK = 0.001;
double DROP_TARGET_NOISE__BLOCK_ON_BIG_BLOCK = 0.003;
double DROP_TARGET_NOISE__BALL_ON_BIG_BLOCK = 0.0066;
double DROP_TARGET_NOISE__BALL_ON_SMALL_BLOCK = 0.02;
double DROP_TARGET_NOISE__ON_BALL = 0.01;

// Noise comes in here
void RobotManipulationSimulator::calcTargetPositionForDrop(double& x, double& y, uint obj_dropped, uint obj_below) {
  // Noise for puton position
  double x_noise = 0., y_noise = 0.;
  // noise [START]
  double std_dev_noise;
  
  double obj_below_size = getSize(obj_below)[0];
  double obj_below_type = getOrsType(obj_below);
//   double obj_dropped_size = getSize(obj_dropped)[0];
  double obj_dropped_type = getOrsType(obj_dropped);
  
  // Below = Table
  if (obj_below == getTableID()) {
    double DROP_TARGET_NOISE__ON_TABLE = 0.09;
    std_dev_noise = DROP_TARGET_NOISE__ON_TABLE;
    uint tries = 0;
    while (true) {
      tries++;
      if (tries>20000) {
          MT_MSG("Can't find empty position on table, throw it whereever!");
          break;
      }
      x_noise = std_dev_noise * rnd.gauss();
      y_noise = std_dev_noise * rnd.gauss() * 0.8 + 0.2; // tisch ist nicht so breit wie lang
//       if (x_noise>0.5 || y_noise>0.5) // stay on table
      if (x_noise>0.5) // stay on table
        continue;
      if (C->bodies(obj_below)->X.pos.p[1] + y_noise < -1.0  ||  C->bodies(obj_below)->X.pos.p[1] + y_noise > -0.05)
        continue;
      if (freePosition(C->bodies(obj_below)->X.pos.p[0]+x_noise, C->bodies(obj_below)->X.pos.p[1]+y_noise, 0.05))
        break;
    }
  }
  // Below = Block
  else if (obj_below_type == ors::boxST) {
    // (1) Dropping block
    if (obj_dropped_type == ors::boxST) {
      // (1a) on small block
      if (TL::areEqual(obj_below_size, BLOCK_SMALL)) {
        std_dev_noise = DROP_TARGET_NOISE__BLOCK_ON_SMALL_BLOCK;
      }
      // (1b) on big block
      else if (TL::areEqual(obj_below_size, BLOCK_BIG)) {
        std_dev_noise = DROP_TARGET_NOISE__BLOCK_ON_BIG_BLOCK;
      }
      else if (TL::areEqual(obj_below_size, BLOCK_VERY_BIG)) {
        std_dev_noise = DROP_TARGET_NOISE__BLOCK_ON_BIG_BLOCK;
      }
      else {NIY;}
    }
    // (2) Dropping ball
    else if (obj_dropped_type == ors::sphereST) {
      // (2a) on small block
      if (TL::areEqual(obj_below_size, BLOCK_SMALL)) {
        std_dev_noise = DROP_TARGET_NOISE__BALL_ON_SMALL_BLOCK;
      }
      // (2b) on big block
      else if (TL::areEqual(obj_below_size, BLOCK_BIG)) {
        std_dev_noise = DROP_TARGET_NOISE__BALL_ON_BIG_BLOCK;
      }
      else {NIY;}
    }
    else {NIY;}
    
    x_noise = std_dev_noise * rnd.gauss();
    y_noise = std_dev_noise * rnd.gauss();
    
//     PRINT(obj_below_size);
//     PRINT(std_dev_noise);
  }
  // Below = Ball
  else if (obj_below_type == ors::sphereST) {
    x_noise = DROP_TARGET_NOISE__ON_BALL * rnd.gauss();
    y_noise = DROP_TARGET_NOISE__ON_BALL * rnd.gauss();
  }
  // noise [END]
  
  x = C->bodies(obj_below)->X.pos.p[0] + x_noise;
  y = C->bodies(obj_below)->X.pos.p[1] + y_noise;
}




// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================


void RobotManipulationSimulator::relaxPosition(const char* message){
  MT::String msg_string(message);
  if (msg_string.N() == 0) {
    msg_string << "Relax position";
  }
  
  uint inhand_id = getInhand();
  ors::Shape* s = NULL;
  // simplification: set off contacts for inhand-object
  if (inhand_id != UINT_MAX) {
    s = C->getBodyByName(convertObjectID2name(inhand_id))->shapes(0);
    s->cont = false;
  }
  
  arr q,dq;
  C->getJointState(q);
  
  arr I(q.N,q.N); I.setId();
  
  DefaultTaskVariable x("full state",*C,qLinearTVT,0,0,0,0,I);
  x.setGainsAsAttractor(20,.1);
  x.y_prec=1000.;
  x.y_target=q0; 
//   x.active_tol=.2;
  // TODO tolerance now?
  TaskVariableList TVs;
  TVs.append(&x);

  uint t;
  for(t=0;t<Tabort;t++){
    MT::String send_string;
    send_string << msg_string /*<< "     \n\n(time " << t << ")"*/;
//     controlledStep(q,W,send_string);
    controlledStep(q,W,C,ode,swift,gl,revel,TVs,send_string);
    if(x.active==1) break;
  }
  
  // simplification: set on contacts for inhand-object
  if (s!=NULL) {
    s->cont = true;
#ifdef MT_SWIFT
    swift->initActivations(*C);
#endif
  }
  
  if(t==Tabort){ indicateFailure(); return; }
}


// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
//   BOX


#  ifdef MT_ODE

#  include <ode/ode.h>
#  include <ode/../internal/objects.h>
#  include <ode/../internal/joints/joints.h>
#  include <ode/../internal/collision_kernel.h>
#  include <ode/../internal/collision_transform.h>
#  ifdef MT_MSVC
#    undef HAVE_UNISTD_H
#    undef HAVE_SYS_TIME_H
#  endif

#endif

void RobotManipulationSimulator::openBox(uint id, const char* message) {
#  ifdef MT_ODE
  MT::String msg_string(message);
  if (msg_string.N() == 0) {
    msg_string << "openBox "<<id;
  }
  
  // move manipulator towards box
  ors::Body* obj = C->bodies(id);
  DefaultTaskVariable x("endeffector",*C,posTVT,"fing1c",0,0,0,0);
  x.setGainsAsAttractor(20,.2);
  x.y_prec=1000.;
  TaskVariableList TVs;
  TVs.append(&x);

  uint t;
  arr q,dq;
  C->getJointState(q);
  for(t=0;t<Tabort;t++){
    x.y_target.setCarray(obj->X.pos.p,3);
    x.y_target.p[2] += 0.15;
    MT::String send_string;
    send_string << msg_string /*<< "     \n\n(time " << t << ")"*/;
//     controlledStep(q,W,send_string);
    controlledStep(q,W,C,ode,swift,gl,revel,TVs,send_string);
    if(x.active==1 || C->getContact(x.i,obj->index)) break;
  }
  if(t==Tabort){ indicateFailure(); return; }
  simulate(30, msg_string);
  
  // open it
  ors::Shape* s = C->bodies(id)->shapes(5);
  s->rel.setText("<t(0 0 .075) t(0 -.05 0) d(80 1 0 0) t(0 .05 .0)>");
#ifdef MT_SWIFT
  swift->initActivations(*C);
#endif
  
  dGeomID geom;
  geom = ode->geoms(s->index);
  dGeomSetQuaternion(geom,*((dQuaternion*)s->rel.rot.p));
  dGeomSetPosition(geom,s->rel.pos(0),s->rel.pos(1),s->rel.pos(2));
#endif
}

void RobotManipulationSimulator::closeBox(uint id, const char* message) {
#  ifdef MT_ODE
  MT::String msg_string(message);
  if (msg_string.N() == 0) {
    msg_string << "closeBox "<<id;
  }
  
  // move manipulator towards box
  ors::Body* obj = C->bodies(id);
  DefaultTaskVariable x("endeffector",*C,posTVT,"fing1c",0,0,0,0);
  x.setGainsAsAttractor(20,.2);
  x.y_prec=1000.;
  TaskVariableList TVs;
  TVs.append(&x);

  uint t;
  arr q,dq;
  C->getJointState(q);
  for(t=0;t<Tabort;t++){
    x.y_target.setCarray(obj->X.pos.p,3);
    x.y_target.p[2] += 0.15;
    MT::String send_string;
    send_string << msg_string /*<< "     \n\n(time " << t << ")"*/;
//     controlledStep(q,W,send_string);
    controlledStep(q,W,C,ode,swift,gl,revel,TVs,send_string);
    if(x.active==1 || C->getContact(x.i,obj->index)) break;
  }
  if(t==Tabort){ indicateFailure(); return; }
  simulate(30, msg_string);
  
  // close it
  ors::Shape* s = C->bodies(id)->shapes(5);
  s->rel.setText("<t(0 0 .075)>");
#ifdef MT_SWIFT
  swift->initActivations(*C);
#endif

  dGeomID geom;
  geom = ode->geoms(s->index);
  dGeomSetQuaternion(geom,*((dQuaternion*)s->rel.rot.p));
  dGeomSetPosition(geom,s->rel.pos(0),s->rel.pos(1),s->rel.pos(2));
#endif
}









// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================

// --------------------------------
//   GENERAL OBJECT INFORMATION
// --------------------------------

void RobotManipulationSimulator::getObjects(uintA& objects) { //!< return list all objects
  objects.clear();
  
  objects.append(getTableID());
  
  uintA blocks;
  getBlocks(blocks);
  objects.append(blocks);
  
  uintA balls;
  getBalls(balls);
  objects.append(balls);
  
  uintA boxes;
  getBoxes(boxes);
  objects.append(boxes);
}


void RobotManipulationSimulator::getTypes(TermTypeL& objects_types, const uintA& objects, const TermTypeL& types) { //!< return list of all object types
  objects_types.resize(objects.N);
  uint i;
  TL::TermType* type_box = NULL, *type_block = NULL, *type_ball = NULL, *type_table = NULL;
  FOR1D(types, i) {
    if (types(i)->name == MT::String("block")) {
      type_block = types(i);
    }
    else if (types(i)->name == MT::String("ball")) {
      type_ball = types(i);
    }
    else if (types(i)->name == MT::String("table")) {
      type_table = types(i);
    }
    else if (types(i)->name == MT::String("box")) {
      type_box = types(i);
    }
  }
  
  CHECK(type_block != NULL, "");
  CHECK(type_ball != NULL, "");
  CHECK(type_table != NULL, "");
  CHECK(type_box != NULL, "");
//   type_block->writeNice(cout);  cout<<endl;
//   type_ball->writeNice(cout);  cout<<endl;
//   type_table->writeNice(cout);  cout<<endl;
//   type_box->writeNice(cout);  cout<<endl;
  
  uint table_id = getTableID();
  uintA blocks;
  getBlocks(blocks);
  uintA balls;
  getBalls(balls);
  uintA boxes;
  getBoxes(boxes);
  
//   PRINT(table_id);
//   PRINT(blocks);
//   PRINT(balls);
//   PRINT(boxes);
  
  FOR1D(objects, i) {
    if (objects(i) == table_id) {
      objects_types(i) = type_table;
    }
    else if (blocks.findValue(objects(i)) >= 0) {
      objects_types(i) = type_block;
    }
    else if (balls.findValue(objects(i)) >= 0) {
      objects_types(i) = type_ball;
    }
    else if (boxes.findValue(objects(i)) >= 0) {
      objects_types(i) = type_box;
    }
    else
      NIY;
  }
}



uint RobotManipulationSimulator::getTableID() {
  ors::Body* n = C->getBodyByName("table");
  return n->index;
}


void RobotManipulationSimulator::getBlocks(uintA& blocks) {
  blocks.clear();
  // assuming that all objects start with "o"
  std::stringstream ss;
  uint i;
  for (i=1;i<=numObjects;i++) {
    ss.str("");
    ss << "o" << i;
    ors::Body* n = C->getBodyByName(ss.str().c_str());
    if (n->shapes.N == 1) {
      if (n->shapes(0)->type == ors::boxST)
        blocks.append(n->index);
    }
  }
}

void RobotManipulationSimulator::getBalls(uintA& balls) {
  balls.clear();
  // assuming that all objects start with "o"
  std::stringstream ss;
  uint i;
  for (i=1;i<=numObjects;i++) {
    ss.str("");
    ss << "o" << i;
    ors::Body* n = C->getBodyByName(ss.str().c_str());
    if (n->shapes.N == 1) {
      if (n->shapes(0)->type == ors::sphereST)
        balls.append(n->index);
    }
  }
}


void RobotManipulationSimulator::getBoxes(uintA& boxes) {
  boxes.clear();
  // assuming that all objects start with "o"
  std::stringstream ss;
  uint i;
  for (i=1;i<=numObjects;i++) {
    ss.str("");
    ss << "o" << i;
    ors::Body* n = C->getBodyByName(ss.str().c_str());
    if (isBox(n->index))
      boxes.append(n->index);
  }
}


bool RobotManipulationSimulator::isBox(uint id) {
  return C->bodies(id)->shapes.N == 6;
}


uint RobotManipulationSimulator::convertObjectName2ID(const char* name) {
  return C->getBodyByName(name)->index;
}


const char* RobotManipulationSimulator::convertObjectID2name(uint ID) {
  if (C->bodies.N > ID)
    return C->bodies(ID)->name;
  else
    return "";
}


int RobotManipulationSimulator::getOrsType(uint id) {
  if (C->bodies(id)->shapes.N == 1) {
    return C->bodies(id)->shapes(0)->type;
  }
  else
    return OBJECT_TYPE__BOX;
}

double* RobotManipulationSimulator::getSize(uint id) {
  return C->bodies(id)->shapes(0)->size;
}

double* RobotManipulationSimulator::getColor(uint id) {
  return C->bodies(id)->shapes(0)->color;
}



// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================
// ==============================================================================

// --------------------------------
// STATE INFORMATION
// --------------------------------


double* RobotManipulationSimulator::getPosition(uint id) {
  return C->bodies(id)->X.pos.p;
}


// orientation is 2d:  orientation(0) = angle to z axis,  orientation(1) = angle of projection to x/y plane
void RobotManipulationSimulator::getOrientation(arr& orientation, uint id) {
  orientation.resize(2);
  
  if (getOrsType(id) == ors::sphereST) {
    orientation.setUni(0.);
    return;
  }
  
//   cout<<C->bodies(id)->name<<endl;
  
  ors::Quaternion rot;
  rot = C->bodies(id)->X.rot;
  
  ors::Vector upvec_z; double maxz=-2;
  if((rot*VEC_x)(2)>maxz){ upvec_z=VEC_x; maxz=(rot*upvec_z)(2); }
  if((rot*VEC_y)(2)>maxz){ upvec_z=VEC_y; maxz=(rot*upvec_z)(2); }
  if((rot*VEC_z)(2)>maxz){ upvec_z=VEC_z; maxz=(rot*upvec_z)(2); }
  if((rot*(-VEC_x))(2)>maxz){ upvec_z=-VEC_x; maxz=(rot*upvec_z)(2); }
  if((rot*(-VEC_y))(2)>maxz){ upvec_z=-VEC_y; maxz=(rot*upvec_z)(2); }
  if((rot*(-VEC_z))(2)>maxz){ upvec_z=-VEC_z; maxz=(rot*upvec_z)(2); }
  double angle_z = acos(maxz);
  if (angle_z < 0.0001)
    angle_z = 0.;
//   if (angle_z>MT_PI/4) {
//     PRINT(MT_PI/4);
//     PRINT((rot*VEC_x)(2));
//     PRINT((rot*VEC_y)(2));
//     PRINT((rot*VEC_z)(2));
//     PRINT((rot*(-VEC_x))(2));
//     PRINT((rot*(-VEC_y))(2));
//     PRINT((rot*(-VEC_z))(2));
//     PRINT(acos((rot*VEC_x)(2)));
//     PRINT(acos((rot*VEC_y)(2)));
//     PRINT(acos((rot*VEC_z)(2)));
//     PRINT(acos((rot*(-VEC_x))(2)));
//     PRINT(acos((rot*(-VEC_y))(2)));
//     PRINT(acos((rot*(-VEC_z))(2)));
//     watch();
//   }
//   CHECK((angle_z<=MT_PI/2)  &&  (angle_z>=0), "invalid angle_z  (upvec_z="<<upvec_z<<", z="<<maxz<<")");
  orientation(0) = angle_z;
  
  ors::Vector upvec_x; double maxx=-2;
  if((rot*VEC_x)(0)>maxx){ upvec_x=VEC_x; maxx=(rot*upvec_x)(0); }
  if((rot*VEC_y)(0)>maxx){ upvec_x=VEC_y; maxx=(rot*upvec_x)(0); }
  if((rot*VEC_z)(0)>maxx){ upvec_x=VEC_z; maxx=(rot*upvec_x)(0); }
  if((rot*(-VEC_x))(0)>maxx){ upvec_x=-VEC_x; maxx=(rot*upvec_x)(0); }
  if((rot*(-VEC_y))(0)>maxx){ upvec_x=-VEC_y; maxx=(rot*upvec_x)(0); }
  if((rot*(-VEC_z))(0)>maxx){ upvec_x=-VEC_z; maxx=(rot*upvec_x)(0); }
  double angle_xy = atan((rot*upvec_x)(1) / maxx);
  if (angle_xy < 0.0001)
    angle_xy = 0.;
//   CHECK((angle_xy<=MT_PI/4)  &&  (angle_xy>=0), "invalid angle_xy (upvec_x="<<upvec_x<<", x="<<maxx<<")");
  orientation(1) = angle_xy;
}



bool RobotManipulationSimulator::isUpright(uint id) {
  // balls are always upright
  if (getOrsType(id) == ors::sphereST)
    return true;
  
  double TOLERANCE = 0.05; // in radians
  
  arr orientation;
  getOrientation(orientation, id);
//   cout << id << " angle = " << angle << endl;
  if (fabs(orientation(0)) < TOLERANCE)
    return true;
  else
    return false;
}

uint RobotManipulationSimulator::getInhand(uint man_id){
  ors::Joint* e;
  if(!C->bodies(man_id)->outLinks.N) return UINT_MAX;
  e=C->bodies(man_id)->outLinks(0);
  return e->to->index;
}

uint RobotManipulationSimulator::getInhand() {
  return getInhand(convertObjectName2ID("fing1c"));
}


void RobotManipulationSimulator::getObjectsAbove(uintA& list,const char *obj_name){
  list.clear();
  
  ors::Body* obj_body = C->getBodyByName(obj_name);
  double obj_rad = 0.5 * getSize(obj_body->index)[2];
  
  uint i;
  uint body_id__a, body_id__b;
  double TOL_COEFF = 0.8;
  double other_rad;
  
  uintA others;
  getObjects(others);
  ors::Body* other_body;
//   C->reportProxies();
  
  for(i=0;i<C->proxies.N;i++){
    if (C->proxies(i)->a  == -1  ||  C->proxies(i)->b  == -1) // on earth
      continue;
    body_id__a = C->shapes(C->proxies(i)->a)->body->index;
    body_id__b = C->shapes(C->proxies(i)->b)->body->index;
    if (body_id__a == obj_body->index) {
      other_body = C->bodies(body_id__b);
    }
    else if (body_id__b == obj_body->index) {
      other_body = C->bodies(body_id__a);
    }
    else
      continue;
    
    double MAX_DISTANCE = 0.02;

    if (C->proxies(i)->d < MAX_DISTANCE) { // small enough distance
      other_rad = 0.5 * getSize(other_body->index)[2];
      // z-axis (height) difference big enough
      if (other_body->shapes(0)->X.pos(2) - obj_body->shapes(0)->X.pos(2)   >   TOL_COEFF * (obj_rad + other_rad)) {
        if (other_body->index == getTableID()  ||  obj_body->index == getTableID()) {
          list.setAppend(other_body->index);
        }
        else {
          // x-axis difference small enough
          if (fabs(other_body->shapes(0)->X.pos(0) - obj_body->shapes(0)->X.pos(0))   <   0.9 * (obj_rad + other_rad)) {
            // y-axis difference small enough
            if (fabs(other_body->shapes(0)->X.pos(1) - obj_body->shapes(0)->X.pos(1))   <   0.9 * (obj_rad + other_rad)) {
              list.setAppend(other_body->index);
            }
          }
        }
      }
    }
  }
}

void RobotManipulationSimulator::getObjectsAbove(uintA& list,const uint obj_id) {
  getObjectsAbove(list, convertObjectID2name(obj_id));
}



bool RobotManipulationSimulator::isClear(uint id) {
  uintA above_objects;
  getObjectsAbove(above_objects, id);
  return above_objects.N == 0;
}



bool RobotManipulationSimulator::freePosition(double x, double y, double radius) {
  uintA objects;
  getObjects(objects);
  objects.removeValue(getTableID());
//     cout<<"Asking for pos "<<x<<"/"<<y<<" within radius "<<radius<<endl;
  uint i;
  FOR1D(objects, i) {
    double* pos = getPosition(objects(i));
    double local_radius = radius;
    if (isBox(objects(i)))
      local_radius = 0.1;
    if (fabs(pos[0] - x) < local_radius)
        return false;
    if (fabs(pos[1] - y) < local_radius)
        return false;
  }
  return true;
}


double RobotManipulationSimulator::highestPosition(double x, double y, double radius, uint id_ignored) {
  uint DEBUG = 0;
  uintA objects;
  getObjects(objects);
  objects.removeValue(getTableID());
  if (DEBUG>0) {
    cout << "highestPosition:"<<endl;
    cout<<"Asking for pos "<<x<<"/"<<y<<" within radius "<<radius<<endl;
  }
  uint i;
  double max_z;
  double* table_pos = getPosition(getTableID());
  max_z = table_pos[2];
  if (DEBUG>0) {cout<<"table_z: "<<max_z<<endl;}
  FOR1D(objects, i) {
    if (objects(i) == id_ignored)
      continue;
    double* pos = getPosition(objects(i));
//         cout<<manipObjs(i)<<" has pos "<<pos[0]<<"/"<<pos[1]<<endl;
    if (fabs(pos[0] - x) < radius  &&  fabs(pos[1] - y) < radius) {
      if (DEBUG>0) cout<<"Object "<<objects(i)<<" within radius at height "<<pos[2]<<endl;
      if (pos[2] > max_z)
        max_z = pos[2];
    }
  }
  if (DEBUG>0) cout<<"max_z = "<<max_z<<endl;
  return max_z;
}


uint RobotManipulationSimulator::getContainedObject(uint box_id) {
  // inbox object has same position as box
  uintA boxes;
  getBoxes(boxes);
  double* box_pos = getPosition(box_id);
  uint i;
  
  uintA blocks;
  getBlocks(blocks);
  FOR1D(blocks, i) {
    double* obj_pos = getPosition(blocks(i));
    if ( fabs(obj_pos[0] - box_pos[0]) < 0.05
         &&  fabs(obj_pos[1] - box_pos[1]) < 0.05
         &&  (box_pos[2] - obj_pos[2]) > -0.02) {
      return blocks(i);
    }
  }
  
  uintA balls;
  getBalls(balls);
  FOR1D(balls, i) {
    double* obj_pos = getPosition(balls(i));
    if ( fabs(obj_pos[0] - box_pos[0]) < 0.05
         &&  fabs(obj_pos[1] - box_pos[1]) < 0.05
         &&  (box_pos[2] - obj_pos[2]) > -0.02) {
      return balls(i);
         }
  }
  return UINT_MAX;
}


bool RobotManipulationSimulator::isClosed(uint box_id) {
  CHECK(C->bodies(box_id)->shapes.N == 6, "isn't a box");
  if (TL::isZero(C->bodies(box_id)->shapes(5)->rel.pos(0))  &&  TL::isZero(C->bodies(box_id)->shapes(5)->rel.pos(1))
      &&  C->bodies(box_id)->shapes(5)->rel.pos(0) < 0.1)
    return true;
  else
    return false;
}


bool RobotManipulationSimulator::containedInBox(uint id) {
  uintA boxes;
  getBoxes(boxes);
  uint i;
  FOR1D(boxes, i) {
    if (getContainedObject(boxes(i)) == id)
      return true;
  }
  return false;
}


bool RobotManipulationSimulator::containedInClosedBox(uint id) {
  uintA boxes;
  getBoxes(boxes);
  uint i;
  FOR1D(boxes, i) {
    if (!isClosed(boxes(i)))
      continue;
    if (getContainedObject(boxes(i)) == id)
      return true;
  }
  return false;
}


bool RobotManipulationSimulator::inContact(uint a,uint b){
  if(C->getContact(a,b)) return true;
  return false;
}

void RobotManipulationSimulator::writeAllContacts(uint id) {
//   C->reportProxies();
//   void sortProxies(bool deleteMultiple=false,bool deleteOld=false);
  
  ors::Proxy *p;
  uint obj=C->getBodyByName(convertObjectID2name(id))->index;
  uint i;
  cout << convertObjectID2name(id) << " is in contact with ";
  for(i=0;i<C->proxies.N;i++)
    if(!C->proxies(i)->age)  // PROXIES SIND LEER!
      if (C->proxies(i)->d<0.02){
        p=C->proxies(i);
        if(p->a==(int)obj && p->b!=(int)obj) {
          cout << C->bodies(p->b)->name << " ";
        }
        if(p->b==(int)obj && p->a!=(int)obj) {
          cout << C->bodies(p->a)->name << " ";
        }
    }
    cout << endl;
}



void RobotManipulationSimulator::printObjectInfo() {
  uintA objects;
  getObjects(objects);
  uint i;
  FOR1D(objects, i) {
    cout << objects(i) << " " << convertObjectID2name(objects(i)) << endl;
  }
}



uint RobotManipulationSimulator::getHandID() {
  return convertObjectName2ID("fing1c");
}




// --------------------------------
//  OpenGL Displaying
// --------------------------------

void RobotManipulationSimulator::displayText(const char* text, uint t) {
  if(gl){
    for(;t--;) {
      gl->text.clr() <<text <<endl;
      gl->update();
    }
  }
}

