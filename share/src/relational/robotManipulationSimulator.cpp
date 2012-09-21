/*  
    Copyright 2008-2012   Tobias Lang
    
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
#include <relational/utilTL.h>

#include "robotManipulationSimulator.h"
#include <sstream>
#include <limits>


// SPECIFIC OBJECT KNOWLEDGE
#define BLOCK_SMALL 0.04
#define BLOCK_BIG 0.06
#define BLOCK_VERY_BIG 0.08
#define SMALL_HEIGHT_STEP 0.05
// table + neutralHeightBonus = NEUTRAL_HEIGHT
#define NEUTRAL_HEIGHT_BONUS 0.5
#define HARD_LIMIT_DIST_Y -0.8



/************************************************
* 
*     Noise for actions
* 
*     THIS IS WHERE YOU CAN BRING IN YOUR OWN IDEAS OF "NOISE" ETC. !!!
* 
************************************************/

double DROP_TARGET_NOISE__BLOCK_ON_SMALL_BLOCK = 0.001;
double DROP_TARGET_NOISE__BLOCK_ON_BIG_BLOCK = 0.003;
double DROP_TARGET_NOISE__BALL_ON_BIG_BLOCK = 0.0066;
double DROP_TARGET_NOISE__BALL_ON_SMALL_BLOCK = 0.02;
double DROP_TARGET_NOISE__ON_BALL = 0.01;
double GRAB_UNCLEAR_OBJ_FAILURE_PROB = 0.4;



/************************************************
* 
*     Low-level control
* 
************************************************/

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
      gl->text.clear();
      gl->text <<text <<endl;
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
  updateState(TVs, *C);
  updateChanges(TVs);
  bayesianControl(TVs,dq,W);
//   if (q.N==0) q.resizeAs(dq); // TOBIAS-Aenderung
  q += dq;
  oneStep(q,C,ode,swift,gl,revel,text);
#endif
}



/************************************************
* 
*     Administration
* 
************************************************/

// How many time-steps until action fails
#define SEC_ACTION_ABORT 1000

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


arr q0, W;

void drawEnv(void* horst){
#ifdef MT_FREEGLUT
  glStandardLight(horst);
//   glDrawFloor(4.,1,1,1);
  glDrawFloor(4., 108./255., 123./255., 139./255.);
#endif
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
  gl->resize(800, 600);
//   gl->resize(1024, 600);
  gl->camera.setPosition(2.5,-7.5,2.3);  // position of camera
  gl->camera.focus(-0.25, -0.6, 1.1);  // rotate the frame to focus the point (x,y,z)
  gl->camera.upright();
  gl->update();
#endif
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





/************************************************
* 
*     Standard simulation
* 
************************************************/


void RobotManipulationSimulator::simulate(uint t, const char* message){
  String msg_string(message);
  arr q;
  TaskVariableList local_TVs;
  local_TVs.clear();
  C->getJointState(q);
  bool change = true;
  for(;t--;){
    MT::String send_string;
    if (msg_string.N == 0) {
      if (t%20==0)
        change = !change;
      if (change)
        send_string << "S";
    }
    else
      send_string << msg_string;
    //     send_string << msg_string << "     \n\n(time " << t << ")";
    controlledStep(q,W,C,ode,swift,gl,revel,local_TVs,send_string);
  }
}


void RobotManipulationSimulator::watch(){
#ifdef MT_FREEGLUT
  gl->text.clear() <<"Watch" <<endl;
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







/************************************************
* 
*     General object information (state-independent)
* 
************************************************/


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


void RobotManipulationSimulator::getCylinders(uintA& cylinders) {
  cylinders.clear();
  // assuming that all objects start with "o"
  std::stringstream ss;
  uint i;
  for (i=1;i<=numObjects;i++) {
    ss.str("");
    ss << "o" << i;
    ors::Body* n = C->getBodyByName(ss.str().c_str());
    if (n->shapes.N == 1) {
      if (n->shapes(0)->type == ors::cylinderST)
        cylinders.append(n->index);
    }
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


MT::String RobotManipulationSimulator::getColorString(uint obj) {
  double red[3];  red[0]=1.0;  red[1]=0.0;   red[2]=0.0;
  double green[3];  green[0]=0.2;  green[1]=1.0;   green[2]=0.0;
  double orange[3];  orange[0]=1.0;  orange[1]=0.5;   orange[2]=0.0;
  double yellow[3];  yellow[0]=1.0;  yellow[1]=1.0;   yellow[2]=0.0;
  double blue[3];  blue[0]=.0;  blue[1]=.0;   blue[2]=1.0;
  double brown[3];  brown[0]=.5;  brown[1]=.3;   brown[2]=.15;
  double yellow_green[3];  yellow_green[0]=.8;  yellow_green[1]=1.;   yellow_green[2]=.0;
  double grey[3];  grey[0]=.6;  grey[1]=.5;   grey[2]=.5;
  double light_blue[3];  light_blue[0]=.4;  light_blue[1]=1.;   light_blue[2]=1.;
  double purple[3];  purple[0]=.4;  purple[1]=0.;   purple[2]=.5;
  double dark_red[3];  dark_red[0]=.7;  dark_red[1]=0.05;   dark_red[2]=.05;
  double dark_blue[3];  dark_blue[0]=.05;  dark_blue[1]=0.;   dark_blue[2]=.7;
  double rose[3];  rose[0]=1.0;  rose[1]=0.5;   rose[2]=.75;

  uint i;
  
  double* color = getColor(obj);
  MT::String name;

  for (i=0; i<3; i++)
  if(!TL::areEqual(color[i], red[i])) break;
  if (i==3) {name = "red";}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], green[i])) break;
  if (i==3) {name = "green";}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], orange[i])) break;
  if (i==3) {name = "orange";}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], yellow[i])) break;
  if (i==3) {name = "yellow";}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], blue[i])) break;
  if (i==3) {name = "blue";}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], brown[i])) break;
  if (i==3) {name = "brown";}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], yellow_green[i])) break;
  if (i==3) {name = "yellow-green";}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], grey[i])) break;
  if (i==3) {name = "grey";}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], light_blue[i])) break;
  if (i==3) {name = "light blue";}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], rose[i])) break;
  if (i==3) {name = "rose";}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], purple[i])) break;
  if (i==3) {name = "purple";}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], dark_red[i])) break;
  if (i==3) {name = "dark red";}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], dark_blue[i])) break;
  if (i==3) {name = "dark blue";}
  
  return name;
}












/************************************************
* 
*     State information
* 
************************************************/

double* RobotManipulationSimulator::getPosition(uint id) {
  return C->bodies(id)->X.pos.p;
}


void RobotManipulationSimulator::getTablePosition(double& x1, double& x2, double& y1, double& y2) {
  double* pos = getPosition(getTableID());
  double* size = getSize(getTableID());
  x1 = pos[0] - size[0]/2;
  x2 = pos[0] + size[0]/2;
  y1 = pos[1] - size[1]/2;
  y1 = pos[1] + size[1]/2;
}


void RobotManipulationSimulator::getObjectPositions(arr& positions) {
  uint i, k;
  uintA objs;
  getObjects(objs);
  positions.resize(objs.N, 3);
  FOR1D(objs, i) {
    double* local_position = getPosition(objs(i));
    for (k=0; k<3; k++) {
      positions(i, k) = local_position[k];
    }
  }
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


void RobotManipulationSimulator::getObjectAngles(arr& angles) {
  uint i, k;
  uintA objs;
  getObjects(objs);
  angles.resize(objs.N, 2);
  FOR1D(objs, i) {
    arr orientation;
    getOrientation(orientation, objs(i));
    CHECK(orientation.N == 2, "too many angles");
    FOR1D(orientation, k) {
      angles(i, k) = orientation(k);
      if (angles(i, k) < 0.00001)
        angles(i, k) = 0.;
    }
  }
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


double RobotManipulationSimulator::getHeight(uint id) {
  return getPosition(id)[2];
}


double RobotManipulationSimulator::getOverallHeight(uintA& objects) {
  uint i;
  double height = 0.;
  uint obj_inhand = getInhand();
  double table_height = getHeight(getTableID());
  cout<<"table "<<table_height<<endl;
  FOR1D(objects, i) {
    if (objects(i) == obj_inhand) {
      cout << objects(i) << " 0 (inhand)"<<endl;
      continue;
    }
    double o_height = (getHeight(objects(i)) - table_height);
    cout<<objects(i)<<" "<<o_height<<endl;
    height += o_height;
  }
//   height /= 1.0 * objects.N;
  return height;
}


uint RobotManipulationSimulator::getInhand(uint man_id){
  ors::Joint* e;
  if(!C->bodies(man_id)->outLinks.N) return TL::UINT_NIL;
  e=C->bodies(man_id)->outLinks(0);
  return e->to->index;
}


uint RobotManipulationSimulator::getInhand() {
  return getInhand(convertObjectName2ID("fing1c"));
}


uint RobotManipulationSimulator::getHandID() {
  return convertObjectName2ID("fing1c");
}


void RobotManipulationSimulator::getObjectsOn(uintA& list,const char *obj_name){
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

   //C->reportProxies();
  
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


void RobotManipulationSimulator::getObjectsOn(uintA& list,const uint obj_id) {
  getObjectsOn(list, convertObjectID2name(obj_id));
}


bool RobotManipulationSimulator::isClear(uint id) {
  uintA above_objects;
  getObjectsOn(above_objects, id);
  return above_objects.N == 0;
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


void RobotManipulationSimulator::getObjectsClose(uintA& list, uint obj){
  list.clear();
  double* pos = getPosition(obj);
  uint i;
  uintA objects;
  getObjects(objects);
  FOR1D(objects, i) {
    if (objects(i) == obj) continue;
    double* pos_other = getPosition(objects(i));
    if (fabs(pos[2] - pos_other[2]) > 0.03) continue;
//     PRINT(fabs(pos[0] - pos_other[0]));
//     PRINT(fabs(pos[1] - pos_other[1]));
    if (fabs(pos[0] - pos_other[0]) > 0.15) continue;
    if (fabs(pos[1] - pos_other[1]) > 0.15) continue;
    list.append(objects(i));
  }
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
  return TL::UINT_NIL;
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












/************************************************
* 
*     Actions - grab
* 
************************************************/
  

void RobotManipulationSimulator::grab_final(const char *manipulator,const char *obj_grabbed, const char* message){
  ors::Body *obj=C->getBodyByName(obj_grabbed);
  bool isTable = obj->index == getTableID();
  if (isTable) {
    cout<<"Cannot grab table"<<endl;
    return;
  }
  bool inClosedBox = containedInClosedBox(convertObjectName2ID(obj_grabbed));
  if (inClosedBox) {
    cout<<"Object "<<obj_grabbed<<" is inside of a closed box and cannot be grabbed."<<endl;
    return;
  }
  bool isHand = obj->index == getHandID();
  if (isHand) HALT("grab_final("<<obj_grabbed<<") --> cannot grab my own hand!");

  uintA list;
  getObjectsOn(list, obj_grabbed);
  bool object_is_clear = list.N == 0;
  
  MT::String msg_string(message);
  if (msg_string.N == 0) {
    msg_string << "grab "<<obj_grabbed;
  }
  
  DefaultTaskVariable x("endeffector", *C, posTVT, manipulator, 0, 0, 0, ARR());
  x.setGainsAsAttractor(20, .2);
  x.y_prec=1000.;
  
  TaskVariableList local_TVs;
  local_TVs.append(&x);
  
  uint t;
  arr q, dq;
  C->getJointState(q);
  
  // (1) drop object if one is in hand
  //   dropInhandObjectOnTable(message);
  uint id_grabbed = getInhand();
  if (id_grabbed != TL::UINT_NIL) {
    // move a bit towards new object
    for(t=0;t<10;t++){
      x.y_target.setCarray(obj->X.pos.p,3);
      if (isTable) {x.y_target(2) = neutralHeight-0.1;}
      MT::String send_msg;
      send_msg << msg_string /*<< "      \n\n(time " << t << ")"*/;
      controlledStep(q,W,C,ode,swift,gl,revel,local_TVs,send_msg);
      // TODO passt das? Ueberall state durch "active" ersetzt
//       if(x.active==1 || C->getContact(x.i,obj->index)) break;
      if(x.active==1 || C->getContact(x.i,obj->index)) break;
    }
    if(C->bodies(id_grabbed)->inLinks.N){
      ors::Joint* e=C->bodies(id_grabbed)->inLinks(0);
      del_edge(e,C->bodies,C->joints,true);
    }
  }
    
  // (2) move towards new object
  C->getJointState(q);
  for(t=0; t<Tabort; t++){
    x.y_target.setCarray(obj->X.pos.p, 3);
    if (isTable) {x.y_target(2) = neutralHeight-0.1;}
    MT::String send_msg;
    send_msg << msg_string /*<< "      \n\n(time " << t << ")"*/;
    controlledStep(q,W,C,ode,swift,gl,revel,local_TVs,send_msg);
    if (gl) {gl->update();}
    double dist = norm(x.y - x.y_target);
//     PRINT(x.y);
//     PRINT(x.y_target);
//     PRINT(dist);
//     PRINT(C->getContact(x.i, obj->index));
    if(dist <.05 || C->getContact(x.i, obj->index)) break;
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
    controlledStep(q,W,C,ode,swift,gl,revel,local_TVs,send_msg);
    if (gl) {gl->update();}
    double dist = norm(x.y - x.y_target);
    if(dist<.05) break;
    
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
  
  
  relaxPosition();
    
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


void RobotManipulationSimulator::grabHere(const char* message) {
  #ifdef MT_SWIFT
  MT::String msg_string(message);
  if (msg_string.N == 0) {
    msg_string << "grabHere: ";
  }

  uint finger = convertObjectName2ID("fing1c");
  // (1) drop object if one is in hand
  dropObject(finger);
  
  double min_distance = std::numeric_limits<double>::max();
  uint closest_object = 0;
  for (uint i = 0; i< C->proxies.N; i++) {
    if (C->proxies(i)->a == finger || C->proxies(i)->b == finger) {
       if (C->proxies(i)->d < min_distance) {
         min_distance = C->proxies(i)->d; 
         closest_object = (C->proxies(i)->a == finger ? C->proxies(i)->b : C->proxies(i)->a );
       }
    }
  }
  if (min_distance > 10e-4) {
    msg_string << "nothing to grab";
  }
  else {
    C->glueBodies(C->getBodyByName("fing1c"), C->getBodyByName(convertObjectID2name(closest_object)));
    msg_string << "Grabed obj " << convertObjectID2name(closest_object); 
  }
  
  #endif
}




/************************************************
* 
*     Actions - drop
* 
************************************************/

void RobotManipulationSimulator::dropObjectAbove_final(const char *obj_dropped, const char *obj_below, const char* message){
  MT::String msg_string(message);
  if (msg_string.N == 0) {
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
  z.setGainsAsAttractor(20,.2);
  z.y_prec=1000.;
  z.y_target.resize(1);  z.y_target = 1.;
    
  c.setGainsAsAttractor(20,.1);
  c.y_prec=10000.;
  if(!swift) c.active=false;
  
  uint t;
  arr q,dq;
  C->getJointState(q);

  TaskVariableList local_TVs;
  local_TVs.append(&o);
  local_TVs.append(&z);
  local_TVs.append(&r);
  local_TVs.append(&c);  
  
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
  updateState(local_TVs, *C);
  o.y_target(2) += .3;
  for(t=0;t<Tabort;t++){
    if (o.y_target(2) < neutralHeight)
      o.y_target(2) += 0.05;
    MT::String send_string;
    send_string << msg_string /*<< "     \n\n(time " << t << ")"*/;
    controlledStep(q,W,C,ode,swift,gl,revel,local_TVs,send_string);
    double diff = norm(o.y - o.y_target);
    if (diff < 0.05) break;
  }
  if(t==Tabort){ indicateFailure(); return; }
  
  
  
  // Phase 2: above object
  o.y_target(0) = x_target;
  o.y_target(1) = y_target;
  // WHERE TO GO ABOVE
  o.y_target(2) = highestPosition(o.y_target(0), o.y_target(1), 0.06, obj_dropped1_index) + .2;
  for(t=0;t<Tabort;t++){
    MT::String send_string;
    send_string << msg_string /*<< "     \n\n(time " << t << ")"*/;
    controlledStep(q,W,C,ode,swift,gl,revel,local_TVs,send_string);
    double diff = norm(o.y - o.y_target);
    if (diff < 0.05) break;
  }
  if(t==Tabort){ indicateFailure(); return; }

  //turn off collision avoidance
  c.active=false;

  
  
  // Phase 3: down
  // IMPORTANT PARAM: set distance to target (relative height-distance in which "hand is opened" / object let loose)
  double Z_ADD_DIST = getSize(obj_dropped1_index)[0]/2 + .03;
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
    controlledStep(q,W,C,ode,swift,gl,revel,local_TVs,send_string);
    double diff = norm(o.y - o.y_target);
    if (diff < 0.001) break;
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


void RobotManipulationSimulator::dropObject(uint manipulator_id) {
  if (C->bodies(manipulator_id)->outLinks.N == 0)
    return;
  CHECK(C->bodies(manipulator_id)->outLinks.N == 1, "too many objects in hand");
  del_edge(C->bodies(manipulator_id)->outLinks(0), C->bodies, C->joints, true);
}


void RobotManipulationSimulator::dropInhandObjectOnTable(const char* message) {
  uint id_grabbed = getInhand();
  if (TL::UINT_NIL == id_grabbed)
    return;
  dropObjectAbove(id_grabbed, getTableID(), message);
  relaxPosition(message);
}


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








/************************************************
* 
*     Actions - Posture control
* 
************************************************/

void RobotManipulationSimulator::relaxPosition(const char* message) {
  MT::String msg_string(message);
  if (msg_string.N == 0) {
    msg_string << "Relax position";
  }
  PRINT(msg_string);
  
  uint inhand_id = getInhand();
  ors::Shape* s = NULL;
  // simplification: set off contacts for inhand-object
  if (inhand_id != TL::UINT_NIL) {
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
  TaskVariableList local_TVs;
  local_TVs.append(&x);

  uint t;
  for(t=0;t<Tabort;t++){
    MT::String send_string;
    send_string << msg_string /*<< "     \n\n(time " << t << ")"*/;
//     controlledStep(q,W,send_string);
    controlledStep(q,W,C,ode,swift,gl,revel,local_TVs,send_string);
    double diff = norm(x.y - x.y_target);
    if (diff < 0.5) break;
//     if(x.active==1) break;
  }
  PRINT(t);
  exit(0);
  
  // simplification: set on contacts for inhand-object
  if (s!=NULL) {
    s->cont = true;
#ifdef MT_SWIFT
    swift->initActivations(*C);
#endif
  }
  
  if(t==Tabort){ indicateFailure(); return; }
}


void RobotManipulationSimulator::moveToPosition(const arr& pos, const char* message) {
#  ifdef MT_ODE
	CHECK(pos.N == 3 && pos.nd == 1, "Not a valid position array");
  MT::String msg_string(message);
  if (msg_string.N == 0) {
    msg_string << "move to position "<< pos;
  }
  
  ors::Body* obj = C->bodies(convertObjectName2ID("fing1c"));
  // move manipulator towards box
  DefaultTaskVariable x("endeffector",*C,posTVT,"fing1c",0,0,0,0);
  x.setGainsAsAttractor(20,.2);
  x.y_prec=1000.;
  TaskVariableList TVs;
  TVs.append(&x);

	double epsilon = 10e-3;

  uint t;
  arr q,dq;
  C->getJointState(q);
  for(t=0;t<Tabort;t++){
    x.y_target.setCarray(pos.p,3);
    MT::String send_string;
    send_string << msg_string;
    controlledStep(q,W,C,ode,swift,gl,revel,TVs,send_string);
		if ((obj->X.pos - pos).length() < epsilon) break;
  }
  if(t==Tabort){ indicateFailure(); return; }
  simulate(30, msg_string);
  
#ifdef MT_SWIFT
  swift->initActivations(*C);
#endif
#endif
}





/************************************************
* 
*     Actions - Boxes
* 
************************************************/


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
  if (msg_string.N == 0) {
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
    double diff = norm(x.y - x.y_target);
    if (diff < 0.01) break;
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
  if (msg_string.N == 0) {
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
    double diff = norm(x.y - x.y_target);
    if (diff < 0.01) break;
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







 /************************************************
  * 
  *     OpenGL Displaying
  * 
  ************************************************/

void RobotManipulationSimulator::displayText(const char* text, uint t) {
  if(gl){
    for(;t--;) {
      gl->text.clear() <<text <<endl;
      gl->update();
    }
  }
}





/************************************************
  * 
  *     Creating ORS-objects
  * 
  ************************************************/


namespace relational {
    
void generateOrsBlocksSample(ors::Graph& ors, const uint numOfBlocks) {
  MT::Array<arr> pos;
  generateBlocksSample(pos, numOfBlocks);
	generateOrsFromSample(ors, pos);
}


void generateOrsFromSample(ors::Graph& ors, const MT::Array<arr>& sample) {
  //for (int i = ors.bodies.N - 1; i >= 0; --i) {
    //if (ors.bodies(i)->name.p[0] == 'o') {
      //ors.bodies.remove(i);  
    //}
  //}
  for (uint i = 0; i < sample.N; i+=2) {
    ors::Body* body = new ors::Body;
    createCylinder(*body, sample(0,i), ARR(1., 0., 0.), sample(0,i+1)); 
		MT::String name;
		name << "o7" << i;
		cout << name << endl;
    body->name = name;
    ors.bodies.append(body);
  }
}


void generateBlocksSample(MT::Array<arr>& sample, const uint numOfBlocks) {
  sample.clear();
  for (uint i = 0; i < numOfBlocks; ++i) {
    arr center3d = ARR(0., -.8) + randn(2,1) * 0.3;

    int t = rand() % 100;
    double blocksize = 0.08;// + (rand() % 100) / 100000.;
    double towersize = 0.69 + blocksize;
    center3d.append(0.69 + 0.5*blocksize);
    center3d.resize(3);

    sample.append(center3d);
    //sample.append(ARR(0.1, 0.1, blocksize, 0.0375));
    sample.append(ARR(blocksize));
    while (t < 50 && i < numOfBlocks-1) {
      i++;
      center3d = center3d + randn(3,1) * 0.02;
      double blocksize = 0.08;// + (rand() % 100) / 1000.;
      center3d(2) = 0.5*blocksize + towersize;
      towersize += blocksize;

      sample.append(center3d);
      //sample.append(ARR(0.1, 0.1, blocksize, 0.0375));
      sample.append(ARR(blocksize));

      t = rand() % 100;
    }
  }
	sample.reshape(1,2*numOfBlocks);
  //sample.reshape(1,numOfBlocks);
}


void createCylinder(ors::Body& cyl, const ors::Vector& pos, const arr& color) {
  arr size = ARR(0.1, 0.1, 0.108, 0.0375);
  createCylinder(cyl, pos, color, size);
}


void createCylinder(ors::Body& cyl, const ors::Vector& pos, const arr& color, const arr& size) {
  ors::Transformation t;
  t.pos = pos;
  ors::Shape* s = new ors::Shape();
  for (uint i = 0; i < 4; ++i) { s->size[i] = size(i);}
  s->type = ors::cylinderST;
  for (uint i = 0; i < 3; ++i) s->color[i] = color(i);
  s->body = &cyl;
  
  cyl.shapes.append(s);
  cyl.X = t; 
}

}  // namespace relational

