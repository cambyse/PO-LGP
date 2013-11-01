/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

/**
 * @file
 * @ingroup group_ors
 */
/**
 * @addtogroup group_ors
 * @{
 */


#undef abs
#include <algorithm>
#include <sstream>
#include "ors.h"

#ifndef MT_ORS_ONLY_BASICS
#  include <Core/registry.h>
//#  include <Gui/plot.h>
#endif
#ifdef MT_extern_ply
#  include <extern/ply/ply.h>
#endif

#define ORS_NO_DYNAMICS_IN_FRAMES

#define SL_DEBUG_LEVEL 1
#define SL_DEBUG(l, x) if(l<=SL_DEBUG_LEVEL) x;

#define Qstate

void lib_ors(){ cout <<"force loading lib/ors" <<endl; }

#define LEN .2

#ifndef MT_ORS_ONLY_BASICS

//===========================================================================
//
// Body implementations
//


/* dm 15.06.2006--these had to be changed with the switch to ODE 0.6
   void Body::copyFrameToOde(){
   CHECK(X.r.isNormalized(), "quaternion is not normalized!");
   CP3(b->posr.pos, X.p);                // dxBody changed in ode-0.6 ! 14. Jun 06 (hh)
   CP4(b->q, X.r); dQtoR(b->q, b->posr.R);
   CP3(b->lvel, X.v);
   CP3(b->avel, X.w);
   }
   void Body::getFrameFromOde(){
   CP3(X.p.v, b->posr.pos);
   CP4(X.r.q, b->q);
   CP3(X.v.v, b->lvel);
   CP3(X.w.v, b->avel);
   CHECK(X.r.isNormalized(), "quaternion is not normalized!");
   }
*/

ors::Body::Body() { reset(); }

ors::Body::Body(const Body& b) { reset(); *this=b; }

ors::Body::Body(Graph& G, const Body* copyBody) {
  reset();
  if(copyBody) *this=*copyBody;
  index=G.bodies.N;
  G.bodies.append(this);
}

ors::Body::~Body() {
  reset();
  listDelete(inLinks);
  listDelete(outLinks);
  listDelete(shapes);
}

void ors::Body::reset() {
  listDelete(ats);
  X.setZero();
  type=dynamicBT;
  shapes.memMove=true;
  com.setZero();
#if 1
  mass = 0.;
  inertia.setZero();
#else
  mass=1.;
  inertia.setId();
  inertia *= .3;
#endif
}

void ors::Body::parseAts() {
  //interpret some of the attributes
  arr x;
  MT::String str;
  ats.getValue<Transformation>(X, "X");
  ats.getValue<Transformation>(X, "pose");
  
  //mass properties
//  double d;
//  if(ats.getValue<double>(d, "mass")) {
//    mass=d;
//    inertia.setId();
//    inertia *= .2*d;
//  }
  
  type=dynamicBT;
  if(ats.getValue<bool>("fixed"))      type=staticBT;
  if(ats.getValue<bool>("static"))     type=staticBT;
  if(ats.getValue<bool>("kinematic"))  type=kinematicBT;
  
  // SHAPE handling {{{
  Item* item;
  // a mesh which consists of multiple convex sub meshes creates multiple
  // shapes that belong to the same body
  item = ats.getItem("mesh");
  if (item) {
    MT::String* filename = item->value<MT::String>();

    // if mesh is not .obj we only have one shape
    if (MT::String(*filename).getLastN(3) != "obj") {
      Shape* shape = new Shape(this);
    }

    // if .obj file create Shape for all submeshes
    else {
      auto subMeshPositions = getSubMeshPositions(*filename);
      for (auto parsing_pos : subMeshPositions) {
        Shape* shape = new Shape(this);
        shape->mesh.parsing_pos_start = std::get<0>(parsing_pos);
        shape->mesh.parsing_pos_end = std::get<1>(parsing_pos);
      }
    }
  }

  // add shape if there is no shape exists yet
  if(ats.getItem("type")) {
    if (shapes.N == 0) {
      Shape* shape = new Shape(this);
    }
  }

  // copy body attributes to shapes 
  const auto attributes = { "mesh", "type", "size", "color", "rel", "meshscale", "contact" };
  for (auto& shape : shapes) {
    for (const auto& itemname : attributes) {
      item=ats.getItem(itemname);
      if (item) {
        shape->ats.append(item->newClone()); 
      }
    }
    shape->parseAts();
  }
}

void ors::Body::write(std::ostream& os) const {
  if(!X.isZero()) os <<"pose=<T " <<X <<" > ";
  uint i; Item *a;
  for_list(i, a, ats)
  if(a->keys(0)!="X" && a->keys(0)!="pose") os <<*a <<' ';
}

void ors::Body::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("body '" <<name <<"' read error: in ");
  parseAts();
}

void ors::Body::read(const char* string) {
  std::istringstream str(string);
  read(str);
}

namespace ors {
std::ostream& operator<<(std::ostream& os, const Body& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Shape& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Joint& x) { x.write(os); return os; }
}

//===========================================================================
//
// Shape implementations
//

ors::Shape::Shape() { reset(); }

ors::Shape::Shape(Body *b) : body(b), ibody(b->index) {
  reset();
  body->shapes.append(this);
}

ors::Shape::Shape(const Shape& s) { body=NULL; *this=s; }

ors::Shape::Shape(Graph& G, Body *b, const Shape *copyShape) {
  reset();
  if(copyShape) *this = *copyShape;
  index=G.shapes.N;
  G.shapes.append(this);
  body=b;
  if(b){
    b->shapes.append(this);
    ibody=b->index;
  }
}

ors::Shape::~Shape() {
  reset();
  if(body) body->shapes.removeValue(this);
}

void ors::Shape::parseAts() {
  double d;
  arr x;
  MT::String str;
  ats.getValue<Transformation>(rel, "rel");
  if(ats.getValue<arr>(x, "size"))          { CHECK(x.N==4,"size=[] needs 4 entries"); memmove(size, x.p, 4*sizeof(double)); }
  if(ats.getValue<arr>(x, "color"))         { CHECK(x.N==3,"color=[] needs 3 entries"); memmove(color, x.p, 3*sizeof(double)); }
  if(ats.getValue<double>(d, "type"))       { type=(ShapeType)(int)d;}
  if(ats.getValue<bool>("contact"))         { cont=true; }
  if(ats.getValue<MT::String>(str, "mesh")) { mesh.readFile(str); }
  if(ats.getValue<double>(d, "meshscale"))  { mesh.scale(d); }

  //add inertia to the body
  if(body) {
    Matrix I;
    double mass=-1.;
    switch(type) {
      case sphereST:   inertiaSphere(I.p(), mass, 1000., size[3]);  break;
      case boxST:      inertiaBox(I.p(), mass, 1000., size[0], size[1], size[2]);  break;
      case cappedCylinderST:
      case cylinderST: inertiaCylinder(I.p(), mass, 1000., size[2], size[3]);  break;
      case noneST:
      default: ;
    }
    if(mass>0.){
      body->mass += mass;
      body->inertia += I;
    }
  }

}

void ors::Shape::reset() {
  type=noneST;
  size[0]=size[1]=size[2]=size[3]=1.;
  color[0]=color[1]=color[2]=.8;
  contactOrientation.setZero();
  listDelete(ats);
  rel.setZero();
  mesh.V.clear();
  cont=false;
}

void ors::Shape::write(std::ostream& os) const {
  os <<"type=" <<type <<' ';
  if(!rel.isZero()) os <<"rel=<T " <<rel <<" > ";
  uint i; Item *a;
  for_list(i,a,ats)
  if(a->keys(0)!="rel" && a->keys(0)!="type") os <<*a <<' ';
}

void ors::Shape::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("shape read error");
  parseAts();
}

uintA stringListToShapeIndices(const MT::Array<const char*>& names, const MT::Array<ors::Shape*>& shapes) {
  uintA I(names.N);
  for(uint i=0; i<names.N; i++) {
    ors::Shape *s = listFindByName(shapes, names(i));
    if(!s) HALT("shape name '"<<names(i)<<"' doesn't exist");
    I(i) = s->index;
  }
  return I;
}

void makeConvexHulls(ShapeL& shapes){
  for_list_(ors::Shape, s, shapes) s->mesh.makeConvexHull();
}


//===========================================================================
//
// Joint implementations
//

ors::Joint::Joint()
  : index(0), qIndex(-1), ifrom(0), ito(0), from(NULL), to(NULL), coupledTo(NULL) { reset(); }

ors::Joint::Joint(const Joint& j)
  : index(0), qIndex(-1), ifrom(0), ito(0), from(NULL), to(NULL), coupledTo(NULL) { reset(); *this=j; }

ors::Joint::Joint(Graph& G, Body *f, Body *t, const Joint* copyJoint)
  : index(0), qIndex(-1), ifrom(0), ito(0), from(NULL), to(NULL), coupledTo(NULL){
  reset();
  if(copyJoint) *this=*copyJoint;
  index=G.joints.N;
  G.joints.append(this);
  from=f;  ifrom=f->index;
  to=t;    ito  =t->index;
  f->outLinks.append(this);
  t-> inLinks.append(this);
}

ors::Joint::~Joint() {
  reset();
  from->outLinks.removeValue(this);
  to->inLinks.removeValue(this);
}

void ors::Joint::parseAts() {
  //interpret some of the attributes
  double d;
  ats.getValue<Transformation>(A, "A");
  ats.getValue<Transformation>(A, "from");
  if(ats.getValue<bool>("BinvA")) B.setInverse(A);
  ats.getValue<Transformation>(B, "B");
  ats.getValue<Transformation>(B, "to");
  ats.getValue<Transformation>(Q, "Q");
  ats.getValue<Transformation>(Q, "q");
  ats.getValue<Transformation>(X, "X");
  if(ats.getValue<double>(d, "type")) type=(JointType)(int)d; else type=JT_hingeX;
  //axis
  arr axis;
  ats.getValue<arr>(axis, "axis");
  if(axis.N) {
    CHECK(axis.N==3,"");
    Vector ax(axis);
    Quaternion rot;  rot.setDiff(Vector_x, ax);
    A.rot = A.rot * rot;
    B.rot = -rot * B.rot;
  }
  //coupled to another joint requires post-processing by the Graph::read!!
  if(ats.getValue<MT::String>("coupledTo")) coupledTo=(Joint*)1;
}

uint ors::Joint::qDim() {
  if(type>=JT_hingeX && type<=JT_transZ) return 1;
  if(type==JT_trans3) return 3;
  if(type==JT_universal) return 2;
  if(type==JT_glue || type==JT_fixed) return 0;
  HALT("shouldn't be here");
  return 0;
}

void ors::Joint::write(std::ostream& os) const {
  if(!A.isZero()) os <<"from=<T " <<A <<" > ";
  if(!B.isZero()) os <<"to=<T " <<B <<" > ";
  if(!Q.isZero()) os <<"q=<T " <<Q <<" > ";
  uint i; Item *a;
  for_list(i,a,ats)
  if(a->keys(0)!="A" && a->keys(0)!="from"
      && a->keys(0)!="axis" //because this was subsumed in A during read
      && a->keys(0)!="B" && a->keys(0)!="to"
      && a->keys(0)!="Q" && a->keys(0)!="q") os <<*a <<' ';
}

void ors::Joint::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("joint (" <<from->name <<' ' <<to->name <<") read read error");
  parseAts();
}


ors::Proxy::Proxy() {
  colorCode = 0;
}

//===========================================================================
//
// Graph implementations
//

void ors::Graph::init(const char* filename) {
  MT::load(*this, filename, true);
  //calcBodyFramesFromJoints();
}

void ors::Graph::clear() {
  listDelete(proxies);
  listDelete(shapes);
  listDelete(joints);
  listDelete(bodies);
  q_dim=0;
  Qlin.clear(); Qoff.clear(); Qinv.clear();
  isLinkTree=false;
}

ors::Graph* ors::Graph::newClone() const {
  Graph *G=new Graph();
  G->q_dim=q_dim;
  G->Qlin = Qlin;  G->Qoff = Qoff;  G->Qinv = Qinv;
  listCopy(G->proxies, proxies);
  listCopy(G->joints, joints);
  listCopy(G->shapes, shapes);
  listCopy(G->bodies, bodies);
  graphMakeLists(G->bodies, G->joints);
  uint i;  Shape *s;  Body *b;
  for_list(i, s, G->shapes) {
    b=G->bodies(s->ibody);
    s->body=b;
    b->shapes.append(s);
  }
  return G;
}

void ors::Graph::operator=(const ors::Graph& G) {
  uint i;  Shape *s;  Body *b;
  q_dim=G.q_dim;
  Qlin = G.Qlin;  Qoff = G.Qoff;  Qinv = G.Qinv;
  listCopy(proxies, G.proxies);
  listCopy(joints, G.joints);
  listCopy(shapes, G.shapes);
  listCopy(bodies, G.bodies);
  graphMakeLists(bodies, joints);
  for_list(i, b, bodies) b->shapes.clear();
  for_list(i, s, shapes) {
    b=bodies(s->ibody);
    s->body=b;
    b->shapes.append(s);
  }
}

void ors::Graph::copyShapesAndJoints(const Graph& G) {
  uint i;  Shape *s;  Body *b;  Joint *j;
  for_list(i, s, shapes)(*s) = *G.shapes(i);
  for_list(i, j, joints)(*j) = *G.joints(i);
  for_list(i, b, bodies) b->shapes.clear();
  for_list(i, s, shapes) {
    b=bodies(s->ibody);
    s->body=b;
    b->shapes.append(s);
  }
  calcBodyFramesFromJoints();
}

/** @brief transforms (e.g., translates or rotates) the joints coordinate system):
  `adds' the transformation f to A and its inverse to B */
void ors::Graph::transformJoint(ors::Joint *e, const ors::Transformation &f) {
  e->A = e->A * f;
  e->B = -f * e->B;
}

void ors::Graph::makeLinkTree() {
  for_list_(Joint, j, joints) {
    for_list_(Shape, s, j->to->shapes)  s->rel = j->B * s->rel;
    for_list_(Joint, j2, j->to->outLinks) j2->A = j->B * j2->A;
    j->B.setZero();
  }
  isLinkTree=true;
}

/// [prelim] some kind of gyroscope
void ors::Graph::getGyroscope(ors::Vector& up) const {
  up.set(0, 0, 1);
  up=bodies(0)->X.rot*up;
}

/** @brief KINEMATICS: given the (absolute) frames of root nodes and the relative frames
    on the edges, this calculates the absolute frames of all other nodes (propagating forward
    through trees and testing consistency of loops). */
void ors::Graph::calcBodyFramesFromJoints() {
  Body *n;
  Joint *e;
  uint i, j;
  ors::Transformation f;
  for_list(j, n, bodies) {
    CHECK(n->inLinks.N<=1,"loopy geometry - body '" <<n->name <<"' has more than 1 input link");
    for_list(i, e, n->inLinks) {
      f = e->from->X;
      f.appendTransformation(e->A);
      e->X = f;
      if(e->type==JT_hingeX || e->type==JT_transX)  e->X.rot.getX(e->axis);
      if(e->type==JT_hingeY || e->type==JT_transY)  e->X.rot.getY(e->axis);
      if(e->type==JT_hingeZ || e->type==JT_transZ)  e->X.rot.getZ(e->axis);
      f.appendTransformation(e->Q);
      if(!isLinkTree) f.appendTransformation(e->B);
      n->X=f;
    }
  }
  calcShapeFramesFromBodies();
}

void ors::Graph::fillInRelativeTransforms() {
  for_list_(Joint, e, joints) {
    if(!e->X.isZero() && e->A.isZero() && e->B.isZero()) {
      e->A.setDifference(e->from->X, e->X);
      e->B.setDifference(e->X, e->to->X);
    }
  }
}

void ors::Graph::calcShapeFramesFromBodies() {
  Body *n;
  Shape *s;
  uint i, j;
  ors::Transformation f;
  for_list(j, n, bodies) {
    for_list(i, s, n->shapes) {
      s->X = n->X;
      s->X.appendTransformation(s->rel);
    }
  }
}

/** @brief given the absolute frames of all nodes and the two rigid (relative)
    frames A & B of each edge, this calculates the dynamic (relative) joint
    frame X for each edge (which includes joint transformation and errors) */
void ors::Graph::calcJointsFromBodyFrames() {
  Joint *e;
  uint i;
  for_list(i, e, joints) {
    ors::Transformation A(e->from->X), B(e->to->X);
    A.appendTransformation(e->A);
    B.appendInvTransformation(e->B);
    e->Q.setDifference(A, B);
  }
}

/** @brief in all edge frames: remove any displacements, velocities and non-x rotations.
    After this, edges and nodes are not coherent anymore. You might want to call
    calcBodyFramesFromJoints() */
void ors::Graph::clearJointErrors() {
  Joint *e;
  ors::Vector xaxis(1, 0, 0);
  uint i;
  for_list(i, e, joints) {
    e->Q.pos.setZero();
    e->Q.vel.setZero();
    e->Q.rot.alignWith(xaxis);
    e->Q.angvel.makeColinear(xaxis);
  }
}

/** @brief invert all velocity variables of all frames */
void ors::Graph::invertTime() {
  Body *n;
  Joint *e;
  uint i, j;
  for_list(j, n, bodies) {
    n->X.vel*=-1.;
    n->X.angvel*=-1.;
    for_list(i, e, n->inLinks) {
      e->Q.vel*=-1.;
      e->Q.angvel*=-1.;
    }
  }
}

arr ors::Graph::naturalQmetric() {
#if 1
  if(!q_dim) getJointStateDimension(true);
  arr Wdiag(q_dim);
  Wdiag=1.;
  return Wdiag;
#else
  //compute generic q-metric depending on tree depth
  arr BM(bodies.N);
  BM=1.;
  for(uint i=BM.N; i--;) {
    for(uint j=0; j<bodies(i)->outLinks.N; j++) {
      BM(i) += BM(bodies(i)->outLinks(j)->to->index);
    }
  }
  if(!q_dim) getJointStateDimension(true);
  arr Wdiag(q_dim);
  for_list_(Joint, j, joints) {
    for(uint i=0; i<j->qDim(); i++) Wdiag(j->qIndex+i)=::pow(BM(j->to->index), 1.);
  }
  if(Qlin.N){
    arr W;
    W.setDiag(Wdiag);
    if(Qlin.N) W = ~Qlin*W*Qlin;
    getDiag(Wdiag, W);
  }
  return Wdiag;
#endif
}

void ors::Graph::computeNaturalQmetric(arr& W) {
  W.setDiag(naturalQmetric());
}

/** @brief revert the topological orientation of a joint (edge),
   e.g., when choosing another body as root of a tree */
void ors::Graph::revertJoint(ors::Joint *e) {
  cout <<"reverting edge (" <<e->from->name <<' ' <<e->to->name <<")" <<endl;
  //revert
  uint i=e->ifrom;  e->ifrom=e->ito;  e->ito=i;
  graphMakeLists(bodies, joints);
  ors::Transformation f;     ///< transformation from parent body to joint (attachment, usually static)
  f=e->A;
  e->A.setInverse(e->B);
  e->B.setInverse(f);
  f=e->Q;
  e->Q.setInverse(f);
}

/** @brief re-orient all joints (edges) such that n becomes
  the root of the configuration */
void ors::Graph::reconfigureRoot(Body *n) {
  MT::Array<Body*> list, list2;
  Body **m,**mstop;
  Joint *e;
  list.append(n);
  uintA level(bodies.N);
  level=0;
  int i=0;
  uint j;
  
  while(list.N>0) {
    i++;
    list2.clear();
    mstop=list.p+list.N;
    for(m=list.p; m!=mstop; m++) {
      level((*m)->index)=i;
      for_list(j, e, (*m)->inLinks) {
        //for_in_edges_save(e, es, (*m))
        if(!level(e->from->index)) { revertJoint(e); j--; }
      }
      for_list(j, e, (*m)->outLinks) list2.append(e->to);
    }
    list=list2;
  }
  
  graphTopsort(bodies, joints);
}

/** @brief returns the joint (actuator) dimensionality */
uint ors::Graph::getJointStateDimension(bool internal) const {
  if(!q_dim) {
    uint jd=0;
    for_list_(Joint, j, joints) {
      if(!j->coupledTo){
        j->qIndex = jd;
        jd += j->qDim();
      }else{
        j->qIndex = j->coupledTo->qIndex;
      }
    }
    ((Graph*)this)->q_dim = jd; //hack to work around const declaration
  }
  
  if(internal || !Qlin.N) return q_dim;
  else {
    CHECK(Qlin.d0==q_dim, "");
    return Qlin.d1;
  }
}

//first version, give series of translated positions of bodies with such indexes (NJ)
void ors::Graph::setExternalState(const arr & x) {
  for(uint i = 0; i < x.N; i+=4) {
    ors::Body *body = bodies((uint)x(i));//index
    body->X.pos = ors::Vector(x(i+1), x(i+2), x(i+3));//3 position coordinates
  }
}

void ors::Graph::zeroGaugeJoints() {
  Body *n;
  Joint *e;
  ors::Vector w;
  uint j;
  for_list(j, n, bodies) if(n->type!=staticBT) {
    e=n->inLinks(0);
    if(e) {
      w=e->Q.rot / e->Q.angvel; e->Q.angvel.setZero();
      e->A.appendTransformation(e->Q);
      e->Q.setZero();
      e->Q.angvel=w;
    }
  }
}

/** @brief returns the joint state vectors separated in positions and
  velocities */
void ors::Graph::getJointState(arr& x, arr& v) const {
  ors::Vector rotv;
  ors::Quaternion rot;
  
  if(!q_dim) getJointStateDimension();
  x.resize(q_dim);
  if(&v) v.resize(q_dim);
  
  uint n=0;
  for_list_(Joint, j, joints) {
    if(j->coupledTo) continue; //don't count dependent joints
    switch(j->type) {
      case JT_hingeX:
      case JT_hingeY:
      case JT_hingeZ: {
        //angle
        j->Q.rot.getRad(x(n), rotv);
        if(x(n)>MT_PI) x(n)-=MT_2PI;
        if(j->type==JT_hingeX && rotv*Vector_x<0.) x(n)=-x(n);  //MT_2PI-x(i);
        if(j->type==JT_hingeY && rotv*Vector_y<0.) x(n)=-x(n);  //MT_2PI-x(i);
        if(j->type==JT_hingeZ && rotv*Vector_z<0.) x(n)=-x(n);  //MT_2PI-x(i);
        //velocity
        if(&v){
          v(n)=j->Q.angvel.length();
          if(j->type==JT_hingeX && j->Q.angvel*Vector_x<0.) v(n)=-v(n);
          if(j->type==JT_hingeY && j->Q.angvel*Vector_y<0.) v(n)=-v(n);
          if(j->type==JT_hingeZ && j->Q.angvel*Vector_z<0.) v(n)=-v(n);
        }
        n++;
      } break;
      case JT_universal:
        //angle
        rot = j->Q.rot;
        if(fabs(rot.w)>1e-15) {
          x(n) = 2.0 * atan(rot.x/rot.w);
          x(n+1) = 2.0 * atan(rot.y/rot.w);
        } else {
          x(n) = MT_PI;
          x(n+1) = MT_PI;
        }
        
        // velocity: need to fix
        if(&v) NIY;

        n+=2;
        break;
      case JT_transX:
        x(n)=j->Q.pos.x;
        if(&v) v(n)=j->Q.vel.x;
        n++;
        break;
      case JT_transY:
        x(n)=j->Q.pos.y;
        if(&v) v(n)=j->Q.vel.y;
        n++;
        break;
      case JT_transZ:
        x(n)=j->Q.pos.z;
        if(&v) v(n)=j->Q.vel.z;
        n++;
        break;
      case JT_trans3:
        x(n)=j->Q.pos.x;
        x(n+1)=j->Q.pos.y;
        x(n+2)=j->Q.pos.z;
        if(&v) {
          v(n)=j->Q.vel.x;
          v(n+1)=j->Q.vel.y;
          v(n+2)=j->Q.vel.z;
        }
        n+=3;
        break;
      case JT_glue:
      case JT_fixed:
        break;
      default: NIY;
    }
  }

  CHECK(n==q_dim,"");

  if(Qlin.N) {
    x=Qinv*(x-Qoff);
    v=Qinv*v;
  }
}

/** @brief returns the joint positions only */
void ors::Graph::getJointState(arr& x) const { getJointState(x, NoArr); }

/** @brief sets the joint state vectors separated in positions and
  velocities */
void ors::Graph::setJointState(const arr& _q, const arr& _v, bool clearJointErrors) {
  Joint *j;
  uint n=0, i;
  ors::Quaternion rot1, rot2;
  arr q=_q, v;
  if(&_v) v=_v;

  if(Qlin.N) {
    CHECK(_q.N==Qlin.d1,"wrong joint dimensions: ors expected " <<Qlin.d1 <<" joints; you gave " <<_q.N <<" joints");
    q = Qlin*_q + Qoff;
    if(&_v) { v = Qlin*_v;  v.reshape(v.N); }
  }

  if(!q_dim) getJointStateDimension();
  CHECK(q.N==q_dim && (!(&_v) || v.N==q_dim), "wrong joint state dimensionalities");
  
  for_list(i, j, joints) {
    if(j->coupledTo){
      j->Q=j->coupledTo->Q;
    }else switch(j->type) {
      case JT_hingeX: {
        //angle
        j->Q.rot.setRadX(q(n));
        
        // check boundaries
        /*if(e->p[0] < e->p[1]){
        tempAngleDeg = q(n); //dm *180.0/MT_PI;
        if(tempAngleDeg <= e->p[0]){ // joint angle is smaller than lower bound (e->p[0])--> clip it
        e->Q.r.setDeg(e->p[0], Vector_x);
        //  cout <<"lower clipping " <<tempAngleDeg <<endl;
        }else if(tempAngleDeg >= e->p[1]){ // joint angle is larger than upper bound (e->p[1])--> clip it
        e->Q.r.setDeg(e->p[1], Vector_x);
        //  cout <<"upper clipping " <<tempAngleDeg <<endl;
        }
        }*/
        
        //velocity
        if(&_v){  j->Q.angvel.set(v(n) ,0., 0.);  j->Q.zeroVels=false;  }
        else{  j->Q.angvel.setZero();  j->Q.zeroVels=true;  }
        //if(e->Q.w.isZero()) e->Q.w=Vector_x;
        //if(e->Q.w*Vector_x<0.) e->Q.w.setLength(-v(n)); else e->Q.w.setLength(v(n));
        
        if(clearJointErrors) {
          j->Q.pos.setZero();
          j->Q.vel.setZero();
          //truely, also the rotations X.r and X.w should be made orthogonal to the x-axis
        }
        n++;
      } break;

      case JT_hingeY: {
        j->Q.rot.setRadY(q(n));
        if(&_v){  j->Q.angvel.set(0., v(n) ,0.);  j->Q.zeroVels=false;  }
        else{  j->Q.angvel.setZero();  j->Q.zeroVels=true;  }
        if(clearJointErrors) {
          j->Q.pos.setZero();
          j->Q.vel.setZero();
        }
        n++;
      } break;

      case JT_hingeZ: {
        j->Q.rot.setRadZ(q(n));
        if(&_v){  j->Q.angvel.set(0., 0., v(n));  j->Q.zeroVels=false;  }
        else{  j->Q.angvel.setZero();  j->Q.zeroVels=true;  }
        if(clearJointErrors) {
          j->Q.pos.setZero();
          j->Q.vel.setZero();
        }
        n++;
      } break;
      
      case JT_universal:
        //angle
        rot1.setRadX(q(n));
        rot2.setRadY(q(n+1));
        
        //check boundaries
        /*if(e->p[0] < e->p[1]){
        // TODO: both angles are restricted to the same boundaries. Could be enhanced
        //     in order to be able to restrict the two angles differently
        tempAngleDeg = q(n); //dm *180.0/MT_PI;
        if(tempAngleDeg <= e->p[0]){ // joint angle is smaller than lower bound (e->p[0])--> clip it
        rot1.setRadX(e->p[0]);
        rot2.setRadY(e->p[0]);
        //  cout <<"lower clipping " <<tempAngleDeg <<endl;
        }else if(tempAngleDeg >= e->p[1]){ // joint angle is larger than upper bound (e->p[1])--> clip it
        rot1.setRadX(e->p[1]);
        rot2.setRadY(e->p[1]);
        //  cout <<"upper clipping " <<tempAngleDeg <<endl;
        }
        }*/
        
        j->Q.rot = rot1*rot2;
        //velocity
        // need to fix
        
        if(clearJointErrors) {
          j->Q.pos.setZero();
          j->Q.vel.setZero();
        }
        n+=2;
        break;
      case JT_transX: {
        j->Q.pos = q(n)*Vector_x;
        
        // check boundaries
        /*if(e->p[0] < e->p[1]){
        tempDeflection = q(n);
        if(tempDeflection <= e->p[0]){ // joint angle is smaller than lower bound (e->p[0])--> clip it
        e->Q.p = e->p[0]*Vector_x;
        cout <<"lower clipping " <<tempDeflection <<endl;
        }else if(tempDeflection >= e->p[1]){ // joint angle is larger than upper bound (e->p[1])--> clip it
        e->Q.p = e->p[1]*Vector_x;
        cout <<"upper clipping " <<tempDeflection <<endl;
        }
        }*/
        
        //velocity
        if(&_v){ j->Q.vel.set(v(n), 0., 0.); j->Q.zeroVels=false; }
        
        if(clearJointErrors) {
          j->Q.rot.setZero();
          j->Q.angvel.setZero();
        }
        n++;
      } break;

      case JT_transY: {
        j->Q.pos = q(n)*Vector_y;
        if(&_v){ j->Q.vel.set(0., v(n), 0.); j->Q.zeroVels=false; }
        if(clearJointErrors) {
          j->Q.rot.setZero();
          j->Q.angvel.setZero();
        }
        n++;
      } break;

      case JT_transZ: {
        j->Q.pos = q(n)*Vector_z;
        if(&_v){ j->Q.vel.set(0., 0., v(n)); j->Q.zeroVels=false; }
        if(clearJointErrors) {
          j->Q.rot.setZero();
          j->Q.angvel.setZero();
        }
        n++;
      } break;

      case JT_trans3: {
        j->Q.pos.set(q(n), q(n+1), q(n+2));
        if(&_v){ j->Q.vel.set(v(n), v(n+1), v(n+2)); j->Q.zeroVels=false; }
        if(clearJointErrors) {
          j->Q.rot.setZero();
          j->Q.angvel.setZero();
        }
        n+=3;
      } break;

      case JT_glue:
      case JT_fixed:
        j->Q.setZero();
        j->Q.zeroVels=true;
        break;
      default: NIY;
    }
  }

  CHECK(n==q_dim,"");
}

/** @brief sets the joint angles with velocity zero - e.g. for kinematic
  simulation only */
void ors::Graph::setJointState(const arr& x, bool clearJointErrors) {
  setJointState(x, NoArr, clearJointErrors);
}

//===========================================================================
//===========================================================================
//===========================================================================
//
// core: kinematics and dynamics
//
// essential papers:
// David Baraff: "Linear-Time Dynamics using Lagrange Multipliers"
// Roy Featherstone, David Orin: "Robot Dynamics: Equations and Algorithms"

/** @brief return the position \f$x = \phi_i(q)\f$ of the i-th body (3 vector) */
void ors::Graph::kinematicsPos(arr& y, uint a, ors::Vector *rel) const {
  ors::Vector pos=bodies(a)->X.pos;
  if(rel) pos += bodies(a)->X.rot*(*rel);
  y = ARRAY(pos);
}

/** @brief return the jacobian \f$J = \frac{\partial\phi_i(q)}{\partial q}\f$ of the position
  of the i-th body (3 x n tensor)*/
void ors::Graph::jacobianPos(arr& J, uint a, ors::Vector *rel) const {
  uint i;
  Joint *ei;
  ors::Vector tmp;
  
  if(!q_dim) getJointStateDimension(true);
  
  //initialize Jacobian
  J.resize(3, q_dim);
  J.setZero();
  
  //get reference frame
  ors::Vector pos = bodies(a)->X.pos;
  if(rel) pos += bodies(a)->X.rot*(*rel);
  
  if(bodies(a)->inLinks.N) { //body a has no inLinks -> zero jacobian
    ei=bodies(a)->inLinks(0);
    while(ei) {
      i=ei->qIndex;
      if(i>=q_dim) {
        CHECK(ei->type==JT_glue || ei->type==JT_fixed, "");
        if(!ei->from->inLinks.N) break;
        ei=ei->from->inLinks(0);
        continue;
      }
      CHECK(ei->type!=JT_glue && ei->type!=JT_fixed, "resort joints so that fixed and glued are last");


      if(ei->type==JT_hingeX || ei->type==JT_hingeY || ei->type==JT_hingeZ) {
        tmp = ei->axis ^ (pos-ei->X.pos);
        J(0, i) += tmp.x;
        J(1, i) += tmp.y;
        J(2, i) += tmp.z;
      }
      else if(ei->type==JT_transX || ei->type==JT_transY || ei->type==JT_transZ) {
        J(0, i) += ei->axis.x;
        J(1, i) += ei->axis.y;
        J(2, i) += ei->axis.z;
      }
      else if(ei->type==JT_trans3) {
        if(ei->coupledTo) NIY;
        arr R(3,3); ei->X.rot.getMatrix(R.p);
        J.setMatrixBlock(R,0,i);
      }

      if(!ei->from->inLinks.N) break;
      ei=ei->from->inLinks(0);
    }
  }

  if(Qlin.N) J=J*Qlin;
//  if(q_dim<j_dim){
//    //we have coupled joints!
//    arr Jleft  = J.sub(0,-1,0,q_dim-1);
//    arr Jright = J.sub(0,-1,q_dim,-1);
//    J = Jleft + Jright * Qlin;
//  }
}

/** @brief return the Hessian \f$H = \frac{\partial^2\phi_i(q)}{\partial q\partial q}\f$ of the position
  of the i-th body (3 x n x n tensor) */
void ors::Graph::hessianPos(arr& H, uint a, ors::Vector *rel) const {
  HALT("this is buggy: a sign error: see examples/Ors/ors testKinematics");
  uint i, j;
  Joint *ei, *ej;
  ors::Vector r;
  
  if(!q_dim)((ors::Graph*)this)->q_dim = getJointStateDimension(true);
  
  //initialize Jacobian
  H.resize(3, q_dim, q_dim);
  H.setZero();
  
  //get reference frame
  ors::Vector pos = bodies(a)->X.pos;
  if(rel) pos += bodies(a)->X.rot*(*rel);
  
  if(bodies(a)->inLinks.N) {
    ei=bodies(a)->inLinks(0);
    while(ei) {
      i=ei->qIndex;

      ej=ei;
      while(ej) {
        j=ej->qIndex;

        if(ei->type>=JT_hingeX && ei->type<=JT_hingeZ && ej->type>=JT_hingeX && ej->type<=JT_hingeZ) { //both are hinges
          r = ej->axis ^ (ei->axis ^ (pos-ei->X.pos));
          H(0, i, j) = H(0, j, i) = r.x;
          H(1, i, j) = H(1, j, i) = r.y;
          H(2, i, j) = H(2, j, i) = r.z;
        }
        if(ei->type>=JT_transX && ei->type<=JT_transZ && ej->type>=JT_hingeX && ej->type<=JT_hingeZ) { //i=trans, j=hinge
          r = ei->axis ^ ej->axis;
          H(0, i, j) = H(0, j, i) = r.x;
          H(1, i, j) = H(1, j, i) = r.y;
          H(2, i, j) = H(2, j, i) = r.z;
        }
        if(ei->type==JT_trans3 && ej->type>=JT_hingeX && ej->type<=JT_hingeZ) { //i=trans3, j=hinge
          Matrix R,A;
          ei->X.rot.getMatrix(R.p());
          A.setSkew(ej->axis);
          R = R*A;
          H(0, i  , j) = H(0, j  , i) = R.m00;
          H(1, i  , j) = H(1, j  , i) = R.m10;
          H(2, i  , j) = H(2, j  , i) = R.m20;
          H(0, i+1, j) = H(0, j, i+1) = R.m01;
          H(1, i+1, j) = H(1, j, i+1) = R.m11;
          H(2, i+1, j) = H(2, j, i+1) = R.m21;
          H(0, i+2, j) = H(0, j, i+2) = R.m02;
          H(1, i+2, j) = H(1, j, i+2) = R.m12;
          H(2, i+2, j) = H(2, j, i+2) = R.m22;
        }
        if(ei->type>=JT_hingeX && ei->type<=JT_hingeZ && ej->type>=JT_transX && ej->type<=JT_trans3) { //i=hinge, j=trans
          //nothing! Hessian is zero (ej is closer to root than ei)
        }

        if(!ej->from->inLinks.N) break;
        ej=ej->from->inLinks(0);
      }
      if(!ei->from->inLinks.N) break;
      ei=ei->from->inLinks(0);
    }
  }

  if(Qlin.N) H=~Qlin*H*Qlin;
}

/// kinematis of the i-th body's z-orientation vector
void ors::Graph::kinematicsVec(arr& y, uint a, ors::Vector *vec) const {
  ors::Transformation f=bodies(a)->X;
  ors::Vector v;
  if(vec) v=f.rot*(*vec); else f.rot.getZ(v);
  y = ARRAY(v);
}

/* takes the joint state x and returns the jacobian dz of
   the position of the ith body (w.r.t. all joints) -> 2D array */
/// Jacobian of the i-th body's z-orientation vector
void ors::Graph::jacobianVec(arr& J, uint a, ors::Vector *vec) const {
  uint i;
  Joint *ei;
  ors::Vector r, ta;
  
  if(!q_dim)((ors::Graph*)this)->q_dim = getJointStateDimension(true);
  
  //initialize Jacobian
  J.resize(3, q_dim);
  J.setZero();
  
  //get reference frame
  if(vec) ta = bodies(a)->X.rot*(*vec);
  else    bodies(a)->X.rot.getZ(ta);
  
  if(bodies(a)->inLinks.N) {
    ei=bodies(a)->inLinks(0);
    while(ei) {
      i=ei->qIndex;
      if(i>=q_dim) {
        CHECK(ei->type==JT_glue || ei->type==JT_fixed, "");
        if(!ei->from->inLinks.N) break;
        ei=ei->from->inLinks(0);
        continue;
      }
      CHECK(ei->type!=JT_glue && ei->type!=JT_fixed, "resort joints so that fixed and glued are last");

      if(ei->type>=JT_hingeX && ei->type<=JT_hingeZ) { //i=hinge
        r = ei->axis ^ ta;
        J(0, i) += r.x;
        J(1, i) += r.y;
        J(2, i) += r.z;
      }
      if(ei->type>=JT_transX && ei->type<=JT_trans3) { //i=trans
        //J(0, i) = J(1, i) = J(2, i) = 0.; /was set zero already
      }

      if(!ei->from->inLinks.N) break;
      ei=ei->from->inLinks(0);
    }
  }

  if(Qlin.N) J=J*Qlin;
//  if(q_dim<j_dim){
//    //we have coupled joints!
//    arr Jleft  = J.sub(0,-1,0,q_dim-1);
//    arr Jright = J.sub(0,-1,q_dim,-1);
//    J = Jleft + Jright * Qlin;
//  }
}

/* takes the joint state x and returns the jacobian dz of
   the position of the ith body (w.r.t. all joints) -> 2D array */
void ors::Graph::jacobianR(arr& J, uint a) const {
  uint i;
  ors::Transformation Xi;
  Joint *ei;
  ors::Vector ti;
  
  if(!q_dim)((ors::Graph*)this)->q_dim = getJointStateDimension(true);
  
  //initialize Jacobian
  J.resize(3, q_dim);
  J.setZero();
  
  //get reference frame -- in this case we always take
  //the Z and X-axis of the world system as references
  // -> don't need to compute explicit reference for object a
  //  object a is relevant in the sense that only the tree-down
  //  joints contribute to this rotation
  
  if(!bodies(a)->inLinks.N) {
  ei=bodies(a)->inLinks(0);
  while(ei) {
    i=ei->qIndex;
    if(i>=q_dim) {
      CHECK(ei->type==JT_glue || ei->type==JT_fixed, "");
      if(!ei->from->inLinks.N) break;
      ei=ei->from->inLinks(0);
      continue;
    }
    CHECK(ei->type!=JT_glue && ei->type!=JT_fixed, "resort joints so that fixed and glued are last");
    
    Xi = ei->X;
    Xi.rot.getX(ti);
    
    J(0, i) = ti.x;
    J(1, i) = ti.y;
    J(2, i) = ti.z;
    
    if(!ei->from->inLinks.N) break;
    ei=ei->from->inLinks(0);
  }
  }
  
  if(Qlin.N) J=J*Qlin;
}

/** @brief return the configuration's inertia tensor $M$ (n x n tensor)*/
void ors::Graph::inertia(arr& M) {
  uint a, i, j;
  ors::Transformation Xa, Xi, Xj;
  Joint *ei, *ej;
  ors::Vector vi, vj, ti, tj;
  double tmp;
  
  if(!q_dim) q_dim = getJointStateDimension(true);
  
  //initialize Jacobian
  M.resize(q_dim, q_dim);
  M.setZero();
  
  for(a=0; a<bodies.N; a++) {
    //get reference frame
    Xa = bodies(a)->X;
    
    ei=bodies(a)->inLinks(0);
    while(ei) {
      i=ei->qIndex;
      
      Xi = ei->from->X;
      Xi.appendTransformation(ei->A);
      Xi.rot.getX(ti);
      
      vi = ti ^(Xa.pos-Xi.pos);
      
      ej=ei;
      while(ej) {
        j=ej->qIndex;
        
        Xj = ej->from->X;
        Xj.appendTransformation(ej->A);
        Xj.rot.getX(tj);
        
        vj = tj ^(Xa.pos-Xj.pos);
        
        tmp = bodies(a)->mass * (vi*vj);
        //tmp += scalarProduct(bodies(a)->a.inertia, ti, tj);
        
        M(i, j) += tmp;
        
        if(!ej->from->inLinks.N) break;
        ej=ej->from->inLinks(0);
      }
      if(!ei->from->inLinks.N) break;
      ei=ei->from->inLinks(0);
    }
  }
  //symmetric: fill in other half
  for(i=0; i<q_dim; i++) for(j=0; j<i; j++) M(j, i) = M(i, j);
}

void ors::Graph::equationOfMotion(arr& M, arr& F, const arr& qd) {
  if(Qlin.N) NIY;
  static ors::LinkTree tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  ors::equationOfMotion(M, F, tree, qd);
}

/** @brief return the joint accelerations \f$\ddot q\f$ given the
  joint torques \f$\tau\f$ (computed via Featherstone's Articulated Body Algorithm in O(n)) */
void ors::Graph::dynamics(arr& qdd, const arr& qd, const arr& tau) {
  static ors::LinkTree tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  //cout <<tree <<endl;
  //ors::fwdDynamics_aba_1D(qdd, tree, qd, tau);
  //ors::fwdDynamics_aba_nD(qdd, tree, qd, tau);
  ors::fwdDynamics_MF(qdd, tree, qd, tau);
}

/** @brief return the necessary joint torques \f$\tau\f$ to achieve joint accelerations
  \f$\ddot q\f$ (computed via the Recursive Newton-Euler Algorithm in O(n)) */
void ors::Graph::inverseDynamics(arr& tau, const arr& qd, const arr& qdd) {
  static ors::LinkTree tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  ors::invDynamics(tau, tree, qd, qdd);
}

/*void ors::Graph::impulsePropagation(arr& qd1, const arr& qd0){
  static MT::Array<Featherstone::Link> tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  mimickImpulsePropagation(tree);
  Featherstone::RF_abd(qdd, tree, qd, tau);
}*/

/// [prelim] some heuristic measure for the joint errors
double ors::Graph::getJointErrors() const {
  Joint *e;
  double err=0.0;
  uint i;
  
  for_list(i, e, joints) err+=e->Q.pos.lengthSqr();
  
  return ::sqrt(err);
}

/** @brief checks if all names of the bodies are disjoint */
bool ors::Graph::checkUniqueNames() const {
  Body *n, *m;
  uint i, j;
  for_list(i, n, bodies) for_list(j, m, bodies) {
    if(n==m) break;
    if(n->name==m->name) return false;
  }
  return true;
}

/// find body with specific name
ors::Body* ors::Graph::getBodyByName(const char* name) const {
  for_list_(Body, b, bodies) if(b->name==name) return b;
  if(strcmp("glCamera", name)!=0)
    MT_MSG("cannot find Body named '" <<name <<"' in Graph");
  return 0;
}

///// find body index with specific name
//uint ors::Graph::getBodyIndexByName(const char* name) const {
//  Body *b=getBodyByName(name);
//  return b?b->index:0;
//}

/// find shape with specific name
ors::Shape* ors::Graph::getShapeByName(const char* name) const {
  for_list_(Shape, s, shapes) if(s->name==name) return s;
  MT_MSG("cannot find Shape named '" <<name <<"' in Graph");
  return NULL;
}

///// find shape index with specific name
//uint ors::Graph::getShapeIndexByName(const char* name) const {
//  Shape *s=getShapeByName(name);
//  return s?s->index:0;
//}

/// find shape with specific name
ors::Joint* ors::Graph::getJointByName(const char* name) const {
  for_list_(Joint, j, joints) if(j->name==name) return j;
  MT_MSG("cannot find Joint named '" <<name <<"' in Graph");
  return NULL;
}

/// find joint connecting two bodies with specific names
ors::Joint* ors::Graph::getJointByBodyNames(const char* from, const char* to) const {
  Body *f=NULL, *t=NULL;
  uint j;
  for_list(j, f, bodies) if(f->name==from) break;
  for_list(j, t, bodies) if(t->name==to) break;
  if(!f || !t) return 0;
  return graphGetEdge<Body, Joint>(f, t);
}

/** @brief creates uniques names by prefixing the node-index-number to each name */
void ors::Graph::prefixNames() {
  Body *n;
  uint j;
  for_list(j, n, bodies) n->name=STRING(n->index<< n->name);
}

/** @brief prototype for \c operator<< */
void ors::Graph::write(std::ostream& os) const {
  for_list_(Body, b, bodies) {
    os <<"body " <<b->name <<" { ";
    b->write(os);  os <<" }\n";
  }
  os <<std::endl;
  for_list_(Shape, s, shapes) {
    os <<"shape (" <<s->body->name <<"){ ";
    s->write(os);  os <<" }\n";
  }
  os <<std::endl;
  for_list_(Joint, j, joints) {
    os <<"joint ";
    if (j->name.N) os <<j->name.p <<' ';
    os <<"(" <<j->from->name <<' ' <<j->to->name <<"){ ";
    j->write(os);  os <<" }\n";
  }
}

#define DEBUG(x) //x

void ors::Graph::read(const char* filename) {
  std::istringstream is(filename);
  read(is);
}

/** @brief prototype for \c operator>> */
void ors::Graph::read(std::istream& is) {
  uint i; Item *it;
  KeyValueGraph G;
  
  G.read(is);
  //cout <<"***MAPGRAPH\n" <<G <<endl;
  
  clear();
  
  ItemL bs = G.getItems("body");
  for_list(i, it, bs) {
    CHECK(it->keys(0)=="body","");
    CHECK(it->valueType()==typeid(KeyValueGraph), "bodies must have value KeyValueGraph");
    
    Body *b=new Body(*this);
    b->name = it->keys(1);
    b->ats = *it->value<KeyValueGraph>();
    b->parseAts();

    for (auto shape : b->shapes) {
        shape->index = shapes.N;
        shapes.append(shape);
    }
  }
  
  ItemL ss = G.getItems("shape");
  for_list(i, it, ss) {
    CHECK(it->keys(0)=="shape","");
    CHECK(it->parents.N<=1,"shapes must have no or one parent");
    CHECK(it->valueType()==typeid(KeyValueGraph),"shape must have value KeyValueGraph");
    
    Shape *s;
    if(it->parents.N==1){
      Body *b = listFindByName(bodies, it->parents(0)->keys(1));
      CHECK(b,"");
      s=new Shape(*this, b);
    }else{
      s=new Shape(*this, NULL);
    }
    if(it->keys.N>1) s->name=it->keys(1);
    s->ats = *it->value<KeyValueGraph>();
    s->parseAts();
  }
  
  uint nCoupledJoints=0;
  ItemL js = G.getItems("joint");
  for_list(i, it, js) {
    CHECK(it->keys(0)=="joint","");
    CHECK(it->parents.N==2,"joints must have two parents");
    CHECK(it->valueType()==typeid(KeyValueGraph),"joints must have value KeyValueGraph");
    
    Body *from=listFindByName(bodies, it->parents(0)->keys(1));
    Body *to=listFindByName(bodies, it->parents(1)->keys(1));
    CHECK(from,"JOINT: from '" <<it->parents(0)->keys(1) <<"' does not exist ["<<*it <<"]");
    CHECK(to,"JOINT: to '" <<it->parents(1)->keys(1) <<"' does not exist ["<<*it <<"]");
    Joint *j=new Joint(*this, from, to);
    if(it->keys.N>1) j->name=it->keys(1);
    j->ats = *it->value<KeyValueGraph>();
    j->parseAts();

    //if the joint is coupled to another:
    if(j->coupledTo) nCoupledJoints++;
  }

#if 0
  if(nCoupledJoints){
    Joint *j; uint i;
    //move them to the back and append a Qlin;
    for_list_rev(i, j, joints) if(j->isCoupled) {
      joints.remove(i);
      joints.append(j);
    }
    for_list(i, j, joints) j->index=i;
    getJointStateDimension();
    CHECK(j_dim == q_dim+nCoupledJoints,"something is wrong");
    MT::String jointName;
    Qlin.resize(nCoupledJoints, q_dim);
    for(i=0; i<nCoupledJoints; i++){
      j=joints(joints.N-nCoupledJoints+i);
      CHECK(j->isCoupled,"");
      bool good = j->ats.getValue<MT::String>(jointName, "coupledTo");
      CHECK(good, "something is wrong");
      Joint *coupledTo = listFindByName(joints, jointName);
      j->type = coupledTo->type;
      if(!coupledTo) HALT("The joint '" <<*j <<"' is declared coupled to '" <<jointName <<"' -- but that doesn't exist!");
      Qlin(i, coupledTo->qIndex) = 1.;
    }
  }
#else
  if(nCoupledJoints){
    for_list_(Joint, j, joints) if(j->coupledTo){
      MT::String jointName;
      bool good = j->ats.getValue<MT::String>(jointName, "coupledTo");
      CHECK(good, "something is wrong");
      j->coupledTo = listFindByName(joints, jointName);
      j->type = j->coupledTo->type;
      if(!j->coupledTo) HALT("The joint '" <<*j <<"' is declared coupled to '" <<jointName <<"' -- but that doesn't exist!");
    }
  }
#endif

  graphMakeLists(bodies, joints);
  fillInRelativeTransforms();
  
  //cout <<"***ORSGRAPH\n" <<*this <<endl;
}

void ors::Graph::writePlyFile(const char* filename) const {
  ofstream os;
  MT::open(os, filename);
  uint nT=0,nV=0;
  uint i,j;
  ors::Shape *s;
  ors::Mesh *m;
  for_list(i,s,shapes) { nV += s->mesh.V.d0; nT += s->mesh.T.d0; }
  
  os <<"\
ply\n\
format ascii 1.0\n\
element vertex " <<nV <<"\n\
property float x\n\
property float y\n\
property float z\n\
property uchar red\n\
property uchar green\n\
property uchar blue\n\
element face " <<nT <<"\n\
property list uchar int vertex_index\n\
end_header\n";

  uint k=0;
  ors::Transformation t;
  ors::Vector v;
  for_list(i,s,shapes) {
    m = &s->mesh;
    t = s->X;
    if(m->C.d0!=m->V.d0) {
      m->C.resizeAs(m->V);
      for(j=0; j<m->C.d0; j++) { m->C(j, 0)=s->color[0]; m->C(j, 1)=s->color[1]; m->C(j, 2)=s->color[2]; }
    }
    for(j=0; j<m->V.d0; j++) {
      v.set(m->V(j, 0), m->V(j, 1), m->V(j, 2));
      v = t*v;
      os <<' ' <<v.x <<' ' <<v.y <<' ' <<v.z
         <<' ' <<int(255.f*m->C(j, 0)) <<' ' <<int(255.f*m->C(j, 1)) <<' ' <<int(255.f*m->C(j, 2)) <<endl;
    }
    k+=j;
  }
  uint offset=0;
  for_list(i,s,shapes) {
    m=&s->mesh;
    for(j=0; j<m->T.d0; j++) {
      os <<"3 " <<offset+m->T(j, 0) <<' ' <<offset+m->T(j, 1) <<' ' <<offset+m->T(j, 2) <<endl;
    }
    offset+=m->V.d0;
  }
}

/// dump the list of current proximities on the screen
void ors::Graph::reportProxies(std::ostream *os) {
  uint i;
  int a, b;
  (*os) <<"Proximity report: #" <<proxies.N <<endl;
  ors::Proxy *p;
  for_list(i,p,proxies) {
    a=p->a;
    b=p->b;
    (*os)
        <<i <<" ("
        <<a <<':' <<(a!=-1?shapes(a)->body->name.p:"earth") <<")-("
        <<b <<':' <<(b!=-1?shapes(b)->body->name.p:"earth")
        <<") d=" <<p->d
        <<" |A-B|=" <<(p->posB-p->posA).length()
        <<" d^2=" <<(p->posB-p->posA).lengthSqr()
        <<" normal=" <<p->normal
        <<" posA=" <<p->posA
        <<" posB=" <<p->posB
        <<endl;
  }
}

bool ProxySortComp(const ors::Proxy *a, const ors::Proxy *b) {
  return (a->a < b->a) || (a->a==b->a && a->b<b->b) || (a->a==b->a && a->b==b->b && a->d < b->d);
}

void ors::Graph::sortProxies(bool deleteMultiple) {
  uint i;
  
  ors::Proxy **proxiesstop=proxies.p+proxies.N;
  std::sort(proxies.p, proxiesstop, ProxySortComp);
  
  if(deleteMultiple) {
    for(i=0; i<proxies.N; i++) {
      if(i && proxies(i)->a==proxies(i-1)->a && proxies(i)->b==proxies(i-1)->b) {
        delete proxies(i);
        proxies.remove(i);
        i--;
      }
    }
  }
}

/** @brief dump a list body pairs for which the upper conditions hold */
void ors::Graph::reportGlue(std::ostream *os) {
  uint i, A, B;
  Body *a, *b;
  bool ag, bg;
  (*os) <<"Glue report: " <<endl;
  for(i=0; i<proxies.N; i++) {
    A=proxies(i)->a; a=(A==(uint)-1?NULL:bodies(A));
    B=proxies(i)->b; b=(B==(uint)-1?NULL:bodies(B));
    if(!a || !b) continue;
    ag=a->ats.getValue<bool>("glue");
    bg=b->ats.getValue<bool>("glue");
    if(ag || bg) {
      (*os)
          <<i <<' '
          <<a->index <<',' <<a->name <<'-'
          <<b->index <<',' <<b->name
          <<" d=" <<proxies(i)->d
          // <<" posA=" <<proxies(i)->posA
          // <<" posB=" <<proxies(i)->posB
          <<" norm=" <<proxies(i)->posB-proxies(i)->posA
          <<endl;
    }
  }
}

void ors::Graph::glueBodies(Body *f, Body *t) {
  Joint *e;
  e=newEdge(f->index, t->index, joints);
  graphMakeLists(bodies, joints);
  e->A.setDifference(f->X, t->X);
  e->A.vel.setZero();
  e->A.angvel.setZero();
  e->type=JT_glue;
  e->Q.setZero();
  e->B.setZero();
}

/** @brief if two bodies touch, the are not yet connected, and one of them has
  the `glue' attribute, add a new edge of FIXED type between them */
void ors::Graph::glueTouchingBodies() {
  uint i, A, B;
  Body *a, *b;//, c;
  bool ag, bg;
  for(i=0; i<proxies.N; i++) {
    A=proxies(i)->a; a=(A==(uint)-1?NULL:bodies(A));
    B=proxies(i)->b; b=(B==(uint)-1?NULL:bodies(B));
    if(!a || !b) continue;
    ag=a->ats.getValue<bool>("glue");
    bg=b->ats.getValue<bool>("glue");
    if(ag || bg) {
      //if(a->index > b->index){ c=a; a=b; b=c; } //order them topolgically
      if(graphGetEdge<Body, Joint>(a, b)) continue;  //they are already connected
      glueBodies(a, b);
      //a->cont=b->cont=false;
    }
  }
}

/// clear all forces currently stored at bodies
void ors::Graph::clearForces() {
  Body *n;
  uint j;
  for_list(j, n, bodies) {
    n->force.setZero();
    n->torque.setZero();
  }
}

/// apply a force on body n at position pos (in world coordinates)
void ors::Graph::addForce(ors::Vector force, Body *n, ors::Vector pos) {
  n->force += force;
  NIY;
  //n->torque += (pos - n->X.p) ^ force;
}

void ors::Graph::frictionToForces(double coeff) {
  HALT("never do this: add it directly in the equations...");
  Joint *e;
  ors::Vector a;
  ors::Transformation X;
  double v;
  uint i;
  for_list(i, e, joints) {
    X = e->from->X;
    X.appendTransformation(e->A);
    X.rot.getX(a);//rotation axis
    
    v=e->Q.angvel.length();
    if(e->Q.angvel*Vector_x<0.) v=-v;
    
    e->from->torque -= (coeff*v)*a;
    e->to->torque   += (coeff*v)*a;
  }
}

void ors::Graph::gravityToForces() {
  Body *n;
  uint j;
  ors::Vector g(0, 0, -9.81);
  for_list(j, n, bodies) n->force += n->mass * g;
}

/// compute forces from the current contacts
void ors::Graph::contactsToForces(double hook, double damp) {
  ors::Vector trans, transvel, force;
  uint i;
  int a, b;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<0.) {
      a=proxies(i)->a; b=proxies(i)->b;
      
      //if(!i || proxies(i-1).a!=a || proxies(i-1).b!=b) continue; //no old reference sticking-frame
      //trans = proxies(i)->rel.p - proxies(i-1).rel.p; //translation relative to sticking-frame
      trans    = proxies(i)->posB-proxies(i)->posA;
      //transvel = proxies(i)->velB-proxies(i)->velA;
      //d=trans.length();
      
      force.setZero();
      force += (hook) * trans; //*(1.+ hook*hook*d*d)
      //force += damp * transvel;
      SL_DEBUG(1, cout <<"applying force: [" <<a <<':' <<b <<"] " <<force <<endl);
      
      if(a!=-1) addForce(force, shapes(a)->body, proxies(i)->posA);
      if(b!=-1) addForce(-force, shapes(b)->body, proxies(i)->posB);
    }
}

/// measure (=scalar kinematics) for the contact cost summed over all bodies
void ors::Graph::phiCollision(arr &y, arr& J, double margin, bool useCenterDist) const {
  y.resize(1);
  y=0.;
  uint i;
  Shape *a, *b;
  ors::Vector normal;
  double cenMarg = 2.;
  arr Jpos;
  if(&J) J.resize(1, getJointStateDimension(false)).setZero();
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<margin) {
      CHECK(proxies(i)->cenD<.8*cenMarg, "sorry I made assumption objects are not too large; rescale cenMarg");
      a=shapes(proxies(i)->a); b=shapes(proxies(i)->b);

      //costs
      double d1 = 1.-proxies(i)->d/margin;
      double d2 = 1.-proxies(i)->cenD/cenMarg;
      //NORMALS ALWAYS GO FROM b TO a !!
      if(!useCenterDist) d2=1.;
      y(0) += d1*d2;
      
      //Jacobian
      if(&J) {
        if(proxies(i)->d>0.) { //we have a gradient on pos only when outside
          ors::Vector arel=a->X.rot/(proxies(i)->posA-a->X.pos);
          ors::Vector brel=b->X.rot/(proxies(i)->posB-b->X.pos);
          CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
          arr posN; posN.referTo(proxies(i)->normal.p(), 3); posN.reshape(1, 3);
          
          //grad on posA
          jacobianPos(Jpos, a->body->index, &arel); J -= d2/margin*(posN*Jpos);
          //grad on posA
          jacobianPos(Jpos, b->body->index, &brel); J += d2/margin*(posN*Jpos);
        }
        
        if(useCenterDist){
          ors::Vector arel=a->X.rot/(proxies(i)->cenA-a->X.pos);
          ors::Vector brel=b->X.rot/(proxies(i)->cenB-b->X.pos);
          CHECK(proxies(i)->cenN.isNormalized(), "proxy normal is not normalized");
          arr cenN; cenN.referTo(proxies(i)->cenN.p(), 3); cenN.reshape(1, 3);
        
          //grad on cenA
          jacobianPos(Jpos, a->body->index, &arel); J -= d1/cenMarg*(cenN*Jpos);
          //grad on cenB
          jacobianPos(Jpos, b->body->index, &brel); J += d1/cenMarg*(cenN*Jpos);
        }
      }
      
    }
}

#if 0 //obsolete:
void ors::Graph::getContactMeasure(arr &x, double margin, bool linear) const {
  x.resize(1);
  x=0.;
  uint i;
  Shape *a, *b;
  double d, discount;
  for(i=0; i<proxies.N; i++) if(!proxies(i)->age && proxies(i)->d<margin) {
      a=shapes(proxies(i)->a); b=shapes(proxies(i)->b);
      d=1.-proxies(i)->d/margin;
      //NORMALS ALWAYS GO FROM b TO a !!
      discount = 1.;
      if(!a->contactOrientation.isZero()) {  //object has an 'allowed contact orientation'
        CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
        CHECK(a->contactOrientation.isNormalized(), "contact orientation is not normalized");
        //double theta = ::acos( proxies(i)->normal*a->contactOrientation);
        double theta = .5*(proxies(i)->normal*a->contactOrientation-1.);
        discount *= theta*theta;
      }
      if(!b->contactOrientation.isZero()) {
        CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
        CHECK(a->contactOrientation.isNormalized(), "contact orientation is not normalized");
        //double theta = ::acos(-proxies(i)->normal*b->contactOrientation);
        double theta = .5*(-proxies(i)->normal*a->contactOrientation-1.);
        discount *= theta*theta;
      }
      if(!linear) x(0) += discount*d*d;
      else        x(0) += discount*d;
    }
}

/// gradient (=scalar Jacobian) of this contact cost
double ors::Graph::getContactGradient(arr &grad, double margin, bool linear) const {
  ors::Vector normal;
  uint i;
  Shape *a, *b;
  double d, discount;
  double cost=0.;
  arr J, dnormal;
  grad.resize(1, getJointStateDimension(false));
  grad.setZero();
  ors::Vector arel, brel;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<margin) {
      a=shapes(proxies(i)->a); b=shapes(proxies(i)->b);
      d=1.-proxies(i)->d/margin;
      discount = 1.;
      if(!a->contactOrientation.isZero()) {  //object has an 'allowed contact orientation'
        CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
        CHECK(a->contactOrientation.isNormalized(), "contact orientation is not normalized");
        //double theta = ::acos( proxies(i)->normal*a->contactOrientation);
        double theta = .5*(proxies(i)->normal*a->contactOrientation-1.);
        discount *= theta*theta;
      }
      if(!b->contactOrientation.isZero()) {
        CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
        CHECK(a->contactOrientation.isNormalized(), "contact orientation is not normalized");
        //double theta = ::acos(-proxies(i)->normal*b->contactOrientation);
        double theta = .5*(-proxies(i)->normal*a->contactOrientation-1.);
        discount *= theta*theta;
      }
      if(!linear) cost += discount*d*d;
      else        cost += discount*d;
      
      arel.setZero();  arel=a->X.rot/(proxies(i)->posA-a->X.pos);
      brel.setZero();  brel=b->X.rot/(proxies(i)->posB-b->X.pos);
      
      CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
      dnormal.referTo(proxies(i)->normal.p(), 3); dnormal.reshape(1, 3);
      if(!linear) {
        jacobianPos(J, a->body->index, &arel); grad -= ((double)2.*discount*d)/margin*(dnormal*J);
        jacobianPos(J, b->body->index, &brel); grad += ((double)2.*discount*d)/margin*(dnormal*J);
      } else {
        jacobianPos(J, a->body->index, &arel); grad -= discount/margin*(dnormal*J);
        jacobianPos(J, b->body->index, &brel); grad += discount/margin*(dnormal*J);
      }
    }
    
  return cost;
}
#endif

/// measure (=scalar kinematics) for the contact cost summed over all bodies
void ors::Graph::getContactConstraints(arr& y) const {
  y.clear();
  uint i;
  for(i=0; i<proxies.N; i++) y.append(proxies(i)->d);
}

/// gradient (=scalar Jacobian) of this contact cost
void ors::Graph::getContactConstraintsGradient(arr &dydq) const {
  dydq.clear();
  ors::Vector normal;
  uint i, con=0;
  Shape *a, *b;
  arr J, dnormal, grad(1, q_dim);
  ors::Vector arel, brel;
  for(i=0; i<proxies.N; i++) {
    a=shapes(proxies(i)->a); b=shapes(proxies(i)->b);
    
    arel.setZero();  arel=a->X.rot/(proxies(i)->posA-a->X.pos);
    brel.setZero();  brel=b->X.rot/(proxies(i)->posB-b->X.pos);
    
    CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
    dnormal.referTo(proxies(i)->normal.p(), 3); dnormal.reshape(1, 3);
    grad.setZero();
    jacobianPos(J, a->body->index, &arel); grad += dnormal*J; //moving a long normal b->a increases distance
    jacobianPos(J, b->body->index, &brel); grad -= dnormal*J; //moving b long normal b->a decreases distance
    dydq.append(grad);
    con++;
  }
  dydq.reshape(con, q_dim);
}


#if 0 //alternative implementation : cost=1 -> contact, other discounting...
double ors::Graph::getContactGradient(arr &grad, double margin) {
  ors::Vector normal;
  uint i;
  Shape *a, *b;
  double d, discount;
  double cost=0.;
  arr J, dnormal;
  grad.resize(1, jd);
  grad.setZero();
  ors::Transformation arel, brel;
  for(i=0; i<proxies.N; i++) if(!proxies(i)->age && proxies(i)->d<margin) {
      a=shapes(proxies(i)->a); b=shapes(proxies(i)->b);
      discount = 1.;
      if(!a->contactOrientation.isZero()) {  //object has an 'allowed contact orientation'
        CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
        CHECK(a->contactOrientation.isNormalized(), "contact orientation is not normalized");
        //double theta = ::acos( proxies(i)->normal*a->contactOrientation);
        double theta = .5*(proxies(i)->normal*a->contactOrientation-1.);
        discount *= theta*theta;
      }
      if(!b->contactOrientation.isZero()) {
        CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
        CHECK(a->contactOrientation.isNormalized(), "contact orientation is not normalized");
        //double theta = ::acos(-proxies(i)->normal*b->contactOrientation);
        double theta = .5*(-proxies(i)->normal*a->contactOrientation-1.);
        discount *= theta*theta;
      }
      double marg=(discount+.1)*margin;
      d=1.-proxies(i)->d/marg;
      if(d<0.) continue;
      cost += d*d;
      
      arel.setZero();  arel.p=a->X.r/(proxies(i)->posA-a->X.p);
      brel.setZero();  brel.p=b->X.r/(proxies(i)->posB-b->X.p);
      
      CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
      dnormal.referTo(proxies(i)->normal.v, 3); dnormal.reshape(1, 3);
      jacobianPos(J, a->body->index, &arel); grad -= (2.*d/marg)*(dnormal*J);
      jacobianPos(J, b->body->index, &brel); grad += (2.*d/marg)*(dnormal*J);
    }
    
  return cost;
}
#endif

void ors::Graph::getLimitsMeasure(arr &x, const arr& limits, double margin) const {
  CHECK(limits.d0==q_dim && limits.d1==2, "joint limits parameter size mismatch");
  x.resize(1);
  x=0.;
  arr q;
  getJointState(q);
  uint i;
  double d;
  for(i=0; i<q.N; i++) {
    d = q(i) - limits(i, 0);
    if(d<margin) {  d-=margin;  x(0) += d*d;  }
    d = limits(i, 1) - q(i);
    if(d<margin) {  d-=margin;  x(0) += d*d;  }
  }
}

double ors::Graph::getLimitsGradient(arr &grad, const arr& limits, double margin) const {
  CHECK(limits.d0==q_dim && limits.d1==2, "");
  uint i;
  double d;
  double cost=0.;
  arr J;
  grad.resize(1, getJointStateDimension(false));
  grad.setZero();
  arr q;
  getJointState(q);
  for(i=0; i<q.N; i++) {
    d = q(i) - limits(i, 0);
    if(d<margin) {  d-=margin;  grad(0, i) += 2.*d;  }
    d = limits(i, 1) - q(i);
    if(d<margin) {  d-=margin;  grad(0, i) -= 2.*d;  }
  }
  return cost;
}

/// center of mass of the whole configuration (3 vector)
double ors::Graph::getCenterOfMass(arr& x_) const {
  double M=0.;
  Body *n;
  uint j;
  ors::Vector x;
  x.setZero();
  for_list(j, n, bodies) {
    M+=n->mass;
    x+=n->mass*n->X.pos;
  }
  x/=M;
  x_ = ARRAY(x);
  return M;
}

/// gradient (Jacobian) of the COM w.r.t. q (3 x n tensor)
void ors::Graph::getComGradient(arr &grad) const {
  double M=0.;
  Body *n;
  uint j;
  arr J(3, getJointStateDimension(true));
  grad.resizeAs(J); grad.setZero();
  for_list(j, n, bodies) {
    M += n->mass;
    jacobianPos(J, n->index);
    grad += n->mass * J;
  }
  grad/=M;
}

/** @brief returns a k-dim vector containing the penetration depths of all bodies */
void ors::Graph::getPenetrationState(arr &vec) const {
  vec.resize(bodies.N);
  vec.setZero();
  ors::Vector d;
  uint i;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<0.) {
      d=proxies(i)->posB - proxies(i)->posA;
      
      if(proxies(i)->a!=-1) vec(proxies(i)->a) += d.length();
      if(proxies(i)->b!=-1) vec(proxies(i)->b) += d.length();
    }
}

ors::Proxy* ors::Graph::getContact(uint a, uint b) const {
  uint i;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<0.) {
      if(proxies(i)->a==(int)a && proxies(i)->b==(int)b) return proxies(i);
      if(proxies(i)->a==(int)b && proxies(i)->b==(int)a) return proxies(i);
    }
  return NULL;
}

/** @brief a vector describing the incoming forces (penetrations) on one object */
void ors::Graph::getGripState(arr& grip, uint j) const {
  ors::Vector d, p;
  ors::Vector sumOfD; sumOfD.setZero();
  ors::Vector torque; torque.setZero();
  double sumOfAbsD = 0.;
  double varOfD = 0.;
  
  p.setZero();
  uint i, n=0;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<0.) {
      if(proxies(i)->a!=(int)j && proxies(i)->b!=(int)j) continue;
      
      n++;
      
      if(proxies(i)->a==(int)j) {
        d=proxies(i)->posB - proxies(i)->posA;
        p=proxies(i)->posA;
      }
      if(proxies(i)->b==(int)j) {
        d=proxies(i)->posA - proxies(i)->posB;
        p=proxies(i)->posB;
      }
      
      sumOfAbsD += d.length();
      sumOfD    += d;
      varOfD    += d.lengthSqr();
      torque    += (p - bodies(j)->X.pos) ^ d;
      
    }
  if(n) { varOfD = (varOfD - sumOfD*sumOfD) / n; }
  
  grip.resize(8);
  grip(0)=sumOfAbsD;
  grip(1)=varOfD;
  grip(2)=sumOfD.x;
  grip(3)=sumOfD.y;
  grip(4)=sumOfD.z;
  grip(5)=torque.x;
  grip(6)=torque.y;
  grip(7)=torque.z;
}

#if 0 //OBSOLETE
/// returns the number of touch-sensors
uint ors::Graph::getTouchDimension() {
  Body *n;
  uint i=0, j;
  
  // count touchsensors
  for_list(j, n, bodies) if(ats.getValue<double>(n->ats, "touchsensor", 0)) i++;
  td=i;
  return i;
}

/// returns the touch vector (penetrations) of all touch-sensors
void ors::Graph::getTouchState(arr& touch) {
  if(!td) td=getTouchDimension();
  arr pen;
  getPenetrationState(pen);
  Body *n;
  uint i=0, j;
  for_list(j, n, bodies) {
    if(ats.getValue<double>(n->ats, "touchsensor", 0)) {
      touch(i)=pen(n->index);
      i++;
    }
  }
  CHECK(i==td, "");
}
#endif

/** @brief */
double ors::Graph::getEnergy() const {
  Body *n;
  uint j;
  double m, v, E;
  ors::Matrix I;
  ors::Vector w;
  
  E=0.;
  for_list(j, n, bodies) {
    m=n->mass;
    ors::Quaternion &rot = n->X.rot;
    I=(rot).getMatrix() * n->inertia * (-rot).getMatrix();
    v=n->X.vel.length();
    w=n->X.angvel;
    E += .5*m*v*v;
    E += 9.81 * m * n->X.pos.z;
    E += .5*(w*(I*w));
  }
  
  return E;
}

void ors::Graph::addObject(ors::Body *b) {
  bodies.append(b);
  int ibody = bodies.N - 1;
  uint i; ors::Shape *s;
  for_list(i, s, b->shapes) {
    s->ibody = ibody;
    s->index = shapes.N;
    shapes.append(s);
  }
}

void ors::Graph::removeNonShapeBodies() {
  for_list_rev_(Body, b, bodies) if(!b->shapes.N && !b->outLinks.N) {
    for_list_rev_(Joint, j, b->inLinks) joints.removeValue(j);
    bodies.remove(b_COUNT);
    delete b;
  }
  for_list_(Body, bb, bodies) bb->index=bb_COUNT;
  for_list_(Joint, j, joints) { j->index=j_COUNT;  j->ifrom = j->from->index;  j->ito = j->to->index;  }
  for_list_(Shape, s, shapes) s->ibody = s->body->index;
  proxies.clear();
}

void ors::Graph::meldFixedJoint() {
  for_list_(Joint, j, joints) if(j->type==JT_fixed) {
    Body *a = j->from;
    Body *b = j->to;
    //reassociate shapes with a
    for_list_(Shape, s, b->shapes) {
      s->body=a;
      s->ibody = a->index;
      s->rel = j->A * s->rel;
      a->shapes.append(s);
    }
    b->shapes.clear();
    //reassociate out-joints with a
    for_list_(Joint, jj, b->outLinks) {
      jj->from=a;
      jj->ifrom=a->index;
      jj->A = j->A * j->B * jj->A;
      a->outLinks.append(jj);
    }
    b->outLinks.clear();
  }
}

// ------------------ end slGraph ---------------------


//===========================================================================
//
// helper routines -- in a classical C interface
//



/** @brief get the center of mass, total velocity, and total angular momemtum */
void ors::Graph::getTotals(ors::Vector& c, ors::Vector& v, ors::Vector& l, ors::Quaternion& ori) const {
  Body *n;
  uint j;
  double m, M;
  
  //dMass mass;
  ors::Matrix ID;
  //ors::Matrix TP;
  ors::Vector r, o;
  
  ID.setId();
  c.setZero();
  v.setZero();
  l.setZero();
  o.setZero();
  //Iall.setZero();
  M=0.;
  for_list(j, n, bodies) {
    l+=n->inertia*n->X.angvel;
    //TP.setTensorProduct(n->X.p, n->X.p);
    //Iall+=m*((n->X.p*n->X.p)*ID + TP);
    
    m=n->mass;
    l+=m*(n->X.pos ^ n->X.vel);
    o+=m*n->X.rot.getVec(r);
    
    M+=m;
    c+=m*n->X.pos;
    v+=m*n->X.vel;
  }
  c/=M;
  v/=M;
  o/=M;
  ori.setVec(o);
}

#endif

#undef LEN

//===========================================================================
// Util
/**
 * @brief Return the position of the submesh in the obj file in bytes (can be
 * used by fseek).
 *
 * @param filename file to parse.
 */
MT::Array<std::tuple<long, long> > getSubMeshPositions(const char* filename) {
  CHECK(MT::String(filename).getLastN(3)=="obj", "getSubMeshPositions parses only obj files.");
  FILE* file;
  char buf[128];
  file = fopen(filename, "r");
  CHECK(file, "CheckManyShapes() failed: can't open data file " <<filename);
  int flag = 0;
  long start_pos = 0;
  long end_pos = 0;

  MT::Array<std::tuple<long, long> > result;
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
      case 'v': {
        if (flag > 0) {
          end_pos = ftell(file) - 1;
          auto tmp = std::make_tuple(start_pos, end_pos);
          result.append(tmp);
          start_pos = end_pos;
          flag =0; }
      } break;
      case 'f': {
        flag=1;
      } break;
    }
  }

  end_pos = ftell(file) - 1;
  auto tmp = std::make_tuple(start_pos, end_pos);
  result.append(tmp);
  return result;
}

//===========================================================================
//-- template instantiations

#include <Core/util_t.h>
template void MT::Parameter<ors::Vector>::initialize();

#ifndef  MT_ORS_ONLY_BASICS
template void MT::save<ors::Graph>(const ors::Graph&, const char*);
template MT::Array<ors::Shape*>::Array(uint);
template ors::Shape* listFindByName(const MT::Array<ors::Shape*>&,const char*);

#include <Core/array_t.h>
template MT::Array<ors::Joint*>::Array();
inline std::istream& operator>>(std::istream& is, TaskVariable*&) {NIY}
inline std::ostream& operator<<(std::ostream& os, const TaskVariable*&) {NIY}
template struct MT::Array<TaskVariable*>;
#endif
/** @} */
