/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
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
#include <climits>
#include "ors.h"
#include "ors_swift.h"
#include "ors_physx.h"
#include "ors_ode.h"
#include <Geo/qhull.h>
#include <Gui/opengl.h>
#include <Algo/algos.h>

#ifndef MLR_ORS_ONLY_BASICS
#  include <Core/registry.h>
//#  include <Gui/plot.h>
#endif
#ifdef MLR_extern_ply
#  include <extern/ply/ply.h>
#endif

#define ORS_NO_DYNAMICS_IN_FRAMES

#define SL_DEBUG_LEVEL 1
#define SL_DEBUG(l, x) if(l<=SL_DEBUG_LEVEL) x;

#define Qstate

void lib_ors(){ cout <<"force loading lib/ors" <<endl; }

#define LEN .2

#ifndef MLR_ORS_ONLY_BASICS

uint ors::KinematicWorld::setJointStateCount = 0;

//===========================================================================
//
// contants
//

ors::Body& NoBody = *((ors::Body*)NULL);
ors::Shape& NoShape = *((ors::Shape*)NULL);
ors::Joint& NoJoint = *((ors::Joint*)NULL);
ors::KinematicWorld& NoWorld = *((ors::KinematicWorld*)NULL);

//===========================================================================
//
// Body implementations
//

//ors::Body::Body() { reset(); }

//ors::Body::Body(const Body& b) { reset(); *this=b; }

ors::Body::Body(KinematicWorld& _world, const Body* copyBody):world(_world) {
  reset();
  index=world.bodies.N;
  world.bodies.append(this);
  if(copyBody) *this=*copyBody;
}

ors::Body::~Body() {
  reset();
  while(inLinks.N) delete inLinks.last();
  while(outLinks.N) delete outLinks.last();
  while(shapes.N) delete shapes.last();
  world.bodies.removeValue(this);
  listReindex(world.bodies);
}

void ors::Body::reset() {
  listDelete(ats);
  X.setZero();
  type=dynamicBT;
  shapes.memMove=true;
  com.setZero();
  mass = 0.;
  inertia.setZero();
}

void ors::Body::parseAts() {
  //interpret some of the attributes
  arr x;
  mlr::String str;
  ats.get(X, "X");
  ats.get(X, "pose");
  
  //mass properties
  double d;
  if(ats.get(d, "mass")) {
    mass=d;
    inertia.setId();
    inertia *= .2*d;
  }

  type=dynamicBT;
  if(ats["fixed"])       type=staticBT;
  if(ats["static"])      type=staticBT;
  if(ats["kinematic"])   type=kinematicBT;
  if(ats.get(d,"dyntype")) type=(BodyType)d;

  // SHAPE handling
  Node* item;
  // a mesh which consists of multiple convex sub meshes creates multiple
  // shapes that belong to the same body
  item = ats.getNode("meshes");
  if(item){
    mlr::FileToken *file = item->getValue<mlr::FileToken>();
    CHECK(file,"somethings wrong");

    // if mesh is not .obj we only have one shape
    if(!file->name.endsWith("obj")) {
      new Shape(world, *this);
    }else{  // if .obj file create Shape for all submeshes
      auto subMeshPositions = getSubMeshPositions(file->name);
      for(uint i=0;i<subMeshPositions.d0;i++){
        auto parsing_pos = subMeshPositions[i];
        Shape *s = new Shape(world, *this);
        s->mesh.parsing_pos_start = parsing_pos(0);
        s->mesh.parsing_pos_end = parsing_pos(1);
        s->mesh.readObjFile(file->getIs());
        s->mesh.makeConvexHull();
        s->type=meshST;
      }
    }
  }

  // add shape if there is no shape exists yet
  if(ats.getNode("type") && !shapes.N){
    Shape *s = new Shape(world, *this);
    s->name = name;
  }

  // copy body attributes to shapes 
  for(Shape *s:shapes) { s->ats=ats;  s->parseAts(); }
  //TODO check if this works! coupled to the listDelete below
  Node *it=ats["type"]; if(it){ delete it; /*ats.removeValue(it);*/ ats.index(); }
  //  listDelete(ats);
}

void ors::Body::write(std::ostream& os) const {
  if(!X.isZero()) os <<"pose=<T " <<X <<" > ";
  if(mass) os <<"mass=" <<mass <<' ';
  if(type!=dynamicBT) os <<"dyntype=" <<(int)type <<' ';
//  uint i; Node *a;
//  for(Type *  a:  ats)
//      if(a->keys(0)!="X" && a->keys(0)!="pose") os <<*a <<' ';
}

void ors::Body::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("body '" <<name <<"' read error: in ");
  parseAts();
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

ors::Shape::Shape(KinematicWorld &_world, Body& b, const Shape *copyShape, bool referenceMeshOnCopy): world(_world), /*ibody(UINT_MAX),*/ body(NULL) {
  reset();
  CHECK(&world,"you need at least a world to attach this shape to!");
  index=world.shapes.N;
  world.shapes.append(this);
  if(&b){
    body = &b;
    b.shapes.append(this);
  }
  if(copyShape) copy(*copyShape, referenceMeshOnCopy);
}

ors::Shape::~Shape() {
  reset();
  if(body){
    body->shapes.removeValue(this);
    listReindex(body->shapes);
  }
  world.shapes.removeValue(this);
  listReindex(world.shapes);
}

void ors::Shape::copy(const Shape& s, bool referenceMeshOnCopy){
  name=s.name; X=s.X; rel=s.rel; type=s.type;
  memmove(size, s.size, 4*sizeof(double)); memmove(color, s.color, 3*sizeof(double));
  if(!referenceMeshOnCopy){
    mesh=s.mesh;
    sscCore=s.sscCore;
  }else{
    mesh.V.referTo(s.mesh.V);
    mesh.T.referTo(s.mesh.T);
    mesh.C.referTo(s.mesh.C);
    mesh.Vn.referTo(s.mesh.Vn);
    sscCore.V.referTo(s.sscCore.V);
    sscCore.T.referTo(s.sscCore.T);
    sscCore.C.referTo(s.sscCore.C);
    sscCore.Vn.referTo(s.sscCore.Vn);
  }
  mesh_radius=s.mesh_radius; cont=s.cont;
  ats=s.ats;
}

void ors::Shape::parseAts() {
  double d;
  arr x;
  mlr::String str;
  mlr::FileToken fil;
  ats.get(rel, "rel");
  if(ats.get(x, "size"))          { CHECK_EQ(x.N,4,"size=[] needs 4 entries"); memmove(size, x.p, 4*sizeof(double)); }
  if(ats.get(x, "color"))         { CHECK_EQ(x.N,3,"color=[] needs 3 entries"); memmove(color, x.p, 3*sizeof(double)); }
  if(ats.get(d, "type"))       { type=(ShapeType)(int)d;}
  if(ats["contact"])           { cont=true; }
  if(ats.get(fil, "mesh"))     { mesh.read(fil.getIs(), fil.name.getLastN(3).p); }
  if(ats.get(d, "meshscale"))  { mesh.scale(d); }

  //create mesh for basic shapes
  switch(type) {
    case ors::noneST: HALT("shapes should have a type - somehow wrong initialization..."); break;
    case ors::boxST:
      mesh.setBox();
      mesh.scale(size[0], size[1], size[2]);
      break;
    case ors::sphereST:
      mesh.setSphere();
      mesh.scale(size[3], size[3], size[3]);
      break;
    case ors::cylinderST:
      CHECK(size[3]>1e-10,"");
      mesh.setCylinder(size[3], size[2]);
      break;
    case ors::cappedCylinderST:
      CHECK(size[3]>1e-10,"");
      mesh.setCappedCylinder(size[3], size[2]);
      break;
    case ors::SSBoxST:
      HALT("deprecated?");
      mesh.setSSBox(size[0], size[1], size[2], size[3]);
      break;
    case ors::markerST:
      break;
    case ors::meshST:
    case ors::pointCloudST:
      CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
      break;
    case ors::ssCvxST:
      CHECK(size[3]>1e-10,"");
      CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
      sscCore=mesh;
      mesh.setSSCvx(sscCore, size[3]);
      break;
    case ors::ssBoxST:
      CHECK(size[3]>1e-10,"");
      sscCore.setBox();
      sscCore.scale(size[0], size[1], size[2]);
      mesh.setSSCvx(sscCore, size[3]);
      break;
    default: NIY;
  }

  //center the mesh:
  if(mesh.V.N){
    Vector c = mesh.center();
    if(c.length()>1e-8 && !ats["rel_includes_mesh_center"]){
      rel.addRelativeTranslation(c);
      ats.append<bool>({"rel_includes_mesh_center"}, {}, true);
    }
    mesh_radius = mesh.getRadius();
  }

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
  listDelete(ats);
  X.setZero();
  rel.setZero();
  mesh.V.clear();
  mesh_radius=0.;
  cont=false;
}

void ors::Shape::write(std::ostream& os) const {
  os <<"type=" <<type <<' ';
  os <<"size=[" <<size[0] <<' '<<size[1] <<' '<<size[2] <<' '<<size[3] <<"] ";
  if(!rel.isZero()) os <<"rel=<T " <<rel <<" > ";
  for(Node * a: ats)
  if(a->keys(0)!="rel" && a->keys(0)!="type" && a->keys(0)!="size") os <<*a <<' ';
}

void ors::Shape::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("shape read error");
  parseAts();
}

uintA stringListToShapeIndices(const mlr::Array<const char*>& names, const mlr::Array<ors::Shape*>& shapes) {
  uintA I(names.N);
  for(uint i=0; i<names.N; i++) {
    ors::Shape *s = listFindByName(shapes, names(i));
    if(!s) HALT("shape name '"<<names(i)<<"' doesn't exist");
    I(i) = s->index;
  }
  return I;
}

uintA shapesToShapeIndices(const mlr::Array<ors::Shape*>& shapes) {
  uintA I;
  resizeAs(I, shapes);
  for(uint i=0; i<shapes.N; i++) I.elem(i) = shapes.elem(i)->index;
  return I;
}

void makeConvexHulls(ShapeL& shapes){
  for(ors::Shape *s: shapes) s->mesh.makeConvexHull();
}

void makeSSBoxApproximations(ShapeL& shapes){
//  for(ors::Shape *s: shapes) s->mesh.makeSSBox(s->mesh.V);
  for(uint i=0;i<shapes.N;i++){
    ors::Shape *s=shapes(i);
    if(!(s->type==ors::meshST && s->mesh.V.N)) continue;
    ors::Transformation t;
    arr x;
    s->mesh.makeSSBox(x, t, s->mesh.V);
    s->type = ors::ssBoxST;
    s->size[0]=2.*x(0); s->size[1]=2.*x(1); s->size[2]=2.*x(2); s->size[3]=x(3);
    s->mesh.setSSBox(s->size[0], s->size[1], s->size[2], s->size[3]);
    s->rel.appendTransformation(t);
  }
}

void computeMeshNormals(ShapeL& shapes){
  for(ors::Shape *s: shapes) if(!s->mesh.Vn.N) s->mesh.computeNormals();
}


//===========================================================================
//
// Joint implementations
//

bool always_unlocked(void*) { return false; }

ors::Joint::Joint(KinematicWorld& G, Body *f, Body *t, const Joint* copyJoint)
  : world(G), index(0), qIndex(UINT_MAX), from(f), to(t), mimic(NULL), agent(0), constrainToZeroVel(false), H(1.) {
  reset();
  if(copyJoint) *this=*copyJoint;
  index=world.joints.N;
  world.joints.append(this);
  f->outLinks.append(this);
  t-> inLinks.append(this);
  world.q.clear();
  world.qdot.clear();
  world.qdim.clear();
}

ors::Joint::~Joint() {
  world.checkConsistency();
  reset();
  if(from){ from->outLinks.removeValue(this); listReindex(from->outLinks); }
  if(to){   to->inLinks.removeValue(this); listReindex(to->inLinks); }
  world.joints.removeValue(this);
  listReindex(world.joints);
  world.q.clear();
  world.qdot.clear();
  world.qdim.clear();
}

void ors::Joint::reset() { 
  listDelete(ats); A.setZero(); B.setZero(); Q.setZero(); X.setZero(); axis.setZero(); limits.clear(); H=1.; type=JT_none; 
  locker=NULL;
}

void ors::Joint::parseAts() {
  //interpret some of the attributes
  double d=0.;
  ats.get(A, "A");
  ats.get(A, "from");
  if(ats["BinvA"]) B.setInverse(A);
  ats.get(B, "B");
  ats.get(B, "to");
  ats.get(Q, "Q");
  ats.get(X, "X");
  ats.get(H, "ctrl_H");
  if(ats.get(d, "type")) type=(JointType)(int)d; else type=JT_hingeX;
  if(type==JT_rigid && !Q.isZero()){ A.appendTransformation(Q); Q.setZero(); }
  if(ats.get(d, "q")){
    if(type==JT_hingeX) Q.addRelativeRotationRad(d, 1., 0., 0.);
    if(type==JT_rigid)  A.addRelativeRotationRad(d, 1., 0., 0.);
    if(type==JT_transX) Q.addRelativeTranslation(d, 0., 0.);
  }
  if(ats.get(d, "agent")) agent=(uint)d;
  if(ats["fixed"]) agent=UINT_MAX;
  //axis
  arr axis;
  ats.get(axis, "axis");
  if(axis.N) {
    CHECK_EQ(axis.N,3,"");
    Vector ax(axis);
    Transformation f;
    f.setZero();
    f.rot.setDiff(Vector_x, ax);
    A = A * f;
    B = -f * B;
  }
  //limit
  arr ctrl_limits;
  ats.get(limits, "limits");
  if(limits.N && type!=JT_rigid){
    CHECK_EQ(limits.N,2*qDim(), "parsed limits have wrong dimension");
  }
  ats.get(ctrl_limits, "ctrl_limits");
  if(ctrl_limits.N && type!=JT_rigid){
    if(!limits.N) limits.resizeAs(ctrl_limits).setZero();
    CHECK_EQ(3,ctrl_limits.N, "parsed ctrl_limits have wrong dimension");
    limits.append(ctrl_limits);
  }
  //coupled to another joint requires post-processing by the Graph::read!!
  if(ats["mimic"]) mimic=(Joint*)1;
}

uint ors::Joint::qDim() {
  if(type>=JT_hingeX && type<=JT_transZ) return 1;
  if(type==JT_transXY) return 2;
  if(type==JT_transXYPhi) return 3;
  if(type==JT_phiTransXY) return 3;
  if(type==JT_trans3) return 3;
  if(type==JT_universal) return 2;
  if(type==JT_quatBall) return 4;
  if(type==JT_free) return 7;
  if(type==JT_glue || type==JT_rigid || type==JT_none) return 0;
  HALT("shouldn't be here");
  return 0;
}

void ors::Joint::write(std::ostream& os) const {
  os <<"type=" <<type <<' ';
  if(!A.isZero()) os <<"from=<T " <<A <<" > ";
  if(!B.isZero()) os <<"to=<T " <<B <<" > ";
  if(!Q.isZero()) os <<"Q=<T " <<Q <<" > ";
  for(Node * a: ats)
  if(a->keys(0)!="A" && a->keys(0)!="from"
      && a->keys(0)!="axis" //because this was subsumed in A during read
      && a->keys(0)!="B" && a->keys(0)!="to"
      && a->keys(0)!="Q" && a->keys(0)!="q"
      && a->keys(0)!="type") os <<*a <<' ';
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

namespace ors{
struct sKinematicWorld{
  OpenGL *gl;
  SwiftInterface *swift;
  PhysXInterface *physx;
  OdeInterface *ode;
  bool swiftIsReference;
  sKinematicWorld():gl(NULL), swift(NULL), physx(NULL), ode(NULL), swiftIsReference(false) {}
  ~sKinematicWorld(){
    if(gl) delete gl;
    if(swift && !swiftIsReference) delete swift;
    if(physx) delete physx;
    if(ode) delete ode;
  }
};
}

ors::KinematicWorld::KinematicWorld():s(NULL),q_agent(0),isLinkTree(false) {
  bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true;
  s=new sKinematicWorld;
}

ors::KinematicWorld::KinematicWorld(const ors::KinematicWorld& other):s(NULL),q_agent(0),isLinkTree(false)  {
  bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true;
  s=new sKinematicWorld;
  *this = other;
}

ors::KinematicWorld::KinematicWorld(const char* filename):s(NULL),q_agent(0),isLinkTree(false)  {
  bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true;
  s=new sKinematicWorld;
  init(filename);
}
ors::KinematicWorld::~KinematicWorld() {
  clear();
  delete s;
  s=NULL;
}

void ors::KinematicWorld::init(const char* filename) {
  *this <<FILE(filename);
}

void ors::KinematicWorld::clear() {
  qdim.clear();
  q.clear();
  qdot.clear();
  listDelete(proxies); checkConsistency();
  while(shapes.N){ delete shapes.last(); checkConsistency(); }
  while(joints.N){ delete joints.last(); checkConsistency();}
  while(bodies.N){ delete bodies.last(); checkConsistency();}
  isLinkTree=false;
}

void ors::KinematicWorld::copy(const ors::KinematicWorld& G, bool referenceMeshesAndSwiftOnCopy) {
  clear();
#if 1
  listCopy(proxies, G.proxies);
  for(Body *b:G.bodies) new Body(*this, b);
  for(Shape *s:G.shapes){
//    if(referenceMeshesAndSwiftOnCopy && !s->mesh.Vn.N) s->mesh.computeNormals(); // the copy references these normals -> if they're not precomputed, you can never display the copy
    new Shape(*this, (s->body?*bodies(s->body->index):NoBody), s, referenceMeshesAndSwiftOnCopy);
  }
  for(Joint *j:G.joints){
    Joint *jj=
        new Joint(*this, bodies(j->from->index), bodies(j->to->index), j);
    if(j->mimic) jj->mimic = joints(j->mimic->index);
  }
  if(referenceMeshesAndSwiftOnCopy){
    s->swift = G.s->swift;
    s->swiftIsReference=true;
  }
  q = G.q;
  qdot = G.qdot;
  qdim = G.qdim;
  q_agent = G.q_agent;
  isLinkTree = G.isLinkTree;
#else
  q = G.q;
  qdot = G.qdot;
  qdim = G.qdim;
  q_agent = G.q_agent;
  isLinkTree = G.isLinkTree;
  listCopy(proxies, G.proxies);
  listCopy(joints, G.joints);
  for(Joint *j: joints) if(j->mimic){
    mlr::String jointName;
    bool good = j->ats.getValue<mlr::String>(jointName, "mimic");
    CHECK(good, "something is wrong");
    j->mimic = listFindByName(G.joints, jointName);
    if(!j->mimic) HALT("The joint '" <<*j <<"' is declared coupled to '" <<jointName <<"' -- but that doesn't exist!");
    j->type = j->mimic->type;
  }
  listCopy(shapes, G.shapes);
  listCopy(bodies, G.bodies);
  graphMakeLists(bodies, joints);
  for(Body *  b:  bodies) b->shapes.clear();
  for(Shape *  s:  shapes) {
    b=bodies(s->ibody);
    s->body=b;
    b->shapes.append(s);
  }
#endif
}

/** @brief transforms (e.g., translates or rotates) the joints coordinate system):
  `adds' the transformation f to A and its inverse to B */
void ors::KinematicWorld::transformJoint(ors::Joint *e, const ors::Transformation &f) {
  e->A = e->A * f;
  e->B = -f * e->B;
}

void ors::KinematicWorld::makeLinkTree() {
  for(Joint *j: joints) {
    for(Shape *s: j->to->shapes)  s->rel = j->B * s->rel;
    for(Joint *j2: j->to->outLinks) j2->A = j->B * j2->A;
    j->B.setZero();
  }
  isLinkTree=true;
}

/** @brief KINEMATICS: given the (absolute) frames of root nodes and the relative frames
    on the edges, this calculates the absolute frames of all other nodes (propagating forward
    through trees and testing consistency of loops). */
void ors::KinematicWorld::calc_fwdPropagateFrames() {
  ors::Transformation f;
  BodyL todoBodies = bodies;
  for(Body *b: todoBodies) {
    for(Joint *j:b->outLinks){ //this has no bailout for loopy graphs!
      f = b->X;
      f.appendTransformation(j->A);
      j->X = f;
      if(j->type==JT_hingeX || j->type==JT_transX)  j->axis = j->X.rot.getX();
      if(j->type==JT_hingeY || j->type==JT_transY)  j->axis = j->X.rot.getY();
      if(j->type==JT_hingeZ || j->type==JT_transZ)  j->axis = j->X.rot.getZ();
      if(j->type==JT_transXYPhi)  j->axis = j->X.rot.getZ();
      if(j->type==JT_phiTransXY)  j->axis = j->X.rot.getZ();
      f.appendTransformation(j->Q);
      if(!isLinkTree) f.appendTransformation(j->B);
      j->to->X=f;
      todoBodies.setAppend(j->to);
    }
  }
  calc_fwdPropagateShapeFrames();
}

void ors::KinematicWorld::calc_fwdPropagateShapeFrames() {
  for(Shape *s: shapes) {
    if(s->body){
      s->X = s->body->X;
      s->X.appendTransformation(s->rel);
    }else{
      s->X = s->rel;
    }
  }
}

void ors::KinematicWorld::calc_missingAB_from_BodyAndJointFrames() {
  for(Joint *e: joints) {
    if(!e->X.isZero() && e->A.isZero() && e->B.isZero()) {
      e->A.setDifference(e->from->X, e->X);
      e->B.setDifference(e->X, e->to->X);
    }
  }
}

/** @brief given the absolute frames of all nodes and the two rigid (relative)
    frames A & B of each edge, this calculates the dynamic (relative) joint
    frame X for each edge (which includes joint transformation and errors) */
void ors::KinematicWorld::calc_Q_from_BodyFrames() {
  for(Joint *j:joints) {
    ors::Transformation A(j->from->X), B(j->to->X);
    A.appendTransformation(j->A);
    B.appendInvTransformation(j->B);
    j->Q.setDifference(A, B);
  }
}

/** @brief in all edge frames: remove any displacements, velocities and non-x rotations.
    After this, edges and nodes are not coherent anymore. You might want to call
    calcBodyFramesFromJoints() */
void ors::KinematicWorld::clearJointErrors() {
  ors::Vector xaxis(1, 0, 0);
  for(Joint *j:joints) {
    j->Q.pos.setZero();
    j->Q.vel.setZero();
    j->Q.rot.alignWith(xaxis);
    j->Q.angvel.makeColinear(xaxis);
  }
}

arr ors::KinematicWorld::naturalQmetric(double power) const {
#if 0
  if(!q.N) getJointStateDimension();
  arr Wdiag(q.N);
  Wdiag=1.;
  return Wdiag;
#else
  //compute generic q-metric depending on tree depth
  arr BM(bodies.N);
  BM=1.;
  for(uint i=BM.N; i--;) {
    for(uint j=0; j<bodies(i)->outLinks.N; j++) {
      BM(i) = mlr::MAX(BM(bodies(i)->outLinks(j)->to->index)+1., BM(i));
//      BM(i) += BM(bodies(i)->outLinks(j)->to->index);
    }
  }
  if(!q.N) getJointStateDimension();
  arr Wdiag(q.N);
  for(Joint *j: joints) {
    for(uint i=0; i<j->qDim(); i++) {
      if(j->agent==q_agent) Wdiag(j->qIndex+i) = ::pow(BM(j->to->index), power);
    }
  }
  return Wdiag;
#endif
}

/** @brief revert the topological orientation of a joint (edge),
   e.g., when choosing another body as root of a tree */
void ors::KinematicWorld::revertJoint(ors::Joint *j) {
  cout <<"reverting edge (" <<j->from->name <<' ' <<j->to->name <<")" <<endl;
  //revert
  j->from->outLinks.removeValue(j);
  j->to->inLinks.removeValue(j);
  Body *b=j->from; j->from=j->to; j->to=b;
  j->from->outLinks.append(j);
  j->to->inLinks.append(j);
  listReindex(j->from->outLinks);
  listReindex(j->from->inLinks);
  checkConsistency();

  ors::Transformation f;
  f=j->A;
  j->A.setInverse(j->B);
  j->B.setInverse(f);
  f=j->Q;
  j->Q.setInverse(f);
}

/** @brief re-orient all joints (edges) such that n becomes
  the root of the configuration */
void ors::KinematicWorld::reconfigureRoot(Body *root) {
  mlr::Array<Body*> list, list2;
  Body **m,**mstop;
  list.append(root);
  uintA level(bodies.N);
  level=0;
  int i=0;
  
  while(list.N>0) {
    i++;
    list2.clear();
    mstop=list.p+list.N;
    for(m=list.p; m!=mstop; m++) {
      level((*m)->index)=i;
      for_list(Joint,  e,  (*m)->inLinks) {
        if(!level(e->from->index)) { revertJoint(e); e_COUNT--; }
      }
      for(Joint *e: (*m)->outLinks) list2.append(e->to);
    }
    list=list2;
  }
  
  graphTopsort(bodies, joints);
}

void ors::KinematicWorld::analyzeJointStateDimensions() {
  uint maxagent=0;
  for(Joint *j: joints) if(j->agent>maxagent) maxagent=j->agent;
  qdim.resize(maxagent+1);
  qdim.setZero();
  for(Joint *j: joints) {
    if(!j->mimic){
      j->qIndex = qdim(j->agent);
      qdim(j->agent) += j->qDim();
    }else{
      CHECK_EQ(j->agent, j->mimic->agent, "");
      j->qIndex = j->mimic->qIndex;
    }
  }
}

/** @brief returns the joint (actuator) dimensionality */
uint ors::KinematicWorld::getJointStateDimension(int agent) const {
  if(agent==-1) agent=q_agent;
  CHECK(agent!=INT_MAX,"");
  if(!qdim.N) ((KinematicWorld*)this)->analyzeJointStateDimensions();
  CHECK((uint)agent<qdim.N,"don't have that agent (analyzeJointStateDimensions before?)");
  return qdim(agent);
}

void ors::KinematicWorld::getJointState(arr &_q, arr& _qdot, int agent) const {
  if(!qdim.N) ((KinematicWorld*)this)->analyzeJointStateDimensions();
  if(q.N!=getJointStateDimension(agent)) ((KinematicWorld*)this)->calc_q_from_Q(false, agent);

  _q=q;
  if(&_qdot){
    _qdot=qdot;
    if(!_qdot.N) _qdot.resizeAs(q).setZero();
  }
}

arr ors::KinematicWorld::getJointState(int agent) const {
  if(!qdim.N) ((KinematicWorld*)this)->analyzeJointStateDimensions();
  if(q.N!=getJointStateDimension(agent)) ((KinematicWorld*)this)->calc_q_from_Q(false, agent);

  return q;
}

/** @brief returns the vector of joint limts */
arr ors::KinematicWorld::getLimits() const {
  uint N=getJointStateDimension();
  arr limits(N,2);
  limits.setZero();
  for(Joint *j: joints) if(j->agent==q_agent && j->limits.N){
    uint i=j->qIndex;
    uint d=j->qDim();
    for(uint k=0;k<d;k++){//in case joint has multiple dimensions
      limits(i+k,0)=j->limits(0); //lo
      limits(i+k,1)=j->limits(1); //up
    }
  }
//  cout <<"limits=" <<limits <<endl;
  return limits;
}

void ors::KinematicWorld::zeroGaugeJoints() {
  Joint *e;
  ors::Vector w;
  for(Body *  n:  bodies) if(n->type!=staticBT) {
    e=n->inLinks(0);
    if(e) {
      w=e->Q.rot / e->Q.angvel; e->Q.angvel.setZero();
      e->A.appendTransformation(e->Q);
      e->Q.setZero();
      e->Q.angvel=w;
    }
  }
}

arr ors::KinematicWorld::calc_q_from_Q(ors::Joint* j, bool calcVels) {
  arr q;
  switch(j->type) {
    case JT_hingeX:
    case JT_hingeY:
    case JT_hingeZ: {
      q.resize(1);
      //angle
      ors::Vector rotv;
      j->Q.rot.getRad(q(0), rotv);
      if(q(0)>MLR_PI) q(0)-=MLR_2PI;
      if(j->type==JT_hingeX && rotv*Vector_x<0.) q(0)=-q(0);
      if(j->type==JT_hingeY && rotv*Vector_y<0.) q(0)=-q(0);
      if(j->type==JT_hingeZ && rotv*Vector_z<0.) q(0)=-q(0);
      //velocity
      if(calcVels){
        qdot(0)=j->Q.angvel.length();
        if(j->type==JT_hingeX && j->Q.angvel*Vector_x<0.) qdot(0)=-qdot(0);
        if(j->type==JT_hingeY && j->Q.angvel*Vector_y<0.) qdot(0)=-qdot(0);
        if(j->type==JT_hingeZ && j->Q.angvel*Vector_z<0.) qdot(0)=-qdot(0);
      }
    } break;

    case JT_universal: {
      q.resize(2);
      //angle
      if(fabs(j->Q.rot.w)>1e-15) {
        q(0) = 2.0 * atan(j->Q.rot.x/j->Q.rot.w);
        q(1) = 2.0 * atan(j->Q.rot.y/j->Q.rot.w);
      } else {
        q(0) = MLR_PI;
        q(1) = MLR_PI;
      }
      
      if(calcVels) NIY; // velocity: need to fix
    } break;

    case JT_quatBall: {
      q.resize(4);
      q(0)=j->Q.rot.w;
      q(1)=j->Q.rot.x;
      q(2)=j->Q.rot.y;
      q(3)=j->Q.rot.z;
      if(calcVels) NIY;  // velocity: need to fix
    } break;

    case JT_transX: {
      q.resize(1);
      q(0)=j->Q.pos.x;
      if(calcVels) qdot(0)=j->Q.vel.x;
    } break;
    case JT_transY: {
      q.resize(1);
      q(0)=j->Q.pos.y;
      if(calcVels) qdot(0)=j->Q.vel.y;
    } break;
    case JT_transZ: {
      q.resize(1);
      q(0)=j->Q.pos.z;
      if(calcVels) qdot(0)=j->Q.vel.z;
    } break;
    case JT_transXY: {
      q.resize(1);
      q(0)=j->Q.pos.x;  
      q(1)=j->Q.pos.y;
      if(calcVels){  
        qdot(0)=j->Q.vel.x;  
        qdot(1)=j->Q.vel.y;  }
    } break;
    case JT_transXYPhi: {
      q.resize(3);
      q(0)=j->Q.pos.x;
      q(1)=j->Q.pos.y;
      ors::Vector rotv;
      j->Q.rot.getRad(q(2), rotv);
      if(q(2)>MLR_PI) q(2)-=MLR_2PI;
      if(rotv*Vector_z<0.) q(2)=-q(2);
      if(calcVels){
        qdot(0)=j->Q.vel.x;
        qdot(1)=j->Q.vel.y;
        qdot(2)=j->Q.angvel.length();
        if(j->Q.angvel*Vector_z<0.) qdot(0)=-qdot(0);
      }
    } break;
    case JT_phiTransXY: {
      q.resize(3);
      ors::Vector rotv;
      j->Q.rot.getRad(q(0), rotv);
      if(q(0)>MLR_PI) q(0)-=MLR_2PI;
      if(rotv*Vector_z<0.) q(0)=-q(0);
      ors::Vector relpos = j->Q.rot/j->Q.pos;
      q(1)=relpos.x;
      q(2)=relpos.y;
      if(calcVels){
        qdot(0)=j->Q.angvel.length();
        if(j->Q.angvel*Vector_z<0.) qdot(0)=-qdot(0);
        ors::Vector relvel = j->Q.rot/j->Q.vel;
        qdot(1)=relvel.x;
        qdot(2)=relvel.y;
      }
    } break;
    case JT_trans3: {
      q.resize(3);
      q(0)=j->Q.pos.x;
      q(1)=j->Q.pos.y;
      q(2)=j->Q.pos.z;
      if(calcVels) {
        qdot(0)=j->Q.vel.x;
        qdot(1)=j->Q.vel.y;
        qdot(2)=j->Q.vel.z;
      }
    } break;
    case JT_glue:
    case JT_rigid:
      break;
    case JT_free:
      q.resize(7);
      q(0)=j->Q.pos.x;
      q(1)=j->Q.pos.y;
      q(2)=j->Q.pos.z;
      q(3)=j->Q.rot.w;
      q(4)=j->Q.rot.x;
      q(5)=j->Q.rot.y;
      q(6)=j->Q.rot.z;
      if(calcVels) NIY;  // velocity: need to fix
      break;
    default: NIY;
  }
  return q;
}

void ors::KinematicWorld::calc_q_from_Q(bool calcVels, int agent) {
  if(agent == -1) agent = q_agent;
//  ors::Quaternion rot;
  
  uint N=getJointStateDimension(agent);
  q.resize(N);
  qdot.resize(N).setZero();

  uint n=0;
  for(Joint *j: joints) if(j->agent==(uint)agent){
    if(j->mimic) continue; //don't count dependent joints
    CHECK_EQ(j->qIndex,n,"joint indexing is inconsistent");
    arr joint_q = calc_q_from_Q(j, calcVels);
    //TODO is there a better way?
    for(uint i=0; i<joint_q.N; ++i)
      q(n+i) = joint_q(i);
    n += joint_q.N;
  }
  CHECK_EQ(n,N,"");
}

void ors::KinematicWorld::calc_Q_from_q(bool calcVels, int agent){
  if(agent==-1) agent = q_agent;
  uint n=0;
  for(Joint *j: joints) if(j->agent==(uint)agent){
    if(j->mimic){
      j->Q=j->mimic->Q;
    }else{
      CHECK_EQ(j->qIndex,n,"joint indexing is inconsistent");
      switch(j->type) {
        case JT_hingeX: {
          j->Q.rot.setRadX(q(n));
          if(calcVels){  j->Q.angvel.set(qdot(n) ,0., 0.);  j->Q.zeroVels=false;  }
          n++;
        } break;

        case JT_hingeY: {
          j->Q.rot.setRadY(q(n));
          if(calcVels){  j->Q.angvel.set(0., qdot(n) ,0.);  j->Q.zeroVels=false;  }
          n++;
        } break;

        case JT_hingeZ: {
          j->Q.rot.setRadZ(q(n));
          if(calcVels){  j->Q.angvel.set(0., 0., qdot(n));  j->Q.zeroVels=false;  }
          n++;
        } break;

        case JT_universal:{
          ors::Quaternion rot1, rot2;
          rot1.setRadX(q(n));
          rot2.setRadY(q(n+1));
          j->Q.rot = rot1*rot2;
          if(calcVels) NIY;
          n+=2;
        } break;

        case JT_quatBall:{
          j->Q.rot.set(q.p+n);
          j->Q.rot.normalize();
          j->Q.rot.isZero=false; //WHY? (gradient check fails without!)
          if(calcVels) NIY;
          n+=4;
        } break;

        case JT_free:{
          j->Q.pos.set(q.p+n);
          if(calcVels){ j->Q.vel.set(qdot.p+n); j->Q.zeroVels=false; }
          j->Q.rot.set(q.p+n+3);
          j->Q.rot.normalize();
          j->Q.rot.isZero=false;
          if(calcVels) NIY;
          n+=7;
        } break;

        case JT_transX: {
          j->Q.pos = q(n)*Vector_x;
          if(calcVels){ j->Q.vel.set(qdot(n), 0., 0.); j->Q.zeroVels=false; }
          n++;
        } break;

        case JT_transY: {
          j->Q.pos = q(n)*Vector_y;
          if(calcVels){ j->Q.vel.set(0., qdot(n), 0.); j->Q.zeroVels=false; }
          n++;
        } break;

        case JT_transZ: {
          j->Q.pos = q(n)*Vector_z;
          if(calcVels){ j->Q.vel.set(0., 0., qdot(n)); j->Q.zeroVels=false; }
          n++;
        } break;

        case JT_transXY: {
          j->Q.pos.set(q(n), q(n+1), 0.);
          if(calcVels){ j->Q.vel.set(qdot(n), qdot(n+1), 0.); j->Q.zeroVels=false; }
          n+=2;
        } break;

        case JT_trans3: {
          j->Q.pos.set(q(n), q(n+1), q(n+2));
          if(calcVels){ j->Q.vel.set(qdot(n), qdot(n+1), qdot(n+2)); j->Q.zeroVels=false; }
          n+=3;
        } break;

        case JT_transXYPhi: {
          j->Q.pos.set(q(n), q(n+1), 0.);
          j->Q.rot.setRadZ(q(n+2));
          if(calcVels){
            j->Q.vel.set(qdot(n), qdot(n+1), 0.);  j->Q.zeroVels=false;
            j->Q.angvel.set(0., 0., qdot(n+2));  j->Q.zeroVels=false;
          }
          n+=3;
        } break;

        case JT_phiTransXY: {
          j->Q.rot.setRadZ(q(n));
          j->Q.pos = j->Q.rot*Vector(q(n+1), q(n+2), 0.);
          if(calcVels){
            j->Q.angvel.set(0., 0., qdot(n));  j->Q.zeroVels=false;
            j->Q.vel = j->Q.rot*Vector(qdot(n+1), qdot(n+2), 0.);  j->Q.zeroVels=false;
          }
          n+=3;
        } break;

        case JT_glue:
        case JT_rigid:
          j->Q.setZero();
          j->Q.zeroVels=true;
          break;
        default: NIY;
      }
    }
  }

  CHECK_EQ(n,q.N,"");
}


/** @brief sets the joint state vectors separated in positions and
  velocities */
void ors::KinematicWorld::setJointState(const arr& _q, const arr& _qdot, bool calcVels, int agent) {
  setJointStateCount++; //global counter

  uint N=getJointStateDimension(agent);
  CHECK(_q.N==N && (!(&_qdot) || _qdot.N==N), "wrong joint state dimensionalities");
  if(&_q!=&q) q=_q;
  if(&_qdot){ if(&_qdot!=&qdot) qdot=_qdot; }else qdot.clear();

  calc_Q_from_q(calcVels, agent);

  calc_fwdPropagateFrames();
}

void ors::KinematicWorld::setAgent(uint agent, bool calcVels){
  if(agent==q_agent) return; //nothing to do
  q_agent = agent;
  calc_q_from_Q(calcVels);
}



//===========================================================================
//
// core: kinematics and dynamics
//

/** @brief return the jacobian \f$J = \frac{\partial\phi_i(q)}{\partial q}\f$ of the position
  of the i-th body (3 x n tensor)*/
void ors::KinematicWorld::kinematicsPos(arr& y, arr& J, Body *b, const ors::Vector& rel) const {
  if(!b){
    MLR_MSG("WARNING: calling kinematics for NULL body");
    if(&y) y.resize(3).setZero();
    if(&J) J.resize(3, getJointStateDimension()).setZero();
    return;
  }

  //get position
  ors::Vector pos_world = b->X.pos;
  if(&rel) pos_world += b->X.rot*rel;
  if(&y) y = conv_vec2arr(pos_world); //return the output
  if(!&J) return; //do not return the Jacobian

  //get Jacobian
  uint N=getJointStateDimension();
  J.resize(3, N).setZero();
  if(b->inLinks.N) { //body a has no inLinks -> zero jacobian
    Joint *j=b->inLinks(0);
    while(j) { //loop backward down the kinematic tree
      uint j_idx=j->qIndex;
      if(j->agent==q_agent && j_idx>=N) CHECK(j->type==JT_glue || j->type==JT_rigid, "");
      if(j->agent==q_agent && j_idx<N){
        if(j->type==JT_hingeX || j->type==JT_hingeY || j->type==JT_hingeZ) {
          ors::Vector tmp = j->axis ^ (pos_world-j->X.pos);
          J(0, j_idx) += tmp.x;
          J(1, j_idx) += tmp.y;
          J(2, j_idx) += tmp.z;
        }
        else if(j->type==JT_transX || j->type==JT_transY || j->type==JT_transZ) {
          J(0, j_idx) += j->axis.x;
          J(1, j_idx) += j->axis.y;
          J(2, j_idx) += j->axis.z;
        }
        else if(j->type==JT_transXY) {
          if(j->mimic) NIY;
          arr R = j->X.rot.getArr();
          J.setMatrixBlock(R.sub(0,-1,0,1), 0, j_idx);
        }
        else if(j->type==JT_transXYPhi) {
          if(j->mimic) NIY;
          arr R = j->X.rot.getArr();
          J.setMatrixBlock(R.sub(0,-1,0,1), 0, j_idx);
          ors::Vector tmp = j->axis ^ (pos_world-(j->X.pos + j->X.rot*j->Q.pos));
          J(0, j_idx+2) += tmp.x;
          J(1, j_idx+2) += tmp.y;
          J(2, j_idx+2) += tmp.z;
        }
        else if(j->type==JT_phiTransXY) {
          if(j->mimic) NIY;
          ors::Vector tmp = j->axis ^ (pos_world-j->X.pos);
          J(0, j_idx) += tmp.x;
          J(1, j_idx) += tmp.y;
          J(2, j_idx) += tmp.z;
          arr R = (j->X.rot*j->Q.rot).getArr();
          J.setMatrixBlock(R.sub(0,-1,0,1), 0, j_idx+1);
        }
        if(j->type==JT_trans3 || j->type==JT_free) {
          if(j->mimic) NIY;
          arr R = j->X.rot.getArr();
          J.setMatrixBlock(R, 0, j_idx);
        }
        if(j->type==JT_quatBall || j->type==JT_free) {
          uint offset = (j->type==JT_free)?3:0;
          arr Jrot = j->X.rot.getArr() * j->Q.rot.getJacobian(); //transform w-vectors into world coordinate
          Jrot = crossProduct(Jrot, conv_vec2arr(pos_world-(j->X.pos+j->X.rot*j->Q.pos)) ); //cross-product of all 4 w-vectors with lever
          Jrot /= sqrt(sumOfSqr(q.subRef(j->qIndex+offset,j->qIndex+offset+3))); //account for the potential non-normalization of q
          for(uint i=0;i<4;i++) for(uint k=0;k<3;k++) J(k,j_idx+offset+i) += Jrot(k,i);
        }
      }
      if(!j->from->inLinks.N) break;
      j=j->from->inLinks(0);
    }
  }
}

/** @brief return the jacobian \f$J = \frac{\partial\phi_i(q)}{\partial q}\f$ of the position
  of the i-th body W.R.T. the 6 axes of an arbitrary shape-frame, NOT the robot's joints (3 x 6 tensor)
  WARNING: this does not check if s is actually in the kinematic chain from root to b.
*/
void ors::KinematicWorld::kinematicsPos_wrtFrame(arr& y, arr& J, Body *b, const ors::Vector& rel, Shape *s) const {
  if(!b && &J){ J.resize(3, getJointStateDimension()).setZero();  return; }

  //get position
  ors::Vector pos_world = b->X.pos;
  if(&rel) pos_world += b->X.rot*rel;
  if(&y) y = conv_vec2arr(pos_world); //return the output
  if(!&J) return; //do not return the Jacobian

  //get Jacobian
  J.resize(3, 6).setZero();
  ors::Vector diff = pos_world - s->X.pos;
  mlr::Array<ors::Vector> axes = {s->X.rot.getX(), s->X.rot.getY(), s->X.rot.getZ()};

  //3 translational axes
  for(uint i=0;i<3;i++){
    J(0, i) += axes(i).x;
    J(1, i) += axes(i).y;
    J(2, i) += axes(i).z;
  }

  //3 rotational axes
  for(uint i=0;i<3;i++){
    ors::Vector tmp = axes(i) ^ diff;
    J(0, 3+i) += tmp.x;
    J(1, 3+i) += tmp.y;
    J(2, 3+i) += tmp.z;
  }
}

/** @brief return the Hessian \f$H = \frac{\partial^2\phi_i(q)}{\partial q\partial q}\f$ of the position
  of the i-th body (3 x n x n tensor) */
void ors::KinematicWorld::hessianPos(arr& H, Body *b, ors::Vector *rel) const {
  HALT("this is buggy: a sign error: see examples/Ors/ors testKinematics");
  Joint *j1, *j2;
  uint j1_idx, j2_idx;
  ors::Vector tmp, pos_a;
  
  uint N=getJointStateDimension();
  
  //initialize Jacobian
  H.resize(3, N, N);
  H.setZero();
  
  //get reference frame
  pos_a = b->X.pos;
  if(rel) pos_a += b->X.rot*(*rel);
  
  if(b->inLinks.N) {
    j1=b->inLinks(0);
    while(j1) {
      CHECK_EQ(j1->agent,q_agent,"NIY");
      j1_idx=j1->qIndex;

      j2=j1;
      while(j2) {
        CHECK_EQ(j2->agent,q_agent,"NIY");
        j2_idx=j2->qIndex;

        if(j1->type>=JT_hingeX && j1->type<=JT_hingeZ && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //both are hinges
          tmp = j2->axis ^ (j1->axis ^ (pos_a-j1->X.pos));
          H(0, j1_idx, j2_idx) = H(0, j2_idx, j1_idx) = tmp.x;
          H(1, j1_idx, j2_idx) = H(1, j2_idx, j1_idx) = tmp.y;
          H(2, j1_idx, j2_idx) = H(2, j2_idx, j1_idx) = tmp.z;
        }
        else if(j1->type>=JT_transX && j1->type<=JT_transZ && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans, j=hinge
          tmp = j1->axis ^ j2->axis;
          H(0, j1_idx, j2_idx) = H(0, j2_idx, j1_idx) = tmp.x;
          H(1, j1_idx, j2_idx) = H(1, j2_idx, j1_idx) = tmp.y;
          H(2, j1_idx, j2_idx) = H(2, j2_idx, j1_idx) = tmp.z;
        }
        else if(j1->type==JT_transXY && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          NIY;
        }
        else if(j1->type==JT_transXYPhi && j1->type==JT_phiTransXY && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          NIY;
        }
        else if(j1->type==JT_trans3 && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          Matrix R,A;
          j1->X.rot.getMatrix(R.p());
          A.setSkew(j2->axis);
          R = R*A;
          H(0, j1_idx  , j2_idx) = H(0, j2_idx  , j1_idx) = R.m00;
          H(1, j1_idx  , j2_idx) = H(1, j2_idx  , j1_idx) = R.m10;
          H(2, j1_idx  , j2_idx) = H(2, j2_idx  , j1_idx) = R.m20;
          H(0, j1_idx+1, j2_idx) = H(0, j2_idx, j1_idx+1) = R.m01;
          H(1, j1_idx+1, j2_idx) = H(1, j2_idx, j1_idx+1) = R.m11;
          H(2, j1_idx+1, j2_idx) = H(2, j2_idx, j1_idx+1) = R.m21;
          H(0, j1_idx+2, j2_idx) = H(0, j2_idx, j1_idx+2) = R.m02;
          H(1, j1_idx+2, j2_idx) = H(1, j2_idx, j1_idx+2) = R.m12;
          H(2, j1_idx+2, j2_idx) = H(2, j2_idx, j1_idx+2) = R.m22;
        }
        else if(j1->type>=JT_hingeX && j1->type<=JT_hingeZ && j2->type>=JT_transX && j2->type<=JT_trans3) { //i=hinge, j=trans
          //nothing! Hessian is zero (ej is closer to root than ei)
        }
        else NIY;

        if(!j2->from->inLinks.N) break;
        j2=j2->from->inLinks(0);
      }
      if(!j1->from->inLinks.N) break;
      j1=j1->from->inLinks(0);
    }
  }
}

/* takes the joint state x and returns the jacobian dz of
   the position of the ith body (w.r.t. all joints) -> 2D array */
/// Jacobian of the i-th body's z-orientation vector
void ors::KinematicWorld::kinematicsVec(arr& y, arr& J, Body *b, const ors::Vector& vec) const {
  //get the vectoreference frame
  ors::Vector vec_referene;
  if(&vec) vec_referene = b->X.rot*vec;
  else     vec_referene = b->X.rot.getZ();
  if(&y) y = conv_vec2arr(vec_referene); //return the vec
  if(&J){
    arr A;
    axesMatrix(A, b);
    J = crossProduct(A, conv_vec2arr(vec_referene));
  }
}

/* takes the joint state x and returns the jacobian dz of
   the position of the ith body (w.r.t. all joints) -> 2D array */
/// Jacobian of the i-th body's z-orientation vector
void ors::KinematicWorld::kinematicsQuat(arr& y, arr& J, Body *b) const { //TODO: allow for relative quat
  ors::Quaternion rot_b = b->X.rot;
  if(&y) y = conv_quat2arr(rot_b); //return the vec
  if(&J){
    arr A;
    axesMatrix(A, b);
    J.resize(4, A.d1);
    for(uint i=0;i<J.d1;i++){
      ors::Quaternion tmp(0., 0.5*A(0,i), 0.5*A(1,i), 0.5*A(2,i) ); //this is unnormalized!!
      tmp = tmp * rot_b;
      J(0, i) = tmp.w;
      J(1, i) = tmp.x;
      J(2, i) = tmp.y;
      J(3, i) = tmp.z;
    }
  }
}

//* This Jacobian directly gives the implied rotation vector: multiplied with \dot q it gives the angular velocity of body b */
void ors::KinematicWorld::axesMatrix(arr& J, Body *b) const {
  uint N = getJointStateDimension();
  J.resize(3, N).setZero();
  if(b->inLinks.N) {
    Joint *j=b->inLinks(0);
    while(j) { //loop backward down the kinematic tree
      uint j_idx=j->qIndex;
      if(j->agent==q_agent && j_idx>=N) CHECK(j->type==JT_glue || j->type==JT_rigid, "");
      if(j->agent==q_agent && j_idx<N){
        if((j->type>=JT_hingeX && j->type<=JT_hingeZ) || j->type==JT_transXYPhi || j->type==JT_phiTransXY) {
          if(j->type==JT_transXYPhi) j_idx += 2; //refer to the phi only
          J(0, j_idx) += j->axis.x;
          J(1, j_idx) += j->axis.y;
          J(2, j_idx) += j->axis.z;
        }
        if(j->type==JT_quatBall || j->type==JT_free) {
          uint offset = (j->type==JT_free)?3:0;
          arr Jrot = j->X.rot.getArr() * j->Q.rot.getJacobian(); //transform w-vectors into world coordinate
          Jrot /= sqrt(sumOfSqr(q.subRef(j->qIndex+offset,j->qIndex+offset+3))); //account for the potential non-normalization of q
          for(uint i=0;i<4;i++) for(uint k=0;k<3;k++) J(k,j_idx+offset+i) += Jrot(k,i);
        }
        //all other joints: J=0 !!
      }
      if(!j->from->inLinks.N) break;
      j=j->from->inLinks(0);
    }
  }
}

/// The position vec1, attached to b1, relative to the frame of b2 (plus vec2)
void ors::KinematicWorld::kinematicsRelPos(arr& y, arr& J, Body *b1, const ors::Vector& vec1, Body *b2, const ors::Vector& vec2) const {
  arr y1,y2,J1,J2;
  kinematicsPos(y1, J1, b1, vec1);
  kinematicsPos(y2, J2, b2, vec2);
  arr Rinv = ~(b2->X.rot.getArr());
  y = Rinv * (y1 - y2);
  if(&J){
    arr A;
    axesMatrix(A, b2);
    J = Rinv * (J1 - J2 - crossProduct(A, y1 - y2));
  }
}

/// The vector vec1, attached to b1, relative to the frame of b2
void ors::KinematicWorld::kinematicsRelVec(arr& y, arr& J, Body *b1, const ors::Vector& vec1, Body *b2) const {
  arr y1,J1;
  kinematicsVec(y1, J1, b1, vec1);
//  kinematicsVec(y2, J2, b2, vec2);
  arr Rinv = ~(b2->X.rot.getArr());
  y = Rinv * y1;
  if(&J){
    arr A;
    axesMatrix(A, b2);
    J = Rinv * (J1 - crossProduct(A, y1));
  }
}

/// The position vec1, attached to b1, relative to the frame of b2 (plus vec2)
void ors::KinematicWorld::kinematicsRelRot(arr& y, arr& J, Body *b1, Body *b2) const {
  ors::Quaternion rot_b = b1->X.rot;
  if(&y) y = conv_vec2arr(rot_b.getVec());
  if(&J){
    double phi=acos(rot_b.w);
    double s=2.*phi/sin(phi);
    double ss=-2./(1.-mlr::sqr(rot_b.w)) * (1.-phi/tan(phi));
    arr A;
    axesMatrix(A, b1);
    J = 0.5 * (rot_b.w*A*s + crossProduct(A, y));
    J -= 0.5 * ss/s/s*(y*~y*A);
  }
}

/** @brief return the configuration's inertia tensor $M$ (n x n tensor)*/
void ors::KinematicWorld::inertia(arr& M) {
  uint j1_idx, j2_idx;
  ors::Transformation Xa, Xi, Xj;
  Joint *j1, *j2;
  ors::Vector vi, vj, ti, tj;
  double tmp;
  
  uint N=getJointStateDimension();
  
  //initialize Jacobian
  M.resize(N, N);
  M.setZero();
  
  for(Body *a: bodies) {
    //get reference frame
    Xa = a->X;
    
    j1=a->inLinks(0);
    while(j1) {
      j1_idx=j1->qIndex;
      
      Xi = j1->from->X;
      Xi.appendTransformation(j1->A);
      ti = Xi.rot.getX();
      
      vi = ti ^(Xa.pos-Xi.pos);
      
      j2=j1;
      while(j2) {
        j2_idx=j2->qIndex;
        
        Xj = j2->from->X;
        Xj.appendTransformation(j2->A);
        tj = Xj.rot.getX();
        
        vj = tj ^(Xa.pos-Xj.pos);
        
        tmp = a->mass * (vi*vj);
        //tmp += scalarProduct(a->a.inertia, ti, tj);
        
        M(j1_idx, j2_idx) += tmp;
        
        if(!j2->from->inLinks.N) break;
        j2=j2->from->inLinks(0);
      }
      if(!j1->from->inLinks.N) break;
      j1=j1->from->inLinks(0);
    }
  }
  //symmetric: fill in other half
  for(j1_idx=0; j1_idx<N; j1_idx++) for(j2_idx=0; j2_idx<j1_idx; j2_idx++) M(j2_idx, j1_idx) = M(j1_idx, j2_idx);
}

void ors::KinematicWorld::equationOfMotion(arr& M, arr& F, bool gravity) {
  ors::LinkTree tree; //TODO: HACK!! Danny: Why was there a static? This fails if there are more than 2 worlds
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  if(gravity){
    clearForces();
    gravityToForces();
  }
  if(!qdot.N) qdot.resize(q.N).setZero();
  ors::equationOfMotion(M, F, tree, qdot);
}

/** @brief return the joint accelerations \f$\ddot q\f$ given the
  joint torques \f$\tau\f$ (computed via Featherstone's Articulated Body Algorithm in O(n)) */
void ors::KinematicWorld::fwdDynamics(arr& qdd, const arr& qd, const arr& tau) {
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
void ors::KinematicWorld::inverseDynamics(arr& tau, const arr& qd, const arr& qdd) {
  static ors::LinkTree tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  ors::invDynamics(tau, tree, qd, qdd);
}

/*void ors::KinematicWorld::impulsePropagation(arr& qd1, const arr& qd0){
  static mlr::Array<Featherstone::Link> tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  mimickImpulsePropagation(tree);
  Featherstone::RF_abd(qdd, tree, qd, tau);
}*/

/// [prelim] some heuristic measure for the joint errors
double ors::KinematicWorld::getJointErrors() const {
  double err=0.0;
  for(Joint * e: joints) err+=e->Q.pos.lengthSqr();
  return ::sqrt(err);
}

/** @brief checks if all names of the bodies are disjoint */
bool ors::KinematicWorld::checkUniqueNames() const {
  for(Body *  n:  bodies) for(Body *b: bodies) {
    if(n==b) break;
    if(n->name==b->name) return false;
  }
  return true;
}

/** @brief checks if all names of the bodies are disjoint */
void ors::KinematicWorld::setShapeNames() {
  for(Body *b: bodies){
    uint i=0;
    for(Shape *s:b->shapes){
      if(!s->name.N){ s->name = b->name; s->name <<'_' <<i; }
      i++;
    }
  }
}

/// find body with specific name
ors::Body* ors::KinematicWorld::getBodyByName(const char* name) const {
  for(Body *b: bodies) if(b->name==name) return b;
  if(strcmp("glCamera", name)!=0)
  MLR_MSG("cannot find Body named '" <<name <<"' in Graph");
  return 0;
}

/// find shape with specific name
ors::Shape* ors::KinematicWorld::getShapeByName(const char* name) const {
  for(Shape *s: shapes) if(s->name==name) return s;
  MLR_MSG("cannot find Shape named '" <<name <<"' in Graph");
  return NULL;
}

/// find shape with specific name
ors::Joint* ors::KinematicWorld::getJointByName(const char* name, bool verbose) const {
  for(Joint *j: joints) if(j->name==name) return j;
  if(verbose) MLR_MSG("cannot find Joint named '" <<name <<"' in Graph");
  return NULL;
}

/// find joint connecting two bodies
ors::Joint* ors::KinematicWorld::getJointByBodies(const Body* from, const Body* to) const {
  for(Joint *j: to->inLinks) if(j->from==from) return j;
  return NULL;
}

/// find joint connecting two bodies with specific names
ors::Joint* ors::KinematicWorld::getJointByBodyNames(const char* from, const char* to) const {
  Body *f = getBodyByName(from);
  Body *t = getBodyByName(to);
  if(!f || !t) return NULL;
  return graphGetEdge<Body, Joint>(f, t);
}

ShapeL ors::KinematicWorld::getShapesByAgent(const uint agent) const {
  ShapeL agent_shapes;
  for(ors::Joint *j : joints) {
    if(j->agent==agent) {
      ShapeL tmp;
      tmp.append(j->from->shapes);
      tmp.append(j->to->shapes);
      for(ors::Shape* s : tmp) {
        if (!agent_shapes.contains(s)) agent_shapes.append(s);
      }
    } 
  }  
  return agent_shapes;
}

uintA ors::KinematicWorld::getShapeIdxByAgent(const uint agent) const {
  uintA agent_shape_idx;
  ShapeL agent_shapes = getShapesByAgent(agent);
  for(ors::Shape* s : agent_shapes)
    agent_shape_idx.append(s->index);
  return agent_shape_idx;
}

/** @brief creates uniques names by prefixing the node-index-number to each name */
void ors::KinematicWorld::prefixNames() {
  for(Body * n: bodies) n->name=STRING(n->index<< n->name);
}

/// return a OpenGL extension
OpenGL& ors::KinematicWorld::gl(const char* window_title){
  if(!s->gl){
    s->gl = new OpenGL(window_title);
    s->gl->add(glStandardScene, 0);
    s->gl->addDrawer(this);
    s->gl->camera.setDefault();
  }
  return *s->gl;
}

/// return a Swift extension
SwiftInterface& ors::KinematicWorld::swift(){
  if(!s->swift) s->swift = new SwiftInterface(*this);
  return *s->swift;
}

void ors::KinematicWorld::swiftDelete() {
  delete s->swift;
  s->swift = nullptr;
}

/// return a PhysX extension
PhysXInterface& ors::KinematicWorld::physx(){
  if(!s->physx){
    s->physx = new PhysXInterface(*this);
    s->physx->setArticulatedBodiesKinematic();
  }
  return *s->physx;
}

/// return a ODE extension
OdeInterface& ors::KinematicWorld::ode(){
  if(!s->ode) s->ode = new OdeInterface(*this);
  return *s->ode;
}

void ors::KinematicWorld::watch(bool pause, const char* txt){
  if(pause) gl().watch(txt);
  else gl().update(txt);
}

void ors::KinematicWorld::glAnimate(){
  animateConfiguration(*this, NULL);
}

void ors::KinematicWorld::glGetMasks(int w, int h, bool rgbIndices){
  gl().clear();
  gl().addDrawer(this);
  if(rgbIndices){
    gl().setClearColors(0,0,0,0);
    orsDrawIndexColors = true;
    orsDrawMarkers = orsDrawJoints = orsDrawProxies = false;
  }
  gl().renderInBack(true, true, w, h);
//  indexRgb = gl().captureImage;
//  depth = gl().captureDepth;

  gl().clear();
  gl().add(glStandardScene, 0);
  gl().addDrawer(this);
  if(rgbIndices){
    gl().setClearColors(1,1,1,0);
    orsDrawIndexColors = false;
    orsDrawMarkers = orsDrawJoints = orsDrawProxies = true;
  }
}

void ors::KinematicWorld::stepSwift(){
  swift().step(*this, false);
}

void ors::KinematicWorld::stepPhysx(double tau){
  physx().step(tau);
}

void ors::KinematicWorld::stepOde(double tau){
#ifdef MLR_ODE
  ode().setMotorVel(qdot, 100.);
  ode().step(tau);
  ode().importStateFromOde();
#endif
}

void ors::KinematicWorld::stepDynamics(const arr& Bu_control, double tau, double dynamicNoise, bool gravity){

  struct DiffEqn:VectorFunction{
    ors::KinematicWorld &S;
    const arr& Bu;
    bool gravity;
    DiffEqn(ors::KinematicWorld& _S, const arr& _Bu, bool _gravity):S(_S), Bu(_Bu), gravity(_gravity){
      VectorFunction::operator=( [this](arr& y, arr& J, const arr& x) -> void {
        this->fv(y, J, x);
      } );
    }
    void fv(arr& y, arr& J, const arr& x){
      S.setJointState(x[0], x[1]);
      arr M,Minv,F;
      S.equationOfMotion(M, F, gravity);
      //inverse_SymPosDef(Minv, M);
      Minv = inverse(M); //TODO why does symPosDef fail?
      y = Minv * (Bu - F);
    }
  } eqn(*this, Bu_control, gravity);

#if 0
  arr M,Minv,F;
  getDynamics(M, F);
  inverse_SymPosDef(Minv,M);

  //noisy Euler integration (Runge-Kutte4 would be much more precise...)
  qddot = Minv * (u_control - F);
  if(dynamicNoise) rndGauss(qddot, dynamicNoise, true);
  q    += tau * qdot;
  qdot += tau * qddot;
  arr x1=cat(s->q, s->qdot).reshape(2,s->q.N);
#else
  arr x1;
  rk4_2ndOrder(x1, cat(q, qdot).reshape(2,q.N), eqn, tau);
  if(dynamicNoise) rndGauss(x1[1](), ::sqrt(tau)*dynamicNoise, true);
#endif

  setJointState(x1[0], x1[1]);
}

/** @brief prototype for \c operator<< */
void ors::KinematicWorld::write(std::ostream& os) const {
  for(Body *b: bodies) {
    os <<"body " <<b->name <<" { ";
    b->write(os);  os <<" }\n";
  }
  os <<std::endl;
  for(Shape *s: shapes) {
    os <<"shape ";
    if(s->name.N) os <<s->name <<' ';
    os <<"(" <<(s->body?(char*)s->body->name:"") <<"){ ";
    s->write(os);  os <<" }\n";
  }
  os <<std::endl;
  for(Joint *j: joints) {
    os <<"joint ";
    if (j->name.N) os <<j->name <<' ';
    os <<"(" <<j->from->name <<' ' <<j->to->name <<"){ ";
    j->write(os);  os <<" }\n";
  }
}

#define DEBUG(x) //x

/** @brief prototype for \c operator>> */
void ors::KinematicWorld::read(std::istream& is) {
  Graph G(is);
  G.checkConsistency();
  init(G);
//  cout <<"***KVG:\n" <<G <<endl;
}
void ors::KinematicWorld::init(const Graph& G) {
  clear();

  NodeL bs = G.getNodes("body");
  for(Node *  it:  bs) {
    CHECK_EQ(it->keys(0),"body","");
    CHECK(it->isGraph(), "bodies must have value Graph");
    
    Body *b=new Body(*this);
    if(it->keys.N>1) b->name=it->keys(1);
    b->ats.copy(it->graph());
    b->parseAts();
  }

  NodeL ss = G.getNodes("shape");
  for(Node *it: ss) {
    CHECK_EQ(it->keys(0),"shape","");
    CHECK(it->parents.N<=1,"shapes must have no or one parent");
    CHECK(it->isGraph(),"shape must have value Graph");
    
    Shape *s;
    if(it->parents.N==1){
      Body *b = listFindByName(bodies, it->parents(0)->keys(1));
      CHECK(b,"");
      s=new Shape(*this, *b);
    }else{
      s=new Shape(*this, NoBody);
    }
    if(it->keys.N>1) s->name=it->keys(1);
    s->ats.copy(it->graph());
    s->parseAts();
  }
  
  uint nCoupledJoints=0;
  NodeL js = G.getNodes("joint");
  for(Node *it: js) {
    CHECK_EQ(it->keys(0),"joint","");
    CHECK_EQ(it->parents.N,2,"joints must have two parents");
    CHECK(it->isGraph(),"joints must have value Graph");
    
    Body *from=listFindByName(bodies, it->parents(0)->keys(1));
    Body *to=listFindByName(bodies, it->parents(1)->keys(1));
    CHECK(from,"JOINT: from '" <<it->parents(0)->keys(1) <<"' does not exist ["<<*it <<"]");
    CHECK(to,"JOINT: to '" <<it->parents(1)->keys(1) <<"' does not exist ["<<*it <<"]");
    Joint *j=new Joint(*this, from, to);
    if(it->keys.N>1) j->name=it->keys(1);
    j->ats.copy(it->graph());
    j->parseAts();

    //if the joint is coupled to another:
    if(j->mimic) nCoupledJoints++;
  }

  if(nCoupledJoints){
    for(Joint *j: joints) if(j->mimic){
      mlr::String jointName;
      bool good = j->ats.get(jointName, "mimic");
      CHECK(good, "something is wrong");
      if(!jointName.N){ j->mimic=NULL; continue; }
      j->mimic = listFindByName(joints, jointName);
      if(!j->mimic) HALT("The joint '" <<*j <<"' is declared coupled to '" <<jointName <<"' -- but that doesn't exist!");
      j->type = j->mimic->type;
    }
  }

  //-- clean up the graph
  analyzeJointStateDimensions();
  checkConsistency();
  topSort();
  //makeLinkTree();
  calc_missingAB_from_BodyAndJointFrames();
  analyzeJointStateDimensions();
  calc_q_from_Q();
  calc_fwdPropagateFrames();
}

void ors::KinematicWorld::writePlyFile(const char* filename) const {
  ofstream os;
  mlr::open(os, filename);
  uint nT=0,nV=0;
  uint j;
  ors::Mesh *m;
  for(Shape *s: shapes) { nV += s->mesh.V.d0; nT += s->mesh.T.d0; }
  
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
  for(Shape * s: shapes) {
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
  for(Shape *s: shapes) {
    m=&s->mesh;
    for(j=0; j<m->T.d0; j++) {
      os <<"3 " <<offset+m->T(j, 0) <<' ' <<offset+m->T(j, 1) <<' ' <<offset+m->T(j, 2) <<endl;
    }
    offset+=m->V.d0;
  }
}

/// dump the list of current proximities on the screen
void ors::KinematicWorld::reportProxies(std::ostream *os, double belowMargin) {
  (*os) <<"Proximity report: #" <<proxies.N <<endl;
  for_list(Proxy, p, proxies) {
    if(belowMargin>0. && p->d>belowMargin) continue;
    ors::Shape *a = shapes(p->a);
    ors::Shape *b = shapes(p->b);
    (*os)
        <<p_COUNT <<" ("
        <<a->name <<':' <<a->body->name <<")-("
        <<b->name <<':' <<b->body->name
        <<") d=" <<p->d
        <<" |A-B|=" <<(p->posB-p->posA).length()
        <<" cenD=" <<p->cenD
//        <<" d^2=" <<(p->posB-p->posA).lengthSqr()
        <<" v=" <<(p->posB-p->posA)
        <<" normal=" <<p->normal
        <<" posA=" <<p->posA
        <<" posB=" <<p->posB
        <<endl;
  }
}

bool ProxySortComp(const ors::Proxy *a, const ors::Proxy *b) {
  return (a->a < b->a) || (a->a==b->a && a->b<b->b) || (a->a==b->a && a->b==b->b && a->d < b->d);
}

void ors::KinematicWorld::glueBodies(Body *f, Body *t) {
  Joint *j = new Joint(*this, f, t);
  j->A.setDifference(f->X, t->X);
  j->A.vel.setZero();
  j->A.angvel.setZero();
  j->type=JT_rigid;
  j->Q.setZero();
  j->B.setZero();
  isLinkTree=false;
}


/// clear all forces currently stored at bodies
void ors::KinematicWorld::clearForces() {
  for(Body *  n:  bodies) {
    n->force.setZero();
    n->torque.setZero();
  }
}

/// apply a force on body n 
void ors::KinematicWorld::addForce(ors::Vector force, ors::Body *n) {
  n->force += force;
  if (!s->physx) {
    NIY;
  }
  else {
    s->physx->addForce(force, n);
  }
  //n->torque += (pos - n->X.p) ^ force;
}

/// apply a force on body n at position pos (in world coordinates)
void ors::KinematicWorld::addForce(ors::Vector force, ors::Body *n, ors::Vector pos) {
  n->force += force;
  if (!s->physx) {
    NIY;
  }
  else {
    s->physx->addForce(force, n, pos);
  }
  //n->torque += (pos - n->X.p) ^ force;
}

void ors::KinematicWorld::frictionToForces(double coeff) {
  HALT("never do this: add it directly in the equations...");
  ors::Vector a;
  ors::Transformation X;
  double v;
  for(Joint *j:joints) {
    X = j->from->X;
    X.appendTransformation(j->A);
    a = X.rot.getX();//rotation axis
    
    v=j->Q.angvel.length();
    if(j->Q.angvel*Vector_x<0.) v=-v;
    
    j->from->torque -= (coeff*v)*a;
    j->to->torque   += (coeff*v)*a;
  }
}

void ors::KinematicWorld::gravityToForces() {
  ors::Vector g(0, 0, -9.81);
  for(Body *  n:  bodies) n->force += n->mass * g;
}

/// compute forces from the current contacts
void ors::KinematicWorld::contactsToForces(double hook, double damp) {
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

void ors::KinematicWorld::kinematicsProxyDist(arr& y, arr& J, Proxy *p, double margin, bool useCenterDist, bool addValues) const {
  ors::Shape *a = shapes(p->a);
  ors::Shape *b = shapes(p->b);

  y.resize(1);
  if(&J) J.resize(1, getJointStateDimension());
  if(!addValues){ y.setZero();  if(&J) J.setZero(); }

//  //costs
//  if(a->type==ors::sphereST && b->type==ors::sphereST){
//    ors::Vector diff=a->X.pos-b->X.pos;
//    double d = diff.length() - a->size[3] - b->size[3];
//    y(0) = d;
//    if(&J){
//      arr Jpos;
//      arr normal = conv_vec2arr(diff)/diff.length(); normal.reshape(1, 3);
//      kinematicsPos(NoArr, Jpos, a->body);  J += (normal*Jpos);
//      kinematicsPos(NoArr, Jpos, b->body);  J -= (normal*Jpos);
//    }
//    return;
//  }
  y(0) = p->d;
  if(&J){
    arr Jpos;
    ors::Vector arel, brel;
    if(p->d>0.) { //we have a gradient on pos only when outside
      arel=a->X.rot/(p->posA-a->X.pos);
      brel=b->X.rot/(p->posB-b->X.pos);
      CHECK(p->normal.isNormalized(), "proxy normal is not normalized");
      arr normal; normal.referTo(&p->normal.x, 3); normal.reshape(1, 3);
      kinematicsPos(NoArr, Jpos, a->body, arel);  J += (normal*Jpos);
      kinematicsPos(NoArr, Jpos, b->body, brel);  J -= (normal*Jpos);
    }
  }
}

void ors::KinematicWorld::kinematicsProxyCost(arr& y, arr& J, Proxy *p, double margin, bool useCenterDist, bool addValues) const {
  ors::Shape *a = shapes(p->a);
  ors::Shape *b = shapes(p->b);
  CHECK(a->mesh_radius>0.,"");
  CHECK(b->mesh_radius>0.,"");

  y.resize(1);
  if(&J) J.resize(1, getJointStateDimension());
  if(!addValues){ y.setZero();  if(&J) J.setZero(); }

  //costs
  if(a->type==ors::sphereST && b->type==ors::sphereST){
    ors::Vector diff=a->X.pos-b->X.pos;
    double d = diff.length() - a->size[3] - b->size[3];
    y(0) = 1. - d/margin;
    if(&J){
      arr Jpos;
      arr normal = conv_vec2arr(diff)/diff.length(); normal.reshape(1, 3);
      kinematicsPos(NoArr, Jpos, a->body);  J -= 1./margin*(normal*Jpos);
      kinematicsPos(NoArr, Jpos, b->body);  J += 1./margin*(normal*Jpos);
    }
    return;
  }
  double ab_radius = margin + 10.*(a->mesh_radius+b->mesh_radius);
  CHECK(p->d<(1.+1e-6)*margin, "something's really wierd here!");
  CHECK(p->cenD<(1.+1e-6)*ab_radius, "something's really wierd here! You disproved the triangle inequality :-)");
  double d1 = 1.-p->d/margin;
  double d2 = 1.-p->cenD/ab_radius;
  if(d2<0.) d2=0.;
  if(!useCenterDist) d2=1.;
  y(0) += d1*d2;
 
  //Jacobian
  if(&J){
    arr Jpos;
    ors::Vector arel, brel;
    if(p->d>0.) { //we have a gradient on pos only when outside
      arel=a->X.rot/(p->posA-a->X.pos);
      brel=b->X.rot/(p->posB-b->X.pos);
      CHECK(p->normal.isNormalized(), "proxy normal is not normalized");
      arr normal; normal.referTo(&p->normal.x, 3); normal.reshape(1, 3);
          
      kinematicsPos(NoArr, Jpos, a->body, arel);  J -= d2/margin*(normal*Jpos);
      kinematicsPos(NoArr, Jpos, b->body, brel);  J += d2/margin*(normal*Jpos);
    }
        
    if(useCenterDist && d2>0.){
      arel=a->X.rot/(p->cenA-a->X.pos);
      brel=b->X.rot/(p->cenB-b->X.pos);
//      CHECK(p->cenN.isNormalized(), "proxy normal is not normalized");
      if(!p->cenN.isNormalized()){
        MLR_MSG("proxy->cenN is not normalized: objects seem to be at exactly the same place");
      }else{
        arr normal; normal.referTo(&p->cenN.x, 3); normal.reshape(1, 3);
        
        kinematicsPos(NoArr, Jpos, a->body, arel);  J -= d1/ab_radius*(normal*Jpos);
        kinematicsPos(NoArr, Jpos, b->body, brel);  J += d1/ab_radius*(normal*Jpos);
      }
    }
  }
}

/// measure (=scalar kinematics) for the contact cost summed over all bodies
void ors::KinematicWorld::kinematicsProxyCost(arr &y, arr& J, double margin, bool useCenterDist) const {
  y.resize(1).setZero();
  if(&J) J.resize(1, getJointStateDimension()).setZero();
  for(Proxy *p:proxies) if(p->d<margin) {
    kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
  }
}

void ors::KinematicWorld::kinematicsProxyConstraint(arr& g, arr& J, Proxy *p, double margin) const {
  if(&J) J.resize(1, getJointStateDimension()).setZero();

  g.resize(1) = margin - p->d;

  //Jacobian
  if(&J){
    arr Jpos, normal;
    ors::Vector arel,brel;
    ors::Shape *a = shapes(p->a);
    ors::Shape *b = shapes(p->b);
    if(p->d>0.) { //we have a gradient on pos only when outside
      arel=a->X.rot/(p->posA-a->X.pos);
      brel=b->X.rot/(p->posB-b->X.pos);
      CHECK(p->normal.isNormalized(), "proxy normal is not normalized");
      normal.referTo(&p->normal.x, 3);
    } else { //otherwise take gradient w.r.t. centers...
      arel=a->X.rot/(p->cenA-a->X.pos);
      brel=b->X.rot/(p->cenB-b->X.pos);
      CHECK(p->cenN.isNormalized(), "proxy normal is not normalized");
      normal.referTo(&p->cenN.x, 3);
    }
    normal.reshape(1, 3);

    kinematicsPos(NoArr, Jpos, a->body, arel);  J -= (normal*Jpos);
    kinematicsPos(NoArr, Jpos, b->body, brel);  J += (normal*Jpos);
  }
}

void ors::KinematicWorld::kinematicsContactConstraints(arr& y, arr &J) const {
  J.clear();
  ors::Vector normal;
  uint i, con=0;
  Shape *a, *b;
  arr Jpos, dnormal, grad(1, q.N);

  y.clear();
  for(i=0; i<proxies.N; i++) y.append(proxies(i)->d);

  if(!&J) return; //do not return the Jacobian

  ors::Vector arel, brel;
  for(i=0; i<proxies.N; i++) {
    a=shapes(proxies(i)->a); b=shapes(proxies(i)->b);
    
    arel.setZero();  arel=a->X.rot/(proxies(i)->posA-a->X.pos);
    brel.setZero();  brel=b->X.rot/(proxies(i)->posB-b->X.pos);
    
    CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
    dnormal.referTo(proxies(i)->normal.p(), 3); dnormal.reshape(1, 3);
    grad.setZero();
    kinematicsPos(NoArr, Jpos, a->body, arel); grad += dnormal*Jpos; //moving a long normal b->a increases distance
    kinematicsPos(NoArr, Jpos, b->body, brel); grad -= dnormal*Jpos; //moving b long normal b->a decreases distance
    J.append(grad);
    con++;
  }
  J.reshape(con, q.N);
}

void ors::KinematicWorld::kinematicsLimitsCost(arr &y, arr &J, const arr& limits, double margin) const {
  y.resize(1).setZero();
  if(&J) J.resize(1, getJointStateDimension()).setZero();
  double d;
  for(uint i=0; i<limits.d0; i++) if(limits(i,1)>limits(i,0)){ //only consider proper limits (non-zero interval)
    double m = margin*(limits(i,1)-limits(i,0));
    d = limits(i, 0) + m - q(i); //lo
    if(d>0.) {  y(0) += d/m;  if(&J) J(0, i)-=1./m;  }
    d = q(i) - limits(i, 1) + m; //up
    if(d>0.) {  y(0) += d/m;  if(&J) J(0, i)+=1./m;  }
  }
}

/// Compute the new configuration q such that body is located at ytarget (with deplacement rel).
void ors::KinematicWorld::inverseKinematicsPos(Body& body, const arr& ytarget,
                                               const ors::Vector& rel_offset, int max_iter) {
  arr q0, q;
  getJointState(q0);
  q = q0;
  arr y; // endeff pos
  arr J; // Jacobian
  arr invJ;
  arr I = eye(q.N);

  // general inverse kinematic update
  // first iteration: $q* = q' + J^# (y* - y')$
  // next iterations: $q* = q' + J^# (y* - y') + (I - J# J)(q0 - q')$
  for (int i = 0; i < max_iter; i++) {
    kinematicsPos(y, J, &body, rel_offset);
    invJ = ~J * inverse(J * ~J);  // inverse_SymPosDef should work!?
    q = q + invJ * (ytarget - y);

    if (i > 0) {
      q += (I - invJ * J) * (q0 - q);
    }
    setJointState(q);
  }
}

/// center of mass of the whole configuration (3 vector)
double ors::KinematicWorld::getCenterOfMass(arr& x_) const {
  double M=0.;
  ors::Vector x;
  x.setZero();
  for(Body *  n:  bodies) {
    M+=n->mass;
    x+=n->mass*n->X.pos;
  }
  x/=M;
  x_ = conv_vec2arr(x);
  return M;
}

/// gradient (Jacobian) of the COM w.r.t. q (3 x n tensor)
void ors::KinematicWorld::getComGradient(arr &grad) const {
  double M=0.;
  arr J(3, getJointStateDimension());
  grad.resizeAs(J); grad.setZero();
  for(Body * n: bodies) {
    M += n->mass;
    kinematicsPos(NoArr, J, n);
    grad += n->mass * J;
  }
  grad/=M;
}

ors::Proxy* ors::KinematicWorld::getContact(uint a, uint b) const {
  uint i;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<0.) {
      if(proxies(i)->a==(int)a && proxies(i)->b==(int)b) return proxies(i);
      if(proxies(i)->a==(int)b && proxies(i)->b==(int)a) return proxies(i);
    }
  return NULL;
}

arr ors::KinematicWorld::getHmetric() const{
  arr H(getJointStateDimension());
  for(ors::Joint *j:joints){
    double h=j->H;
    CHECK(h>0.,"Hmetric should be larger than 0");
    for(uint k=0;k<j->qDim();k++) H(j->qIndex+k)=h;
  }
  return H;
}

/** @brief */
double ors::KinematicWorld::getEnergy() const {
  double m, v, E;
  ors::Matrix I;
  ors::Vector w;
  
  E=0.;
  for(Body * n: bodies) {
    m=n->mass;
    ors::Quaternion &rot = n->X.rot;
    I=(rot).getMatrix() * n->inertia * (-rot).getMatrix();
    v=n->X.vel.length();
    w=n->X.angvel;
    E += .5*m*v*v;
    E += 9.81 * m * (n->X*n->com).z;
    E += .5*(w*(I*w));
  }
  
  return E;
}

void ors::KinematicWorld::removeUselessBodies(int verbose) {
  //-- remove bodies and their in-joints
  for_list_rev(Body, b, bodies) if(!b->shapes.N && !b->outLinks.N) {
    if(verbose>0) LOG(0) <<" -- removing useless body " <<b->name <<endl;
    delete b;
  }
  //-- reindex
  listReindex(bodies);
  listReindex(joints);
  checkConsistency();
//  for(Joint * j: joints) j->index=j_COUNT;  j->ifrom = j->from->index;  j->ito = j->to->index;  }
//  for(Shape *s: shapes) s->ibody = s->body->index;
  //-- clear all previous index related things
  qdim.clear();
  proxies.clear();
  analyzeJointStateDimensions();
  calc_q_from_Q();
}

bool ors::KinematicWorld::checkConsistency(){
  if(qdim.N){
    uint N=getJointStateDimension();
    CHECK_EQ(N, qdim(q_agent), "");
    if(q.N) CHECK_EQ(N, q.N, "");
    if(qdot.N) CHECK_EQ(N, qdot.N, "");

    uintA myqdim(qdim.N);
    myqdim.setZero();
    for(Joint *j: joints){
      CHECK(j->agent<qdim.N, "");

      if(j->mimic){
        CHECK_EQ(j->qIndex, j->mimic->qIndex, "");
      }else{
        CHECK_EQ(j->qIndex, myqdim(j->agent), "joint indexing is inconsistent");
        myqdim(j->agent) += j->qDim();
      }
    }
    CHECK_EQ(myqdim, qdim, "qdim is wrong");
  }

  for(Body *b: bodies){
    CHECK(&b->world, "");
    CHECK(&b->world==this,"");
    CHECK_EQ(b,bodies(b->index),"");
    for(Joint *j: b->outLinks) CHECK_EQ(j->from,b,"");
    for(Joint *j: b->inLinks)  CHECK_EQ(j->to,b,"");
    for(Shape *s: b->shapes)   CHECK_EQ(s->body,b,"");
    b->ats.checkConsistency();
  }
  for(Joint *j: joints){
    CHECK(&j->world && j->from && j->to, "");
    CHECK(&j->world==this,"");
    CHECK_EQ(j,joints(j->index),"");
    CHECK(j->from->outLinks.findValue(j)>=0,"");
    CHECK(j->to->inLinks.findValue(j)>=0,"");
    j->ats.checkConsistency();
  }
  for(Shape *s: shapes){
    CHECK(&s->world, "");
    CHECK(&s->world==this,"");
    CHECK_EQ(s,shapes(s->index),"");
    if(s->body) CHECK(s->body->shapes.findValue(s)>=0,"");
    s->ats.checkConsistency();
  }
  return true;
}

void ors::KinematicWorld::meldFixedJoints(int verbose) {
  checkConsistency();
  for(Joint *j: joints) if(j->type==JT_rigid) {
    if(verbose>0) LOG(0) <<" -- melding fixed joint " <<j->name <<" (" <<j->from->name <<' ' <<j->to->name <<" )" <<endl;
    Body *a = j->from;
    Body *b = j->to;
    Transformation bridge = j->A * j->Q * j->B;
    //reassociate shapes with a
    for(Shape *s: b->shapes) {
      s->body=a;
      s->rel = bridge * s->rel;
      a->shapes.append(s);
    }
    b->shapes.clear();
    //joints from b-to-c now become joints a-to-c
    for(Joint *jj: b->outLinks) {
      jj->from=a;
      jj->A = bridge * jj->A;
      a->outLinks.append(jj);
    }
    b->outLinks.clear();
    //reassociate mass
    a->mass += b->mass;
    a->inertia += b->inertia;
    b->mass = 0.;
  }
  qdim.clear();
  proxies.clear();
  analyzeJointStateDimensions();
  calc_q_from_Q();
  checkConsistency();
  //-- remove fixed joints and reindex
  for_list_rev(Joint, jj, joints) if(jj->type==JT_rigid) delete jj;
  listReindex(joints);
  //for(Joint * j: joints) { j->index=j_COUNT;  j->ifrom = j->from->index;  j->ito = j->to->index;  }
  checkConsistency();
}

//===========================================================================

ors::KinematicSwitch::KinematicSwitch()
  : symbol(none), jointType(JT_none), timeOfApplication(UINT_MAX), fromId(UINT_MAX), toId(UINT_MAX){
  jA.setZero();
  jB.setZero();
}

void ors::KinematicSwitch::apply(KinematicWorld& G){
  Shape *from=G.shapes(fromId), *to=G.shapes(toId);
  if(symbol==deleteJoint){
    Joint *j = G.getJointByBodies(from->body, to->body);
    CHECK(j,"can't find joint between '"<<from->name <<"--" <<to->name <<"' Deleted before?");
    delete j;
    return;
  }
  if(symbol==addJointZero){
    Joint *j = new Joint(G, from->body, to->body);
    j->type=jointType;
    j->constrainToZeroVel=true;
    j->A = from->rel * jA;
    j->B = jB * (-to->rel);
    G.isLinkTree=false;
    G.calc_fwdPropagateFrames();
    return;
  }
  if(symbol==addJointAtFrom){
    Joint *j = new Joint(G, from->body, to->body);
    j->type=jointType;
    j->constrainToZeroVel=true;
    j->B.setDifference(from->body->X, to->body->X);
    j->A.setZero();
    G.isLinkTree=false;
    return;
  }
  if(symbol==addJointAtTo){
    Joint *j = new Joint(G, from->body, to->body);
    j->type=jointType;
    j->A.setDifference(from->body->X, to->body->X);
    j->B.setZero();
    G.isLinkTree=false;
    return;
  }
  HALT("shouldn't be here!");
}

void ors::KinematicSwitch::temporallyAlign(const ors::KinematicWorld& Gprevious, ors::KinematicWorld& G, bool copyFromBodies){
  if(symbol==addJointAtFrom){
    Joint *j = G.getJointByBodies(G.shapes(fromId)->body, G.shapes(toId)->body);
    if(!j/* || j->type!=jointType*/) HALT("");
    if(copyFromBodies){
      j->B.setDifference(Gprevious.shapes(fromId)->body->X, Gprevious.shapes(toId)->body->X);
    }else{//copy from previous, if exists
      Joint *jprev = Gprevious.getJointByBodies(Gprevious.shapes(fromId)->body, Gprevious.shapes(toId)->body);
      if(!jprev || jprev->type!=j->type){//still copy from bodies
        j->B.setDifference(Gprevious.shapes(fromId)->body->X, Gprevious.shapes(toId)->body->X);
      }else{
        j->B = jprev->B;
      }
    }
//    j->A.setZero();
    G.calc_fwdPropagateFrames();
    return;
  }
  if(symbol==addJointAtTo){
    Joint *j = G.getJointByBodies(G.shapes(fromId)->body, G.shapes(toId)->body);
    if(!j || j->type!=jointType) HALT(""); //return;
    if(copyFromBodies){
      j->A.setDifference(Gprevious.shapes(fromId)->body->X, Gprevious.shapes(toId)->body->X);
    }else{
      Joint *jprev = Gprevious.getJointByBodies(Gprevious.shapes(fromId)->body, Gprevious.shapes(toId)->body);
      if(!jprev || jprev->type!=j->type){
        j->A.setDifference(Gprevious.shapes(fromId)->body->X, Gprevious.shapes(toId)->body->X);
      }else{
        j->A = jprev->A;
      }
    }
//    j->B.setZero();
    G.calc_fwdPropagateFrames();
    return;
  }
}

void ors::KinematicSwitch::write(std::ostream& os) const{
  os <<"  symbol=" <<symbol <<endl;
  os <<"  jointType=" <<jointType <<endl;
  os <<"  timeOfApplication=" <<timeOfApplication <<endl;
  os <<"  fromId=" <<fromId <<endl;
  os <<"  toId=" <<toId <<endl;
}

//===========================================================================

ors::KinematicSwitch* ors::KinematicSwitch::newSwitch(const Node *specs, const ors::KinematicWorld& world, uint Tinterval, uint Tzero){
  if(specs->parents.N<2) return NULL;

  //-- get tags
  mlr::String& tt=specs->parents(0)->keys.last();
  mlr::String& type=specs->parents(1)->keys.last();
  const char *ref1=NULL, *ref2=NULL;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;

  if(tt!="MakeJoint") return NULL;

  //-- create switch
  ors::KinematicSwitch *sw= new ors::KinematicSwitch();
  if(type=="addRigid"){ sw->symbol=ors::KinematicSwitch::addJointZero; sw->jointType=ors::JT_rigid; }
//  else if(type=="addRigidRel"){ sw->symbol = ors::KinematicSwitch::addJointAtTo; sw->jointType=ors::JT_rigid; }
  else if(type=="rigidAtTo"){ sw->symbol = ors::KinematicSwitch::addJointAtTo; sw->jointType=ors::JT_rigid; }
  else if(type=="rigidAtFrom"){ sw->symbol = ors::KinematicSwitch::addJointAtFrom; sw->jointType=ors::JT_rigid; }
  else if(type=="rigidZero"){ sw->symbol = ors::KinematicSwitch::addJointZero; sw->jointType=ors::JT_rigid; }
  else if(type=="transXYPhiAtFrom"){ sw->symbol = ors::KinematicSwitch::addJointAtFrom; sw->jointType=ors::JT_transXYPhi; }
  else if(type=="transXYPhiZero"){ sw->symbol = ors::KinematicSwitch::addJointZero; sw->jointType=ors::JT_transXYPhi; }
  else if(type=="freeAtTo"){ sw->symbol = ors::KinematicSwitch::addJointAtTo; sw->jointType=ors::JT_free; }
  else if(type=="delete"){ sw->symbol = ors::KinematicSwitch::deleteJoint; }
  else HALT("unknown type: "<< type);
  sw->fromId = world.getShapeByName(ref1)->index;
  if(!ref2){
    CHECK_EQ(sw->symbol, ors::KinematicSwitch::deleteJoint, "");
    ors::Body *b = world.shapes(sw->fromId)->body;
    if(b->inLinks.N==1){
//      CHECK_EQ(b->outLinks.N, 0, "");
      sw->toId = sw->fromId;
      sw->fromId = b->inLinks(0)->from->shapes.first()->index;
    }else if(b->outLinks.N==1){
      CHECK_EQ(b->inLinks.N, 0, "");
      sw->toId = b->outLinks(0)->from->shapes.first()->index;
    }else if(b->inLinks.N==0 && b->outLinks.N==0){
      MLR_MSG("No link to delete for shape '" <<ref1 <<"'");
      delete sw;
      return NULL;
    }else HALT("that's ambiguous");
  }else{
    sw->toId = world.getShapeByName(ref2)->index;
  }
  sw->timeOfApplication = Tzero + Tinterval + 1;
  if(specs->isGraph()){
    const Graph& params = specs->graph();
    sw->timeOfApplication = Tzero + params.get<double>("time",1.)*Tinterval + 1;
    params.get(sw->jA, "from");
    params.get(sw->jB, "to");
  }
  return sw;
}


//===========================================================================
//
// helper routines -- in a classical C interface
//

#endif

#undef LEN

double forceClosureFromProxies(ors::KinematicWorld& ORS, uint bodyIndex, double distanceThreshold, double mu, double torqueWeights) {
  ors::Vector c, cn;
  arr C, Cn;
  for(ors::Proxy * p: ORS.proxies){
    int body_a = ORS.shapes(p->a)->body?ORS.shapes(p->a)->body->index:-1;
    int body_b = ORS.shapes(p->b)->body?ORS.shapes(p->b)->body->index:-1;
    if(p->d<distanceThreshold && (body_a==(int)bodyIndex || body_b==(int)bodyIndex)) {
      if(body_a==(int)bodyIndex) {
        c = p->posA;
        cn=-p->normal;
      } else {
        c = p->posB;
        cn= p->normal;
      }
      C.append(conv_vec2arr(c));
      Cn.append(conv_vec2arr(cn));
    }
  }
  C .reshape(C.N/3, 3);
  Cn.reshape(C.N/3, 3);
  double fc=forceClosure(C, Cn, ORS.bodies(bodyIndex)->X.pos, mu, torqueWeights, NULL);
  return fc;
}

void transferQbetweenTwoWorlds(arr& qto, const arr& qfrom, const ors::KinematicWorld& to, const ors::KinematicWorld& from){
  arr q = to.getJointState();
  uint T = qfrom.d0;
  uint Nfrom = qfrom.d1;

  if (qfrom.d1==0) {T = 1; Nfrom = qfrom.d0;}

  qto = repmat(~q,T,1);

  intA match(Nfrom);
  match = -1;
  for(ors::Joint* jfrom:from.joints){
    ors::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) if(match(i)!=-1){
    for(uint t=0;t<T;t++){
      if (qfrom.d1==0) {
        qto(t, match(i)) = qfrom(i);
      } else {
        qto(t, match(i)) = qfrom(t,i);
      }
    }
  }

  if (qfrom.d1==0) qto.reshape(qto.N);
}


void transferQDotbetweenTwoWorlds(arr& qDotTo, const arr& qDotFrom, const ors::KinematicWorld& to, const ors::KinematicWorld& from){
  //TODO: for saveness reasons, the velocities are zeroed.
  arr qDot;
  qDot = zeros(to.getJointStateDimension());
  uint T, dim;
  if(qDotFrom.d1 > 0) {
    T = qDotFrom.d0;
    qDotTo = repmat(~qDot,T,1);
    dim = qDotFrom.d1;
  } else {
    T = 1;
    qDotTo = qDot;
    dim = qDotFrom.d0;
  }

  intA match(dim);
  match = -1;
  for(ors::Joint* jfrom:from.joints){
    ors::Joint* jto = to.getJointByName(jfrom->name, false); //OLD: to.getJointByBodyNames(jfrom->from->name, jfrom->to->name); why???
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }
  if(qDotFrom.d1 > 0) {
    for(uint i=0;i<match.N;i++) if(match(i)!=-1){
      for(uint t=0;t<T;t++){
        qDotTo(t, match(i)) = qDotFrom(t,i);
      }
    }
  } else {
    for(uint i=0;i<match.N;i++) if(match(i)!=-1){
      qDotTo(match(i)) = qDotFrom(i);
    }
  }

}

void transferKpBetweenTwoWorlds(arr& KpTo, const arr& KpFrom, const ors::KinematicWorld& to, const ors::KinematicWorld& from){
  KpTo = zeros(to.getJointStateDimension(),to.getJointStateDimension());
  //use Kp gains from ors file for toWorld, if there are no entries of this joint in fromWorld
  for_list(ors::Joint, j, to.joints) {
    if(j->qDim()>0) {
      arr *info;
      info = j->ats.getValue<arr>("gains");
      if(info) {
        KpTo(j->qIndex,j->qIndex)=info->elem(0);
      }
    }
  }

  intA match(KpFrom.d0);
  match = -1;
  for(ors::Joint* jfrom : from.joints){
    ors::Joint* jto = to.getJointByName(jfrom->name, false); // OLD: ors::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) {
    for(uint j=0;j<match.N;j++){
      KpTo(match(i), match(j)) = KpFrom(i,j);
    }
  }
}

void transferKdBetweenTwoWorlds(arr& KdTo, const arr& KdFrom, const ors::KinematicWorld& to, const ors::KinematicWorld& from) {
  KdTo = zeros(to.getJointStateDimension(),to.getJointStateDimension());

  //use Kd gains from ors file for toWorld, if there are no entries of this joint in fromWorld
  for_list(ors::Joint, j, to.joints) {
    if(j->qDim()>0) {
      arr *info;
      info = j->ats.getValue<arr>("gains");
      if(info) {
        KdTo(j->qIndex,j->qIndex)=info->elem(1);
      }
    }
  }

  intA match(KdFrom.d0);
  match = -1;
  for(ors::Joint* jfrom : from.joints){
    ors::Joint* jto = to.getJointByName(jfrom->name, false); // OLD: ors::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) {
    for(uint j=0;j<match.N;j++){
      KdTo(match(i), match(j)) = KdFrom(i,j);
    }
  }
}


void transferU0BetweenTwoWorlds(arr& u0To, const arr& u0From, const ors::KinematicWorld& to, const ors::KinematicWorld& from){
  u0To = zeros(to.getJointStateDimension());

  intA match(u0From.d0);
  match = -1;
  for(ors::Joint* jfrom : from.joints){
    ors::Joint* jto = to.getJointByName(jfrom->name, false); // OLD: ors::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) {
    u0To(match(i)) = u0From(i);
  }
}


void transferKI_ft_BetweenTwoWorlds(arr& KI_ft_To, const arr& KI_ft_From, const ors::KinematicWorld& to, const ors::KinematicWorld& from){
  uint numberOfColumns = KI_ft_From.d1;
  if(KI_ft_From.d1 == 0) {
    numberOfColumns = 1;
    KI_ft_To = zeros(to.getJointStateDimension());
  } else {
    KI_ft_To = zeros(to.getJointStateDimension(), KI_ft_From.d1);
  }

  intA match(KI_ft_From.d0);
  match = -1;
  for(ors::Joint* jfrom : from.joints){
    ors::Joint* jto = to.getJointByName(jfrom->name, false); // OLD: ors::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) {
    for(uint j=0;j < numberOfColumns;j++){
      if(numberOfColumns > 1) {
        KI_ft_To(match(i), j) = KI_ft_From(i,j);
      } else {
        KI_ft_To(match(i)) = KI_ft_From(i);
      }
    }
  }
}

//===========================================================================
//-- template instantiations

#include <Core/util.tpp>

#ifndef  MLR_ORS_ONLY_BASICS
template mlr::Array<ors::Shape*>::Array(uint);
template ors::Shape* listFindByName(const mlr::Array<ors::Shape*>&,const char*);

#include <Core/array.tpp>
template mlr::Array<ors::Joint*>::Array();
#endif
/** @} */
