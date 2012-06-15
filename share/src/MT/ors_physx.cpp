#ifdef MT_PHYSX

#include <PxPhysicsAPI.h>
#include <PxExtensionsAPI.h>
#include <PxDefaultErrorCallback.h>
#include <PxDefaultAllocator.h>
#include <PxDefaultSimulationFilterShader.h>
#include <PxDefaultCpuDispatcher.h>
#include <PxShapeExt.h>
#include <PxMat33.h>
//#include <PxMat33Legacy.h>
#include <PxSimpleFactory.h>

#include "ors_physx.h"
#include "opengl.h"

using namespace physx;

static PxFoundation* mFoundation = NULL;
static PxPhysics* gPhysicsSDK = NULL;
static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;

struct sPhysXInterface {
  PxScene* gScene;
  PxReal timestep;
  MT::Array<PxRigidActor*> objects;
  
  sPhysXInterface() {
    gScene = NULL;
    timestep = 1.0f/60.0f;
  }
};

PhysXInterface::PhysXInterface() {
  s = new sPhysXInterface;
}

PhysXInterface::~PhysXInterface() {
  delete s;
}

void PhysXInterface::step() {
  s->gScene->simulate(s->timestep);
  pullState();
  
  //...perform useful work here using previous frame's state data
  while(!s->gScene->fetchResults()) {
    // do something useful
  }
}

void PhysXInterface::create() {
  CHECK(G,"");
  if(!mFoundation) {
    mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
    gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale());
    if(!gPhysicsSDK)
      HALT("Error creating PhysX3 device.");
      
    if(!PxInitExtensions(*gPhysicsSDK))
      HALT("PxInitExtensions failed!");
  }
  
  //PxExtensionVisualDebugger::connect(gPhysicsSDK->getPvdConnectionManager(),"localhost",5425, 10000, true);
  
  //-- Create the scene
  PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());
  sceneDesc.gravity=PxVec3(0.f, 0.f, -9.8f);
  
  if(!sceneDesc.cpuDispatcher) {
    PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    if(!mCpuDispatcher)
      cerr<<"PxDefaultCpuDispatcherCreate failed!"<<endl;
    sceneDesc.cpuDispatcher = mCpuDispatcher;
  }
  if(!sceneDesc.filterShader)
    sceneDesc.filterShader  = gDefaultFilterShader;
    
  s->gScene = gPhysicsSDK->createScene(sceneDesc);
  if(!s->gScene)
    cerr<<"createScene failed!"<<endl;
    
  s->gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0);
  s->gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
  
  
  //-- Create objects
  PxMaterial* mMaterial = gPhysicsSDK->createMaterial(0.5,0.5,0.5);
  
  //Create ground plane
  //PxReal d = 0.0f;
  PxTransform pose = PxTransform(PxVec3(0.f, 0.f, 0.f),PxQuat(-PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f)));
  
  PxRigidStatic* plane = gPhysicsSDK->createRigidStatic(pose);
  CHECK(plane, "create plane failed!");
  
  PxShape* planeShape = plane->createShape(PxPlaneGeometry(), *mMaterial);
  CHECK(planeShape, "create shape failed!");
  s->gScene->addActor(*plane);
  
  //loop through ors
  uint i,j;
  ors::Body *b;
  ors::Shape *s;
  
  for_list(i,b,G->bodies) {
    CHECK(b->shapes.N==1,"can't handle multiple shapes yet");
    for_list(j,s,b->shapes) {
      //2) Create cube
      PxTransform transform(PxVec3(s->X.pos(0), s->X.pos(1)+1., s->X.pos(2)+1.), PxQuat(s->X.rot.p[1], s->X.rot.p[2], s->X.rot.p[3], s->X.rot.p[0]));
      PxBoxGeometry geometry(.5*PxVec3(s->size[0], s->size[1], s->size[2]));
      
      PxRigidDynamic *actor = PxCreateDynamic(*gPhysicsSDK, transform, geometry, *mMaterial, 1.f);
      actor->setAngularDamping(0.75);
      actor->setLinearVelocity(PxVec3(b->X.vel(0), b->X.vel(1), b->X.vel(2)));
      CHECK(actor, "create actor failed!");
      this->s->gScene->addActor(*actor);
      
      this->s->objects.append(actor);
      //WARNING: objects must be aligned (indexed) exactly as G->bodies
    }
  }
}

void PhysXInterface::pullState() {
  uint i;
  for_index(i,s->objects) {
    PxTransform pose = s->objects(i)->getGlobalPose();
    ors::Transformation& f=G->bodies(i)->X;
    f.pos.set(pose.p.x, pose.p.y, pose.p.z);
    f.rot.set(pose.q.w, pose.q.x, pose.q.y, pose.q.z);
  }
  G->calcShapeFramesFromBodies();
}

void PhysXInterface::ShutdownPhysX() {
  uint i;
  for_index(i,s->objects) {
    s->gScene->removeActor(*s->objects(i));
    s->objects(i)->release();
  }
  s->gScene->release();
  gPhysicsSDK->release();
}


void getColumnMajor(PxMat33 m, PxVec3 t, float* mat) {
  mat[0] = m.column0[0];
  mat[1] = m.column0[1];
  mat[2] = m.column0[2];
  mat[3] = 0;
  
  mat[4] = m.column1[0];
  mat[5] = m.column1[1];
  mat[6] = m.column1[2];
  mat[7] = 0;
  
  mat[8] = m.column2[0];
  mat[9] = m.column2[1];
  mat[10] = m.column2[2];
  mat[11] = 0;
  
  mat[12] = t[0];
  mat[13] = t[1];
  mat[14] = t[2];
  mat[15] = 1;
}

void DrawBox(PxShape* pShape) {
  PxTransform pT = PxShapeExt::getGlobalPose(*pShape);
  PxBoxGeometry bg;
  pShape->getBoxGeometry(bg);
  PxMat33 m(pT.q);
  float mat[16];
  getColumnMajor(m,pT.p, mat);
  glPushMatrix();
  glMultMatrixf(mat);
  glutSolidCube(bg.halfExtents.x*2);
  glPopMatrix();
}

void DrawShape(PxShape* shape) {
  PxGeometryType::Enum type = shape->getGeometryType();
  switch(type) {
    case PxGeometryType::eBOX:
      DrawBox(shape);
      break;
  }
}

void DrawActor(PxRigidActor* actor) {
  PxU32 nShapes = actor->getNbShapes();
  PxShape** shapes=new PxShape*[nShapes];
  
  actor->getShapes(shapes, nShapes);
  while(nShapes--) {
    DrawShape(shapes[nShapes]);
  }
  delete [] shapes;
}

void PhysXInterface::glDraw() {
  uint i;
  for_index(i,s->objects)  DrawActor(s->objects(i));
}

void glPhysXInterface(void *classP) {
  PhysXInterface *phys = (PhysXInterface*)classP;
  phys->glDraw();
}

#else //MT_PHYSX

#include "ors_physx.h"

PhysXInterface::PhysXInterface(){ NICO }
PhysXInterface::~PhysXInterface(){ NICO }
void PhysXInterface::create(){ NICO }
void PhysXInterface::step(){ NICO }
void PhysXInterface::glDraw(){ NICO }
void PhysXInterface::pushState(){ NICO }
void PhysXInterface::pullState(){ NICO }
void PhysXInterface::ShutdownPhysX(){ NICO }
void glPhysXInterface(void *classP){ NICO }

#endif
