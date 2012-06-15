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
#include <PxTkStream.h>

#include "ors_physx.h"
#include "opengl.h"

using namespace physx;

static PxFoundation* mFoundation = NULL;
static PxPhysics* mPhysics = NULL;
static PxCooking* mCooking = NULL;
static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;

void PxTrans2OrsTrans(ors::Transformation& f, const PxTransform& pose){
  f.pos.set(pose.p.x, pose.p.y, pose.p.z);
  f.rot.set(pose.q.w, pose.q.x, pose.q.y, pose.q.z);
}

PxTransform OrsTrans2PxTrans(const ors::Transformation& f){
  return PxTransform(PxVec3(f.pos(0), f.pos(1), f.pos(2)), PxQuat(f.rot.p[1], f.rot.p[2], f.rot.p[3], f.rot.p[0]));
}

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
    mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale());
    PxCookingParams cookParams; cookParams.skinWidth = .001f;
    mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, cookParams);
    if(!mCooking) HALT("PxCreateCooking failed!");
    if(!mPhysics) HALT("Error creating PhysX3 device.");
      
    if(!PxInitExtensions(*mPhysics))
      HALT("PxInitExtensions failed!");
  }
  
  //PxExtensionVisualDebugger::connect(mPhysics->getPvdConnectionManager(),"localhost",5425, 10000, true);
  
  //-- Create the scene
  PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
  sceneDesc.gravity=PxVec3(0.f, 0.f, -9.8f);
  
  if(!sceneDesc.cpuDispatcher) {
    PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    if(!mCpuDispatcher)
      cerr<<"PxDefaultCpuDispatcherCreate failed!"<<endl;
    sceneDesc.cpuDispatcher = mCpuDispatcher;
  }
  if(!sceneDesc.filterShader)
    sceneDesc.filterShader  = gDefaultFilterShader;
    
  s->gScene = mPhysics->createScene(sceneDesc);
  if(!s->gScene)
    cerr<<"createScene failed!"<<endl;
    
  s->gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0);
  s->gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
  
  
  //-- Create objects
  PxMaterial* mMaterial = mPhysics->createMaterial(0.5,0.5,0.5);
  
  //Create ground plane
  //PxReal d = 0.0f;
  PxTransform pose = PxTransform(PxVec3(0.f, 0.f, 0.f),PxQuat(-PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f)));
  
  PxRigidStatic* plane = mPhysics->createRigidStatic(pose);
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
      PxGeometry *geometry;
      floatA Vfloat;
      switch(s->type){
	case ors::boxST:{
	  geometry = new PxBoxGeometry(.5*PxVec3(s->size[0], s->size[1], s->size[2]));
	}  break;
	case ors::sphereST:{
	  geometry = new PxSphereGeometry(s->size[3]);
	}  break;
	case ors::meshST:{
	  //PxSimpleTriangleMesh *me = new PxSimpleTriangleMesh();
	  //s->mesh.fuseNearVertices(1e-4);
	  //s->mesh.makeConvexHull();
	  copy(Vfloat,s->mesh.V); //convert vertices to float array..
	  //PxTriangleMesh* triangleMesh = PxToolkit::createTriangleMesh32(*mPhysics, *mCooking, (PxVec3*)Vfloat.p, s->mesh.V.d0, s->mesh.T.p, s->mesh.T.d0);
	  PxConvexMesh* triangleMesh = PxToolkit::createConvexMesh(*mPhysics, *mCooking, (PxVec3*)Vfloat.p, s->mesh.V.d0, PxConvexFlag::eCOMPUTE_CONVEX|PxConvexFlag::eINFLATE_CONVEX);
	  geometry = new PxConvexMeshGeometry(triangleMesh);
	}  break;
      }
      PxRigidDynamic *actor = PxCreateDynamic(*mPhysics, OrsTrans2PxTrans(s->X), *geometry, *mMaterial, 1.f);
      CHECK(actor, "create actor failed!");
      actor->setAngularDamping(0.75);
      actor->setLinearVelocity(PxVec3(b->X.vel(0), b->X.vel(1), b->X.vel(2)));
      actor->setAngularVelocity(PxVec3(b->X.angvel(0), b->X.angvel(1), b->X.angvel(2)));
      this->s->gScene->addActor(*actor);
      
      this->s->objects.append(actor);
      //WARNING: objects must be aligned (indexed) exactly as G->bodies
    }
  }
}

void PhysXInterface::pullState() {
  uint i;
  for_index(i,s->objects) {
    PxTrans2OrsTrans(G->bodies(i)->X, s->objects(i)->getGlobalPose());
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
  mPhysics->release();
}

void DrawActor(PxRigidActor* actor,ors::Body *body) {
  PxU32 nShapes = actor->getNbShapes();
  PxShape** shapes=new PxShape*[nShapes];
  
  actor->getShapes(shapes, nShapes);
  while(nShapes--) {
    PxShape *shape = shapes[nShapes];
    ors::Transformation f;
    double mat[16];
    PxTrans2OrsTrans(f, PxShapeExt::getGlobalPose(*shape));
    glLoadMatrixd(f.getAffineMatrixGL(mat));
    switch(shape->getGeometryType()){
      case PxGeometryType::eBOX:{
	PxBoxGeometry g;
	shape->getBoxGeometry(g);
	glutSolidCube(g.halfExtents.x*2);
      } break;
      case PxGeometryType::eSPHERE:{
	PxSphereGeometry g;
	shape->getSphereGeometry(g);
	glutSolidSphere(g.radius, 10, 10);
      } break;
      case PxGeometryType::eCONVEXMESH:{
#if 1
        PxConvexMeshGeometry g;
	shape->getConvexMeshGeometry(g);
	ors::Mesh mesh;
	floatA Vfloat((float*)g.convexMesh->getVertices(),3*g.convexMesh->getNbVertices());
	copy(mesh.V,Vfloat);
	mesh.V.reshape(g.convexMesh->getNbVertices(),3);
	mesh.makeConvexHull();
	mesh.glDraw();
#else
        body->shapes(nShapes)->mesh.glDraw();
#endif
      } break;
      
      default:
	MT_MSG("can't draw this type");
    }
  }
  delete [] shapes;
}

void PhysXInterface::glDraw() {
  uint i;
  for_index(i,s->objects)  DrawActor(s->objects(i), G->bodies(i));
}

void glPhysXInterface(void *classP) {
  PhysXInterface *phys = (PhysXInterface*)classP;
  phys->glDraw();
}


///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

//stuff from Samples/PxToolkit

using namespace PxToolkit;

PxConvexMesh* PxToolkit::createConvexMesh(PxPhysics& physics, PxCooking& cooking, const PxVec3* verts, PxU32 vertCount, PxConvexFlags flags)
{
	PxConvexMeshDesc convexDesc;
	convexDesc.points.count			= vertCount;
	convexDesc.points.stride		= sizeof(PxVec3);
	convexDesc.points.data			= verts;
	convexDesc.flags				= flags;

	MemoryOutputStream buf;
	if(!cooking.cookConvexMesh(convexDesc, buf))
		return NULL;

	PxToolkit::MemoryInputData input(buf.getData(), buf.getSize());
	return physics.createConvexMesh(input);
}

PxTriangleMesh* PxToolkit::createTriangleMesh32(PxPhysics& physics, PxCooking& cooking, const PxVec3* verts, PxU32 vertCount, const PxU32* indices32, PxU32 triCount)
{
	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count			= vertCount;
	meshDesc.points.stride			= 3*sizeof(float);
	meshDesc.points.data			= verts;

	meshDesc.triangles.count		= triCount;
	meshDesc.triangles.stride		= 3*sizeof(uint);
	meshDesc.triangles.data			= indices32;

	PxToolkit::MemoryOutputStream writeBuffer;
	bool status = cooking.cookTriangleMesh(meshDesc, writeBuffer);
	if(!status)
		return NULL;

	PxToolkit::MemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
	return physics.createTriangleMesh(readBuffer);
}

MemoryOutputStream::MemoryOutputStream() :
	mData		(NULL),
	mSize		(0),
	mCapacity	(0)
{
}

MemoryOutputStream::~MemoryOutputStream()
{
	if(mData)
		delete[] mData;
}

PxU32 MemoryOutputStream::write(const void* src, PxU32 size)
{
	PxU32 expectedSize = mSize + size;
	if(expectedSize > mCapacity)
	{
		mCapacity = expectedSize + 4096;

		PxU8* newData = new PxU8[mCapacity];
		PX_ASSERT(newData!=NULL);

		if(newData)
		{
			memcpy(newData, mData, mSize);
			delete[] mData;
		}
		mData = newData;
	}
	memcpy(mData+mSize, src, size);
	mSize += size;
	return size;
}

///////////////////////////////////////////////////////////////////////////////

MemoryInputData::MemoryInputData(PxU8* data, PxU32 length) :
	mSize	(length),
	mData	(data),
	mPos	(0)
{
}

PxU32 MemoryInputData::read(void* dest, PxU32 count)
{
	PxU32 length = PxMin<PxU32>(count, mSize-mPos);
	memcpy(dest, mData+mPos, length);
	mPos += length;
	return length;
}

PxU32 MemoryInputData::getLength() const
{
	return mSize;
}

void MemoryInputData::seek(PxU32 offset)
{
	mPos = PxMin<PxU32>(mSize, offset);
}

PxU32 MemoryInputData::tell() const
{
	return mPos;
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

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


