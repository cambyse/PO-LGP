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
 * @ingroup group_ors
 * @{
 */


#ifdef MT_PHYSX

#include <physx/PxPhysicsAPI.h>
#include <physx/extensions/PxExtensionsAPI.h>
#include <physx/extensions/PxDefaultErrorCallback.h>
#include <physx/extensions/PxDefaultAllocator.h>
#include <physx/extensions/PxDefaultSimulationFilterShader.h>
#include <physx/extensions/PxDefaultCpuDispatcher.h>
#include <physx/extensions/PxShapeExt.h>
#include <physx/foundation/PxMat33.h>
//#include <PxMat33Legacy.h>
#include <physx/extensions/PxSimpleFactory.h>
#include <physx/toolkit/PxTkStream.h>

#include "ors_physx.h"
#include <Gui/opengl.h>

using namespace physx;

static PxFoundation* mFoundation = NULL;
static PxPhysics* mPhysics = NULL;
static PxCooking* mCooking = NULL;
static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;


// ============================================================================
/**
 * @brief Connect ors with PhysX and add a cmaera.
 *
 * See bindOrsToOpenGL for a similar function.
 *
 * @param graph the graph PhysX is going to use.
 * @param gl the gl output.
 * @param physx the PhyxXInteface which handles the ors graph.
 */
void bindOrsToPhysX(ors::Graph& graph, OpenGL& gl, PhysXInterface& physx) {
  physx.G = &graph;
  physx.create();
  
  gl.add(glStandardScene, NULL);
  gl.add(glPhysXInterface, &physx);
  gl.setClearColors(1., 1., 1., 1.);
  
  ors::Body* glCamera = graph.getBodyByName("glCamera");
  if(glCamera) {
    *(gl.camera.X) = glCamera->X;
  } else {
    gl.camera.setPosition(10., -15., 8.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();
  }
  gl.update();
}


// ============================================================================
void PxTrans2OrsTrans(ors::Transformation& f, const PxTransform& pose) {
  f.pos.set(pose.p.x, pose.p.y, pose.p.z);
  f.rot.set(pose.q.w, pose.q.x, pose.q.y, pose.q.z);
}

PxTransform OrsTrans2PxTrans(const ors::Transformation& f) {
  return PxTransform(PxVec3(f.pos.x, f.pos.y, f.pos.z), PxQuat(f.rot.x, f.rot.y, f.rot.z, f.rot.w));
}

struct sPhysXInterface {
  PxScene* gScene;
  PxReal timestep;
  MT::Array<PxRigidActor*> actors;
  
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
  //-- push positions of all kinematic objects
  uint i;
  ors::Body *b;
  for_list(i,b,G->bodies) if(b->type==ors::kinematicBT) {
    ((PxRigidDynamic*)s->actors(i))->setKinematicTarget(OrsTrans2PxTrans(b->X));
  }
  
  //-- dynamic simulation
  s->gScene->simulate(s->timestep);
  
  //...perform useful work here using previous frame's state data
  while(!s->gScene->fetchResults()) {
  }
  
  //-- pull state of all objects
  pullState();
  
}

/**
 * @brief Create the PhysX interface which then can be used by OpenGL.
 *
 * - setup some physx stuff
 * - create PhysX equivalent to the ors graph
 */
void PhysXInterface::create() {
  CHECK(G, "");
  if(!mFoundation) {
    mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
    mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale());
    PxCookingParams cookParams;
    cookParams.skinWidth = .001f;
    mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, cookParams);
    if(!mCooking) HALT("PxCreateCooking failed!");
    if(!mPhysics) HALT("Error creating PhysX3 device.");
    
    if(!PxInitExtensions(*mPhysics))
      HALT("PxInitExtensions failed!");
  }
  
  //PxExtensionVisualDebugger::connect(mPhysics->getPvdConnectionManager(),"localhost",5425, 10000, true);
  
  //-- Create the scene
  PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.f, 0.f, -9.8f);
  
  if(!sceneDesc.cpuDispatcher) {
    PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    if(!mCpuDispatcher) {
      cerr << "PxDefaultCpuDispatcherCreate failed!" << endl;
    }
    sceneDesc.cpuDispatcher = mCpuDispatcher;
  }
  if(!sceneDesc.filterShader) {
    sceneDesc.filterShader  = gDefaultFilterShader;
  }
  
  s->gScene = mPhysics->createScene(sceneDesc);
  if(!s->gScene) {
    cerr << "createScene failed!" << endl;
  }
  
  s->gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0);
  s->gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
  
  //-- Create objects
  PxMaterial* mMaterial = mPhysics->createMaterial(1.f, 1.f, 0.5f);
  
  //Create ground plane
  //PxReal d = 0.0f;
  PxTransform pose = PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(-PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f)));
  
  PxRigidStatic* plane = mPhysics->createRigidStatic(pose);
  CHECK(plane, "create plane failed!");
  
  PxShape* planeShape = plane->createShape(PxPlaneGeometry(), *mMaterial);
  CHECK(planeShape, "create shape failed!");
  s->gScene->addActor(*plane);
  // create ORS equivalent in PhysX
  // loop through ors
  uint i, j;
  ors::Body* b;
  ors::Shape* s;
  
  for_list(i, b, G->bodies) {
    PxRigidDynamic* actor;
    switch(b->type) {
      case ors::staticBT:
        actor = (PxRigidDynamic*) mPhysics->createRigidStatic(OrsTrans2PxTrans(b->X));
        break;
      case ors::dynamicBT:
        actor = mPhysics->createRigidDynamic(OrsTrans2PxTrans(b->X));
        break;
      case ors::kinematicBT:
        actor = mPhysics->createRigidDynamic(OrsTrans2PxTrans(b->X));
        actor->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
        break;
      case ors::noneBT:
        actor = mPhysics->createRigidDynamic(OrsTrans2PxTrans(b->X));
        break;
    }
    CHECK(actor, "create actor failed!");
    for_list(j, s, b->shapes) {
      PxGeometry* geometry;
      switch(s->type) {
        case ors::boxST: {
          geometry = new PxBoxGeometry(.5 * s->size[0], .5 * s->size[1], .5 * s->size[2]);
        }
        break;
        case ors::sphereST: {
          geometry = new PxSphereGeometry(s->size[3]);
        }
        break;
        case ors::cappedCylinderST: {
          geometry = new PxCapsuleGeometry(s->size[3], s->size[2]);
        }
        break;
        case ors::cylinderST:
        case ors::meshST: {
          // Note: physx can't decompose meshes itself.
          // Physx doesn't support triangle meshes in dynamic objects! See:
          // file:///home/mtoussai/lib/PhysX/Documentation/PhysXGuide/Manual/Shapes.html
          // We have to decompose the meshes "by hand" and feed them to PhysX.
          
          // PhysX uses float for the vertices
          floatA Vfloat;
          
          // if mesh is only a single convex part
          if(!s->mesh.subMeshSizes.N) {
            Vfloat.clear();
            copy(Vfloat, s->mesh.V); //convert vertices from double to float array..
            PxConvexMesh* triangleMesh = PxToolkit::createConvexMesh(
                                           *mPhysics, *mCooking, (PxVec3*)Vfloat.p, Vfloat.d0,
                                           PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eINFLATE_CONVEX);
            geometry = new PxConvexMeshGeometry(triangleMesh);
          }
          // the mesh consists of several convex sub-parts
          else {
            uint offset = 0;
            for(uint i = 0; i < s->mesh.subMeshSizes.N; i++) {
              Vfloat.resize(s->mesh.subMeshSizes(i), 3);
              for(uint v = 0; v < s->mesh.subMeshSizes(i); v++) {
                Vfloat(v, 0) = s->mesh.V(offset + v, 0);
                Vfloat(v, 1) = s->mesh.V(offset + v, 1);
                Vfloat(v, 2) = s->mesh.V(offset + v, 2);
              }
              offset += s->mesh.subMeshSizes(i);
              
              PxConvexMesh* triangleMesh = PxToolkit::createConvexMesh(
                                             *mPhysics, *mCooking, (PxVec3*)Vfloat.p, Vfloat.d0,
                                             PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eINFLATE_CONVEX);
              geometry = new PxConvexMeshGeometry(triangleMesh);
              PxShape* shape = actor->createShape(*geometry, *mMaterial, OrsTrans2PxTrans(s->rel));
              CHECK(shape, "create shape failed!");
            }
            geometry = NULL;
          }
        }
        break;
        case ors::markerST: {
          geometry = NULL;
        }
        break;
        default:
          NIY;
      }
      if(geometry) {
        PxShape* shape = actor->createShape(*geometry, *mMaterial, OrsTrans2PxTrans(s->rel));
        CHECK(shape, "create shape failed!");
      }
      //actor = PxCreateDynamic(*mPhysics, OrsTrans2PxTrans(s->X), *geometry, *mMaterial, 1.f);
    }
    if(b->ats.getValue<double>("mass")) {
      PxRigidBodyExt::setMassAndUpdateInertia(*actor, *(b->ats.getValue<double>("mass")));
    }
    if (b->type == ors::dynamicBT) {
      if(!b->ats.getValue<double>("mass")) 
        PxRigidBodyExt::updateMassAndInertia(*actor, 1.f);
    }
    if(b->type == ors::dynamicBT) {
      PxRigidBodyExt::updateMassAndInertia(*actor, 1.f);
      actor->setAngularDamping(0.75);
      actor->setLinearVelocity(PxVec3(b->X.vel.x, b->X.vel.y, b->X.vel.z));
      actor->setAngularVelocity(PxVec3(b->X.angvel.x, b->X.angvel.y, b->X.angvel.z));
    }
    this->s->gScene->addActor(*actor);
    
    this->s->actors.append(actor);
    //WARNING: actors must be aligned (indexed) exactly as G->bodies
  }
  
  /// ADD joints here!
  ors::Joint* jj;
  for_list(i, jj, G->joints) {
    PxTransform A = OrsTrans2PxTrans(jj->A);
    PxTransform B = OrsTrans2PxTrans(jj->B);
    switch(jj->type) {
      case ors::JT_hingeX: {
        PxRevoluteJoint* desc;
        //  CHECK(A.p!=B.p,"Something is horribly wrong!");
        desc = PxRevoluteJointCreate(*mPhysics, this->s->actors(jj->ifrom), A, this->s->actors(jj->ito), B.getInverse());
        
        if(jj->ats.getValue<arr>("limit")) {
          arr limits = *(jj->ats.getValue<arr>("limit"));
          PxJointLimitPair limit(limits(0), limits(1), 0.1f);
          limit.restitution = limits(2);
          limit.spring = limits(3);
          limit.damping= limits(4);
          desc->setLimit(limit);
          desc->setRevoluteJointFlag(physx::PxRevoluteJointFlag::eLIMIT_ENABLED, true);
          //desc->setProjectionAngularTolerance(3.14);
        }
        
      }
      break;
      case ors::JT_fixed: {
        // PxFixedJoint* desc =
        PxFixedJointCreate(*mPhysics, this->s->actors(jj->ifrom), A, this->s->actors(jj->ito), B.getInverse());
        // desc->setProjectionLinearTolerance(1e10);
        // desc->setProjectionAngularTolerance(3.14);
      }
      break;
      default:
        NIY;
    }
  }
  /// end of joints
}

void PhysXInterface::pullState() {
  for_index(i, s->actors) PxTrans2OrsTrans(G->bodies(i)->X, s->actors(i)->getGlobalPose());
  G->calcShapeFramesFromBodies();
  G->calcJointsFromBodyFrames();
}

void PhysXInterface::ShutdownPhysX() {
  for_index(i, s->actors) {
    s->gScene->removeActor(*s->actors(i));
    s->actors(i)->release();
  }
  s->gScene->release();
  mPhysics->release();
}

void DrawActor(PxRigidActor* actor, ors::Body *body) {
  PxU32 nShapes = actor->getNbShapes();
  PxShape** shapes=new PxShape*[nShapes];
  //cout <<"#shapes=" <<nShapes;
  
  actor->getShapes(shapes, nShapes);
  while(nShapes--) {
    PxShape *shape = shapes[nShapes];

    // use the color of the first shape of the body for the entire body
    ors::Shape *s = body->shapes(0);
    glColor(s->color[0], s->color[1], s->color[2], .8);

    ors::Transformation f;
    double mat[16];
    PxTrans2OrsTrans(f, PxShapeExt::getGlobalPose(*shape));
    glLoadMatrixd(f.getAffineMatrixGL(mat));
    //cout <<"drawing shape " <<body->name <<endl;
    switch(shape->getGeometryType()) {
      case PxGeometryType::eBOX: {
        PxBoxGeometry g;
        shape->getBoxGeometry(g);
        //glutSolidCube(g.halfExtents.x*2, g.halfExtents.y*2, g.halfExtents.z*2);
        glDrawBox(g.halfExtents.x*2, g.halfExtents.y*2, g.halfExtents.z*2);
      } break;
      case PxGeometryType::eSPHERE: {
        PxSphereGeometry g;
        shape->getSphereGeometry(g);
        glutSolidSphere(g.radius, 10, 10);
      } break;
      case PxGeometryType::eCAPSULE: {
        PxCapsuleGeometry g;
        shape->getCapsuleGeometry(g);
        glDrawCappedCylinder(g.radius, g.halfHeight*2);
      } break;
      case PxGeometryType::eCONVEXMESH: {
#if 1
        PxConvexMeshGeometry g;
        shape->getConvexMeshGeometry(g);
        floatA Vfloat((float*)g.convexMesh->getVertices(), 3*g.convexMesh->getNbVertices()); //reference
        ors::Mesh mesh;
        copy(mesh.V,Vfloat);
        mesh.V.reshape(g.convexMesh->getNbVertices(),3);
        mesh.makeConvexHull();
        mesh.glDraw();
#else
        s->mesh.glDraw();
#endif
      } break;
      
      default:
        MT_MSG("can't draw this type");
    }
  }
  delete [] shapes;
}

void PhysXInterface::glDraw() {
  for_index(i, s->actors)  DrawActor(s->actors(i), G->bodies(i));
}

void glPhysXInterface(void *classP) {
  ((PhysXInterface*)classP)->glDraw();
}


///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

//stuff from Samples/PxToolkit

using namespace PxToolkit;

PxConvexMesh* PxToolkit::createConvexMesh(PxPhysics& physics, PxCooking& cooking, const PxVec3* verts, PxU32 vertCount, PxConvexFlags flags) {
  PxConvexMeshDesc convexDesc;
  convexDesc.points.count     = vertCount;
  convexDesc.points.stride    = sizeof(PxVec3);
  convexDesc.points.data      = verts;
  convexDesc.flags        = flags;
  
  MemoryOutputStream buf;
  if(!cooking.cookConvexMesh(convexDesc, buf))
    return NULL;
    
  PxToolkit::MemoryInputData input(buf.getData(), buf.getSize());
  return physics.createConvexMesh(input);
}

PxTriangleMesh* PxToolkit::createTriangleMesh32(PxPhysics& physics, PxCooking& cooking, const PxVec3* verts, PxU32 vertCount, const PxU32* indices32, PxU32 triCount) {
  PxTriangleMeshDesc meshDesc;
  meshDesc.points.count     = vertCount;
  meshDesc.points.stride      = 3*sizeof(float);
  meshDesc.points.data      = verts;
  
  meshDesc.triangles.count    = triCount;
  meshDesc.triangles.stride   = 3*sizeof(uint);
  meshDesc.triangles.data     = indices32;
  
  PxToolkit::MemoryOutputStream writeBuffer;
  bool status = cooking.cookTriangleMesh(meshDesc, writeBuffer);
  if(!status)
    return NULL;
    
  PxToolkit::MemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
  return physics.createTriangleMesh(readBuffer);
}

MemoryOutputStream::MemoryOutputStream() :
  mData(NULL),
  mSize(0),
  mCapacity(0) {
}

MemoryOutputStream::~MemoryOutputStream() {
  if(mData)
    delete[] mData;
}

PxU32 MemoryOutputStream::write(const void* src, PxU32 size) {
  PxU32 expectedSize = mSize + size;
  if(expectedSize > mCapacity) {
    mCapacity = expectedSize + 4096;
    
    PxU8* newData = new PxU8[mCapacity];
    PX_ASSERT(newData!=NULL);
    
    if(newData) {
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
  mSize(length),
  mData(data),
  mPos(0) {
}

PxU32 MemoryInputData::read(void* dest, PxU32 count) {
  PxU32 length = PxMin<PxU32>(count, mSize-mPos);
  memcpy(dest, mData+mPos, length);
  mPos += length;
  return length;
}

PxU32 MemoryInputData::getLength() const {
  return mSize;
}

void MemoryInputData::seek(PxU32 offset) {
  mPos = PxMin<PxU32>(mSize, offset);
}

PxU32 MemoryInputData::tell() const {
  return mPos;
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

#else //MT_PHYSX

#include "ors_physx.h"

PhysXInterface::PhysXInterface() { NICO }
PhysXInterface::~PhysXInterface() { NICO }
void PhysXInterface::create() { NICO }
void PhysXInterface::step() { NICO }
void PhysXInterface::glDraw() { NICO }
void PhysXInterface::pushState() { NICO }
void PhysXInterface::pullState() { NICO }
void PhysXInterface::ShutdownPhysX() { NICO }
void glPhysXInterface(void *classP) { NICO }
void bindOrsToPhysX(ors::Graph& graph, OpenGL& gl, PhysXInterface& physx) { NICO }

#endif
/** @} */
