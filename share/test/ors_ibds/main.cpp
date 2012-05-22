
#include <stdlib.h>
#include <MT/ors.h>
#include <MT/opengl.h>
//#include<MT/ors_ibds.h>
#include <DynamicSimulation/Simulation.h>
#include <DynamicSimulation/RigidBody.h>
#include <DynamicSimulation/BallJoint.h>
#include <DynamicSimulation/BallSliderJoint.h>
#include <DynamicSimulation/HingeJoint.h>
#include <DynamicSimulation/MotorHingeJoint.h>
#include <DynamicSimulation/MotorSliderJoint.h>
#include <DynamicSimulation/SpringJoint.h>
#include <DynamicSimulation/AngularPIDController.h>
#include <DynamicSimulation/SphereGeometry.h>
#include <Math/SimMath.h>
#include <DynamicSimulation/MeshGeometry.h>



using namespace IBDS;

void timeStep();
void buildModel();
void render();
void exit();
Geometry* createCubeGeometry(Real sx, Real sy, Real sz, float r, float g, float b, float a);
void addCollisionCube(RigidBody *b, Real sx, Real sy, Real sz);
Geometry* createSphereGeometry(Real radius, float r, float g, float b, float a);
void addCollisionSphere(RigidBody *b, Real radius);

#define MAX_OBJECTS 50
int controllerIndex = Simulation::TIMESTEP_ITERATIVE;


// main

struct IbdsModule {
  Simulation *sim;
  MT::Array<RigidBody*> bodies;
  
  RigidBody *floor;
  
  ~IbdsModule() { delete sim; }
};

void stepIbdsSimulation(IbdsModule& ibds) {
  for(int i=0; i<1; i++) ibds.sim->timeStep();
}

void createIbds(const ors::Graph& C,IbdsModule& ibds) {
  Simulation *sim = Simulation::getCurrent();
  ibds.sim = sim;
  /*
  sim->setCollisionDetectionMethod(CollisionDetection::CD_BULLET);
  sim->setMaxDistance(controllerIndex, 0.000001);
  sim->setMaxVelDiff(controllerIndex, 0.001);
  sim->setMaxCorrectionSteps(controllerIndex, 100);
  sim->setCRMethod(Simulation::CR_JBNEWTON);
  sim->setTimeOfImpactComputation(Simulation::TOI_BINARYSEARCH);
  (*sim->getCollisionResponseController())[sim->getCRMethod()]->setMaxCorrectionSteps(100);
  (*sim->getCollisionResponseController())[sim->getCRMethod()]->setMaxVelDiff(0.001);
  (*sim->getCollisionResponseController())[sim->getCRMethod()]->setShockPropagation(true);
  sim->setMaxDistanceContacts(controllerIndex, 0.0001);
  sim->setMaxCorrectionStepsContacts(controllerIndex, 100);
  (*sim->getTimeStepController())[controllerIndex]->setVCorrectionContacts(false);
  sim->getCollisionDetection()->setTolerance(0.04);
  sim->setVCorrection(controllerIndex, true);
  sim->setMaxCorrImpulse(controllerIndex, 10);
  sim->setMaxJointImpulse(controllerIndex, -1);
  sim->setIntegrationMethod(controllerIndex, TimeStepController::RUNGE_KUTTA);
  TimeManager::getCurrent()->setTimeStepSize(0.01);
  */
  sim->setGravitation(Vector3D(0,0,-9.81));

  srand((unsigned) time(NULL));
  
  // Boden
  ibds.floor = new RigidBody();
  ibds.floor->setMass(1);
  ibds.floor->setInertiaTensor(Vector3D(1, 1, 1));
  ibds.floor->setCenterOfMass(Vector3D(0,0.0,-.15));
  ibds.floor->setDynamic(false);
  //ibds.floor->setRestitution(0.6);
  //ibds.floor->setDynamicFriction(0.1);
  //ibds.floor->setStaticFriction(0.1);
  //ibds.floor->addGeometry(createCubeGeometry(100, 100, .2, 0.0f,0.0f,0.8f,1.0f));
  addCollisionCube(ibds.floor, 100, 100, .2);
  
  //double *mass,*shape,*type,*fixed,*cont,typeD=ors::cylinderST;
  
  double rot[9];
  Matrix3x3 mat;
  
  ibds.bodies.resize(C.bodies.N);
  ors::Body *n;
  RigidBody *b;
  int i;
  for(i=0; i<C.bodies.N; i++) {
    n=C.bodies(i);
    b=new RigidBody();
    ibds.bodies(i)=b;
    
    b->setMass(n->mass);
    n->X.rot.getMatrix(rot);
    mat[0][0]=rot[0]; mat[0][1]=rot[1]; mat[0][2]=rot[2];
    mat[1][0]=rot[3]; mat[1][1]=rot[4]; mat[1][2]=rot[5];
    mat[2][0]=rot[6]; mat[2][1]=rot[7]; mat[2][2]=rot[8];
    b->setRotationMatrix(mat);
    b->setCenterOfMass(Vector3D(n->X.pos(0),n->X.pos(1),n->X.pos(2)));
    //b->setAngularVelocity (Vector3D (vx,0.0,vz));
    b->setDynamic(true);
    //b->setRestitution(0.8);
    //b->setDynamicFriction(0.4);
    //b->setStaticFriction(0.4);
    
    
    
    // need to fix: "mass" is not a mass but a density (in dMassSetBox)
    
    ors::Shape *s = n->shapes(0);
    switch(s->type) {
    default:
    case ors::boxST: // box
      //b->addGeometry(createCubeGeometry(s->size[0], s->size[1], s->size[2], 0.0f,0.6f,0.0f,1.0f));
      b->setInertiaTensor(SimMath::computeBoxIntertiaTensor(n->mass, s->size[0], s->size[1], s->size[2]));
      sim->addCollisionObject(b, s->mesh.V.d0, s->mesh.V.p, s->mesh.T.d0, (int*)s->mesh.T.p, NULL);
      //addCollisionCube(b, s->size[0], s->size[1], s->size[2]);
      break;
    case ors::sphereST: // sphere
      //b->addGeometry(createSphereGeometry(s->size[3], 0.0f,0.3f,0.6f,1.0f));
      b->setInertiaTensor(SimMath::computeSphereIntertiaTensor(n->mass, s->size[3]));
      sim->addCollisionObject(b, s->mesh.V.d0, s->mesh.V.p, s->mesh.T.d0, (int*)s->mesh.T.p, NULL);
      //addCollisionSphere(b, s->size[3]);
      break;
    case ors::cylinderST:
    case ors::cappedCylinderST:
    case ors::meshST:
      MeshGeometry *geo = new MeshGeometry();
      cout <<s->mesh.V.d0 <<endl;
      sim->addCollisionObject(b, s->mesh.V.d0, s->mesh.V.p, s->mesh.T.d0, (int*)s->mesh.T.p, NULL);
      break;
    }
    
    b->setName(n->name.p);
    sim->addBody(controllerIndex,b);
  }
  
  sim->buildModel();
  
}


void importStateFromIbds(ors::Graph& C,IbdsModule& ibds) {
  // Bodies
  ors::Body *n;
  RigidBody *b;
  int i;
  for(int i=0; i<C.bodies.N; i++) {
    n=C.bodies(i);
    b=ibds.bodies(i);
    
    Matrix3x3 *rotation = b->getRotationMatrix();
    double rot[9];
    rot[0] = (*rotation)[0][0]; rot[3] = (*rotation)[0][1]; rot[6] = (*rotation)[0][2];
    rot[1] = (*rotation)[1][0]; rot[4] = (*rotation)[1][1]; rot[7] = (*rotation)[1][2];
    rot[2] = (*rotation)[2][0]; rot[5] = (*rotation)[2][1]; rot[8] = (*rotation)[2][2];
    
    n->X.rot.setMatrix(rot);
    n->X.pos.set(b->getCenterOfMass()->v);
    n->X.vel.set(b->getCenterOfMassV()->v);
  }
  C.calcShapeFramesFromBodies();
}


/** Erzeugt eine W�rfel-Geometrie mit der �bergebenen Skalierung und gibt sie zur�ck.
  */
Geometry* createCubeGeometry(Real sx, Real sy, Real sz, float r, float g, float b, float a) {
  // Cube data
  const Real vertices[] = {
    -0.5,  0.5,  0.5,
    -0.5,  -0.5,  0.5,
    -0.5,  -0.5,  -0.5,
    -0.5,  0.5,  -0.5,
    0.5,  -0.5,  0.5,
    0.5,  -0.5,  -0.5,
    0.5,  0.5,  0.5,
    0.5,  0.5,  -0.5
  };
  
  const int faces[] = {
    0, 2, 1,
    0, 3, 2,
    0, 1, 4,
    6, 0, 4,
    4, 5, 6,
    5, 7, 6,
    2, 3, 5,
    5, 3, 7,
    0, 6, 3,
    3, 6, 7,
    1, 2, 4,
    4, 2, 5
  };
  MeshGeometry *geo = new MeshGeometry();
  Vector3D scale = Vector3D(sx, sy, sz);
  geo->setTriangles(8, vertices, 12, faces, NULL, NULL, &scale);
  geo->setColor(r, g, b, a);
  return geo;
}

/** Erzeugt eine W�rfel-Geometrie mit der �bergebenen Skalierung und gibt sie zur�ck.
  */
Geometry* createSphereGeometry(Real radius, float r, float g, float b, float a) {
  SphereGeometry *geo = new SphereGeometry();
  geo->setRadius(radius);
  geo->setColor(r, g, b, a);
  return geo;
}

/** Erzeugt eine W�rfel-Geometrie mit der �bergebenen Skalierung und verwendet sie als Kollisionsobjekt.
  */
void addCollisionCube(RigidBody *b, Real sx, Real sy, Real sz) {
  // Cube data
  const Real vertices[] = {
    -0.5,  0.5,  0.5,
    -0.5,  -0.5,  0.5,
    -0.5,  -0.5,  -0.5,
    -0.5,  0.5,  -0.5,
    0.5,  -0.5,  0.5,
    0.5,  -0.5,  -0.5,
    0.5,  0.5,  0.5,
    0.5,  0.5,  -0.5
  };
  
  const int faces[] = {
    0, 2, 1,
    0, 3, 2,
    0, 1, 4,
    6, 0, 4,
    4, 5, 6,
    5, 7, 6,
    2, 3, 5,
    5, 3, 7,
    0, 6, 3,
    3, 6, 7,
    1, 2, 4,
    4, 2, 5
  };
  Simulation *sim = Simulation::getCurrent();
  
  Vector3D scale = Vector3D(sx, sy, sz);
  sim->addCollisionObject(b, 8, vertices, 12, faces, &scale);
}

/** Erzeugt eine Kugel-Geometrie mit der �bergebenen Skalierung und verwendet sie als Kollisionsobjekt.
  */
void addCollisionSphere(RigidBody *b, Real radius) {
  Simulation *sim = Simulation::getCurrent();
  
  sim->addCollisionSphere(b, Vector3D(), radius);
}

void drawEnv(void*) {
  glStandardLight(NULL);
  glDrawFloor(4.,1,1,1);
}

int main(int argc, char **argv) {
  IbdsModule I;
  ors::Graph C;
  OpenGL gl;

  C.init("test_ibds.ors");
  
  gl.add(drawEnv,0);
  gl.add(ors::glDrawGraph,&C);
  gl.setClearColors(1.,1.,1.,1.);
  gl.camera.setPosition(10.,-15.,8.);
  gl.camera.focus(0,0,1.);
  gl.watch();
  
  createIbds(C,I);
  
  uint t;
  for(t=0;; t++) {
    stepIbdsSimulation(I);
    importStateFromIbds(C,I);
    cout <<"\r t = " <<t <<" energy = " <<C.getEnergy() <<std::flush;
    gl.watch();
  }
  
  return 0;
}
