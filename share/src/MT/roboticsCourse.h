#ifndef MT_roboticsCourse_h
#define MT_roboticsCourse_h

#include "ors.h"
#include "array.h"

struct sSimulator; //forward declaration

struct Simulator {
  sSimulator *s; //hidden (private) space
  uint n;
  
  Simulator(const char* orsFile);
  ~Simulator();
  
  void watch(bool pause=true);                                    //pauses and lets you watch the OpenGL window
  
  //-- KINEMATICS
  //set the joint angles AND compute the frames of all bodies via
  //forward chaining of transformations AND update the robot display
  void setJointAngles(const arr& q, bool updateDisplay=true);
  void getJointAngles(arr& q);                     //get the joint angle vector for the current state
  uint getJointDimension();                        //get the dimensionality of q
  void kinematicsPos(arr& y, const char* bodyName, const arr* rel=0); //get the position of the body (names = "handR" or "handL", for instance)
  void jacobianPos(arr& J, const char* bodyName, const arr* rel=0);   //get the Jacobian of the body's position
  void kinematicsVec(arr& y, const char* bodyName, const arr* vec=0); //get the z-axis of the body
  void jacobianVec(arr& J, const char* bodyName, const arr* vec=0);   //get the Jacobian of the body's z-axis
  void kinematicsCOM(arr& y);
  void jacobianCOM(arr& J);
  void kinematicsContacts(arr& y);                     //get a scalar meassuring current collision costs
  void jacobianContacts(arr& J);                //get gradient of the collision cost
  void setContactMargin(double margin);            //set the collision margin
  void reportProxies();                            //write info on collisions to console
  
  //-- DYNAMICS
  //set the joint angles and velocities AND compute the frames and
  //linear & angular velocities of all bodies via
  //forward chaining of dynamic transformations AND update the robot display
  void setJointAnglesAndVels(const arr& q, const arr& qdot);
  void getJointAnglesAndVels(arr& q, arr& qdot);
  void getDynamics(arr& M, arr& F); //get the mass matrix and force vector describing the system equation
  double getEnergy();

  //-- step dynamic simulation
  void stepDynamic(const arr& u_control, double tau);
  void setDynamicSimulationNoise(double noise);
  void setDynamicGravity(bool gravity);
  
  //-- Physical Simulation using the OpenDynamicsEngine (ODE)
  void stepOde(const arr& qdot, bool updateDisplay=true);
  
  void anchorKinematicChainIn(const char* bodyName);
};

struct sVisionSimulator; //forward declaration

struct VisionSimulator {
  sVisionSimulator *s;
  
  VisionSimulator();
  ~VisionSimulator();
  
  void watch();
  
  arr getCameraTranslation();
  void getRandomWorldPoints(arr& X, uint N);
  void projectWorldPointsToImagePoints(arr& x, const arr& X, double noiseInPixel=1.);
};

struct Gaussian{  arr A;  arr a;  };

struct CarSimulator{
  double x,y,theta; //this is the true state -- accessing it means cheating...
  double tau,L;
  double dynamicsNoise, observationNoise;
  arr landmarks;
  arr particlesToDraw;
  MT::Array<Gaussian> gaussiansToDraw;
  OpenGL *gl;
  
  CarSimulator();
  void step(const arr& u); 
  //get a (noisy) observation (meassuring landmarks) in the current state
  void getRealNoisyObservation(arr& Y);

  //-- access for tracking, where the true landmarks are known (Kalman/particle filter)
  //compute the ideal mean observation if you were in state (x,y,theta): use this to compute Y for a particle and compare with meassured observation
  void getMeanObservationAtState(arr& Y, const arr& X);
  void getObservationJacobianAtState(arr& dy_dx, const arr& X);

  //-- access for SLAM, where only estimated landmarks are accessible
  void getMeanObservationAtStateAndLandmarks(arr& Y, const arr& x, const arr& landmarks);
  void getObservationJacobianAtStateAndLandmarks(arr& dy_dx, arr& dy_dlandmarks, const arr& x, const arr& landmarks);

  void getLinearObservationModelAtState(arr& C, arr& c, const arr& X);

};


#endif


