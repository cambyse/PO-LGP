#include <MT/opengl.h>
#include <MT/ors.h>

struct GraspEvaluation{
  struct sGraspEvaluation *s;
  
  ors::Graph grasp; //!< the isolated grasp with only two bodies: hand and object
  OpenGL gl;
  arr contactPoints, contactNormals;
  double forceClosureMeassure;
  
  GraspEvaluation();
  
  
  void copyGraspFromOrs(const ors::Graph& all,
			const char* palmBodyName,
			const char* objShapeName);
  
  void closeFingers();
  
  void getContactPoints(double distanceThreshold=0.01);
  
  void simulateInPhysX();
};
