#include <Gui/opengl.h>
#include <Kin/kin.h>

struct GraspEvaluation{
  struct sGraspEvaluation *s;
  
  mlr::KinematicWorld grasp; ///< the isolated grasp with only two bodies: hand and object
  OpenGL gl;
  arr contactPoints, contactNormals;

  double score_forceClosure;
  double score_torqueClosure;
  double score_wrenchClosure;
  
  GraspEvaluation();
  
  
  void copyGraspFromOrs(const mlr::KinematicWorld& all,
			const char* palmBodyName,
			const char* objShapeName);
  
  void closeFingers();
  
  void getContactPoints(double distanceThreshold=0.01);
  
  void simulateInPhysX();
};
