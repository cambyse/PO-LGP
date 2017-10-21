#include <KOMO/komo.h>

//===============================================================================

struct KOMO_fineManip : KOMO{
  KOMO_fineManip(const mlr::KinematicWorld& K) : KOMO(K){}

  void setFineGrasp(double time, const char *endeff, const char *object, const char* gripper);

  void setFinePlace(double time, const char *endeff, const char *object, const char* placeRef, const char* gripper);

};
