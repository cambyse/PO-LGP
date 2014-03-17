#include <Core/module.h>
#include <Core/array.h>

bool rosOk();

struct RosCom:Module{
  struct sRosCom *s;
  ACCESS(arr, q_ref);
  ACCESS(arr, qdot_ref);
  ACCESS(arr, q_obs);
  ACCESS(arr, qdot_obs);

  RosCom();

  void publishJointReference();
  void open();
  void step();
  void close();
};

