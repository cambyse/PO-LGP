#include <Core/module.h>
#include <Core/array.h>

bool rosOk();

struct RosCom:Module{
  struct sRosCom *s;
  ACCESS(arr, q);
  ACCESS(arr, qdot);
  ACCESS(arr, q_ref);
  ACCESS(arr, qdot_ref);

  RosCom();

  void publishJointReference();
  void open();
  void step();
  void close();
};

