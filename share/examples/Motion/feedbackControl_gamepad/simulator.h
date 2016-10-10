#include <Core/array.h>
#include <Core/thread.h>

struct PR2Simulator : Thread {
  ACCESS(arr, q_ref);
  ACCESS(arr, qdot_ref);
  ACCESS(arr, q_obs);
  ACCESS(arr, qdot_obs);

  struct sPR2Simulator *s;
  PR2Simulator();
  ~PR2Simulator();
  void open();
  void step();
  void close();
};
