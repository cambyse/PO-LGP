#include <vector>

struct LockboxSwig{
  struct sLockboxSwig *s;

  LockboxSwig();
  ~LockboxSwig();

  bool testJoint(int jointNumber);
  double getJointPosition(int jointNumber);
};
