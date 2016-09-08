#include <vector>

struct LockboxSwig{
  struct sLockboxSwig *s;

  LockboxSwig();
  ~LockboxSwig();

  bool testJoint(int jointNumber);
  double getJointPosition(int jointNumber);

  void testArray(std::vector<double> x);
};
