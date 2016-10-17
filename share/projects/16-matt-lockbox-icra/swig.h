#include <vector>

struct LockboxSwig{
  struct sLockboxSwig *s;

  LockboxSwig();
  ~LockboxSwig();

  bool testJoint(const int jointNumber);
  double getJointPosition(const int jointNumber);

//  void testArray(std::vector<double> x);
};
