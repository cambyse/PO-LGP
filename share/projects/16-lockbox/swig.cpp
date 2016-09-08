#include "swig.h"

#include "lockbox/lockbox.h"

struct sLockboxSwig{
  MyBaxter B;
  Lockbox L;

  sLockboxSwig();

};



LockboxSwig::LockboxSwig()
  :s (new sLockboxSwig){
}

LockboxSwig::~LockboxSwig(){
  delete s;
}

bool LockboxSwig::testJoint(int jointNumber){
  cout <<"HI! " <<jointNumber <<endl;
  return false;
}

double LockboxSwig::getJointPosition(int jointNumber){
  return 1.0;
}

void LockboxSwig::testArray(std::vector<double> _x){
  arr x = conv_stdvec2arr(_x);
  cout <<"i received an array: " <<x <<endl;
}




sLockboxSwig::sLockboxSwig():L(&B){

}
