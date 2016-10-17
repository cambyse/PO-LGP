#include "util_sharedMemStuff.h"

int main(int argn, char** argv){
  mlr::SHM shm;
  shm.open("hall", 100000, false);

  mlr::Shared<int> x(shm,"int");

  for(uint i=0;i<4;i++){
    int j;
    x.get(j);
    cout <<"x=" <<j <<endl;
    x.set(j+1);
    mlr::wait(1.);
  }

  shm.close();
  shm.destroy();
}
