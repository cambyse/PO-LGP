#include <Algo/priorityQueue.h>
#include <Core/util.h>
#include <memory>

void TEST(PriorityQueue) {

  PriorityQueue<mlr::String*> P;

  for(uint k=0;k<30;k++){
    mlr::String str;
    str.setRandom();
    double p=1.;
    for(uint i=str.N;i--;){ p /= 128.; p+=(int)str(i); }
    P.add(p,new String(str));
  }

  cout <<P <<endl;
}


int MAIN(int argc,char** argv){

  testPriorityQueue();

  return 0;
}
