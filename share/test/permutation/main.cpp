#include <MT/util.h>
#include <MT/array.h>

using namespace MT;


int main(int argc, char *argv[]){
  uintA p;
  rnd.seed(3);

  p.setStraightPerm(10);
  std::cout <<"init:      " <<p;

  p.permute(2,5);
  std::cout <<"permute(2,5): " <<p;

  p.setRandomPerm();
  std::cout <<"random:    " <<p;

  for(uint i=0;i<p.N;i++) std::cout <<i <<":" <<p(i) <<"\n";

  return 0;
}

