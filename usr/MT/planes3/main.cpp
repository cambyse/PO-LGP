#include <Perception/plane.h>

void testGradients(){
  uint ndata = 100;
  arr n = randn(3);
  arr m = randn(3);
  arr X = randn(ndata,3);
  arr transform = randn(7);
  transform.subRef(3,6)() /= length(transform.sub(3,6));

  CostFct_PlanePoints f(n, m, X, transform);

  checkGradient(f.f_transform(), transform, 1e-4);

}

int main(int argc,char **argv){
  rnd.clockSeed();
  testGradients();
}
