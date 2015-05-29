#include <vision_cuda.h>
#include "cudaModules.h"

int main(int argc,char **argv){
  CudaInterface CU;
  CU.initMapping();

  uint16A D;
  FILE("z.depth") >>D;



}
