
#include "array.h"

struct CudaInterface {
  void initMapping();
  void resizeMapping(arr& X, int N);
  
  void alloc(arr& X);
  void upload(const arr& X);
  void download(arr& X);
  void free(arr& X);
};




#ifdef  MT_IMPLEMENTATION
#  include "cudaModules.cpp"
#endif
