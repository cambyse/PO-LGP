#include "ors.h"

#ifdef MT_Lewiner
#include <stdio.h>
#include "Lewiner/MarchingCubes.h"

void ors::Mesh::setImplicitSurface(double(*fct)(double, double, double, void*), void *p, double lo, double hi, uint res) {
  MarchingCubes mc(res, res, res);
  mc.init_all() ;
  
  //compute data
  uint k=0, j=0, i=0;
  float x=lo, y=lo, z=lo;
  for(k=0; k<res; k++) {
    z = lo+k*(hi-lo)/res;
    for(j=0; j<res; j++) {
      y = lo+j*(hi-lo)/res;
      for(i=0; i<res; i++) {
        x = lo+i*(hi-lo)/res;
        mc.set_data((double)fct(x, y, z, p), i, j, k) ;
      }
    }
  }
  
  mc.run();
  mc.clean_temps();
  
  //convert to Mesh
  clear();
  V.resize(mc.nverts(), 3);
  T.resize(mc.ntrigs(), 3);
  for(i=0; i<V.d0; i++) {
    V(i, 0)=lo+mc.vert(i)->x*(hi-lo)/res;
    V(i, 1)=lo+mc.vert(i)->y*(hi-lo)/res;
    V(i, 2)=lo+mc.vert(i)->z*(hi-lo)/res;
  }
  for(i=0; i<T.d0; i++) {
    T(i, 0)=mc.trig(i)->v1;
    T(i, 1)=mc.trig(i)->v2;
    T(i, 2)=mc.trig(i)->v3;
  }
}

#else
void ors::Mesh::setImplicitSurface(double(*fct)(double, double, double, void*), void *p, double lo, double hi, uint res) {
  NIY;
}
#endif
