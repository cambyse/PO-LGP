#include "taskMap_proxy.h"

ProxyTaskMap::ProxyTaskMap(PTMtype _type,
             uintA _shapes,
             double _margin,
             bool _linear) {
  type=_type;
  shapes=_shapes;
  margin=_margin;
  linear=_linear;
  cout <<"creating ProxyTaskMap with shape list" <<shapes <<endl;
}

//TODO: defined in Ors/ors_taskVariables.cpp !!
void addAContact(double& y, arr& J, const ors::Proxy *p, const ors::Graph& G, double margin, bool linear);

void ProxyTaskMap::phi(arr& y, arr& J, const ors::Graph& G){
  uint i;
  ors::Proxy *p;

  y.resize(1);  y.setZero();
  if(&J){ J.resize(1, G.getJointStateDimension(false));  J.setZero(); }

  switch(type) {
    case allCTVT:
      for_list(i,p,G.proxies)  if(p->d<margin) {
        addAContact(y(0), J, p, G, margin, linear);
        p->colorCode = 1;
      }
      break;
    case allListedCTVT:
      for_list(i,p,G.proxies)  if(p->d<margin) {
        if(shapes.contains(p->a) && shapes.contains(p->b)) {
          addAContact(y(0), J, p, G, margin, linear);
          p->colorCode = 2;
        }
      }
    case allExceptListedCTVT:
      for_list(i,p,G.proxies)  if(p->d<margin) {
        if(!shapes.contains(p->a) && !shapes.contains(p->b)) {
          addAContact(y(0), J, p, G, margin, linear);
          p->colorCode = 3;
        }
      }
      break;
    case bipartiteCTVT:
      for_list(i,p,G.proxies)  if(p->d<margin) {
        if((shapes.contains(p->a) && shapes2.contains(p->b)) ||
            (shapes.contains(p->b) && shapes2.contains(p->a))) {
          addAContact(y(0), J, p, G, margin, linear);
          p->colorCode = 4;
        }
      }
      break;
    case pairsCTVT: {
      shapes.reshape(shapes.N/2,2);
      // only explicit paris in 2D array shapes
      uint j;
      for_list(i,p,G.proxies)  if(p->d<margin) {
        for(j=0; j<shapes.d0; j++) {
          if((shapes(j,0)==(uint)p->a && shapes(j,1)==(uint)p->b) || (shapes(j,0)==(uint)p->b && shapes(j,1)==(uint)p->a))
            break;
        }
        if(j<shapes.d0) { //if a pair was found
          addAContact(y(0), J, p, G, margin, linear);
          p->colorCode = 5;
        }
      }
    } break;
    case allExceptPairsCTVT: {
      shapes.reshape(shapes.N/2,2);
      // only explicit paris in 2D array shapes
      uint j;
      for_list(i,p,G.proxies)  if(p->d<margin) {
        for(j=0; j<shapes.d0; j++) {
          if((shapes(j,0)==(uint)p->a && shapes(j,1)==(uint)p->b) || (shapes(j,0)==(uint)p->b && shapes(j,1)==(uint)p->a))
            break;
        }
        if(j==shapes.d0) { //if a pair was not found
          addAContact(y(0), J, p, G, margin, linear);
          p->colorCode = 5;
        }
      }
    } break;
    case vectorCTVT: {
      //outputs a vector of collision meassures, with entry for each explicit pair
      shapes.reshape(shapes.N/2,2);
      y.resize(shapes.d0);  y.setZero();
      if(&J){ J.resize(shapes.d0,J.d1);  J.setZero(); }
      uint j;
      for_list(i,p,G.proxies)  if(p->d<margin) {
        for(j=0; j<shapes.d0; j++) {
          if((shapes(j,0)==(uint)p->a && shapes(j,1)==(uint)p->b) || (shapes(j,0)==(uint)p->b && shapes(j,1)==(uint)p->a))
            break;
        }
        if(j<shapes.d0) {
          addAContact(y(j), J[j](), p, G, margin, linear);
          p->colorCode = 5;
        }
      }
    } break;
    default: NIY;
  }
}

uint ProxyTaskMap::dim_phi(const ors::Graph& G){
  switch(type) {
  case allCTVT:
  case allListedCTVT:
  case allExceptListedCTVT:
  case bipartiteCTVT:
  case pairsCTVT:
  case allExceptPairsCTVT:
    return 1;
  case vectorCTVT:
    return shapes.d0;
  default: NIY;
  }
}
