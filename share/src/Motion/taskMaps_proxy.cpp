/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#include "taskMaps.h"

TaskMap_Proxy::TaskMap_Proxy(PTMtype _type,
                           uintA _shapes,
                           double _margin,
                           bool _useCenterDist,
                           bool _useDistNotCost) {
  type=_type;
  shapes=_shapes;
  margin=_margin;
  useCenterDist=_useCenterDist;
  useDistNotCost=_useDistNotCost;
  cout <<"creating TaskMap_Proxy with shape list" <<shapes <<endl;
}

void TaskMap_Proxy::phi(arr& y, arr& J, const ors::KinematicWorld& G, int t){
  uintA shapes_t;
  shapes_t.referTo(shapes);

  y.resize(1).setZero();
  if(&J) J.resize(1, G.getJointStateDimension(false)).setZero();

  switch(type) {
    case allPTMT:
      for(ors::Proxy *p: G.proxies)  if(p->d<margin) {
        G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
        p->colorCode = 1;
      }
      break;
    case listedVsListedPTMT:
      for(ors::Proxy *p: G.proxies)  if(p->d<margin) {
        if(shapes.contains(p->a) && shapes.contains(p->b)) {
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
          p->colorCode = 2;
        }
      }
      break;
    case allVsListedPTMT: {
      if(t && shapes.nd==2) shapes_t.referToDim(shapes,t);
      for(ors::Proxy *p: G.proxies)  if(p->d<margin) {
        if(shapes_t.contains(p->a) || shapes_t.contains(p->b)) {
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
          p->colorCode = 2;
        }
      }
    } break;
    case allExceptListedPTMT:
      for(ors::Proxy *p: G.proxies)  if(p->d<margin) {
        if(!(shapes.contains(p->a) && shapes.contains(p->b))) {
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
          p->colorCode = 3;
        }
      }
      break;
    case bipartitePTMT:
      for(ors::Proxy *p: G.proxies)  if(p->d<margin) {
        if((shapes.contains(p->a) && shapes2.contains(p->b)) ||
            (shapes.contains(p->b) && shapes2.contains(p->a))) {
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
          p->colorCode = 4;
        }
      }
      break;
    case pairsPTMT: {
      shapes.reshape(shapes.N/2,2);
      // only explicit paris in 2D array shapes
      uint j;
      for(ors::Proxy *p: G.proxies) if(p->d<margin) {
        for(j=0; j<shapes.d0; j++) {
          if((shapes(j,0)==(uint)p->a && shapes(j,1)==(uint)p->b) || (shapes(j,0)==(uint)p->b && shapes(j,1)==(uint)p->a))
            break;
        }
        if(j<shapes.d0) { //if a pair was found
          if(useDistNotCost) G.kinematicsProxyDist(y, J, p, margin, useCenterDist, true);
          else G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
          p->colorCode = 5;
        }
      }
    } break;
    case allExceptPairsPTMT: {
      shapes.reshape(shapes.N/2,2);
      // only explicit paris in 2D array shapes
      uint j;
      for(ors::Proxy *p: G.proxies)  if(p->d<margin) {
        for(j=0; j<shapes.d0; j++) {
          if((shapes(j,0)==(uint)p->a && shapes(j,1)==(uint)p->b) || (shapes(j,0)==(uint)p->b && shapes(j,1)==(uint)p->a))
            break;
        }
        if(j==shapes.d0) { //if a pair was not found
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
          p->colorCode = 5;
        }
      }
    } break;
    case vectorPTMT: {
      //outputs a vector of collision meassures, with entry for each explicit pair
      shapes.reshape(shapes.N/2,2);
      y.resize(shapes.d0, 1);  y.setZero();
      if(&J){ J.resize(shapes.d0,J.d1);  J.setZero(); }
      uint j;
      for(ors::Proxy *p: G.proxies)  if(p->d<margin) {
        for(j=0; j<shapes.d0; j++) {
          if((shapes(j,0)==(uint)p->a && shapes(j,1)==(uint)p->b) || (shapes(j,0)==(uint)p->b && shapes(j,1)==(uint)p->a))
            break;
        }
        if(j<shapes.d0) {
          G.kinematicsProxyCost(y[j](), (&J?J[j]():NoArr), p, margin, useCenterDist, true);
          p->colorCode = 5;
        }
      }
      y.reshape(shapes.d0);
    } break;
    default: NIY;
  }
}

uint TaskMap_Proxy::dim_phi(const ors::KinematicWorld& G){
  switch(type) {
  case allPTMT:
  case listedVsListedPTMT:
  case allVsListedPTMT:
  case allExceptListedPTMT:
  case bipartitePTMT:
  case pairsPTMT:
  case allExceptPairsPTMT:
    return 1;
  case vectorPTMT:
    return shapes.d0;
  default: NIY;
  }
}
