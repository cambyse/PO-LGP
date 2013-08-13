/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

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

#ifndef _MT_taskMap_proxy_h
#define _MT_taskMap_proxy_h

#include "motion.h"

enum PTMtype {
  allPTMT, //phi=sum over all proxies (as is standard)
  allListedPTMT, //phi=sum over all proxies between listed shapes
  allExceptListedPTMT, //as above, but excluding listed shapes
  bipartitePTMT, //sum over proxies between the two sets of shapes (shapes, shapes2)
  pairsPTMT, //sum over proxies of explicitly listed pairs (shapes is n-times-2)
  allExceptPairsPTMT, //sum excluding these pairs
  vectorPTMT //vector of all pair proxies (this is the only case where dim(phi)>1)
};

/** Proxy task variable */
struct ProxyTaskMap:public TaskMap {
  /// @name data fields
  PTMtype type;
  uintA shapes,shapes2;
  double margin;
  bool linear;
  
  ProxyTaskMap();
  ProxyTaskMap(PTMtype _type,
               uintA _shapes,
               double _margin=.02,
               bool _linear=false);
  
  virtual void phi(arr& y, arr& J, const ors::Graph& G);
  virtual uint phiDim(const ors::Graph& G);
};

/** proxy align task variable */
struct ProxyAlignTaskMap:public TaskMap {
  /// @name data fields
  PTMtype type;
  uintA shapes,shapes2;
  double margin;
  bool linear;
  
  ProxyAlignTaskMap();
  ProxyAlignTaskMap(PTMtype _type,
                    uintA _shapes,
                    double _margin=3.,
                    bool _linear=true);
  
  virtual void phi(arr& y, arr& J, const ors::Graph& G);
  virtual uint phiDim(const ors::Graph& G);
};

#endif
