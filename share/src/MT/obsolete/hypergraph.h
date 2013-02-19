/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
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


#ifndef MT_hypergraph_h
#define MT_hypergraph_h

#include "util.h"
#include "array.h"

#define maxDegree 10

struct Element;
//list of elements:
// actually this is the whole `HyperGraph' data structure
// the class below only adds convenience stuff
typedef MT::Array<Element*> ElementL;

struct Element {
  uint index;              //!< index of this element
  MT::String type, name;   //!< type & name
  ElementL parents;        //!< this element has a set of parents (e.g., an edge has two parents)
  ElementL parentOf[maxDegree]; //!< this elememt is the parent of ... in a certain 'slot'

  KeyValueGraph ats;             //!< list of any-type attributes
  
  void write(std::ostream& os) const;
  void read(std::istream& is, ElementL& list);
};
stdOutPipe(Element);


struct HyperGraph:ElementL {
  uintA unused;
  
  ElementL& getOutEdges(uint i);
  Element *add(const uintA& tuple);
  void del(Element *e);
  Element *get(const char* name);
  
  void write(std::ostream &os) const;
  void read(std::istream &is);
};
stdPipes(HyperGraph);


inline void sortByDotOrder(ElementL& G){
  uintA perm(G.N);
  Element *e;
  double *order;
  uint i;
  for_list(i,e,G){
    order = anyListGet<double>(e->ats, "dot_order", 1);
    if(!order){ MT_MSG("doesn't have dot_order attribute"); return; }
    perm(i) = (uint)*order;
  }
  G.permuteInv(perm);
  for_list(i,e,G) e->index=i;
}

inline void writeDot(ElementL& G){
  ofstream fil;
  MT::open(fil, "z.dot");
  fil <<"graph G{" <<endl;
  fil <<"graph [ rankdir=\"LR\", ranksep=0.05 ];" <<endl;
  fil <<"node [ fontsize=9, width=.3, height=.3 ];" <<endl;
  fil <<"edge [ arrowtail=dot, arrowsize=.5, fontsize=6 ];" <<endl;
  uint i, j;
  Element *e, *n;
  for_list(i, e, G){
    fil <<e->index <<" [ ";
    if(e->name.N) fil <<"label=\"" <<e->name <<"\", ";
    if(e->type=="edge" || e->type=="joint" || e->type=="Process" || e->type=="factor") fil <<"shape=box";
    else if(e->type=="shape") fil <<"shape=diamond";
    else fil <<"shape=ellipse";
    fil <<" ];" <<endl;
    for_list(j, n, e->parents){
      if(n->index<e->index)
        fil <<n->index <<" -- " <<e->index <<" [ ";
      else
        fil <<e->index <<" -- " <<n->index <<" [ ";
      fil <<"label=" <<j;
      fil <<" ];" <<endl;
    }
  }
  fil <<"}" <<endl;
  fil.close();
}

#ifdef  MT_IMPLEMENTATION
#  include "hypergraph.cpp"
#endif

#endif
