#ifndef MT_hypergraph_h
#define MT_hypergraph_h

#include "util.h"
#include "array.h"
#include "ors.h"

#define maxDegree 10

struct Element;
typedef MT::Array<Element*> ElementL;

struct Element{
  uint id;                 //!< id of this element
  String type,name;        //!< type & name
  uintA    linksIds;       //!< this elem links a set of other elems
  ElementL links;          //!< this elem links a set of other elems
  ElementL elemof[maxDegree]; //!< this elem is linked by other elems in a certain 'slot'
  AnyList ats;             //!< list of any-type attributes

  void write(std::ostream& os) const;  
  void read(std::istream& is,ElementL& list);
};
stdOutPipe(Element);

struct HyperGraph{
  uint N[maxDegree];
  uintA unused;
  ElementL T;
  
  ElementL& getOutEdges(uint i);
  Element *add(const uintA& tuple);
  void del(Element *e);
  Element *get(const char* name);
  
  void write(std::ostream &os) const;  
  void read(std::istream &is);
};
stdPipes(HyperGraph);


void writeDot(ElementL G){
  ofstream fil;
  MT::open(fil,"z.dot");
  fil <<"graph G{" <<endl;
  fil <<"node [ fontsize=9 ];" <<endl;
  fil <<"edge [ arrowtail=dot, arrowsize=.5, fontsize=6 ];" <<endl;
  uint i,j;
  Element *e,*n;
  for_list(i,e,G){
    fil <<e->id <<" [ ";
    if(e->name.N()) fil <<"label=\"" <<e->name <<"\", ";
    if(e->type=="edge" || e->type=="joint" || e->type=="Process" || e->type=="factor") fil <<"shape=box";
    else if(e->type=="shape") fil <<"shape=diamond";
    else fil <<"shape=ellipse";
    fil <<" ];" <<endl;
    for_list(j,n,e->links){
      fil <<e->id <<" -- " <<n->id <<" [ ";
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
