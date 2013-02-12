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


#include "hypergraph.h"

void Element::write(std::ostream& os) const {
  uint i;  Element *e;
  if(type.N) os <<type <<' ';
  if(name.N) os <<name <<' '; else os <<index <<' ';
  if(parents.N){
    os <<" (";
    for_list(i, e, parents){
      if(i) os <<' ';
      if(e->name.N) os <<e->name; else os <<e->index;
    }
    os <<") ";
  }
  if(ats.N){
    os <<"{ ";
    listWrite(ats, os, ", ");
    os <<" }";
  }
}

void Element::read(std::istream& is, ElementL& list){
  uint i, j;
  MT::String link;
  Element *e, *e2;
  type.read(is, " \t\n\r", " \t\n\r({", false);
  name.read(is, " \t\n\r", " \t\n\r({", false);
  if(MT::peerNextChar(is) =='('){
    MT::parse(is, "(");
    for(j=0;; j++){
      link.read(is, " , ", " , )", false);
      if(!link.N) break;
      e=listFindByName(list, link);
      if(e){ //sucessfully found
        parents.append(e);
//        linkIds.append(e->index);
        e->parentOf[j].append(this);
      }else{//this element is not known!!
        HALT("line:" <<MT::lineCount <<" reading element '" <<name <<"': unknown " <<j <<"th linked element '" <<link <<"'"); //DON'T DO THIS YET
        //check if this is a derived element (notationally: new_name = old_name+'one_char')
        MT::String sublink;
        sublink.set(link.p, link.N-1);
        for_list(i, e, list) if(e->name==sublink) break;
        if(i<list.N){//sucessfully found
          //create new element with same type and attributes, but extended name!!
          e2=new Element;
          e2->name=link;
          e2->type=e->type;
          e2->ats=e->ats;
          e2->index=list.N;
          list.append(e2);
          parents.append(e2);
//          linkIds.append(e2->index);
          e2->parentOf[j].append(this);
        }else{
          HALT("reading element '" <<name <<"': unknown " <<j <<"th contained element '" <<link <<"'");
        }
      }
    }
    MT::parse(is, ")");
  }
  if(MT::peerNextChar(is) =='{'){
    MT::parse(is, "{");
    anyListRead(ats, is);
    MT::parse(is, "}");
  }
}


ElementL& HyperGraph::getOutEdges(uint i){
  return elem(i)->parentOf[0];
}

Element *HyperGraph::add(const uintA& tuple){
  CHECK(tuple.N<maxDegree, "");
  uint i;
  Element *e = new Element;
  if(unused.N){
    e->index=unused.popLast();
    elem(e->index)=e;
    
  }else{
    e->index=N;
    append(e); }
//  e->linkIds=tuple;
  //N[tuple.N]++;
  for(i=0; i<tuple.N; i++){
    elem(tuple(i))->parentOf[i].append(e);
    e->parents.append(elem(tuple(i)));
  }
  return e;
}

void HyperGraph::del(Element *e){
  CHECK(elem(e->index)==e, "this is not an element of the hypergraph")
  uint i;
  for(i=0; i<maxDegree; i++){
    if(e->parentOf[i].N){
      HALT("can't delete element " <<e->index <<" - it is containted in others:\n");
      e->write(cerr);
    }
  }
  //N[e->links.N]--;
  unused.append(e->index);
  elem(e->index) = NULL;
  delete e;
}

Element *HyperGraph::get(const char* name){ return listFindByName(*this, name); }

void HyperGraph::write(std::ostream &os) const {
  listWrite(*this, os, "\n");
}

/*void connectElements(){
uint i;
Element *e;
for_list(i, e, T){
  e->id=i;
  for(i=0;i<e->linksIds.N;i++){
    T(e->linksIds(i))->elemof[i].append(e);
    e->links.append(T(e->linksIds(i)));
    }
    }
    }*/

void HyperGraph::read(std::istream &is){
  CHECK(!N, "delete the list before reading!");
  char c;
  Element *e;
  for(;;){
    c=MT::peerNextChar(is);
    if(!is.good()){ is.clear(); break; }
    if(c=='}') break;
    if(c=='#'){ MT::skipLine(is); continue; }
    e=new Element;
    e->read(is, *this);
    e->index=N;
    append(e);
    if(!is.good()){
      MT_MSG("reading failed at " <<e->index <<"th element");
      break;
    }
  }
}
