#include "hypergraph.h"

void Element::write(std::ostream& os) const {
  uint i;  Element *e;
  if(type.N) os <<type <<' ';
  if(name.N) os <<name <<' '; else os <<id <<' ';
  if(links.N){
    os <<" (";
    for_list(i, e, links){
      if(i) os <<' ';
      if(e->name.N) os <<e->name; else os <<e->id;
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
        links.append(e);
        linksIds.append(e->id);
        e->elemof[j].append(this);
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
          e2->id=list.N;
          list.append(e2);
          links.append(e2);
          linksIds.append(e2->id);
          e2->elemof[j].append(this);
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
  return elem(i)->elemof[0];
}

Element *HyperGraph::add(const uintA& tuple){
  CHECK(tuple.N<maxDegree, "");
  uint i;
  Element *e = new Element;
  if(unused.N){
    e->id=unused.popLast();
    elem(e->id)=e;
    
  }else{
    e->id=N;
    append(e); }
  e->linksIds=tuple;
  //N[tuple.N]++;
  for(i=0; i<tuple.N; i++){
    elem(tuple(i))->elemof[i].append(e);
    e->links.append(elem(tuple(i)));
  }
  return e;
}

void HyperGraph::del(Element *e){
  CHECK(elem(e->id)==e, "this is not an element of the hypergraph")
  uint i;
  for(i=0; i<maxDegree; i++){
    if(e->elemof[i].N){
      HALT("can't delete element " <<e->id <<" - it is containted in others:\n");
      e->write(cerr);
    }
  }
  //N[e->links.N]--;
  unused.append(e->id);
  elem(e->id) = NULL;
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
    e->id=N;
    append(e);
    if(!is.good()){
      MT_MSG("reading failed at " <<e->id <<"th element");
      break;
    }
  }
}
