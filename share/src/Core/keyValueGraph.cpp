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


#include <map>

#include "util_t.h"
#include "array_t.h"
#include "keyValueGraph.h"
#include "registry.h"

const ItemL& NoItemL=*((ItemL*)NULL);

//===========================================================================
//
//  Item methods
//

Item::Item(KeyValueGraph& _container):container(_container){
  index=container.N;
  container.ItemL::append(this);
}

Item::Item(KeyValueGraph& _container, const ItemL& _parents)
  : container(_container), parents(_parents){
  index=container.N;
  container.ItemL::append(this);
  for(Item *i: parents) i->parentOf.append(this);
}

Item::~Item() {
  for(Item *i: parents) i->parentOf.removeValue(this);
  for(Item *i: parentOf) i->parents.removeValue(this);
  container.removeValue(this);
  for_list(Item, i, container) i->index=i_COUNT;
}

void Item::write(std::ostream& os) const {
  //-- write keys
  keys.write(os, " ", "", "\0\0");
  
  //-- write parents
  if(parents.N) {
    os <<" (";
    for_list(Item, it, parents) {
      if(it_COUNT) os <<' ';
      CHECK(it->keys.N,"");
      os <<it->keys.last();
    }
    os <<")";
  }
  
  //-- write value
  if(!hasValue()) return;
  if(getValueType()==typeid(KeyValueGraph)) {
    os <<" {";
    getValue<KeyValueGraph>()->write(os, " ");
    os <<" }";
  } else if(getValueType()==typeid(ItemL)) {
    os <<"=(";
    for(Item *it: (*getValue<ItemL>())) os <<' ' <<it->keys.last();
    os <<" )";
  } else if(getValueType()==typeid(MT::String)) {
    os <<"=\"" <<*getValue<MT::String>() <<'"';
  } else if(getValueType()==typeid(MT::FileToken)) {
    os <<"='" <<getValue<MT::FileToken>()->name <<'\'';
  } else if(getValueType()==typeid(arr)) {
    os <<'=' <<*getValue<arr>();
  } else if(getValueType()==typeid(double)) {
    os <<'=' <<*getValue<double>();
  } else if(getValueType()==typeid(bool)) {
    os <<'=' <<(*getValue<bool>()?"true":"false");
  } else {
    Item *it = reg_findType(getValueType().name());
    if(it && it->keys.N>1) {
      os <<" = <" <<it->keys(1) <<' ';
      writeValue(os);
      os <<'>';
    } else {
      os <<" = < ";
      writeValue(os);
      os <<'>';
    }
  }
}

KeyValueGraph Item::ParentOf(){
  KeyValueGraph G;
  G.ItemL::operator=(parentOf);
  G.isReference = true;
  return G;
}

Item *readItem(KeyValueGraph& containingKvg, std::istream& is, bool verbose=false, KeyValueGraph* parentGraph=NULL) {
  MT::String str;
  StringA keys;
  ItemL parents;
  Item *item=NULL;
  
  if(verbose) { cout <<"\nITEM (line="<<MT::lineCount <<")"; }
  
#define PARSERR(x) { cerr <<"[[error in parsing KeyValueGraph file (line=" <<MT::lineCount <<"):\n"\
                            <<"  item keys=" <<keys <<"\n  error=" <<x <<"]]"; is.clear(); }
  
  //-- read keys
  MT::skip(is," \t\n\r");
  for(;;) {
    if(!str.read(is, " \t", " \t\n\r,;([{}=", false)) break;
    keys.append(str);
  }
  //if(!keys.N) return false;
  
  if(verbose) { cout <<" keys:" <<keys <<flush; }
  
  //-- read parents
  char c=MT::getNextChar(is," \t"); //don't skip new lines
  if(c=='(') {
    for(uint j=0;; j++) {
      if(!str.read(is, " \t\n\r,", " \t\n\r,)", false)) break;
      Item *e=containingKvg.getItem(str);
      if(e) { //sucessfully found
        parents.append(e);
      } else { //this element is not known!!
        PARSERR("unknown " <<j <<". parent '" <<str <<"'");
        MT::skip(is, NULL, ")", false);
      }
    }
    MT::parse(is, ")");
    c=MT::getNextChar(is);
  }
  
  if(verbose) { cout <<" parents:"; if(!parents.N) cout <<"none"; else listWrite(parents,cout," ","()"); cout <<flush; }
  
  //-- read value
  if(c=='=' || c=='{' || c=='[' || c=='<' || c=='\'' || c=='"') {
    if(c=='=') c=MT::getNextChar(is);
    if((c>='a' && c<='z') || (c>='A' && c<='Z')) { //MT::String or boolean
      is.putback(c);
      str.read(is, "", " \n\r\t,;}", false);
      if(str=="true") item = new Item_typed<bool>(containingKvg, keys, parents, new bool(true));
      else if(str=="false") item = new Item_typed<bool>(containingKvg, keys, parents, new bool(false));
      else item = new Item_typed<MT::String>(containingKvg, keys, parents, new MT::String(str));
    } else if(MT::contains("-.0123456789", c)) {  //single double
      is.putback(c);
      double d;
      try { is >>d; } catch(...) PARSERR("can't parse double");
      item = new Item_typed<double>(containingKvg, keys, parents, new double(d));
    } else switch(c) {
        case '\'': { //MT::FileToken
          str.read(is, "", "\'", true);
          MT::FileToken *f = new MT::FileToken(str, false);
          try{
            f->getIs(); //creates the ifstream and might throw an error
            item = new Item_typed<MT::FileToken>(containingKvg, keys, parents, f);
          } catch(...){
            PARSERR("kvg indicates file which does not exist -> converting to string!");
            item = new Item_typed<MT::String>(containingKvg, keys, parents, new MT::String(str));
          }
        } break;
        case '\"': { //MT::String
          str.read(is, "", "\"", true);
          item = new Item_typed<MT::String>(containingKvg, keys, parents, new MT::String(str));
        } break;
        case '[': { //arr
          is.putback(c);
          arr reals;
          is >>reals;
          item = new Item_typed<arr>(containingKvg, keys, parents, new arr(reals));
        } break;
        case '<': { //any type parser
          str.read(is, " \t", " \t\n\r()`-=~!@#$%^&*()+[]{};'\\:|,./<>?", false);
//      str.read(is, " \t", " \t\n\r()`1234567890-=~!@#$%^&*()_+[]{};'\\:|,./<>?", false);
          item = readTypeIntoItem(containingKvg, str, is);
          if(!item) {
            is.clear();
            MT_MSG("could not parse value of type '" <<str <<"' -- no such type has been registered");
            str.read(is,"",">",false);
            MT_MSG("ignoring: '"<<str<<"'");
          } else {
            item->keys = keys;
            item->parents = parents;
          }
          MT::parse(is, ">");
        } break;
        case '{': { // KeyValueGraph (e.g., attribute list)
          KeyValueGraph *subList = new KeyValueGraph;
          subList->isItemOfParentKvg = &containingKvg;
          subList->read(is);
          MT::parse(is, "}");
          item = new Item_typed<KeyValueGraph>(containingKvg, keys, parents, subList);
        } break;
        case '(': { // referring KeyValueGraph
          KeyValueGraph refs;
          refs.isReference=true;
          for(uint j=0;; j++) {
            str.read(is, " , ", " , )", false);
            if(!str.N) break;
            Item *e=containingKvg.getItem(str);
            if(!e && parentGraph) e=parentGraph->getItem(str);
            if(e) { //sucessfully found
              refs.ItemL::append(e);
            } else { //this element is not known!!
              HALT("line:" <<MT::lineCount <<" reading item '" <<keys <<"': unknown "
                   <<j <<"th linked element '" <<str <<"'"); //DON'T DO THIS YET
            }
          }
          MT::parse(is, ")");
          item = new Item_typed<KeyValueGraph>(containingKvg, keys, parents, new KeyValueGraph(refs));
        } break;
        default: { //error
          is.putback(c);
          PARSERR("unknown value indicator '" <<c <<"'");
          return NULL;
        }
      }
  } else { //no '=' or '{' -> boolean
    is.putback(c);
    item = new Item_typed<bool>(containingKvg, keys, parents, new bool(true));
  }

#undef PARSERR

  if(verbose) {
    if(item) { cout <<" value:"; item->writeValue(cout); cout <<" FULL:"; item->write(cout); cout <<endl; }
    else { cout <<"FAILED" <<endl; }
  }
  
  if(item){
//    for(Item *it:item->parents) it->parentOf.append(item);
//    containingKvg.appendItem(item);
  }else {
    cout <<"FAILED reading item with keys ";
    keys.write(cout, " ", NULL, "()");
    cout <<" and parents ";
    listWrite(parents,cout," ","()");
    cout <<endl;
  }
  
  c=MT::getNextChar(is);
  if(c==',' || c==';') {} else is.putback(c);
  
  return item;
}


//===========================================================================
//
//  KeyValueGraph methods
//

struct sKeyValueGraph {
//  std::map<std::string, Item*> keyMap;
};

KeyValueGraph::KeyValueGraph():s(NULL), isReference(false), isItemOfParentKvg(NULL) {
  ItemL::memMove=true;
//  s = new sKeyValueGraph;
}

KeyValueGraph::~KeyValueGraph() {
//  delete s;
  if(!isReference) listDelete(*this);
}

Item *KeyValueGraph::append(const uintA& parentIdxs) {
  ItemL parents(parentIdxs.N);
  for(uint i=0;i<parentIdxs.N; i++) parents(i) = ItemL::elem(parentIdxs(i));
  return append<int>(STRINGS_1(ItemL::N), parents, NULL);
}


Item* KeyValueGraph::getItem(const char *key) {
  for(Item *it: (*this))
    for(const MT::String& k:it->keys) if(k==key) return it;
  if(isItemOfParentKvg) return isItemOfParentKvg->getItem(key);
  return NULL;
}

Item* KeyValueGraph::getItem(const char *key1, const char *key2) {
  for(Item *it: (*this)) {
    for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key1) {
        for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key2)
            return it;
      }
  }
  return NULL;
}

Item* KeyValueGraph::getItem(const StringA &keys) {
  bool found;
  for(Item *it: (*this)) {
    found = true;
    for(uint k = 0; k < keys.N && found; k++) {
      found = false;
      for(const String &key: it->keys) {
        if(keys(k) == key) {
          found = true;
          break;
        }
      }
    }
    if(found) return it;
  }
  return NULL;
}

KeyValueGraph KeyValueGraph::getItems(const char* key) {
  KeyValueGraph ret;
  ret.isReference = true;
  for(Item *it: (*this)) {
    for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key) { ret.ItemL::append(it); break; }
  }
  return ret;
}

KeyValueGraph KeyValueGraph::getTypedItems(const char* key, const std::type_info& type) {
  KeyValueGraph ret;
  ret.isReference = true;
  for(Item *it: (*this)) if(it->getValueType()==type) {
    if(!key) ret.appendItem(it);
    else for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key) {
      ret.ItemL::append(it);
      break;
    }
  }
  return ret;
}

Item* KeyValueGraph::merge(Item *m){
  KeyValueGraph KVG = getTypedItems(m->keys(0), m->getValueType());
  //CHECK(KVG.N<=1, "can't merge into multiple items yet");
  Item *it=NULL;
  if(KVG.N) it=KVG.elem(0);
  if(it){
    CHECK(m->getValueType()==it->getValueType(), "can't merge items of different types!");
    if(it->getValueType()==typeid(KeyValueGraph)){ //merge the KVGs
      it->getValue<KeyValueGraph>()->merge(*m->getValue<KeyValueGraph>());
    }else{ //overwrite the value
      it->takeoverValue(m);
    }
    if(&m->container==this) removeItem(m);
  }else{ //nothing to merge, append
    if(&m->container!=this){
      Item *it = m->newClone(*this);
      for(uint i=0;i<it->parents.N;i++){
        it->parents(i) = elem(it->parents(i)->index);
        it->parents(i)->parentOf.append(it);
      }
    }
    return m;
  }
  return NULL;
}

KeyValueGraph& KeyValueGraph::operator=(const KeyValueGraph& G) {
  if(!isReference) listDelete(*this);
  { for_list(Item, i, G) i->index=i_COUNT; }
  for(Item *it:G) it->newClone(*this);
  //rewire links
  for(Item *it:*this) it->parentOf.clear();
  for(Item *it:*this){
    for(uint i=0;i<it->parents.N;i++){
      it->parents(i) = elem(it->parents(i)->index);
      it->parents(i)->parentOf.append(it);
    }
  }
  return *this;
}

void KeyValueGraph::read(std::istream& is) {
  //read all generic attributes
  //MT::lineCount=1;
  for(;;) {
    char c=MT::peerNextChar(is, " \n\r\t,");
    if(c=='%'){ //special caracter
      MT::String str;
      str.read(is,""," \n\r\t",true);
      if(str=="%include"){
        is >>str;
        read(FILE(str).getIs());
      }else HALT("don't know special command " <<str);
    }else{
      if(!is.good() || c=='}') { is.clear(); break; }
      Item *it = readItem(*this, is);
      if(!it) break;
      if(it->keys.N && it->keys(0)=="Include"){
        read(it->getValue<MT::FileToken>()->getIs());
        ItemL::removeValue(it);
      }
    }
  }
  //-- merge all Merge keys
  KeyValueGraph merges = getItems("Merge");
  for(Item *m:merges){
    m->keys.remove(0);
    merge(m);
  }
}

void KeyValueGraph::write(std::ostream& os, const char *ELEMSEP, const char *delim) const {
  uint i;
  if(delim) os <<delim[0];
  for(i=0; i<N; i++) { if(i) os <<ELEMSEP;  if(elem(i)) os <<*elem(i) <<flush; else os <<"<NULL>"; }
  if(delim) os <<delim[1] <<std::flush;
}

void KeyValueGraph::writeDot(std::ostream& os, bool withoutHeader) {
  if(!withoutHeader){
    os <<"graph G{" <<endl;
    os <<"graph [ rankdir=\"TD\", ranksep=0.05 ];" <<endl;
    os <<"node [ fontsize=9, width=.3, height=.3 ];" <<endl;
    os <<"edge [ arrowtail=dot, arrowsize=.5, fontsize=6 ];" <<endl;
  }
  for_list(Item, i, list()) i->index = i_COUNT;
  for(Item *it: list()) {
    if(it->parents.N==2 && it->getValueType()==typeid(bool)){ //an edge
      os <<it->parents(0)->index <<" -- " <<it->parents(1)->index <<" [ ";
      if(it->keys.N){
        os <<"label=\"" <<it->keys(0);
        for(uint i=1;i<it->keys.N;i++) os <<'_' <<it->keys(i);
        os <<"\" ";
      }
      os <<"];" <<endl;
    }else{
      os <<it->index <<" [ ";
      if(it->keys.N){
        os <<"label=\"" <<it->keys(0);
        for(uint i=1;i<it->keys.N;i++) os <<'\n' <<it->keys(i);
        os <<"\" ";
      }
      if(it->parents.N) os <<"shape=box";
      else os <<"shape=ellipse";
      os <<" ];" <<endl;
      for_list(Item, pa, it->parents) {
        if(pa->index<it->index)
          os <<pa->index <<" -- " <<it->index <<" [ ";
        else
          os <<it->index <<" -- " <<pa->index <<" [ ";
        os <<"label=" <<pa_COUNT;
        os <<" ];" <<endl;
      }
    }
    if(it->getValueType()==typeid(KeyValueGraph)){
      it->getValue<KeyValueGraph>()->writeDot(os, true);
    }
  }
  if(!withoutHeader){
    os <<"}" <<endl;
    HALT("TODO: counter offset to index items dotlike...")
  }
}

void KeyValueGraph::sortByDotOrder() {
  uintA perm;
  perm.setStraightPerm(N);
  for_list(Item, it, list()) {
    if(it->getValueType()==typeid(KeyValueGraph)) {
      double *order = it->getValue<KeyValueGraph>()->getValue<double>("dot_order");
      if(!order) { MT_MSG("doesn't have dot_order attribute"); return; }
      perm(it_COUNT) = (uint)*order;
    }
  }
  permuteInv(perm);
  for_list(Item, it2, list()) it2->index=it2_COUNT;
}

