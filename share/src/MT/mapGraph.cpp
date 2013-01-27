#include <map>

#include "mapGraph.h"
#include "registry.h"

/*struct Parser{
  virtual ~Parser(){};
  virtual Item* readItem (std::istream& is) = 0;
};

struct TransformationParser:Parser{
  Item* readItem(std::istream& is){
    ors::Transformation t;
    is >>t;
    return new Item_typed<ors::Transformation>(t);
  }
};*/

//===========================================================================
//
//  Item methods
//

void Item::write(std::ostream& os) const {
  //-- write keys
  keys.write(os, " ", "", "\0\0");

  //-- write parents
  if(parents.N){
    os <<" (";
    for_list_(Item, it, parents){
      if(it_COUNT) os <<' ';
      CHECK(it->keys.N,"");
      os <<it->keys.last();
    }
    os <<")";
  }

  //-- write value
  if(valueType()==typeid(MapGraph)){
    os <<" {";
    value<MapGraph>().write(os, ",");
    os <<" }";
  }else if(valueType()==typeid(MT::String)){ os <<"='" <<value<MT::String>() <<'\'';
  }else if(valueType()==typeid(arr)){        os <<'=' <<value<arr>();
  }else if(valueType()==typeid(double)){     os <<'=' <<value<double>();
  }else if(valueType()==typeid(bool)){       os <<',';
  }else{
    TypeRegistration *t = reg_find(valueType().name());
    if(t && t->keys.N>1){
      os <<"=<" <<t->keys(1) <<' ';
      writeValue(os);
      os <<'>';
    }else{
      os <<"=<" <<valueType().name() <<' ';
      writeValue(os);
      os <<'>';
    }
  }
}

bool readItem(MapGraph& list, std::istream& is, bool verbose=true){
  MT::String str;
  StringA keys;
  ItemL parents;
  Item *item=NULL;

  if(verbose){ cout <<"\nITEM (line="<<MT::lineCount <<")"; }

#define PARSERR(x) { cout <<"[[error in parsing MapGraph file (line=" <<MT::lineCount <<"):\n"\
  <<"  item keys=" <<keys <<"\n  error=" <<x <<"]]"; is.clear(); return false; }

  //-- read keys
  for(;;){
    if(!str.read(is, " \t\n\r", " \t\n\r,;({=", false)) break;
    keys.append(str);
  }
  if(!keys.N) return false;

  if(verbose){ cout <<" keys:" <<keys <<flush; }

  //-- read parents
  char c=MT::getNextChar(is);
  if(c=='('){
    for(uint j=0;;j++){
      if(!str.read(is, " \t\n\r,", " \t\n\r,)", false)) break;
      Item *e=list.getItem(str);
      if(e){ //sucessfully found
        parents.append(e);
        e->parentOf.append(list);
      }else{//this element is not known!!
        PARSERR("unknown " <<j <<". parent '" <<str <<"'");
      }
    }
    MT::parse(is, ")");
    c=MT::getNextChar(is);
  }

  if(verbose){ cout <<" parents:"; if(!parents.N) cout <<"none"; else listWrite(parents,cout," ","()"); cout <<flush; }

  //-- read value
  if(c==',' || c==';'){ //boolean
    item = new Item_typed<bool>(keys, parents, true);
  }else if(c=='=' || c=='{'){
    if(c=='=') c=MT::getNextChar(is);
    if((c>='a' && c<='z') || (c>='A' && c<='Z')) { //MT::String
      is.putback(c);
      str.read(is, "", " \n\r\t,;}", false);
      item = new Item_typed<MT::String>(keys, parents, str);
    }else if(MT::contains("-.0123456789", c)) {  //single double
      is.putback(c);
      double d;
      try { is >>d; } catch(...) PARSERR("can't parse double");
      item = new Item_typed<double>(keys, parents, d);
    }else switch(c) {
    case '\'': { //MT::String
      str.read(is, "", "\'", true);
      item = new Item_typed<MT::String>(keys, parents, str);
    } break;
    case '\"': { //MT::String
      str.read(is, "", "\"", true);
      item = new Item_typed<MT::String>(keys, parents, str);
    } break;
    case '[': { //arr
      is.putback(c);
      arr reals;
      is >>reals;
      item = new Item_typed<arr>(keys, parents, reals);
    } break;
    case '<': { //any type parser
      str.read(is, " \t", " \t\n\r()`-=~!@#$%^&*()+[]{};'\\:|,./<>?", false);
//      str.read(is, " \t", " \t\n\r()`1234567890-=~!@#$%^&*()_+[]{};'\\:|,./<>?", false);
      item = readTypeIntoItem(str,is);
      if(!item){
        is.clear();
        MT_MSG("could not parse value of type '" <<str <<"' -- no such type has been registered");
        str.read(is,"",">",false);
        MT_MSG("ignoring: '"<<str<<"'");
      }else{
        item->keys = keys;
        item->parents = parents;
      }
      MT::parse(is, ">");
    } break;
    case '{': { // MapGraph (e.g., attribute list)
      MapGraph subList;
      subList.read(is);
      MT::parse(is, "}");
      item = new Item_typed<MapGraph>(keys, parents, subList);
    } break;
    case '(': { // ItemL
      ItemL refs;
      for(uint j=0;;j++){
        str.read(is, " , ", " , )", false);
        if(!str.N) break;
        Item *e=list.getItem(str);
        if(e){ //sucessfully found
          refs.append(e);
        }else{ //this element is not known!!
          HALT("line:" <<MT::lineCount <<" reading item '" <<keys <<"': unknown "
               <<j <<"th linked element '" <<str <<"'"); //DON'T DO THIS YET
        }
      }
      MT::parse(is, ")");
      item = new Item_typed<ItemL>(keys, parents, refs);
    } break;
    default: { //boolean
      is.putback(c);
      PARSERR("unknown value indicator '" <<c <<"' (booleans need comma or semicolon)");
    }
    }
  }
  if(verbose){
    if(item){ cout <<" value:"; item->writeValue(cout); cout <<" FULL:"; item->write(cout); cout <<endl; }
    else{ cout <<"FAILED" <<endl; }
  }

  if(item) list.ItemL::append(item);
  else{
    cout <<"FAILED reading item with keys ";
    keys.write(cout, " ", NULL, "()");
    cout <<" and parents ";
    listWrite(parents,cout," ","()");
    cout <<endl;
  }

  c=MT::getNextChar(is);
  if(c==',' || c==';'){}else is.putback(c);

  return true;
}


//===========================================================================
//
//  MapGraph methods
//

struct sMapGraph{
  std::map<std::string, Item*> keyMap;
};

MapGraph::MapGraph():s(NULL){
  ItemL::memMove=true;
  s = new sMapGraph;
}

MapGraph::~MapGraph(){
  delete s;
}

Item* MapGraph::getItem(const char* key){
  uint i;
  for_list_(Item, it, (*this))
    for(i=0;i<it->keys.N;i++) if(it->keys(i)==key) return it;
  return NULL;
}

ItemL MapGraph::getItems(const char* key){
  uint i;
  ItemL ret;
  for_list_(Item, it, (*this)){
    for(i=0;i<it->keys.N;i++) if(it->keys(i)==key){ ret.append(it); break; }
  }
  return ret;
}

void sortByDotOrder(ItemL& G){
  NIY;
#if 0
  uintA perm(G.N);
  Item *e;
  double *order;
  uint i;
  for_list(i,e,G){
    order = anyListGet<double>(e->ats, "dot_order", 1);
    if(!order){ MT_MSG("doesn't have dot_order attribute"); return; }
    perm(i) = (uint)*order;
  }
  G.permuteInv(perm);
  for_list(i,e,G) e->index=i;
#endif
}

void writeDot(ItemL& G){
  NIY;
#if 0
  ofstream fil;
  MT::open(fil, "z.dot");
  fil <<"graph G{" <<endl;
  fil <<"graph [ rankdir=\"LR\", ranksep=0.05 ];" <<endl;
  fil <<"node [ fontsize=9, width=.3, height=.3 ];" <<endl;
  fil <<"edge [ arrowtail=dot, arrowsize=.5, fontsize=6 ];" <<endl;
  uint i, j;
  Item *e, *n;
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
#endif
}

MapGraph& MapGraph::operator=(const MapGraph& G){
  listDelete(*this);
  this->resize(G.N);
  uint i;
  for(i=0; i<G.N; i++) elem(i)=G.elem(i)->newClone();
  return *this;
}

void MapGraph::read(std::istream& is) {
  //read all generic attributes
  //MT::lineCount=1;
  for(;;) {
    char c=MT::peerNextChar(is, " \n\r\t,");
    if(!is.good() || c=='}'){ is.clear(); break; }
    if(!readItem(*this, is)) break;
  }
}

void MapGraph::write(std::ostream& os, const char *ELEMSEP, const char *delim) const{
  uint i;
  if(delim) os <<delim[0];
  for(i=0; i<N; i++) { if(i) os <<ELEMSEP;  if(elem(i)) os <<*elem(i); else os <<"<NULL>"; }
  if(delim) os <<delim[1] <<std::flush;
}

void MapGraph::writeDot(std::ostream& os){
}

Item *MapGraph::add(const uintA& tuple){
  NIY;
}

ItemL& MapGraph::getParents(uint i){
  NIY;
}

void MapGraph::sortByDotOrder(){
  NIY;
}