struct Any;
typedef MT::Array<Any*>   AnyList;

//-- AnyLists
void anyListRead(AnyList& ats, std::istream& is);
template<class T> T* anyListGet(const AnyList& L, const char *tag, uint n);
//template<class T> MT::Array<T> get(const AnyList& L, const char* tag); //TODO obsolete?


//===========================================================================
//
// generic any container
//

struct Any {
  char *tag;
  const char *type;
  void *p;
  uint n;
  char delim;
  virtual ~Any() {};
  virtual void write(std::ostream &os) const = 0;
  virtual Any *newClone() = 0;
};
stdOutPipe(Any)

template<class T> Any* anyNew(const char* tag, const T &x);
template<class T> Any* anyNew(const char* tag, const T *x, uint n, char delim);




//===========================================================================
//
// generic any container
//

//this is a typed instance of the general Any struct
template<class T> struct Any_typed:public Any {
  virtual ~Any_typed() { free(); };
  Any_typed(const char* _tag, const T &x) {                      tag=NULL; p=NULL; set(_tag, &x, 0, 0);  }
  Any_typed(const char* _tag, const T *_p, uint _n, char _delim) { tag=NULL; p=NULL; set(_tag, _p, _n, _delim); }
  virtual void write(std::ostream &os) const {
    if(!p) { os <<tag; return; }  //boolean
    os <<tag <<"[" <<type <<"] = ";
    if(!n) {
      if(typeid(T)==typeid(const char*) || typeid(T)==typeid(char*) || typeid(T)==typeid(MT::String)) os <<'\'' <<*((T*)p) <<'\'';
      else os <<*((T*)p);
    } else {
      T *t=(T*)p;
      os <<delim <<t[0];
      for(uint i=1; i<n; i++) os <<' ' <<t[i];
      if(delim=='(') os <<')';
      else if(delim=='[') os <<']';
      else if(delim=='{') os <<'}';
      else os <<delim;
    }
  }
  virtual void free() {
    if(!tag) { CHECK(!p, ""); return; }
    delete[] tag;
    if(!p) return;
    if(!n) delete((T*)p);
    else   delete[]((T*)p);
    p=NULL;
  }
  virtual void set(const char* _tag, const T *_p, uint _n, char _delim) {
    free();
    type=typeid(T).name();
    tag=new char[strlen(_tag)+1];
    strcpy(tag, _tag);
    if(!_p) { p=NULL; n=0; delim=0; return; }  //assume this is a ``boolean tag'' without data
    n=_n;
    delim=_delim;
    if(!n) {
      p = new T(_p[0]);
    } else {
      p = new T[n];
      T *t=(T*)p;
      for(uint i=0; i<n; i++) t[i]=_p[i];
    }
  }
  virtual Any* newClone() { return new Any_typed<T>(tag, (T*)p, n, delim); }
};

template<class T> Any* anyNew(const char* tag, const T &x) {        return new Any_typed<T>(tag, x); }
template<class T> Any* anyNew(const char* tag, const T *x, uint n, char delim) { return new Any_typed<T>(tag, x, n, delim); }


template<class T> T* anyListGet(const AnyList& L, const char *tag, uint n) {
  uint i;
  for(i=0; i<L.N; i++) {
    if(!strcmp(tag, L(i)->tag)) {
      if(strcmp(typeid(T).name(), L(i)->type)) {
        HALT("ABORT GETTING ATTRIBUTE -- found tag (" <<tag <<") but with different type (" <<typeid(T).name() <<") than requested (" <<L(i)->type <<")");
        return NULL;
      }
      if(n>1 && n!=L(i)->n) {
        HALT("ABORT GETTING ATTRIBUTE -- found tag (" <<tag <<") but with different size (" <<L(i)->n <<") than requested (" <<n <<")");
        return NULL;
      }
      if(n==1 && !L(i)->p) {
        HALT("ABORT GETTING ATTRIBUTE -- found tag (" <<tag <<") but as boolean flag instead of size 1)");
        return NULL;
      }
      if(n==0 && L(i)->p) {
        HALT("ABORT GETTING ATTRIBUTE -- found tag (" <<tag <<") but with size (" <<L(i)->n <<") instead fo boolean flag)");
        return NULL;
      }
      if(!L(i)->p) return (T*)1;  //boolean return - tag found but no data
      return (T*)(L(i)->p);
    }
  }
  //HALT("ABORT GETTING ATTRIBUTE -- couldn't find tag (" <<tag <<") in anylist");
  return NULL;
}

/*#include <MT/array.h>
void testAny(){
  AnyList bag;
  bag.append(anyNew<double>("bla0",.125));
  bag.append(anyNew<const char*>("bla1","blublu"));
  bag.append(anyNew<double>("bla2",ARR(.1,.2,3.,.4).p,4,'['));

  listWrite(bag,cout); cout <<endl;
  AnyList B;
  listClone(B,bag);
  listWrite(B,cout); cout <<endl;
  listDelete(B);

  MT::String buf;
  listWrite(bag,buf);
  cout <<buf <<endl;
  anyListRead(B,buf);
  listWrite(B,cout); cout <<endl;

  listDelete(bag);
  listDelete(B);
}*/

template MT::Array<Any*>::Array();
template MT::Array<Any*>::~Array();

//===========================================================================
//
// lists
//

void anyListRead(AnyList& ats, std::istream& is) {
  char c, delim;
  MT::String tag, str;
  double d;
  arr reals;
  MT::Array<MT::String> strings;

  //read all generic attributes
  for(;;) {
    tag.read(is, " \t\r\n", " \t=}, ;([\n\r", false);
    if(!tag.N) {
      MT::skip(is, " \t\r\n;");
      is.clear();
      break;
    }
    MT::skip(is);  is.get(c);
    if(c=='=') { MT::skip(is); is.get(c); }
    switch(c) {
      case '(': { //vector of strings
        delim=c;
        strings.clear();
        for(;;) {
          MT::skip(is);
          is.get(c);
          if(c==')') break; else is.putback(c);
          str.read(is, "", "), \t\r\n", false);
          strings.append(str);
        }
        if(strings.N==1) {  //not nice - one should clearly distinguish between a vector and scalar...
          ats.append(anyNew<MT::String>(tag, strings(0)));
        } else {
          ats.append(anyNew<MT::String>(tag, strings.p, strings.N, delim));
        }
      } break;
      case '[': { //vector of reals
        delim=c;
        reals.clear();
        for(;;) {
          MT::skip(is);
          is.get(c);
          if(c==']' || c==')') break; else is.putback(c);
          is >>d;
          reals.append(d);
          if(!is.good()) HALT("ERROR");
        }
        if(reals.N==1) {  //not nice - one should clearly distinguish between a vector and scalar...
          ats.append(anyNew<double>(tag, reals(0)));
        } else {
          ats.append(anyNew<double>(tag, reals.p, reals.N, delim));
        }
      } break;
      case '\'': { //string
        str.read(is, "", "\'", true);
        ats.append(anyNew<MT::String>(tag, str));
      } break;
      case '\"': { //string
        str.read(is, "", "\"", true);
        ats.append(anyNew<MT::String>(tag, str));
      } break;
      case '<': { //string
        str.read(is, "", ">", true);
        ats.append(anyNew<MT::String>(tag, str));
      } break;
      default: { //single double or nothing
        is.putback(c);
        if(MT::contains("-.0123456789", c)) {  //single double
          is >>d;
          ats.append(anyNew<double>(tag, d));
        } else { //bool
          ats.append(anyNew<double>(tag, (double*)0, 0, 0));
        }
      } break;
    }
    MT::skip(is, " \n\t, ");
  }
}
