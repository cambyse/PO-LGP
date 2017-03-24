#include <Core/util.h>

class Engine;

template<class T>
void provideReadAccess(Engine&, Access<T>&);

template<class T>
struct Access{
  struct ReadToken{
    Access<T> *a;
    ReadToken(Access<T> *_a):a(_a){ a->readAccess(); }
    ~ReadToken(){ a->deAccess(); }
    const T& operator()(){ return *a->data; }
  };
  struct WriteToken{
    Access<T> *a;
    WriteToken(Access<T> *_a):a(_a){ a->writeAccess(); }
    ~WriteToken(){ a->deAccess(); }
    T& operator()(){ return *a->data; }
  };

  T *data;

  Access(const char* name=NULL):Access(name), data(NULL){}
  const T& get(){ return ReadToken(this)(); }
  T& set(){ return WriteToken(this)(); }
  T& operator()(){ CHECK_EQ(variable->rwlock.state,-1,"");  return *data; } //TODO ensure that it is locked
  virtual void* createOwnData();
  virtual void setData(void* _data);
};

struct Module{
  AccessL accesses;

  virtual Module() = 0;
  virtual ~Module() = 0;
  virtual void step();
  virtual bool test(){ return true; }

  virtual char* ModuleKey() = 0;
  virtual char* ModuleDescription(){ return NULL; }
};


struct MyModule : Thread {
  Access<arr> input;
  Access<double> output;
  MyModule():input(this), output(this){}
  ~MyModule(){}
  void step(){ ouput = sum(input()); }
};
