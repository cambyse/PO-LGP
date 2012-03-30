#ifndef MT_biros_h
#define MT_biros_h

#include <MT/array.h>
#include <MT/util.h>

//===========================================================================
//
// forward declarations
//

struct Variable;
struct Process;
struct Parameter;

typedef MT::Array<Variable*> VariableL;
typedef MT::Array<Process*> ProcessL;
typedef MT::Array<Parameter*> ParameterL;

#define PROCESS(name) \
struct name:Process { \
  struct s##name *s;  \
  name();             \
  virtual ~name();    \
  void open();        \
  void step();        \
  void close();       \
};


//===========================================================================
//
// automatic setters and getters and info for Variable fields
//

struct FieldInfo{
  void *p;
  const char* name;
  const char* userType;
  const char* sysType;
  virtual void write_value(ostream& os) const = 0;
};
  
template<class T>
struct FieldInfo_typed:FieldInfo{
  FieldInfo_typed(T *_p, const char* _name, const char* _userType){
    p = _p;
    name = _name;
    userType = _userType;
    sysType = typeid(T).name();
  }
  void write_value(ostream& os) const{ os <<*((T*)p); }
};

#define FIELD(type, name) \
  type name; \
  inline void set_##name(const type& _x, Process *p){ \
    writeAccess(p);  name=(type&)_x;  deAccess(p); } \
  inline void get_##name(type& _x, Process *p){ \
    readAccess(p);   _x=name;  deAccess(p); } \
  inline type get_##name(Process *p){ \
    type _x; readAccess(p); _x=name; deAccess(p);  return _x;  } \
  inline void reg_##name(){ \
    fields.append(new FieldInfo_typed<type>(&name,#name,#type)); }


//===========================================================================
//
// Variable
//

struct Variable {
  struct sVariable *s;  ///< private
  uint id;              ///< unique identifyer
  const char* name;     ///< Variable name
  uint revision;        ///< revision (= number of write accesses) number
  MT::Array<FieldInfo*> fields;
  ProcessL listeners;

  Variable(const char* name);
  ~Variable();
  
  //-- to be overloaded by the specific implementation
  //virtual void write(ostream& os){ os <<name; }
  //virtual void read(istream& is){ }
  
  //-- access control, to be called by a processes before access
  void readAccess(Process*);  //might set the caller to sleep
  void writeAccess(Process*); //might set the caller to sleep
  void deAccess(Process*);

  //-- info
  int lockState(); // 0=no lock, -1=write access, positive=#readers
  uint get_revision(){ readAccess(NULL); uint r = revision; deAccess(NULL); return r; }
};


//===========================================================================
//
// Process
//

struct Process {
  struct sProcess *s;   ///< private
  uint id;              ///< unique identifyer
  const char* name;     ///< Process name
  
  Process(const char* name);
  virtual ~Process();
  
  //-- to be overloaded by the specific implementation
  virtual void open() = 0;    ///< is called within the thread when the thread is created
  virtual void close() = 0;   ///< is called within the thread when the thread is destroyed
  virtual void step() = 0;    ///< is called within the thread when trigerring a step from outside (or when permanently looping)
  
  //-- a scalar function which may depend only on the referenced variables
  //   -- code correctness requires that a call of _step() may only decrease _f() !!
  //virtual double _f(){ return 0.; }
  
  //-- to be called from `outside' (e.g. the main) to start/step/close the thread
  void threadOpen(int priority=0);      ///< start the thread (in idle mode) (should be positive for changes)
  void threadClose();                   ///< close the thread (stops looping and waits for idle mode before joining the thread)
  
  void threadStep(uint steps=1, bool wait=false);     ///< trigger (multiple) step (idle -> working mode) (wait until idle? otherwise calling during non-idle -> error)
  void threadStepOrSkip(uint maxSkips); ///< trigger a step (idle -> working mode) or skip if still busy (counts skips.., maxSkip=0 -> no warnings)
  void threadWait();                    ///< wait until step is done (working -> idle mode)
  bool threadIsIdle();                  ///< check if in idle mode

  void threadListenTo(Variable *var);
  void threadListenTo(const VariableL &signalingVars);

  void threadLoop();                    ///< loop, stepping forever
  void threadLoopWithBeat(double sec);  ///< loop with a fixed beat (cycle time)
  void threadStop();                    ///< stop looping
};


//===========================================================================
//
// Parameters
// (these are usually not created directly by the user,
//  they are created automatically by a call of `getParameter')
//

struct Parameter{
  uint id;              ///< unique identifyer
  void *pvalue;
  const char* name;
  ProcessL processes;
  Parameter();
  virtual void writeValue(ostream& os) const = 0;
  virtual const char* typeName() const = 0;
};
  
template<class T>
struct Parameter_typed:Parameter{
  T value;
  Parameter_typed(const char* _name, const T& _default):Parameter(){
    name = _name;
    pvalue = &value;
    if(&_default) MT::getParameter<T>(value, name, _default);
    else          MT::getParameter<T>(value, name);
  }
  void writeValue(ostream& os) const{ os <<value; }
  const char* typeName() const{ return typeid(T).name(); }
};


//===========================================================================
//
// basic access to global system info
//

struct BirosInfo:Variable{
  VariableL variables;
  ProcessL processes;
  ParameterL parameters;
  Process *getProcessFromPID();
  BirosInfo():Variable("GlobalInfo"){ }
  template<class T>  void getVariable(T*& v, const char* name, Process *p){
    writeAccess(p);
    v = (T*)listFindByName(variables, name);
    deAccess(p);
    if(!v) MT_MSG("can't find biros variable '" <<name <<"' -- Process '" <<(p?p->name:"NULL") <<"' will not connect");
  }
  template<class T>  T* getProcess(const char* name, Process *p){
    writeAccess(p);
    T *pname = (T*)listFindByName(processes, name);
    deAccess(p);
    if(!pname) MT_MSG("can't find biros process '" <<name <<"' -- Process '" <<(p?p->name:"NULL") <<"' will not connect");
  }
  template<class T> T getParameter(const char *name, Process *p, const T *_default=NULL){
    Parameter_typed<T> *par;
    writeAccess(p);
    par = (Parameter_typed<T>*)listFindByName(parameters, name);
    deAccess(p);
    if(!par) par = new Parameter_typed<T>(name, *_default);
    if(!par->processes.contains(p)) par->processes.append(p);
    return par->value;
  }
  template<class T> T getParameter(const char *name){
    return getParameter<T>(name, getProcessFromPID(), NULL);
  }
  template<class T> T getParameter(const char *name, const T& _default){
    return getParameter<T>(name, getProcessFromPID(), &_default);
  }
  void dump(); //dump everything -- for debugging
};

extern BirosInfo birosInfo;


//===========================================================================
//
// WorkingCopy
//

template<class T>
struct WorkingCopy{
  T *var;             ///< pointer to the Variable (T must be derived from Variable)
  T copy;
  Process *p;         ///< pointer to the Process that might want to access the Variable
  uint last_revision; ///< last revision of a read/write access
  
  WorkingCopy(){ p=NULL; var=NULL; last_revision = 0;  }
  T& operator()(){ return copy; }
  
  void init(T *_v, Process *_p){
    p=_p;
    var=_v;
    var->readAccess(p);
    copy = *var;
    last_revision = var->revision;
    var->deAccess(p);
  }
  void init(const char* var_name, Process *_p){
    T *_v;
    birosInfo.getVariable(_v, "GeometricState", _p);
    init(_v, _p);
  }
  bool needsUpdate(){
    return last_revision != var->get_revision();
  }
  void push(){
    if(var->get_revision()>last_revision) MT_MSG("Warning: push overwrites revision");
    var->writeAccess(p);
    *var = copy;
    last_revision = var->revision; //(was incremented already on writeAccess)
    var->deAccess(p);
  }
  void pull(){
    if(last_revision == var->get_revision()) return;
    var->readAccess(p);
    copy = *var;
    last_revision = var->revision;
    var->deAccess(p);
  }
};


//===========================================================================
//
// very basic low-level X11 Monitor
//

struct ThreadInfoWin:public Process {
  struct sThreadInfoWin *s;
  
  ThreadInfoWin();
  ~ThreadInfoWin();
  
  void open();
  void close();
  void step();
};


//===========================================================================
//
// handling groups
//

void step(const ProcessL& P);
void loop(const ProcessL& P);
void loopWithBeat(const ProcessL& P, double sec);
void stop(const ProcessL& P);
void wait(const ProcessL& P);
void close(const ProcessL& P);


#ifdef  MT_IMPLEMENTATION
#  include "biros.cpp"
#endif

#endif
