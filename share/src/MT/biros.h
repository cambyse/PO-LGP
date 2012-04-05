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

//===========================================================================
//
// automatic setters and getters and info for Variable fields
//

struct _Variable_field_info_base {
  void *p;
  const char* name;
  virtual void write_value(ostream& os) const = 0;
  virtual void read_value(istream& os) const = 0;
  virtual MT::String type() const = 0;
};

template<class T>
struct _Variable_field_info:_Variable_field_info_base {
  _Variable_field_info(T *_p, const char* _name) { p=_p; name=_name; }
  void write_value(ostream& os) const { os <<*((T*)p); }
  void read_value(istream& is) const { is >> *((T*)p); }
  MT::String type() const { MT::String s(typeid(T).name()); return s; }
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
    fields.append(new _Variable_field_info<type>(&name,#name)); }


//===========================================================================
//
// Variable
//

struct Variable {
  struct sVariable *s;  ///< private
  uint id;              ///< unique identifyer
  MT::String name;     ///< Variable name
  volatile uint revision;        ///< revision (= number of write accesses) number //TODO: why volatile?
  MT::Array<_Variable_field_info_base*> fields;
  bool logValues;
  bool dbDrivenReplay;
  pthread_mutex_t replay_mutex; //TODO: move to sVariable! (that's the point of private..)
  
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
  
  //-- condition variable control, to be called from processes to broadcast (publish) or wait for broadcast (subscribe)
  void broadcastCondition(int i=0);
  int  getCondition();
  void waitForConditionSignal(double seconds=-1.);
  void waitForConditionEq(int i);    //might set the caller to sleep
  void waitForConditionNotEq(int i); //might set the caller to sleep
  
  virtual void serializeToString(MT::String &string) const;
  virtual void deSerializeFromString(const MT::String &string);
};


//===========================================================================
//
// Process
//

struct Process {
  struct sProcess *s;  ///< private
  int id;              ///< unique identifier
  uint step_count;     ///< step count
  const char* name;    ///< Process name
  
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
  
  void threadStepOrSkip(uint maxSkips); ///< trigger a step (idle -> working mode) or skip if still busy (counts skips.., maxSkip=0 -> no warnings)
  void threadStep(bool wait=false);     ///< trigger a step (idle -> working mode) (wait until idle? otherwise calling during non-idle -> error)
  void threadSteps(uint steps);         ///< trigger multiple steps (idle -> working mode)
  void threadWait();                    ///< caller waits until step is done (working -> idle mode)
  bool threadIsIdle();                  ///< check if in idle mode
  bool threadIsClosed();                ///< check if closed
  
  void threadLoop();                    ///< loop, stepping forever
  void threadLoopWithBeat(double sec);  ///< loop with a fixed beat (cycle time)
  void threadLoopSyncWithDone(Process& p); ///< loop in sync with another process
  void threadStop();                    ///< stop looping
};

//===========================================================================
//
// Parameters
// (these are usually not created directly by the user,
//  they are created automatically by a call of `getParameter')
//

struct Parameter {
  uint id;              ///< unique identifyer
  void *pvalue;
  const char* name;
  ProcessL processes;
  Parameter(const char* name);
  virtual void writeValue(ostream& os) const = 0;
  virtual const char* typeName() const = 0;
};

template<class T>
struct Parameter_typed:Parameter {
  T value;
  Parameter_typed(const char* name, const T& _default):Parameter(name) {
    pvalue=&value;
    if (&_default) MT::getParameter<T>(value, name, _default);
    else           MT::getParameter<T>(value, name);
  }
  void writeValue(ostream& os) const { os <<value; }
  const char* typeName() const { return typeid(T).name(); }
};


//===========================================================================
//
// Access (preliminary - not used yet)
//

template<class T>
struct Access {
  T *var;             ///< pointer to the Variable (T must be derived from Variable)
  Process *p;         ///< pointer to the Process that might want to access the Variable
  uint last_revision; ///< last revision of a read/write access
  
  Access(Process *_p) { p=_p; last_revision = 0;  }
  T& operator()() { return *var; }
  
  bool needsUpdate() {  return last_revision != var->revision;  } //Does this need a lock?
  void readAccess() {  var->readAccess(p);  }
  void writeAccess() {  var->writeAccess(p);  }
  void deAccess() {  last_revision=var->revision;  var->deAccess(p);  }
};


//===========================================================================
//
// basic access to global system info
//

struct BirosInfo:Variable {

  VariableL variables;
  ProcessL processes;
  ParameterL parameters;
  
  Process *getProcessFromPID();
  
  BirosInfo();
  
  template<class T>  void getVariable(T*& v, const char* name, Process *p) {
    writeAccess(p);
    v = (T*)listFindByName(variables, name);
    deAccess(p);
    if (!v) MT_MSG("can't find biros variable '" <<name <<"' -- Process '" <<p->name <<"' will not connect");
  }
  template<class T> T getParameter(const char *name, Process *p, const T *_default=NULL) {
    Parameter_typed<T> *par;
    writeAccess(p);
    par = (Parameter_typed<T>*)listFindByName(parameters, name);
    deAccess(p);
    if (!par) par = new Parameter_typed<T>(name, *_default);
    if (!par->processes.contains(p)) par->processes.append(p);
    return par->value;
  }
  template<class T> T getParameter(const char *name) {
    return getParameter<T>(name, getProcessFromPID(), NULL);
  }
  template<class T> T getParameter(const char *name, const T& _default) {
    return getParameter<T>(name, getProcessFromPID(), &_default);
  }
  void dump(); //dump everything -- for debugging
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

void open(const ProcessL& P);
void loop(const ProcessL& P);
void loopSerialized(const ProcessL& P);
void loopWithBeat(const ProcessL& P, double sec);
void stop(const ProcessL& P);
void wait(const ProcessL& P);
void close(const ProcessL& P);


#ifdef  MT_IMPLEMENTATION
#  include "biros.cpp"
#endif

#endif
