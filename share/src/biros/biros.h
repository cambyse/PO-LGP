#ifndef MT_biros_h
#define MT_biros_h

#include <MT/array.h>
#include <MT/util.h>

/* NOTES:

  -- derive all biros objects from bObject -> specific lists become generic lists
  -- insideOut -> one view with all bObjects

  -- flow chart: x-axis=time, rows=processes&variables, transactions=vertical arrows, on each row: bars indicate duration of step of process / duration of revision of variable; arrow onto variable -> change of revision/color/bar of that variable; each process bar is labeled by step number and process name; each variable bar by revision and variable name.
  */

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
/**
 * A Variable is a dumb data holder which is used to exchange information
 * between processes.
 *
 * Inherit from the class Variable to create your own variable.
 */
struct Variable {
  struct sVariable *s;        ///< private
  MT::String name;            ///< Variable name
  ConditionVariable revision; ///< revision (= number of write accesses) number
  RWLock rwlock;              ///< rwLock (usually handled via read/writeAccess -- but views may access directly...)

  /// @name c'tor/d'tor
  Variable(const char* name);
  virtual ~Variable();
  
  /// @name access control, to be called by a processes before access, returns the revision
  int readAccess(Process*);  //might set the caller to sleep
  int writeAccess(Process*); //might set the caller to sleep
  int deAccess(Process*);
  
  /// @name syncing via a variable - the caller is set to sleep
  void waitForNextWriteAccess();
  int  waitForRevisionGreaterThan(int rev); //returns the revision
  
  /// @name info
  struct FieldRegistration& get_field(uint i) const;
};


//===========================================================================
/**
 * A Process does some calculation and shares the result via a Variable.
 *
 * Inherit from the class Process to create your own variable.
 * You need to implement open(), close(), and step().
 * step() should contain the actual calculation.
 */
struct Process {
  struct sProcess *s;      ///< private
  MT::String name;         ///< Process name
  ConditionVariable state; ///< the condition variable indicates the state of the thread: positive=steps-to-go, otherwise it is a ThreadState
  uint step_count;         ///< step count

  /// @name c'tor/d'tor
  Process(const char* name);
  virtual ~Process();

  /// @name to be overloaded by the specific implementation
  // TODO is it really necessary that open() and close() are pure virtual? the implementation normally does nothing.
  virtual void open() = 0;    ///< is called within the thread when the thread is created
  virtual void close() = 0;   ///< is called within the thread when the thread is destroyed
  virtual void step() = 0;    ///< is called within the thread when trigerring a step from outside (or when permanently looping)
  
  //-- a scalar function which may depend only on the referenced variables
  //code correctness requires that a call of _step() may only decrease _f() !!
  //virtual double _f(){ return 0.; }
  
  /// @name to be called from `outside' (e.g. the main) to start/step/close the thread
  void threadOpen(int priority=0);      ///< start the thread (in idle mode) (should be positive for changes)
  void threadClose();                   ///< close the thread (stops looping and waits for idle mode before joining the thread)
  void threadStep(uint steps=1, bool wait=false);     ///< trigger (multiple) step (idle -> working mode) (wait until idle? otherwise calling during non-idle -> error)
  void threadLoop();                    ///< loop, stepping forever
  void threadLoopWithBeat(double sec);  ///< loop with a fixed beat (cycle time)
  void threadStop();                    ///< stop looping

  void waitForIdle();                   ///< caller waits until step is done (working -> idle mode)
  bool isIdle();                        ///< check if in idle mode
  bool isClosed();                      ///< check if closed
  
  /// @name lisetn to variable
  void listenTo(Variable *var); //TODO: rename to 'listenTo' (because this is not doing anything WITHIN the thread)
  void listenTo(const VariableL &signalingVars);
  void stopListeningTo(Variable *var);
};


//===========================================================================
/**
 * Registration of fields in a Variable.
 */
struct FieldRegistration {
  const char* name;
  const char* userType;
  const char* sysType;
  void *p;           /// pointer to object
  Variable *var;     /// pointer to the containing variable
  virtual void writeValue(ostream& os) const = 0;
  virtual void readValue(istream& os) const { NIY; }
};

template<class T>
struct FieldRegistration_typed:FieldRegistration {
  FieldRegistration_typed(T *_p, Variable *_var, const char* _name, const char* _userType) {
    name = _name;
    userType = _userType;
    sysType = typeid(T).name();
    p = _p;
    var = _var;
  }
  void writeValue(ostream& os) const { os <<*((T*)p); }
  //void readValue(istream& is) const { is >>*((T*)p); }
};

void registerField(Variable *v, FieldRegistration* f);

/**
 * Macro for automatic setters and getters of members in Variables.
 */
#define FIELD(type, name) \
  type name; \
  inline int set_##name(const type& _x, Process *p){ \
    writeAccess(p);  name=(type&)_x;  return deAccess(p); } \
  inline int get_##name(type& _x, Process *p){ \
    readAccess(p);   _x=name;  return deAccess(p); } \
  inline type get_##name(Process *p){ \
    type _x; readAccess(p); _x=name; deAccess(p);  return _x;  } \
  inline void reg_##name(){ \
    registerField(this, new FieldRegistration_typed<type>(&name,this,#name,#type)); }


//===========================================================================
/**
 * Parameters.
 * TODO what do they actually do?
 * (these are usually not created directly by the user,
 * they are created automatically by a call of `getParameter')
 */
struct Parameter {
  void *pvalue;
  const char* name;
  ProcessL dependers;
  Parameter();
  virtual void writeValue(ostream& os) const = 0;
  virtual const char* typeName() const = 0;
};


/**
 * @brief Like Parameter but with type T.
 *
 * @tparam T the type of the parameter.
 */
template<class T>
struct Parameter_typed:Parameter {
  T value;
  Parameter_typed(const char* _name, const T& _default):Parameter() {
    name = _name;
    pvalue = &value;
    if(&_default) MT::getParameter<T>(value, name, _default);
    else          MT::getParameter<T>(value, name);
  }
  void writeValue(ostream& os) const { os <<value; }
  const char* typeName() const { return typeid(T).name(); }
};


//===========================================================================
/**
 * Macro to create a Processs.
 *
 * TODO This is actually only used in motion.h. Do we really need it?
 */
#define PROCESS(name)   \
  struct name:Process { \
    struct s##name *s;  \
    name();             \
    virtual ~name();    \
    void open();        \
    void step();        \
    void close();       \
  };


//===========================================================================
/**
 * Macro to easily acess a Variable.
 */
#define VAR(Type) \
  Type *_##Type;  _##Type = biros().getVariable<Type>(#Type, NULL);



//===========================================================================
/**
 * The class Biros allows basic access/step control and access to global system info.
 *
 * Biros reprents the graph of variables and processes.
 */
struct Biros:Variable {
  struct sBirosEventController *acc;

  VariableL variables;
  ProcessL processes;
  ParameterL parameters;

  /// @name c'tor/d'tor
  Biros();
  ~Biros();

  /// @name access existing processes, variables and parameters.
  Process *getProcessFromPID();
  template<class T> T* getVariable(const char* name, Process *p, bool required = false);
  template<class T> void getVariable(T*& v, const char* name, Process *p, bool required = false);
  template<class T> T* getProcess (const char* name, Process *p, bool required = false);
  template<class T> T getParameter(const char *name, Process *p=NULL);
  template<class T> T getParameter(const char *name, const T& _default, Process *p=NULL);
  template<class T> void setParameter(const char *name, T value);

  /// @name dump ALL available information
  void dump();

  /// @name system control
  void enableAccessLog();
  void dumpAccessLog();
  void blockAllAccesses();
  void unblockAllAccesses();
  void stepToNextAccess();
  void stepToNextWriteAccess();
};

Biros& biros(); //get access to the global info struct


//===========================================================================
//
// WorkingCopy TODO: remove this!
//

template<class T>
struct WorkingCopy {
  T *var;             ///< pointer to the Variable (T must be derived from Variable)
  T copy;
  Process *p;         ///< pointer to the Process that might want to access the Variable
  int last_revision; ///< last revision of a read/write access
  
  WorkingCopy() { p=NULL; var=NULL; last_revision = 0;  }
  T& operator()() { return copy; }
  
  void init(T *_v, Process *_p) {
    p=_p;
    var=_v;
    var->readAccess(p);
    copy = *var;
    last_revision = var->revision.getValue();
    var->deAccess(p);
    copy.name <<"_WorkingCopy_" <<(p?p->name:STRING("GLOBAL"));
  }
  void init(const char* var_name, Process *_p) {
    T *_v = biros().getVariable<T>(var_name, _p);
    init(_v, _p);
  }
  bool needsUpdate() {
    return last_revision != var->get_revision();
  }
  void push() {
    if (var->get_revision()>last_revision) MT_MSG("Warning: push overwrites revision");
    var->writeAccess(p);
    NIY //never do this: you can't overwrite the members name, field, s, id, revision, etc!!
    *var = copy;
    last_revision = var->revision; //(was incremented already on writeAccess)
    var->deAccess(p);
  }
  void pull() {
    if (last_revision == var->revision.getValue()) return;
    var->readAccess(p);
    copy = *var;
    last_revision = var->revision.getValue();
    var->deAccess(p);
  }
};


//===========================================================================
//
// handling groups of processes
//
void open(const ProcessL& P);
void step(const ProcessL& P);
void loop(const ProcessL& P);
void loopWithBeat(const ProcessL& P, double sec);
void stop(const ProcessL& P);
void wait(const ProcessL& P);
void close(const ProcessL& P);

//===========================================================================
//
// helpers
//
void writeInfo(ostream& os, Process& p, bool brief, char nl='\n');
void writeInfo(ostream& os, Variable& v, bool brief, char nl='\n');
void writeInfo(ostream& os, FieldRegistration& f, bool brief, char nl='\n');
void writeInfo(ostream& os, Parameter& pa, bool brief, char nl='\n');


#include "biros_views.h"
#include "biros_t.cxx"

#ifdef  MT_IMPLEMENTATION
#  include "biros.cpp"
#endif

#endif
