#ifndef MT_process_h
#define MT_process_h

#include "array.h"
#include "util.h"

//===========================================================================
//
// forward declarations
//

struct Variable;
struct Process;
struct Monitor;

typedef MT::Array<Variable*> VariableL;
typedef MT::Array<Process*> ProcessL;

#define READ_ACCESS(var, name){\
    var->readAccess(this);\
    name=var->name;\
    var->deAccess(this);

//===========================================================================
//
// Info for Variables fields
//

struct _Variable_field_info_base{
  void *p;
  const char* name;
};
  
template<class T>
struct _Variable_field_info:_Variable_field_info_base{
  _Variable_field_info(T *_p, const char* _name){ p=_p; name=_name; }
};

#define FIELD(type, name) \
  type name; \
  inline void set_##name(const type& _x, Process *p){ \
    writeAccess(p);  name=_x;  deAccess(p); } \
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
  struct sVariable *s; //private
  const char* name;
  MT::Array<_Variable_field_info_base*> fields;

  //MT::Array<Process*> processes;
  
  Variable(const char* name);
  ~Variable();
  
  //-- to be overloaded by the specific implementation
  virtual void write(ostream& os){ os <<name; }
  virtual void read(istream& is){ }
  
  //-- access control, to be called by a processes before access
  void readAccess(Process*);  //might set the caller to sleep
  void writeAccess(Process*); //might set the caller to sleep
  void deAccess(Process*);
  
  //-- the following is preliminary

  //-- condition variable control, to be used by processes to broadcast (publish) or wait for broadcast (subscribe)
  int  getCondition();
  void setCondition(int i);
  void waitForConditionSignal();
  void waitForConditionEq(int i);    //might set the caller to sleep
  void waitForConditionNotEq(int i); //might set the caller to sleep
  
  //-- ideas to let a variable auto initialize itself
  void clampToDefault();
  void saveAsDefault();
  bool isClamped();
};


//===========================================================================
//
// Process
//

struct Process {
  struct sProcess *s;
  const char* name;
  VariableL V;
  
  Process(const char* name);
  ~Process();
  
  //-- to be overloaded by the specific implementation
  virtual void open() = 0;  ///< is called within the thread when the thread is created
  virtual void close() = 0; ///< is called within the thread when the thread is destroyed
  virtual void step() = 0;  ///< is called within the thread when trigerring a step from outside (or when permanently looping)
  
  //-- a scalar function which may depend only on the referenced variables
  //   -- code correctness requires that a call of _step() may only decrease _f() !!
  virtual double _f(){ return 0.; }
  
  //--
  void threadOpen(int priority=0);      ///< start the thread (in idle mode) (should be positive for changes)
  void threadClose();                   ///< close the thread (stops looping and waits for idle mode before joining the thread)
  
  void threadStepOrSkip(uint maxSkips); ///< trigger a step (idle -> working mode) or skip if still busy (counts skips.., maxSkip=0 -> no warnings)
  void threadStep(bool wait=false);     ///< trigger a step (idle -> working mode) (wait until idle? otherwise calling during non-idle -> error)
  void threadSteps(uint steps);         ///< trigger multiple steps (idle -> working mode)
  void threadWait();                    ///< wait until step is done (working -> idle mode)
  bool threadIsIdle();                  ///< check if in idle mode
  
  void threadLoop();                    ///< loop stepping forever
  void threadLoopWithBeat(double sec);
  void threadLoopSyncWithDone(Process& p);
  void threadStop();                    ///< stop looping
};


//===========================================================================
//
// Global Stuff / Monitor
//

//! a simple struct to realize a strict tic tac timing (call step() once in a loop)
struct Metronome {
  long targetDt;
  timespec ticTime, lastTime;
  uint tics;
  const char* name;                   ///< name
  
  Metronome(const char* name, long _targetDt); //!< set tic tac time in milli seconds
  ~Metronome();
  
  void reset();
  void waitForTic();              //!< waits until the next tic
  double getTimeSinceTic();       //!< time since last tic
};

//! a really simple thing to meassure cycle and busy times
struct CycleTimer {
  uint steps;
  double cyclDt, cyclDtMean, cyclDtMax;  ///< internal variables to measure step time
  double busyDt, busyDtMean, busyDtMax;  ///< internal variables to measure step time
  timespec now, lastTime;
  const char* name;                    ///< name
  CycleTimer(const char *_name=NULL);
  ~CycleTimer();
  void reset();
  void cycleStart();
  void cycleDone();
};


//===========================================================================
//
// Basic low-level X11 Monitor
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
// preliminary
//

struct Group {
  VariableL V;
  ProcessL P;
  
  void set(const VariableL&, const ProcessL&);
  void loop();
  void stop();
  void close();
  
};

void reportGlobalProcessGraph();

#ifdef  MT_IMPLEMENTATION
#  include "process.cpp"
#endif

#endif
