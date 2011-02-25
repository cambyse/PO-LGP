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

//(``private'') spaces used by the implementation but hidden from this header
struct sVariable;
struct sProcess;
struct sMonitor;

typedef MT::Array<Variable*> VariableL;
typedef MT::Array<Process*> ProcessL;

#define READ_ACCESS(var,name) {\
  var->readAccess(this);\
  name=var->name;\
  var->deAccess(this);
  
//===========================================================================
//
// Variable
//

struct Variable{
  sVariable *s;
  const char* name;
  //MT::Array<Process*> processes;

  Variable(const char* name);
  ~Variable();

  //-- to be overloaded by the specific implementation
  virtual void write(ostream& os){ os <<name; }
  virtual void read (istream& is){ }

  //-- access control, to be called by a processes before access
  void readAccess(Process*);  //might set the caller to sleep
  void writeAccess(Process*); //might set the caller to sleep
  void deAccess(Process*);

  //-- condition variable control, to be used by processes to broadcast (publish) or wait for broadcast (subscribe)
  int  getCondition();
  void setCondition(int i);
  void waitForConditionSignal();
  void waitForConditionEq(int i);    //might set the caller to sleep
  void waitForConditionNotEq(int i); //might set the caller to sleep

  //--
  void clampToDefault();
  void saveAsDefault();
  bool isClamped();
};


//===========================================================================
//
// Process
//

struct Process{
  sProcess *s;
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

struct Group{
  VariableL V;
  ProcessL P;

  void set(const VariableL&,const ProcessL&);
  void loop();
  void stop();
  void close();

};

void reportGlobalProcessGraph();


struct sThreadInfoWin;
struct ThreadInfoWin:public Process{
  sThreadInfoWin *s;
      
  ThreadInfoWin();
  ~ThreadInfoWin();
  
  void open();
  void close();
  void step();
};

#ifdef MT_IMPLEMENTATION
#  include "process.cpp"
#endif

#endif
