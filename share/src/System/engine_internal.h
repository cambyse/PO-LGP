#include "module.h"

enum ProcessState { tsIDLE=0, tsCLOSE=-1, tsOPENING=-2, tsLOOPING=-3, tsBEATING=-4 }; //positive states indicate steps-to-go

//===========================================================================
/**
 * A Process does some calculation and shares the result via a Variable.
 *
 * Inherit from the class Process to create your own variable.
 * You need to implement open(), close(), and step().
 * step() should contain the actual calculation.
 */
struct Process: Thread{
  Module *module;
  Type *moduleDcl;
  ConditionVariable state;       ///< the condition variable indicates the state of the thread: positive=steps-to-go, otherwise it is a ThreadState
  pid_t tid;                     ///< system thread id
  VariableL listensTo;
  //ParameterL dependsOn;
  Metronome *metronome;          ///< used for beat-looping

  /// @name c'tor/d'tor
  Process(Type *_moduleDcl=NULL);
  virtual ~Process();

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

  /// @name listen to variable
  void listenTo(Variable *var);
  void listenTo(const VariableL &signalingVars);
  void stopListeningTo(Variable *var);

  virtual void main(); //virtual method for Thread
};
