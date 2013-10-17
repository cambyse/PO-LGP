#include <Core/module.h>


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
  //VariableL listensTo;

  /// @name c'tor/d'tor
  Process(Type *_moduleDcl=NULL):Thread("TODO: module name"), module(NULL), moduleDcl(_moduleDcl){}
  Process(const char* name):Thread(name), module(NULL), moduleDcl(NULL){}
  virtual void open(){ if(!moduleDcl) HALT(""); module = (Module*)moduleDcl->newInstance();  module->proc = this; }
  virtual void step(){ if(!module) HALT(""); module->step(); module->step_count++; }
  virtual void close(){ if(!module) HALT(""); delete module; module=NULL; }
};
