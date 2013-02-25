#include "biros.h"
#include <MT/module.h>

//! this creates a model (reffered to by name) out of the registry() and wraps it as a process
struct Module_Process: Process {
  Module *mod;
  Module_Process(const char* name):Process(name),mod(NULL){}
  void open();
  void close(){ delete mod; mod=NULL; }
  void step(){ mod->step(); }
};
