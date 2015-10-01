/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#include "module.h"

Module *currentlyCreating = NULL;
AccessL *currentlyCreatingAccessL = NULL;
ModuleL& NoModuleL = *((ModuleL*)NULL);

Singleton<Graph> moduleSystem;

void openModules(Graph&){
  NodeL ms = moduleSystem().getTypedNodes<Module>();
  for(Node* m:ms){ m->V<Module>().open(); }
}

void stepModules(Graph&){
  NodeL ms = moduleSystem().getTypedNodes<Module>();
  for(Node* m:ms){ m->V<Module>().step(); }
}

void closeModules(Graph&){
  NodeL ms = moduleSystem().getTypedNodes<Module>();
  for(Node* m:ms){ m->V<Module>().close(); }
}


Node* getModuleNode(Module *module){
  NodeL ms = moduleSystem().getTypedNodes<Module>();
  for(auto& m:ms){ if(m->getValue<Module>()==module) return m; }
  MT_MSG("module not found!");
  return NULL;
}

Node* getVariable(const char* name){
  return moduleSystem().getNode(name);
//  NodeL vars = moduleSystem().getTypedNodes<RevisionedAccessGatedClass>();
//  for(auto& v:vars){ if(v->V<RevisionedAccessGatedClass>().name==name) return v; }
//  MT_MSG("module not found!");
//  return NULL;
}


void threadOpenModules(Graph&, bool waitForOpened){
//  signal(SIGINT, signalhandler);
  NodeL mods = moduleSystem().getTypedNodes<Module>();
  for(Node *m: mods) m->V<Module>().threadOpen();
  if(waitForOpened) for(Node *m: mods) m->V<Module>().waitForOpened();
  for(Node *m: mods){
    Module& mod=m->V<Module>();
    if(mod.beat>0.) mod.threadLoopWithBeat(mod.beat);
    else if(mod.beat<-.1) mod.threadLoop();
    //otherwise the module is listening (hopefully)
  }
}

void threadCloseModules(Graph&){
  NodeL mods = moduleSystem().getTypedNodes<Module>();
  for(Node *m: mods) m->V<Module>().threadClose();
}
