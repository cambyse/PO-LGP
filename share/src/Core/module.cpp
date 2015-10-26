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

#include <signal.h>
#include <iomanip>

#include "module.h"

Singleton<ConditionVariable> shutdown;

void signalhandler(int s){
  int calls = shutdown().incrementValue();
  cerr <<"\n*** System received signal " <<s <<" -- count=" <<calls;
  if(calls==1){
    LOG(0) <<" -- waiting for main loop to break on shutdown().getValue()" <<endl;
  }
  if(calls==2){
    LOG(0) <<" -- smoothly closing modules directly" <<endl;
    threadCloseModules(); //might lead to a hangup of the main loop, but processes should close
    LOG(0) <<" -- DONE" <<endl;
  }
  if(calls==3){
    LOG(0) <<" -- cancelling threads to force closing" <<endl;
    threadCancelModules();
    LOG(0) <<" -- DONE" <<endl;
  }
  if(calls>3){
    LOG(3) <<" ** shutdown failed - hard exit!" <<endl;
    exit(1);
  }
}

void openModules(){
  NodeL ms = registry().getTypedNodes<Module>();
  for(Node* m:ms){ m->V<Module>().open(); }
}

void stepModules(){
  NodeL ms = registry().getTypedNodes<Module>();
  for(Node* m:ms){ m->V<Module>().step(); }
}

void closeModules(){
  NodeL ms = registry().getTypedNodes<Module>();
  for(Node* m:ms){ m->V<Module>().close(); }
}


Node* getModuleNode(Module *module){
  NodeL ms = registry().getTypedNodes<Module>();
  for(auto& m:ms){ if(m->getValue<Module>()==module) return m; }
  MLR_MSG("module not found!");
  return NULL;
}

Node* getVariable(const char* name){
  return registry().getNode(name);
//  NodeL vars = registry().getTypedNodes<RevisionedAccessGatedClass>();
//  for(auto& v:vars){ if(v->V<RevisionedAccessGatedClass>().name==name) return v; }
//  MLR_MSG("module not found!");
//  return NULL;
}


void threadOpenModules(bool waitForOpened){
  signal(SIGINT, signalhandler);
  NodeL mods = registry().getTypedNodes<Module>();
  for(Node *m: mods) m->V<Module>().threadOpen();
  if(waitForOpened) for(Node *m: mods) m->V<Module>().waitForOpened();
  for(Node *m: mods){
    Module& mod=m->V<Module>();
    if(mod.metronome.ticInterval>=0.) mod.threadLoop();
    //otherwise the module is listening (hopefully)
  }
}

void threadCloseModules(){
  NodeL mods = registry().getTypedNodes<Module>();
  for(Node *m: mods) m->V<Module>().threadClose();
}

void threadCancelModules(){
  NodeL mods = registry().getTypedNodes<Module>();
  for(Node *m: mods) m->V<Module>().threadCancel();
}


void modulesReportCycleTimes(){
  cout <<"Cycle times for all Modules (msec):" <<endl;
  NodeL mods = registry().getTypedNodes<Module>();
  for(Node *m: mods){
    Module& mod=m->V<Module>();
    cout <<std::setw(30) <<mod.name <<" : ";
    mod.timer.report();
  }
}

