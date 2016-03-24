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

Singleton<ConditionVariable> moduleShutdown;

void signalhandler(int s){
  int calls = moduleShutdown().incrementValue();
  cerr <<"\n*** System received signal " <<s <<" -- count=" <<calls <<endl;
  if(calls==1){
    LOG(0) <<" -- waiting for main loop to break on moduleShutdown().getValue()" <<endl;
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
    LOG(3) <<" ** moduleShutdown failed - hard exit!" <<endl;
    exit(1);
  }
}

void openModules(){
  NodeL threads = registry().getNodesOfType<Thread*>();
  for(Node* th:threads){ th->get<Thread*>()->open(); }
}

void stepModules(){
  NodeL threads = registry().getNodesOfType<Thread*>();
  for(Node* th:threads){ th->get<Thread*>()->step(); }
}

void closeModules(){
  NodeL threads = registry().getNodesOfType<Thread*>();
  for(Node* th:threads){ th->get<Thread*>()->close(); }
}

RevisionedAccessGatedClassL getVariables(){
  return registry().getValuesOfType<RevisionedAccessGatedClass>();
}

void threadOpenModules(bool waitForOpened, bool setSignalHandler){
  if(setSignalHandler) signal(SIGINT, signalhandler);
  NodeL threads = registry().getNodesOfType<Thread*>();
  for(Node *th: threads) th->get<Thread*>()->threadOpen();
  if(waitForOpened) for(Node *th: threads) th->get<Thread*>()->waitForOpened();
  for(Node *th: threads){
    Thread *mod=th->get<Thread*>();
    if(mod->metronome.ticInterval>=0.) mod->threadLoop();
    //otherwise the module is listening (hopefully)
  }
}

void threadCloseModules(){
  NodeL threads = registry().getNodesOfType<Thread*>();
  for(Node *th: threads) th->get<Thread*>()->threadClose();
  modulesReportCycleTimes();
}

void threadCancelModules(){
  NodeL threads = registry().getNodesOfType<Thread*>();
  for(Node *th: threads) th->get<Thread*>()->threadCancel();
}

void modulesReportCycleTimes(){
  cout <<"Cycle times for all Modules (msec):" <<endl;
  NodeL threads = registry().getNodesOfType<Thread*>();
  for(Node *th: threads){
    Thread *thread=th->get<Thread*>();
    cout <<std::setw(30) <<thread->name <<" : ";
    thread->timer.report();
  }
}

