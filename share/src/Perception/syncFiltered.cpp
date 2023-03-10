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
#include "syncFiltered.h"
#include <Kin/frame.h>

SyncFiltered::SyncFiltered(const char* outputWorld_name)
  : Thread("SyncFiltered", -1.),
    percepts_filtered(this, "percepts_filtered", true),
    outputWorld(this, outputWorld_name){
  threadOpen();
}

SyncFiltered::~SyncFiltered(){
  threadClose();
}

void SyncFiltered::open(){
//  outputWorld.set() = modelWorld.get();
}

void SyncFiltered::step(){
  uintA existingIDs;

  percepts_filtered.writeAccess();
  for(Percept *p:percepts_filtered()){
    p->syncWith(outputWorld.set());
    existingIDs.append(p->id);
  }
  percepts_filtered.deAccess();

  // delete non-existing bodies
  outputWorld.writeAccess();
  for(mlr::Frame *b:outputWorld().frames){
    if(b->name.startsWith("perc_")){
      uint id;
      b->name.resetIstream();
      b->name >>PARSE("perc_") >>id;
      if(!existingIDs.contains(id)){
        delete b;
      }
    }
  }
  outputWorld.deAccess();

}


