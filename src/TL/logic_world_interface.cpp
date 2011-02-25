/*  
    Copyright 2009   Tobias Lang
    
    Homepage:  cs.tu-berlin.de/~lang/
    E-mail:    lang@cs.tu-berlin.de
    
    This file is part of libPRADA.

    libPRADA is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libPRADA is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libPRADA.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "logic_world_interface.h"
#include <string>
#include "MT/ors.h"
#include "TL/bwLanguage.h"



	
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
//      B W  -  L O G I C  -  S I M U L A T O R  -  I N T E R F A C E
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------



TL::State* TL::logic_world_interface::bw::observeLogic(ActionInterface* ai, LogicEngine* le) {
  CHECK(le!=NULL, "Missing logic engine");
	State* state = new State;
  
	uint i, j;
  uint table_id = ai->getTableID();
  
  // TABLE
  state->pi_prim.append(le->getPIorig(TL::bwLanguage::createPredicateInstance_table(table_id, le)));
  // BLOCKS
  uintA blocks;
  ai->getBlocks(blocks);
  FOR1D(blocks, i) {
    state->pi_prim.append(le->getPIorig(TL::bwLanguage::createPredicateInstance_block(blocks(i), le)));
  }
  // BALLS
  uintA balls;
  ai->getBalls(balls);
  FOR1D(balls, i) {
    state->pi_prim.append(le->getPIorig(TL::bwLanguage::createPredicateInstance_ball(balls(i), le)));
  }
  // BOXES
  uintA boxes;
  ai->getBoxes(boxes);
  FOR1D(boxes, i) {
    state->pi_prim.append(le->getPIorig(TL::bwLanguage::createPredicateInstance_box(boxes(i), le)));

  }
  uintA all_objs;
  ai->getObjects(all_objs);
  // SIZE
  if (le->getFunction(MT::String("size")) != NULL) {
    double length;
    FOR1D(all_objs, i) {
      length = ai->getSize(all_objs(i))[0];
      state->fv_prim.append(le->getFVorig(TL::bwLanguage::createFunctionValue_size(all_objs(i), REPLACE_SIZE(length), le)));
    }
  }
  
  // HOMIES
  if (le->getPredicate(MT::String("homies")) != NULL) {
    uint k;
    FOR1D(all_objs, i) {
      for (k=i+1; k<all_objs.N; k++) {
        if (
            areEqual(ai->getColor(all_objs(k))[0], ai->getColor(all_objs(i))[0])
            &&    areEqual(ai->getColor(all_objs(k))[1], ai->getColor(all_objs(i))[1])
            &&    areEqual(ai->getColor(all_objs(k))[2], ai->getColor(all_objs(i))[2])) {
          state->pi_prim.append(le->getPIorig(TL::bwLanguage::createPredicateInstance_homies(all_objs(i), all_objs(k), le)));
          state->pi_prim.append(le->getPIorig(TL::bwLanguage::createPredicateInstance_homies(all_objs(k), all_objs(i), le)));
        }
      }
    }
  }
  
  
  // INHAND
  uint catchedID = ai->getInhand();
  if (catchedID != UINT_MAX) {
    state->pi_prim.append(le->getPIorig(TL::bwLanguage::createPredicateInstance_inhand(ai->getInhand(), le)));
  }
  
	// ON relations
  uintA aboveObjs;
  FOR1D(all_objs, i) {
    ai->getObjectsAbove(aboveObjs, all_objs(i));
//     cout<<"above "<<all_objs(i)<<" are "<<aboveObjs<<endl;
		FOR1D(aboveObjs, j) {
      if (all_objs.findValue(aboveObjs(j)) >= 0)
        state->pi_prim.append(le->getPIorig(TL::bwLanguage::createPredicateInstance_on(aboveObjs(j), all_objs(i), le)));
		}
	}
  
  // UPRIGHT
  if (le->getPredicate(MT::String("upright")) != NULL) {
    FOR1D(all_objs, i) {
      if (ai->isUpright(all_objs(i)))
        state->pi_prim.append(le->getPIorig(TL::bwLanguage::createPredicateInstance_upright(all_objs(i), le)));
    }
  }
  // OUT
  FOR1D(all_objs, i) {
    if (ai->onGround(all_objs(i)))
      state->pi_prim.append(le->getPIorig(TL::bwLanguage::createPredicateInstance_out(all_objs(i), le)));
  }
  
  // CONTAINS
  state->pi_prim.memMove = true;
  FOR1D(boxes, i) {
    uint o = ai->getContainedObject(boxes(i));
    if (o != UINT_MAX) {
      state->pi_prim.removeValueSafe(le->getPIorig(TL::bwLanguage::createPredicateInstance_on(o, table_id, le)));
      state->pi_prim.append(le->getPIorig(TL::bwLanguage::createPredicateInstance_contains(boxes(i), o, le)));
    }
  }
  
  // CLOSED
  FOR1D(boxes, i) {
    if (ai->isClosed(boxes(i))) {
      state->pi_prim.append(le->getPIorig(TL::bwLanguage::createPredicateInstance_closed(boxes(i), le)));
    }
  }

 
 
  le->derive(state);
  return state;
}




void TL::logic_world_interface::bw::observeAngles(arr& angles, ActionInterface* ai, LogicEngine* le) {
  uint i, k;
  uintA objs;
  ai->getObjects(objs);
  angles.resize(objs.N, 2);
  FOR1D(objs, i) {
    arr orientation;
    ai->getOrientation(orientation, objs(i));
    CHECK(orientation.N == 2, "too many angles");
    FOR1D(orientation, k) {
      angles(i, k) = orientation(k);
      if (angles(i, k) < 0.00001)
        angles(i, k) = 0.;
    }
  }
}

void TL::logic_world_interface::bw::observePositions(arr& positions, ActionInterface* ai, LogicEngine* le) {
  uint i, k;
  uintA objs;
  ai->getObjects(objs);
  positions.resize(objs.N, 3);
  FOR1D(objs, i) {
    double* local_position = ai->getPosition(objs(i));
    for (k=0; k<3; k++) {
      positions(i, k) = local_position[k];
    }
  }
}

void TL::logic_world_interface::bw::writeFeatures(std::ostream& os, ActionInterface* ai) {
  uint i, k;
  uintA objs;
  ai->getObjects(objs);
  os<<"{"<<endl;
  // position
  os<<"["<<endl;
  FOR1D(objs, i) {
    os<<objs(i)<<" ";
    double* pos = ai->getPosition(objs(i));
    os<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<" ";
    os<<endl;
  }
  os<<"]"<<endl;
  // orientation
  os<<"["<<endl;
  FOR1D(objs, i) {
    os<<objs(i)<<" ";
    arr orientation;
    ai->getOrientation(orientation, objs(i));
    FOR1D(orientation, k) {
      if (orientation(k) < 0.00001)
        orientation(k) = 0.;
      os<<orientation(k)<<" ";
    }
    os<<endl;
  }
  os<<"]"<<endl;
  os<<"}"<<endl;
}






void TL::logic_world_interface::bw::performAction(PredicateInstance* action, ActionInterface* ai, uint secs_wait_after_action, const char* message) {
  if (action == NULL  ||  action->pred->name == "no_action"  ||  action->pred->id == 0) {
    // don't do anything
    return;
  }
  else if (action->args.N > 0) {
    // special care for out of reach objects
    if (ai->onGround(action->args(0))) {
        return;
    }
  }
  if (action->pred->name == "grab") {
    uintA list;
    ai->getObjectsAbove(list, action->args(0));
    if (ai->getOrsType(action->args(0)) == OBJECT_TYPE__BOX   // cannot lift box
        ||   ai->containedInClosedBox(action->args(0)))    // cannot take from closed box
      ai->simulate(30, message);
    else
      ai->grab(action->args(0), message);
	}
	else if (action->pred->name == "puton") {
    if (ai->getOrsType(action->args(0)) == OBJECT_TYPE__BOX  // don't do anything if object = filled open box
          && !ai->isClosed(action->args(0))
          &&  ai->getContainedObject(action->args(0)) != UINT_MAX) 
      ai->simulate(30, message); 
    else if (ai->containedInBox(action->args(0))) // don't do anything if object in box
      ai->simulate(30, message); 
    else
      ai->dropObjectAbove(action->args(0), message);
	}
  else if (action->pred->name == "lift") {
    uintA list;
    ai->getObjectsAbove(list, action->args(1));
    if (ai->getOrsType(action->args(0)) == OBJECT_TYPE__BOX   // cannot lift box
        ||   ai->containedInClosedBox(action->args(0)))    // cannot take from closed box
      ai->simulate(10);
    else if (ai->getOrsType(action->args(1)) != OBJECT_TYPE__BOX  &&     // cannot lift from not-box if not on not-box
        list.findValue(action->args(0)) < 0)
      ai->simulate(10);
    else
      ai->grab(action->args(0));
  }
  else if (action->pred->name == "place") {
    if (action->args(0) != ai->getInhand()) // don't do anything if 1st object not the one in hand
      ai->simulate(10); 
    else if (ai->containedInBox(action->args(1))) // don't do anything if 2nd object in box
      ai->simulate(10); 
    else if (ai->getOrsType(action->args(1)) == OBJECT_TYPE__BOX  // don't do anything if 2nd object = filled open box
          && !ai->isClosed(action->args(1))
          &&  ai->getContainedObject(action->args(1)) != UINT_MAX) 
      ai->simulate(10); 
    else
      ai->dropObjectAbove(action->args(0), action->args(1));
  }
  else if (action->pred->name == "openBox") {
    if (!ai->isBox(action->args(0)))
      ai->simulate(30, message);
    else {
      uintA aboves;
      ai->getObjectsAbove(aboves,action->args(0));
      if (aboves.N > 0)
        ai->simulate(30, message); // don't do anything something on box
      else
        ai->openBox(action->args(0), message);
    }
  }
  else if (action->pred->name == "closeBox") {
    if (!ai->isBox(action->args(0)))
      ai->simulate(30, message);
    else {
//     if (ai->getInhand() != UINT_MAX) // don't do anything if something inhand
//       ai->simulate(10);
//     else
      ai->closeBox(action->args(0), message);
    }
  }
	else
		NIY
   
  ai->relaxPosition(message); // needed to have a observe correct subsequent state
  ai->simulate(secs_wait_after_action); // needed to have a correct subsequent state
}


