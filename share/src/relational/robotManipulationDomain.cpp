/*  
    Copyright 2011   Tobias Lang
    
    E-mail:    tobias.lang@fu-berlin.de
    
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

#include "robotManipulationDomain.h"
#include "logicReasoning.h"



/* ---------------------
    TOP-LEVEL LOGIC-SIMULATOR-INTERFACE
  --------------------- */


TL::State* TL::RobotManipulationDomain::observeLogic(RobotManipulationSimulator* sim) {
  State* state = new State;
  
  uint i, j;
  
  // TABLE
  uint table_id = sim->getTableID();
  if (TL::logicObjectManager::constants.findValue(table_id) < 0)
    table_id = TL::UINT_NIL;
  if (table_id != TL::UINT_NIL) {
    state->lits_prim.append(TL::logicObjectManager::getLiteralOrig(TL::RobotManipulationDomain::createLiteral_table(table_id)));
  }
  // BLOCKS
  uintA blocks;
  sim->getBlocks(blocks);
  FOR1D(blocks, i) {
    state->lits_prim.append(TL::logicObjectManager::getLiteralOrig(TL::RobotManipulationDomain::createLiteral_block(blocks(i))));
  }
  // BALLS
  uintA balls;
  sim->getBalls(balls);
  FOR1D(balls, i) {
    state->lits_prim.append(TL::logicObjectManager::getLiteralOrig(TL::RobotManipulationDomain::createLiteral_ball(balls(i))));
  }
  // BOXES
  uintA boxes;
  sim->getBoxes(boxes);
  FOR1D(boxes, i) {
    state->lits_prim.append(TL::logicObjectManager::getLiteralOrig(TL::RobotManipulationDomain::createLiteral_box(boxes(i))));

  }
  uintA all_objs;
  sim->getObjects(all_objs);
  // filter for logic
  FOR1D_DOWN(all_objs, i) {
    if (TL::logicObjectManager::constants.findValue(all_objs(i)) < 0) {
      all_objs.removeValueSafe(all_objs(i));
    }
  }
  
  // SIZE
  if (TL::logicObjectManager::getFunction(MT::String("size")) != NULL) {
    double length;
    FOR1D(all_objs, i) {
      length = sim->getSize(all_objs(i))[0];
      state->fv_prim.append(TL::logicObjectManager::getFVorig(TL::RobotManipulationDomain::createFunctionValue_size(all_objs(i), REPLACE_SIZE(length))));
    }
  }
  
  // HOMIES
  if (TL::logicObjectManager::getPredicate(MT::String("homies")) != NULL) {
    uint k;
    FOR1D(all_objs, i) {
      for (k=i+1; k<all_objs.N; k++) {
        if (
            areEqual(sim->getColor(all_objs(k))[0], sim->getColor(all_objs(i))[0])
            &&    areEqual(sim->getColor(all_objs(k))[1], sim->getColor(all_objs(i))[1])
            &&    areEqual(sim->getColor(all_objs(k))[2], sim->getColor(all_objs(i))[2])) {
          state->lits_prim.append(TL::logicObjectManager::getLiteralOrig(TL::RobotManipulationDomain::createLiteral_homies(all_objs(i), all_objs(k))));
          state->lits_prim.append(TL::logicObjectManager::getLiteralOrig(TL::RobotManipulationDomain::createLiteral_homies(all_objs(k), all_objs(i))));
        }
      }
    }
  }
  
  
  // INHAND
  uint catchedID = sim->getInhand();
  if (catchedID != UINT_MAX) {
    state->lits_prim.append(TL::logicObjectManager::getLiteralOrig(TL::RobotManipulationDomain::createLiteral_inhand(sim->getInhand())));
  }
  
  // ON relations
  uintA objects_on;
  FOR1D(all_objs, i) {
    sim->getObjectsOn(objects_on, all_objs(i));
    FOR1D(objects_on, j) {
      if (all_objs.findValue(objects_on(j)) >= 0)
        state->lits_prim.append(TL::logicObjectManager::getLiteralOrig(TL::RobotManipulationDomain::createLiteral_on(objects_on(j), all_objs(i))));
    }
  }
  
  // UPRIGHT
  if (TL::logicObjectManager::getPredicate(MT::String("upright")) != NULL) {
    FOR1D(all_objs, i) {
      if (sim->isUpright(all_objs(i)))
        state->lits_prim.append(TL::logicObjectManager::getLiteralOrig(TL::RobotManipulationDomain::createLiteral_upright(all_objs(i))));
    }
  }
  // OUT
  FOR1D(all_objs, i) {
    if (sim->onGround(all_objs(i)))
      state->lits_prim.append(TL::logicObjectManager::getLiteralOrig(TL::RobotManipulationDomain::createLiteral_out(all_objs(i))));
  }
  
  // CONTAINS
  state->lits_prim.memMove = true;
  FOR1D(boxes, i) {
    uint o = sim->getContainedObject(boxes(i));
    if (o != UINT_MAX) {
      if (table_id == TL::UINT_NIL) {
        state->lits_prim.removeValueSafe(TL::logicObjectManager::getLiteralOrig(TL::RobotManipulationDomain::createLiteral_on(o, table_id)));
      }
      state->lits_prim.append(TL::logicObjectManager::getLiteralOrig(TL::RobotManipulationDomain::createLiteral_contains(boxes(i), o)));
    }
  }
  
  // CLOSED
  FOR1D(boxes, i) {
    if (sim->isClosed(boxes(i))) {
      state->lits_prim.append(TL::logicObjectManager::getLiteralOrig(TL::RobotManipulationDomain::createLiteral_closed(boxes(i))));
    }
  }

 
 
  TL::logicReasoning::derive(state);
  return state;
}




void TL::RobotManipulationDomain::observeAngles(arr& angles, RobotManipulationSimulator* sim) {
  uint i, k;
  uintA objs;
  sim->getObjects(objs);
  angles.resize(objs.N, 2);
  FOR1D(objs, i) {
    arr orientation;
    sim->getOrientation(orientation, objs(i));
    CHECK(orientation.N == 2, "too many angles");
    FOR1D(orientation, k) {
      angles(i, k) = orientation(k);
      if (angles(i, k) < 0.00001)
        angles(i, k) = 0.;
    }
  }
}

void TL::RobotManipulationDomain::observePositions(arr& positions, RobotManipulationSimulator* sim) {
  uint i, k;
  uintA objs;
  sim->getObjects(objs);
  positions.resize(objs.N, 3);
  FOR1D(objs, i) {
    double* local_position = sim->getPosition(objs(i));
    for (k=0; k<3; k++) {
      positions(i, k) = local_position[k];
    }
  }
}

void TL::RobotManipulationDomain::writeFeatures(std::ostream& os, RobotManipulationSimulator* sim) {
  uint i, k;
  uintA objs;
  sim->getObjects(objs);
  os<<"{"<<endl;
  // position
  os<<"["<<endl;
  FOR1D(objs, i) {
    os<<objs(i)<<" ";
    double* pos = sim->getPosition(objs(i));
    os<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<" ";
    os<<endl;
  }
  os<<"]"<<endl;
  // orientation
  os<<"["<<endl;
  FOR1D(objs, i) {
    os<<objs(i)<<" ";
    arr orientation;
    sim->getOrientation(orientation, objs(i));
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






void TL::RobotManipulationDomain::performAction(Atom* action, RobotManipulationSimulator* sim, uint secs_wait_after_action, const char* message) {
  if (action == NULL  ||  action->pred->name == "doNothing"  ||  action->pred->id == 0) {
    // don't do anything
    return;
  }
  else if (action->args.N > 0) {
    // special care for out of reach objects
    if (sim->onGround(action->args(0))) {
        return;
    }
  }
  if (action->pred->name == "grab"  ||  action->pred->name == "grab_puton") {
    uintA list;
    sim->getObjectsOn(list, action->args(0));
    if (sim->getOrsType(action->args(0)) == OBJECT_TYPE__BOX   // cannot lift box
        ||   sim->containedInClosedBox(action->args(0)))    // cannot take from closed box
      sim->simulate(30, message);
    else
      sim->grab(action->args(0), message);
  }
  else if (action->pred->name == "puton"  ||  action->pred->name == "puton_puton"  ||  action->pred->name == "puton_grab") {
    if (sim->getOrsType(action->args(0)) == OBJECT_TYPE__BOX  // don't do anything if object = filled open box
          && !sim->isClosed(action->args(0))
          &&  sim->getContainedObject(action->args(0)) != UINT_MAX) 
      sim->simulate(30, message); 
    else if (sim->containedInBox(action->args(0))) // don't do anything if object in box
      sim->simulate(30, message); 
    else
      sim->dropObjectAbove(action->args(0), message);
  }
  else if (action->pred->name == "lift") {
    uintA list;
    sim->getObjectsOn(list, action->args(1));
    if (sim->getOrsType(action->args(0)) == OBJECT_TYPE__BOX   // cannot lift box
        ||   sim->containedInClosedBox(action->args(0)))    // cannot take from closed box
      sim->simulate(10);
    else if (sim->getOrsType(action->args(1)) != OBJECT_TYPE__BOX  &&     // cannot lift from not-box if not on not-box
        list.findValue(action->args(0)) < 0)
      sim->simulate(10);
    else
      sim->grab(action->args(0));
  }
  else if (action->pred->name == "place") {
    if (action->args(0) != sim->getInhand()) // don't do anything if 1st object not the one in hand
      sim->simulate(10); 
    else if (sim->containedInBox(action->args(1))) // don't do anything if 2nd object in box
      sim->simulate(10); 
    else if (sim->getOrsType(action->args(1)) == OBJECT_TYPE__BOX  // don't do anything if 2nd object = filled open box
          && !sim->isClosed(action->args(1))
          &&  sim->getContainedObject(action->args(1)) != UINT_MAX) 
      sim->simulate(10); 
    else
      sim->dropObjectAbove(action->args(0), action->args(1));
  }
  else if (action->pred->name == "openBox") {
    if (!sim->isBox(action->args(0)))
      sim->simulate(30, message);
    else {
      uintA aboves;
      sim->getObjectsOn(aboves,action->args(0));
      if (aboves.N > 0)
        sim->simulate(30, message); // don't do anything something on box
      else
        sim->openBox(action->args(0), message);
    }
  }
  else if (action->pred->name == "closeBox") {
    if (!sim->isBox(action->args(0)))
      sim->simulate(30, message);
    else {
//     if (sim->getInhand() != UINT_MAX) // don't do anything if something inhand
//       sim->simulate(10);
//     else
      sim->closeBox(action->args(0), message);
    }
  }
  else
    NIY
   
  sim->relaxPosition(message); // needed to have a observe correct subsequent state
  sim->simulate(secs_wait_after_action); // needed to have a correct subsequent state
}

















/* ---------------------
    LANGUAGE
  --------------------- */



// -------------------------
//   LOGIC ENGINE
// -------------------------

// #define CLEARANCE
// #define TOWER

void TL::RobotManipulationDomain::setupLogic(uintA& constants) {
  PredL p_state;
  // Primitives
  p_state.append(getPredicate_table());
  p_state.append(getPredicate_block());
  p_state.append(getPredicate_ball());
  p_state.append(getPredicate_on());
  p_state.append(getPredicate_inhand());
  p_state.append(getPredicate_upright());
  p_state.append(getPredicate_out());
  // Derived
  p_state.append(getPredicate_clear());
  p_state.append(getPredicate_inhandNil());
  
  FuncL f_state;
  // Primitive
  f_state.append(getFunction_size());
  
  PredL p_action;
  p_action.append(getPredicate_action_default());
  p_action.append(getPredicate_action_grab());
  p_action.append(getPredicate_action_puton());
//   p_action.append(getPredicate_action_lift());
//   p_action.append(getPredicate_action_place());

  
#ifdef CLEARANCE
  p_state.append(getPredicate_above());    // CLEARANCE
  f_state.append(getFunction_height());
#endif
  
#ifdef CLEARANCE
  p_state.append(getPredicate_homies());  // CLEARANCE
  p_state.append(getPredicate_above());    // CLEARANCE
  p_state.append(getPredicate_aboveNotable());    // CLEARANCE
  p_state.append(getPredicate_dirtyGuyBelow());    // CLEARANCE
  p_state.append(getPredicate_diffTower());   // CLEARANCE
  p_state.append(getPredicate_withoutHomies());    // CLEARANCE
  p_state.append(getPredicate_inorder());    // CLEARANCE
  
  f_state.append(getFunction_countInorder());    // CLEARANCE
#endif
  
#ifdef TOWER
  p_state.append(getPredicate_box());
  p_state.append(getPredicate_contains()); 
  p_state.append(getPredicate_closed()); 
  p_state.append(getPredicate_onBox());
  p_action.append(getPredicate_action_openBox());
  p_action.append(getPredicate_action_closeBox());
#endif
  
  TL::logicObjectManager::setConstants(constants);
  
  TL::logicObjectManager::addActionPredicates(p_action);
  TL::logicObjectManager::addStatePredicates(p_state);
  TL::logicObjectManager::addStateFunctions(f_state);
}








// ---------------
//  PRIMTIVES - PREDS
// ---------------


TL::Predicate* p_ON = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_on() {
  if (p_ON == NULL) {
    p_ON = new Predicate;
    p_ON->d = 2;
    p_ON->name = "on";
    p_ON->id = HAND_ID__PRED_ON;
  }
  return p_ON;
}

TL::Predicate* p_TABLE = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_table() {
  if (p_TABLE== NULL) {
    p_TABLE = new Predicate;
    p_TABLE->d = 1;
    p_TABLE->name = "table";
    p_TABLE->id = HAND_ID__PRED_TABLE;
  }
  return p_TABLE;
}

TL::Predicate* p_BLOCK = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_block() {
  if (p_BLOCK == NULL) {
    p_BLOCK = new Predicate;
    p_BLOCK->d = 1;
    p_BLOCK->name = "block";
    p_BLOCK->id = HAND_ID__PRED_BLOCK;
  }
  return p_BLOCK;
}

TL::Predicate* p_BOX = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_box() {
  if (p_BOX == NULL) {
    p_BOX = new Predicate;
    p_BOX->d = 1;
    p_BOX->name = "box";
    p_BOX->id = HAND_ID__PRED_BOX;
  }
  return p_BOX;
}

TL::Predicate* p_BALL = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_ball() {
  if (p_BALL == NULL) {
    p_BALL = new Predicate;
    p_BALL->d = 1;
    p_BALL->name = "ball";
    p_BALL->id = HAND_ID__PRED_BALL;
  }
  return p_BALL;
}

TL::Predicate* p_INHAND = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_inhand() {
  if (p_INHAND == NULL) {
    p_INHAND = new Predicate;
    p_INHAND->d = 1;
    p_INHAND->name = "inhand";
    p_INHAND->id = HAND_ID__PRED_INHAND;
  }
  return p_INHAND;
}

TL::Predicate* p_UPRIGHT = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_upright() {
  if (p_UPRIGHT == NULL) {
    p_UPRIGHT = new Predicate;
    p_UPRIGHT->d = 1;
    p_UPRIGHT->name = "upright";
    p_UPRIGHT->id = HAND_ID__PRED_UPRIGHT;
  }
  return p_UPRIGHT;
}

TL::Predicate* p_OUT = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_out() {
  if (p_OUT == NULL) {
    p_OUT = new Predicate;
    p_OUT->d = 1;
    p_OUT->name = "out";
    p_OUT->id = HAND_ID__PRED_OUT;
  }
  return p_OUT;
}

TL::Predicate* p_HOMIES = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_homies() {
  if (p_HOMIES == NULL) {
    p_HOMIES = new Predicate;
    p_HOMIES->d = 2;
    p_HOMIES->name = "homies";
    p_HOMIES->id = HAND_ID__PRED_HOMIES;
  }
  return p_HOMIES;
}

TL::Predicate* p_CONTAINS = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_contains() {
  if (p_CONTAINS == NULL) {
    p_CONTAINS = new Predicate;
    p_CONTAINS->d = 2;
    p_CONTAINS->name = "contains";
    p_CONTAINS->id = HAND_ID__PRED_CONTAINS;
  }
  return p_CONTAINS;
}

TL::Predicate* p_CLOSED = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_closed() {
  if (p_CLOSED == NULL) {
    p_CLOSED = new Predicate;
    p_CLOSED->d = 1;
    p_CLOSED->name = "closed";
    p_CLOSED->id = HAND_ID__PRED_CLOSED;
  }
  return p_CLOSED;
}






// ---------------
//  ACTIONS
// ---------------


TL::Predicate* p_ACTION_DEFAULT = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_action_default() {
  if (p_ACTION_DEFAULT == NULL) {
    p_ACTION_DEFAULT = new Predicate;
    p_ACTION_DEFAULT->d = 0;
    p_ACTION_DEFAULT->name = "default";
    p_ACTION_DEFAULT->id = TL::DEFAULT_ACTION_PRED__ID;
    p_ACTION_DEFAULT->type = TL::Predicate::predicate_action;
  }
  return p_ACTION_DEFAULT;
}

TL::Predicate* p_GRAB = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_action_grab() {
  if (p_GRAB == NULL) {
    p_GRAB = new Predicate;
    p_GRAB->d = 1;
    p_GRAB->name = "grab";
    p_GRAB->id = HAND_ID__GRAB;
    p_GRAB->type = TL::Predicate::predicate_action;
  }
  return p_GRAB;
}

TL::Predicate* p_PUTON = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_action_puton() {
  if (p_PUTON == NULL) {
    p_PUTON = new Predicate;
    p_PUTON->d = 1;
    p_PUTON->name = "puton";
    p_PUTON->id = HAND_ID__PUTON;
    p_PUTON->type = TL::Predicate::predicate_action;
  }
  return p_PUTON;
}

TL::Predicate* p_LIFT = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_action_lift() {
  if (p_LIFT == NULL) {
    p_LIFT = new Predicate;
    p_LIFT->d = 2;
    p_LIFT->name = "lift";
    p_LIFT->id = HAND_ID__LIFT;
    p_LIFT->type = TL::Predicate::predicate_action;
  }
  return p_LIFT;
}

TL::Predicate* p_PLACE = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_action_place() {
  if (p_PLACE == NULL) {
    p_PLACE = new Predicate;
    p_PLACE->d = 2;
    p_PLACE->name = "place";
    p_PLACE->id = HAND_ID__PLACE;
    p_PLACE->type = TL::Predicate::predicate_action;
  }
  return p_PLACE;
}

TL::Predicate* p_OPEN_BOX = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_action_openBox() {
  if (p_OPEN_BOX == NULL) {
    p_OPEN_BOX= new Predicate;
    p_OPEN_BOX->d = 1;
    p_OPEN_BOX->name = "openBox";
    p_OPEN_BOX->id = HAND_ID__OPEN_BOX;
    p_OPEN_BOX->type = TL::Predicate::predicate_action;
  }
  return p_OPEN_BOX;
}

TL::Predicate* p_CLOSE_BOX = NULL;
TL::Predicate* TL::RobotManipulationDomain::getPredicate_action_closeBox() {
  if (p_CLOSE_BOX == NULL) {
    p_CLOSE_BOX = new Predicate;
    p_CLOSE_BOX->d = 1;
    p_CLOSE_BOX->name = "closeBox";
    p_CLOSE_BOX->id = HAND_ID__CLOSE_BOX;
    p_CLOSE_BOX->type = TL::Predicate::predicate_action;
  }
  return p_CLOSE_BOX;
}





// ---------------
//  PRIMTIIVES - FUNCTIONS
// ---------------

TL::Function* f_SIZE = NULL;
TL::Function* TL::RobotManipulationDomain::getFunction_size() {
  if (f_SIZE==NULL) {
    f_SIZE = new Function;
    f_SIZE->d = 1;
    f_SIZE->name = "size";
    f_SIZE->id = HAND_ID__FUNCTION_SIZE;
  }
  return f_SIZE;
}







// ---------------
//  DERIVED - PREDICATES
// ---------------

TL::ConjunctionPredicate* p_CLEAR = NULL;
TL::ConjunctionPredicate* TL::RobotManipulationDomain::getPredicate_clear() {
  if (p_CLEAR == NULL) {
    p_CLEAR = new TL::ConjunctionPredicate;
    p_CLEAR->d = 1;
    p_CLEAR->name = "clear";
    p_CLEAR->id = HAND_ID__PRED_CLEAR;
    p_CLEAR->basePreds.append(getPredicate_on());
    p_CLEAR->basePreds_positive.append(false);
    p_CLEAR->basePreds_mapVars2conjunction.resize(2);
    p_CLEAR->basePreds_mapVars2conjunction(0) = p_CLEAR->d;
    p_CLEAR->basePreds_mapVars2conjunction(1) = 0;
    p_CLEAR->freeVarsAllQuantified = true;
  }
  return p_CLEAR;
}

TL::TransClosurePredicate* p_ABOVE = NULL;
TL::TransClosurePredicate* TL::RobotManipulationDomain::getPredicate_above() {
  if (p_ABOVE == NULL) {
    p_ABOVE = new TL::TransClosurePredicate;
    p_ABOVE->d = 2;
    p_ABOVE->name = "above";
    p_ABOVE->id = HAND_ID__PRED_ABOVE;
    p_ABOVE->basePred = getPredicate_on();
  }
  return p_ABOVE;
}


TL::ConjunctionPredicate* p_ABOVE_NOTABLE = NULL;
TL::ConjunctionPredicate* TL::RobotManipulationDomain::getPredicate_aboveNotable() {
  if (p_ABOVE_NOTABLE == NULL) {
    p_ABOVE_NOTABLE = new TL::ConjunctionPredicate;
    p_ABOVE_NOTABLE->d = 2;
    p_ABOVE_NOTABLE->name = "aboveNotable";
    p_ABOVE_NOTABLE->id = HAND_ID__PRED_ABOVE_NOTABLE;
    p_ABOVE_NOTABLE->basePreds.append(getPredicate_above());
    p_ABOVE_NOTABLE->basePreds.append(getPredicate_table());
    p_ABOVE_NOTABLE->basePreds_positive.append(true);
    p_ABOVE_NOTABLE->basePreds_positive.append(false);
    p_ABOVE_NOTABLE->basePreds_mapVars2conjunction.resize(3);
    p_ABOVE_NOTABLE->basePreds_mapVars2conjunction(0) = 0;
    p_ABOVE_NOTABLE->basePreds_mapVars2conjunction(1) = 1;
    p_ABOVE_NOTABLE->basePreds_mapVars2conjunction(2) = 1;
  }
  return p_ABOVE_NOTABLE;
}


TL::ConjunctionPredicate* p_DIRTY_GUY_BELOW = NULL;
TL::ConjunctionPredicate* TL::RobotManipulationDomain::getPredicate_dirtyGuyBelow() {
  if (p_DIRTY_GUY_BELOW == NULL) {
    p_DIRTY_GUY_BELOW= new TL::ConjunctionPredicate;
    p_DIRTY_GUY_BELOW->name = "dirtyGuyBelow";
    p_DIRTY_GUY_BELOW->id = HAND_ID__PRED_DIRTY_GUY_BELOW;
    p_DIRTY_GUY_BELOW->d = 1;
    p_DIRTY_GUY_BELOW->freeVarsAllQuantified = false;
    p_DIRTY_GUY_BELOW->basePreds.resize(3);
    p_DIRTY_GUY_BELOW->basePreds(0) = getPredicate_table();
    p_DIRTY_GUY_BELOW->basePreds(1) = getPredicate_homies();
    p_DIRTY_GUY_BELOW->basePreds(2) = getPredicate_on();
    p_DIRTY_GUY_BELOW->basePreds_mapVars2conjunction.resize(5);
    p_DIRTY_GUY_BELOW->basePreds_mapVars2conjunction(0) = 1;
    p_DIRTY_GUY_BELOW->basePreds_mapVars2conjunction(1) = 0;
    p_DIRTY_GUY_BELOW->basePreds_mapVars2conjunction(2) = 1;
    p_DIRTY_GUY_BELOW->basePreds_mapVars2conjunction(3) = 0;
    p_DIRTY_GUY_BELOW->basePreds_mapVars2conjunction(4) = 1;
    p_DIRTY_GUY_BELOW->basePreds_positive.resize(3);
    p_DIRTY_GUY_BELOW->basePreds_positive(0) = false;
    p_DIRTY_GUY_BELOW->basePreds_positive(1) = false;
    p_DIRTY_GUY_BELOW->basePreds_positive(2) = true;
  }
  return p_DIRTY_GUY_BELOW;
}





TL::ConjunctionPredicate* p_DIFF_TOWER = NULL;
TL::ConjunctionPredicate* TL::RobotManipulationDomain::getPredicate_diffTower() {
  if (p_DIFF_TOWER == NULL) {
    p_DIFF_TOWER = new TL::ConjunctionPredicate;
    p_DIFF_TOWER->name = "diffTower";
    p_DIFF_TOWER->id = HAND_ID__PRED_DIFF_TOWER;
    p_DIFF_TOWER->d = 2;
    p_DIFF_TOWER->basePreds.resize(2);
    p_DIFF_TOWER->basePreds(0) = getPredicate_above();
    p_DIFF_TOWER->basePreds(1) = getPredicate_above();
    p_DIFF_TOWER->basePreds_mapVars2conjunction.resize(4);
    p_DIFF_TOWER->basePreds_mapVars2conjunction(0) = 0;
    p_DIFF_TOWER->basePreds_mapVars2conjunction(1) = 1;
    p_DIFF_TOWER->basePreds_mapVars2conjunction(2) = 1;
    p_DIFF_TOWER->basePreds_mapVars2conjunction(3) = 0;
    p_DIFF_TOWER->basePreds_positive.resize(2);
    p_DIFF_TOWER->basePreds_positive(0) = false;
    p_DIFF_TOWER->basePreds_positive(1) = false;
    p_DIFF_TOWER->freeVarsAllQuantified = false;
  }
  return p_DIFF_TOWER;
}


TL::ConjunctionPredicate* p_WITHOUT_HOMIES = NULL;
TL::ConjunctionPredicate* TL::RobotManipulationDomain::getPredicate_withoutHomies() {
  if (p_WITHOUT_HOMIES == NULL) {
    p_WITHOUT_HOMIES = new TL::ConjunctionPredicate;
    p_WITHOUT_HOMIES->name = "withoutHomies";
    p_WITHOUT_HOMIES->id = HAND_ID__PRED_WITHOUT_HOMIES;
    p_WITHOUT_HOMIES->d = 1;
    p_WITHOUT_HOMIES->basePreds.resize(2);
    p_WITHOUT_HOMIES->basePreds(0) = getPredicate_homies();
    p_WITHOUT_HOMIES->basePreds(1) = getPredicate_diffTower();
    p_WITHOUT_HOMIES->basePreds_mapVars2conjunction.resize(4);
    p_WITHOUT_HOMIES->basePreds_mapVars2conjunction(0) = 0;
    p_WITHOUT_HOMIES->basePreds_mapVars2conjunction(1) = 1;
    p_WITHOUT_HOMIES->basePreds_mapVars2conjunction(2) = 0;
    p_WITHOUT_HOMIES->basePreds_mapVars2conjunction(3) = 1;
    p_WITHOUT_HOMIES->basePreds_positive.resize(2);
    p_WITHOUT_HOMIES->basePreds_positive(0) = true;
    p_WITHOUT_HOMIES->basePreds_positive(1) = true;
    p_WITHOUT_HOMIES->freeVarsAllQuantified = false;
  }
  return p_WITHOUT_HOMIES;
}



TL::ConjunctionPredicate* p_INORDER = NULL;
TL::ConjunctionPredicate* TL::RobotManipulationDomain::getPredicate_inorder() {
  if (p_INORDER == NULL) {
    p_INORDER = new TL::ConjunctionPredicate;
    p_INORDER->name = "inorder";
    p_INORDER->id = HAND_ID__PRED_INORDER;
    p_INORDER->d = 1;
    p_INORDER->basePreds.resize(3);
    p_INORDER->basePreds(0) = getPredicate_table();
    p_INORDER->basePreds(1) = getPredicate_withoutHomies();
    p_INORDER->basePreds(2) = getPredicate_dirtyGuyBelow();
    p_INORDER->basePreds_mapVars2conjunction.resize(3);
    p_INORDER->basePreds_mapVars2conjunction(0) = 0;
    p_INORDER->basePreds_mapVars2conjunction(1) = 0;
    p_INORDER->basePreds_mapVars2conjunction(2) = 0;
    p_INORDER->basePreds_positive.resize(3);
    p_INORDER->basePreds_positive(0) = false;
    p_INORDER->basePreds_positive(1) = false;
    p_INORDER->basePreds_positive(2) = false;
  }
  return p_INORDER;
}


TL::ConjunctionPredicate* p_INHAND_NIL = NULL;
TL::ConjunctionPredicate* TL::RobotManipulationDomain::getPredicate_inhandNil() {
  if (p_INHAND_NIL == NULL) {
    p_INHAND_NIL = new TL::ConjunctionPredicate;
    p_INHAND_NIL->name = "inhandNil";
    p_INHAND_NIL->id = HAND_ID__PRED_INHAND_NIL;
    p_INHAND_NIL->d = 0;
    p_INHAND_NIL->basePreds.resize(1);
    p_INHAND_NIL->basePreds(0) = getPredicate_inhand();
    p_INHAND_NIL->basePreds_mapVars2conjunction.resize(1);
    p_INHAND_NIL->basePreds_mapVars2conjunction(0) = 0;
    p_INHAND_NIL->basePreds_positive.resize(1);
    p_INHAND_NIL->basePreds_positive(0) = false;
    p_INHAND_NIL->freeVarsAllQuantified = true;
  }
  return p_INHAND_NIL;
}



TL::ConjunctionPredicate* p_ON_BOX = NULL;
TL::ConjunctionPredicate* TL::RobotManipulationDomain::getPredicate_onBox() {
  if (p_ON_BOX == NULL) {
    p_ON_BOX = new ConjunctionPredicate;
    p_ON_BOX->d = 1;
    p_ON_BOX->name = "onBox";
    p_ON_BOX->id = HAND_ID__PRED_ON_BOX;
    p_ON_BOX->basePreds.resize(2);
    p_ON_BOX->basePreds(0) = getPredicate_on();
    p_ON_BOX->basePreds(1) = getPredicate_box();
    p_ON_BOX->basePreds_mapVars2conjunction.resize(3);
    p_ON_BOX->basePreds_mapVars2conjunction(0) = 0;
    p_ON_BOX->basePreds_mapVars2conjunction(1) = 1;
    p_ON_BOX->basePreds_mapVars2conjunction(2) = 1;
    p_ON_BOX->basePreds_positive.resize(2);
    p_ON_BOX->basePreds_positive(0) = true;
    p_ON_BOX->basePreds_positive(1) = true;
    p_ON_BOX->freeVarsAllQuantified = false;
  }
  return p_ON_BOX;
}






// ---------------
//  DERIVED - FUNCTIONS
// ---------------

TL::CountFunction* f_HEIGHT = NULL;
TL::CountFunction* TL::RobotManipulationDomain::getFunction_height() {
  if (f_HEIGHT == NULL) {
    f_HEIGHT = new TL::CountFunction;
    f_HEIGHT->d = 1;
    f_HEIGHT->name = "height";
    f_HEIGHT->id = HAND_ID__FUNCTION_HEIGHT;
    f_HEIGHT->countedPred = getPredicate_aboveNotable();
    f_HEIGHT->countedPred_mapVars2derived.append(0);
    f_HEIGHT->countedPred_mapVars2derived.append(f_HEIGHT->d);
  }
  return f_HEIGHT;
}

TL::AverageFunction* f_AVG_HEIGHT = NULL;
TL::AverageFunction* TL::RobotManipulationDomain::getFunction_avgheight() {
  if (f_AVG_HEIGHT == NULL) {
    f_AVG_HEIGHT = new TL::AverageFunction;
    f_AVG_HEIGHT->d = 0;
    f_AVG_HEIGHT->name = "avg_height";
    f_AVG_HEIGHT->id = HAND_ID__FUNCTION_AVG_HEIGHT;
    f_AVG_HEIGHT->f_base = getFunction_height();
  }
  return f_AVG_HEIGHT;
}

TL::SumFunction* f_SUM_HEIGHT = NULL;
TL::SumFunction* TL::RobotManipulationDomain::getFunction_sumheight() {
  if (f_SUM_HEIGHT == NULL) {
    f_SUM_HEIGHT = new TL::SumFunction;
    f_SUM_HEIGHT->id = HAND_ID__FUNCTION_SUM_HEIGHT;
    f_SUM_HEIGHT->d = 0;
    f_SUM_HEIGHT->name = "sum_height";
    f_SUM_HEIGHT->f_base = getFunction_height();
  }
  return f_SUM_HEIGHT;
}


TL::CountFunction* f_COUNT_INORDER = NULL;
TL::CountFunction* TL::RobotManipulationDomain::getFunction_countInorder() {
  if (f_COUNT_INORDER == NULL) {
    f_COUNT_INORDER= new TL::CountFunction;
    f_COUNT_INORDER->id = HAND_ID__FUNCTION_COUNT_INORDER;
    f_COUNT_INORDER->name = "count_inorder";
    f_COUNT_INORDER->d = 0;
    uint i;
    for (i=0;i<20;i++) {f_COUNT_INORDER->range.append(i);}
    f_COUNT_INORDER->countedPred = getPredicate_inorder();
    f_COUNT_INORDER->countedPred_mapVars2derived.resize(1);
    f_COUNT_INORDER->countedPred_mapVars2derived(0) = 0;
  }
  return f_COUNT_INORDER;
}


    
	



// -------------------------
//   CONCEPT MANAGEMENT
// -------------------------

void TL::RobotManipulationDomain::shutdownLogic() {
  if (p_ON != NULL)
    delete p_ON;
  NIY;
}













// -----------------------------------
//   PREDICATE TUPLES
// -----------------------------------


TL::FunctionValue* TL::RobotManipulationDomain::createFunctionValue_size(uint obj, double size) {
  uintA sa(1);  sa(0)=obj;
  return logicObjectManager::getFV(logicObjectManager::getFunction(MT::String("size")), sa, size);
}
	


TL::Literal* TL::RobotManipulationDomain::createLiteral_on(uint above, uint below) {
  uintA sa(2);  sa(0)=above;  sa(1)=below;
  return logicObjectManager::getLiteral(logicObjectManager::getPredicate(MT::String("on")), true, sa);
}

TL::Literal* TL::RobotManipulationDomain::createLiteral_table(uint obj) {
  uintA sa(1);  sa(0)=obj;
  return logicObjectManager::getLiteral(logicObjectManager::getPredicate(MT::String("table")), true, sa);
}

TL::Literal* TL::RobotManipulationDomain::createLiteral_block(uint obj) {
  uintA sa(1);  sa(0)=obj;
  return logicObjectManager::getLiteral(logicObjectManager::getPredicate(MT::String("block")), true, sa);
}

TL::Literal* TL::RobotManipulationDomain::createLiteral_ball(uint obj) {
  uintA sa(1);  sa(0)=obj;
  return logicObjectManager::getLiteral(logicObjectManager::getPredicate(MT::String("ball")), true, sa);
}

TL::Literal* TL::RobotManipulationDomain::createLiteral_box(uint obj) {
  uintA sa(1);  sa(0)=obj;
  return logicObjectManager::getLiteral(logicObjectManager::getPredicate(MT::String("box")), true, sa);
}

TL::Literal* TL::RobotManipulationDomain::createLiteral_upright(uint obj) {
  uintA sa(1);  sa(0)=obj;
  return logicObjectManager::getLiteral(logicObjectManager::getPredicate(MT::String("upright")), true, sa);
}

TL::Literal* TL::RobotManipulationDomain::createLiteral_out(uint obj) {
  uintA sa(1);  sa(0)=obj;
  return logicObjectManager::getLiteral(logicObjectManager::getPredicate(MT::String("out")), true, sa);
}

TL::Literal* TL::RobotManipulationDomain::createLiteral_inhand(uint obj) {
  uintA sa(1);  sa(0)=obj;
  return logicObjectManager::getLiteral(logicObjectManager::getPredicate(MT::String("inhand")), true, sa);
}

TL::Literal* TL::RobotManipulationDomain::createLiteral_homies(uint obj1, uint obj2) {
  uintA sa(2);  sa(0)=obj1;  sa(1)=obj2;
  return logicObjectManager::getLiteral(logicObjectManager::getPredicate(MT::String("homies")), true, sa);
}

TL::Literal* TL::RobotManipulationDomain::createLiteral_contains(uint box, uint obj)  {
  uintA sa(2);  sa(0)=box;  sa(1)=obj;
  return logicObjectManager::getLiteral(logicObjectManager::getPredicate(MT::String("contains")), true, sa);
}

TL::Literal* TL::RobotManipulationDomain::createLiteral_closed(uint box)  {
  uintA sa(1);  sa(0)=box;
  return logicObjectManager::getLiteral(logicObjectManager::getPredicate(MT::String("closed")), true, sa);
}


TL::Literal* TL::RobotManipulationDomain::createLiteral_grab(uint obj) {
  uintA sa(1);  sa(0)=obj;
  return logicObjectManager::getLiteral(logicObjectManager::getPredicate(MT::String("grab")), true, sa);
}


TL::Literal* TL::RobotManipulationDomain::createLiteral_puton(uint obj, uint on) {
  uintA sa(1);  sa(0)=obj;
  return logicObjectManager::getLiteral(logicObjectManager::getPredicate(MT::String("puton")), true, sa);
}











// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    GOAL LIBRARY
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------



TL::Reward* TL::RobotManipulationDomain::RewardLibrary::on(uint o1, uint o2) {
  TL::Predicate* p_ON = logicObjectManager::getPredicate(MT::String("on"));
  uintA sa2(2);
  sa2(0)=o1;
  sa2(1)=o2;
  TL::Literal* pt = logicObjectManager::getLiteral(p_ON, true, sa2);
  Reward* reward = new LiteralReward(pt);
  return reward;
}

TL::Reward* TL::RobotManipulationDomain::RewardLibrary::inhand(uint o1) {
  TL::Predicate* p_INHAND = logicObjectManager::getPredicate(MT::String("inhand"));
  uintA sa1(1);
  sa1(0)=o1;
  TL::Literal* pt = logicObjectManager::getLiteral(p_INHAND, true, sa1);
  Reward* reward = new LiteralReward(pt);
  return reward;
}

TL::Reward* TL::RobotManipulationDomain::RewardLibrary::stack() {
  // needs p_ABOVE, f_HEIGHT, f_SUM_HEIGHT
  MT_MSG("Stack defined by sum (not max) over heights");
  uintA empty;
  FuncL funcs2add;
  PredL preds2add;
  if (!logicObjectManager::getPredicate(MT::String("above"))) {
    TL::TransClosurePredicate* p_ABOVE1 = getPredicate_above();
    p_ABOVE1->basePred = logicObjectManager::getPredicate(MT::String("on")); // HACK da in regelfiles bis juni 2009 on andere id hat
    preds2add.append(p_ABOVE1);
  }
  if (!logicObjectManager::getPredicate(MT::String("aboveNotable"))) {
    preds2add.append(getPredicate_aboveNotable());
  }
  if (!logicObjectManager::getFunction(MT::String("height"))) {
    funcs2add.append(getFunction_height());
  }
  if (!logicObjectManager::getFunction(MT::String("sum_height"))) {
    funcs2add.append(getFunction_sumheight());
  }
  if (preds2add.N > 0)
    logicObjectManager::addStatePredicates(preds2add);
  if (funcs2add.N > 0)
    logicObjectManager::addStateFunctions(funcs2add);
  TL::FunctionAtom* fa = logicObjectManager::getFA(logicObjectManager::getFunction(MT::String("sum_height")), empty);
  Reward* reward = new MaximizeFunctionReward(fa);
  return reward;
}


TL::Reward* TL::RobotManipulationDomain::RewardLibrary::tower(uintA& objs) {
  TL::Predicate* p_ON = logicObjectManager::getPredicate(MT::String("on"));
  LitL pts;
  uint i;
  uintA sa2(2);
  FOR1D(objs, i) {
    sa2(0)=objs(i);
    if (i<objs.N-1)
      sa2(1)=objs(i+1);
    else
      sa2(1)=60;  // table id in my ors simulator
    pts.append(logicObjectManager::getLiteral(p_ON, true, sa2));
  }
  LiteralListReward* reward = new LiteralListReward(pts);
  return reward;
}


TL::Reward* TL::RobotManipulationDomain::RewardLibrary::clearance() {
  FuncL funcs2add;
  PredL preds2add;
  if (!logicObjectManager::getPredicate(MT::String("above"))) {
    TL::TransClosurePredicate* p_ABOVE1 = getPredicate_above();
    p_ABOVE1->basePred = logicObjectManager::getPredicate(MT::String("on")); // HACK da in regelfiles bis juni 2009 on andere id hat
    preds2add.append(p_ABOVE1);
  }
  if (!logicObjectManager::getPredicate(MT::String("dirtyGuyBelow"))) {
    preds2add.append(getPredicate_dirtyGuyBelow());
  }
  if (!logicObjectManager::getPredicate(MT::String("differentTower"))) {
    preds2add.append(getPredicate_diffTower());
  }
  if (!logicObjectManager::getPredicate(MT::String("withoutHomies"))) {
    preds2add.append(getPredicate_withoutHomies());
  }
  if (!logicObjectManager::getPredicate(MT::String("inorder"))) {
    preds2add.append(getPredicate_inorder());
  }
  if (!logicObjectManager::getFunction(MT::String("count_inorder"))) {
    funcs2add.append(getFunction_countInorder());
  }
  
  logicObjectManager::addStatePredicates(preds2add);
  logicObjectManager::addStateFunctions(funcs2add);
  
//   logicObjectManager::dependencyGraph.writeNice();
  uintA empty;
  TL::FunctionAtom* fi = logicObjectManager::getFA(getFunction_countInorder(), empty);
  Reward* reward = new MaximizeFunctionReward(fi);
  return reward;
}



TL::LiteralListReward* TL::RobotManipulationDomain::sampleGroundGoal__stack(const uintA& blocks, const uintA& balls, uint table_id, bool leave_existing_towers, TL::State* state) {
  uint DEBUG = 1;
  if (DEBUG>0) {cout<<"TL::RobotManipulationDomain::sampleGroundGoal__stack [START]"<<endl;}
  if (DEBUG>1) {PRINT(blocks);  PRINT(balls);  PRINT(table_id);}
  uintA objs;
  objs.append(blocks);
  objs.append(balls);
  
  // -----------------------------------------------------------------------------
  // (1) Determine heights of towers
  
  arr weights_tower_heights;
  uint i, k;
  double basis = 1.2;
//   if (objs.N >= 10)
//     basis = 1.1;
  
  for (i=2; /*i<objs.N*/ i<5; i++) {
    weights_tower_heights.append(pow(basis, i-2));
  }
  weights_tower_heights /= sum(weights_tower_heights);
  uint target_height = basic_sample(weights_tower_heights) + 2;
  if (DEBUG>1) {PRINT(weights_tower_heights);  PRINT(target_height);}
  
  
  // -----------------------------------------------------------------------------
  // (2) Fill towers with objects
  
  MT::Array< uintA > towers;
  if (!leave_existing_towers) {
    if (DEBUG>0) {cout<<"Dumb sampling"<<endl;}
    for (i=0; (i+1)*target_height <= objs.N; i++) {
      uintA tower;
      for (k=0; k<target_height; k++) {
        uint block;
        do {
          block = rnd.num(objs.N);
        }
        while (objs(block) == UINT_NIL);
        tower.append(objs(block));
        objs(block) = UINT_NIL;
      }
      tower.insert(0, table_id);
      towers.append(tower);
    }
    
    uintA last_tower;
    last_tower.append(table_id);
  
    FOR1D(objs, i) {
      if (objs(i) != UINT_NIL)
        last_tower.append(objs(i));
    }
    if (last_tower.N > 1) {
      towers.append(last_tower);
    }
  }
  else {
    if (DEBUG>0) {cout<<"Preserving sampling"<<endl;}
    uintA piles;
    calcPiles(*state, piles, table_id);
    if (DEBUG>1) {PRINT(piles);}
    // leave high towers
    uintA used_objects;
    for (i=0; i<piles.d0; i++) {
      k=0;
      while (k < piles.d1 && piles(i,k) != UINT_MAX) {
        k++;
      }
      if (k > 2) {
        uintA true_tower;
        k = 0;
        while (k < piles.d1 && piles(i,k) != UINT_MAX) {
          true_tower.append(piles(i,k));
          used_objects.setAppend(piles(i,k));
          k++;
        }
        towers.append(true_tower);
      }
    }
    
    // randomly split other objects on top of these
    uintA remaining_objs;
    remaining_objs = objs;
    setMinus<uint>(remaining_objs, used_objects);
    if (DEBUG>1) {PRINT(used_objects);  PRINT(remaining_objs);}
    while (remaining_objs.N > 0) {
      uint obj = remaining_objs(rnd.num(remaining_objs.N));
      // lower chance for balls
      if (balls.findValue(obj) >= 0) {
        obj = remaining_objs(rnd.num(remaining_objs.N));
        if (balls.N >= 3 && balls.findValue(obj) >= 0) {
          obj = remaining_objs(rnd.num(remaining_objs.N));
            if (balls.N >= 4 && balls.findValue(obj) >= 0) {
              obj = remaining_objs(rnd.num(remaining_objs.N));
            }
        }
      }
      FOR1D(towers, i) {
        if (towers(i).N - 1 < target_height) {
          towers(i).append(obj);
          break;
        }
      }
      if (i == towers.N) {
        uintA new_tower;
        new_tower.append(table_id);
        new_tower.append(obj);
        towers.append(new_tower);
        if (DEBUG>1) {PRINT(new_tower);}
      }
      remaining_objs.removeValue(obj);
    }
  }

  if (DEBUG>0) {
    cout<<"****************"<<endl;
    cout<<"TOWERS:"<<endl;
    FOR1D(towers, i) {
      FOR1D(towers(i), k) {
        cout<<" ";
        cout<<towers(i)(k);
        if (balls.findValue(towers(i)(k)) >= 0) {
          cout<<"o";
        }
      }
      cout << endl;
    }
    cout<<"****************"<<endl;
    
    cerr<<"****************"<<endl;
    cerr<<"TOWERS:"<<endl;
    FOR1D(towers, i) {
      FOR1D(towers(i), k) {
        cerr<<" ";
        cerr<<towers(i)(k);
        if (balls.findValue(towers(i)(k)) >= 0) {
          cerr<<"o";
        }
      }
      cerr << endl;
    }
    cerr<<"****************"<<endl;
  }
  
  // -----------------------------------------------------------------------------
  // (3) Translate to logic
  
  LitL lits;
  FOR1D(towers, i) {
    FOR1D(towers(i), k) {
      if (k==0)
        continue;
      lits.append(createLiteral_on(towers(i)(k), towers(i)(k-1)));
    }
  }
  
  if (DEBUG>0) {cout<<"LOGIC:"<<endl;  TL::write(lits); cout<<endl;}
  
  if (DEBUG>0) {cout<<"TL::RobotManipulationDomain::sampleGroundGoal__stack [END]"<<endl;}
  return new LiteralListReward(lits);
}









TL::LiteralListReward* TL::RobotManipulationDomain::sampleGroundGoal__clearance(const TL::State& current_state, uint table_id) {
  uint DEBUG = 1;
  if (DEBUG>0) {cout<<"TL::RobotManipulationDomain::sampleGroundGoal__clearance [START]"<<endl;}

  uint i, k;

  //-----------------------------------------------------------------------------
  // (1) Determine classes to be ordered
  
  MT::Array< uintA > gangs;
  getHomieGangs(gangs, current_state);

  // Determine gangs (= gangs of homies)
  if (DEBUG>1) {
    cout<<"GANGS:"<<endl;
    FOR1D(gangs, i) {
      cout << i << ":  " << gangs(i) << endl;
    }
  }
  
  uintA gang_ids;
  FOR1D(gangs, i) {
    if (!isInorderGang(gangs(i), current_state))
      gang_ids.append(i);
  }
  MT::Array< uintA > subsets;
  allSubsets(subsets, gang_ids, false, false);
  
  arr sample_weights(subsets.N);
  FOR1D(subsets, i) {
    sample_weights(i) = subsets(i).N;
    if (sample_weights(i) > 2) // HACK 
      sample_weights(i) = 2;
  }
  sample_weights /= sum(sample_weights);
  
  uintA combo = subsets(basic_sample(sample_weights));
  
  
  if (DEBUG>1) {
    PRINT(subsets);
    PRINT(sample_weights);
    PRINT(combo);
  }
  
  //-----------------------------------------------------------------------------
  // (2) Stack them together
  
  MT::Array< uintA > towers;
  FOR1D(combo, i) {
    uintA tower;
    uintA& local_gang = gangs(combo(i));
    FOR1D(local_gang, k) {
      uint candidate_position = rnd.num(tower.N+1);
      // lower chance for balls to be inside
//       if (isBall(local_gang(k), current_state)  &&  candidate_position != tower.N)
//         candidate_position = rnd.num(tower.N+1);
      tower.insert(candidate_position, local_gang(k));
    }
    tower.insert(0, table_id);
    towers.append(tower);
  }
  
  if (DEBUG>0) {
    cout<<"****************"<<endl;
    cout<<"TOWERS:"<<endl;
    FOR1D(towers, i) {
      FOR1D(towers(i), k) {
        cout<<" ";
        cout<<towers(i)(k);
        if (isBall(towers(i)(k), current_state)) {
          cout<<"o";
        }
      }
      cout << endl;
    }
    cout<<"****************"<<endl;
    
    cerr<<"****************"<<endl;
    cerr<<"TOWERS:"<<endl;
    FOR1D(towers, i) {
      FOR1D(towers(i), k) {
        cerr<<" ";
        cerr<<towers(i)(k);
        if (isBall(towers(i)(k), current_state)) {
          cerr<<"o";
        }
      }
      cerr << endl;
    }
    cerr<<"****************"<<endl;
  }
  

  //-----------------------------------------------------------------------------
  // (3) Translate to logic
  LitL lits;
  FOR1D(towers, i) {
    FOR1D(towers(i), k) {
      if (k==0)
        continue;
      lits.append(createLiteral_on(towers(i)(k), towers(i)(k-1)));
    }
  }
  
  if (DEBUG>1) {cout<<"LOGIC:"<<endl;  TL::write(lits); cout<<endl;}
  
  if (DEBUG>0) {cout<<"TL::RobotManipulationDomain::sampleGroundGoal__clearance [END]"<<endl;}
  return new LiteralListReward(lits);
}

















// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    STATE INFORMATION HELPERS
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


bool TL::RobotManipulationDomain::isBlock(uint id, const TL::State& s) {
  return logicReasoning::holds_straight(id, MT::String("block"), s);
}


bool TL::RobotManipulationDomain::isOut(uint id, const TL::State& s) {
  return logicReasoning::holds_straight(id, MT::String("out"), s);
}

bool TL::RobotManipulationDomain::isInhand(uint id, const TL::State& s) {
  return logicReasoning::holds_straight(id, MT::String("inhand"), s);
}

bool TL::RobotManipulationDomain::isTable(uint id, const TL::State& s) {
  return logicReasoning::holds_straight(id, MT::String("table"), s);
}

bool TL::RobotManipulationDomain::isBall(uint id, const TL::State& s) {
  return logicReasoning::holds_straight(id, MT::String("ball"), s);
}

bool TL::RobotManipulationDomain::isBox(uint id, const TL::State& s) {
  if (logicObjectManager::getPredicate(MT::String("box")) == NULL)
    return false;
  return logicReasoning::holds_straight(id, MT::String("box"), s);
}

bool TL::RobotManipulationDomain::isClosed(uint box_id, const TL::State& s) {
  return logicReasoning::holds_straight(box_id, MT::String("closed"), s);
}

bool TL::RobotManipulationDomain::isInorderGang(const uintA gang, const TL::State& s) {
  CHECK(gang.N > 0, "");
  if (logicObjectManager::getPredicate(MT::String("inorder")) == NULL) {
    NIY;
  }
  else {
    return logicReasoning::holds_straight(gang(0), MT::String("inorder"), s);
  }
}




uint TL::RobotManipulationDomain::getBelow(uint id, const TL::State& s) {
  TL::Predicate* p_ON = logicObjectManager::getPredicate(MT::String("on"));
  uint i;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred == p_ON) {
      if (s.lits_prim(i)->atom->args(0) == id)
        return s.lits_prim(i)->atom->args(1);
    }
  }
  return UINT_MAX;
}

uint TL::RobotManipulationDomain::getAbove(uint id, const TL::State& s) {
  TL::Predicate* p_ON = logicObjectManager::getPredicate(MT::String("on"));
  uint i;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred == p_ON) {
      if (s.lits_prim(i)->atom->args(1) == id)
        return s.lits_prim(i)->atom->args(0);
    }
  }
  return UINT_MAX;
}

void TL::RobotManipulationDomain::getBelowObjects(uintA& ids, uint id_top, const TL::State& s) {
  logicReasoning::getRelatedObjects(ids, id_top, true, *logicObjectManager::getPredicate(MT::String("above")), s);
  /*
  ids.clear();
  TL::Predicate* p_ABOVE = ;
  uint i;
  FOR1D(s.pt_derived, i) {
  if (s.pt_derived(i)->pred == p_ABOVE) {
  if (s.pt_derived(i)->args(0) == id_top)
  ids.append(s.pt_derived(i)->args(1));
}
}*/
}

void TL::RobotManipulationDomain::getAboveObjects(uintA& ids, uint id_top, const TL::State& s) {
  logicReasoning::getRelatedObjects(ids, id_top, false, *logicObjectManager::getPredicate(MT::String("above")), s);
}

uint TL::RobotManipulationDomain::getInhand(const TL::State& s) {
  TL::Predicate* p_INHAND = logicObjectManager::getPredicate(MT::String("inhand"));
  uint i;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred == p_INHAND) {
      return s.lits_prim(i)->atom->args(0);
    }
  }
  return UINT_MAX;
}

void TL::RobotManipulationDomain::getBoxes(uintA& ids, const TL::State& s) {
  ids.clear();
  TL::Predicate* p_BOX = logicObjectManager::getPredicate(MT::String("box"));
  uint i;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred == p_BOX) {
      ids.append(s.lits_prim(i)->atom->args(0));
    }
  }
}


uint TL::RobotManipulationDomain::getContainingBox(uint obj_id, const TL::State& s) {
  TL::Predicate* p_CONTAINS = logicObjectManager::getPredicate(MT::String("contains"));
  uint i;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred == p_CONTAINS) {
      if (s.lits_prim(i)->atom->args(1) == obj_id)
        return s.lits_prim(i)->atom->args(0);
    }
  }
  return UINT_MAX;
}


uint TL::RobotManipulationDomain::getContainedObject(uint box_id, const TL::State& s) {
  TL::Predicate* p_CONTAINS = logicObjectManager::getPredicate(MT::String("contains"));
  uint i;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred == p_CONTAINS) {
      if (s.lits_prim(i)->atom->args(0) == box_id)
        return s.lits_prim(i)->atom->args(1);
    }
  }
  return UINT_MAX;
}


void TL::RobotManipulationDomain::getHomieGangs(MT::Array< uintA >& homieGangs, const TL::State& s) {
  homieGangs.clear();
  Predicate* p_HOMIES = logicObjectManager::getPredicate(MT::String("homies"));
  if (p_HOMIES == NULL)
    return;
  uint i, k;
  boolA constants_covered(logicObjectManager::constants.N);
  constants_covered.setUni(false);
  FOR1D(logicObjectManager::constants, i) {
    if (logicReasoning::holds_straight(logicObjectManager::constants(i), MT::String("table"), s))
      continue;
    if (constants_covered(i))
      continue;
    uintA homies;
    logicReasoning::getRelatedObjects(homies, logicObjectManager::constants(i), true, *p_HOMIES, s);
    homies.insert(0, logicObjectManager::constants(i));
    constants_covered(i) = true;
    FOR1D(homies, k) {
      constants_covered(logicObjectManager::constants.findValue(homies(k))) = true;
    }
    homieGangs.append(homies);
  }
}





// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
//      CALCULATING HEIGHTS
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------

double TL::RobotManipulationDomain::reward_buildTower(const State& s) {
  uint DEBUG=0;
  if (DEBUG>0) {cout<<"RobotManipulationDomain::reward_buildTower [START]"<<endl;}
  uintA piles;
  uint id_table = UINT_NIL;
  uint i;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred->name == "table"  &&  s.lits_prim(i)->positive) {
      id_table = s.lits_prim(i)->atom->args(0);
      break;
    }
  }
  CHECK(i<s.lits_prim.N, "table id not found");
  calcPiles(s, piles, id_table);
  uint height;
  for(height=0; height<piles.d1; height++) {
    if (piles(0, height)==UINT_NIL)
      break;
  }
  double reward = 1.0 * height;
  if (DEBUG>0) {
    PRINT(piles)
        PRINT(reward)
  }
  if (DEBUG>0) {cout<<"RobotManipulationDomain::reward_buildTower [END]"<<endl;}
  return reward;
}



void TL::RobotManipulationDomain::calcPiles(const State& s, uintA& piles, uint sort_type) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<" TL::RobotManipulationDomain::calcPiles [START]"<<endl;}
  if (DEBUG>0) {cout<<"STATE:  ";  s.write(cout, true);  cout<<endl;}
  // calc piles (unsorted)
  MT::Array< uintA > piles_unsorted;
  uint i;
  bool inserted;
  FOR1D(s.lits_prim, i) {
    // on(A,B)
    if (s.lits_prim(i)->atom->pred->id == HAND_ID__PRED_ON) {
      inserted = false;
      uint A_2_top;
      FOR1D(piles_unsorted, A_2_top) {
        if (piles_unsorted(A_2_top).N == 0)
          continue;
        // pile with [A, ..., top]  -->  put B below
        if (piles_unsorted(A_2_top)(0) == s.lits_prim(i)->atom->args(0)) {
          piles_unsorted(A_2_top).insert(0, s.lits_prim(i)->atom->args(1));
          inserted = true;
        }
      }
      uint table_2_B;
      FOR1D(piles_unsorted, table_2_B) {
        if (piles_unsorted(table_2_B).N == 0)
          continue;
        // pile with [table, lastBlock, ..., B]  -->  put A on top
        if (piles_unsorted(table_2_B).last() == s.lits_prim(i)->atom->args(1)) {
          if (inserted) {
            // when trying to insert a second time
            // find previous insertion point
            FOR1D(piles_unsorted, A_2_top) {
              if (piles_unsorted(A_2_top).N == 0)
                continue;
              // pile with [A, ..., top]  -->  put B below
              if (piles_unsorted(A_2_top)(0) == s.lits_prim(i)->atom->args(1)) {  // anderer check als oben, verdammt, da B ja jetzt schon eingefuegt!
                break;
              }
            }
            CHECK(A_2_top != piles_unsorted.N, "");
            piles_unsorted(table_2_B).setAppend(piles_unsorted(A_2_top)); // schmeisst doppeleintrag raus
            piles_unsorted(A_2_top).clear();
          }
          else {
            piles_unsorted(table_2_B).append(s.lits_prim(i)->atom->args(0));
            inserted = true;
          }
        }
      }
      if (!inserted) {
        uintA newPile;
        newPile.append(s.lits_prim(i)->atom->args(1));
        newPile.append(s.lits_prim(i)->atom->args(0));
        piles_unsorted.append(newPile);
      }
    }
  }
  
  MT::Array< uintA > piles_unsorted2;
  FOR1D(piles_unsorted, i) {
    if (piles_unsorted(i).N > 0) {
      piles_unsorted2.append(piles_unsorted(i));
    }
  }
  piles_unsorted = piles_unsorted2;
  
  uint j;
    
  if (DEBUG>0) {
    cout <<"piles_unsorted:"<<endl;
    FOR1D(piles_unsorted, i) {
      FOR1D(piles_unsorted(i), j) {
        cout << piles_unsorted(i)(j) << " ";
      }
      cout << endl;
    }
  }
  
  uintA heights;
  FOR1D(piles_unsorted, i) {
    heights.append(piles_unsorted(i).d0);
  }
  if (DEBUG>0) {
    FOR1D(heights, i) {
      PRINT(heights(i))
    }
  }
  uintA sortedIndices;
  sort_desc_keys(sortedIndices, heights); // descending
  piles.resize(piles_unsorted.d0, heights(sortedIndices(0)));
  piles.setUni(UINT_NIL);
  
  // sort by height
  if (sort_type == 1) {
    // reorder piles
    FOR1D(piles, i) {
      FOR1D(piles_unsorted(sortedIndices(i)), j) {
        piles(i,j) = piles_unsorted(sortedIndices(i))(j);
      }
    }
  }
  // sort by id
  else if (sort_type == 2) {
    uintA state_constants;
    logicReasoning::getConstants(s, state_constants);
    uint next_pile_id = 0;
    FOR1D(state_constants, i) {
      FOR1D(piles_unsorted, j) {
        if (piles_unsorted(j)(1) == state_constants(i)) {   // (1) for table
          uint k;
          FOR1D(piles_unsorted(j), k) {
            piles(next_pile_id,k) = piles_unsorted(j)(k);
          }
          next_pile_id++;
          break;
        }
      }
    }
    if (next_pile_id != piles_unsorted.N) {PRINT(piles_unsorted);  PRINT(piles);  PRINT(state_constants);  PRINT(next_pile_id);  HALT("resorting failed");}
  }
  else {
    FOR1D(piles_unsorted, i) {
      FOR1D(piles_unsorted(i), j) {
        piles(i,j) = piles_unsorted(i)(j);
      }
    }
  }
  
  if (DEBUG>0) {PRINT(piles);}
  if (DEBUG>0) {cout<<" TL::RobotManipulationDomain::calcPiles [END]"<<endl;}
}




// unfound objects get height = 0
void TL::RobotManipulationDomain::calcHeights(const uintA& objects, const uintA& piles, uintA& object_heights, uint id_table) {
  uint i, k, l;
  bool found;
  object_heights.resize(objects.N);
  FOR1D(objects, i) {
    found = false;
    FOR2D(piles, k, l) {
      if (piles(k,l)==objects(i)) {
        object_heights(i) = l+1;
        found = true;
        break;
      }
    }
    if (!found) {
      object_heights(i) = 0;
    }
  }
}


void TL::RobotManipulationDomain::calcSkyscraperWeights(const uintA& heights, double skyscraper_bias, arr& weights, bool highGood, uint id_table) {
  uint i;
  weights.resize(heights.N);
  weights.setUni(1.0);
  FOR1D(heights, i) {
    if (highGood)
      weights(i) += heights(i) * skyscraper_bias;
    else
      weights(i) -= heights(i) * skyscraper_bias;
  }
  if (!highGood) {
    double minValue = weights.min();
    FOR1D(weights, i) {
      weights(i) -= minValue - 1.0;
    }
  }
}

void TL::RobotManipulationDomain::calcSkyscraperWeights(const uintA& objects, const uintA& piles, double skyscraper_bias, arr& weights, bool highGood, uint id_table) {
  uintA object_heights;
//     PRINT(piles)
  calcHeights(objects, piles, object_heights, id_table);
  calcSkyscraperWeights(object_heights, skyscraper_bias, weights, highGood, id_table);
    // special care for table if putting on
  if (highGood) {
    uint i;
    FOR1D(objects, i) {
      if (objects(i)==id_table)
        break;
    }
    if (i<objects.N)
      weights(i) = skyscraper_bias;
  }
}






void TL::RobotManipulationDomain::writeStateInfo(const State& s, ostream& out) {
  out<<"--"<<endl;
  uint i, k;
  uint id_table = UINT_MAX;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred->name == "table") {
      id_table = s.lits_prim(i)->atom->args(0);
      break;
    }
  }
//   CHECK(id_table != UINT_MAX, "");
  
  // Piles
  uintA piles;
  calcPiles(s, piles, 2);
  
//   PRINT(piles);
  
  for (i=0; i<piles.d0; i++) {
    k = 0;
    while (k < piles.d1 && piles(i,k) != UINT_MAX) {
      if (k>0) out<<" ";
      if (isBox(piles(i,k), s))
        out << "b";
      out<<piles(i,k);
      if (isBall(piles(i,k), s))
        out<<"o";
      else if (isBox(piles(i,k), s)) {
        out<<"[ ";
        uint obj = getContainedObject(piles(i,k), s);
        if (obj != UINT_MAX) {
          out<<obj;
          if (isBall(obj, s))
            out<<"o";
          out<<" ";
        }
        if (isClosed(piles(i,k), s))
          out<<"]";
      }
      k++;
    }
    out << endl;
  }
  
  out<<"--"<<endl;
  
  out<<"H ";
  uint id_inhand = getInhand(s);
  if (id_inhand != UINT_MAX) {
    out<<id_inhand;
    if (isBall(id_inhand, s))
      out<<"o";
  }
  else
    out << "-";
  out<<endl;
  
  // Out objects
  uintA out_objects;
  logicReasoning::getArguments(out_objects, s, *logicObjectManager::getPredicate(MT::String("out")));
  if (out_objects.N>0) {
    out<<"--"<<endl;
    out<<"OUT:  ";
    FOR1D(out_objects, i) {
      out<<out_objects(i);
      if (isBall(id_inhand, s))
        out<<"o";
      out<<" ";
    }
    out<<endl;
  }
    
  MT::Array< uintA > homieGangs;
  getHomieGangs(homieGangs, s);
  if (homieGangs.N > 0   &&   homieGangs(0).N > 1) {
    out<<"--"<<endl;
    out<<"Gangs:"<<endl;
    FOR1D(homieGangs, i) {
      out<<homieGangs(i)<<endl;
    }
  }
  
  out<<"--"<<endl;
  
  out<<"(existing objects: "<<logicObjectManager::constants<<")"<<endl;
}



bool TL::RobotManipulationDomain::has_maximum_stack_value(const TL::State& s) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"RobotManipulationDomain::has_maximum_stack_value [START]"<<endl;}
  if (DEBUG>0) {cout<<"STATE:   ";  s.write();  cout<<endl;}
  
  TL::Predicate* p_BLOCK = logicObjectManager::getPredicate(MT::String("block"));
  TL::Predicate* p_BALL = logicObjectManager::getPredicate(MT::String("ball"));
  TL::Predicate* p_BOX = logicObjectManager::getPredicate(MT::String("box"));
  
  uintA blocks, balls, boxes;
  logicReasoning::getArguments(blocks, s, *p_BLOCK);
  logicReasoning::getArguments(balls, s, *p_BALL);
  logicReasoning::getArguments(boxes, s, *p_BOX);
  
  if (DEBUG>0) {PRINT(blocks);  PRINT(balls);  PRINT(boxes);}
  
  uint i;
  double maximum_stack_value = 0.;
  if (boxes.N > 0) {
    FOR1D(blocks, i) {
      maximum_stack_value += i+1;
    }
  }
  else {
    FOR1D(blocks, i) {
      maximum_stack_value += i;
    }
  }
  
  FOR1D(balls, i) {
    // erster ball auf die bloecke
    if (i==0) {
      maximum_stack_value += blocks.N;
    }
    // weitere baelle auf die kisten
    else if (boxes.N > i-1) {
      maximum_stack_value += 1;
    }
    else break;
  }
  
  TL::Function* f_SUM_HEIGHT = logicObjectManager::getFunction(MT::String("sum_height"));
  double real_stack_value = logicReasoning::getValue(f_SUM_HEIGHT, s);
  
  if (DEBUG>0) {PRINT(maximum_stack_value);  PRINT(real_stack_value);}
  if (DEBUG>0) {cout<<"RobotManipulationDomain::has_maximum_stack_value [END]"<<endl;}
  return TL::areEqual(maximum_stack_value, real_stack_value);
}


