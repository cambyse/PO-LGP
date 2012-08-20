/*  
    Copyright 2008-2012   Tobias Lang
    
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

#include "robotManipulationInterface.h"
#include "robotManipulationSymbols.h"
#include "reason.h"

#include <biros/logging.h>
SET_LOG(rmi, DEBUG);


namespace RMSim {


using namespace relational;
  
/* ---------------------
    TOP-LEVEL LOGIC-SIMULATOR-INTERFACE
  --------------------- */


SymbolicState* RobotManipulationInterface::calculateSymbolicState(RobotManipulationSimulator* sim) {
  SymbolicState* state = new SymbolicState;
  
  uint i, j;
  
  // TABLE
  uint table_id = sim->getTableID();
  if (reason::getConstants().findValue(table_id) < 0)
    table_id = TL::UINT_NIL;
  if (table_id != TL::UINT_NIL) {
    state->lits.append(Literal::get(Symbol::get("table"), TUP(table_id), 1.));
  }
  // BLOCKS
  uintA blocks;
  sim->getBlocks(blocks);
  FOR1D(blocks, i) {
    state->lits.append(Literal::get(Symbol::get("block"), TUP(blocks(i)), 1.));
  }
  // BALLS
  uintA balls;
  sim->getBalls(balls);
  FOR1D(balls, i) {
    state->lits.append(Literal::get(Symbol::get("ball"), TUP(balls(i)), 1.));
  }
  // BOXES
  uintA boxes;
  sim->getBoxes(boxes);
  FOR1D(boxes, i) {
    state->lits.append(Literal::get(Symbol::get("box"), TUP(boxes(i)), 1.));
  }
  // CYLINDERS
  uintA cylinders;
  sim->getCylinders(cylinders);
  FOR1D(cylinders, i) {
    state->lits.append(Literal::get(Symbol::get("cylinder"), TUP(cylinders(i)), 1.));
  }
  uintA all_objs;
  sim->getObjects(all_objs);
  // filter for logic
  FOR1D_DOWN(all_objs, i) {
    if (reason::getConstants().findValue(all_objs(i)) < 0) {
      all_objs.removeValueSafe(all_objs(i));
    }
  }
  
  // SIZE
  if (Symbol::get(MT::String("size")) != NULL) {
    double length;
    FOR1D(all_objs, i) {
      length = sim->getSize(all_objs(i))[0];
      state->lits.append(Literal::get(Symbol::get("size"), TUP(all_objs(i)), TL::REPLACE_SIZE(length)));
    }
  }
  
  // HOMIES
  if (Symbol::get(MT::String("homies")) != NULL) {
    uint k;
    FOR1D(all_objs, i) {
      for (k=i+1; k<all_objs.N; k++) {
        if (
            TL::areEqual(sim->getColor(all_objs(k))[0], sim->getColor(all_objs(i))[0])
            &&    TL::areEqual(sim->getColor(all_objs(k))[1], sim->getColor(all_objs(i))[1])
            &&    TL::areEqual(sim->getColor(all_objs(k))[2], sim->getColor(all_objs(i))[2])) {
          state->lits.append(Literal::get(Symbol::get("homies"), TUP(all_objs(i), all_objs(k)), 1.));
          state->lits.append(Literal::get(Symbol::get("homies"), TUP(all_objs(k), all_objs(i)), 1.));
        }
      }
    }
  }
  
  
  // INHAND
  uint catchedID = sim->getInhand();
  if (catchedID != UINT_MAX) {
    state->lits.append(Literal::get(Symbol::get("inhand"), TUP(sim->getInhand()), 1.));
  }
  
  // ON relations
  uintA objects_on;
  FOR1D(all_objs, i) {
    if (all_objs(i) != table_id) continue;
    sim->getObjectsOn(objects_on, all_objs(i));
    FOR1D(objects_on, j) {
      
      if (all_objs.findValue(objects_on(j)) >= 0)
        state->lits.append(Literal::get(Symbol::get("on"), TUP(objects_on(j), all_objs(i)), 1.));
    }
  }
  
  // UPRIGHT
  if (Symbol::get(MT::String("upright")) != NULL) {
    FOR1D(all_objs, i) {
      if (sim->isUpright(all_objs(i)))
        state->lits.append(Literal::get(Symbol::get("upright"), TUP(all_objs(i)), 1.));
    }
  }
  // OUT
  FOR1D(all_objs, i) {
    if (sim->onGround(all_objs(i)))
      state->lits.append(Literal::get(Symbol::get("out"), TUP(all_objs(i)), 1.));
  }
  
  // CONTAINS
  state->lits.memMove = true;
  FOR1D(boxes, i) {
    uint o = sim->getContainedObject(boxes(i));
    if (o != UINT_MAX) {
      if (table_id == TL::UINT_NIL) {
        state->lits.removeValueSafe(Literal::get(Symbol::get("on"), TUP(o, table_id), 1.));
      }
      state->lits.append(Literal::get(Symbol::get("contains"), TUP(boxes(i), o), 1.));
    }
  }
  
  // CLOSED
  FOR1D(boxes, i) {
    if (sim->isClosed(boxes(i))) {
      state->lits.append(Literal::get(Symbol::get("closed"), TUP(boxes(i)), 1.));
    }
  }

  state->state_constants = all_objs;
 
  //reason::derive(state);
  return state;
}


void RobotManipulationInterface::performAction(Literal* action, RobotManipulationSimulator* sim, uint secs_wait_after_action, const char* message) {
  if (action == NULL  ||  action->s->name == "doNothing"  ||  action == Literal::getLiteral_default_action()) {
    // don't do anything
    return;
  }
  else if (action->args.N > 0) {
    // special care for out of reach objects
    if (sim->onGround(action->args(0))) {
        return;
    }
  }
  if (action->s->name == "grab"  ||  action->s->name == "grab_puton") {
    uintA list;
    sim->getObjectsOn(list, action->args(0));
    if (sim->getOrsType(action->args(0)) == OBJECT_TYPE__BOX   // cannot lift box
        ||   sim->containedInClosedBox(action->args(0)))    // cannot take from closed box
      sim->simulate(30, message);
    else
      sim->grab(action->args(0), message);
  }
  else if (action->s->name == "puton"  ||  action->s->name == "puton_puton"  ||  action->s->name == "puton_grab") {
    if (sim->getOrsType(action->args(0)) == OBJECT_TYPE__BOX  // don't do anything if object = filled open box
          && !sim->isClosed(action->args(0))
          &&  sim->getContainedObject(action->args(0)) != UINT_MAX) 
      sim->simulate(30, message); 
    else if (sim->containedInBox(action->args(0))) // don't do anything if object in box
      sim->simulate(30, message); 
    else
      sim->dropObjectAbove(action->args(0), message);
  }
  else if (action->s->name == "lift") {
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
  else if (action->s->name == "place") {
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
  else if (action->s->name == "openBox") {
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
  else if (action->s->name == "closeBox") {
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
































// Sampling



bool RobotManipulationInterface::SAMPLING__WATCH_AFTER_ACTION = false;
uint RobotManipulationInterface::SAMPLING__WAIT_SEC_AFTER_ACTION = 50;
double RobotManipulationInterface::SAMPLING__PROB_SENSIBLE_ACTION = 1.0;
double RobotManipulationInterface::SAMPLING__PROB_GRAB_CLEARGUY =  0.8;
double RobotManipulationInterface::SAMPLING__PROB_PUTON_CLEARGUY = 0.8;


Literal* RobotManipulationInterface::generateAction(const SymbolicState& s, uint id_table) {
  return generateAction_wellBiased(s, id_table);
//   return generateAction_wellBiased_2Dactions(s, id_table);
}


Literal* RobotManipulationInterface::generateAction_wellBiased(const SymbolicState& state, uint id_table) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"generateAction_wellBiased [START]"<<endl;}
    // blocks on high towers have lower prob to be grabbed
  double skyscraper_bias = 200; // 115.0;
    
  uint i;
  // determine inhand object
  uint id_hand = RobotManipulationSymbols::getInhand(state);
  
  // determine number of non-out blocks
  uint num_out_blocks = 0;
  FOR1D(state.lits, i) {
    if (state.lits(i)->s->name == "out") {
      num_out_blocks++;
    }
  }
  uint no_non_out_blocks = reason::getConstants().N-1-num_out_blocks;
  if (no_non_out_blocks <= 1) {
    MT_MSG("Too few objects left for sensible acting!");
    return NULL;
  }

  Symbol* p_GRAB = Symbol::get(MT::String("grab"));
  Symbol* p_PUTON = Symbol::get(MT::String("puton"));
  Symbol* p_LIFT = Symbol::get(MT::String("lift"));
  Symbol* p_PLACE = Symbol::get(MT::String("place"));
  Symbol* p_ON = Symbol::get(MT::String("on"));
  Symbol* p_OUT = Symbol::get(MT::String("out"));
  
  // determine action
  double randNum;
  randNum = rnd.uni();
  Symbol* p_action;
  if (randNum < SAMPLING__PROB_SENSIBLE_ACTION) {
    if (id_hand == UINT_MAX) {
      if (p_GRAB != NULL)
        p_action = p_GRAB;
      else
        p_action = p_LIFT;
    }
    else {
      if (p_PUTON != NULL)
        p_action = p_PUTON;
      else
        p_action = p_PLACE;
    }
    if (DEBUG>0) cout << "SENSIBLE action TYPE   " << p_action->name << endl;
  }
  else {
    if (id_hand == UINT_MAX) {
      if (p_PUTON != NULL)
        p_action = p_PUTON;
      else
        p_action = p_PLACE;
    }
    else {
      if (p_GRAB != NULL)
        p_action = p_GRAB;
      else
        p_action = p_LIFT;
    }
    if (DEBUG>0) cout << "STUPID action TYPE   " << p_action->name << endl;
  }
    
  // calc clear and not-clear guys (out guys are not contained!)
  uintA clearGuys = reason::getConstants();
  uintA nonClearGuys;
  FOR1D(state.lits, i) {
    if (state.lits(i)->s == p_ON) {
      clearGuys.removeValueSafe(state.lits(i)->args(1));
      nonClearGuys.setAppend(state.lits(i)->args(1));
    }
  }
  if (id_hand != UINT_MAX) {
    clearGuys.removeValueSafe(id_hand);
    nonClearGuys.setAppend(id_hand);
  }
     PRINT(clearGuys)
     PRINT(nonClearGuys)
  CHECK(nonClearGuys.N + clearGuys.N == reason::getConstants().N, "Clear guy calculation failed");
  CHECK(numberSharedElements(nonClearGuys, clearGuys) == 0, "Clear guy calculation failed");
            
  // calc out-guys
  uintA outGuys;
  FOR1D(state.lits, i) {
    if (state.lits(i)->s == p_OUT) {
      outGuys.setAppend(state.lits(i)->args(0));
    }
  }
    
  // calc sensible targets
  uintA sensibleGuys = clearGuys;
  setMinus(sensibleGuys, outGuys);
  uintA nonSensibleGuys = nonClearGuys;
  nonSensibleGuys.setAppend(outGuys);
    
  // introduce bias for building high tower
  uintA piles;
  RobotManipulationSymbols::calcPiles(state, piles, id_table);

  uintA sa;
  
  // grab / lift
  if (p_action == p_GRAB  ||  p_action == p_LIFT) {
    if (p_action == p_LIFT)
      nonSensibleGuys.removeValue(id_table);
    sa.resize(1);
    randNum = rnd.uni();
    double fraction_sensible = (1.0 * sensibleGuys.N) / (sensibleGuys.N + nonSensibleGuys.N);
    double limit = TL_MAX(SAMPLING__PROB_GRAB_CLEARGUY, 0.8*fraction_sensible);
    if (p_action == p_LIFT && nonSensibleGuys.N == 1 && nonSensibleGuys(0) == id_table) // don't perform stupid action if only table grabable candidate then
      randNum = 0.;
    if (DEBUG>1) {
      PRINT(sensibleGuys);
      PRINT(nonSensibleGuys);
      PRINT(fraction_sensible);
      PRINT(limit);
      PRINT(randNum);
    }
    if (randNum < limit) {
      if (DEBUG>0) cout<<"SENSIBLE ACTION TARGET"<<endl;
      if (sensibleGuys.N>0) {
        // take low one
        arr weights;
        RobotManipulationSymbols::calcSkyscraperWeights(sensibleGuys, piles, skyscraper_bias, weights, false, id_table);
//                 cout<<"PICKUP: ";PRINT(weights);cout<<endl;
        sa(0) = sensibleGuys(TL::basic_sample(weights));
      }
      else // must grab non_clear guy
        sa(0) = nonSensibleGuys(rnd.num(nonSensibleGuys.N));
    }
    else { 
      if (DEBUG>0) cout<<"STUPID ACTION TARGET"<<endl;
      if (numberSharedElements(nonSensibleGuys, outGuys) < nonSensibleGuys.N) {
        do {
          sa(0) = nonSensibleGuys(rnd.num(nonSensibleGuys.N));
        } while (p_action == p_LIFT && outGuys.findValue(sa(0)) >= 0);
      }
      else // must pickup clear guy
        sa(0) = sensibleGuys(rnd.num(sensibleGuys.N));
    }
  }
  // puton / place
  else {
    sensibleGuys.append(id_table); // table is always clear
    nonSensibleGuys.removeValueSafe(id_table);
    sa.resize(1);
    randNum = rnd.uni();
    double fraction_sensible = (1.0 * sensibleGuys.N) / (sensibleGuys.N + nonSensibleGuys.N);
    double limit = TL_MAX(SAMPLING__PROB_PUTON_CLEARGUY, fraction_sensible);
    if (DEBUG>1) {
      PRINT(sensibleGuys);
      PRINT(nonSensibleGuys);
      PRINT(fraction_sensible);
      PRINT(limit);
      PRINT(randNum);
    }
    if (randNum < limit) {  // PUTTING ON CLEAR GUY
      if (DEBUG>0) cout<<"SENSIBLE ACTION TARGET"<<endl;
      if (sensibleGuys.N>0) {  // clear guy indeed available
                // put on high guy, digga
        arr weights;
        RobotManipulationSymbols::calcSkyscraperWeights(sensibleGuys, piles, skyscraper_bias, weights, true, id_table);
                // bias for putting on blocks (instead of on balls)
        FOR1D(sensibleGuys, i) {
//                   cout<<sensibleGuys(i);
          if (RobotManipulationSymbols::isBlock(sensibleGuys(i), state)) {   // prefer blocks
//                     cout<<" is block"<<endl;
            weights(i) *= 3.;
          }
        }
//                 cout<<"PUTON: ";PRINT(weights);cout<<endl;
        sa(0) = sensibleGuys(TL::basic_sample(weights));
                // lowering chance for table
        if (sa(0) == id_table) {
          sa(0) = sensibleGuys(TL::basic_sample(weights));
        }
      }
      else // no clear guy available --> put on non-clear guy
        sa(0) = nonSensibleGuys(rnd.num(nonSensibleGuys.N));
    }
    else {   // PUTTING ON NON-CLEAR GUY
      if (DEBUG>0) cout<<"STUPID ACTION TARGET"<<endl;
      if (nonSensibleGuys.N>0) { // non-clear guy available
        sa(0) = nonSensibleGuys(rnd.num(nonSensibleGuys.N));
                // artifically lower chance for putting on inhand- or out-object
                // by sampling a second time
        if (sa(0) == id_hand || outGuys.findValue(sa(0))>=0) {
          sa(0) = nonSensibleGuys(rnd.num(nonSensibleGuys.N));
          if (sa(0) == id_hand || outGuys.findValue(sa(0))>=0) {
            if (sa(0) == id_hand || outGuys.findValue(sa(0))>=0) {
              sa(0) = nonSensibleGuys(rnd.num(nonSensibleGuys.N));
            }
          }
        }
      }
      else // no non-clear guy available --> put on clear guy
        sa(0) = sensibleGuys(rnd.num(sensibleGuys.N));
    }
  }
  
  if (p_action == p_LIFT) {
    uint arg1 = sa(0);
    sa.resize(2);
    sa(0) = arg1;
    // get below
    sa(1) = RobotManipulationSymbols::getBelow(arg1, state);
  }
  else if (p_action == p_PLACE) {
    uint arg2 = sa(0);
    sa.resize(2);
    sa(1) = arg2;
    // get inhand
    sa(0) = id_hand;
  }
  
  if (DEBUG>0) {cout<<"generateAction_wellBiased [END]"<<endl;}
  return Literal::get(p_action, sa, 1.);
}



// -----------------------------------------------
//  LIFT und PLACE
// -----------------------------------------------
uint HACK_ACT = 0;

Literal* RobotManipulationInterface::generateAction_wellBiased_2Dactions(const SymbolicState& state, uint id_table) {
  uint DEBUG = 1;
  
  // --------------------------------------------
  //  GENERAL STATE INFORMATION
  
  Symbol* p_LIFT = Symbol::get(MT::String("lift"));
  Symbol* p_PLACE = Symbol::get(MT::String("place"));
  Symbol* p_ON = Symbol::get(MT::String("on"));
  Symbol* p_OUT = Symbol::get(MT::String("out"));
  
  uint i;
  // determine inhand object
  uint id_hand = RobotManipulationSymbols::getInhand(state);
  
  // determine number of non-out blocks
  uint num_out_objects = 0;
  FOR1D(state.lits, i) {
    if (state.lits(i)->s == p_OUT) {
      num_out_objects++;
    }
  }
  uint num_non_out_objects = reason::getConstants().N-1-num_out_objects;
  if (num_non_out_objects <= 1) {
    MT_MSG("Too few objects left for sensible acting!");
    return NULL;
  }
  
  // calc clear and not-clear guys (out guys are not contained!)
  uintA clearGuys = reason::getConstants();
  uintA nonClearGuys;
  FOR1D(state.lits, i) {
    if (state.lits(i)->s == p_ON) {
      clearGuys.removeValueSafe(state.lits(i)->args(1));
      nonClearGuys.setAppend(state.lits(i)->args(1));
    }
  }
  if (id_hand != UINT_MAX) {
    clearGuys.removeValueSafe(id_hand);
    nonClearGuys.setAppend(id_hand);
  }
  // PRINT(clearGuys)
  // PRINT(nonClearGuys)
  CHECK(nonClearGuys.N + clearGuys.N == reason::getConstants().N, "Clear guy calculation failed");
  CHECK(numberSharedElements(nonClearGuys, clearGuys) == 0, "Clear guy calculation failed");
  
  // calc out-guys
  uintA outGuys;
  FOR1D(state.lits, i) {
    if (state.lits(i)->s == p_OUT) {
      outGuys.setAppend(state.lits(i)->args(0));
    }
  }
  
  
  // --------------------------------------------
  //  ACTION DETERMINATION

  Symbol* p_action;
  
  double randNum;
  bool sensible_action;
  if (rnd.uni() < SAMPLING__PROB_SENSIBLE_ACTION)
    sensible_action = true;
  else
    sensible_action = false;
  
  uintA args(2);
    
  
  
  // [start] TODO
  if (rnd.uni() < 0.25) { // Do something with box
    Symbol* p_BOX = Symbol::get(MT::String("box"));
    uintA boxes;
    FOR1D(state.lits, i) {
      if (state.lits(i)->s == p_BOX)
        boxes.append(state.lits(i)->args(0));
    }
    p_action = NULL;
    
    // Do something with other object
    if (rnd.uni() < 0.6) {
      args.resize(2);
      if (id_hand != UINT_MAX) {
        args(0) = id_hand;
        args(1) = boxes(rnd.num(boxes.N));
        p_action = p_PLACE;
      }
      else {
        // try to grab object out of box
        uintA boxedObjs;
        FOR1D(boxes, i) {
          uint obj = RobotManipulationSymbols::getContainedObject(boxes(i), state);
          if (obj != UINT_MAX)
            boxedObjs.append(obj);
        }
        if (boxedObjs.N > 0) {
          args(0) = boxedObjs(rnd.num(boxedObjs.N));
          args(1) = RobotManipulationSymbols::getContainingBox(args(0), state);
          p_action = p_LIFT;
        }
      }
    }
    
    // Close / open
    if (p_action == NULL) {
      uint box = boxes(rnd.num(boxes.N));
      args.resize(1);
      args(0) = box;
      if (sensible_action) {
        if (RobotManipulationSymbols::isClosed(args(0), state))
          p_action = Symbol::get(MT::String("openBox"));
        else
          p_action = Symbol::get(MT::String("closeBox"));
      }
      else {
        if (RobotManipulationSymbols::isClosed(args(0), state))
          p_action = Symbol::get(MT::String("closeBox"));
        else
          p_action = Symbol::get(MT::String("openBox"));
      }
    }
    return Literal::get(p_action, args, 1.); 
  }
  // [end] TODO
  
  
  
  // --------------------------------
  //   (1) SENSIBLE Action
  if (sensible_action) {
    if (id_hand == UINT_MAX) {
      p_action = p_LIFT;
    }
    else {
      p_action = p_PLACE;
    }
    if (DEBUG>0) cout << "SENSIBLE action   with type   " << p_action->name << endl;
    
    bool sensible_target;
    
    //  ------------------------------------------------------------------
    // (1.1) SENSIBLE LIFT
    if (p_action == p_LIFT) {
      // calc sensible first args
      uintA sensibleFirstArgs = clearGuys;
      setMinus(sensibleFirstArgs, outGuys);
      // calc non-sensible first args
      uintA nonSensibleFirstArgs = nonClearGuys;
      setMinus(nonSensibleFirstArgs, outGuys);
      nonSensibleFirstArgs.removeValue(id_table);
      
      double fraction_sensible = (1.0 * sensibleFirstArgs.N) / (sensibleFirstArgs.N + nonSensibleFirstArgs.N);
      double limit = TL_MAX(SAMPLING__PROB_GRAB_CLEARGUY, 0.8*fraction_sensible);
      if (nonSensibleFirstArgs.N == 1 && nonSensibleFirstArgs(0) == id_table) // don't perform stupid action if only table grabable candidate then
        limit = 1.;
      randNum = rnd.uni();
      if (randNum < limit)
        sensible_target = true;
      else
        sensible_target = false;
      
      if (DEBUG>1) {
        PRINT(sensibleFirstArgs);
        PRINT(nonSensibleFirstArgs);
        PRINT(fraction_sensible);
        PRINT(limit);
        PRINT(randNum);
        PRINT(sensible_target);
      }
      
      //  (1.1.1) lift clear object
      if (sensible_target) {
        if (DEBUG>0) cout<<"SENSIBLE action TARGET"<<endl;
        if (sensibleFirstArgs.N>0) {
          arr weights(sensibleFirstArgs.N);
          weights.setUni(1.0);
          args(0) = sensibleFirstArgs(rnd.num(sensibleFirstArgs.N));
        }
        else { // must lift non-clear guy
          args(0) = nonSensibleFirstArgs(rnd.num(nonSensibleFirstArgs.N));  // ensured not to be table
        }
      }
      //  (1.1.2) lift non-clear object
      else {
        if (DEBUG>0) cout<<"STUPID action TARGET"<<endl;
        if (numberSharedElements(nonSensibleFirstArgs, outGuys) < nonSensibleFirstArgs.N) {
          do {
            args(0) = nonSensibleFirstArgs(rnd.num(nonSensibleFirstArgs.N));
          } while (outGuys.findValue(args(0)) >= 0);
        }
        else // must pickup clear guy
          args(0) = sensibleFirstArgs(rnd.num(sensibleFirstArgs.N));
      }
      args(1) = RobotManipulationSymbols::getBelow(args(0), state);
      if (UINT_MAX == args(1))
        args(1) = RobotManipulationSymbols::getContainingBox(args(0), state);
    }
    //  ------------------------------------------------------------------
    // (1.2) SENSIBLE PLACE
    else {
      // calc sensible 2nd args
      uintA sensibleSecondArgs = clearGuys;
      setMinus(sensibleSecondArgs, outGuys);
      sensibleSecondArgs.removeValueSafe(id_hand);
      sensibleSecondArgs.append(id_table);
      // calc non-sensible 2nd args
      uintA nonSensibleSecondArgs = nonClearGuys;
      nonSensibleSecondArgs.removeValue(id_table);
      
      double fraction_sensible = (1.0 * sensibleSecondArgs.N) / (sensibleSecondArgs.N + nonSensibleSecondArgs.N);
      double limit = TL_MAX(SAMPLING__PROB_PUTON_CLEARGUY, fraction_sensible);
      if (nonSensibleSecondArgs.N == 1 && nonSensibleSecondArgs(0) == id_table) // don't perform stupid action if only table grabable candidate then
        limit = 1.;
      randNum = rnd.uni();
      if (randNum < limit)
        sensible_target = true;
      else
        sensible_target = false;
      
      if (DEBUG>1) {
        PRINT(sensibleSecondArgs);
        PRINT(nonSensibleSecondArgs);
        PRINT(fraction_sensible);
        PRINT(limit);
        PRINT(randNum);
        PRINT(sensible_target);
      }
      
      // (1.2.1) place on clear object
      if (sensible_target) {
        if (DEBUG>0) cout<<"SENSIBLE action TARGET"<<endl;
        if (sensibleSecondArgs.N>0) {  // clear guy indeed available
          // put on high guy, digga
          arr weights;
          double skyscraper_bias = 200;
          // introduce bias for building high tower
          uintA piles;
          RobotManipulationSymbols::calcPiles(state, piles, id_table);
          RobotManipulationSymbols::calcSkyscraperWeights(sensibleSecondArgs, piles, skyscraper_bias, weights, true, id_table);
          // bias for putti~ng on blocks (instead of on balls)
          FOR1D(sensibleSecondArgs, i) {
            if (RobotManipulationSymbols::isBlock(sensibleSecondArgs(i), state)) {   // prefer blocks
              weights(i) *= 5.;
            }
          }
          args(1) = sensibleSecondArgs(TL::basic_sample(weights));
          // lowering chance for table
          if (args(1) == id_table) {
            args(1) = sensibleSecondArgs(TL::basic_sample(weights));
          }
        }
        else // no clear guy available --> put on non-clear guy
          args(1) = nonSensibleSecondArgs(rnd.num(nonSensibleSecondArgs.N));
      }
      // (1.2.2) place on non-clear object
      else {
        if (DEBUG>0) cout<<"STUPID action TARGET"<<endl;
        if (nonSensibleSecondArgs.N>0) { // non-clear guy available
          do {
            args(1) = nonSensibleSecondArgs(rnd.num(nonSensibleSecondArgs.N));
            // artifically lower chance for putting on out-object  by sampling a second time
            if (outGuys.findValue(args(1))>=0) {
              args(1) = nonSensibleSecondArgs(rnd.num(nonSensibleSecondArgs.N));
              if (outGuys.findValue(args(1))>=0) {
                if (outGuys.findValue(args(1))>=0) {
                  args(1) = nonSensibleSecondArgs(rnd.num(nonSensibleSecondArgs.N));
                }
              }
            }
            if (nonSensibleSecondArgs.N == 1)
              break;
          } while (args(1) == id_hand);
        }
        else // no non-clear guy available --> put on clear guy
          args(1) = sensibleSecondArgs(rnd.num(sensibleSecondArgs.N));
      }
      args(0) = id_hand;
    }
  }
  // -------------------------------------------
  //   (2) NON-SENSIBLE Action
#define PROB_NON_SENSIBLE_PLACE 0.3
  else {
    if (rnd.uni() < PROB_NON_SENSIBLE_PLACE)
      p_action = p_PLACE;
    else
      p_action = p_LIFT;
    
    if (DEBUG>0) cout << "STUPID action   with type   " << p_action->name << endl;
    
    // (2.1)
    if (p_action == p_PLACE) {
      // (2.1.1)   if inhand-nil:  place(a,b) with random a and b
      if (id_hand == UINT_MAX) {
        args(0) = reason::getConstants()(rnd.num(reason::getConstants().N));
        do {
          args(1) = reason::getConstants()(rnd.num(reason::getConstants().N));
        } while (args(0) == args(1));
      }
      // (2.1.2)   if inhand(a):  place(b,c) with b and c random and not a
      else {
        do {
          args(0) = reason::getConstants()(rnd.num(reason::getConstants().N));
        } while (args(0) == id_hand);
        do {
          args(1) = reason::getConstants()(rnd.num(reason::getConstants().N));
        } while (args(0) == args(1)   ||   args(1) == id_hand);
      }
    }
    // (2.2)
    else {
      // (2.2.1)  if inhand(a):  lift(b,c)   with b not inhand and on(b,c)
      if (id_hand != UINT_MAX  && rnd.uni() < 0.95) {
        do {
          args(0) = reason::getConstants()(rnd.num(reason::getConstants().N));
          args(1) = RobotManipulationSymbols::getBelow(args(0), state);
        } while (args(0) == id_hand  ||  args(1) == UINT_MAX);
      }
      // (2.2.2)  if inhand-nil:  lift(a,b)  with  a and b random, but _not_ on(a,b)
      else {
        uint sa0_below;
        do {
          args(0) = reason::getConstants()(rnd.num(reason::getConstants().N));
          args(1) = reason::getConstants()(rnd.num(reason::getConstants().N));
          sa0_below = RobotManipulationSymbols::getBelow(args(0), state);
        } while (args(1) == sa0_below);
      }
    }
  }
  
  if (reason::getConstants().findValue(args(0)) < 0  ||  reason::getConstants().findValue(args(1)) < 0) {
    PRINT(args(0));
    PRINT(args(1));
    HALT("");
  }
  
  return Literal::get(p_action, args, 1.);
}



#if 0
Literal* RobotManipulationInterface::generateAction_wellBiased_2Dactions(const SymbolicState& s, uint id_table) {
  uint DEBUG = 1;
    // blocks on high towers have lower prob to be grabbed
  double skyscraper_bias = 200; // 115.0;
    
  uint i;
  // determine inhand object
  uint id_hand = RobotManipulationInterface::getInhand(s);
  
  // determine number of non-out blocks
  uint num_out_blocks = 0;
  FOR1D(state.lits, i) {
    if (state.lits(i)->s->name == "out") {
      num_out_blocks++;
    }
  }
  uint no_non_out_blocks = reason::getConstants().N-1-num_out_blocks;
  if (no_non_out_blocks <= 1) {
    MT_MSG("Too few objects left for sensible acting!");
    return NULL;
  }

  Symbol* p_LIFT = Symbol::get(MT::String("lift"));
  Symbol* p_PLACE = Symbol::get(MT::String("place"));
  Symbol* p_ON = Symbol::get(MT::String("on"));
  Symbol* p_OUT = Symbol::get(MT::String("out"));
  
  // calc clear and not-clear guys (out guys are not contained!)
  uintA clearGuys = reason::getConstants();
  uintA nonClearGuys;
  FOR1D(state.lits, i) {
    if (state.lits(i)->s == p_ON) {
      clearGuys.removeValueSafe(state.lits(i)->args(1));
      nonClearGuys.setAppend(state.lits(i)->args(1));
    }
  }
  if (id_hand != UINT_MAX) {
    clearGuys.removeValueSafe(id_hand);
    nonClearGuys.setAppend(id_hand);
  }
  // PRINT(clearGuys)
  // PRINT(nonClearGuys)
  CHECK(nonClearGuys.N + clearGuys.N == reason::getConstants().N, "Clear guy calculation failed");
  CHECK(numberSharedElements(nonClearGuys, clearGuys) == 0, "Clear guy calculation failed");
  
  // calc out-guys
  uintA outGuys;
  FOR1D(state.lits, i) {
    if (state.lits(i)->s == p_OUT) {
      outGuys.setAppend(state.lits(i)->args(0));
    }
  }
  
  
  // determine action
  double randNum;
  bool sensible_action;
  if (rnd.uni() < SAMPLING__PROB_SENSIBLE_ACTION)
    sensible_action = true;
  else
    sensible_action = false;
  Symbol* p_action;
  if (sensible_action) {
    if (id_hand == UINT_MAX) {
      p_action = p_LIFT;
    }
    else {
      p_action = p_PLACE;
    }
    if (DEBUG>0) cout << "SENSIBLE action TYPE   " << p_action->name << endl;
  }
  else {
    if (id_hand == UINT_MAX) {
      p_action = p_PLACE;
    }
    else {
      p_action = p_LIFT;
    }
    if (DEBUG>0) cout << "STUPID action TYPE   " << p_action->name << endl;
  }
    
  
  // calc sensible targets
  uintA sensibleGuys = clearGuys;
  setMinus(sensibleGuys, outGuys);
  uintA nonSensibleGuys = nonClearGuys;
  nonSensibleGuys.setAppend(outGuys);
    
  // introduce bias for building high tower
  uintA piles;
  RobotManipulationInterface::calcPiles(s, piles, id_table);

  uintA sa(2);
  
  // lift
  if (p_action == p_LIFT) {
    nonSensibleGuys.removeValue(id_table);
    randNum = rnd.uni();
    double fraction_sensible = (1.0 * sensibleGuys.N) / (sensibleGuys.N + nonSensibleGuys.N);
    double limit = TL_MAX(SAMPLING__PROB_GRAB_CLEARGUY, 0.8*fraction_sensible);
    if (nonSensibleGuys.N == 1 && nonSensibleGuys(0) == id_table) // don't perform stupid action if only table grabable candidate then
      randNum = 0.;
    if (DEBUG>1) {
      PRINT(sensibleGuys);
      PRINT(nonSensibleGuys);
      PRINT(fraction_sensible);
      PRINT(limit);
      PRINT(randNum);
    }
    // -------------------------
    //  Argument 1 = active lifted object
    if (randNum < limit) {
      if (DEBUG>0) cout<<"SENSIBLE ACTION TARGET"<<endl;
      if (sensibleGuys.N>0) {
        // take low one
        arr weights(sensibleGuys.N);
        weights.setUni(1.0);
//         RobotManipulationInterface::calcSkyscraperWeights(sensibleGuys, piles, skyscraper_bias, weights, false, id_table);
        sa(0) = sensibleGuys(TL::basic_sample(weights));
      }
      else // must grab non_clear guy
        sa(0) = nonSensibleGuys(rnd.num(nonSensibleGuys.N));
    }
    else { 
      if (DEBUG>0) cout<<"STUPID ACTION TARGET"<<endl;
      if (numberSharedElements(nonSensibleGuys, outGuys) < nonSensibleGuys.N) {
        do {
          sa(0) = nonSensibleGuys(rnd.num(nonSensibleGuys.N));
        } while (outGuys.findValue(sa(0)) >= 0);
      }
      else // must pickup clear guy
        sa(0) = sensibleGuys(rnd.num(sensibleGuys.N));
    }
    
    // -------------------------
    //  Argument 2 = passive object (where to lift from)
    if (rnd.uni() > 0.9  ||  !sensible_action) {
      sa(1) = reason::getConstants()(rnd.num(reason::getConstants().N));
    }
    else {
      sa(1) = RobotManipulationInterface::getBelow(sa(0), s);
    }
  }
  // place
  else {
    sensibleGuys.append(id_table); // table is always clear
    sensibleGuys.append(id_hand);
    nonSensibleGuys.removeValueSafe(id_table);
    randNum = rnd.uni();
    double fraction_sensible = (1.0 * sensibleGuys.N) / (sensibleGuys.N + nonSensibleGuys.N);
    double limit = TL_MAX(SAMPLING__PROB_PUTON_CLEARGUY, fraction_sensible);
    if (DEBUG>1) {
      PRINT(sensibleGuys);
      PRINT(nonSensibleGuys);
      PRINT(fraction_sensible);
      PRINT(limit);
      PRINT(randNum);
    }
    // -------------------------
    //  Argument 2 = target object, where object shall be placed
    if (randNum < limit) {  // PUTTING ON CLEAR GUY
      if (DEBUG>0) cout<<"SENSIBLE ACTION TARGET"<<endl;
      if (sensibleGuys.N>0) {  // clear guy indeed available
        // put on high guy, digga
        arr weights;
        RobotManipulationInterface::calcSkyscraperWeights(sensibleGuys, piles, skyscraper_bias, weights, true, id_table);
                // bias for putting on blocks (instead of on balls)
        FOR1D(sensibleGuys, i) {
//                   cout<<sensibleGuys(i);
          if (RobotManipulationInterface::isBlock(sensibleGuys(i), s)) {   // prefer blocks
//                     cout<<" is block"<<endl;
            weights(i) *= 3.;
          }
        }
//                 cout<<"PUTON: ";PRINT(weights);cout<<endl;
        sa(1) = sensibleGuys(TL::basic_sample(weights));
        // lowering chance for table
        if (sa(1) == id_table) {
          sa(1) = sensibleGuys(TL::basic_sample(weights));
        }
      }
      else // no clear guy available --> put on non-clear guy
        sa(1) = nonSensibleGuys(rnd.num(nonSensibleGuys.N));
    }
    else {   // PUTTING ON NON-CLEAR GUY
      if (DEBUG>0) cout<<"STUPID ACTION TARGET"<<endl;
      if (nonSensibleGuys.N>0) { // non-clear guy available
        sa(1) = nonSensibleGuys(rnd.num(nonSensibleGuys.N));
        // artifically lower chance for putting on inhand- or out-object
        // by sampling a second time
        if (sa(1) == id_hand || outGuys.findValue(sa(1))>=0) {
          sa(1) = nonSensibleGuys(rnd.num(nonSensibleGuys.N));
          if (sa(1) == id_hand || outGuys.findValue(sa(1))>=0) {
            if (sa(1) == id_hand || outGuys.findValue(sa(1))>=0) {
              sa(1) = nonSensibleGuys(rnd.num(nonSensibleGuys.N));
            }
          }
        }
      }
      else // no non-clear guy available --> put on clear guy
        sa(1) = sensibleGuys(rnd.num(sensibleGuys.N));
    }
    
    // -------------------------
    //  Argument 1 = moved object
    if (rnd.uni() > 0.9  ||  !sensible_action) {
      sa(0) = reason::getConstants()(rnd.num(reason::getConstants().N));
    }
    else {
      sa(0) = id_hand;
    }
  }
  
  return logicObjectManager::getLiteral(p_action, true, sa);
}
#endif



Literal* RobotManipulationInterface::generateAction_trulyRandom(const SymbolicState& s, uint id_table) {
  uint i;
  
  // choose action
  HALT("funktioniertso nicht mit default action");
  SymL syms_action;
  Symbol::get_action(syms_action);
  Symbol* action = syms_action(rnd.num(syms_action.N-1)+1); // omitting default action
 
  uintA sa(action->arity);
  FOR1D(sa, i) {
    sa(i) = reason::getConstants()(rnd.num(reason::getConstants().N));
    // first should never be table
    if (i==0) {
      while( sa(i) != id_table ){
        sa(i) = reason::getConstants()(rnd.num(reason::getConstants().N));
      }
    }
  }
  
  return Literal::get(action, sa, 1.);
}


StateTransitionL& RobotManipulationInterface::generateSimulationSequence(RobotManipulationSimulator* sim, uint maxSeqLength, uint id_table) {
  uint DEBUG = 3;
  if (DEBUG>0) cout << "New sampling sequence" << endl;
  if (DEBUG > 1) {
    cout << "sampleSimulationSequence [START]" << endl;
    sim->printObjectInfo();
  }
  uint i=0;
  StateTransitionL experiences;
  SymbolicState* state, *state_old = NULL;
  state = RobotManipulationInterface::calculateSymbolicState(sim);
  if (DEBUG > 2) {cout<<endl; state->write(cout); cout<<endl;}
  Literal* action;
  while (i<maxSeqLength) {
    if (DEBUG>0) cout << "*** Step " << i << endl;
    state_old = state;
    action = generateAction(experiences.last()->post, id_table);
    if (action == NULL) {
      if (DEBUG>0) {cout<<"No more sensible action possible, so we stop generating."<<endl;}
      break;
    }
    if (DEBUG > 2) {action->write(cout); cout << endl;}
    RobotManipulationInterface::performAction(action, sim, SAMPLING__WAIT_SEC_AFTER_ACTION);
//     sim->relaxPosition(); // needed to have a observe correct subsequent state
//     sim->simulate(SAMPLING__WAIT_SEC_AFTER_ACTION); // needed to have a observe correct subsequent state
    state = RobotManipulationInterface::calculateSymbolicState(sim);
    if (DEBUG > 2) {state->write(cout,true); cout<<endl;}
    experiences.append(new StateTransition(*state_old, action, *state));
    i++;
    if (SAMPLING__WATCH_AFTER_ACTION)
      sim->watch();
    if (DEBUG > 2) {
      cout<<"CHANGE:  ";
      cout<<"\tBefore: "<<experiences.last()->add<<endl;
      cout<<"\tAfter: "<<experiences.last()->del<<endl;
    }
  }
  if (DEBUG > 1)
    cout << "sampleSimulationSequence [END]" << endl;
  return experiences;
}







void RobotManipulationInterface::generateSimulationSequence_realistic(std::ostream& os, RobotManipulationSimulator* sim, uint maxSeqLength, uint id_table) {
  uint DEBUG = 3;
  if (DEBUG>0) cout << "New sampling sequence" << endl;
  if (DEBUG > 1) {
    cout << "sampleSimulationSequence [START]" << endl;
//     sim->printObjectInfo();
  }
  uint i;
  
  // (2) Objects
  os<<endl;
  os<<"# objects"<<endl;
  FOR1D(reason::getConstants(), i) {
    os<<reason::getConstants()(i)<<" ";
  }
  os<<endl;
  
  // (3) Data
  os<<endl;
  os<<"# data"<<endl;
  SymbolicState* logic_state, *logic_state_old;
  logic_state = RobotManipulationInterface::calculateSymbolicState(sim);
  logic_state->write(os);
  os<<endl;
//   RobotManipulationInterface::writeFeatures(os, sim);  os<<endl;
  if (DEBUG > 2) {cout<<endl; logic_state->write(cout, true); cout<<endl;}

  Literal* action;
  i=0;
  while (i<maxSeqLength) {
    if (DEBUG>0) cout << "*** Step " << i << endl;
    action = generateAction(*logic_state, id_table);
    os<<endl;
    action->write(os);
    os<<endl;os<<endl;
    if (action == NULL) {
      if (DEBUG>0) {cout<<"No more sensible action possible, so we stop generating."<<endl;}
      break;
    }
    if (DEBUG > 2) {cout<<"ACTION: "; action->write(cout); cout << endl;}
    RobotManipulationInterface::performAction(action, sim, SAMPLING__WAIT_SEC_AFTER_ACTION);
//     sim->relaxPosition(); // needed to have a observe correct subsequent state
//     sim->simulate(SAMPLING__WAIT_SEC_AFTER_ACTION); // needed to have a observe correct subsequent state
    logic_state_old = logic_state;
    logic_state = RobotManipulationInterface::calculateSymbolicState(sim);
    logic_state->write(os);
    os<<endl;
//     RobotManipulationInterface::writeFeatures(os, sim);  os<<endl;
    if (DEBUG > 2) {logic_state->write(cout); cout<<endl;}
    if (DEBUG>2) {RobotManipulationSymbols::writeStateInfo(*logic_state);}
    i++;
    if (SAMPLING__WATCH_AFTER_ACTION)
      sim->watch();
    if (DEBUG > 2) {
      uintA changedConstants;
      LitL holdOnlyPre, holdOnlyPost;
      SymbolicState::calcDifferences(holdOnlyPre, holdOnlyPost, changedConstants, *logic_state_old, *logic_state);
      cout<<"CHANGE:  ";
      cout<<"\tBefore: "<<holdOnlyPre<<endl;
      cout<<"\tAfter: "<<holdOnlyPost<<endl;
    }
  }
  if (DEBUG > 1)
    cout << "sampleSimulationSequence [END]" << endl;
}

}  // namespace PRADA
