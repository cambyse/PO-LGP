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

#include "relational/robotManipulationDomain.h"
#include "relational/robotManipulationSampling.h"
#include "relational/logicReasoning.h"



bool TL::robotManipulationSampling::SAMPLING__WATCH_AFTER_ACTION = false;
uint TL::robotManipulationSampling::SAMPLING__WAIT_SEC_AFTER_ACTION = 50;
double TL::robotManipulationSampling::SAMPLING__PROB_SENSIBLE_ACTION = 1.0;
double TL::robotManipulationSampling::SAMPLING__PROB_GRAB_CLEARGUY =  0.8;
double TL::robotManipulationSampling::SAMPLING__PROB_PUTON_CLEARGUY = 0.8;


TL::Atom* TL::robotManipulationSampling::generateAction(const State& s, uint id_table) {
  return generateAction_wellBiased(s, id_table);
//   return generateAction_wellBiased_2Dactions(s, id_table);
}


TL::Atom* TL::robotManipulationSampling::generateAction_wellBiased(const State& s, uint id_table) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"generateAction_wellBiased [START]"<<endl;}
    // blocks on high towers have lower prob to be grabbed
  double skyscraper_bias = 200; // 115.0;
    
  uint i;
  // determine inhand object
  uint id_hand = TL::RobotManipulationDomain::getInhand(s);
  
  // determine number of non-out blocks
  uint no_out_blocks = 0;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred->name == "out") {
      no_out_blocks++;
    }
  }
  uint no_non_out_blocks = logicObjectManager::constants.N-1-no_out_blocks;
  if (no_non_out_blocks <= 1) {
    MT_MSG("Too few objects left for sensible acting!");
    return NULL;
  }

  TL::Predicate* p_GRAB = logicObjectManager::getPredicate(MT::String("grab"));
  TL::Predicate* p_PUTON = logicObjectManager::getPredicate(MT::String("puton"));
  TL::Predicate* p_LIFT = logicObjectManager::getPredicate(MT::String("lift"));
  TL::Predicate* p_PLACE = logicObjectManager::getPredicate(MT::String("place"));
  TL::Predicate* p_ON = logicObjectManager::getPredicate(MT::String("on"));
  TL::Predicate* p_OUT = logicObjectManager::getPredicate(MT::String("out"));
  
  // determine action
  double randNum;
  randNum = rnd.uni();
  TL::Predicate* p_action;
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
  uintA clearGuys = logicObjectManager::constants;
  uintA nonClearGuys;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred == p_ON) {
      clearGuys.removeValueSafe(s.lits_prim(i)->atom->args(1));
      nonClearGuys.setAppend(s.lits_prim(i)->atom->args(1));
    }
  }
  if (id_hand != UINT_MAX) {
    clearGuys.removeValueSafe(id_hand);
    nonClearGuys.setAppend(id_hand);
  }
    // PRINT(clearGuys)
    // PRINT(nonClearGuys)
  CHECK(nonClearGuys.N + clearGuys.N == logicObjectManager::constants.N, "Clear guy calculation failed");
  CHECK(numberSharedElements(nonClearGuys, clearGuys) == 0, "Clear guy calculation failed");
            
  // calc out-guys
  uintA outGuys;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred == p_OUT) {
      outGuys.setAppend(s.lits_prim(i)->atom->args(0));
    }
  }
    
  // calc sensible targets
  uintA sensibleGuys = clearGuys;
  setMinus(sensibleGuys, outGuys);
  uintA nonSensibleGuys = nonClearGuys;
  nonSensibleGuys.setAppend(outGuys);
    
  // introduce bias for building high tower
  uintA piles;
  TL::RobotManipulationDomain::calcPiles(s, piles, id_table);

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
        TL::RobotManipulationDomain::calcSkyscraperWeights(sensibleGuys, piles, skyscraper_bias, weights, false, id_table);
//                 cout<<"PICKUP: ";PRINT(weights);cout<<endl;
        sa(0) = sensibleGuys(basic_sample(weights));
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
        TL::RobotManipulationDomain::calcSkyscraperWeights(sensibleGuys, piles, skyscraper_bias, weights, true, id_table);
                // bias for putting on blocks (instead of on balls)
        FOR1D(sensibleGuys, i) {
//                   cout<<sensibleGuys(i);
          if (TL::RobotManipulationDomain::isBlock(sensibleGuys(i), s)) {   // prefer blocks
//                     cout<<" is block"<<endl;
            weights(i) *= 3.;
          }
        }
//                 cout<<"PUTON: ";PRINT(weights);cout<<endl;
        sa(0) = sensibleGuys(basic_sample(weights));
                // lowering chance for table
        if (sa(0) == id_table) {
          sa(0) = sensibleGuys(basic_sample(weights));
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
    sa(1) = TL::RobotManipulationDomain::getBelow(arg1, s);
  }
  else if (p_action == p_PLACE) {
    uint arg2 = sa(0);
    sa.resize(2);
    sa(1) = arg2;
    // get inhand
    sa(0) = id_hand;
  }
  
  if (DEBUG>0) {cout<<"generateAction_wellBiased [END]"<<endl;}
  return logicObjectManager::getAtom(p_action, sa);
}



// -----------------------------------------------
//  LIFT und PLACE
// -----------------------------------------------
uint HACK_ACT = 0;

TL::Atom* TL::robotManipulationSampling::generateAction_wellBiased_2Dactions(const State& s, uint id_table) {
  uint DEBUG = 1;
  
  // --------------------------------------------
  //  GENERAL STATE INFORMATION
  
  TL::Predicate* p_LIFT = logicObjectManager::getPredicate(MT::String("lift"));
  TL::Predicate* p_PLACE = logicObjectManager::getPredicate(MT::String("place"));
  TL::Predicate* p_ON = logicObjectManager::getPredicate(MT::String("on"));
  TL::Predicate* p_OUT = logicObjectManager::getPredicate(MT::String("out"));
  
  uint i;
  // determine inhand object
  uint id_hand = TL::RobotManipulationDomain::getInhand(s);
  
  // determine number of non-out blocks
  uint num_out_objects = 0;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred == p_OUT) {
      num_out_objects++;
    }
  }
  uint num_non_out_objects = logicObjectManager::constants.N-1-num_out_objects;
  if (num_non_out_objects <= 1) {
    MT_MSG("Too few objects left for sensible acting!");
    return NULL;
  }
  
  // calc clear and not-clear guys (out guys are not contained!)
  uintA clearGuys = logicObjectManager::constants;
  uintA nonClearGuys;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred == p_ON) {
      clearGuys.removeValueSafe(s.lits_prim(i)->atom->args(1));
      nonClearGuys.setAppend(s.lits_prim(i)->atom->args(1));
    }
  }
  if (id_hand != UINT_MAX) {
    clearGuys.removeValueSafe(id_hand);
    nonClearGuys.setAppend(id_hand);
  }
  // PRINT(clearGuys)
  // PRINT(nonClearGuys)
  CHECK(nonClearGuys.N + clearGuys.N == logicObjectManager::constants.N, "Clear guy calculation failed");
  CHECK(numberSharedElements(nonClearGuys, clearGuys) == 0, "Clear guy calculation failed");
  
  // calc out-guys
  uintA outGuys;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred == p_OUT) {
      outGuys.setAppend(s.lits_prim(i)->atom->args(0));
    }
  }
  
  
  // --------------------------------------------
  //  ACTION DETERMINATION

  TL::Predicate* p_action;
  
  double randNum;
  bool sensible_action;
  if (rnd.uni() < SAMPLING__PROB_SENSIBLE_ACTION)
    sensible_action = true;
  else
    sensible_action = false;
  
  uintA args(2);
    
  
  
  // [start] TODO
  if (rnd.uni() < 0.25) { // Do something with box
    TL::Predicate* p_BOX = logicObjectManager::getPredicate(MT::String("box"));
    uintA boxes;
    FOR1D(s.lits_prim, i) {
      if (s.lits_prim(i)->atom->pred == p_BOX)
        boxes.append(s.lits_prim(i)->atom->args(0));
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
          uint obj = TL::RobotManipulationDomain::getContainedObject(boxes(i), s);
          if (obj != UINT_MAX)
            boxedObjs.append(obj);
        }
        if (boxedObjs.N > 0) {
          args(0) = boxedObjs(rnd.num(boxedObjs.N));
          args(1) = TL::RobotManipulationDomain::getContainingBox(args(0), s);
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
        if (TL::RobotManipulationDomain::isClosed(args(0), s))
          p_action = logicObjectManager::getPredicate(MT::String("openBox"));
        else
          p_action = logicObjectManager::getPredicate(MT::String("closeBox"));
      }
      else {
        if (TL::RobotManipulationDomain::isClosed(args(0), s))
          p_action = logicObjectManager::getPredicate(MT::String("closeBox"));
        else
          p_action = logicObjectManager::getPredicate(MT::String("openBox"));
      }
    }
    return logicObjectManager::getAtom(p_action, args); 
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
      args(1) = TL::RobotManipulationDomain::getBelow(args(0), s);
      if (UINT_MAX == args(1))
        args(1) = TL::RobotManipulationDomain::getContainingBox(args(0), s);
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
          TL::RobotManipulationDomain::calcPiles(s, piles, id_table);
          TL::RobotManipulationDomain::calcSkyscraperWeights(sensibleSecondArgs, piles, skyscraper_bias, weights, true, id_table);
          // bias for putti~ng on blocks (instead of on balls)
          FOR1D(sensibleSecondArgs, i) {
            if (TL::RobotManipulationDomain::isBlock(sensibleSecondArgs(i), s)) {   // prefer blocks
              weights(i) *= 5.;
            }
          }
          args(1) = sensibleSecondArgs(basic_sample(weights));
          // lowering chance for table
          if (args(1) == id_table) {
            args(1) = sensibleSecondArgs(basic_sample(weights));
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
        args(0) = logicObjectManager::constants(rnd.num(logicObjectManager::constants.N));
        do {
          args(1) = logicObjectManager::constants(rnd.num(logicObjectManager::constants.N));
        } while (args(0) == args(1));
      }
      // (2.1.2)   if inhand(a):  place(b,c) with b and c random and not a
      else {
        do {
          args(0) = logicObjectManager::constants(rnd.num(logicObjectManager::constants.N));
        } while (args(0) == id_hand);
        do {
          args(1) = logicObjectManager::constants(rnd.num(logicObjectManager::constants.N));
        } while (args(0) == args(1)   ||   args(1) == id_hand);
      }
    }
    // (2.2)
    else {
      // (2.2.1)  if inhand(a):  lift(b,c)   with b not inhand and on(b,c)
      if (id_hand != UINT_MAX  && rnd.uni() < 0.95) {
        do {
          args(0) = logicObjectManager::constants(rnd.num(logicObjectManager::constants.N));
          args(1) = TL::RobotManipulationDomain::getBelow(args(0), s);
        } while (args(0) == id_hand  ||  args(1) == UINT_MAX);
      }
      // (2.2.2)  if inhand-nil:  lift(a,b)  with  a and b random, but _not_ on(a,b)
      else {
        uint sa0_below;
        do {
          args(0) = logicObjectManager::constants(rnd.num(logicObjectManager::constants.N));
          args(1) = logicObjectManager::constants(rnd.num(logicObjectManager::constants.N));
          sa0_below = TL::RobotManipulationDomain::getBelow(args(0), s);
        } while (args(1) == sa0_below);
      }
    }
  }
  
  if (logicObjectManager::constants.findValue(args(0)) < 0  ||  logicObjectManager::constants.findValue(args(1)) < 0) {
    PRINT(args(0));
    PRINT(args(1));
    HALT("");
  }
  
  return logicObjectManager::getAtom(p_action, args);
}



#if 0
TL::Atom* TL::robotManipulationSampling::generateAction_wellBiased_2Dactions(const State& s, uint id_table) {
  uint DEBUG = 1;
    // blocks on high towers have lower prob to be grabbed
  double skyscraper_bias = 200; // 115.0;
    
  uint i;
  // determine inhand object
  uint id_hand = TL::RobotManipulationDomain::getInhand(s);
  
  // determine number of non-out blocks
  uint no_out_blocks = 0;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred->name == "out") {
      no_out_blocks++;
    }
  }
  uint no_non_out_blocks = logicObjectManager::constants.N-1-no_out_blocks;
  if (no_non_out_blocks <= 1) {
    MT_MSG("Too few objects left for sensible acting!");
    return NULL;
  }

  TL::Predicate* p_LIFT = logicObjectManager::getPredicate(MT::String("lift"));
  TL::Predicate* p_PLACE = logicObjectManager::getPredicate(MT::String("place"));
  TL::Predicate* p_ON = logicObjectManager::getPredicate(MT::String("on"));
  TL::Predicate* p_OUT = logicObjectManager::getPredicate(MT::String("out"));
  
  // calc clear and not-clear guys (out guys are not contained!)
  uintA clearGuys = logicObjectManager::constants;
  uintA nonClearGuys;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred == p_ON) {
      clearGuys.removeValueSafe(s.lits_prim(i)->atom->args(1));
      nonClearGuys.setAppend(s.lits_prim(i)->atom->args(1));
    }
  }
  if (id_hand != UINT_MAX) {
    clearGuys.removeValueSafe(id_hand);
    nonClearGuys.setAppend(id_hand);
  }
  // PRINT(clearGuys)
  // PRINT(nonClearGuys)
  CHECK(nonClearGuys.N + clearGuys.N == logicObjectManager::constants.N, "Clear guy calculation failed");
  CHECK(numberSharedElements(nonClearGuys, clearGuys) == 0, "Clear guy calculation failed");
  
  // calc out-guys
  uintA outGuys;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred == p_OUT) {
      outGuys.setAppend(s.lits_prim(i)->atom->args(0));
    }
  }
  
  
  // determine action
  double randNum;
  bool sensible_action;
  if (rnd.uni() < SAMPLING__PROB_SENSIBLE_ACTION)
    sensible_action = true;
  else
    sensible_action = false;
  TL::Predicate* p_action;
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
  TL::RobotManipulationDomain::calcPiles(s, piles, id_table);

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
//         TL::RobotManipulationDomain::calcSkyscraperWeights(sensibleGuys, piles, skyscraper_bias, weights, false, id_table);
        sa(0) = sensibleGuys(basic_sample(weights));
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
      sa(1) = logicObjectManager::constants(rnd.num(logicObjectManager::constants.N));
    }
    else {
      sa(1) = TL::RobotManipulationDomain::getBelow(sa(0), s);
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
        TL::RobotManipulationDomain::calcSkyscraperWeights(sensibleGuys, piles, skyscraper_bias, weights, true, id_table);
                // bias for putting on blocks (instead of on balls)
        FOR1D(sensibleGuys, i) {
//                   cout<<sensibleGuys(i);
          if (TL::RobotManipulationDomain::isBlock(sensibleGuys(i), s)) {   // prefer blocks
//                     cout<<" is block"<<endl;
            weights(i) *= 3.;
          }
        }
//                 cout<<"PUTON: ";PRINT(weights);cout<<endl;
        sa(1) = sensibleGuys(basic_sample(weights));
        // lowering chance for table
        if (sa(1) == id_table) {
          sa(1) = sensibleGuys(basic_sample(weights));
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
      sa(0) = logicObjectManager::constants(rnd.num(logicObjectManager::constants.N));
    }
    else {
      sa(0) = id_hand;
    }
  }
  
  return logicObjectManager::getLiteral(p_action, true, sa);
}
#endif



TL::Atom* TL::robotManipulationSampling::generateAction_trulyRandom(const State& s, uint id_table) {
  uint i;
	
	// choose action
  HALT("funktioniertso nicht mit default action");
  TL::Predicate* action = logicObjectManager::p_actions(rnd.num(logicObjectManager::p_actions.N-1)); // omitting default action
 
  uintA sa(action->d);
  FOR1D(sa, i) {
    sa(i) = logicObjectManager::constants(rnd.num(logicObjectManager::constants.N));
		// first should never be table
    if (i==0) {
      while( sa(i) != id_table ){
        sa(i) = logicObjectManager::constants(rnd.num(logicObjectManager::constants.N));
      }
    }
  }
  
  return logicObjectManager::getAtom(action, sa);
}





TL::Trial* TL::robotManipulationSampling::generateSimulationSequence(RobotManipulationSimulator* sim, uint maxSeqLength, uint id_table) {
  uint DEBUG = 3;
  if (DEBUG>0) cout << "New sampling sequence" << endl;
  if (DEBUG > 1) {
    cout << "sampleSimulationSequence [START]" << endl;
    sim->printObjectInfo();
  }
  uint i=0;
  Trial* trial = new Trial;
  TL::State* state;
  trial->constants = logicObjectManager::constants;
  state = TL::RobotManipulationDomain::observeLogic(sim);
  if (DEBUG > 2) {cout<<endl; state->write(cout); cout<<endl;}
  trial->states.append(state);
  TL::Atom* action;
  while (i<maxSeqLength) {
    if (DEBUG>0) cout << "*** Step " << i << endl;
    action = generateAction(*trial->states.last(), id_table);
    if (action == NULL) {
      if (DEBUG>0) {cout<<"No more sensible action possible, so we stop generating."<<endl;}
      break;
    }
    if (DEBUG > 2) {action->write(cout); cout << endl;}
    TL::RobotManipulationDomain::performAction(action, sim, SAMPLING__WAIT_SEC_AFTER_ACTION);
//     sim->relaxPosition(); // needed to have a observe correct subsequent state
//     sim->simulate(SAMPLING__WAIT_SEC_AFTER_ACTION); // needed to have a observe correct subsequent state
    state = TL::RobotManipulationDomain::observeLogic(sim);
    if (DEBUG > 2) {state->write(cout,true); cout<<endl;}
    trial->states.append(state);
    trial->actions.append(action);
    i++;
    if (SAMPLING__WATCH_AFTER_ACTION)
      sim->watch();
    if (DEBUG > 2) {
      uintA changedConstants;
      LitL holdOnlyPre, holdOnlyPost;
      logicReasoning::changes(*trial->states(trial->states.N-2), *trial->states(trial->states.N-1), changedConstants, holdOnlyPre, holdOnlyPost);
      cout<<"CHANGE:  ";
      cout<<"\tBefore: ";write(holdOnlyPre);cout<<endl;
      cout<<"\tAfter: ";write(holdOnlyPost);cout<<endl;
    }
  }
  if (DEBUG > 1)
    cout << "sampleSimulationSequence [END]" << endl;
  return trial;
}







void TL::robotManipulationSampling::generateSimulationSequence_realistic(std::ostream& os, RobotManipulationSimulator* sim, uint maxSeqLength, uint id_table) {
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
  FOR1D(logicObjectManager::constants, i) {
    os<<logicObjectManager::constants(i)<<" ";
  }
  os<<endl;
  
  // (3) Data
  os<<endl;
  os<<"# data"<<endl;
  TL::State* logic_state, *logic_state_old;
  logic_state = TL::RobotManipulationDomain::observeLogic(sim);
  logic_state->write(os);
  os<<endl;
//   TL::RobotManipulationDomain::writeFeatures(os, sim);  os<<endl;
  if (DEBUG > 2) {cout<<endl; logic_state->write(cout, true); cout<<endl;}

  TL::Atom* action;
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
    TL::RobotManipulationDomain::performAction(action, sim, SAMPLING__WAIT_SEC_AFTER_ACTION);
//     sim->relaxPosition(); // needed to have a observe correct subsequent state
//     sim->simulate(SAMPLING__WAIT_SEC_AFTER_ACTION); // needed to have a observe correct subsequent state
    logic_state_old = logic_state;
    logic_state = TL::RobotManipulationDomain::observeLogic(sim);
    logic_state->write(os);
    os<<endl;
//     TL::RobotManipulationDomain::writeFeatures(os, sim);  os<<endl;
    if (DEBUG > 2) {logic_state->write(cout); cout<<endl;}
    if (DEBUG>2) {TL::RobotManipulationDomain::writeStateInfo(*logic_state);}
    i++;
    if (SAMPLING__WATCH_AFTER_ACTION)
      sim->watch();
    if (DEBUG > 2) {
      uintA changedConstants;
      LitL holdOnlyPre, holdOnlyPost;
      logicReasoning::changes(*logic_state_old, *logic_state, changedConstants, holdOnlyPre, holdOnlyPost);
      cout<<"CHANGE:  "<<endl;
      cout<<"  Before: ";write(holdOnlyPre);cout<<endl;
      cout<<"  After: ";write(holdOnlyPost);cout<<endl;
    }
  }
  if (DEBUG > 1)
    cout << "sampleSimulationSequence [END]" << endl;
}





