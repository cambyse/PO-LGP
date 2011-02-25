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

#include "TL/bwLanguage.h"




// -------------------------
//   LOGIC ENGINE
// -------------------------

// #define CLEARANCE
// #define TOWER

TL::LogicEngine* TL::bwLanguage::createLogicEngine(uintA& constants) {
  PredA p_prim;
  p_prim.append(getPredicate_table());
  p_prim.append(getPredicate_block());
  p_prim.append(getPredicate_ball());
  p_prim.append(getPredicate_on());
  p_prim.append(getPredicate_inhand());
//   p_prim.append(getPredicate_upright());
  p_prim.append(getPredicate_out());
  
  PredA p_comparisons;
  p_comparisons.append(getPredicate_comparison_constant());
  p_comparisons.append(getPredicate_comparison_dynamic());
  
  FuncA f_prim;
  f_prim.append(getFunction_size());
  
  PredA p_actions;
  p_actions.append(getPredicate_action_default());
  p_actions.append(getPredicate_action_grab());
  p_actions.append(getPredicate_action_puton());
//   p_actions.append(getPredicate_action_lift());
//   p_actions.append(getPredicate_action_place());
  
  PredA p_derived;
  p_derived.append(getPredicate_clear());
  p_derived.append(getPredicate_inhandNil());
  
  FuncA f_derived;

#ifdef CLEARANCE
  p_derived.append(getPredicate_above());    // CLEARANCE
  f_derived.append(getFunction_height());
#endif
  
#ifdef CLEARANCE
  p_prim.append(getPredicate_homies());  // CLEARANCE
  
  p_derived.append(getPredicate_above());    // CLEARANCE
  p_derived.append(getPredicate_aboveNotable());    // CLEARANCE
  p_derived.append(getPredicate_dirtyGuyBelow());    // CLEARANCE
  p_derived.append(getPredicate_diffTower());   // CLEARANCE
  p_derived.append(getPredicate_withoutHomies());    // CLEARANCE
  p_derived.append(getPredicate_inorder());    // CLEARANCE
  
  f_derived.append(getFunction_countInorder());    // CLEARANCE
#endif
  
#ifdef TOWER
  p_prim.append(getPredicate_box());
  p_prim.append(getPredicate_contains()); 
  p_prim.append(getPredicate_closed()); 
  
  p_derived.append(getPredicate_onBox());
  
  p_actions.append(getPredicate_action_openBox());
  p_actions.append(getPredicate_action_closeBox());
#endif
    
  return new TL::LogicEngine(constants, p_prim, p_derived, p_comparisons, f_prim, f_derived, p_actions);
}








// ---------------
//  PRIMTIVES - PREDS
// ---------------


TL::Predicate* p_ON = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_on() {
  if (p_ON == NULL) {
    p_ON = new Predicate;
    p_ON->d = 2;
    p_ON->name = "on";
    p_ON->id = HAND_ID__PRED_ON;
  }
  return p_ON;
}

TL::Predicate* p_TABLE = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_table() {
  if (p_TABLE== NULL) {
    p_TABLE = new Predicate;
    p_TABLE->d = 1;
    p_TABLE->name = "table";
    p_TABLE->id = HAND_ID__PRED_TABLE;
  }
  return p_TABLE;
}

TL::Predicate* p_BLOCK = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_block() {
  if (p_BLOCK == NULL) {
    p_BLOCK = new Predicate;
    p_BLOCK->d = 1;
    p_BLOCK->name = "block";
    p_BLOCK->id = HAND_ID__PRED_BLOCK;
  }
  return p_BLOCK;
}

TL::Predicate* p_BOX = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_box() {
  if (p_BOX == NULL) {
    p_BOX = new Predicate;
    p_BOX->d = 1;
    p_BOX->name = "box";
    p_BOX->id = HAND_ID__PRED_BOX;
  }
  return p_BOX;
}

TL::Predicate* p_BALL = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_ball() {
  if (p_BALL == NULL) {
    p_BALL = new Predicate;
    p_BALL->d = 1;
    p_BALL->name = "ball";
    p_BALL->id = HAND_ID__PRED_BALL;
  }
  return p_BALL;
}

TL::Predicate* p_INHAND = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_inhand() {
  if (p_INHAND == NULL) {
    p_INHAND = new Predicate;
    p_INHAND->d = 1;
    p_INHAND->name = "inhand";
    p_INHAND->id = HAND_ID__PRED_INHAND;
  }
  return p_INHAND;
}

TL::Predicate* p_UPRIGHT = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_upright() {
  if (p_UPRIGHT == NULL) {
    p_UPRIGHT = new Predicate;
    p_UPRIGHT->d = 1;
    p_UPRIGHT->name = "upright";
    p_UPRIGHT->id = HAND_ID__PRED_UPRIGHT;
  }
  return p_UPRIGHT;
}

TL::Predicate* p_OUT = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_out() {
  if (p_OUT == NULL) {
    p_OUT = new Predicate;
    p_OUT->d = 1;
    p_OUT->name = "out";
    p_OUT->id = HAND_ID__PRED_OUT;
  }
  return p_OUT;
}

TL::Predicate* p_HOMIES = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_homies() {
  if (p_HOMIES == NULL) {
    p_HOMIES = new Predicate;
    p_HOMIES->d = 2;
    p_HOMIES->name = "homies";
    p_HOMIES->id = HAND_ID__PRED_HOMIES;
  }
  return p_HOMIES;
}

TL::Predicate* p_CONTAINS = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_contains() {
  if (p_CONTAINS == NULL) {
    p_CONTAINS = new Predicate;
    p_CONTAINS->d = 2;
    p_CONTAINS->name = "contains";
    p_CONTAINS->id = HAND_ID__PRED_CONTAINS;
  }
  return p_CONTAINS;
}

TL::Predicate* p_CLOSED = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_closed() {
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
TL::Predicate* TL::bwLanguage::getPredicate_action_default() {
  if (p_ACTION_DEFAULT == NULL) {
    p_ACTION_DEFAULT = new Predicate;
    p_ACTION_DEFAULT->d = 0;
    p_ACTION_DEFAULT->name = "default";
    p_ACTION_DEFAULT->id = TL_DEFAULT_ACTION_PRED__ID;
    p_ACTION_DEFAULT->type = TL_PRED_ACTION;
  }
  return p_ACTION_DEFAULT;
}

TL::Predicate* p_GRAB = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_action_grab() {
  if (p_GRAB == NULL) {
    p_GRAB = new Predicate;
    p_GRAB->d = 1;
    p_GRAB->name = "grab";
    p_GRAB->id = HAND_ID__GRAB;
    p_GRAB->type = TL_PRED_ACTION;
  }
  return p_GRAB;
}

TL::Predicate* p_PUTON = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_action_puton() {
  if (p_PUTON == NULL) {
    p_PUTON = new Predicate;
    p_PUTON->d = 1;
    p_PUTON->name = "puton";
    p_PUTON->id = HAND_ID__PUTON;
    p_PUTON->type = TL_PRED_ACTION;
  }
  return p_PUTON;
}

TL::Predicate* p_LIFT = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_action_lift() {
  if (p_LIFT == NULL) {
    p_LIFT = new Predicate;
    p_LIFT->d = 2;
    p_LIFT->name = "lift";
    p_LIFT->id = HAND_ID__LIFT;
    p_LIFT->type = TL_PRED_ACTION;
  }
  return p_LIFT;
}

TL::Predicate* p_PLACE = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_action_place() {
  if (p_PLACE == NULL) {
    p_PLACE = new Predicate;
    p_PLACE->d = 2;
    p_PLACE->name = "place";
    p_PLACE->id = HAND_ID__PLACE;
    p_PLACE->type = TL_PRED_ACTION;
  }
  return p_PLACE;
}

TL::Predicate* p_OPEN_BOX = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_action_openBox() {
  if (p_OPEN_BOX == NULL) {
    p_OPEN_BOX= new Predicate;
    p_OPEN_BOX->d = 1;
    p_OPEN_BOX->name = "openBox";
    p_OPEN_BOX->id = HAND_ID__OPEN_BOX;
    p_OPEN_BOX->type = TL_PRED_ACTION;
  }
  return p_OPEN_BOX;
}

TL::Predicate* p_CLOSE_BOX = NULL;
TL::Predicate* TL::bwLanguage::getPredicate_action_closeBox() {
  if (p_CLOSE_BOX == NULL) {
    p_CLOSE_BOX = new Predicate;
    p_CLOSE_BOX->d = 1;
    p_CLOSE_BOX->name = "closeBox";
    p_CLOSE_BOX->id = HAND_ID__CLOSE_BOX;
    p_CLOSE_BOX->type = TL_PRED_ACTION;
  }
  return p_CLOSE_BOX;
}



// ---------------
//  PRIMTIVES - COMPARISONS
// ---------------

TL::ComparisonPredicate* p_COMP_CONST = NULL;
TL::ComparisonPredicate* TL::bwLanguage::getPredicate_comparison_constant() {
  if (p_COMP_CONST == NULL) {
    p_COMP_CONST = new TL::ComparisonPredicate;
    p_COMP_CONST->d = 1;
    p_COMP_CONST->name = "comp_constant";
    p_COMP_CONST->id = HAND_ID__PRED_COMP_CONSTANT;
    p_COMP_CONST->constantBound = true;
  }
  return p_COMP_CONST;
}

TL::ComparisonPredicate* p_COMP_DYN = NULL;
TL::ComparisonPredicate* TL::bwLanguage::getPredicate_comparison_dynamic() {
  if (p_COMP_DYN == NULL) {
    p_COMP_DYN = new TL::ComparisonPredicate;
    p_COMP_DYN->d = 2;
    p_COMP_DYN->name = "comp_dynamic";
    p_COMP_DYN->id = HAND_ID__PRED_COMP_DYNAMIC;
    p_COMP_DYN->constantBound = false;
  }
  return p_COMP_DYN;
}


// ---------------
//  PRIMTIIVES - FUNCTIONS
// ---------------

TL::Function* f_SIZE = NULL;
TL::Function* TL::bwLanguage::getFunction_size() {
  if (f_SIZE==NULL) {
    f_SIZE = new Function;
    f_SIZE->d = 1;
    f_SIZE->name = "size";
    f_SIZE->id = HAND_ID__FUNCTION_SIZE;
    f_SIZE->range = TUP(1,2,3,4,5);
  }
  return f_SIZE;
}







// ---------------
//  DERIVED - PREDICATES
// ---------------

TL::ConjunctionPredicate* p_CLEAR = NULL;
TL::ConjunctionPredicate* TL::bwLanguage::getPredicate_clear() {
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
TL::TransClosurePredicate* TL::bwLanguage::getPredicate_above() {
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
TL::ConjunctionPredicate* TL::bwLanguage::getPredicate_aboveNotable() {
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
TL::ConjunctionPredicate* TL::bwLanguage::getPredicate_dirtyGuyBelow() {
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
TL::ConjunctionPredicate* TL::bwLanguage::getPredicate_diffTower() {
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
TL::ConjunctionPredicate* TL::bwLanguage::getPredicate_withoutHomies() {
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
TL::ConjunctionPredicate* TL::bwLanguage::getPredicate_inorder() {
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
TL::ConjunctionPredicate* TL::bwLanguage::getPredicate_inhandNil() {
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
TL::ConjunctionPredicate* TL::bwLanguage::getPredicate_onBox() {
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
TL::CountFunction* TL::bwLanguage::getFunction_height() {
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
TL::AverageFunction* TL::bwLanguage::getFunction_avgheight() {
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
TL::SumFunction* TL::bwLanguage::getFunction_sumheight() {
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
TL::CountFunction* TL::bwLanguage::getFunction_countInorder() {
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

void TL::bwLanguage::shutdownLogic() {
  if (p_ON != NULL)
    delete p_ON;
  NIY;
}













// -----------------------------------
//   PREDICATE TUPLES
// -----------------------------------


TL::FunctionValue* TL::bwLanguage::createFunctionValue_size(uint obj, double size, LogicEngine* le) {
  uintA sa(1);  sa(0)=obj;
  return le->getFV(le->getFunction(MT::String("size")), sa, size);
}
	


TL::PredicateInstance* TL::bwLanguage::createPredicateInstance_on(uint above, uint below, LogicEngine* le) {
  uintA sa(2);  sa(0)=above;  sa(1)=below;
  return le->getPI(le->getPredicate(MT::String("on")), true, sa);
}

TL::PredicateInstance* TL::bwLanguage::createPredicateInstance_table(uint obj, LogicEngine* le) {
  uintA sa(1);  sa(0)=obj;
  return le->getPI(le->getPredicate(MT::String("table")), true, sa);
}

TL::PredicateInstance* TL::bwLanguage::createPredicateInstance_block(uint obj, LogicEngine* le) {
  uintA sa(1);  sa(0)=obj;
  return le->getPI(le->getPredicate(MT::String("block")), true, sa);
}

TL::PredicateInstance* TL::bwLanguage::createPredicateInstance_ball(uint obj, LogicEngine* le) {
  uintA sa(1);  sa(0)=obj;
  return le->getPI(le->getPredicate(MT::String("ball")), true, sa);
}

TL::PredicateInstance* TL::bwLanguage::createPredicateInstance_box(uint obj, LogicEngine* le) {
  uintA sa(1);  sa(0)=obj;
  return le->getPI(le->getPredicate(MT::String("box")), true, sa);
}

TL::PredicateInstance* TL::bwLanguage::createPredicateInstance_upright(uint obj, LogicEngine* le) {
  uintA sa(1);  sa(0)=obj;
  return le->getPI(le->getPredicate(MT::String("upright")), true, sa);
}

TL::PredicateInstance* TL::bwLanguage::createPredicateInstance_out(uint obj, LogicEngine* le) {
  uintA sa(1);  sa(0)=obj;
  return le->getPI(le->getPredicate(MT::String("out")), true, sa);
}

TL::PredicateInstance* TL::bwLanguage::createPredicateInstance_inhand(uint obj, LogicEngine* le) {
  uintA sa(1);  sa(0)=obj;
  return le->getPI(le->getPredicate(MT::String("inhand")), true, sa);
}

TL::PredicateInstance* TL::bwLanguage::createPredicateInstance_homies(uint obj1, uint obj2, LogicEngine* le) {
  uintA sa(2);  sa(0)=obj1;  sa(1)=obj2;
  return le->getPI(le->getPredicate(MT::String("homies")), true, sa);
}

TL::PredicateInstance* TL::bwLanguage::createPredicateInstance_contains(uint box, uint obj, LogicEngine* le)  {
  uintA sa(2);  sa(0)=box;  sa(1)=obj;
  return le->getPI(le->getPredicate(MT::String("contains")), true, sa);
}

TL::PredicateInstance* TL::bwLanguage::createPredicateInstance_closed(uint box, LogicEngine* le)  {
  uintA sa(1);  sa(0)=box;
  return le->getPI(le->getPredicate(MT::String("closed")), true, sa);
}


TL::PredicateInstance* TL::bwLanguage::createPredicateInstance_grab(uint obj, LogicEngine* le) {
  uintA sa(1);  sa(0)=obj;
  return le->getPI(le->getPredicate(MT::String("grab")), true, sa);
}


TL::PredicateInstance* TL::bwLanguage::createPredicateInstance_puton(uint obj, uint on, LogicEngine* le) {
  uintA sa(1);  sa(0)=obj;
  return le->getPI(le->getPredicate(MT::String("puton")), true, sa);
}











// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    GOAL LIBRARY
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------



TL::Reward* TL::bwLanguage::RewardLibrary::on(uint o1, uint o2, LogicEngine* le) {
  TL::Predicate* p_ON = le->getPredicate(MT::String("on"));
  uintA sa2(2);
  sa2(0)=o1;
  sa2(1)=o2;
  TL::PredicateInstance* pt = le->getPI(p_ON, true, sa2);
  Reward* reward = new PredicateReward(pt);
  return reward;
}

TL::Reward* TL::bwLanguage::RewardLibrary::inhand(uint o1, LogicEngine* le) {
  TL::Predicate* p_INHAND = le->getPredicate(MT::String("inhand"));
  uintA sa1(1);
  sa1(0)=o1;
  TL::PredicateInstance* pt = le->getPI(p_INHAND, true, sa1);
  Reward* reward = new PredicateReward(pt);
  return reward;
}

TL::Reward* TL::bwLanguage::RewardLibrary::stack(LogicEngine* le) {
  // needs p_ABOVE, f_HEIGHT, f_SUM_HEIGHT
  MT_MSG("Stack defined by average (not max)");
  uintA empty;
  FuncA funcs2add;
  PredA preds2add;
  if (!le->getPredicate(MT::String("above"))) {
    TL::TransClosurePredicate* p_ABOVE1 = getPredicate_above();
    p_ABOVE1->basePred = le->getPredicate(MT::String("on")); // HACK da in regelfiles bis juni 2009 on andere id hat
    preds2add.append(p_ABOVE1);
  }
  if (!le->getPredicate(MT::String("aboveNotable"))) {
    preds2add.append(getPredicate_aboveNotable());
  }
  if (!le->getFunction(MT::String("height"))) {
    funcs2add.append(getFunction_height());
  }
  if (!le->getFunction(MT::String("sum_height"))) {
    funcs2add.append(getFunction_sumheight());
  }
  if (preds2add.N > 0)
    le->addPredicates(preds2add);
  if (funcs2add.N > 0)
    le->addFunctions(funcs2add);
  TL::FunctionInstance* fvw = le->getFI(le->getFunction(MT::String("sum_height")), empty);
  Reward* reward = new MaximizeFunctionReward(fvw);
  return reward;
}


TL::Reward* TL::bwLanguage::RewardLibrary::tower(uintA& objs, LogicEngine* le) {
  TL::Predicate* p_ON = le->getPredicate(MT::String("on"));
  PredIA pts;
  uint i;
  uintA sa2(2);
  FOR1D(objs, i) {
    sa2(0)=objs(i);
    if (i<objs.N-1)
      sa2(1)=objs(i+1);
    else
      sa2(1)=60;  // table id in my ors simulator
    pts.append(le->getPI(p_ON, true, sa2));
  }
  PredicateListReward* reward = new PredicateListReward(pts);
  return reward;
}


TL::Reward* TL::bwLanguage::RewardLibrary::clearance(LogicEngine* le) {
  FuncA funcs2add;
  PredA preds2add;
  if (!le->getPredicate(MT::String("above"))) {
    TL::TransClosurePredicate* p_ABOVE1 = getPredicate_above();
    p_ABOVE1->basePred = le->getPredicate(MT::String("on")); // HACK da in regelfiles bis juni 2009 on andere id hat
    preds2add.append(p_ABOVE1);
  }
  if (!le->getPredicate(MT::String("dirtyGuyBelow"))) {
    preds2add.append(getPredicate_dirtyGuyBelow());
  }
  if (!le->getPredicate(MT::String("differentTower"))) {
    preds2add.append(getPredicate_diffTower());
  }
  if (!le->getPredicate(MT::String("withoutHomies"))) {
    preds2add.append(getPredicate_withoutHomies());
  }
  if (!le->getPredicate(MT::String("inorder"))) {
    preds2add.append(getPredicate_inorder());
  }
  if (!le->getFunction(MT::String("count_inorder"))) {
    funcs2add.append(getFunction_countInorder());
  }
  
  le->addPredicates(preds2add);
  le->addFunctions(funcs2add);
  
//   le->dependencyGraph.writeNice();
  uintA empty;
  TL::FunctionInstance* fi = le->getFI(getFunction_countInorder(), empty);
  Reward* reward = new MaximizeFunctionReward(fi);
  return reward;
}



TL::PredicateListReward* TL::bwLanguage::sampleGroundGoal__stack(const uintA& blocks, const uintA& balls, uint table_id, LogicEngine& le, bool leave_existing_towers, TL::State* state) {
  uint DEBUG = 1;
  if (DEBUG>0) {cout<<"TL::bwLanguage::sampleGroundGoal__stack [START]"<<endl;}
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
  
  PredIA pis;
  FOR1D(towers, i) {
    FOR1D(towers(i), k) {
      if (k==0)
        continue;
      pis.append(createPredicateInstance_on(towers(i)(k), towers(i)(k-1), &le));
    }
  }
  
  if (DEBUG>0) {cout<<"LOGIC:"<<endl;  TL::writeNice(pis); cout<<endl;}
  
  if (DEBUG>0) {cout<<"TL::bwLanguage::sampleGroundGoal__stack [END]"<<endl;}
  return new PredicateListReward(pis);
}









TL::PredicateListReward* TL::bwLanguage::sampleGroundGoal__clearance(const TL::State& current_state, uint table_id, LogicEngine& le) {
  uint DEBUG = 1;
  if (DEBUG>0) {cout<<"TL::bwLanguage::sampleGroundGoal__clearance [START]"<<endl;}

  uint i, k;

  //-----------------------------------------------------------------------------
  // (1) Determine classes to be ordered
  
  MT::Array< uintA > gangs;
  getHomieGangs(gangs, current_state, &le);

  // Determine gangs (= gangs of homies)
  if (DEBUG>1) {
    cout<<"GANGS:"<<endl;
    FOR1D(gangs, i) {
      cout << i << ":  " << gangs(i) << endl;
    }
  }
  
  uintA gang_ids;
  FOR1D(gangs, i) {
    if (!isInorderGang(gangs(i), current_state, &le))
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
  // (2) Stack em fucking somehow togetha
  
  MT::Array< uintA > towers;
  FOR1D(combo, i) {
    uintA tower;
    uintA& local_gang = gangs(combo(i));
    FOR1D(local_gang, k) {
      uint candidate_position = rnd.num(tower.N+1);
      // lower chance for balls to be inside
//       if (isBall(local_gang(k), current_state, &le)  &&  candidate_position != tower.N)
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
        if (isBall(towers(i)(k), current_state, &le)) {
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
        if (isBall(towers(i)(k), current_state, &le)) {
          cerr<<"o";
        }
      }
      cerr << endl;
    }
    cerr<<"****************"<<endl;
  }
  

  //-----------------------------------------------------------------------------
  // (3) Translate to logic
  PredIA pis;
  FOR1D(towers, i) {
    FOR1D(towers(i), k) {
      if (k==0)
        continue;
      pis.append(createPredicateInstance_on(towers(i)(k), towers(i)(k-1), &le));
    }
  }
  
  if (DEBUG>1) {cout<<"LOGIC:"<<endl;  TL::writeNice(pis); cout<<endl;}
  
  if (DEBUG>0) {cout<<"TL::bwLanguage::sampleGroundGoal__clearance [END]"<<endl;}
  return new PredicateListReward(pis);
}

















// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    STATE INFORMATION HELPERS
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


bool TL::bwLanguage::isBlock(uint id, const TL::State& s, LogicEngine* le) {
  return le->holds_straight(id, MT::String("block"), s);
}


bool TL::bwLanguage::isOut(uint id, const TL::State& s, LogicEngine* le) {
  return le->holds_straight(id, MT::String("out"), s);
}

bool TL::bwLanguage::isInhand(uint id, const TL::State& s, LogicEngine* le) {
  return le->holds_straight(id, MT::String("inhand"), s);
}

bool TL::bwLanguage::isTable(uint id, const TL::State& s, LogicEngine* le) {
  return le->holds_straight(id, MT::String("table"), s);
}

bool TL::bwLanguage::isBall(uint id, const TL::State& s, LogicEngine* le) {
  return le->holds_straight(id, MT::String("ball"), s);
}

bool TL::bwLanguage::isBox(uint id, const TL::State& s, LogicEngine* le) {
  if (le->getPredicate(MT::String("box")) == NULL)
    return false;
  return le->holds_straight(id, MT::String("box"), s);
}

bool TL::bwLanguage::isClosed(uint box_id, const TL::State& s, LogicEngine* le) {
  return le->holds_straight(box_id, MT::String("closed"), s);
}

bool TL::bwLanguage::isInorderGang(const uintA gang, const TL::State& s, LogicEngine* le) {
  CHECK(gang.N > 0, "");
  if (le->getPredicate(MT::String("inorder")) == NULL) {
    NIY;
  }
  else {
    return le->holds_straight(gang(0), MT::String("inorder"), s);
  }
}




uint TL::bwLanguage::getBelow(uint id, const TL::State& s, LogicEngine* le) {
  TL::Predicate* p_ON = le->getPredicate(MT::String("on"));
  uint i;
  FOR1D(s.pi_prim, i) {
    if (s.pi_prim(i)->pred == p_ON) {
      if (s.pi_prim(i)->args(0) == id)
        return s.pi_prim(i)->args(1);
    }
  }
  return UINT_MAX;
}

uint TL::bwLanguage::getAbove(uint id, const TL::State& s, LogicEngine* le) {
  TL::Predicate* p_ON = le->getPredicate(MT::String("on"));
  uint i;
  FOR1D(s.pi_prim, i) {
    if (s.pi_prim(i)->pred == p_ON) {
      if (s.pi_prim(i)->args(1) == id)
        return s.pi_prim(i)->args(0);
    }
  }
  return UINT_MAX;
}

void TL::bwLanguage::getBelowObjects(uintA& ids, uint id_top, const TL::State& s, LogicEngine* le) {
  LogicEngine::getRelatedObjects(ids, id_top, true, *le->getPredicate(MT::String("above")), s);
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

void TL::bwLanguage::getAboveObjects(uintA& ids, uint id_top, const TL::State& s, LogicEngine* le) {
  LogicEngine::getRelatedObjects(ids, id_top, false, *le->getPredicate(MT::String("above")), s);
}

uint TL::bwLanguage::getInhand(const TL::State& s, LogicEngine* le) {
  TL::Predicate* p_INHAND = le->getPredicate(MT::String("inhand"));
  uint i;
  FOR1D(s.pi_prim, i) {
    if (s.pi_prim(i)->pred == p_INHAND) {
      return s.pi_prim(i)->args(0);
    }
  }
  return UINT_MAX;
}

void TL::bwLanguage::getBoxes(uintA& ids, const TL::State& s, LogicEngine* le) {
  ids.clear();
  TL::Predicate* p_BOX = le->getPredicate(MT::String("box"));
  uint i;
  FOR1D(s.pi_prim, i) {
    if (s.pi_prim(i)->pred == p_BOX) {
      ids.append(s.pi_prim(i)->args(0));
    }
  }
}


uint TL::bwLanguage::getContainingBox(uint obj_id, const TL::State& s, LogicEngine* le) {
  TL::Predicate* p_CONTAINS = le->getPredicate(MT::String("contains"));
  uint i;
  FOR1D(s.pi_prim, i) {
    if (s.pi_prim(i)->pred == p_CONTAINS) {
      if (s.pi_prim(i)->args(1) == obj_id)
        return s.pi_prim(i)->args(0);
    }
  }
  return UINT_MAX;
}


uint TL::bwLanguage::getContainedObject(uint box_id, const TL::State& s, LogicEngine* le) {
  TL::Predicate* p_CONTAINS = le->getPredicate(MT::String("contains"));
  uint i;
  FOR1D(s.pi_prim, i) {
    if (s.pi_prim(i)->pred == p_CONTAINS) {
      if (s.pi_prim(i)->args(0) == box_id)
        return s.pi_prim(i)->args(1);
    }
  }
  return UINT_MAX;
}


void TL::bwLanguage::getHomieGangs(MT::Array< uintA >& homieGangs, const TL::State& s, LogicEngine* le) {
  homieGangs.clear();
  Predicate* p_HOMIES = le->getPredicate(MT::String("homies"));
  if (p_HOMIES == NULL)
    return;
  uint i, k;
  boolA constants_covered(le->constants.N);
  constants_covered.setUni(false);
  FOR1D(le->constants, i) {
    if (le->holds_straight(le->constants(i), MT::String("table"), s))
      continue;
    if (constants_covered(i))
      continue;
    uintA homies;
    LogicEngine::getRelatedObjects(homies, le->constants(i), true, *p_HOMIES, s);
    homies.insert(0, le->constants(i));
    constants_covered(i) = true;
    FOR1D(homies, k) {
      constants_covered(le->constants.findValue(homies(k))) = true;
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

double TL::bwLanguage::reward_buildTower(const State& s) {
  uint DEBUG=0;
  if (DEBUG>0) {cout<<"bwLanguage::reward_buildTower [START]"<<endl;}
  uintA piles;
  uint id_table = UINT_NIL;
  uint i;
  FOR1D(s.pi_prim, i) {
    if (s.pi_prim(i)->pred->name == "table"  &&  s.pi_prim(i)->positive) {
      id_table = s.pi_prim(i)->args(0);
      break;
    }
  }
  CHECK(i<s.pi_prim.N, "table id not found");
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
  if (DEBUG>0) {cout<<"bwLanguage::reward_buildTower [END]"<<endl;}
  return reward;
}



void TL::bwLanguage::calcPiles(const State& s, uintA& piles, uint id_table) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<" TL::bwLanguage::calcPiles [START]"<<endl;}
  MT::Array< uintA > piles_unsorted;
  uint i;
  bool inserted;
  FOR1D(s.pi_prim, i) {
    // on(A,B)
    if (s.pi_prim(i)->pred->id == HAND_ID__PRED_ON) {
      inserted = false;
      uint A_2_top;
      FOR1D(piles_unsorted, A_2_top) {
        if (piles_unsorted(A_2_top).N == 0)
          continue;
        // pile with [A, ..., top]  -->  put B below
        if (piles_unsorted(A_2_top)(0) == s.pi_prim(i)->args(0)) {
          piles_unsorted(A_2_top).insert(0, s.pi_prim(i)->args(1));
          inserted = true;
        }
      }
      uint table_2_B;
      FOR1D(piles_unsorted, table_2_B) {
        if (piles_unsorted(table_2_B).N == 0)
          continue;
        // pile with [table, lastBlock, ..., B]  -->  put A on top
        if (piles_unsorted(table_2_B).last() == s.pi_prim(i)->args(1)) {
          if (inserted) {
            // when trying to insert a second time
            // find previous insertion point
            FOR1D(piles_unsorted, A_2_top) {
              if (piles_unsorted(A_2_top).N == 0)
                continue;
              // pile with [A, ..., top]  -->  put B below
              if (piles_unsorted(A_2_top)(0) == s.pi_prim(i)->args(1)) {  // anderer check als oben, verdammt, da B ja jetzt schon eingefuegt!
                break;
              }
            }
            CHECK(A_2_top != piles_unsorted.N, "");
            piles_unsorted(table_2_B).setAppend(piles_unsorted(A_2_top)); // schmeisst doppeleintrag raus
            piles_unsorted(A_2_top).clear();
          }
          else {
            piles_unsorted(table_2_B).append(s.pi_prim(i)->args(0));
            inserted = true;
          }
        }
      }
      if (!inserted) {
        uintA newPile;
        newPile.append(s.pi_prim(i)->args(1));
        newPile.append(s.pi_prim(i)->args(0));
        piles_unsorted.append(newPile);
      }
    }
  }
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
    
    // sort by height
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
    // reorder piles
  piles.resize(piles_unsorted.d0, heights(sortedIndices(0)));
  piles.setUni(UINT_NIL);
  FOR1D(piles, i) {
    FOR1D(piles_unsorted(sortedIndices(i)), j) {
      piles(i,j) = piles_unsorted(sortedIndices(i))(j);
    }
  }
  if (DEBUG>0)
    PRINT(piles);
  if (DEBUG>0) {cout<<" TL::bwLanguage::calcPiles [END]"<<endl;}
}




// unfound objects get height = 0
void TL::bwLanguage::calcHeights(const uintA& objects, const uintA& piles, uintA& object_heights, uint id_table) {
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


void TL::bwLanguage::calcSkyscraperWeights(const uintA& heights, double skyscraper_bias, arr& weights, bool highGood, uint id_table) {
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

void TL::bwLanguage::calcSkyscraperWeights(const uintA& objects, const uintA& piles, double skyscraper_bias, arr& weights, bool highGood, uint id_table) {
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






void TL::bwLanguage::writeStateInfo(const State& s, LogicEngine* le, ostream& out) {
  out<<"--"<<endl;
  uint i, k;
  uint id_table = UINT_MAX;
  FOR1D(s.pi_prim, i) {
    if (s.pi_prim(i)->pred->name == "table") {
      id_table = s.pi_prim(i)->args(0);
      break;
    }
  }
  CHECK(id_table != UINT_MAX, "");
  
  // Piles
  uintA piles;
  calcPiles(s, piles, id_table);
  
//   PRINT(piles);
  
  for (i=0; i<piles.d0; i++) {
    k = 0;
    while (k < piles.d1 && piles(i,k) != UINT_MAX) {
      if (k>0) out<<" ";
      if (isBox(piles(i,k), s, le))
        out << "b";
      out<<piles(i,k);
      if (isBall(piles(i,k), s, le))
        out<<"o";
      else if (isBox(piles(i,k), s, le)) {
        out<<"[ ";
        uint obj = getContainedObject(piles(i,k), s, le);
        if (obj != UINT_MAX) {
          out<<obj;
          if (isBall(obj, s, le))
            out<<"o";
          out<<" ";
        }
        if (isClosed(piles(i,k), s, le))
          out<<"]";
      }
      k++;
    }
    out << endl;
  }
  
  
  // Boxes
//   uintA boxes;
//   if (le->getPredicate(MT::String("box")) != NULL) {
//     getBoxes(boxes, s, le);
//     if (boxes.N>0) {
//       out<<"--"<<endl;
//       FOR1D(boxes, i) {
//         out<<boxes(i)<<" [ ";
//         uint obj = getContainedObject(boxes(i), s, le);
//         if (obj != UINT_MAX) {
//           out<<obj;
//           if (isBall(obj, s, le))
//             out<<"o";
//           out<<" ";
//         }
//         if (isClosed(boxes(i), s, le))
//           out<<" ]";
//         out<<endl;
//       }
//     }
//   }
  
  out<<"--"<<endl;
  
  out<<"H ";
  uint id_inhand = getInhand(s, le);
  if (id_inhand != UINT_MAX) {
    out<<id_inhand;
    if (isBall(id_inhand, s, le))
      out<<"o";
  }
  else
    out << "-";
  out<<endl;
  
  
  MT::Array< uintA > homieGangs;
  getHomieGangs(homieGangs, s, le);
  if (homieGangs.N > 0   &&   homieGangs(0).N > 1) {
    out<<"--"<<endl;
    out<<"Gangs:"<<endl;
    FOR1D(homieGangs, i) {
      out<<homieGangs(i)<<endl;
    }
  }
  
  out<<"--"<<endl;
}

