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

#include "relationalLogic.h"

#define NOISE_MARKER 'N'


bool TL::TermType::subsumes(const TermType& t) const {
  return this->type_id == t.type_id  ||  t.type_id == ANY_id  || this->type_id == ANY_id;
}


bool TL::DisjunctionTermType::subsumes(const TermType& t) const {
  bool subsumes = false;
  uint i;
  FOR1D(this->base_types, i) {
    if (this->base_types(i)->subsumes(t))
      subsumes = true;
  }
  return subsumes;
}



void TL::PredicateInstance::name(MT::String& name) const {
  name.clear();
  if (!positive) name<<"N";
  name<<this->pred->name<<"(";
  uint i;
  FOR1D(args, i) {
    if (i == args.N-1)
      name << args(i);
    else
      name << args(i) << " ";
  }
  name<<")";
}


void TL::ConjunctionPredicate::getFreeVars(uintA& freeVars) const {
  freeVars.clear();
  uint i;
  FOR1D(basePreds_mapVars2conjunction, i) {
    if (basePreds_mapVars2conjunction(i)>=d)
      freeVars.setAppend(basePreds_mapVars2conjunction(i));
  }
}

bool TL::ComparisonPredicateInstance::compare(double value) {
    CHECK(this->hasConstantBound(), "CPT should have constant bound!")
    return TL::compare(value, this->bound, this->comparisonType);
}

bool TL::ComparisonPredicateInstance::compare(double value_left, double value_right) {
    CHECK(!this->hasConstantBound(), "CPT should have dynamic bound!")
    return TL::compare(value_left, value_right, this->comparisonType);
}


void TL::Rule::involvedConcepts(PredA& preds, FuncA& funcs) {
    uint i, k;
    FOR1D(this->context, i) {
        preds.setAppend(this->context(i)->pred);
        if (this->context(i)->pred->type == TL_PRED_COMPARISON) {
            TL::ComparisonPredicateInstance* cpt = dynamic_cast<TL::ComparisonPredicateInstance*>(this->context(i));
            CHECK(cpt!=NULL,"cast failed");
            funcs.setAppend(cpt->f);
            PredA p_pre;
            FuncA f_pre;
            baseConcepts(cpt->f, p_pre, f_pre);
            preds.setAppend(p_pre);
            funcs.setAppend(f_pre);
        }
        else {
            PredA p_pre;
            FuncA f_pre;
            baseConcepts(this->context(i)->pred, p_pre, f_pre);
            preds.setAppend(p_pre);
            funcs.setAppend(f_pre);
        }
    }
    FOR1D(this->outcomes, i) {
      FOR1D(this->outcomes(i), k) {
        preds.setAppend(this->outcomes(i)(k)->pred);
        if (this->outcomes(i)(k)->pred->type == TL_PRED_COMPARISON) {
          TL::ComparisonPredicateInstance* cpt = dynamic_cast<TL::ComparisonPredicateInstance*>(this->outcomes(i)(k));
          CHECK(cpt!=NULL,"cast failed");
          funcs.setAppend(cpt->f);
          PredA p_pre;
          FuncA f_pre;
          baseConcepts(cpt->f, p_pre, f_pre);
          preds.setAppend(p_pre);
          funcs.setAppend(f_pre);
        }
        else {
          PredA p_pre;
          FuncA f_pre;
          baseConcepts(this->outcomes(i)(k)->pred, p_pre, f_pre);
          preds.setAppend(p_pre);
          funcs.setAppend(f_pre);
        }
      }
    }
}


void TL::Rule::copyBody(const TL::Rule& other) {
  this->context = other.context;
  this->action = other.action;
  this->outcomes.resize(other.outcomes.N);
  uint i;
  FOR1D(other.outcomes, i) {
    this->outcomes(i) = other.outcomes(i);
  }
  this->probs = other.probs;
  this->noise_changes = other.noise_changes;
  this->outcome_rewards = other.outcome_rewards;
}






// ------------------------------------------------
// ------------------------------------------------
// ------------------------------------------------
//   W R I T E
// ------------------------------------------------
// ------------------------------------------------
// ------------------------------------------------


void TL::TermType::write(ostream& os) const {
  os << type_id << " " << name << " " << typeI;
}

void TL::DisjunctionTermType::write(ostream& os) const {
  TermType::write(os);
  uint i;
  os << " [";
  FOR1D(base_types, i) {
    os << " " << base_types(i)->type_id;
  }
  os << " ]";
}

void TL::Function::write(ostream& os) const {
    os << id << " " << name << " " << d << " " << type << " " << category << " " << range;
}


void TL::CountFunction::write(ostream& os) const {
    Function::write(os);
    os << " " << countedPred->id << " " << countedPred_mapVars2derived;
}

void TL::AverageFunction::write(ostream& os) const {
  Function::write(os);
  os << " " << f_base->id;
}

void TL::SumFunction::write(ostream& os) const {
  Function::write(os);
  os << " " << f_base->id;
}

void TL::MaxFunction::write(ostream& os) const {
  Function::write(os);
  os << " " << f_base->id;
}

void TL::ChangeFunction::write(ostream& os) const {
  Function::write(os);
  os << " " << f_base->id;
}

void TL::RewardFunction::write(ostream& os) const {
  Function::write(os);
  os << reward_value;
  os<<" ( ";
  uint i;
  FOR1D(grounded_pis, i) {
    os<<" ( ";
    grounded_pis(i)->write(os);
    os<<" ) ";
  }
  os<<" )";
}


void TL::Predicate::write(ostream& os) const {
    os << id << " " << name << " " << d << " " << type << " " << category;
    uint i;
    FOR1D(arg_types, i) {
      os << " " << arg_types(i)->type_id;
    }
}


void TL::ComparisonPredicate::write(ostream& os) const {
    Predicate::write(os);
    os << " " << constantBound;
}


void TL::ConjunctionPredicate::write(ostream& os) const {
    Predicate::write(os);
//     os << id << " " << name << " " << d << " " << type << " " << category;
    os << " [ ";
    os << basePreds.N << " ";
    uint i;
    os << freeVarsAllQuantified;
    os << " [ ";
    FOR1D(basePreds, i) {
        os << basePreds(i)->id << " ";
    }
    os << " ] ";
//     os << " [ ";
    os << basePreds_positive;
//     os << " ] ";
//     os << " [ ";
    os << basePreds_mapVars2conjunction;
//     os << " ] ";
    os << " ] ";
}


void TL::TransClosurePredicate::write(ostream& os) const {
    Predicate::write(os);
//     os << id << " " << name << " " << d << " " << type << " " << category;
    os << " [ ";
    os <<  basePred->id;
    os << " ] ";
}


void TL::CountPredicate::write(ostream& os) const {
    Predicate::write(os);
//     os << id << " " << name << " " << d << " " << type << " " << category;
    os << " [ ";
    os << countedPred->id << " ";
    os << countedPred_mapVars2derived << " ";
    os << bound << " ";
    os << compType << " ";
    os << countedPred_isComparison << " ";
    if (countedPred_isComparison) {
        os << comparison_f->id << " ";
        os << comparison_bound << " ";
        os << comparison_compType << " ";
        
    }
    os << " ] ";
}



void TL::TermType::writeNice(ostream& os) const {
  os << type_id << " " << name;
}

void TL::DisjunctionTermType::writeNice(ostream& os) const {
  TermType::writeNice(os);
  uint i;
  os << " = (";
  FOR1D(base_types, i) {
    if (i>0)
      os<<" OR";
    os << " "<<base_types(i)->name;
  }
  os << ")";
}


void TL::Function::writeNice(ostream& os) const {
	os << "f " << id << " " << name << " /" << d;
    switch(category) {
        case TL_PRIMITIVE: os << " primitive "; break;
        case TL_DERIVED: os << " f_derived "; break;
    }
}

void TL::Predicate::writeNice(ostream& os) const {
	os << "p " << id << " " << name << " /" << d;
  switch(category) {
      case TL_PRIMITIVE: os << " primitive "; break;
      case TL_DERIVED: os << " p_derived "; break;
  }
	switch(type) {
        case TL_PRED_ACTION: os << " action"; break;
        case TL_PRED_SIMPLE: os << " simple"; break;
        case TL_PRED_COMPARISON: os << " comparison "; break;
        case TL_PRED_CONJUNCTION: os << " conjunction "; break;
        case TL_PRED_TRANS_CLOSURE: os << " transClosure "; break;
        case TL_PRED_COUNT: os << " count "; break;
	}
  if (arg_types.N > 0) {
    uint i;
    os<<"(";
    FOR1D(arg_types, i) {
      if (i>0)
        os<<" ";
      os << arg_types(i)->name;
    }
    os<<")";
  }
}


void TL::CountFunction::writeNice(ostream& os) const {
    Function::writeNice(os);
    os << " <-- ";
    os << "(";
    uint i;
    for(i=0;i<d;i++) {
      if (i == d-1)
        os << i;
      else
        os << i << " ";
    }
    os<<"): ";
    os<<" #"<<countedPred->name<<"(";
    FOR1D(countedPred_mapVars2derived, i) {
        os<<countedPred_mapVars2derived(i)<<" ";
    }
    os<<")";
    if (max_value != -1) {
      os << "   max_value="<< max_value;
    }
}


void TL::AverageFunction::writeNice(ostream& os) const {
  Function::writeNice(os);
  os << " <-- ";
  os << "(";
  uint i;
  for(i=0;i<d;i++) {
    if (i == d-1)
      os << i;
    else
      os << i << " ";
  }
  os<<"): ";
  os<<" AVG("<<f_base->name<<")";
}


void TL::SumFunction::writeNice(ostream& os) const {
  Function::writeNice(os);
  os << " <-- ";
  os << "(";
  uint i;
  for(i=0;i<d;i++) {
    if (i == d-1)
      os << i;
    else
      os << i << " ";
  }
  os<<"): ";
  os<<" SUM("<<f_base->name<<")";
}


void TL::MaxFunction::writeNice(ostream& os) const {
  Function::writeNice(os);
  os << " <-- ";
  os << "(";
  uint i;
  for(i=0;i<d;i++) {
    if (i == d-1)
      os << i;
    else
      os << i << " ";
  }
  os<<"): ";
  os<<" MAX("<<f_base->name<<")";
}

void TL::ChangeFunction::writeNice(ostream& os) const {
  Function::writeNice(os);
  os << " <-- ";
  os << "(";
  uint i;
  for(i=0;i<d;i++) {
    if (i == d-1)
      os << i;
    else
      os << i << " ";
  }
  os<<"): ";
  os<<" CHANGE("<<f_base->name<<")";
}


void TL::RewardFunction::writeNice(ostream& os) const {
  Function::writeNice(os);
  os << " <-- ";
  uint i;
  FOR1D(grounded_pis, i) {
    grounded_pis(i)->writeNice(os); os<<" ";
  }
  os<<"   reward_value="<<reward_value;
  
}


void TL::ConjunctionPredicate::writeNice(ostream& os) const {
	Predicate::writeNice(os);
	os << " <-- ";
    os << "(";
    uint i;
    for(i=0;i<d;i++) {
      if (i == d-1)
        os << i;
      else
        os << i << " ";
    }
    os<<"): ";
    uintA freeVars;
    this->getFreeVars(freeVars);
    if (freeVars.N>0) {
        if (!freeVarsAllQuantified)
            os << "ex[";
        else
            os << "all[";
        FOR1D(freeVars, i) {
          if (freeVars.N-1 == i)
            os<<freeVars(i);
          else
            os<<freeVars(i)<<" ";
        }
        os << "]";
        os <<": ";
    }
	uint j, k;
	k = 0;
	FOR1D(basePreds, i) {
		if (!basePreds_positive(i))
			os << "-";
		os << basePreds(i)->name << "(";
		for (j=0; j<basePreds(i)->d; j++) {
      if (j==basePreds(i)->d-1)
        os << basePreds_mapVars2conjunction(k++);
      else
        os << basePreds_mapVars2conjunction(k++) << " ";
    }
		os << ") ";
	}
}


void TL::TransClosurePredicate::writeNice(ostream& os) const {
    Predicate::writeNice(os);
    os << " <--  +";
    os << basePred->name;
}

void TL::CountPredicate::writeNice(ostream& os) const {
    Predicate::writeNice(os);
    os << " <--  ";
    os << "[";
    uint i;
    for(i=0;i<d;i++) {
      if (i == d-1)
        os << i;
      else
        os << i << " ";
    }
    os<<"]: ";
    if (countedPred->type == TL_PRED_COMPARISON) {
        CHECK(dynamic_cast<TL::ComparisonPredicate*>(countedPred)!=NULL, "cast failed");
        CHECK(((TL::ComparisonPredicate*)(countedPred))->constantBound, "dynamic bound NIY");
        os << "[";
        os << comparison_f->name << "(";
        FOR1D(countedPred_mapVars2derived, i) {
            cout<<countedPred_mapVars2derived(i)<<" ";
        }
        os << ")";
        writeComparison(comparison_compType, os);
        os << comparison_bound;
        os << "]";
    }
    else {
        os << countedPred->name << "(";
        FOR1D(countedPred_mapVars2derived, i) {
            cout<<countedPred_mapVars2derived(i)<<" ";
        }
        os << ")";
    }
    writeComparison(compType, os);
    os << bound;
}


// void TL::ComparisonPredicate::writeNice(ostream& os) const {
// 	Predicate::writeNice(os);
// 	os << ": " << f->name << " ";
// 	switch(comparisonType) {
// 		case TL_LESS: os << "<"; break;
// 		case TL_GREATER: os << ">"; break;
// 		case TL_EQUAL: os << "="; break;
// 	}
// }

void TL::FunctionValue::writeNice(ostream& os) const {
// 	os << "fv ";
  os << f->name << "(";
  uint i;
  FOR1D(args, i) {
    if (i == args.N-1)
      os << args(i);
    else
      os << args(i) << " ";
  }
  os << ")";
	os << "=" << value;
}

// void TL::FunctionValue::write_alt(ostream& os) const {
// // 	os << "fv ";
//   os << f->category << " " << f->id << " " << value;
//   uint i;
//   FOR1D(args, i) {
//     os << " " << args(i);
//   }
// }

void TL::FunctionValue::write(ostream& os) const {
  os << f->id << " " << value;
  uint i;
  FOR1D(args, i) {
    os << " " << args(i);
  }
}

void TL::FunctionInstance::writeNice(ostream& os) const {
// 	os << "fv ";
  os << f->name << "(";
  uint i;
  FOR1D(args, i) {
    if (args(i) == 0) os << "X";
    else if (args(i) == 1) os << "Y";
    else if (args(i) == 2) os << "Z";
    else if (args(i) == 3) os << "V";
    else os << args(i);
    if (i < args.N-1)
      os << " ";
  }
  os << ")";
}


// void TL::FunctionInstance::write_alt(ostream& os) const {
//   os << f->category << " " << f->id;
//   uint i;
//   FOR1D(args, i) {
//     os << " " << args(i);
//   }
// }

void TL::FunctionInstance::write(ostream& os) const {
  os << f->id;
  uint i;
  FOR1D(args, i) {
    os << " " << args(i);
  }
}


void TL::PredicateInstance::writeNice(ostream& os, bool withTypes) const {
// 	os << "pi ";
	if (!positive)
		os << "-";
	os << pred->name << "(";
	uint i;
	FOR1D(args, i) {
    if (args(i) == 0) os << "X";
    else if (args(i) == 1) os << "Y";
    else if (args(i) == 2) os << "Z";
    else if (args(i) == 3) os << "V";
    else os << args(i);
    if (withTypes) {
      if (pred->arg_types.N > 0) {
        os<<"/"<<pred->arg_types(i)->name;
      }
    }
    if (i < args.N-1)
      os << " ";
	}
	os << ")";
}


void TL::ComparisonPredicateInstance::writeNice(ostream& os, bool withTypes) const {
// 	os << "comppt ";
  if (hasConstantBound()) {
      os << f->name << "(";
      uint i;
      FOR1D(args, i) {
        if (args(i) == 0) os << "X";
        else if (args(i) == 1) os << "Y";
        else if (args(i) == 2) os << "Z";
        else if (args(i) == 3) os << "V";
        if (i < args.N-1)
          os << " ";
      }
      os << ")";
      writeComparison(comparisonType, os);
      os << bound;
  }
  else {
      CHECK(args.N%2==0, "Maldefined dynamic cpt!")
      os << f->name << "(";
      uint i;
      for (i=0; i<args.N/2; i++) {
        if (i == args.N/2-1)
          os << args(i);
        else
          os << args(i) << " ";
      }
      os << ")";
      writeComparison(comparisonType, os);
      os << f->name << "(";
      for (i=args.N/2; i<args.N; i++) {
        if (i == args.N-1)
          os << args(i);
        else
          os << args(i) << " ";
      }
      os << ")";
  }
}


// void TL::PredicateInstance::write_alt(ostream& os) const {
//   os << pred->category << " ";
//   os << pred->type << " ";
//   os << pred->id << " ";
//   if (positive)
//     os << "1 ";
//   else
//     os << "0 ";
//   uint i;
//   FOR1D(args, i) {
//     os << args(i) << " ";
//   }
// }

void TL::PredicateInstance::write(ostream& os) const {
//     os << pred->category << " ";
//     os << pred->type << " ";
    os << pred->id << " ";
    if (positive)
        os << "1 ";
    else
        os << "0 ";
    uint i;
    FOR1D(args, i) {
        os << args(i) << " ";
    }
}


// void TL::ComparisonPredicateInstance::write_alt(ostream& os) const {
// //   os << pred->category << " ";
// //   os << pred->type << " ";
//   os << pred->id << " ";
//   if (positive)
//     os << "1 ";
//   else
//     os << "0 ";
//   os << f->id << " ";
//   os << comparisonType << " ";
//   os << hasConstantBound() << " ";
//   if (hasConstantBound()) {
//     os << bound << " ";
//   }
//   uint i;
//   FOR1D(args, i) {
//     os << args(i) << " ";
//   }
// }

void TL::ComparisonPredicateInstance::write(ostream& os) const {
//     os << pred->category << " ";
//     os << pred->type << " ";
    os << pred->id << " ";
    if (positive)
        os << "1 ";
    else
        os << "0 ";
    os << f->id << " ";
    os << comparisonType << " ";
    os << hasConstantBound() << " ";
    if (hasConstantBound()) {
        os << bound << " ";
    }
    uint i;
    FOR1D(args, i) {
        os << args(i) << " ";
    }
}


void TL::Rule::write(ostream& os) const {
  if (action->pred->id == TL_DEFAULT_ACTION_PRED__ID) { // Default rule
    os<<"# Noisy default rule"<<endl;
    os<<"D"<<endl;
    os<<"# Probability of noise outcome"<<endl;
    os<<probs(1)<<endl;
    os<<"# Excepted number of changes in noise outcome"<<endl;
    os<<noise_changes<<endl;
  }
  else {
    uint i, j;
    // Info [START]
    os << "# PRE: ";
    FOR1D(context, i) {
      context(i)->writeNice(os);
  //     os<<context(i);
      os << " ";
    }
    os << endl;
    // Default rule does not have an action specified...
    os << "# ACTION: ";
    if (action != NULL)
      action->writeNice(os);
    else
      os << "default";
    os << endl;
    os << "# POST:" << endl;
    FOR1D(outcomes, i) {
      os << "#  " << probs(i) << " ";
      FOR1D(outcomes(i), j) {
        outcomes(i)(j)->writeNice(os);
  //       os<<outcomes(i)(j);
        os << " ";
      }
      if (i==outcomes.N-1)
        os<<noise_changes;
      if (outcome_rewards.N > 0) {
        if (!TL::isZero(outcome_rewards(i)))
          os<<"  = REWARD "<<outcome_rewards(i);
      }
      os << endl;
    }
    // Info [END]
//     os << "# Rule for action "; action->writeNice(os); os<<endl;
    os << "[ " << endl;
    FOR1D(context, i) {
      context(i)->write(os);
      os << endl;
    }
    os << "]" << endl;
    action->write(os);  os<<endl;
    os << "[ " << endl;
    CHECK(outcomes.N == probs.N, "outcomes.N != probs.N");
    FOR1D(outcomes, i) {
      os << "[ " << endl;
      os << probs(i) << endl;
      if (i==outcomes.N-1) { // noise outcome
        os<<NOISE_MARKER;
        os<<" "<<noise_changes<<endl;
      }
      else if (outcomes(i).N > 0) {
        FOR1D(outcomes(i), j) {
          outcomes(i)(j)->write(os);  os<<" "<<endl;
        }
      }
      os << "] " << endl;
    }
    os << "]" << endl;
  }
}

void TL::Rule::writeNice(ostream& os, bool withAction) const {
//  os << "r" << endl;
  uint i, j;
  // Default rule does not have an action specified...
  if (withAction) {
    os << "ACTION: ";
    if (action != NULL)
      action->writeNice(os, true);
    else
      os << "default";
    os << endl;
  }
  os << "CONTEXT:    "<<endl<<"  ";
  if (context.N == 0)
    os << "-";
  FOR1D(context, i) {
    context(i)->writeNice(os);
//     os<<context(i);
    os << " ";
  }
  os << endl;
  os << "POST:" << endl;
  FOR1D(outcomes, i) {
    os.precision(3);
    os << "  ";
    if (probs(i) < 0.001) os << "0";
    else os << probs(i);
    os << " ";
    FOR1D(outcomes(i), j) {
      outcomes(i)(j)->writeNice(os);
//       os<<outcomes(i)(j);
      os << " ";
    }
    if (i==outcomes.N-1)
//       os<<noise_changes;
      os << "noise";
    if (outcome_rewards.N > 0) {
      if (!TL::isZero(outcome_rewards(i)))
        os<<"  = REWARD "<<outcome_rewards(i);
    }
    os << endl;
  }
}

void TL::Substitution::writeNice(ostream& os) {
// 	os << "s ";
	uint i;
	FOR1D(ins, i) {
		os << ins(i) << "->" << getSubs(ins(i)) << " ";
	}
}


void TL::State::writeNice(ostream& os, bool breaks, bool primOnly) const {
// 	os << "s ";
	uint i;
	FOR1D(pi_prim, i) {
		pi_prim(i)->writeNice(os);
		os << " ";
	}
  if (pi_prim.N>0 && breaks) os << endl;
  FOR1D(fv_prim, i) {
    fv_prim(i)->writeNice(os);
    os << " ";
  }
  if (fv_prim.N>0 && breaks) os << endl;
	FOR1D(pi_comp, i) {
		pi_comp(i)->writeNice(os);
		os << " ";
	}
  if (primOnly)
    return;
  if (pi_comp.N>0 && breaks) os << endl;
  FOR1D(pi_derived, i) {
    if (pi_derived(i)->pred->id == 52)  // HAND_ID__PRED_DIFF_TOWER
      continue;
    pi_derived(i)->writeNice(os);
    os << " ";
  }
  if (pi_derived.N>0 && breaks) os << endl;
  FOR1D(fv_derived, i) {
      fv_derived(i)->writeNice(os);
        os << " ";
  }
}

void TL::State::write(ostream& os) const {
//  os << "s ";
  uint i;
  os << "[" << endl;
  os << "[" << endl;
  FOR1D(pi_prim, i) {
    pi_prim(i)->write(os);
    os<<endl;
  }
  os << "]" << endl;
  os << "[" << endl;
  FOR1D(fv_prim, i) {
    fv_prim(i)->write(os);
    os<<endl;
  }
  os << "]" << endl;
  os << "]" << endl;
}



void TL::Trial::writeNice(ostream& os) const {
  os << "Trial:" << endl;
  os << "constants = " << constants << endl;
  uint i;
  for (i=0; i<states.N-1; i++) {
    cout << "(" << i << ") ";
    states(i)->writeNice(os);
    os << endl;
    actions(i)->writeNice(os);  
    os << endl;
  }
  cout << "(" << i << ") ";
  states(states.N-1)->writeNice(os);
  os << endl;
}

void TL::Trial::write(ostream& os, bool writeContinuousFeatures) const {
  uint i, k, l;
  FOR1D(constants, i) {
    os << constants(i) << " ";
  }
  os << endl;
  os << endl;
//  FOR1D(permanentFValues, i)
//    os << *permanentFValues(i) << " " << endl;
//  os << endl;
  for (i=0; i<states.N-1; i++) {
    os << "# " << i << endl;
    os << *states(i) << endl;
    if (writeContinuousFeatures) {
      os << "{" <<endl;
      // positions
      os << "[" <<endl;
      FOR1D(constants, k) {
        os << constants(k);
        for (l=0; l<positions(i).d1; l++) {
          os << " " <<positions(i)(k,l);
        }
        os << endl;
      }
      os << "]" <<endl;
      // angles
      os << "[" <<endl;
      FOR1D(constants, k) {
        os << constants(k);
        for (l=0; l<angles(i).d1; l++) {
          os << " " <<angles(i)(k,l);
        }
        os << endl;
      }
      os << "]" <<endl;
      os << "}" <<endl;
      os << endl << endl;
    }
    actions(i)->write(os);  os<<endl<<endl;
	}
	os << *states.last() << endl;
  if (writeContinuousFeatures) {
    os << "{" <<endl;
      // positions
    os << "[" <<endl;
    FOR1D(constants, k) {
      os << constants(k);
      for (l=0; l<positions(i).d1; l++) {
        os << " " <<positions(i)(k,l);
      }
      os << endl;
    }
    os << "]" <<endl;
      // angles
    os << "[" <<endl;
    FOR1D(constants, k) {
      os << constants(k);
      for (l=0; l<angles(i).d1; l++) {
        os << " " <<angles(i)(k,l);
      }
      os << endl;
    }
    os << "]" <<endl;
    os << "}" <<endl;
  }
}









// ------------------------------------------------
// ------------------------------------------------
// ------------------------------------------------
//   O P E R A T O R
// ------------------------------------------------
// ------------------------------------------------
// ------------------------------------------------


bool TL::TermType::operator==(const TermType& t) const {
  return t.type_id == this->type_id;
}


bool TL::Function::operator==(const Function& f) const {
    // not via id!!
    if (this->d != f.d  ||  this->type != f.type  ||  this->category != f.category)
        return false;
    return (MT::String) this->name == (MT::String) f.name;
}

bool TL::Predicate::operator==(const Predicate& p) const {
    // not via id!!
    if (this->d != p.d  ||  this->type != p.type  ||  this->category != p.category)
        return false;
    return (MT::String) this->name == (MT::String) p.name;
}


bool TL::PredicateInstance::operator==(TL::PredicateInstance& pi) const {
	if (pred != pi.pred)
		return false;
	if (positive != pi.positive)
		return false;
	if (this->args.N==0 && pi.args.N==0) // special case if no slot assignments
		return true;
	if (args == pi.args)
		return true;
	else
		return false;
}

bool TL::ComparisonPredicateInstance::operator==(TL::PredicateInstance& pi) const {
	TL::ComparisonPredicateInstance* cpt = dynamic_cast<TL::ComparisonPredicateInstance*>(&pi);
	if (cpt == NULL)
		return false;
	return pred == cpt->pred 
					&& positive == cpt->positive
					&& args == cpt->args;
}

bool TL::PredicateInstance::operator!=(TL::PredicateInstance& pi) const {
	return !(*this == pi);
}


bool TL::FunctionValue::operator==(TL::FunctionValue& fv) const {
	return f == fv.f 
					&& TL::isZero(value - fv.value)
					&& args == fv.args;
}

bool TL::FunctionValue::operator!=(TL::FunctionValue& fv) const {
	return !(*this==fv);
}



bool TL::State::operator==(const TL::State& s) const {
	return equivalent(pi_prim, s.pi_prim)
      && equivalent(fv_prim, s.fv_prim);
}

bool TL::State::operator!=(const TL::State& s) const {
	return !(*this == s);
}



std::ostream& operator<<(std::ostream& os, const TL::Predicate& p) {
    p.write(os); return os;
}

std::ostream& operator<<(std::ostream& os, const TL::Function& f) {
    f.write(os); return os;
}

std::ostream& operator<<(std::ostream& os, const TL::State& s) {
  s.write(os); return os;
}

std::ostream& operator<<(std::ostream& os, const TL::Trial& w) {
  w.write(os); return os;
}

std::ostream& operator<<(std::ostream& os, const TL::PredicateInstance& p) {
  p.writeNice(os); return os;
}

std::ostream& operator<<(std::ostream& os, const TL::FunctionValue& f) {
  f.writeNice(os); return os;
}

std::ostream& operator<<(std::ostream& os, const TL::Rule& r) {
  r.write(os); return os;
}



/*
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
	  S U B S T I T U T I O N
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
	
*/
	
uint TL::Rule::globalRuleCounter = 0;

int TL::Substitution::globalCounter_Substitution = 0;

void TL::Substitution::apply(const PredIA& unsubPreds, PredIA& subPreds) {
	subPreds.clear();
	uint i;
	FOR1D(unsubPreds, i) {
		subPreds.append(apply(unsubPreds(i)));
	}
}

void TL::Substitution::apply(const FuncVA& unsubFVs, FuncVA& subFVs) {
  subFVs.clear();
  uint i;
  FOR1D(unsubFVs, i) {
    subFVs.append(apply(unsubFVs(i)));
  }
}

TL::State* TL::Substitution::apply(const TL::State& state) {
  TL::State* s_new = new TL::State;
  apply(state.pi_prim, s_new->pi_prim);
  apply(state.pi_derived, s_new->pi_derived);
  apply(state.fv_prim, s_new->fv_prim);
  apply(state.fv_derived, s_new->fv_derived);
  
  return s_new;
}




TL::PredicateInstance* TL::Substitution::apply(PredicateInstance* inPI) {
//   inPI->writeNice(); cout<<endl;
//   PRINT(ins); PRINT(outs);
  PredicateInstance* outPI = inPI->clone();
	uint i;
  FOR1D(inPI->args, i) {
    outPI->args(i) = getSubs(inPI->args(i));
	}
//   outPI->writeNice(); cout<<endl;
  return outPI;
}

TL::FunctionValue* TL::Substitution::apply(FunctionValue* inFV) {
  FunctionValue* outFV = inFV->clone();
  uint i;
  FOR1D(inFV->args, i) {
    outFV->args(i) = getSubs(inFV->args(i));
  }
  return outFV;
}


bool my_uint_compare(const uint& a, const uint& b) {
  return a <= b;
}

uint TL::Substitution::getSubs(uint in) {
//   cout<<"getSubs [START]"<<endl;
//   PRINT(in);
//   PRINT(ins);
//   PRINT(outs);
  uint i = ins.findInSorted(in, &my_uint_compare);
//   PRINT(i);
  if (i < ins.N) {
    if (in == ins(i)) {
//       cout<<"raus = "<<outs(i)<<endl;
//       cout<<"getSubs [END]"<<endl;
      return outs(i);
    }
  }
//   cout<<"raus = "<< in <<endl;
//   cout<<"getSubs [END]"<<endl;
  return in;
}


bool TL::Substitution::empty() {
    return ins.N == 0;
}

void TL::Substitution::getIns(uintA& ids) const {
	ids.clear();
	uint i;
	FOR1D(ins, i) {
		ids.append(ins(i));
  }
}

void TL::Substitution::getOuts(uintA& ids) const {
	ids.clear();
	uint i;
	FOR1D(outs, i) {
    ids.append(outs(i));
  }
}


void TL::Substitution::addSubs2Variable(uint in) {
	// FIND FREE VARIABLE
	uint out;
	for (out=0;;out++) {
		if (outs.findValue(out) < 0)
			break;
	}
	addSubs(in, out);
}


void TL::Substitution::addSubs(uint in, uint out) {
  uint i = ins.findInSorted(in, &my_uint_compare);
  if (i < ins.N) {
    if (in == ins(i)) { // in already in usage
      outs(i) = out;
    }
    else {  // in is new
      ins.insert(i, in);
      outs.insert(i, out);
    }
  }
  else {
    ins.append(in);
    outs.append(out);
  }
}


bool TL::Substitution::hasSubs(uint in) {
  uint i = ins.findInSorted(in, &my_uint_compare);
  if (i < ins.N) {
    if (in == ins(i))
      return true;
  }
  return false;
}


void TL::Substitution::getInverse(TL::Substitution& invSub) {
  CHECK(invSub.empty(), "substituschn already filled!");
	uint i;
	FOR1D(ins, i) {
    invSub.addSubs(outs(i), ins(i));
	}
}

bool TL::Substitution::mapsToDistinct() {
  return ins.N == outs.N  &&  !outs.containsDoubles();
}


TL::Substitution* TL::Substitution::combine(TL::Substitution& sub1, TL::Substitution& sub2) {
  Substitution* s = new Substitution;
  uint i;
  uintA ids1;
  sub1.getIns(ids1);
  FOR1D(ids1, i) {
      s->addSubs(ids1(i), sub1.getSubs(ids1(i)));
  }
  uintA ids2;
  sub2.getIns(ids2);
  FOR1D(ids2, i) {
      s->addSubs(ids2(i), sub2.getSubs(ids2(i)));
  }
  return s;
}


TL::Substitution& TL::Substitution::operator=(const Substitution& s) {
  this->ins = s.ins;
  this->outs = s.outs;
  return *this;
}





/*
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
	  M I S C
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
	
*/


void TL::sort(PredA& preds) {
  if (preds.N==0) return;
  PredA sortedPreds;
  uint min_id, last_id = 0;
  TL::Predicate* min_pred=NULL;
  uint i, k;
  FOR1D(preds, i) {
    min_id = UINT_MAX;
    FOR1D(preds, k) {
      if (preds(k)->id <= min_id) {
        if (i==0  ||  preds(k)->id > last_id) {
          min_pred = preds(k);
          min_id = min_pred->id;
        }
      }
    }
    sortedPreds.append(min_pred);
    last_id = min_id;
  }
  preds = sortedPreds;
}



void TL::sort(FuncA& funcs) {
  if (funcs.N==0) return;
  FuncA sortedFuncs;
  uint min_id, last_id = 0;
  TL::Function* min_func=NULL;
  uint i, k;
  FOR1D(funcs, i) {
    min_id = UINT_MAX;
    FOR1D(funcs, k) {
      if (funcs(k)->id <= min_id) {
        if (i==0  ||  funcs(k)->id > last_id) {
          min_func = funcs(k);
          min_id = min_func->id;
        }
      }
    }
    sortedFuncs.append(min_func);
    last_id = min_id;
  }
  funcs = sortedFuncs;
}


bool TL::equivalent(const PredIA& p1, const PredIA& p2) {
	if (p1.N != p2.N)
		return false;
	uint i, j;
	// assumption: all predicate tuples in p1 are distinct
	FOR1D(p1, i) {
		FOR1D(p2, j) {
			if (*(p1(i))==*(p2(j)))
				break;
		}
		if (j== p2.N)
			return false;
	}
	return true;
}


bool TL::equivalent(const FuncVA& fv1, const FuncVA& fv2) {
	if (fv1.N != fv2.N)
		return false;
	uint i, j;
	// assumption: all predicate tuples in p1 are distinct
	FOR1D(fv1, i) {
		FOR1D(fv2, j) {
			if (*(fv1(i))==*(fv2(j)))
				break;
		}
		if (j==fv2.N)
			return false;
	}
	return true;
}




void TL::writeComparison(uint comparisonType, std::ostream& os) {
    switch(comparisonType) {
        case TL_COMPARISON_EQUAL: os << "=="; break;
        case TL_COMPARISON_LESS: os << "<"; break;
        case TL_COMPARISON_LESS_EQUAL: os << "<="; break;
        case TL_COMPARISON_GREATER: os << ">"; break;
        case TL_COMPARISON_GREATER_EQUAL: os << ">="; break;
        default: HALT("Unknown comparison type")
    }
}



void TL::baseConcepts(TL::Predicate* p, PredA& p_base, FuncA& f_base) {
    p_base.clear();
    f_base.clear();
    if (p->type == TL_PRED_SIMPLE) {
        return;
    }
    else if (p->type == TL_PRED_COMPARISON) {
        return;
    }
    else if (p->type == TL_PRED_CONJUNCTION) {
        TL::ConjunctionPredicate* scp = dynamic_cast<TL::ConjunctionPredicate*>(p);
        CHECK(scp!=NULL, "cast failed");
        uint i;
        FOR1D(scp->basePreds, i) {
            p_base.setAppend(scp->basePreds(i));
            PredA base_p_base;
            FuncA base_f_base;
            baseConcepts(scp->basePreds(i), base_p_base, base_f_base);
            p_base.setAppend(base_p_base);
            f_base.setAppend(base_f_base);
        }
    }
    else if (p->type == TL_PRED_COUNT) {
        TL::CountPredicate* cp = dynamic_cast<TL::CountPredicate*>(p);
        CHECK(cp!=NULL, "cast failed");
        p_base.setAppend(cp->countedPred);
        PredA base_p_base;
        FuncA base_f_base;
        baseConcepts(cp->countedPred, base_p_base, base_f_base);
        p_base.setAppend(base_p_base);
        f_base.setAppend(base_f_base);
        if (cp->countedPred_isComparison) {
            f_base.setAppend(cp->comparison_f);
            baseConcepts(cp->comparison_f, base_p_base, base_f_base);
            p_base.setAppend(base_p_base);
            f_base.setAppend(base_f_base);
        }
    }
    else if (p->type == TL_PRED_TRANS_CLOSURE) {
        TL::TransClosurePredicate* tcp = dynamic_cast<TL::TransClosurePredicate*>(p);
        CHECK(tcp!=NULL, "cast failed");
        p_base.setAppend(tcp->basePred);
        PredA base_p_base;
        FuncA base_f_base;
        baseConcepts(tcp->basePred, base_p_base, base_f_base);
        p_base.setAppend(base_p_base);
        f_base.setAppend(base_f_base);
    }
    else {
      p->writeNice();
        NIY
    }
}


void TL::baseConcepts(TL::Function* f, PredA& p_base, FuncA& f_base) {
    p_base.clear();
    f_base.clear();
    if (f->type == TL_FUNC_SIMPLE) {
        return;
    }
    else if (f->type == TL_FUNC_COUNT) {
        TL::CountFunction* fc = dynamic_cast<TL::CountFunction*>(f);
        CHECK(fc!=NULL, "cast failed");
        p_base.setAppend(fc->countedPred);
        PredA base_p_base;
        FuncA base_f_base;
        baseConcepts(fc->countedPred, base_p_base, base_f_base);
        p_base.setAppend(base_p_base);
        f_base.setAppend(base_f_base);
        
    }
    else {
        NIY
    }
}





/*
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
	  C L O N E
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
	
*/


TL::ConjunctionPredicate* TL::ConjunctionPredicate::clone() {
    ConjunctionPredicate* p = new ConjunctionPredicate;
    *p = *this;
    return p;
}

TL::TransClosurePredicate* TL::TransClosurePredicate::clone() {
    TransClosurePredicate* p = new TransClosurePredicate;
    *p = *this;
    return p;
}

TL::CountPredicate* TL::CountPredicate::clone() {
    CountPredicate* p = new CountPredicate;
    *p = *this;
    return p;
}




TL::PredicateInstance* TL::PredicateInstance::clone() {
//     cout<<"old: ";this->writeNice(cout);cout<<endl;
    TL::PredicateInstance* pi = new TL::PredicateInstance;
    *pi = *this;
//     cout<<"new: ";pi->writeNice(cout);cout<<endl;
    return pi;
}

TL::ComparisonPredicateInstance* TL::ComparisonPredicateInstance::clone() {
    TL::ComparisonPredicateInstance* pi = new TL::ComparisonPredicateInstance;
    *pi = *this;
//     cout<<"Clone cpt:  in -- ";writeNice();cout<<"    vs.   out -- ";pi->writeNice();cout<<endl;
    return pi;
}

TL::FunctionValue* TL::FunctionValue::clone() {
    TL::FunctionValue* fv = new TL::FunctionValue;
    *fv = *this;
    return fv;
}



TL::State::~State() {
  //  object deletion of concept-instances6 is managed ny the LogicEngine now!
//     uint i;
//     FOR1D(pi_prim, i) {
//         delete pi_prim(i);
//     }
//     FOR1D(pi_derived, i) {
//         delete pi_derived(i);
//     }
//     FOR1D(pi_comp, i) {
//         delete pi_comp(i);
//     }
//     FOR1D(fv_prim, i) {
//         delete fv_prim(i);
//     }
//     FOR1D(fv_derived, i) {
//         delete fv_derived(i);
//     }
}


TL::Trial::~Trial() {
//     uint i;
//     FOR1D(actions, i) {
//         delete actions(i);
//     }
//     FOR1D(states, i) {
//         delete states(i);
//     }
}




// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    E X A M P L E
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

TL::Experience::Experience(TL::State* pre, TL::PredicateInstance* action, TL::State* post) {
  this->pre = pre;
  this->action = action;
  this->post = post;
  

  // only look at p_prim
  changedConstants.clear();
  del.clear();
  add.clear();
  uint i;
  // pre+, post-
  FOR1D(pre->pi_prim, i) {
    if (post->pi_prim.findValue(pre->pi_prim(i)) < 0) {
      del.append(pre->pi_prim(i));
      changedConstants.setAppend(pre->pi_prim(i)->args);
    }
  }
  // pre-, post+
  FOR1D(post->pi_prim, i) {
    if (pre->pi_prim.findValue(post->pi_prim(i)) < 0) {
      add.append(post->pi_prim(i));
      changedConstants.setAppend(post->pi_prim(i)->args);
    }
  }
}


TL::Experience::~Experience() {
}

void TL::Experience::writeNice(ostream& os) {
  os << "PRE: ";
  this->pre->writeNice(os);
  os << endl;
  os << "ACTION: ";
  this->action->writeNice(os);
  os << endl;
//   os << "POST: ";
//   this->post->writeNice(os);
//   os << endl;
  os << "DEL: ";  TL::writeNice(del, os);  os<<endl;
  os << "ADD: ";  TL::writeNice(add, os);  os<<endl;
}

bool TL::Experience::noChange() {
  return changedConstants.N == 0  &&  del.N == 0  &&  add.N == 0;
}





/*
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
	  RuleSet
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
*/

TL::RuleSet::RuleSet() {
    ra.memMove = true;
}
		
TL::RuleSet::~RuleSet() {
    clear();
}

int TL::RuleSet::findValue(TL::Rule* r) {
    return ra.findValue(r);
}

void TL::RuleSet::clear() {
    uint i;
    FOR1D(ra, i) {
      del(ra(i));
    }
    ra.clear();
}
void TL::RuleSet::append(TL::Rule* r) {
  ra.append(TL::getRef(r));
}

uint TL::RuleSet::num() const {
    return ra.N;
}

TL::Rule* TL::RuleSet::elem(uint i) const {
    return ra(i);
}

void TL::RuleSet::remove(uint i) {
    TL::del(ra(i));
    ra.remove(i);
}

void TL::RuleSet::overwrite(uint i, TL::Rule* r) {
    del(ra(i));
    ra(i) = TL::getRef(r);
}

TL::RuleSet& TL::RuleSet::operator=(const RuleSet& rs) {
    this->clear();
    uint i;
    FOR1D(rs.ra, i) {
      this->append(rs.ra(i));
    }
    return *this;
}

void TL::RuleSet::sort() {
  MT::Array< TL::Rule* > ra_sorted;
  uintA action_ids;
  uint r, i;
  FOR1D(this->ra, r) {
    action_ids.setAppend(this->ra(r)->action->pred->id);
  }
  FOR1D(action_ids, i) {
    FOR1D(this->ra, r) {
      if (this->ra(r)->action->pred->id == action_ids(i))
        ra_sorted.append(this->ra(r));
    }
  }
  this->ra = ra_sorted;
}


void TL::RuleSet::sort_using_args() {
  uint max_arity = 0;
  uint r2;
  FOR1D(this->ra, r2) {
    if (this->ra(r2)->action->pred->d > max_arity)
      max_arity = this->ra(r2)->action->pred->d;
  }
  
  if (max_arity <= 2) {
    MT::Array< TL::Rule* > ra_sorted;
    uintA action_ids;
    uint r, i;
    uint id;
    
    FOR1D(this->ra, r) {
      if (this->ra(r)->action->pred->id == TL_DEFAULT_ACTION_PRED__ID)
        continue;
      if (this->ra(r)->action->pred->d == 0)
        id = this->ra(r)->action->pred->id * 100;
      else if (this->ra(r)->action->pred->d == 1)
        id = this->ra(r)->action->pred->id * 100  +  this->ra(r)->action->args(0);
      else if (this->ra(r)->action->pred->d == 2)
        id = this->ra(r)->action->pred->id * 10000  +  100 * this->ra(r)->action->args(0) + this->ra(r)->action->args(1);
      else
        HALT("");
      if (action_ids.findValue(id) >= 0)
        continue;
      FOR1D(action_ids, i) {
        if (action_ids(i) > id) {
          action_ids.insert(i, id);
          break;
        }
      }
      if (action_ids.N == i)
        action_ids.append(id);
    }
    
    // DEFAULT RULE always first rule
    uint DEFAULT_ACTION_ID = TL_DEFAULT_ACTION_PRED__ID * 100;
    action_ids.insert(0, DEFAULT_ACTION_ID);
    
    FOR1D(action_ids, i) {
      FOR1D(this->ra, r) {
        if (this->ra(r)->action->pred->d == 0) {
          if (this->ra(r)->action->pred->id * 100 == action_ids(i))
            ra_sorted.append(this->ra(r));
        }
        else if (this->ra(r)->action->pred->d == 1) {
          if (this->ra(r)->action->pred->id * 100  +  this->ra(r)->action->args(0) == action_ids(i))
            ra_sorted.append(this->ra(r));
        }
        else if (this->ra(r)->action->pred->d == 2) {
          if (this->ra(r)->action->pred->id * 10000  +  100 * this->ra(r)->action->args(0) + this->ra(r)->action->args(1) == action_ids(i))
            ra_sorted.append(this->ra(r));
        }
        else
          HALT("");
      }
    }
    this->ra = ra_sorted;
  }
  // max_arity > 2
  else {
    writeRules(*this, "ground_rules.dat.unsorted_backup");
    
    MT::Array< TL::Rule* > ra_sorted;
    ra_sorted.memMove = true;
    uint r, r2, d;
    FOR1D(this->ra, r) {
      bool inserted = false;
      FOR1D(ra_sorted, r2) {
        // put before action-predicate with higher id
        if (this->ra(r)->action->pred->id > ra_sorted(r2)->action->pred->id) {
          ra_sorted.insert(r2, this->ra(r));
          inserted = true;
        }
        // same action-predicate
        else if (this->ra(r)->action->pred->id == ra_sorted(r2)->action->pred->id) {
          FOR1D(this->ra(r)->action->args, d) {
            if (this->ra(r)->action->args(d) < ra_sorted(r2)->action->args(d)) {
              ra_sorted.insert(r2, this->ra(r));
              inserted = true;
              break;
            }
          }
        }
        if (inserted)
          break;
      }
      if (!inserted)
        ra_sorted.append(this->ra(r));
      
//       FOR1D(ra_sorted, r3) {
//         ra_sorted(r3)->action->writeNice(); cout<<"  ";
//       }
//       cout<<endl;
    }
    
//     FOR1D(ra_sorted, r3) {
//       ra_sorted(r3)->action->writeNice(); cout<<"  ";
//     }
//     cout<<endl;
    
    CHECK(ra.N == ra_sorted.N, "Some strange rule-sorting mistake.");
    this->ra = ra_sorted;
  }
}

void TL::RuleSet::writeNice(ostream& os) {
    uint i;
    os<<"#rules = "<<ra.N<<endl;
    FOR1D(ra, i) {
        os<<"["<<i<<"]"<<endl;
        ra(i)->writeNice(os);
        os<<endl;
    }
}





/*
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
	  SubstitutionSet
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
*/


TL::SubstitutionSet::SubstitutionSet() {
    sa.memMove = true;
}
    
TL::SubstitutionSet::~SubstitutionSet() {
    clear();
}
		
int TL::SubstitutionSet::findValue(TL::Substitution* s) {
    return sa.findValue(s);
}
    
void TL::SubstitutionSet::clear() {
    uint i;
    FOR1D(sa, i) {
      del(sa(i));
    }
    sa.clear();
}
    
void TL::SubstitutionSet::append(TL::Substitution* s) {
    sa.append(TL::getRef(s));
}
        
void TL::SubstitutionSet::append(TL::SubstitutionSet& ss) {
    uint i;
    FOR1D_(ss, i) {
        this->append(ss.elem(i));
    }
}
    
uint TL::SubstitutionSet::num() const {
    return sa.N;
}
    
TL::Substitution* TL::SubstitutionSet::elem(uint i) const {
    return sa(i);
}
    
TL::Substitution* TL::SubstitutionSet::last() const {
    return sa(sa.d0-1);
}
		
void TL::SubstitutionSet::remove(uint i) {
    TL::del(sa(i));
    sa.remove(i);
}
    
void TL::SubstitutionSet::overwrite(uint i, TL::Substitution* s) {
    del(sa(i));
    sa(i) = TL::getRef(s);
}
    
TL::SubstitutionSet& TL::SubstitutionSet::operator=(const SubstitutionSet& ss) {
    this->clear();
    uint i;
    FOR1D(ss.sa, i) {
        this->append(ss.sa(i));
    }
    return *this;
}




/*
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
	  Reading  &  Writing  of  Lists
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
*/


void TL::writeNice(const PredA& pa, ostream& os) {
  uint k;
  FOR1D(pa, k) {
    pa(k)->writeNice(os);os<<endl;
  }
}


void TL::writeNice(const FuncA& fa, ostream& os) {
  uint k;
  FOR1D(fa, k) {
    fa(k)->writeNice(os);os<<endl;
  }
}


void TL::writeNice(const PredIA& predTs, ostream& os) {
  uint k;
  FOR1D(predTs, k) {
    if (predTs(k)!=NULL)
      predTs(k)->writeNice(os);
    else
      os<<"NULL";
    os << " ";
  }
  os<<"    ";
//   FOR1D(predTs, k) {
//     os<<predTs(k);
//     os << " ";
//   }
}

void TL::writeNice(const FuncVA& fvs, ostream& os) {
  uint k;
  FOR1D(fvs, k) {
    fvs(k)->writeNice(os);
    os << " ";
  }
}


void TL::writeNice(const FuncIA& fis, ostream& os) {
  uint k;
  FOR1D(fis, k) {
    fis(k)->writeNice(os);
    os << " ";
  }
}


void TL::writeNice(const MT::Array< PredIA >& outcomes, ostream& os) {
  uint k;
  FOR1D(outcomes, k) {
    os << "(" << k << ") ";
    writeNice(outcomes(k), os);
    os << endl;
  }
}





/*
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
	  Reading  &  Writing  of  Complex objects
	-----------------------------------------------------------
	-----------------------------------------------------------
	-----------------------------------------------------------
*/

TL::Trial* TL::readTrial(ifstream& in, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const PredA& actions, const FuncA& f_prim, const FuncA& f_derived, bool readContinuousFeatures) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"readTrial [START]"<<endl;}
  
  TL::Trial* trial = new TL::Trial;
  
  uint id;
  TL::State* state;
  MT::String line;
  CHECK(in.is_open(), "File couldn't be opened!");
    
	// Ignore description beginning of file.
  while(true) {
    if (MT::skip(in) == '#') {
      MT::skipUntil(in, "\n");
    }
    else {
      break;
    }
  }

	// CONSTANTS
  do {
    in >> id;
    trial->constants.append(id);
  } while (MT::skip(in, " \t") != '\n');
  if (DEBUG>0) PRINT(trial->constants);
  
  MT::skip(in);
		
	// read first state
  // STATE [start]
  state = readState(in, p_prim, p_derived, p_comp, f_prim, f_derived);
  trial->states.append(state);
//   state->writeNice(cout);
  MT::skip(in);
  // Account for potential feature information START
  // if necessary, skip additional feature information
  if (MT::peerNextChar(in) == '{') {
    if (!readContinuousFeatures) {
      MT::skipUntil(in,"}");
      MT::skipLine(in);
      MT::skip(in);
    }
    else { // read additional info
      MT::skipUntil(in,"{");
      MT::skipLine(in); // "{"
      MT::skipLine(in); // "["
      // positions
      arr positions(trial->constants.N, 3);
      int constant;
      uint i;
      FOR1D(trial->constants, i) {
        in >> constant;
        CHECK(constant == (int) trial->constants(i), "positions: wrong constant "<<trial->constants(i)<< " vs. "<<constant);
        in >> positions(i, 0);
        in >> positions(i, 1);
        in >> positions(i, 2);
        MT::skipLine(in);
      }
      trial->positions.append(positions);
      MT::skipUntil(in,"[");
      MT::skipLine(in);
      // angles
      arr angles(trial->constants.N, 2);
      FOR1D(trial->constants, i) {
//         cout<<i<<endl;
        in >> constant;
        CHECK(constant == (int) trial->constants(i), "angles: wrong constant "<<trial->constants(i)<< " vs. "<<constant);
        in >> angles(i, 0);
        in >> angles(i, 1);
        MT::skipLine(in);
      }
//       cout<<"finished stuff"<<endl;
      trial->angles.append(angles);
      MT::skipUntil(in,"}");
      MT::skipLine(in);
      MT::skip(in);
    }
  }
  //   cout<<MT::peerNextChar(in)<<endl;
  // Account for potential feature information END
  // STATE [end]
 
	// read in timesteps
  do {
		// ACTION [start]
    PredicateInstance* action = readAction(in, actions);
    trial->actions.append(action);
    if (DEBUG>0) {action->writeNice(cout);}
		// ACTION [end]
    MT::skip(in);
		
		// STATE [start]
    state = readState(in, p_prim, p_derived, p_comp, f_prim, f_derived);
    trial->states.append(state);
//     PRINT(trial->states.N);
    // Account for potential feature information START
  // if necessary, skip additional feature information
    if (!readContinuousFeatures) {
      if (MT::skip(in) == '{') {
        MT::skipUntil(in,"}");
        MT::skipLine(in);
        MT::skip(in);
      }
    }
    else { // read additional info
      MT::skipUntil(in,"{");
      MT::skipLine(in); // "{"
      MT::skipLine(in); // "["
      // positions
      arr positions(trial->constants.N, 3);
      int constant;
      uint i;
      FOR1D(trial->constants, i) {
        in >> constant;
        CHECK(constant == (int) trial->constants(i), "positions: wrong constant "<<trial->constants(i)<< " vs. "<<constant);
        in >> positions(i, 0);
        in >> positions(i, 1);
        in >> positions(i, 2);
        MT::skipLine(in);
      }
      trial->positions.append(positions);
      MT::skipUntil(in,"[");
      MT::skipLine(in);
      // angles
      arr angles(trial->constants.N, 2);
      FOR1D(trial->constants, i) {
//         cout<<i<<endl;
        in >> constant;
        CHECK(constant == (int) trial->constants(i), "angles: wrong constant "<<trial->constants(i)<< " vs. "<<constant);
        if (isdigit(MT::peerNextChar(in)) || MT::peerNextChar(in) == '-')
          in >> angles(i, 0);
        else {
          CHECK(MT::peerNextChar(in) == 'n', "should be nan");
          cerr<<"skipped a nan"<<endl;
          MT::skipUntil(in, " ");
          angles(i, 0) = 0.;
        }
        if (isdigit(MT::peerNextChar(in)) || MT::peerNextChar(in) == '-')
          in >> angles(i, 1);
        else {
          CHECK(MT::peerNextChar(in) == 'n', "should be nan");
          cerr<<"skipped a nan"<<endl;
          MT::skipLine(in);
          angles(i, 1) = 0.;
        }
        MT::skipLine(in);
      }
//       cout<<"finished stuff"<<endl;
      trial->angles.append(angles);
      MT::skipUntil(in,"}");
      MT::skipLine(in);
//       MT::skip(in);
    }
  //   cout<<MT::peerNextChar(in)<<endl;
    // Account for potential feature information END
		// STATE [end]
  } while (MT::skip(in) != -1);
  
  if (trial->actions.N == trial->states.N) {
    HALT("THIS SHOULD NEVER HAPPEN")
        trial->actions.memMove = true;
    trial->actions.remove(trial->actions.N-1);
  }
  if (DEBUG>0) {cout<<"readTrial [END]"<<endl;}
  return trial;
}






TL::State* TL::readState(ifstream& in, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const FuncA& f_prim, const FuncA& f_derived) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"readState [START]"<<endl;}
  State* s = new State;
  MT::skipUntil(in, "[");
  MT::skipLine(in);MT::skipLine(in); // skip two "["
  // PredicateInstances
  while (MT::peerNextChar(in) != ']') {
      PredicateInstance* pi = readPredicateInstance(in, p_prim, p_derived, p_comp, f_prim, f_derived);
      if (pi==NULL) {
          MT_MSG("Couldn't read predicate tuple.");
          continue;
      }
      if (DEBUG>0) {pi->writeNice(); cout<<endl;}
      switch(pi->pred->type) {
        case TL_PRED_SIMPLE: s->pi_prim.append(pi); break;
        default: HALT("Predicate type unknown")
      }
  }
  MT::skipLine(in);MT::skipLine(in); // skip "]\n["
  // FunctionValues
  while (MT::peerNextChar(in) != ']') {
      FunctionValue* fv = readFunctionValue(in, f_prim, f_derived);
      if (fv==NULL) {
          MT_MSG("Couldn't read function value.");
          continue;
      }
      if (DEBUG>0) {fv->writeNice(); cout<<endl;}
      s->fv_prim.append(fv);
  }
  MT::skipLine(in); MT::skipLine(in); // skip two "]"
  
  if (DEBUG>0) {s->writeNice(cout, true, false);}
  if (DEBUG>0) {cout<<"readState [END]"<<endl;}
  return s;
}




// eats whole line
TL::PredicateInstance* TL::readPredicateInstance(ifstream& in, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const FuncA& f_prim, const FuncA& f_derived) {
  uint DEBUG = 0;
  MT::String line;
  uint id;
  bool positive;
  line.read(in, NULL, "\n");
  if (DEBUG>0) PRINT(line)
  if(line.N() == 0) return NULL;  
//   line >> category;
//   line >> type;
  line >> id;
  line >> positive;
  uint i;
  TL::Predicate* predicate = NULL;
  FOR1D(p_prim, i) {
    if (p_prim(i)->id == id) {
      predicate = p_prim(i);
      break;
    }
  }
  if (predicate == NULL) {
    FOR1D(p_derived, i) {
      if (p_derived(i)->id == id) {
        predicate = p_derived(i);
        break;
      }
    }
    if (predicate == NULL) {
      FOR1D(p_comp, i) {
        if (p_comp(i)->id == id) {
          predicate = p_comp(i);
          break;
        }
      }
    }
  }
  if (predicate == NULL) {
    HALT("Predicate with id=" << id << " has not been found");
  }
  
  if (predicate->type == TL_PRED_COMPARISON) {
    TL::ComparisonPredicateInstance* pi = new TL::ComparisonPredicateInstance;
    pi->positive = positive;
    pi->pred = predicate;
    line >> id; // now function id
    FOR1D(f_prim, i) {
      if (f_prim(i)->id == id) {
        pi->f=f_prim(i);
        break;
      }
    }
    if (f_prim.N == i) {
      FOR1D(f_derived, i) {
        if (f_derived(i)->id == id) {
          pi->f=f_derived(i);
          break;
        }
      }
      CHECK(i<f_derived.N, "Function not found.");
    }
    line >> pi->comparisonType;
    bool hasConstantBound;
    line >> hasConstantBound;
    if (hasConstantBound) {
      line >> pi->bound;
      if (pi->f->name == MT::String("size")) {
        // comment if needed (depending on rule-file format)
        //   pi->bound = REPLACE_SIZE(pi->bound);
      }
    }
    uint arity = pi->f->d;
    if (!hasConstantBound)
        arity *= 2;
    pi->args.resize(arity);
    FOR1D(pi->args, i) {
        line >> pi->args(i);
    }
    
    if (DEBUG>0) {pi->writeNice(cout); cout<<endl;}
    return pi;
  }
  else {
    TL::PredicateInstance* pi = new TL::PredicateInstance;
    pi->positive = positive;
    pi->pred = predicate;
    pi->args.resize(pi->pred->d);
    FOR1D(pi->args, i) {
      line >> pi->args(i);
      if (pi->args(i) == INT_MAX)
        pi->args(i) = UINT_MAX;
    }
    if (DEBUG>0) {pi->writeNice(cout); cout<<endl;}
    return pi;
  }
}


// eats whole line
TL::FunctionValue* TL::readFunctionValue(ifstream& in, const FuncA& f_prim, const FuncA& f_derived) {
    uint DEBUG = 0;
    MT::String line;
    uint id;
    in >> line;  // eats \n
    if (DEBUG>0) PRINT(line)
    if(line.N() == 0) return NULL;
    TL::FunctionValue* fv = new TL::FunctionValue;
    line >> id;
    uint i;
    FOR1D(f_prim, i) {
      if (f_prim(i)->id == id) {
        fv->f=f_prim(i);
        break;
      }
    }
    if (f_prim.N == i) {
      FOR1D(f_derived, i) {
        if (f_derived(i)->id == id) {
          fv->f=f_derived(i);
          break;
        }
      }
      if (i==f_derived.N) {
        HALT("Unknown function while function value reading.");
        return NULL;
      }
    }
    line >> fv->value;
    fv->args.resize(fv->f->d);
    FOR1D(fv->args, i) {
        line >> fv->args(i);
        if (fv->args(i) == INT_MAX)
            fv->args(i) = UINT_MAX;
    }
    if (DEBUG>0) {fv->writeNice(); cout<<endl;}
    return fv;
}


// eats whole line
TL::FunctionInstance* TL::readFunctionInstance(ifstream& in, const FuncA& f_prim, const FuncA& f_derived) {
  uint DEBUG = 0;
  MT::String line;
  uint id;
  in >> line;  // eats \n
  if (DEBUG>0) PRINT(line);
  if(line.N() == 0) return NULL;
  TL::FunctionInstance* fi = new TL::FunctionInstance;
  line >> id;
  uint i;
  FOR1D(f_prim, i) {
    if (f_prim(i)->id == id) {
      fi->f=f_prim(i);
      break;
    }
  }
  if (f_prim.N == i) {
    FOR1D(f_derived, i) {
      if (f_derived(i)->id == id) {
        fi->f=f_derived(i);
        break;
      }
    }
    if (i==f_derived.N) {
      MT_MSG("Unknown function while function instance reading.");
      return NULL;
    }
  }
  fi->args.resize(fi->f->d);
  FOR1D(fi->args, i) {
    line >> fi->args(i);
    if (fi->args(i) == INT_MAX)
      fi->args(i) = UINT_MAX;
  }
  if (DEBUG>0) {fi->writeNice(); cout<<endl;}
  return fi;
}



TL::PredicateInstance* TL::readAction(ifstream& in, const PredA& actions) {
  PredicateInstance* action = new PredicateInstance;
  MT::String line;
//     uint category, type;
  uint id;
  in >> line; // frisst den Zeilenumbruch u.ae.!!
//     line >> category; // never needed here?!
//     line >> type;
  line >> id;
  uint i;
  action->pred = NULL;
  FOR1D(actions, i) {
    if (actions(i)->id == id) {
      action->pred = actions(i);
      break;
    }
  }
  if (action->pred == NULL) {
    HALT("Action predicate with id=" << id << " has not been found.");
  }
  line >> action->positive;
  action->args.resize(action->pred->d);
  FOR1D(action->args, i) {
      line >> action->args(i);
      if (action->args(i) == INT_MAX)
          action->args(i) = UINT_MAX;
  }
  return action;
}


void TL::readRules(const char* filename, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const PredA& actions, const FuncA& f_prim, const FuncA& f_derived, RuleSet& rules) {
  rules.clear();
  ifstream in;
  in.open(filename);
  readRules(in, p_prim, p_derived, p_comp, actions, f_prim, f_derived, rules);
}


void TL::readRules(ifstream& in, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const PredA& actions, const FuncA& f_prim, const FuncA& f_derived, RuleSet& rules) {
  rules.clear();
  CHECK(in.is_open(), "Rule-file can't be opened!");
  bool default_rule_has_been_read = false;
  while (MT::skip(in) != -1) {
    // check whether first is Noisy Default Rule
    if (!default_rule_has_been_read) {
      if (MT::peerNextChar(in) == 'D') {
        MT::skipLine(in);
        MT::skip(in);
        double prob_noise;
        double changes;
        in >> prob_noise;
        MT::skip(in);
        in >> changes;
        
        TL::Rule* r_default = new TL::Rule;
        uint i;
        TL::Predicate* p_DEFAULT_ACTION = NULL;
        FOR1D(actions, i) {
          if (actions(i)-> id == TL_DEFAULT_ACTION_PRED__ID) {
            p_DEFAULT_ACTION = actions(i);
            break;
          }
        }
        CHECK(p_DEFAULT_ACTION != NULL, "default action predicate has not been found");
        r_default->action = new TL::PredicateInstance;
        r_default->action->positive = true;
        r_default->action->pred = p_DEFAULT_ACTION;
        PredIA sameOutcome;
        r_default->outcomes.append(sameOutcome);
        r_default->probs.append(1-prob_noise);
        PredIA noiseOutcome;
        r_default->outcomes.append(noiseOutcome);
        r_default->probs.append(prob_noise);
        r_default->noise_changes = changes;
        
//         r_default->writeNice();
        
        rules.append(r_default);
        default_rule_has_been_read = true;
        continue;
      }
    }
    rules.append(readRule(in, p_prim, p_derived, p_comp, actions, f_prim, f_derived));
//     if (rules.elem(rules.num()-1)->action->pred->name(0) == 'g') {
//       rules.remove(rules.num()-1);
// //       cout<<"weg";
//     }
  }
}


TL::Rule* TL::readRule(ifstream& in, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const PredA& actions, const FuncA& f_prim, const FuncA& f_derived) {
    uint DEBUG = 0;
    CHECK(in.is_open(), "Input stream ain't open!");
    TL::Rule* r = new TL::Rule;
    MT::String line;
    char c;
    c = MT::skip(in);
    CHECK(c == '[', "Wrong rule format");
    MT::skipLine(in);
    c = MT::peerNextChar(in);
	// PRECONDITIONS
    while (c != ']') {
      TL::PredicateInstance* pi = readPredicateInstance(in, p_prim, p_derived, p_comp, f_prim, f_derived);
      r->context.append(pi);
      if (DEBUG>0) {pi->write(cout);  cout<<endl;}
      c = MT::skip(in);
    }
    MT::skipLine(in);
	// ACTION [start]
    r->action = readAction(in, actions);
    if (DEBUG>0) {r->action->write(cout);  cout<<endl;}
	// ACTION [end]
				
	// OUTCOMES
    c = MT::peerNextChar(in);
    if (DEBUG>0) PRINT(c);
    MT::skipLine(in); // skip opening "[" for all outcomes
    c = MT::peerNextChar(in);
    if (DEBUG>0) PRINT(c);
    MT::Array< PredIA* > outcomes;
    double prob;
	// read in single outcomes
    while (c != ']') {
        MT::skipLine(in); // skips "["
        in >> line;
        if (DEBUG>0) PRINT(line);
        line >> prob;
        r->probs.append(prob);
        PredIA new_outcome;
        c = MT::peerNextChar(in);
        if (DEBUG>0) PRINT(c);
		// read in a single predicates
        while (c != ']') {
          if (c == NOISE_MARKER) {
            in >> line;
            line >> c; // NOISE_MARKER
            line >> r->noise_changes;
          }
          else {
            TL::PredicateInstance* pi = readPredicateInstance(in, p_prim, p_derived, p_comp, f_prim, f_derived);
            new_outcome.append(pi);
            if (DEBUG>0) {pi->write(cout);  cout<<endl;}
          }
          MT::skip(in);
          c = MT::peerNextChar(in);
        }
        r->outcomes.append(new_outcome);
        MT::skipLine(in); // skips "]"
        c = MT::peerNextChar(in);
    }
    MT::skipLine(in);
    if (DEBUG>0) r->writeNice(cout);
    if (fabs(sum(r->probs) - 1.0) > 0.001) {
      r->writeNice(cout);
      printf("%10.10f\n", sum(r->probs));
      HALT("bad probs for rule");
    }
    return r;
}



// LANGUAGE

void TL::writeLanguage(const PredA& preds, const FuncA& funcs, const PredA& actions, TermTypeA& types, const char* filename) {
  ofstream out;
  out.open(filename);
  writeLanguage(preds, funcs, actions, types, out);
  out.close();
}

void TL::writeLanguage(const PredA& preds, const FuncA& funcs, const PredA& actions, TermTypeA& types, ostream& out) {
  uint i;
  out<<"["<<endl;
  FOR1D(preds, i) {
    out<< "#  ";  preds(i)->writeNice(out);  out << endl;
    out << (*preds(i)) << endl;
  }
  out<<"]"<<endl;
  out<<endl;
  out<<"["<<endl;
  FOR1D(funcs, i) {
    out << "#  ";  funcs(i)->writeNice(out);  out << endl;
    out << (*funcs(i)) << endl;
  }
  out<<"]"<<endl;
  out<<endl;
  out<<"["<<endl;
  FOR1D(actions, i) {
    out << "#  ";  actions(i)->writeNice(out);  out << endl;
    out << (*actions(i)) << endl;
  }
  out<<"]"<<endl;
  
  if (types.N > 0) {
    out<<endl;
    out<<"["<<endl;
    FOR1D(types, i) {
      types(i)->write(out);  out << endl;
    }
    out<<"]"<<endl;
  }
}



TL::Predicate* TL::readPredicate(ifstream& in, uintA& baseIds) {
    uint DEBUG = 0;
    MT::String line;
    line.read(in, NULL, "\n");
    if (DEBUG>0) PRINT(line);
    if(line.N() == 0) return NULL;
    
    uint id, d, type, category;
    MT::String name;
    
    line >> id;
    name.read(line, NULL, " ");
    line >> d;
    line >> type;
    line >> category;
    
    uint i;
    if (type == TL_PRED_ACTION) {
        TL::Predicate* p = new TL::Predicate;
        p->id = id;
        p->name = name;
        p->d = d;
        p->type = type;
        p->arg_types.resize(p->d);
        FOR1D(p->arg_types, i) {
          TermType* t = new TermType;
          line >> t->type_id;
          p->arg_types(i) = t;
        }
        return p;
    }
    else if (type == TL_PRED_SIMPLE) {
        TL::Predicate* p = new TL::Predicate;
        p->id = id;
        p->name = name;
        p->d = d;
        p->type = type;
        return p;
    }
    else if (type == TL_PRED_COMPARISON) {
        TL::ComparisonPredicate* p = new TL::ComparisonPredicate;
        p->id = id;
        p->name = name;
        p->d = d;
        p->type = type;
        line >> p->constantBound;
        return p;
    }
    else if (type == TL_PRED_CONJUNCTION) {
        TL::ConjunctionPredicate* p = new TL::ConjunctionPredicate;
        p->id = id;
        p->name = name;
        p->d = d;
        char bracket_left_dummy;
        line >> bracket_left_dummy;
//         PRINT(bracket_left_dummy);
        uint no_basePreds;
        line >> no_basePreds;
        line >> p->freeVarsAllQuantified;
        line >> bracket_left_dummy;
//         PRINT(bracket_left_dummy);
        for(i=0; i<no_basePreds; i++) {
            line >> id;
            baseIds.append(id);
        }
        char bracket_right_dummy;
        line >> bracket_right_dummy;
//         PRINT(bracket_right_dummy);
        line >> p->basePreds_positive;
        line >> p->basePreds_mapVars2conjunction;
        if (DEBUG>1) {
            PRINT(no_basePreds);
            PRINT(p->freeVarsAllQuantified);
            PRINT(baseIds);
            PRINT(p->basePreds_positive);
            PRINT(p->basePreds_mapVars2conjunction);
        }
        return p;
    }
    else if (type == TL_PRED_TRANS_CLOSURE) {
        TL::TransClosurePredicate* p = new TL::TransClosurePredicate;
        p->id = id;
        p->name = name;
        p->d = d;
        char bracket_left_dummy;
        line >> bracket_left_dummy;
        line >> id;
        baseIds.append(id);
        char bracket_right_dummy;
        line >> bracket_right_dummy;
        return p;
    }
    else if (type == TL_PRED_COUNT) {
        TL::CountPredicate* p = new TL::CountPredicate;
        p->id = id;
        p->name = name;
        p->d = d;
        MT::String trash;
        trash.read(line, " "); // "["
        line >> id;
        baseIds.append(id);
        line >> p->countedPred_mapVars2derived;
        line >> p->bound;
        line >> p->compType;
        line >> p->countedPred_isComparison;
        if (p->countedPred_isComparison) {
            line >> id;
            baseIds.append(id);
            line >> p->comparison_bound;
            line >> p->comparison_compType;
        }
        return p;
    }
    else {
        NIY;
        return NULL;
    }
}


TL::Function* TL::readFunction(ifstream& in, uintA& baseIds) {
  uint DEBUG = 0;
  MT::String line;
  line.read(in, NULL, "\n");
  if (DEBUG>0) PRINT(line);
  if(line.N() == 0) return NULL;
    
    uint id, d, type, category;
    MT::String name;
    uintA range;
    
    line >> id;
    name.read(line, NULL, " ");
    line >> d;
    line >> type;
    line >> category;
    line >> range;
    
    if (type == TL_FUNC_SIMPLE) {
        TL::Function* f = new TL::Function;
        f->id = id;
        f->name = name;
        f->d = d;
        f->range = range;
        if (f->range.N ==0) { // if no range specified, set it to [1,2,3,4,5]
          uint h;
          for (h=1; h<=5; h++)
            f->range.append(h);
        }
        return f;
    }
    else if (type == TL_FUNC_COUNT) {
        TL::CountFunction* f = new TL::CountFunction;
        f->id = id;
        f->name = name;
        f->d = d;
        f->range = range;
        if (f->range.N ==0) { // if no range specified, set it to [0,...,9]
          uint h;
          for (h=0; h<=9; h++)
            f->range.append(h);
        }
        line >> id;
        baseIds.append(id);
        line >> f->countedPred_mapVars2derived;
        return f;
    }
    else {
        NIY;
        return NULL;
    }
}


TL::TermType* TL::readTermType(ifstream& in, const TermTypeA& existing_types) {
  uint DEBUG = 0;
  MT::String line;
  line.read(in, NULL, "\n");
  if (DEBUG>0) PRINT(line);
  if(line.N() == 0) return NULL;
 
  uint type_id, typeI;
  MT::String name;
  line >> type_id;
  name.read(line, NULL, " ");
  line >> typeI;
  
  if (typeI == TL_TERM_TYPE_SIMPLE) {
    TL::TermType* t = new TL::TermType;
    t->type_id = type_id;
    t->name = name;
    if (DEBUG>0) {t->writeNice(); cout<<endl;}
    return t;
  }
  else if (typeI == TL_TERM_TYPE_DISJUNCTION) {
    uintA base_types__ids;
    line >> base_types__ids;
    
    TermTypeA base_types;
    uint i, k;
    FOR1D(base_types__ids, i) {
      FOR1D(existing_types, k) {
        if (existing_types(k)->type_id == base_types__ids(i)) {
          base_types.append(existing_types(k));
          break;
        }
      }
      CHECK(i < base_types__ids.N, "base type has not been found");
    }
    
    TL::DisjunctionTermType* t = new TL::DisjunctionTermType;
    t->type_id = type_id;
    t->name = name;
    t->base_types = base_types;
    if (DEBUG>0) {t->writeNice(); cout<<endl;}
    return t;
  }
  else
    NIY;
}



void TL::readLanguage(const char *filename, PredA& p_prim, PredA& p_derived, PredA& p_comp, PredA& actions, FuncA& f_prim, FuncA& f_derived, TermTypeA& types) {
    ifstream in;
    in.open(filename);
    CHECK(in.is_open(), "File can't be opened!");
    readLanguage(in, p_prim, p_derived, p_comp, actions, f_prim, f_derived, types);
}


void TL::readLanguage(ifstream& in, PredA& p_prim, PredA& p_derived, PredA& p_comp, PredA& actions, FuncA& f_prim, FuncA& f_derived, TermTypeA& types) {
  uint DEBUG = 0;
  p_prim.clear();
  p_derived.clear();
  p_comp.clear();
  actions.clear();
  f_prim.clear();
  f_derived.clear();
  types.clear();
  
  // READING
  std::map<uint, uintA> f_id2baseIds;
  std::map<uint, uintA> p_id2baseIds;
  
  // Ignore description beginning of file.
  MT::skipUntil(in, "[");
  MT::skipLine(in); // skip "["
  // (1) Predicates
  while (MT::peerNextChar(in) != ']') {
//       PRINT(MT::peerNextChar(in));
    uintA baseIds;
    TL::Predicate* p = readPredicate(in, baseIds);
    p_id2baseIds[p->id] = baseIds;
    CHECK(p!=NULL, "wrong input format");
    if (DEBUG>0) {
      cout<<"Read predicate "<<p->name<<"  baseIds="<<baseIds<<endl;
    }
    if (p->type == TL_PRED_COMPARISON)
      p_comp.append(p);
    else if (p->category == TL_PRIMITIVE)
      p_prim.append(p);
    else
      p_derived.append(p);
  }
  // (2) Functions
  MT::skipUntil(in, "[");  // skip "]\n["
  MT::skipLine(in);
  while (MT::peerNextChar(in) != ']') {
    uintA baseIds;
    TL::Function* f = readFunction(in, baseIds);
    f_id2baseIds[f->id] = baseIds;
    CHECK(f!=NULL, "wrong input format");
    if (f->category == TL_PRIMITIVE)
      f_prim.append(f);
    else
      f_derived.append(f);
  }
  // (3) Actions
  MT::skipUntil(in, "[");  // skip "]\n["
  MT::skipLine(in);
  while (MT::peerNextChar(in) != ']') {
    uintA baseIds;
    TL::Predicate* p = readPredicate(in, baseIds);
    actions.append(p);
  }
  MT::skipLine(in); // skip "]"
  // (4) Types
  if (MT::skip(in) == '[') {
    // build default type
    TL::TermType* default_t = new TermType;
    default_t->type_id = TL::TermType::ANY_id;
    default_t->name = "any";
    types.append(default_t);
    // read in other types
    MT::skipLine(in);
    while (MT::peerNextChar(in) != ']') {
      TL::TermType* t = readTermType(in, types);
      if (t->type_id == TermType::ANY_id) {
        HALT("ANY type should not be specified in file");
      }
      types.append(t);
//         t->writeNice(); cout<<endl;
    }
    // set correct TermType instances for Actions
    uint i, k, l;
    FOR1D(actions, i) {
      FOR1D(actions(i)->arg_types, k) {
        FOR1D(types, l) {
          if (types(l)->type_id == actions(i)->arg_types(k)->type_id) {
            delete actions(i)->arg_types(k);
            actions(i)->arg_types(k) = types(l);
            break;
          }
        }
        if (l == types.N) {
          HALT("Could not find correct type for action with id="<<actions(i)->id);
        }
      }
    }
  }
  else {
    // build default type
    TL::TermType* default_t = new TermType;
    default_t->type_id = TL::TermType::ANY_id;
    default_t->name = "any";
    types.append(default_t);
    
    // set correct TermType instances for Actions
    uint i, k;
    FOR1D(actions, i) {
      FOR1D(actions(i)->arg_types, k) {
        actions(i)->arg_types(k) = default_t;
      }
    }
  }
  
  
  // POSTPROCESSING
  // postprocessing for base predicates
  uint i, k=400, l;
  FOR1D(p_derived, i) {
    if (DEBUG>0) {cout<<"Postproc. of pred. "<<p_derived(i)->name<<" ...   "<<std::flush;}
    if (p_derived(i)->type == TL_PRED_CONJUNCTION) {
      TL::ConjunctionPredicate* p = dynamic_cast<TL::ConjunctionPredicate*>(p_derived(i));
      CHECK(p!=NULL, "cast failed");
      FOR1D(p_id2baseIds[p->id], k) {
        FOR1D(p_prim, l) {
          if (p_prim(l)->id == p_id2baseIds[p->id](k)) {
            p->basePreds.append(p_prim(l));
            break;
          }
        }
        FOR1D(p_derived, l) {
          if (p_derived(l)->id == p_id2baseIds[p->id](k)) {
            p->basePreds.append(p_derived(l));
            break;
          }
        }
      }
      if (DEBUG>0) {cout<<"done: "; p->writeNice(cout); cout<<endl;}
    }
    else if (p_derived(i)->type == TL_PRED_TRANS_CLOSURE) {
      TL::TransClosurePredicate* p = dynamic_cast<TL::TransClosurePredicate*>(p_derived(i));
      CHECK(p!=NULL, "cast failed");
      CHECK(p_id2baseIds[p->id].N == 1, "invalid trans closure predicate");
      p->basePred = NULL;
      FOR1D(p_prim, l) {
        if (p_prim(l)->id == p_id2baseIds[p->id](0)) {
          p->basePred = p_prim(l);
          break;
        }
      }
      if (p->basePred == NULL) {
        FOR1D(p_derived, l) {
          if (p_derived(l)->id == p_id2baseIds[p->id](0)) {
            p->basePred = p_derived(l);
            break;
          }
        }
      }
      if (DEBUG>0) {cout<<"done: "; p->writeNice(cout); cout<<endl;}
    }
    else if (p_derived(i)->type == TL_PRED_COUNT) {
      TL::CountPredicate* p = dynamic_cast<TL::CountPredicate*>(p_derived(i));
      CHECK(p!=NULL, "cast failed");
      CHECK(p_id2baseIds[p->id].N > 0, "invalid count predicate");
      p->countedPred = NULL;
      FOR1D(p_prim, l) {
        if (p_prim(l)->id == p_id2baseIds[p->id](k)) {
          p->countedPred = p_prim(l);
          break;
        }
      }
      if (p->countedPred== NULL) {
        FOR1D(p_derived, l) {
          if (p_derived(l)->id == p_id2baseIds[p->id](k)) {
            p->countedPred = p_derived(l);
            break;
          }
        }
      }
      if (p->countedPred_isComparison) {
        CHECK(p_id2baseIds[p->id].N == 2, "invalid count predicate with comparison");
        p->comparison_f = NULL;
        FOR1D(f_prim, l) {
          if (f_prim(l)->id == p_id2baseIds[p->id](1)) {
            p->comparison_f = f_prim(l);
            break;
          }
        }
        if (p->comparison_f == NULL) {
          FOR1D(f_derived, l) {
            if (f_derived(l)->id == p_id2baseIds[p->id](1)) {
              p->comparison_f = f_derived(l);
              break;
            }
          }
        }
      }
      if (DEBUG>0) {cout<<"done: "; p->writeNice(cout); cout<<endl;}
    }
  }
  
  FOR1D(f_derived, i) {
    if (DEBUG>0) {cout<<"Postproc. of function "<<p_derived(i)->name<<" ...   "<<std::flush;}
    if (f_derived(i)->type == TL_FUNC_COUNT) {
      TL::CountFunction* f = dynamic_cast<TL::CountFunction*>(f_derived(i));
      CHECK(f!=NULL, "cast failed");
      CHECK(f_id2baseIds[f->id].N == 1, "invalid count function");
      f->countedPred = NULL;
      FOR1D(p_prim, l) {
        if (p_prim(l)->id == f_id2baseIds[f->id](0)) {
          f->countedPred = p_prim(l);
          break;
        }
      }
      if (f->countedPred == NULL) {
        FOR1D(p_derived, l) {
          if (p_derived(l)->id == f_id2baseIds[f->id](0)) {
            f->countedPred = p_derived(l);
            break;
          }
        }
      }
      if (DEBUG>0) {cout<<"done: "; f->writeNice(cout); cout<<endl;}
    }
  }
  
  // SORTING
  sort(p_prim);
  sort(p_derived);
  sort(p_comp);
  sort(f_prim);
  sort(f_derived);
}







void TL::readLanguageSimpleFormat(const char* filename, PredA& p_prim, PredA& p_derived, PredA& p_comp, PredA& actions, FuncA& f_prim, FuncA& f_derived, TermTypeA& types) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"readLanguageSimpleFormat [START]"<<endl;}
  MT_MSG("TermTypeA types IGNORED THUS FAR!!");
  
  ifstream in;
  in.open(filename);
  CHECK(in.is_open(), "File can't be opened!");

  p_prim.clear();
  p_derived.clear();
  p_comp.clear();
  actions.clear();
  f_prim.clear();
  f_derived.clear();

  // READING
  // Ignore description beginning of file.
  MT::skipUntil(in, "[");
  MT::skipLine(in); // skip "["
  // -----------------------------------
  // Predicates
  uint pred_id = 11;
  while (MT::peerNextChar(in) != ']') {
    MT::skip(in);
//     if (MT::peerNextChar(in) == '#')
//       MT::skipLine(in);
//     MT::skip(in);
    if (MT::peerNextChar(in) != '*') {
      TL::Predicate* p = new Predicate;
      MT::String line;
      line.read(in, NULL, "\n");
      p->name.read(line, NULL, " ");
      line >> p->d;
      p->id = pred_id++;
      p_prim.append(p);
      if (DEBUG>0) {p->writeNice(cout); cout<<endl;}
    }
    else {  // * human-rule1-helper() 0 human-onboard() human-alive()
      MT::skipUntil(in, " ");
      TL::ConjunctionPredicate* p = new ConjunctionPredicate;
      MT::String line;
      line.read(in, NULL, "\n");
      p->name.read(line, NULL, " ");
      line >> p->d;
      // Very simple conjunction predicate: conjunction of positive primitives of 0-arity
      if (p->d == 0) {
        MT_MSG("Reading derived predicate: attention -- severe restrictions (only conjunction of positive primitives of 0-arity)");
        while (MT::skip(line) != -1) {
          MT::String name_base_p;
          name_base_p.read(line, NULL, " ");
  //         PRINT(name_base_p);
          uint q;
          FOR1D(p_prim, q) {
  //           PRINT(p_prim(q)->name);
            if (p_prim(q)->name == name_base_p) {
              p->basePreds.append(p_prim(q));
              p->basePreds_positive.append(true);
              break;
            }
          }
  //         line >> ")";
        }
        p->id = pred_id++;
        p_derived.append(p);
      }
      // Conjunction predicate: conjunction of positive potentially existentially quantified primitives
      else {
//         PRINT(MT::peerNextChar(line));
        if (MT::peerNextChar(line) == '%') {
          line >> "%";
          MT::skip(line);
//           PRINT(MT::peerNextChar(line));
          if (MT::peerNextChar(line) == 'e') {
            p->freeVarsAllQuantified = 0;
            line >> "e";
          }
          else if (MT::peerNextChar(line) == 'a') {
            p->freeVarsAllQuantified = 1;
            line >> "e";
          }
          else
            HALT("");
          while (MT::skip(line,"\n\r\t ,") != -1) {
            // Sub-predicate name
            MT::String name_base_p;
            name_base_p.read(line, NULL, "(");
//             PRINT(name_base_p);
            uint q;
            FOR1D(p_prim, q) {
              if (p_prim(q)->name == name_base_p) {
//                 PRINT(p_prim(q)->name);
                p->basePreds.append(p_prim(q));
                p->basePreds_positive.append(true);
                break;
              }
            }
  //         line >> ")";
            // Sub-predicate args
            while (MT::peerNextChar(line) != ')') {
              uint arg;
              line >> arg;
//               PRINT(arg);
//               PRINT(MT::peerNextChar(line));
              MT::skip(line, ",");
//               PRINT(MT::peerNextChar(line));
              p->basePreds_mapVars2conjunction.append(arg);
            }
            MT::skip(line, ")");
//             PRINT(MT::peerNextChar(line));
          }
          p->id = pred_id++;
          p_derived.append(p);
        }
      }
      if (DEBUG>0) {p->writeNice(cout); cout<<endl;}
    }
  }
  MT::skipUntil(in, "[");  // skip "]\n["
  MT::skipLine(in);
  // -----------------------------------
  // Functions
  uint func_id = 11;
  while (MT::peerNextChar(in) != ']') {
    MT::String line;
    line.read(in, NULL, "\n");
    if (MT::peerNextChar(line) == '*') {
      MT::skip(line, "* ");
      MT::String name_f;
      name_f.read(line, NULL, " ");
//       PRINT(name_f);
      uint arity, type;
      line >> arity;
      MT::skip(line);
      line >> type;
      MT::skip(line);
//       PRINT(arity);
//       PRINT(type);
      if (type == TL_FUNC_COUNT) {
        TL::CountFunction* cf = new TL::CountFunction;
        cf->d = arity;
        cf->name = name_f;
        CHECK(arity == 0, "");
        cf->countedPred_mapVars2derived.append(0);
        MT::String name_base_p;
        name_base_p.read(line, NULL, " ");
//         PRINT(name_base_p);
        uint q;
        FOR1D(p_prim, q) {
          if (p_prim(q)->name == name_base_p) {
//             PRINT(p_prim(q)->name);
            cf->countedPred = p_prim(q);
            break;
          }
        }
        if (cf->countedPred == NULL) {
          FOR1D(p_derived, q) {
            if (p_derived(q)->name == name_base_p) {
//               PRINT(p_derived(q)->name);
              cf->countedPred = p_derived(q);
              break;
            }
          }
        }
        cf->id = func_id++;
        if (DEBUG>0) {cf->writeNice(cout); cout<<endl;}
        f_derived.append(cf);
      }
      else
        HALT("");
    }
    // Funktionen machen mer net.
  }
  MT::skipUntil(in, "[");  // skip "]\n["
  MT::skipLine(in);
  // -----------------------------------
  // Actions
  uint action_id = 1;
  bool action_rewards = false;
  if (MT::peerNextChar(in) == 'R') {
    action_rewards = true;
    MT::skipLine(in);
  }
  while (MT::peerNextChar(in) != ']') {
    TL::Predicate* p = new Predicate;
    MT::String line;
    line.read(in, NULL, "\n");
    p->name.read(line, NULL, " ");
    line >> p->d;
//     if (action_rewards)
//       line >> p->reward;
    p->type = TL_PRED_ACTION;
    p->id = action_id++;
    actions.append(p);
    if (DEBUG>0) {p->writeNice(cout); cout<<endl;}
  }
  MT::skipLine(in); // skip "]"

  // SORTING
  sort(p_prim);
  sort(p_derived);
  sort(p_comp);
  sort(f_prim);
  sort(f_derived);
  if (DEBUG>0) {cout<<"readLanguageSimpleFormat [END]"<<endl;}
}






void TL::writeRules(const RuleSet& rules, const char* filename) {
  ofstream out;
  out.open(filename);
  CHECK(out.is_open(), "Can't write to simulation sequence file.");
  uint i;
  for (i=0; i<rules.num(); i++) {
    out << "# Rule " << i << "/" << rules.num()<<endl;
    out << *rules.elem(i) << endl;
  }
  out.close();
}




