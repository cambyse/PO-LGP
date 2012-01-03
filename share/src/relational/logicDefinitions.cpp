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

#include "logicDefinitions.h"

#define NOISE_MARKER 'N'

// TL::LiteralReward::LiteralReward(TL::Literal* _pt) : Reward(REWARD_TYPE__PREDICATE_INSTANCE) {
//   CHECK(_pt->positive, "");
//   lit = _pt;
// }


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



void TL::Atom::name(MT::String& name) const {
  name.clear();
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

void TL::Literal::name(MT::String& name) const {
  name.clear();
  if (!positive) name<<"-";
  name<<*this->atom;
}


void TL::ConjunctionPredicate::getFreeVars(uintA& freeVars) const {
  freeVars.clear();
  uint i;
  FOR1D(basePreds_mapVars2conjunction, i) {
    if (basePreds_mapVars2conjunction(i)>=d)
      freeVars.setAppend(basePreds_mapVars2conjunction(i));
  }
}


void TL::Rule::involvedConcepts(PredL& preds, FuncL& funcs) {
    uint i, k;
    FOR1D(this->context, i) {
        preds.setAppend(this->context(i)->atom->pred);
        if (this->context(i)->atom->pred->type == TL::Predicate::predicate_comparison) {
            TL::ComparisonAtom* ca = dynamic_cast<TL::ComparisonAtom*>(this->context(i)->atom);
            CHECK(ca!=NULL,"cast failed");
            funcs.setAppend(ca->fa1->f);
            PredL p_pre;
            FuncL f_pre;
            baseConcepts(ca->fa1->f, p_pre, f_pre);
            preds.setAppend(p_pre);
            funcs.setAppend(f_pre);
        }
        else {
            PredL p_pre;
            FuncL f_pre;
            baseConcepts(this->context(i)->atom->pred, p_pre, f_pre);
            preds.setAppend(p_pre);
            funcs.setAppend(f_pre);
        }
    }
    FOR1D(this->outcomes, i) {
      FOR1D(this->outcomes(i), k) {
        preds.setAppend(this->outcomes(i)(k)->atom->pred);
        if (this->outcomes(i)(k)->atom->pred->type == TL::Predicate::predicate_comparison) {
          TL::ComparisonAtom* ca = dynamic_cast<TL::ComparisonAtom*>(this->outcomes(i)(k));
          CHECK(ca!=NULL,"cast failed");
          funcs.setAppend(ca->fa1->f);
          PredL p_pre;
          FuncL f_pre;
          baseConcepts(ca->fa1->f, p_pre, f_pre);
          preds.setAppend(p_pre);
          funcs.setAppend(f_pre);
        }
        else {
          PredL p_pre;
          FuncL f_pre;
          baseConcepts(this->outcomes(i)(k)->atom->pred, p_pre, f_pre);
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


void TL::TermType::write_deprecated(ostream& os) const {
  os << type_id << " " << name << " " << typeI;
}

void TL::DisjunctionTermType::write_deprecated(ostream& os) const {
  TermType::write_deprecated(os);
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
    grounded_pis(i)->write_deprecated(os);
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
        case category_primitive: os << " primitive "; break;
        case category_derived: os << " f_derived "; break;
    }
}

void TL::Predicate::writeNice(ostream& os) const {
	os << "p " << id << " " << name << " /" << d;
  switch(category) {
      case category_primitive: os << " primitive "; break;
      case category_derived: os << " p_derived "; break;
  }
	switch(type) {
        case predicate_action: os << " action"; break;
        case TL::Predicate::predicate_simple: os << " simple"; break;
        case TL::Predicate::predicate_comparison: os << " comparison "; break;
        case predicate_conjunction: os << " conjunction "; break;
        case predicate_transClosure: os << " transClosure "; break;
        case predicate_count: os << " count "; break;
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
    grounded_pis(i)->write(os); os<<" ";
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
    if (countedPred->type == TL::Predicate::predicate_comparison) {
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



void TL::FunctionValue::write(ostream& os) const {
// 	os << "fv ";
  atom->write(os);
	os << "=" << value;
}


void TL::FunctionValue::write_deprecated(ostream& os) const {
  atom->write_deprecated(os);
  os << " " << value;
}

void TL::FunctionAtom::write(ostream& os) const {
// 	os << "fv ";
  os << f->name << "(";
  uint i;
  FOR1D(args, i) {
    if (args(i) == 0) os << "X";
    else if (args(i) == 1) os << "Y";
    else if (args(i) == 2) os << "Z";
    else if (args(i) == 3) os << "V";
    else if (args(i) == 4) os << "W";
    else if (args(i) == 5) os << "U";
    else os << args(i);
    if (i < args.N-1)
      os << " ";
  }
  os << ")";
}


void TL::FunctionAtom::write_deprecated(ostream& os) const {
  os << f->id;
  uint i;
  FOR1D(args, i) {
    os << " " << args(i);
  }
}



void TL::Atom::write(ostream& os, bool withTypes) const {
  os << pred->name << "(";
  uint i;
  FOR1D(args, i) {
    if (args(i) == 0) os << "X";
    else if (args(i) == 1) os << "Y";
    else if (args(i) == 2) os << "Z";
    else if (args(i) == 3) os << "V";
    else if (args(i) == 4) os << "W";
    else if (args(i) == 5) os << "U";
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


void TL::Literal::write(ostream& os, bool withTypes) const {
  if (!positive) os << "-";
  atom->write(os, withTypes);
}


void TL::ComparisonAtom::write(ostream& os, bool withTypes) const {
// 	os << "comppt ";
  if (hasConstantBound()) {
    os << *fa1;
    writeComparison(comparisonType, os);
    os << bound;
  }
  else {
    CHECK(args.N%2==0, "Maldefined dynamic clit!")
    os << *fa1;
    writeComparison(comparisonType, os);
    os << *fa2;
  }
}



void TL::Atom::write_deprecated(ostream& os) const {
//     os << pred->category << " ";
//     os << pred->type << " ";
    os << pred->id << " ";
    uint i;
    FOR1D(args, i) {
        os << args(i) << " ";
    }
}


void TL::Literal::write_deprecated(ostream& os) const {
  atom->write_deprecated(os);
  if (positive)
    os << "1 ";
  else
    os << "0 ";
}


void TL::ComparisonAtom::write_deprecated(ostream& os) const {
  NIY;
//   /*  os << pred->id << " ";
//     os << f->id << " ";
//     os << comparisonType << " ";
//     os << hasConstantBound() << " ";
//     if (hasConstantBound()) {
//         os << bound << " ";
//     }
//     uint i;
//     FOR1D(args, i) {
//         os << args(i) << " ";
//     }*/
}


void TL::Rule::write_deprecated(ostream& os) const {
  if (action->pred->id == TL::DEFAULT_ACTION_PRED__ID) { // Default rule
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
    os << "# CONTEXT: ";
    FOR1D(context, i) {
      context(i)->write(os);
  //     os<<context(i);
      os << " ";
    }
    os << endl;
    // Default rule does not have an action specified...
    os << "# ACTION: ";
    if (action != NULL)
      action->write(os);
    else
      os << "default";
    os << endl;
    os << "# POST:" << endl;
    FOR1D(outcomes, i) {
      os << "#  " << probs(i) << " ";
      FOR1D(outcomes(i), j) {
        outcomes(i)(j)->write(os);
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
//     os << "# Rule for action "; action->write(os); os<<endl;
    os << "[ " << endl;
    FOR1D(context, i) {
      context(i)->write_deprecated(os);
      os << endl;
    }
    os << "]" << endl;
    action->write_deprecated(os);  os<<endl;
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
          outcomes(i)(j)->write_deprecated(os);  os<<" "<<endl;
        }
      }
      os << "] " << endl;
    }
    os << "]" << endl;
  }
}

void TL::Rule::write(ostream& os, bool withAction) const {
//  os << "r" << endl;
  uint i, j;
  // Default rule does not have an action specified...
  if (withAction) {
    os << "ACTION:"<<endl<<"  ";
    if (action != NULL)
      action->write(os, true);
    else
      os << "default";
    os << endl;
  }
  os << "CONTEXT:"<<endl<<"  ";
  if (context.N == 0)
    os << "--";
  FOR1D(context, i) {
    context(i)->write(os);
//     os<<context(i);
    os << " ";
  }
  os << endl;
  os << "OUTCOMES:" << endl;
  FOR1D(outcomes, i) {
    os.precision(3);
    os << "  ";
    if (probs(i) < 0.001) os << "0";
    else os << probs(i);
    os << " ";
    FOR1D(outcomes(i), j) {
      outcomes(i)(j)->write(os);
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

void TL::Substitution::write(ostream& os) {
// 	os << "s ";
	uint i;
	FOR1D(ins, i) {
    if (ins(i) == 0) os << "X";
    else if (ins(i) == 1) os << "Y";
    else if (ins(i) == 2) os << "Z";
    else if (ins(i) == 3) os << "V";
    else if (ins(i) == 4) os << "W";
    else if (ins(i) == 5) os << "U";
    else os << ins(i);
		os << "->" << getSubs(ins(i)) << " ";
	}
}


void TL::State::write(ostream& os, bool primOnly) const {
  bool breaks = true;
// 	os << "s ";
	uint i;
	FOR1D(lits_prim, i) {
		lits_prim(i)->write(os);
		os << " ";
	}
  if (/*lits_prim.N>0 && */breaks) os << endl;
  FOR1D(fv_prim, i) {
    fv_prim(i)->write(os);
    os << " ";
  }
  if (fv_prim.N>0 && breaks) os << endl;
  if (primOnly)
    return;
  FOR1D(lits_derived, i) {
    if (lits_derived(i)->atom->pred->id == 52)  // HAND_ID__PRED_DIFF_TOWER
      continue;
    lits_derived(i)->write(os);
    os << " ";
  }
  if (lits_derived.N>0 && breaks) os << endl;
  FOR1D(fv_derived, i) {
      fv_derived(i)->write(os);
        os << " ";
  }
}

void TL::State::write_deprecated(ostream& os) const {
//  os << "s ";
  uint i;
  os << "[" << endl;
  os << "[" << endl;
  FOR1D(lits_prim, i) {
    lits_prim(i)->write_deprecated(os);
    os<<endl;
  }
  os << "]" << endl;
  os << "[" << endl;
  FOR1D(fv_prim, i) {
    fv_prim(i)->write_deprecated(os);
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
    states(i)->write(os);
    os << endl;
    actions(i)->write(os);  
    os << endl;
  }
  cout << "(" << i << ") ";
  states(states.N-1)->write(os);
  os << endl;
}


void TL::Trial::write(const char* filename) const {
  ofstream out(filename);
  out << constants << endl << endl;
  uint i;
  FOR1D(states, i) {
    TL::write(states(i)->lits_prim, out);  out<<endl;
    TL::write(states(i)->fv_prim, out);  out<<endl;
    out<<endl;
    if (actions.N > i) {
      out<<*actions(i)<<endl<<endl;
    }
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


bool TL::Atom::operator==(TL::Atom& a) const {
  if (pred != a.pred)
    return false;
  if (this->args.N==0 && a.args.N==0) // special case if no slot assignments
    return true;
  if (args == a.args)
    return true;
  else
    return false;
}


bool TL::Literal::operator==(TL::Literal& l) const {
	if (positive == l.positive  &&  atom == l.atom)
		return true;
  else
    return false;
}

bool TL::ComparisonAtom::operator==(TL::Atom& a) const {
	TL::ComparisonAtom* clit = dynamic_cast<TL::ComparisonAtom*>(&a);
	if (clit == NULL)
		return false;
	return pred == clit->pred 
					&& args == clit->args;
}


bool TL::Atom::operator!=(TL::Atom& a) const {
  return !(*this == a);
}


bool TL::Literal::operator!=(TL::Literal& lit) const {
	return !(*this == lit);
}


bool TL::FunctionAtom::operator==(const TL::FunctionAtom& fa) const {
  return this->f == fa.f  &&  this->args == fa.args;
}

bool TL::FunctionAtom::operator!=(const TL::FunctionAtom& fa) const {
  return !(*this==fa);
}



bool TL::FunctionValue::operator==(TL::FunctionValue& fv) const {
	return atom == fv.atom
					&& TL::isZero(value - fv.value);
}

bool TL::FunctionValue::operator!=(TL::FunctionValue& fv) const {
	return !(*this==fv);
}



bool TL::State::operator==(const TL::State& s) const {
	return equivalent(lits_prim, s.lits_prim)
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
  w.writeNice(os); return os;
}

std::ostream& operator<<(std::ostream& os, const TL::Atom& a) {
  a.write(os); return os;
}

std::ostream& operator<<(std::ostream& os, const TL::Literal& l) {
  l.write(os); return os;
}

std::ostream& operator<<(std::ostream& os, const TL::ComparisonAtom& ca) {
  ca.write(os); return os;
}

std::ostream& operator<<(std::ostream& os, const TL::FunctionAtom& fa) {
  fa.write(os); return os;
}

std::ostream& operator<<(std::ostream& os, const TL::FunctionValue& f) {
  f.write(os); return os;
}

std::ostream& operator<<(std::ostream& os, const TL::Rule& r) {
  r.write(os); return os;
}

std::ostream& operator<<(std::ostream& os, const TL::Experience& e) {
  e.write(os); return os;
}

std::ostream& operator<<(std::ostream& os, const MT::Array<TL::Literal*> & lits) {
  TL::write(lits, os);
  return os;
}

std::ostream& operator<<(std::ostream& os, const MT::Array<TL::Atom*> & atoms) {
  TL::write(atoms, os);
  return os;
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

void TL::Substitution::apply(const LitL& unsub_lits, LitL& sub_lits) {
	sub_lits.clear();
	uint i;
	FOR1D(unsub_lits, i) {
		sub_lits.append(apply(unsub_lits(i)));
	}
}

void TL::Substitution::apply(const FuncVL& unsubFVs, FuncVL& subFVs) {
  subFVs.clear();
  uint i;
  FOR1D(unsubFVs, i) {
    subFVs.append(apply(unsubFVs(i)));
  }
}

TL::State* TL::Substitution::apply(const TL::State& state) {
  TL::State* s_new = new TL::State;
  apply(state.lits_prim, s_new->lits_prim);
  apply(state.lits_derived, s_new->lits_derived);
  apply(state.fv_prim, s_new->fv_prim);
  apply(state.fv_derived, s_new->fv_derived);
  
  return s_new;
}




TL::Atom* TL::Substitution::apply(Atom* in_a) {
  if (in_a->pred->type == TL::Predicate::predicate_comparison) {
    ComparisonAtom* in_ca = (ComparisonAtom*) in_a;
    ComparisonAtom* out_ca = new ComparisonAtom;
    out_ca->pred = in_ca->pred;
    out_ca->args = in_ca->args;
    out_ca->bound = in_ca->bound;
    out_ca->comparisonType = in_ca->comparisonType;
    out_ca->fa1 = apply(in_ca->fa1);
    if (in_ca->fa2 != NULL) {
      out_ca->fa2 = apply(in_ca->fa2);
    }
    return out_ca;
  }
  else {
    Atom* out_a = new Atom;
    out_a->pred = in_a->pred;
    out_a->args = in_a->args;
    uint i;
    FOR1D(in_a->args, i) {
      out_a->args(i) = getSubs(in_a->args(i));
    }
    return out_a;
  }
}

TL::Literal* TL::Substitution::apply(Literal* in_lit) {
  if (in_lit->atom->pred->type == TL::Predicate::predicate_comparison) {
    ComparisonLiteral* in_clit = (ComparisonLiteral*) in_lit;
    ComparisonLiteral* out_clit = new ComparisonLiteral;
    out_clit->positive = in_clit->positive;
    out_clit->atom = apply(in_clit->atom);
    return out_clit;
  }
  else {
    Literal* out_lit = new Literal;
    out_lit->positive = in_lit->positive;
    out_lit->atom = apply(in_lit->atom);
    return out_lit;
  }
}

TL::FunctionAtom* TL::Substitution::apply(FunctionAtom* in_fa) {
  FunctionAtom* out_fa = new FunctionAtom;
  out_fa->f = in_fa->f;
  out_fa->args = in_fa->args;
  uint i;
  FOR1D(in_fa->args, i) {
    out_fa->args(i) = getSubs(in_fa->args(i));
  }
  return out_fa;
}
  
TL::FunctionValue* TL::Substitution::apply(FunctionValue* in_fv) {
  FunctionValue* out_fv = new FunctionValue;
  out_fv->value = in_fv->value;
  out_fv->atom = apply(in_fv->atom);
  return out_fv;
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

void TL::Substitution::getOutsDistinct(uintA& ids) const {
  ids.clear();
  uint i;
  FOR1D(outs, i) {
    ids.setAppend(outs(i));
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


void TL::sort(PredL& preds) {
  if (preds.N==0) return;
  PredL sortedPreds;
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



void TL::sort(FuncL& funcs) {
  if (funcs.N==0) return;
  FuncL sortedFuncs;
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


bool TL::equivalent(const LitL& p1, const LitL& p2) {
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


bool TL::equivalent(const FuncVL& fv1, const FuncVL& fv2) {
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




void TL::writeComparison(ComparisonType comparisonType, std::ostream& os) {
  switch(comparisonType) {
    case comparison_equal: os << "=="; break;
    case comparison_less: os << "<"; break;
    case comparison_lessEqual: os << "<="; break;
    case comparison_greater: os << ">"; break;
    case comparison_greaterEqual: os << ">="; break;
    default: HALT("Unknown comparison type")
  }
}



void TL::baseConcepts(TL::Predicate* p, PredL& p_base, FuncL& f_base) {
    p_base.clear();
    f_base.clear();
    if (p->type == TL::Predicate::predicate_simple) {
        return;
    }
    else if (p->type == TL::Predicate::predicate_comparison) {
        return;
    }
    else if (p->type == TL::Predicate::predicate_conjunction) {
        TL::ConjunctionPredicate* scp = dynamic_cast<TL::ConjunctionPredicate*>(p);
        CHECK(scp!=NULL, "cast failed");
        uint i;
        FOR1D(scp->basePreds, i) {
            p_base.setAppend(scp->basePreds(i));
            PredL base_p_base;
            FuncL base_f_base;
            baseConcepts(scp->basePreds(i), base_p_base, base_f_base);
            p_base.setAppend(base_p_base);
            f_base.setAppend(base_f_base);
        }
    }
    else if (p->type == TL::Predicate::predicate_count) {
        TL::CountPredicate* cp = dynamic_cast<TL::CountPredicate*>(p);
        CHECK(cp!=NULL, "cast failed");
        p_base.setAppend(cp->countedPred);
        PredL base_p_base;
        FuncL base_f_base;
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
    else if (p->type == TL::Predicate::predicate_transClosure) {
        TL::TransClosurePredicate* tcp = dynamic_cast<TL::TransClosurePredicate*>(p);
        CHECK(tcp!=NULL, "cast failed");
        p_base.setAppend(tcp->basePred);
        PredL base_p_base;
        FuncL base_f_base;
        baseConcepts(tcp->basePred, base_p_base, base_f_base);
        p_base.setAppend(base_p_base);
        f_base.setAppend(base_f_base);
    }
    else {
      p->writeNice();
        NIY
    }
}


void TL::baseConcepts(TL::Function* f, PredL& p_base, FuncL& f_base) {
  p_base.clear();
  f_base.clear();
  if (f->type == TL::Function::function_simple) {
    return;
  }
  else if (f->type == TL::Function::function_count) {
    TL::CountFunction* fc = dynamic_cast<TL::CountFunction*>(f);
    CHECK(fc!=NULL, "cast failed");
    p_base.setAppend(fc->countedPred);
    PredL base_p_base;
    FuncL base_f_base;
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


TL::ConjunctionPredicate* TL::ConjunctionPredicate::newClone() {
    ConjunctionPredicate* p = new ConjunctionPredicate;
    *p = *this;
    return p;
}

TL::TransClosurePredicate* TL::TransClosurePredicate::newClone() {
    TransClosurePredicate* p = new TransClosurePredicate;
    *p = *this;
    return p;
}

TL::CountPredicate* TL::CountPredicate::newClone() {
    CountPredicate* p = new CountPredicate;
    *p = *this;
    return p;
}






TL::State::~State() {
  //  object deletion of concept-instances6 is managed ny the LogicEngine now!
//     uint i;
//     FOR1D(lits_prim, i) {
//         delete lits_prim(i);
//     }
//     FOR1D(lits_derived, i) {
//         delete lits_derived(i);
//     }
//     FOR1D(lits_comp, i) {
//         delete lits_comp(i);
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

TL::Experience::Experience(const TL::State& pre, TL::Atom* action, const TL::State& post) {
  this->pre = pre;
  this->action = action;
  this->post = post;
  calcChanges();
}


TL::Experience::Experience() {}

TL::Experience::~Experience() {
}

void TL::Experience::calcChanges() {
  // only look at p_prim
  changedConstants.clear();
  del.clear();
  add.clear();
  uint i;
  // pre+, post-
  FOR1D(pre.lits_prim, i) {
    if (post.lits_prim.findValue(pre.lits_prim(i)) < 0) {
      del.append(pre.lits_prim(i));
      changedConstants.setAppend(pre.lits_prim(i)->atom->args);
    }
  }
  // pre-, post+
  FOR1D(post.lits_prim, i) {
    if (pre.lits_prim.findValue(post.lits_prim(i)) < 0) {
      add.append(post.lits_prim(i));
      changedConstants.setAppend(post.lits_prim(i)->atom->args);
    }
  }
}


void TL::Experience::write(ostream& os) const {
  os << "ACTION: " << *action << endl;
  os << "PRE:    ";
  this->pre.write(os, true);
  
//   uint i, k;
//   uintA constants;
//   FOR1D(pre.lits_prim, i) {
//     FOR1D(pre.lits_prim(i)->atom->args, k) {
//       constants.setAppend(pre.lits_prim(i)->atom->args(k));
//     }
//   }
//   FOR1D(constants, i) {
//     LitL lits;
//     FOR1D(pre.lits_prim, k) {
//       if (pre.lits_prim(k)->atom->args.findValue(constants(i)) >= 0)
//         lits.append(pre.lits_prim(k));
//     }
//     cout<<constants(i)<<":  "<<lits<<endl;
//   }
  
//   os << endl;
  os << "POST:   ";
  this->post.write(os, true);
//   os << "Diff: "<<(add.N + del.N)<<" (+"<<add.N<<", -"<<del.N<<")"<<endl;
//   os << "ADD: ";  TL::write(add, os);  os<<endl;
//   os << "DEL: ";  TL::write(del, os);  os<<endl;
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
      if (this->ra(r)->action->pred->id == TL::DEFAULT_ACTION_PRED__ID)
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
    uint DEFAULT_ACTION_ID = TL::DEFAULT_ACTION_PRED__ID * 100;
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
    TL::write(*this, "ground_rules.dat.unsorted_backup");
    
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
//         ra_sorted(r3)->action->write(); cout<<"  ";
//       }
//       cout<<endl;
    }
    
//     FOR1D(ra_sorted, r3) {
//       ra_sorted(r3)->action->write(); cout<<"  ";
//     }
//     cout<<endl;
    
    CHECK(ra.N == ra_sorted.N, "Some strange rule-sorting mistake.");
    this->ra = ra_sorted;
  }
}

void TL::RuleSet::write(ostream& os) {
    uint i;
    os<<"#rules = "<<ra.N<<endl;
    FOR1D(ra, i) {
        os<<"["<<i<<"]"<<endl;
        ra(i)->write(os);
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

void TL::SubstitutionSet::write(ostream& os) {
  uint i;
  for (i=0; i<num(); i++) {
    this->elem(i)->write(os); os<<endl;
  }
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


void TL::writeNice(const PredL& pa, ostream& os) {
  uint k;
  FOR1D(pa, k) {
    pa(k)->writeNice(os);os<<endl;
  }
}


void TL::writeNice(const FuncL& fa, ostream& os) {
  uint k;
  FOR1D(fa, k) {
    fa(k)->writeNice(os);os<<endl;
  }
}


void TL::write(const AtomL& as, ostream& os) {
  uint k;
  FOR1D(as, k) {
    if (as(k)!=NULL)
      as(k)->write(os);
    else
      os<<"NULL";
    os << " ";
  }
}

void TL::write(const LitL& lits, ostream& os) {
  uint k;
  FOR1D(lits, k) {
    if (lits(k)!=NULL)
      lits(k)->write(os);
    else
      os<<"NULL";
    os << " ";
  }
}

void TL::write(const FuncVL& fvs, ostream& os) {
  uint k;
  FOR1D(fvs, k) {
    fvs(k)->write(os);
    os << " ";
  }
}


void TL::write(const FuncAL& fis, ostream& os) {
  uint k;
  FOR1D(fis, k) {
    fis(k)->write(os);
    os << " ";
  }
}


void TL::write(const MT::Array< LitL >& outcomes, ostream& os) {
  uint k;
  FOR1D(outcomes, k) {
    os << "(" << k << ") ";
    write(outcomes(k), os);
    os << endl;
  }
}


void TL::write(const ExperienceA& exs, ostream& os) {
  uint k;
  FOR1D(exs, k) {
    os << "(" << k << ") ";
    exs(k)->write(os);
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


// LANGUAGE

void TL::writeLanguage(const PredL& preds, const FuncL& funcs, const PredL& actions, TermTypeL& types, const char* filename) {
  ofstream out;
  out.open(filename);
  writeLanguage(preds, funcs, actions, types, out);
  out.close();
}

void TL::writeLanguage(const PredL& preds, const FuncL& funcs, const PredL& actions, TermTypeL& types, ostream& out) {
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
    if (actions(i)->id == TL::DEFAULT_ACTION_PRED__ID) continue;
    out << "#  ";  actions(i)->write(out);  out << endl;
    out << (*actions(i)) << endl;
  }
  out<<"]"<<endl;
  
  if (types.N > 0  &&  !(types.N==1 && types(0)->type_id == TL::TermType::ANY_id)) {
    out<<endl;
    out<<"["<<endl;
    FOR1D(types, i) {
      if (types(i)->type_id == TL::TermType::ANY_id) continue;
      types(i)->write_deprecated(out);  out << endl;
    }
    out<<"]"<<endl;
  }
}





void TL::write(const RuleSet& rules, ostream& out) {
  uint i;
  for (i=0; i<rules.num(); i++) {
    out << "# Rule #" << i << "  ("<< (i+1) << " out of " << rules.num() << ")" <<endl;
    out << *rules.elem(i) << endl;
  }
}



void TL::write(const RuleSet& rules, const char* filename) {
  ofstream out;
  out.open(filename);
  CHECK(out.is_open(), "Can't write to simulation sequence file.");
  write(rules, out);
  out.close();
}




