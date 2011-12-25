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


#include "logicReasoning.h"
#include "logicObjectManager.h"
#include <stdlib.h>

#define LE_fast 1



// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    H E L P E R   M E T H O D S
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------



uint TL::logicReasoning::calcNumTerms(const TL::Atom& a) {
  uintA terms;
  return calcTerms(a, terms);
}

uint TL::logicReasoning::calcNumTerms(const TL::Literal& lit) {
  return calcNumTerms(*lit.atom);
}

uint TL::logicReasoning::getArguments(uintA& args, const TL::Literal& lit) {
  args.clear();
  if (lit.atom->pred->type == TL::Predicate::predicate_comparison) {
    ComparisonAtom* ca = (ComparisonAtom*) lit.atom;
    args.setAppend(ca->fa1->args);
    if (ca->fa2 != NULL) args.setAppend(ca->fa2->args);
  }
  else {
    args.setAppend(lit.atom->args);
  }
  return args.N;
}

uint TL::logicReasoning::calcTerms(const TL::Atom& a, uintA& terms) {
  uint i;
  FOR1D(a.args, i) {
    terms.setAppend(a.args(i));
  }
  return terms.N;
}

uint TL::logicReasoning::calcTerms(const TL::Literal& lit, uintA& terms) {
  return calcTerms(*lit.atom, terms);
}

uint TL::logicReasoning::calcTerms(const AtomL& as, uintA& terms) {
  uint i;
  FOR1D(as, i) {
    uintA local_terms;
    calcTerms(*as(i), local_terms);
    terms.setAppend(local_terms);
  }
  return terms.N;
}

uint TL::logicReasoning::calcTerms(const LitL& lits, uintA& terms) {
  uint i;
  FOR1D(lits, i) {
    uintA local_terms;
    calcTerms(*lits(i), local_terms);
    terms.setAppend(local_terms);
  }
  return terms.N;
}




bool TL::logicReasoning::containsNegativeBasePredicate(TL::Predicate* p) {
    TL::ConjunctionPredicate* cp = dynamic_cast<TL::ConjunctionPredicate*>(p);
	if (cp == NULL)
		return false;
	uint i;
	FOR1D(cp->basePreds_positive, i) {
		if (!cp->basePreds_positive(i))
			return true;
	}
	return false;
}


void TL::logicReasoning::filterState_full(TL::State& s_filtered, const TL::State& s_full, const uintA& filter_objects, bool primOnly) {
  uint i, k;
  
  // lits_prim
  s_filtered.lits_prim.clear();
  FOR1D(s_full.lits_prim, i) {
    FOR1D(s_full.lits_prim(i)->atom->args, k) {
      if (filter_objects.findValue(s_full.lits_prim(i)->atom->args(k))<0)
        break;
    }
    if (k==s_full.lits_prim(i)->atom->args.N)
      s_filtered.lits_prim.append(s_full.lits_prim(i));
  }
  
  
  // fv_prim
  s_filtered.fv_prim.clear();
  FOR1D(s_full.fv_prim, i) {
    FOR1D(s_full.fv_prim(i)->atom->args, k) {
      if (filter_objects.findValue(s_full.fv_prim(i)->atom->args(k))<0)
        break;
    }
    if (k==s_full.fv_prim(i)->atom->args.N)
      s_filtered.fv_prim.append(s_full.fv_prim(i));
  }

  if (!primOnly) {
    // lits_derived
    s_filtered.lits_derived.clear();
    FOR1D(s_full.lits_derived, i) {
      FOR1D(s_full.lits_derived(i)->atom->args, k) {
        if (filter_objects.findValue(s_full.lits_derived(i)->atom->args(k))<0)
          break;
      }
      if (k==s_full.lits_derived(i)->atom->args.N)
        s_filtered.lits_derived.append(s_full.lits_derived(i));
    }
   
    // fv_derived
    s_filtered.fv_derived.clear();
    FOR1D(s_full.fv_derived, i) {
      FOR1D(s_full.fv_derived(i)->atom->args, k) {
        if (filter_objects.findValue(s_full.fv_derived(i)->atom->args(k))<0)
          break;
      }
      if (k==s_full.fv_derived(i)->atom->args.N)
        s_filtered.fv_derived.append(s_full.fv_derived(i));
    }
  }
  else {
    s_filtered.lits_derived.clear();
    s_filtered.fv_derived.clear();
    s_filtered.derivedDerived = false;
//     derive(&s_filtered);  // don't derive!
  }
}



void TL::logicReasoning::filterState_atleastOne(TL::State& s_filtered, const TL::State& s_full, const uintA& filter_objects, bool primOnly) {
  uint i, k;
  
  // lits_prim
  s_filtered.lits_prim.clear();
  FOR1D(s_full.lits_prim, i) {
    if (s_full.lits_prim(i)->atom->args.N == 0)
      s_filtered.lits_prim.append(s_full.lits_prim(i));
    FOR1D(s_full.lits_prim(i)->atom->args, k) {
      if (filter_objects.findValue(s_full.lits_prim(i)->atom->args(k))>=0) {
        s_filtered.lits_prim.append(s_full.lits_prim(i));
        break;
      }
    }
  }
  
  // fv_prim
  s_filtered.fv_prim.clear();
  FOR1D(s_full.fv_prim, i) {
    if (s_full.fv_prim(i)->atom->args.N == 0)
      s_filtered.fv_prim.append(s_full.fv_prim(i));
    FOR1D(s_full.fv_prim(i)->atom->args, k) {
      if (filter_objects.findValue(s_full.fv_prim(i)->atom->args(k))>=0) {
        s_filtered.fv_prim.append(s_full.fv_prim(i));
        break;
      }
    }
  }

  if (!primOnly) {
    // lits_derived
    s_filtered.lits_derived.clear();
    FOR1D(s_full.lits_derived, i) {
      if (s_full.lits_derived(i)->atom->args.N == 0)
        s_filtered.lits_derived.append(s_full.lits_derived(i));
      FOR1D(s_full.lits_derived(i)->atom->args, k) {
        if (filter_objects.findValue(s_full.lits_derived(i)->atom->args(k))>=0) {
          s_filtered.lits_derived.append(s_full.lits_derived(i));
          break;
        }
      }
    }
    
    // fv_derived
    s_filtered.fv_derived.clear();
    FOR1D(s_full.fv_derived, i) {
      if (s_full.fv_derived(i)->atom->args.N == 0)
        s_filtered.fv_derived.append(s_full.fv_derived(i));
      FOR1D(s_full.fv_derived(i)->atom->args, k) {
        if (filter_objects.findValue(s_full.fv_derived(i)->atom->args(k))>=0) {
          s_filtered.fv_derived.append(s_full.fv_derived(i));
          break;
        }
      }
    }
  }
  else {
    s_filtered.lits_derived.clear();
    s_filtered.fv_derived.clear();
    s_filtered.derivedDerived = false;
//     derive(&s_filtered);  // don't derive!
  }
}


void TL::logicReasoning::getConstants(const FuncVL& fvs, uintA& constants) {
  constants.clear();
  uint i, s;
  FOR1D(fvs, i) {
      FOR1D(fvs(i)->atom->args, s) {
          constants.setAppend(fvs(i)->atom->args(s));
      }
  }
}


void TL::logicReasoning::getConstants(const LitL& lits, uintA& constants) {
  constants.clear();
  uint i, s;
  FOR1D(lits, i) {
    FOR1D(lits(i)->atom->args, s) {
      constants.setAppend(lits(i)->atom->args(s));
    }
  }
}



void TL::logicReasoning::getConstants(const TL::State& s, uintA& constants) {
  uintA localConstants;
  getConstants(s.lits_prim, localConstants);
  constants.setAppend(localConstants);
  localConstants.clear();
  
  getConstants(s.lits_derived, localConstants);
  constants.setAppend(localConstants);
  localConstants.clear();
  
  getConstants(s.fv_prim, localConstants);
  constants.setAppend(localConstants);
  localConstants.clear();
  
  getConstants(s.fv_derived, localConstants);
  constants.setAppend(localConstants);
  localConstants.clear();
  
  TL::sort_asc(constants);
  
  // HACK if no constants found, probably almost not literal held true...
  // This is the case if there are not always true typing predicates.
  if (constants.N < 3) constants = TL::logicObjectManager::constants;
}


uint TL::logicReasoning::getArgument(const TL::State& s, const TL::Predicate& pred) {
  CHECK(pred.d == 1, "");
  uintA args;
  getArguments(args, s, pred);
  if (args.N == 1)
    return args(0);
  else
    return UINT_MAX;
}

void TL::logicReasoning::getArguments(uintA& args, const TL::State& s, const TL::Predicate& pred) {
  args.clear();
  uint i;
  if (pred.category == category_primitive) {
    FOR1D(s.lits_prim, i) {
      if (*s.lits_prim(i)->atom->pred == pred)
        args.setAppend(s.lits_prim(i)->atom->args);
    }
  }
  else {
    FOR1D(s.lits_derived, i) {
      if (*s.lits_derived(i)->atom->pred == pred)
        args.setAppend(s.lits_derived(i)->atom->args);
    }
  }
}


void TL::logicReasoning::getUnconstrainedNegatedArguments(uintA& args, const LitL& lits) {
  args.clear();
  uint i, k;
  uintA args_pos;
  FOR1D(lits, i) {
    if (lits(i)->positive) {
      if (lits(i)->atom->pred->type == TL::Predicate::predicate_comparison) {
        ComparisonAtom* ca = (ComparisonAtom*) lits(i)->atom;
        args_pos.setAppend(ca->fa1->args);
        if (ca->fa2 != NULL) args_pos.setAppend(ca->fa1->args);
      }
      else {
        args_pos.setAppend(lits(i)->atom->args);
      }
    }
  }
  FOR1D(lits, i) {
    if (!lits(i)->positive) {
      FOR1D(lits(i)->atom->args, k) {
        if (args_pos.findValue(lits(i)->atom->args(k)) < 0)
          args.setAppend(lits(i)->atom->args(k));
      }
    }
  }
}


void TL::logicReasoning::getRelatedObjects(uintA& objs_related, uint id, bool id_covers_first, const TL::Predicate& pred, const TL::State& s) {
  objs_related.clear();
  uint i;
  if (pred.category == category_primitive) {
    FOR1D(s.lits_prim, i) {
      if (*s.lits_prim(i)->atom->pred == pred) {
        if (id_covers_first) {
          if (s.lits_prim(i)->atom->args(0) == id)
            objs_related.append(s.lits_prim(i)->atom->args(1));
        }
        else {
          if (s.lits_prim(i)->atom->args(1) == id)
            objs_related.append(s.lits_prim(i)->atom->args(0));
        }
      }
    }
  }
  else {
    FOR1D(s.lits_derived, i) {
      if (*s.lits_derived(i)->atom->pred == pred) {
        if (id_covers_first) {
          if (s.lits_derived(i)->atom->args(0) == id)
            objs_related.append(s.lits_derived(i)->atom->args(1));
        }
        else {
          if (s.lits_derived(i)->atom->args(1) == id)
            objs_related.append(s.lits_derived(i)->atom->args(0));
        }
      }
    }
  }
  
}


void TL::logicReasoning::getGeneralRelatedObjects(uintA& objs_related, uint id, const TL::State& s) {
  objs_related.clear();
  uint i;
  FOR1D(s.lits_prim, i) {
    if (s.lits_prim(i)->atom->pred->d > 1) {
      if (s.lits_prim(i)->atom->args.findValue(id) >= 0)
        objs_related.setAppend(s.lits_prim(i)->atom->args);
    }
  }
  FOR1D(s.lits_derived, i) {
    if (s.lits_derived(i)->atom->pred->d > 1) {
      if (s.lits_derived(i)->atom->args.findValue(id) >= 0)
        objs_related.setAppend(s.lits_derived(i)->atom->args);
    }
  }
  objs_related.removeValueSafe(id);
}




void TL::logicReasoning::getValues(arr& values, const TL::State& s, const TL::Function& f, const uintA& objs) {
  uint i;
  int k;
  values.resize(objs.N);
  if (f.category == category_primitive) {
    FOR1D(s.fv_prim, i) {
      if (*s.fv_prim(i)->atom->f == f) {
        k = objs.findValue(s.fv_prim(i)->atom->args(0));
        if (k >= 0)
          values(k) = s.fv_prim(i)->value;
      }
    }
  }
  else {
    FOR1D(s.fv_derived, i) {
      if (*s.fv_derived(i)->atom->f == f) {
        k = objs.findValue(s.fv_derived(i)->atom->args(0));
        if (k >= 0)
          values(objs.findValue(s.fv_derived(i)->atom->args(0))) = s.fv_derived(i)->value;
      }
    }
  }
}

double TL::logicReasoning::getValue(uint id, const MT::String& function_name, const TL::State& s) {
  return getValue(id, TL::logicObjectManager::getFunction(function_name), s);
}

double TL::logicReasoning::getValue(TL::Function* f, const TL::State& s) {
  CHECK(f->d == 0, "incorrect arity of function");
  uint i;
  if (f->category == category_primitive) {
    FOR1D(s.fv_prim, i) {
      if (s.fv_prim(i)->atom->f == f) {
        return s.fv_prim(i)->value;
      }
    }
  }
  else {
    FOR1D(s.fv_derived, i) {
      if (s.fv_derived(i)->atom->f == f) {
        return s.fv_derived(i)->value;
      }
    }
  }
  HALT("no function value found");
  return TL_DOUBLE_NIL;
}

double TL::logicReasoning::getValue(uint id, TL::Function* f, const TL::State& s) {
  uintA args;
  args.append(id);
  return getValue(args, f, s);
}


double TL::logicReasoning::getValue(const uintA& args, TL::Function* f, const TL::State& s) {
  FunctionAtom* fa = logicObjectManager::getFA(f, args);
  double value = getValue(*fa, s.fv_prim);
  if (!TL::areEqual(value, TL_DOUBLE_NIL))
    return value;
  value = getValue(*fa, s.fv_derived);
  if (!TL::areEqual(value, TL_DOUBLE_NIL))
    return value;
  // No function value found :-(
  cerr<<endl<<endl<<endl;
  cerr<<"STATE:   ";  s.write(cerr);  cerr<<endl;
  HALT("no function value found for function "<<f->name<<" with id="<<f->id<<" and args="<<args);
  return TL_DOUBLE_NIL;
}


bool TL::logicReasoning::holds_straight(uint id, const MT::String& predicate_name, const TL::State& s) {
  TL::Predicate* pred = TL::logicObjectManager::getPredicate(predicate_name);
  uint i;
  if (pred->category == category_primitive) {
    FOR1D(s.lits_prim, i) {
      if (s.lits_prim(i)->atom->pred == pred) {
        if (s.lits_prim(i)->atom->args(0) == id)
          return true;
      }
    }
  }
  else {
    FOR1D(s.lits_derived, i) {
      if (s.lits_derived(i)->atom->pred == pred) {
        if (s.lits_derived(i)->atom->args(0) == id)
          return true;
      }
    }
  }
  return false;
}



bool TL::logicReasoning::negativesLast(const LitL& lits) {
	bool positivesDone = false;
	uint i;
	FOR1D(lits, i) {
// 		PRINT(lits(i)->positive)
		if (!lits(i)->positive && !positivesDone) {
			positivesDone = true;
			continue;
		}
		else if (lits(i)->positive && positivesDone)
			return false;
	}
	return true;
}

bool TL::logicReasoning::negativesUngroundedLast(const LitL& lits) {
	bool negativesUngroundedStarted = false;
	uint i;
	FOR1D(lits, i) {
		if (!isGrounded(lits(i)) && !lits(i)->positive && !negativesUngroundedStarted) {
			negativesUngroundedStarted = true;
			continue;
		}
		else if (lits(i)->positive && negativesUngroundedStarted)
			return false;
	}
	return true;
}




void TL::logicReasoning::generateBaseInstantiations(LitL& pts_base, FuncVL& fvs_base, TL::Literal* lit) {
  generateBaseInstantiations(pts_base, fvs_base, lit, TL::logicObjectManager::constants);
}

void TL::logicReasoning::generateBaseInstantiations(LitL& pts_base, FuncVL& fvs_base, TL::Literal* lit, uintA& free_constants) {
    uint DEBUG = 0;
    if (lit->atom->pred->category == category_primitive)
        return;
    uint i, k, c;
    if (lit->atom->pred->type == TL::Predicate::predicate_conjunction) {
        TL::ConjunctionPredicate* scp = (TL::ConjunctionPredicate*) lit->atom->pred;
        if (DEBUG>0) {scp->writeNice(cout);cout<<endl;}
        uint l=0;
        uint basePred_startID;
        FOR1D(scp->basePreds, i) {
            uintA localFreeVars;
            basePred_startID=l;
            for (k=0; k<scp->basePreds(i)->d; k++) {
                if (scp->basePreds_mapVars2conjunction(basePred_startID+k) >= scp->d)
                    localFreeVars.setAppend(scp->basePreds_mapVars2conjunction(basePred_startID+k));
            }
            if (localFreeVars.N > 0) {
                MT::Array< uintA > combos;
                TL::allPossibleLists(combos, free_constants, localFreeVars.N, true, true);
                FOR1D(combos, c) {
                    uintA base_args(scp->basePreds(i)->d);
                    FOR1D(base_args, k) {
                        if (scp->basePreds_mapVars2conjunction(basePred_startID+k) < scp->d)
                            base_args(k) = lit->atom->args(scp->basePreds_mapVars2conjunction(basePred_startID+k));
                        else
                            base_args(k) = combos(c)(scp->basePreds_mapVars2conjunction(basePred_startID+k) - scp->d);
                    }
                    TL::Literal* base_pt = logicObjectManager::getLiteral(scp->basePreds(i), scp->basePreds_positive(i), base_args);
                    pts_base.append(base_pt);
                }
            }
            else {
                uintA base_args(scp->basePreds(i)->d);
                FOR1D(base_args, k) {
                    base_args(k) = lit->atom->args(scp->basePreds_mapVars2conjunction(basePred_startID+k));
                }
                TL::Literal* base_pt = logicObjectManager::getLiteral(scp->basePreds(i), scp->basePreds_positive(i), base_args);
                pts_base.append(base_pt);
            }
            basePred_startID += scp->basePreds(i)->d;
        }
    }
    else if (lit->atom->pred->type == TL::Predicate::predicate_count) {
        NIY
//         TL::CountPredicate* cp = (TL::CountPredicate*) p->pred;
//         uintA base_args(scp->basePreds(i)->d);
//         FOR1D(base_args, k) {
//             if (scp->basePreds_mapVars2conjunction(k) < scp->basePreds(i)->d)
//                 base_args(k) = p->args(scp->basePreds_mapVars2conjunction(k));
//             else
//                 base_args(k) = scp->basePreds_mapVars2conjunction(k);
//         }
//         countedPred_mapVars2derived
//                 
//         TL::Literal* lit = getLiteral(cp->countedPred, true, base_args);
//         pts_base.append(lit);
    }
    else if (lit->atom->pred->type == TL::Predicate::predicate_transClosure) {
        TL::TransClosurePredicate* tcp = (TL::TransClosurePredicate*) lit->atom->pred;
        CHECK(tcp->d == 2, "tcps so far implemented nur fuer binary");
        LitL lits_base_cands;
        logicObjectManager::getLiterals(lits_base_cands, tcp->basePred, free_constants, true);
        FOR1D(lits_base_cands, i) {
            if (lit->atom->args(0) != lits_base_cands(i)->atom->args(1)
                &&  lit->atom->args(1) != lits_base_cands(i)->atom->args(0))
                pts_base.append(lits_base_cands(i));
        }
    }
    else {
        NIY
    }
}





uint TL::logicReasoning::numberLiterals(const MT::Array< LitL >& LitLs) {
	uint no = 0;
	uint i;
	FOR1D(LitLs, i) {
		no += LitLs(i).N;
	}
	return no;
}


bool TL::logicReasoning::containsNegativeLiterals(const LitL& LitLs) {
	uint i;
	FOR1D(LitLs, i) {
		if (!LitLs(i)->positive)
			return true;
	}
	return false;
}


bool TL::logicReasoning::containsNegativeLiterals(const TL::State& s) {
  if (containsNegativeLiterals(s.lits_prim))
		return true;;
	return false;
}


// positives first
void TL::logicReasoning::sort(LitL& lits) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"logicReasoning::sort [START]"<<endl;}
  
  if (DEBUG>0) {cout<<"Unsorted:  "; TL::write(lits); cout<<endl;}
  
  // sowas aehnliches schon mal in RuleSet::sort
  uint i;
  uintA keys(lits.N);
  // Stellen 1-4: Argumente
  // Stellen 5-6: Praedikate-ID
  // Stelle 7: primitive Praedikat
  // Stelle 8: positiv
  FOR1D(lits, i) {
    uint key = 0;
    // Stellen 1-4: Argumente
    uintA args = lits(i)->atom->args;
    if (lits(i)->atom->pred->type == TL::Predicate::predicate_comparison) {
      ComparisonAtom* ca = (ComparisonAtom*) lits(i)->atom;
      args.clear();  args.append(ca->fa1->args);  if (ca->fa2 != NULL) args.append(ca->fa2->args);
    }
    if (args.N > 0) {
      CHECK(args(0) < 100, "");
      key += 10e2 * args(0);
    }
    if (args.N > 1) {
      CHECK(args(1) < 100, "");
      key += args(1);
    }
    CHECK(args.N <= 2, "");
    // Stellen 5-6: Praedikate-ID
    key += 10e4 * lits(i)->atom->pred->id;
    // Stelle 7: primitive Praedikat
    if (lits(i)->atom->pred->category == category_derived) {
      key += 10e6;
    }
    // Stelle 8: positiv
    if (!lits(i)->positive) {
      key += 10e7;
    }
    keys(i) = key;
//     cout<<"("<<i<<") "<<*lits(i)<<"\t"<<keys(i)<<endl;
  }

  uintA sortedIndices;
//   PRINT(sortedIndices);
  TL::sort_asc_keys(sortedIndices, keys);
//   PRINT(sortedIndices);

  LitL pis_sorted;
  FOR1D(sortedIndices, i) {
    pis_sorted.append(lits(sortedIndices(i)));
  }

  lits = pis_sorted;
  
  if (DEBUG>0) {cout<<"Sorted:  "; TL::write(lits); cout<<endl<<endl;}
  if (DEBUG>0) {cout<<"logicReasoning::sort [END]"<<endl;}
}


void TL::logicReasoning::sort(AtomL& atoms) {
  LitL lits;
  uint i;
  FOR1D(atoms, i) {
    lits.append(logicObjectManager::getLiteral(atoms(i)));
  }
  sort(lits);
  atoms.clear();
  FOR1D(lits, i) {
    atoms.append(lits(i)->atom);
  }
}


void TL::logicReasoning::sortNegativesUngroundedLast(LitL& p) {
    LitL p_ordered;
    LitL p_negUngrounded;
    uint i;
    FOR1D(p, i) {
        if (!p(i)->positive && !isGrounded(p(i))) {
            p_negUngrounded.append(p(i));
        }
        else
            p_ordered.append(p(i));
    }
    p_ordered.append(p_negUngrounded);
    p=p_ordered;
}




bool TL::logicReasoning::isEqualityLiteral(TL::Literal* lit) {
  if (lit->atom->pred->type != TL::Predicate::predicate_comparison)
    return false;
  TL::ComparisonLiteral* clit = dynamic_cast<TL::ComparisonLiteral*>(lit);
  if (clit == NULL) HALT("cast failed for lit = "<<*lit);
  if (((ComparisonAtom*) clit->atom)->comparisonType == comparison_equal)
    return true;
  else
    return false;
}


void TL::logicReasoning::removeRedundant(LitL& p) {
	LitL p1;
	uint i, j;
	FOR1D(p, i) {
		FOR1D(p1, j) {
			if (*p(i) == *p1(j))
				break;
		}
		if (j == p1.N)
			p1.append(p(i));
	}
	p = p1;
}








void TL::logicReasoning::usedValues(const TL::Function& f, const TL::State& s, arr& values) {
    values.clear();
    uint i;
    if (f.category == category_primitive) {
        FOR1D(s.fv_prim, i) {
            if (s.fv_prim(i)->atom->f->id == f.id) {
                values.setAppend(s.fv_prim(i)->value); 
            }
        }
    }
    else {
        FOR1D(s.fv_derived, i) {
            if (s.fv_derived(i)->atom->f->id == f.id) {
                values.setAppend(s.fv_derived(i)->value); 
            }
        }
    }
}


void TL::logicReasoning::calcFreeVars(const TL::ConjunctionPredicate& scp, uintA& freeVars_pos, uintA& freeVars_neg) {
    uint all_i = 0;
    uint j, k;
    FOR1D(scp.basePreds, j) {
        for(k=0; k<scp.basePreds(j)->d; k++) {
            // if free var
            if (scp.basePreds_mapVars2conjunction(all_i) >= scp.d) {
                if (scp.basePreds_positive(j))
                    freeVars_pos.setAppend(scp.basePreds_mapVars2conjunction(all_i));
                else
                    freeVars_neg.setAppend(scp.basePreds_mapVars2conjunction(all_i));
            }
            all_i++;
        }
    }
}









void TL::logicReasoning::filterPurelyAbstract(const LitL& allPreds, LitL& filteredPreds) {
  filteredPreds.clear();
  uint i;
  FOR1D(allPreds, i) {
    if (isAbstract(allPreds(i)))
      filteredPreds.append(allPreds(i));
  }
}



void TL::logicReasoning::negate(const LitL& lits, LitL& predTs_negated) {
  predTs_negated.clear();
  uint i;
  TL::Literal* lit;
  FOR1D(lits, i) {
    lit = TL::logicObjectManager::getLiteral(lits(i)->atom->pred, !lits(i)->positive, lits(i)->atom->args);
    predTs_negated.append(lit);
  }
}


int TL::logicReasoning::findPattern(const AtomL& actions, uint minRepeats) {
  uint length, repeat, pos;
  int max_repeat_length = 0;
  for (length=1; length<=actions.N / minRepeats; length++) {
    bool successful_repeat = true;
    for (repeat=1; repeat<minRepeats && successful_repeat; repeat++) {
      for (pos=0; pos<length && successful_repeat; pos++) {
        if (actions(repeat * length + pos) != actions(pos))
          successful_repeat = false;
      }
    }
    if (successful_repeat)
      max_repeat_length = length;
  }
  return max_repeat_length;
}


double TL::logicReasoning::getValue(const FunctionAtom& fa, const FuncVL& fvs) {
  uint i;
  FOR1D(fvs, i) {
    if (*fvs(i)->atom == fa)
      return fvs(i)->value;
  }
  return TL_DOUBLE_NIL;
}




bool TL::logicReasoning::containsLiteral(const LitL& p, const TL::Literal& literal) {
  uint i;
  FOR1D(p, i) {
    if (literal == *p(i))
      return true;
  }
  return false;
}




/*
	Example: clear(X) <-- not on(Y,X)
	Then, we need to ground X before we start cover, but _not_ Y, since X is bound
	while Y is not. In other words, the meaning is: for a fixed X, for all Y it holds
	that they are not on X.
	This method returns X.
*/
void TL::logicReasoning::calcUnconstrainedNegatedArguments(TL::ConjunctionPredicate* cp, uintA& vars) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"calcUnconstrainedNegatedArguments [START]"<<endl;
  if (DEBUG>0) {cp->writeNice(cout); cout<<endl;}
  uint i, j;
  // Start by taking all vars.
  FOR1D(cp->basePreds_mapVars2conjunction, i)
            if (vars.findValue(cp->basePreds_mapVars2conjunction(i)) < 0)
            vars.append(cp->basePreds_mapVars2conjunction(i));
  if (DEBUG>0) PRINT(vars)
  j = 0;
  // Then, remove all vars which are constrained by a positive literal.
  FOR1D(cp->basePreds, i) {
    if (cp->basePreds_positive(i)) {
      for(;j<cp->basePreds(i)->d;j++)
                vars.removeValueSafe(cp->basePreds_mapVars2conjunction(j++));
    }
    else {
      j+=cp->basePreds(i)->d;
    }
  }
  // Finally, remove all vars which appear as arguments in the TL::logicObjectManager::p_derived predicate.
  for (i=vars.N; i>0; i--) {
    if (vars(i-1) >= cp->d)
      vars.remove(i-1);
  }
    
  if (DEBUG>0) PRINT(vars)
  if (DEBUG>0) cout<<"calcUnconstrainedNegatedArguments [END]"<<endl;
}












void TL::logicReasoning::createAllPossibleSubstitutions(const uintA& vars, TL::SubstitutionSet& subs) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"createAllPossibleSubstitutions [START]"<<endl;
  if (DEBUG>0) {PRINT(vars) PRINT(TL::logicObjectManager::constants)}
  TL::Substitution* sub;
  uint j;
  uintA assignment(vars.N);
  assignment.setUni(0);
  int id = vars.N-1;
  // beim change ganz nach hinten rutschen und wieder alle zuruecksetzen
  while (true) {
    if (DEBUG>0) PRINT(assignment)
    sub = new TL::Substitution;
    FOR1D(vars, j)
      sub->addSubs(vars(j), TL::logicObjectManager::constants(assignment(j)));
    if (DEBUG>1) {sub->write(cout); cout<<endl;}
    subs.append(sub);
    
    while (id>=0 && assignment(id) >= TL::logicObjectManager::constants.N-1) {
      id--;
    }
    if (id<0)
      break;
    
    assignment(id)++;
    while (++id < (int) assignment.N) {
      assignment(id) = 0;
    }
    id=assignment.N-1;
  }
  CHECK(subs.num() == pow(TL::logicObjectManager::constants.N, vars.N), "Oh no, we forgot some subs!")
  if (DEBUG>0) cout<<"createAllPossibleSubstitutions [END]"<<endl;
}






// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    C H E C K   G R O U N D   T R U T H
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------



bool TL::logicReasoning::holds_positive(const LitL& positive_grounded_lits, TL::Literal* positive_lit) {
  uint DEBUG = 0;
  if (DEBUG>0) cout << "holds_positive [START]" << endl;
  if (DEBUG>0) {
    cout<<"Checking truth of: "; positive_lit->write(cout); cout<<endl;
    cout<<"In knowledge-base: "; write(positive_grounded_lits);cout<<endl;
  }
  
  uint i;
  CHECK(positive_lit->positive, "Predicate is negative!");
  CHECK(!containsNegativeLiterals(positive_grounded_lits), "KB findValue negative predicates!");
    
  bool literalFound = false;
  FOR1D(positive_grounded_lits, i) {
    if (*positive_grounded_lits(i) == *positive_lit) {
      literalFound = true;
      break;
    }
  }
  
  if (DEBUG>0) cout<<"  ---> "<<literalFound<<endl;
  if (DEBUG>0) cout << "holds_positive [END]" << endl;
  
  return literalFound;
}

bool TL::logicReasoning::holds(const LitL& positive_grounded_lits, TL::Literal* lit) {
  uint DEBUG = 0;
  if (DEBUG>0) cout << "holds [START]" << endl;
  if (DEBUG>0) {
    cout<<"Checking truth of "<<*lit<<endl<<"In: ";  TL::write(positive_grounded_lits);  cout<<endl;
  }
  bool isTrue;
  if (lit->positive)
    isTrue = holds_positive(positive_grounded_lits, lit);
  else {
    TL::Literal* lit_pos = logicObjectManager::getLiteralNeg(lit);
    isTrue = !holds_positive(positive_grounded_lits, lit_pos);
  }
  if (DEBUG>0) cout<<" --> "<<isTrue<<endl;
  if (DEBUG>0) cout << "holds [END]" << endl;
  return isTrue;
}


bool TL::logicReasoning::holds(const FuncVL& fvs, TL::ComparisonLiteral* lit) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"holds(const FuncVL& fvs, TL::ComparisonLiteral* lit) [START]"<<endl; }
  if (DEBUG>0) {cout<<"lit: "<<*lit<<endl;  cout<<"fvs:  "; TL::write(fvs);  cout<<endl;}
  ComparisonAtom* ca = (ComparisonAtom*) lit->atom;
  bool isTrue;
  if (ca->hasConstantBound()) {
    isTrue = TL::compare(getValue(*ca->fa1, fvs), ca->bound, ca->comparisonType);
  }
  else {
    isTrue = TL::compare(getValue(*ca->fa1, fvs), getValue(*ca->fa2, fvs), ca->comparisonType);
  }
  if (DEBUG>0) {PRINT(isTrue);}
  if (DEBUG>0) {cout<<"holds(const FuncVL& fvs, TL::ComparisonLiteral* lit) [END]"<<endl; }
  return isTrue;
}



bool TL::logicReasoning::holds(const TL::State& s, TL::Literal* lit) {
  uint DEBUG = 0;
  if (DEBUG>0) cout << "holds [START]" << endl;
  if (DEBUG>0) {
          cout<<"Checking truth of: "; lit->write(cout); cout<<endl;
          cout<<"In state: "; s.write(cout); cout<<endl;
  }	
  bool isTrue;
  if (lit->atom->pred->category == category_derived) {
    isTrue = holds(s.lits_derived, lit);
  }
  else {
    if (lit->atom->pred->type == TL::Predicate::predicate_simple) {
      isTrue = holds(s.lits_prim, lit);
    }
    else if (lit->atom->pred->type == TL::Predicate::predicate_comparison) {
      TL::ComparisonLiteral* clit = dynamic_cast<TL::ComparisonLiteral*>(lit);
      CHECK(clit!=NULL, "Invalid comparison predicate tuple")
      if (((ComparisonAtom*)clit->atom)->fa1->f->category == category_primitive)
        isTrue = holds(s.fv_prim, clit);
      else
        isTrue = holds(s.fv_derived, clit);
    }
    else
      HALT("A crazy reference predicate indeed.");
  }
	
	if (DEBUG>0) cout<<" --> "<<isTrue<<endl;
	if (DEBUG>0) cout << "holds [END]" << endl;
	
	return isTrue;
}


bool TL::logicReasoning::holds(const TL::State& s, const LitL& lits_grounded) {
  uint i;
  FOR1D(lits_grounded, i) {
    if (!holds(s, lits_grounded(i)))
      return false;
  }
  return true;
}

bool TL::logicReasoning::holds(const TL::State& s1, const TL::State& s2) {
  return holds(s1, s2.lits_prim);
}













// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    U N I F I C A T I O N   A N D   C O V E R I N G
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


bool TL::logicReasoning::unify_basic(const uintA& grounded_args, const uintA& other_args, TL::Substitution& sub) {
  uint i;
  FOR1D(grounded_args, i) {
    CHECK(isConstant(grounded_args(i)), "So not, my friend, I got some ungrounded grounded_args here!")
  }
  CHECK(grounded_args.N==other_args.N, "wrong slot assignments:  grounded_args="<<grounded_args<<"  vs.  other_args="<<other_args);
  FOR1D(other_args, i) {
    if (isConstant(other_args(i))) {
      if (other_args(i) != grounded_args(i))
        return false;
    }
    else {
      if (sub.hasSubs(other_args(i))) {
        if (sub.getSubs(other_args(i)) != grounded_args(i))
          return false;
      }
      else {
        sub.addSubs(other_args(i), grounded_args(i));
      }
    }
  }
  return true;
}


bool TL::logicReasoning::unify(const TL::Atom* grounded_atom, const TL::Atom* other_atom, TL::Substitution& sub) {
  uint DEBUG = 0;
  if (!isGrounded(grounded_atom)) {
    HALT("Atom is not grounded:  "<<*grounded_atom);
  }

  if (grounded_atom->pred != other_atom->pred)
    return false;

  if (DEBUG>0) {grounded_atom->write(cout); cout << endl;}
   
  if (grounded_atom->args.N != other_atom->args.N) { 
      cout<<"In  logicReasoning::unify(const TL::Atom* grounded_atom, const TL::Atom* other_atom, TL::Substitution& sub):"<<endl;
      cout<<"grounded_atom: ";grounded_atom->write(cout);cout<<endl;
      cout<<"other_lit: ";other_atom->write(cout);cout<<endl;
      sub.write(cout);
      HALT("");
  }
  
  return unify_basic(grounded_atom->args, other_atom->args, sub);
}


bool TL::logicReasoning::unify(const TL::Literal* grounded_lit, const TL::Literal* other_lit, TL::Substitution& sub) {
  CHECK(grounded_lit->positive, "We only unify with positives, my friend!");
  if (grounded_lit->positive != other_lit->positive)
    return false;
  return unify(grounded_lit->atom, other_lit->atom, sub);
}


bool TL::logicReasoning::unify(TL::FunctionValue* grounded_fv, TL::ComparisonLiteral* clit_constant, TL::Substitution& sub) {
  TL::ComparisonAtom* ca = (ComparisonAtom*) clit_constant->atom;
  CHECK(ca->hasConstantBound(), "this unify deals only with constant-bound cpts")
  CHECK(isGrounded(grounded_fv), "Dis functionValue ain't grounded!")
	
  if (grounded_fv->atom->f != ca->fa1->f)
    return false;
  // check whether comparison would fit at all
  if (!TL::compare(grounded_fv->value, ca->bound, ca->comparisonType))
    return false;
    
  // check whether unification possible
  return unify_basic(grounded_fv->atom->args, ((ComparisonAtom*)clit_constant->atom)->fa1->args, sub);
}




bool TL::logicReasoning::unify(TL::FunctionValue* grounded_fv_left, TL::FunctionValue* grounded_fv_right, TL::ComparisonLiteral* clit_dynamic, TL::Substitution& sub) {
  uint DEBUG = 0;
  ComparisonAtom* ca = (ComparisonAtom*) clit_dynamic;
  CHECK(isGrounded(grounded_fv_left), "Dis functionValue ain't grounded!")
  CHECK(isGrounded(grounded_fv_right), "Dis functionValue ain't grounded!")
  CHECK(!ca->hasConstantBound(), "Dis clit ain't dynamic-bounded!")
  if (grounded_fv_left->atom->f != ca->fa1->f)
    return false;
  if (grounded_fv_right->atom->f != ca->fa2->f)
    return false;
  // check whether comparison would fit at all
  if (TL::compare(grounded_fv_left->value, grounded_fv_right->value, ca->comparisonType))
    return false;
  // check whether unification possible
  uintA args_left, args_right;
  uint i;
  for(i=0;i<((ComparisonAtom*)clit_dynamic->atom)->args.d0;i++) {
    if (i<((ComparisonAtom*)clit_dynamic->atom)->args.d0/2)
      args_left.append(((ComparisonAtom*)clit_dynamic->atom)->args(i));
    else
      args_right.append(((ComparisonAtom*)clit_dynamic->atom)->args(i));
  }
  if (DEBUG>0) {cout<<"Init sub: ";sub.write(cout);cout<<endl; PRINT(args_left) PRINT(args_right)}
  
  if (grounded_fv_left->atom->args.N != args_left.N) {
    cout<<"logicReasoning::unify(TL::FunctionValue* grounded_fv_left, TL::FunctionValue* grounded_fv_right, TL::ComparisonLiteral* clit_dynamic, TL::Substitution& sub)"<<endl;
    cout<<"grounded_fv_left: ";grounded_fv_left->write(cout);cout<<endl;
    cout<<"grounded_fv_right: ";grounded_fv_right->write(cout);cout<<endl;
    cout<<"clit_dynamic: ";clit_dynamic->write(cout);cout<<endl;
    PRINT(args_left);
    sub.write(cout);
  }
  
  if (unify_basic(grounded_fv_left->atom->args, args_left, sub)) { 
    if (grounded_fv_right->atom->args.N != args_right.N) {
      cout<<"logicReasoning::unify(TL::FunctionValue* grounded_fv_left, TL::FunctionValue* grounded_fv_right, TL::ComparisonLiteral* clit_dynamic, TL::Substitution& sub)"<<endl;
      cout<<"grounded_fv_right: ";grounded_fv_right->write(cout);cout<<endl;
      cout<<"clit_dynamic: ";clit_dynamic->write(cout);cout<<endl;
      PRINT(args_right);
      sub.write(cout);
    }
    if (unify_basic(grounded_fv_right->atom->args, args_right, sub)) {
      return true;
    }
    else
      return false;
  }
  else
    return false;
}



// BASIC COVERAGE
// FOR ALL
bool TL::logicReasoning::cover(const TL::State& s, TL::Literal* lit, TL::SubstitutionSet& subs, bool freeNegVarsAllQuantified, TL::Substitution* initSub) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"cover - basic [START]"<<endl;

  CHECK(subs.num()==0, "Already subs given, Aldaaaaaa!")
  
  if (DEBUG>1) {
    cout << "State: "; s.write(cout); cout<<endl;
    cout << "lit: "; lit->write(cout); cout<<endl;
    PRINT(freeNegVarsAllQuantified)
    cout << "initSub: "; if (initSub==NULL) cout<<" NULL"; else initSub->write(cout); cout<<endl;
  }
    
  // ------------------------------------------------------------
  // Grounded after initSub
  // ASSUMPTION: states contain only positive Literals
  // determine grounded pred tuple as determined by initSub
  TL::Literal* init_grounded_lit;
  if (initSub != NULL)
    init_grounded_lit = applyOriginalSub(*initSub, lit);
  else
    init_grounded_lit = lit;
  if (isGrounded(init_grounded_lit)) {
    if (DEBUG>1) cout<<"lit is grounded after initSub:  init_grounded_lit="<<*init_grounded_lit<<endl;
    if (init_grounded_lit->positive) {
      if (holds(s, init_grounded_lit)) {
        TL::Substitution* s = new TL::Substitution;
        if (initSub != NULL) *s = *initSub;
        subs.append(s);
      }
    }
    else {
      TL::Literal* init_grounded_lit_pos;
      init_grounded_lit_pos = TL::logicObjectManager::getLiteralNeg(init_grounded_lit);
      if (!holds(s, init_grounded_lit_pos)) {
        TL::Substitution* s = new TL::Substitution;
        if (initSub != NULL) *s = *initSub;
        subs.append(s);
      }
    }
    if (DEBUG>1) cout<<"covered? " << (subs.num()>0) << endl;
    if (DEBUG>0) cout<<"cover - basic [END]"<<endl;
    if (subs.num() > 0)
        return true;
    else
        return false;
  }
    
      
    
  // ------------------------------------------------------------
  // Non-grounded after initSub
  if (DEBUG>0) {cout<<"non-grounded after initSub"<<endl;}
  uint i;
  // ------------------------------------------------------------
  // Positive
  if (lit->positive) {
    TL::Substitution* sub1;
    bool literalTrue;
    if (lit->atom->pred->category == category_derived) {
      CHECK(s.derivedDerived, "calculate derived predicates first")
      FOR1D(s.lits_derived, i) {
        sub1 = new TL::Substitution;
        if (initSub != NULL) *sub1 = *initSub;
        literalTrue = unify(s.lits_derived(i), lit, *sub1);
        if (literalTrue) subs.append(sub1);
        else delete sub1;
      }
    }
    else if (lit->atom->pred->type == TL::Predicate::predicate_simple) {
      FOR1D(s.lits_prim, i) {
        sub1 = new TL::Substitution;
        if (initSub != NULL) *sub1 = *initSub;
        literalTrue = unify(s.lits_prim(i), lit, *sub1);
        if (literalTrue) subs.append(sub1);
        else delete sub1;
      }
    }
    else if (lit->atom->pred->type == TL::Predicate::predicate_comparison) {
      TL::ComparisonLiteral* clit = static_cast<TL::ComparisonLiteral*>(lit);
      CHECK(clit!=NULL, "Maldefined comparison predicate")
      if (((ComparisonAtom*)clit->atom)->hasConstantBound()) {
        if (DEBUG>1) cout<<"Coverage of constant clit [START]"<<endl;
        if (((ComparisonAtom*)clit->atom)->fa1->f->category == category_primitive) {
          FOR1D(s.fv_prim, i) {
            sub1 = new TL::Substitution;
            if (initSub != NULL) *sub1 = *initSub;
            literalTrue = unify(s.fv_prim(i), clit, *sub1);
            if (literalTrue) subs.append(sub1);
            else delete sub1;
          }
        }
        else {
          FOR1D(s.fv_derived, i) {
            sub1 = new TL::Substitution;
            if (initSub != NULL) *sub1 = *initSub;
            literalTrue = unify(s.fv_derived(i), clit, *sub1);
            if (literalTrue) subs.append(sub1);
            else delete sub1;
          }
        }
      }
      // dynamic CLIT
      else {
        if (DEBUG>1) cout<<"Coverage of dynamic clit [START]"<<endl;
        if (DEBUG>1) {cout<<"Checking coverage of  "<<*lit<<"  with init sub  ("; 
                        initSub->write(cout); cout<<")  in  "; write(s.fv_prim);
                        write(s.fv_derived); cout<<"."<<endl;}
        uint l, r;
        uintA args_left, args_right;
        for(l=0;l<clit->atom->args.d0;l++) {
          if (l<clit->atom->args.d0/2)
            args_left.append(clit->atom->args(l));
          else
            args_right.append(clit->atom->args(l));
        }
        if (DEBUG>0) {PRINT(args_left) PRINT(args_right)}
        FuncVL allValues;
        allValues.append(s.fv_prim);
        allValues.append(s.fv_derived);
        // checking left side
        FOR1D(allValues, l) {
          if (allValues(l)->atom->f != ((ComparisonAtom*)clit->atom)->fa1->f)
            continue;
          TL::Substitution sub_left;
          sub_left = *initSub;
            
          if (allValues(l)->atom->args.N != args_left.N) {
            cout<<"logicReasoning::cover(TL::State* s, TL::Literal* lit, TL::SubstitutionSet& subs, bool freeNegVarsAllQuantified, TL::Substitution* initSub)"<<endl;
            cout<<"allValues(l): ";allValues(l)->write(cout);cout<<endl;
            cout<<"clit: ";clit->write(cout);cout<<endl;
            sub_left.write(cout);
          }
            
          if (unify_basic(allValues(l)->atom->args, args_left, sub_left)) {
            // checking right side
            FOR1D(allValues, r) {
              if (allValues(r)->atom->f != ((ComparisonAtom*)clit->atom)->fa1->f)
                  continue;
              TL::Substitution sub_right;
              sub_right = sub_left;
                  
              if (allValues(r)->atom->args.N != args_right.N) {
                  cout<<"logicReasoning::cover(TL::State* s, TL::Literal* lit, TL::SubstitutionSet& subs, bool freeNegVarsAllQuantified, TL::Substitution* initSub)"<<endl;
                  cout<<"allValues(r): ";allValues(r)->write(cout);cout<<endl;
                  cout<<"clit: ";clit->write(cout);cout<<endl;
                  sub_right.write(cout);
              }
                  
              if (unify_basic(allValues(r)->atom->args, args_right, sub_right)) {
                if (TL::compare(allValues(l)->value, allValues(r)->value, ((ComparisonAtom*)clit->atom)->comparisonType)) {
                  sub1 = new TL::Substitution;
                  *sub1 = sub_right;
                  subs.append(sub1);
                  if (DEBUG>1) {cout<<"Possible substitution: ";subs.last()->write(cout);cout<<"   since left: ";allValues(l)->write(cout);cout<<"  and right: ";allValues(r)->write(cout);cout<<endl;}
                }
                else {
                  if (DEBUG>1) {
                    cout<<"Subs "; sub_right.write(cout); cout<<" failing due to comparison."<<endl;
                  }
                }
              }
            }
          }
        }
        if (DEBUG>1) {
          FOR1D_(subs, r) {
            subs.elem(r)->write(cout); cout<<endl;
          }
        }
        if (DEBUG>1) cout<<"Coverage of dynamic clit [END]"<<endl;
      }
    }
    else {
      cerr<<"A crazy reference predicate indeed in lit "<<*lit<<endl;
      lit->atom->pred->writeNice(cerr);  cerr<<endl;
      HALT("");
    }
  }
  // ------------------------------------------------------------
  // Negative
  else {
    if (DEBUG>0) {cout<<"Negative"<<endl;}
    // ------------------------------------------------------------
    // All-quantified
    //
    // Algorithm checks that the positive version cannot be unified with state.
    if (freeNegVarsAllQuantified) {
      if (DEBUG>0) {cout<<"All quantification"<<endl;}
      bool literalTrue = false;
      uint i;
      TL::Substitution* sub1;
      TL::Literal* lit_pos;
      lit_pos = TL::logicObjectManager::getLiteralNeg(lit);
      bool heldOnce = false;
      if (lit->atom->pred->type == TL::Predicate::predicate_simple) {
        FOR1D(s.lits_prim, i) {
          sub1 = new TL::Substitution;
          if (initSub != NULL) *sub1 = *initSub;
          literalTrue = unify(s.lits_prim(i), lit_pos, *sub1);
          delete sub1;
          if (literalTrue) heldOnce = true;
        }
      }
      else if (lit->atom->pred->type == TL::Predicate::predicate_conjunction) {
        CHECK(s.derivedDerived, "derive TL::logicObjectManager::p_derived predicates first")
        FOR1D(s.lits_derived, i) {
          sub1 = new TL::Substitution;
          if (initSub != NULL) *sub1 = *initSub;
          literalTrue = unify(s.lits_derived(i), lit_pos, *sub1);
          delete sub1;
          if (literalTrue) heldOnce = true;
        }
      }
      else if (lit->atom->pred->type == TL::Predicate::predicate_comparison) {
        HALT("Comparison Predicate Tuples should never be used in negated form (so far)");
        NIY
      }
      // If positive predicate tuple never held, the negated version is true.
      // We add the init substitution or an empty if the first does not exist.
      if (DEBUG>0) PRINT(heldOnce);
      if (!heldOnce) {
          TL::Substitution* sub2 = new TL::Substitution;
          if (initSub != NULL)
              *sub2 = *initSub;
          subs.append(sub2);
      }
    }
    // ------------------------------------------------------------
    // Existential-quantified
    // 
    // Algorithm tries all possible substitutions.
    else {
      if (DEBUG>0) {cout<<"Exists quantification"<<endl;}
      // determine grounded pred tuple as determined by initSub
      TL::Literal* init_grounded_lit;
      init_grounded_lit = applyOriginalSub(*initSub, lit);
      // determine free variables
      uintA freeVars;
      FOR1D(init_grounded_lit->atom->args, i) {
        if (!isConstant(init_grounded_lit->atom->args(i)))
          freeVars.setAppend(init_grounded_lit->atom->args(i));
      }
      // create all possible substitutions
      TL::SubstitutionSet cand_subs;
      createAllPossibleSubstitutions(freeVars, cand_subs);
      // check which lead to coverage by creating positive tuples and unifying them with the given state
      TL::Literal* grounded_lit;
      TL::Literal* grounded_lit_pos;
      // filter out such constants which don't appear in the state
      uintA state_constants;
      logicReasoning::getConstants(s, state_constants);
      FOR1D_(cand_subs, i) {
        uintA outs_distinct;  cand_subs.elem(i)->getOutsDistinct(outs_distinct);
        if (DEBUG>1) {cout<<"cand_subs.elem(i="<<i<<"):  ";  cand_subs.elem(i)->write();  cout<<endl;  PRINT(outs_distinct);}
        if (numberSharedElements(outs_distinct, state_constants) != outs_distinct.N) {
          if (DEBUG>1) {cout<<" -> invalid cand_subs"<<endl;}
          continue;
        }
        grounded_lit = applyOriginalSub(*cand_subs.elem(i), init_grounded_lit);
        grounded_lit_pos = TL::logicObjectManager::getLiteralNeg(grounded_lit);
        // If positive version of grounded literal does not hold,
        // then its negation _does_ hold.
        if (!holds(s, grounded_lit_pos)) {
          subs.append(TL::Substitution::combine(*initSub, *cand_subs.elem(i)));
          if (DEBUG>1) {cout<<" -> WORKS (positive PI does not hold!)"<<endl;}
        }
      }
    }
  }
    

  if (DEBUG>1) {
    if (subs.num()==0) cout << "Not covered." << endl;
    else {
      cout <<"Covered by the following substitutions:"<<endl;
      FOR1D_(subs, i) {
        subs.elem(i)->write(cout); cout<<endl;
      }
    }
  }
  if (DEBUG>0) cout<<"cover - basic [END]"<<endl;
  return subs.num() > 0;
}








bool TL::logicReasoning::cover(const TL::State& s, const LitL& lits, TL::SubstitutionSet& subs, bool freeNegVarsAllQuantified, TL::Substitution* initSub) {
  int DEBUG = 0;

  if (DEBUG>0) cout<<"cover [START]"<<endl;
  if (DEBUG > 0) {
    cout << "lits: "; write(lits); cout << endl;
    cout << "State: ";
    s.write(cout);
    cout << endl;
  }
  
//   orderNegativesUngroundedLast(lits);
  CHECK(negativesUngroundedLast(lits), "Positive literals need to be first!");
  CHECK(subs.num()==0, "Already subs given, Aldaaaaaa!");

  TL::Substitution* initSub_copy = new TL::Substitution;
  if (initSub != NULL)
    *initSub_copy = *initSub;
  subs.append(initSub_copy);

  if (DEBUG > 0) {
    cout << "initSub: ";
    if (initSub != NULL) {initSub->write(cout); cout<<endl;}
    else {cout<<"-"<<endl;}
  }

  uint i, j, k;
  FOR1D(lits, i) {
    TL::SubstitutionSet all_next_subs;
    if (DEBUG > 0) {
      cout << "Inspecting lit #"<<i<<": ";
      lits(i)->write(cout);
      cout << endl;
    }
    FOR1D_(subs, j) {
      TL::SubstitutionSet next_subs;
      if (cover(s, lits(i), next_subs, freeNegVarsAllQuantified, subs.elem(j))) {
        FOR1D_(next_subs, k) {
          all_next_subs.append(next_subs.elem(k));
        }
      }
    }
        
    subs = all_next_subs;
    // if no more subs found = if no more coverage
    if (all_next_subs.num() == 0) {
      break;
    }
    if (DEBUG > 2) {
      cout << "After lit #"<<i<<": "; PRINT(subs.num())
      FOR1D_(subs, j) {
        subs.elem(j)->write(cout); cout << endl;
      }
    }
  }
  
  if (DEBUG>0) {
    cout << "Covered: " << (subs.num() > 0) << endl;
  }
  if (DEBUG>0) cout<<"cover [END]"<<endl;
  return subs.num() > 0;
}





// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    STATE UNIFICATION
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


void TL::logicReasoning::calcDifferences(LitL& lits_diff_1to2, FuncVL& fv_diff_1to2, LitL& lits_diff_2to1, FuncVL& fv_diff_2to1, const TL::State state1, const TL::State state2) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calcDifferences [START]"<<endl;}
  
  if (DEBUG>0) {
    cout<<"State 1:"<<endl;  state1.write();  cout<<endl;
    cout<<"State 2:"<<endl;  state2.write();  cout<<endl;
  }
  
  lits_diff_1to2.clear();
  fv_diff_1to2.clear();
  lits_diff_2to1.clear();
  fv_diff_2to1.clear();
  
  uint i;
  FOR1D(state1.lits_prim, i) {
    if (state2.lits_prim.findValue(state1.lits_prim(i)) < 0)
      lits_diff_1to2.append(state1.lits_prim(i));
  }
  FOR1D(state2.lits_prim, i) {
    if (state1.lits_prim.findValue(state2.lits_prim(i)) < 0)
      lits_diff_2to1.append(state2.lits_prim(i));
  }
  
  FOR1D(state1.fv_prim, i) {
    if (state2.fv_prim.findValue(state1.fv_prim(i)) < 0)
      fv_diff_1to2.append(state1.fv_prim(i));
  }
  FOR1D(state2.fv_prim, i) {
    if (state1.fv_prim.findValue(state2.fv_prim(i)) < 0)
      fv_diff_2to1.append(state2.fv_prim(i));
  }
  
  if (DEBUG>0) {
    cout<<"lits_diff_1to2:    "; TL::write(lits_diff_1to2); cout<<endl;
    cout<<"lits_diff_2to1:    "; TL::write(lits_diff_2to1); cout<<endl;
    cout<<"fv_diff_1to2:    "; TL::write(fv_diff_1to2); cout<<endl;
    cout<<"fv_diff_2to1:    "; TL::write(fv_diff_2to1); cout<<endl;
  }
  
  if (DEBUG>0) {cout<<"calcDifferences [END]"<<endl;}
}



uint TL::logicReasoning::createLeastGeneralSuperState(const TL::Atom& action1, const TL::State& state1, 
                                                   const TL::Atom& action2, const TL::State& state2) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"createLeastGeneralSuperState [START]"<<endl;}
  
  if (DEBUG>0) {
    cout << "Action 1:  "<<action1<<endl;
    cout << "State 1:   ";  state1.write(cout, true);  cout<<endl;
    cout << "Action 2:  "<<action1<<endl;
    cout << "State 2:   ";  state2.write(cout, true);  cout<<endl;
  }
  
  if (action1.pred != action2.pred)
    HALT("action have different types");
  
  uint i;
  TL::Substitution sub;
  CHECK(action1.args.N == 1, "");
  FOR1D(action1.args, i) {
    sub.addSubs(action1.args(1), i);
  }
    NIY;
    
  if (DEBUG>0) {cout<<"createLeastGeneralSuperState [END]"<<endl;}
  
  return 5;
}






uint TL::logicReasoning::unifyAsMuchAsPossible(SubstitutionSet& subs, const TL::State& state1, const TL::State& state2, TL::Substitution* initSub) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"unifyAsMuchAsPossible [START]"<<endl;}
  
  if (DEBUG>0) {
    cout << "State 1:   "; state1.write(cout, true);  cout<<endl;
    cout << "State 2:   "; state2.write(cout, true);  cout<<endl;
    cout<<"initSub:  "<<flush; if (initSub == NULL) cout<<"NULL"<<endl;  else initSub->write();  cout<<endl;
  }
  
  // (1) Calc Differences (with initSub)
//   TL::State* state1_initSubbed = initSub->apply(state1);
//   makeOriginal(*state1_initSubbed);
  LitL lits_diff_1to2, lits_diff_2to1;
  FuncVL fv_diff_1to2, fv_diff_2to1;
  calcDifferences(lits_diff_1to2, fv_diff_1to2, lits_diff_2to1, fv_diff_2to1, state1, state2);
  if (DEBUG>0) {
//     cout << "State 1 (with init-sub):  "; state1_initSubbed->writeNice(cout, false, true);  cout<<endl;
    cout<<"Differences:"<<endl;
    cout<<"lits_diff_1to2:    "; TL::write(lits_diff_1to2); cout<<endl;
    cout<<"fv_diff_1to2:    "; TL::write(fv_diff_1to2); cout<<endl;
    cout<<"lits_diff_2to1:    "; TL::write(lits_diff_2to1); cout<<endl;
    cout<<"fv_diff_2to1:    "; TL::write(fv_diff_2to1); cout<<endl;
  }
//   delete state1_initSubbed;
  
  
  uint i, k, l;
  
  
  // (2) Determine substitutions leading from state1 to state2
  uintA in_ids, out_ids;
  SubstitutionSet potential_subs;
  if (initSub == NULL)
    potential_subs.append(new TL::Substitution);
  else {
    potential_subs.append(initSub);
    initSub->getIns(in_ids);
    initSub->getIns(out_ids);
  }
  
  if (DEBUG>0) {PRINT(in_ids);}
  
  // (2.1) LOOK AT UNARIES
  // LitL
  FOR1D(lits_diff_1to2, i) {
    if (lits_diff_1to2(i)->atom->pred->d == 1   &&  in_ids.findValue(lits_diff_1to2(i)->atom->args(0)) < 0) {
      uintA possibleSubstitutes;
      FOR1D(lits_diff_2to1, k) {
        if (lits_diff_1to2(i)->atom->pred == lits_diff_2to1(k)->atom->pred    // same predicate
//                &&    substituted_ids.findValue(lits_diff_2to1(k)->atom->args(0)) < 0      // substituting-id not already used
               ){
          possibleSubstitutes.append(lits_diff_2to1(k)->atom->args(0));
        }
      }
      FOR1D(possibleSubstitutes, k) {
        FOR1D_(potential_subs, l) {
          potential_subs.elem(l)->addSubs(lits_diff_1to2(i)->atom->args(0), possibleSubstitutes(k));
          in_ids.setAppend(lits_diff_1to2(i)->atom->args(0));
        }
      }
    }
  }
  #if 0 // ignore FVs
  // FVs
  FOR1D(fv_diff_1to2, i) {
    if (fv_diff_1to2(i)->f->d == 1   &&  in_ids.findValue(fv_diff_1to2(i)->args(0)) < 0) {
      uintA possibleSubstitutes;
      FOR1D(fv_diff_2to1, k) {
        if (fv_diff_1to2(i)->f == fv_diff_2to1(k)->f    // same function
//                &&   substituted_ids.findValue(fv_diff_2to1(k)->args(0)) < 0  // substituting-id not already used
        ) {
          possibleSubstitutes.append(fv_diff_2to1(k)->args(0));
        }
      }
      FOR1D(possibleSubstitutes, k) {
        FOR1D_(potential_subs, l) {
          potential_subs.elem(l)->addSubs(fv_diff_1to2(i)->args(0), possibleSubstitutes(k));
          in_ids.setAppend(fv_diff_1to2(i)->args(0));
        }
      }
    }
  }
  #endif
  
  // (2.2) LOOK AT BINARIES
  FOR1D(lits_diff_1to2, i) {
    if (lits_diff_1to2(i)->atom->pred->d == 2) {
      bool first_arg_covered = in_ids.findValue(lits_diff_1to2(i)->atom->args(0)) >= 0;
      bool second_arg_covered = in_ids.findValue(lits_diff_1to2(i)->atom->args(1)) >= 0;
      if (DEBUG>2) {lits_diff_1to2(i)->write();  cout<<"  "<<first_arg_covered<<"  "<<second_arg_covered<<endl;}
      if (!first_arg_covered) {
        uintA possibleSubstitutes;
        FOR1D(lits_diff_2to1, k) {
          if (lits_diff_1to2(i)->atom->pred == lits_diff_2to1(k)->atom->pred     // same predicate
//                 &&   substituted_ids.findValue(lits_diff_2to1(k)->atom->args(0)) < 0         // substituting-id not already used
                &&   lits_diff_1to2(i)->atom->args(0) != lits_diff_2to1(k)->atom->args(0)) {  // different arguments --> true substitution
            possibleSubstitutes.append(lits_diff_2to1(k)->atom->args(0));
          }
        }
        if (DEBUG>2) {cout<<"1:  "<<possibleSubstitutes<<endl;}
        FOR1D(possibleSubstitutes, k) {
          FOR1D_(potential_subs, l) {
            potential_subs.elem(l)->addSubs(lits_diff_1to2(i)->atom->args(0), possibleSubstitutes(k));
            in_ids.setAppend(lits_diff_1to2(i)->atom->args(0));
          }
        }
      }
      if (!second_arg_covered) {
        uintA possibleSubstitutes;
        FOR1D(lits_diff_2to1, k) {
          if (lits_diff_1to2(i)->atom->pred == lits_diff_2to1(k)->atom->pred     // same predicate
//                 &&   substituted_ids.findValue(lits_diff_2to1(k)->atom->args(1)) < 0         // substituting-id not already used
                &&   lits_diff_1to2(i)->atom->args(1) != lits_diff_2to1(k)->atom->args(1)) {  // different arguments --> true substitution
            possibleSubstitutes.append(lits_diff_2to1(k)->atom->args(1));
          }
        }
        if (DEBUG>2) {cout<<"2:  "<<possibleSubstitutes<<endl;}
        FOR1D(possibleSubstitutes, k) {
          FOR1D_(potential_subs, l) {
            potential_subs.elem(l)->addSubs(lits_diff_1to2(i)->atom->args(1), possibleSubstitutes(k));
            in_ids.setAppend(lits_diff_1to2(i)->atom->args(1));
          }
        }
      }
    }
  }
  
  if (DEBUG>0) {
    PRINT(in_ids);
  }
  
  
  // (3) RESUBSTITUTE ALL VARS which are not covered yet; e.g. falls nur 71-->70 bisher, fuehere dann auch 70-->71 ein
  FOR1D_(potential_subs, i) {
    if (DEBUG>0) {cout<<"Completing potential substitution: ";  potential_subs.elem(i)->write();  cout<<endl;}
//     CHECK(potential_subs.elem(i)->mapsToDistinct(), "should map to distinct");
    if (!potential_subs.elem(i)->mapsToDistinct())
      continue;
    
    uintA ins, outs;
    potential_subs.elem(i)->getIns(ins);
    potential_subs.elem(i)->getOuts(outs);
    uintA unsubstituted_outs;
    FOR1D(outs, k) {
      int idx = ins.findValue(outs(k));
      if (idx < 0) {
        unsubstituted_outs.append(outs(k));
      }
    }
    
    if (DEBUG>0) {PRINT(unsubstituted_outs);}
    while (unsubstituted_outs.N > 0) {
      potential_subs.elem(i)->getIns(ins);
      potential_subs.elem(i)->getOuts(outs);
      uintA free_ins;  // for backward substitution
      FOR1D(ins, k) {
        int idx = outs.findValue(ins(k));
        if (idx < 0) {
          free_ins.append(ins(k));
        }
      }
      CHECK(free_ins.N <= unsubstituted_outs.N, "");
      if (DEBUG>0) {PRINT(free_ins);}
      FOR1D_DOWN(free_ins, k) {
        potential_subs.elem(i)->addSubs(unsubstituted_outs(k), free_ins(k));
        unsubstituted_outs.remove(k);
      }
    }
    if (DEBUG>0) {
      cout<<"Final potential substitution: ";  potential_subs.elem(i)->write();  cout<<endl;
    }
  }
  
  
  // (4) TRY OUT POTENTIAL SUBS
  uint minDiff = 10000; // over all subsitutions
  FOR1D_(potential_subs, i) {
    // "potential_state2 should be state2
    TL::State* potential_state2 = potential_subs.elem(i)->apply(state1);
    TL::logicObjectManager::makeOriginal(*potential_state2);
    if (DEBUG>0) {
      cout<<"Potential sub #" << i << ":"<<endl;
      potential_subs.elem(i)->write(cout);  cout<<endl;
      cout<<"Potential state 2 (built from state 1 using the substitution):"<<endl; potential_state2->write(cout, true); cout<<endl;
    }
    if (*potential_state2 == state2) {
      subs.append(potential_subs.elem(i));
      if (DEBUG>0) {cout<<"FITS!"<<endl;}
    }
    
    LitL local__pi_diff_1to2, local__pi_diff_2to1;
    FuncVL local__fv_diff_1to2, local__fv_diff_2to1;
    calcDifferences(local__pi_diff_1to2, local__fv_diff_1to2, local__pi_diff_2to1, local__fv_diff_2to1, *potential_state2, state2);
    uint diff = local__pi_diff_1to2.N  +  local__fv_diff_1to2.N  +  local__pi_diff_2to1.N  +  local__fv_diff_2to1.N;
    
    if (DEBUG>0) {
      cout<<"Differences between potential state 2 and true state 2:"<<endl;
      cout<<"local__pi_diff_1to2:    "; TL::write(local__pi_diff_1to2); cout<<endl;
      cout<<"local__fv_diff_1to2:    "; TL::write(local__fv_diff_1to2); cout<<endl;
      cout<<"local__pi_diff_2to1:    "; TL::write(local__pi_diff_2to1); cout<<endl;
      cout<<"local__fv_diff_2to1:    "; TL::write(local__fv_diff_2to1); cout<<endl;
      PRINT(diff);
    }
    
    minDiff = TL_MIN(diff,  minDiff);
  }
  
  
  if (DEBUG>0) {
    cout<<"Final subs:"<<endl;
    FOR1D_(subs, i) {
      subs.elem(i)->write(cout);  cout<<endl;
    }
    PRINT(minDiff);
  }
  
  if (DEBUG>0) {cout<<"unifyAsMuchAsPossible [END]"<<endl;}
  
  return minDiff;
}







bool TL::logicReasoning::unify(SubstitutionSet& subs, const TL::State& state1, const TL::State& state2, TL::Substitution* initSub) {
  uint minDiff = unifyAsMuchAsPossible(subs, state1, state2, initSub);
  return minDiff == 0;
}


bool TL::logicReasoning::unifiable(const TL::State& state1, const TL::State& state2) {
  SubstitutionSet subs;
  return unify(subs, state1, state2);
}


uint TL::logicReasoning::unifyAsMuchAsPossible(SubstitutionSet& subs,
                                const TL::State& state1, const TL::Atom& action1,
                                const TL::State& state2, const TL::Atom& action2) {
  TL::Substitution* action_sub = new TL::Substitution; // don't delete
  CHECK(action1.pred == action2.pred, "Try to unify nicht-zusammenpassende predicates:  " << action2.pred->id << " vs " << action1.pred);
  uint i;
  FOR1D(action1.args, i) {
    action_sub->addSubs(action1.args(i), action2.args(i));
  }
  return unifyAsMuchAsPossible(subs, state1, state2, action_sub);
}



// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


// abstracting

void TL::logicReasoning::createInverseSubstitution(const TL::Atom& a, TL::Substitution& sub) {
  CHECK(sub.empty(), "Sub not empty");
  uint i;
  FOR1D(a.args, i) {
    if (isConstant(a.args(i)) && !sub.hasSubs(a.args(i))) {
      sub.addSubs2Variable(a.args(i));
    }
  }
}

void TL::logicReasoning::createInverseSubstitution(const TL::Literal& lit, TL::Substitution& sub) {
  return createInverseSubstitution(*lit.atom, sub);
}


// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    G R O U N D I N G S
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


bool TL::logicReasoning::isConstant(uint obj) {
  CHECK(TL::logicObjectManager::constants.N > 0, "TL::logicObjectManager::constants have not been set!");
  return obj >= TL::logicObjectManager::constants.min();  // Die Objekte koennen sich inzwischen aendern
}


bool TL::logicReasoning::isGrounded(const TL::Atom* a) {
  uint i;
  if (a->pred->type == TL::Predicate::predicate_comparison) {
    if (!isGrounded(((ComparisonAtom*)a)->fa1)) return false;
    if (((ComparisonAtom*)a)->fa2 != NULL  &&  !isGrounded(((ComparisonAtom*)a)->fa2)) return false;
    return true;
  }
  else {
    FOR1D(a->args, i) {
      if (!isConstant(a->args(i)))
        return false;
    }
    return true;
  }
}


bool TL::logicReasoning::isGrounded(const TL::Literal* lit) {
  return isGrounded(lit->atom);
}


bool TL::logicReasoning::isGrounded(const TL::FunctionAtom* fa) {
  uint i;
  FOR1D(fa->args, i) {
    if (!isConstant(fa->args(i)))
      return false;
  }
  return true;
}


bool TL::logicReasoning::isGrounded(const TL::FunctionValue* fv) {
  return isGrounded(fv->atom);
}


bool TL::logicReasoning::isAbstract(const TL::Atom* a) {
  uint i;
  FOR1D(a->args, i)
    if (isConstant(a->args(i)))
      return false;
  return true;
}

bool TL::logicReasoning::isAbstract(const TL::Literal* lit) {
  return isAbstract(lit->atom);
}









// derives all ConjunctionPredicates that hold in this state (at the current moment)
bool TL::logicReasoning::deriveLiterals_conjunction(TL::ConjunctionPredicate& p, TL::State& s) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"deriveLiterals_conjunction [START]"<<endl;}
//     double t_start = MT::cpuTime();
  if (DEBUG>1) {cout<<"ConjunctionPredicate: ";p.writeNice(cout);cout<<endl;cout<<"State: ";s.write(cout);cout<<endl;}
  uint i, j, k;
  // (1) create base predicate tuples = BPT
  LitL base_lits;
  k=0;
  FOR1D(p.basePreds, i) {
    uintA args1;
    for (j=0; j<p.basePreds(i)->d; j++) {
      args1.append(p.basePreds_mapVars2conjunction(k++));
    }
    TL::Literal* lit = TL::logicObjectManager::getLiteral(p.basePreds(i), p.basePreds_positive(i), args1);
    base_lits.append(lit);
  }
  sort(base_lits);
  if (DEBUG>1) {cout<<"Abstract base predicate tuples "; write(base_lits); cout<<endl;}
  // (2) check whether BPT hold
  bool newFound=false;
  // Free Vars EXISTENTIAL
  if (!p.freeVarsAllQuantified) {
    if (DEBUG>1) {cout<<"free vars EXISTS"<<endl;}
    // simply call cover
    TL::SubstitutionSet subs;
    if (cover(s, base_lits, subs, false)) {
      FOR1D_(subs, j) {
        if (p.d == 2  &&  !subs.elem(j)->mapsToDistinct())
          continue;
        TL::Literal* cpt_cand = TL::logicObjectManager::getConjunctionLiteral(&p, subs.elem(j));
        s.lits_derived.setAppend(cpt_cand);
        if (DEBUG>1) {cout<<"TL::logicObjectManager::p_derived "; s.lits_derived.last()->write(cout); cout<<endl;}
        newFound = true;
      }
    }
  }
  // Free Vars ALL
  else {
    if (DEBUG>1) {cout<<"free vars ALL"<<endl;}
    // We must treat positive vars with special care, since in cover(.) positives are always ex quantified.
    // calc free vars in base predicates
    uintA freeVars_pos, freeVars_neg;
    calcFreeVars(p, freeVars_pos, freeVars_neg);
    if (DEBUG>1) {PRINT(freeVars_pos);  PRINT(freeVars_neg);}
    // (1) Create possible argument-combos for ConjunctionPredicate.
    MT::Array< uintA > argument_lists;
    // only use constants which are provided in the state
    uintA state_constants ;
    getConstants(s, state_constants);
    TL::allPossibleLists(argument_lists, state_constants, p.d, false, true);
    if (DEBUG>0) {PRINT(argument_lists);}
    // (2) Test each possible argument-combo.
    FOR1D(argument_lists, i) {
      TL::Substitution s_arguments;
      FOR1D(argument_lists(i), j) {
        s_arguments.addSubs(j, argument_lists(i)(j));
      }
      // (2a) Create possible subs for free positive arguments.
      // --> ALL must hold.
      MT::Array< uintA > argument_freePos_lists;
      // here use indeed all available constants
      TL::allPossibleLists(argument_freePos_lists, state_constants, freeVars_pos.d0, true, true);
      FOR1D(argument_freePos_lists, j) {
        TL::Substitution s_arguments_inclFreePos;
        s_arguments_inclFreePos = s_arguments;
        FOR1D(freeVars_pos, k) {
          s_arguments_inclFreePos.addSubs(freeVars_pos(k), argument_freePos_lists(j)(k));
        }
        TL::SubstitutionSet subs_unneeded;
        if (!cover(s, base_lits, subs_unneeded, true, &s_arguments_inclFreePos))
          break;
      }
      // If all possible subs for freePos covered, then we can use the current s_arguments!
      if (argument_freePos_lists.d0 == j) {
          TL::Literal* cpt_cand = TL::logicObjectManager::getConjunctionLiteral(&p, &s_arguments);
        s.lits_derived.setAppend(cpt_cand);
        if (DEBUG>1) {cout<<"TL::logicObjectManager::p_derived "; s.lits_derived.last()->write(cout); cout<<endl;}
        newFound = true;
      }
    }
  }
//     double t_finish = MT::cpuTime();
//     cout<<"deriveLiterals_conjunction time = "<<(t_finish - t_start)<<endl;
  if (DEBUG>0) {cout<<"Derived state literals ["<<s.lits_derived.N<<"]: "<<s.lits_derived<<endl;}
  if (DEBUG>0) {cout<<"deriveLiterals_conjunction [END]"<<endl;}
  return newFound;
}


// assumes acyclicity!
// seems standard graph problem i don't know standard solution of
bool TL::logicReasoning::deriveLiterals_transClosure(TL::TransClosurePredicate& p, TL::State& s) {
    uint DEBUG = 0;
    if (DEBUG>0) {cout<<"deriveLiterals_transClosure [START]"<<endl;}
    if (DEBUG>0) {p.writeNice(); cout<<endl;}
    CHECK(p.d==2, "transitive closure defined only for binary preds")
    uint i;
    // (1) find edges
    std::map< uint, uintA > right;
    std::map< uint, uintA >::iterator iter;
    if (DEBUG>0) {cout<<"Given: ";}
    if (p.basePred->category == category_primitive) {
        if (p.basePred->type != TL::Predicate::predicate_comparison) {
            FOR1D(s.lits_prim, i) {
                if (s.lits_prim(i)->atom->pred->id == p.basePred->id) {
                    if (DEBUG>0) {s.lits_prim(i)->write(cout);cout<<" ";}
                    right[s.lits_prim(i)->atom->args(0)].setAppend(s.lits_prim(i)->atom->args(1));
                }
            }
        }
        else {
          NIY;
        }
    }
    else {
        FOR1D(s.lits_derived, i) {
            if (s.lits_derived(i)->atom->pred->id == p.basePred->id) {
                if (DEBUG>0) {s.lits_derived(i)->write(cout);cout<<" ";}
                right[s.lits_derived(i)->atom->args(0)].setAppend(s.lits_derived(i)->atom->args(1));
            }
        }
    }
    if (DEBUG>0) {cout<<endl;}
    
    if (DEBUG>1) {
        cout<<"Direct right neighbors:"<<endl;
        for (iter = right.begin(); iter != right.end(); iter++) {
            cout<<iter->first<<": "<<iter->second<<endl;
        }
    }
    if (right.empty())
        return false;
    // (2) build connections
    bool extended;
    uint no;
    do {
        extended = false;
        for (iter = right.begin(); iter != right.end(); iter++) {
            no = iter->second.d0;
            uintA newGuys_candidates;
            FOR1D(iter->second, i) {
                newGuys_candidates.setAppend(right[iter->second(i)]);
            }
            iter->second.setAppend(newGuys_candidates);
            if (iter->second.d0 > no)
                extended = true;
        }
    } while (extended);
    if (DEBUG>1) {
        cout<<"All right neighbors:"<<endl;
        for (iter = right.begin(); iter != right.end(); iter++) {
            cout<<iter->first<<": "<<iter->second<<endl;
        }
//         cout<<"All left neighbors:"<<endl;
//         for (iter = left.begin(); iter != left.end(); iter++) {
//             cout<<iter->first<<": "<<iter->second<<endl;
//         }
    }
    // (3) build TCP tuples
    if (DEBUG>0) {cout<<"TL::logicObjectManager::p_derived: ";}
    for (iter = right.begin(); iter != right.end(); iter++) {
        FOR1D(iter->second, i) {
            uintA args(2);
            args(0)=iter->first;
            args(1)=iter->second(i);
            TL::Literal* lit = TL::logicObjectManager::getTransClosureLiteral(&p, args);
            s.lits_derived.setAppend(lit);
            if (DEBUG>0) {lit->write(cout);cout<<" ";}
        }
    }
    if (DEBUG>0) {cout<<endl;}
    
    if (DEBUG>0) {cout<<"deriveLiterals_transClosure [END]"<<endl;}
    return true;
}



bool TL::logicReasoning::deriveLiterals_count(TL::CountPredicate& p, TL::State& s) {
    uint DEBUG=0;
    if (DEBUG>0) {cout<<"deriveLiterals_count [START]"<<endl;}
    if (DEBUG>0) {p.writeNice(cout);cout<<endl;}
    uint i, j;
    uint counter;
    MT::Array< uintA > possibleArgumentLists;
    TL::allPossibleLists(possibleArgumentLists, TL::logicObjectManager::constants, p.d, true, true);
    bool heldOnce = false;
    FOR1D(possibleArgumentLists, i) {
        if (DEBUG>1) {PRINT(possibleArgumentLists(i))}
        counter=0;
        uintA tester_args;
        tester_args = p.countedPred_mapVars2derived;
        FOR1D(tester_args, j) {
            if (tester_args(j) < p.d)
                tester_args(j) = possibleArgumentLists(i)(tester_args(j));
        }
        if (p.countedPred->category == category_primitive) {
            if (p.countedPred->type == TL::Predicate::predicate_simple) {
                TL::Literal* tester = TL::logicObjectManager::getLiteral(p.countedPred, true, tester_args);
                if (DEBUG>1) {cout<<"Tester: ";tester->write(cout);}
                FOR1D(s.lits_prim, j) {
                    TL::Substitution sub;
                    if (unify(s.lits_prim(j), tester, sub)) {
                        counter++;
                        if (DEBUG>1) {cout<<" 1";}
                    }
                }
            }
            else {
                // comp lits only created on demand...
                CHECK(dynamic_cast<TL::ComparisonPredicate*>(p.countedPred)!=NULL, "cast failed");
                CHECK(((TL::ComparisonPredicate*) p.countedPred)->constantBound, "dynamic bound NIY");
                TL::ComparisonLiteral* tester = TL::logicObjectManager::getCompLiteral_constant(p.comparison_f, p.comparison_compType, p.comparison_bound, tester_args);
                if (DEBUG>1) {cout<<"Tester: ";tester->write(cout);}
                if (((ComparisonAtom*)tester->atom)->fa1->f->category == category_primitive) {
                    FOR1D(s.fv_prim, j) {
                        if (DEBUG>1) {cout<<" ";s.fv_prim(j)->write(cout);}
                        TL::Substitution sub;
                        if (unify(s.fv_prim(j), tester, sub)) {
                            counter++;
                            if (DEBUG>1) {cout<<" 1";}
                        }
                    }
                }
                else {
                    FOR1D(s.fv_derived, j) {
                        if (DEBUG>1) {cout<<" ";s.fv_derived(j)->write(cout);}
                        TL::Substitution sub;
                        if (unify(s.fv_derived(j), tester, sub)) {
                            counter++;
                            if (DEBUG>1) {cout<<" 1";}
                        }
                    }
                }
            }
        }
        else {
            TL::Literal* tester = TL::logicObjectManager::getLiteral(p.countedPred, true, tester_args);
            if (DEBUG>1) {cout<<"Tester: ";tester->write(cout);}
            FOR1D(s.lits_derived, j) {
                TL::Substitution sub;
                if (unify(s.lits_derived(j), tester, sub)) {
                    counter++;
                    if (DEBUG>1) {cout<<" 1";}
                }
            }
        }
        if (DEBUG>1) {cout<<endl;}
        bool holds = TL::compare((int) counter, (int) p.bound, p.compType);
        if (holds) heldOnce=true;
        if (holds) {
            TL::Literal* lit = TL::logicObjectManager::getCountLiteral(&p, possibleArgumentLists(i));
            s.lits_derived.setAppend(lit);
            if (DEBUG>0) {cout<<"Found ";lit->write(cout);cout<<endl;}
        }
    }
    if (DEBUG>0) {cout<<"deriveLiterals_count [END]"<<endl;}
    return heldOnce;
}




bool TL::logicReasoning::deriveFunctionValues_count(TL::CountFunction& f, TL::State& s) {
    uint DEBUG = 0;
    if (DEBUG>0) {cout<<"deriveFunctionValues_count [START]"<<endl;}
    if (DEBUG>0) {f.writeNice();cout<<endl;}
    uint i, j;
    MT::Array< uintA > possibleArgumentLists;
    uintA local_constants;
    getConstants(s, local_constants);
    TL::allPossibleLists(possibleArgumentLists, local_constants, f.d, true, true);
    uint counter;
    FOR1D(possibleArgumentLists, i) {
        if (DEBUG>1) {PRINT(possibleArgumentLists(i))}
        counter=0;
        uintA tester_args;
        tester_args = f.countedPred_mapVars2derived;
        FOR1D(tester_args, j) {
            if (tester_args(j) < f.d)
                tester_args(j) = possibleArgumentLists(i)(tester_args(j));
        }
        TL::Literal* tester = TL::logicObjectManager::getLiteral(f.countedPred, true, tester_args);
        if (f.countedPred->category == category_primitive) {
          if (f.countedPred->type == TL::Predicate::predicate_comparison) {
            HALT("not implemented comparison predicates");
          }
          FOR1D(s.lits_prim, j) {
            TL::Substitution sub;
            if (unify(s.lits_prim(j), tester, sub)) {
              counter++;
            }
          }
        }
        else {
            FOR1D(s.lits_derived, j) {
                TL::Substitution sub;
                if (unify(s.lits_derived(j), tester, sub)) {
                    counter++;
                }
            }
        }
        // function value bauen
        TL::FunctionValue* fv = TL::logicObjectManager::getFV(&f, possibleArgumentLists(i), counter);
        if (DEBUG>0) {cout<<" ";fv->write(cout);}
        s.fv_derived.setAppend(fv);
    }
    if (DEBUG>0) {cout<<"deriveFunctionValues_count [END]"<<endl;}
    return true; // always holds
}


bool TL::logicReasoning::deriveFunctionValues_avg(TL::AverageFunction& f, TL::State& s) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"deriveFunctionValues_avg [START]"<<endl;}
  CHECK(f.d==0, "only implemented for zero-ary avg functions");
  uint i, k;
  MT::Array< uintA > combos;
  uintA local_constants;
  getConstants(s, local_constants);
  TL::allPossibleLists(combos, local_constants, f.f_base->d, true, true);
  double avg = 0.0;
  FOR1D(combos, i) {
    if (DEBUG>1) {PRINT(combos(i))}
    if (f.f_base->category == category_primitive) {
      FOR1D(s.fv_prim, k) {
        if (s.fv_prim(k)->atom->f == f.f_base  
            &&  s.fv_prim(k)->atom->args == combos(i)) {
          avg += s.fv_prim(k)->value;
          break;
        }
      }
    }
    else {
      FOR1D(s.fv_derived, k) {
        if (s.fv_derived(k)->atom->f == f.f_base  
            &&  s.fv_derived(k)->atom->args == combos(i)) {
          avg += s.fv_derived(k)->value;
          break;
        }
      }
    }
  }
  avg /= combos.N;
  // function value bauen
  uintA empty;
  TL::FunctionValue* fv = TL::logicObjectManager::getFV(&f, empty, avg);
  s.fv_derived.setAppend(fv);
  if (DEBUG>0) {cout<<" ";fv->write(cout);}
  if (DEBUG>0) {cout<<"deriveFunctionValues_avg [END]"<<endl;}
  return true; // always holds
}


bool TL::logicReasoning::deriveFunctionValues_sum(TL::SumFunction& f, TL::State& s) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"deriveFunctionValues_sum [START]"<<endl;}
  CHECK(f.d==0, "only implemented for zero-ary avg functions");
  uint i, k;
  MT::Array< uintA > combos;
  uintA local_constants;
  getConstants(s, local_constants);
  TL::allPossibleLists(combos, local_constants, f.f_base->d, true, true);
  double sum = 0.0;
  FOR1D(combos, i) {
    if (DEBUG>1) {PRINT(combos(i))}
    if (f.f_base->category == category_primitive) {
      FOR1D(s.fv_prim, k) {
        if (s.fv_prim(k)->atom->f == f.f_base  
            &&  s.fv_prim(k)->atom->args == combos(i)) {
          sum += s.fv_prim(k)->value;
          break;
        }
      }
    }
    else {
      FOR1D(s.fv_derived, k) {
        if (s.fv_derived(k)->atom->f == f.f_base  
            &&  s.fv_derived(k)->atom->args == combos(i)) {
          sum += s.fv_derived(k)->value;
          break;
        }
      }
    }
  }
  // function value bauen
  uintA empty;
  TL::FunctionValue* fv = TL::logicObjectManager::getFV(&f, empty, sum);
  s.fv_derived.setAppend(fv);
  if (DEBUG>0) {cout<<" ";fv->write(cout);}
  if (DEBUG>0) {cout<<"deriveFunctionValues_avg [END]"<<endl;}
  return true; // always holds
}



bool TL::logicReasoning::deriveFunctionValues_max(TL::MaxFunction& f, TL::State& s) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"deriveFunctionValues_max [START]"<<endl;}
  CHECK(f.d==0, "only implemented for zero-ary max functions");
  uint i, k;
  MT::Array< uintA > combos;
  uintA local_constants;
  getConstants(s, local_constants);
  TL::allPossibleLists(combos, local_constants, f.f_base->d, true, true);
  arr values;
  FOR1D(combos, i) {
    if (DEBUG>1) {PRINT(combos(i))}
    if (f.f_base->category == category_primitive) {
      FOR1D(s.fv_prim, k) {
        if (s.fv_prim(k)->atom->f == f.f_base  
            &&  s.fv_prim(k)->atom->args == combos(i)) {
          values.append(s.fv_prim(k)->value);
          break;
        }
      }
    }
    else {
      FOR1D(s.fv_derived, k) {
        if (s.fv_derived(k)->atom->f == f.f_base  
            &&  s.fv_derived(k)->atom->args == combos(i)) {
          values.append(s.fv_derived(k)->value);
          break;
        }
      }
    }
  }
  double maxVal = values.max();
  // function value bauen
  uintA empty;
  TL::FunctionValue* fv = TL::logicObjectManager::getFV(&f, empty, maxVal);
  s.fv_derived.setAppend(fv);
  if (DEBUG>0) {cout<<" ";fv->write(cout);}
  if (DEBUG>0) {cout<<"deriveFunctionValues_max [END]"<<endl;}
  return true; // always holds
}


bool TL::logicReasoning::deriveFunctionValues_reward(TL::RewardFunction& f, TL::State& s) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"deriveFunctionValues_reward [START]"<<endl;}
  CHECK(f.d==0, "only zero-ary functions");
  double value = 0;
  if (logicReasoning::holds(s, f.grounded_pis))
    value = f.reward_value;
  // function value bauen
  uintA empty;
  empty.resize(f.d);
  TL::FunctionValue* fv = TL::logicObjectManager::getFV(&f, empty, value);
  s.fv_derived.setAppend(fv);
  if (DEBUG>0) {cout<<" ";fv->write(cout);}
  if (DEBUG>0) {cout<<"deriveFunctionValues_reward [END]"<<endl;}
  return true; // always holds
}






// construct TL::logicObjectManager::p_derived predicates and functions
// assumptions: no self recursion
// negation in base predicates is only allowed for primitive predicates
void TL::logicReasoning::derive(const LitL& lits_prim, const FuncVL& fv_prim, LitL& lits_derived, FuncVL& fv_derived) {
    uint DEBUG = 0;
    if (DEBUG > 0) cout << "derive [START]" << endl;
    if (DEBUG > 1) {
        cout << "Primitive Lits: ";
        write(lits_prim);
        cout << endl;
    }
  
    TL::State s;
    s.derivedDerived = true; // needs to be set here, since TL::logicObjectManager::p_derived predicates that build on other TL::logicObjectManager::p_derived predicates require that the state be TL::logicObjectManager::p_derived; in short: setting this state as derivedDerived is safe here since we will derive all the stuff now anyway
    s.lits_prim.append(lits_prim);
    s.fv_prim.append(fv_prim);
    
    uintA ordered_ids;
    boolA isPredicate;
    TL::logicObjectManager::dependencyGraph.getWellDefinedOrder(ordered_ids, isPredicate, true);
    
    uint i, d;
    FOR1D(ordered_ids, i) {
        if (isPredicate(i)) {
            FOR1D(TL::logicObjectManager::p_derived, d) {
                if (TL::logicObjectManager::p_derived(d)->id == ordered_ids(i)) break;
            }
            CHECK(d<TL::logicObjectManager::p_derived.N, "TL::logicObjectManager::p_derived predicate not found");
            if (TL::logicObjectManager::p_derived(d)->type == TL::Predicate::predicate_conjunction) {
                TL::ConjunctionPredicate* scp = dynamic_cast<TL::ConjunctionPredicate*>(TL::logicObjectManager::p_derived(d));
                CHECK(scp!=NULL, "cast failed");
                deriveLiterals_conjunction(*scp, s);
            }
            else if (TL::logicObjectManager::p_derived(d)->type == TL::Predicate::predicate_transClosure) {
                TL::TransClosurePredicate* tcp = dynamic_cast<TL::TransClosurePredicate*>(TL::logicObjectManager::p_derived(d));
                CHECK(tcp!=NULL, "cast failed");
                deriveLiterals_transClosure(*tcp, s);
            }
            else if (TL::logicObjectManager::p_derived(d)->type == TL::Predicate::predicate_count) {
                TL::CountPredicate* p = dynamic_cast<TL::CountPredicate*>(TL::logicObjectManager::p_derived(d));
                CHECK(p!=NULL, "cast failed");
                deriveLiterals_count(*p, s);
            }
            else {
                HALT("Unknown predicate")
            }
        }
        else {
            FOR1D(TL::logicObjectManager::f_derived, d) {
                if (TL::logicObjectManager::f_derived(d)->id == ordered_ids(i)) break;
            }
            CHECK(d<TL::logicObjectManager::f_derived.N, "TL::logicObjectManager::p_derived function not found; with id="<<ordered_ids(i));
            if (TL::logicObjectManager::f_derived(d)->type == TL::Function::function_count) {
                TL::CountFunction* f = dynamic_cast<TL::CountFunction*>(TL::logicObjectManager::f_derived(d));
                CHECK(f!=NULL, "cast failed");
                deriveFunctionValues_count(*f, s);
            }
            else if (TL::logicObjectManager::f_derived(d)->type == TL::Function::function_avg) {
              TL::AverageFunction* f = dynamic_cast<TL::AverageFunction*>(TL::logicObjectManager::f_derived(d));
              CHECK(f!=NULL, "cast failed");
              deriveFunctionValues_avg(*f, s);
            }
            else if (TL::logicObjectManager::f_derived(d)->type == TL::Function::function_max) {
              TL::MaxFunction* f = dynamic_cast<TL::MaxFunction*>(TL::logicObjectManager::f_derived(d));
              CHECK(f!=NULL, "cast failed");
              deriveFunctionValues_max(*f, s);
            }
            else if (TL::logicObjectManager::f_derived(d)->type == TL::Function::function_sum) {
              TL::SumFunction* f = dynamic_cast<TL::SumFunction*>(TL::logicObjectManager::f_derived(d));
              CHECK(f!=NULL, "cast failed");
              deriveFunctionValues_sum(*f, s);
            }
            else if (TL::logicObjectManager::f_derived(d)->type == TL::Function::function_reward) {
              TL::RewardFunction* f = dynamic_cast<TL::RewardFunction*>(TL::logicObjectManager::f_derived(d));
              CHECK(f!=NULL, "cast failed");
              deriveFunctionValues_reward(*f, s);
            }
            else {
                HALT("Unknown function")
            }
        }
    }
  
    lits_derived.clear();
    FOR1D(s.lits_derived, i) {lits_derived.append(s.lits_derived(i));}
    
    fv_derived.clear();
    FOR1D(s.fv_derived, i) {fv_derived.append(s.fv_derived(i));}
  
    if (DEBUG > 1) {
        cout << "TL::logicObjectManager::p_derived Tuples: ";
        write(lits_derived);
        cout << endl;
        cout << "TL::logicObjectManager::p_derived funcValues: ";
        write(fv_derived);
        cout << fv_derived;
    }
  
    if (DEBUG > 0) cout << "derive [END]" << endl;
}

void TL::logicReasoning::derive(TL::State* s) {
	if (s->derivedDerived)
		return;
	LitL derivedPTs;
  FuncVL derivedFVs;
  derive(s->lits_prim, s->fv_prim, derivedPTs, derivedFVs);
  s->lits_derived.append(derivedPTs);
  s->fv_derived.append(derivedFVs);
	s->derivedDerived = true;
//     s->writeNice(cout, true);
}

void TL::logicReasoning::dederive(TL::State* s) {
    s->lits_derived.clear();
    s->fv_derived.clear();
    s->derivedDerived = false;
}




// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
//    G E N E R A L   L O G I C  stuff der sonst net einzuordnen ist
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------


bool TL::logicReasoning::nonContradicting(const LitL& l1, const LitL& l2) {
  uint i, k;
  FOR1D(l1, i) {
    FOR1D(l2, k) {
      if (l1(i)->atom == l2(k)->atom) {
	if (l1(i)->positive != l2(k)->positive)
	  return false;
      }
    }
  }
  return true;
}




// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
//    Advanced Logic
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------

void TL::logicReasoning::determineStupidDerivedConcepts(MT::Array< TL::Trial* >& data, PredL& stupidPredicates, FuncL& stupidFunctions, bool deleteFromlogicReasoning) {
    uint DEBUG=0;
    if (DEBUG>0) {cout<<"determineStupidDerivedPredicates [START]"<<endl;}
    boolA dp_values;
    uint w, s, c, p;
    // first derive states
    FOR1D(data, w) {
        FOR1D(data(w)->states, s) {
            derive(data(w)->states(s));
        }
    }
    
    bool value, change, localTruth;
    value = change = localTruth = false;
    FOR1D(TL::logicObjectManager::p_derived, p) {
        MT::Array< uintA > combos;
        TL::allPossibleLists(combos, TL::logicObjectManager::constants, TL::logicObjectManager::p_derived(p)->d, true, true);
        change = false;
        if (DEBUG>2) {cout<<"Examining ";TL::logicObjectManager::p_derived(p)->writeNice(cout);cout<<endl;}
        FOR1D(combos, c) {
            TL::Literal* lit = TL::logicObjectManager::getLiteral(TL::logicObjectManager::p_derived(p), true, combos(c));
            if (DEBUG>2) {cout<<"Examining lit ";lit->write(cout);cout<<endl;}
            FOR1D(data, w) {
                FOR1D(data(w)->states, s) {
                    localTruth = holds(*data(w)->states(s), lit);
                    if (DEBUG>3) {
                        cout<<"w"<<w<<" s"<<s<<":"; data(w)->states(s)->write(cout);cout<<endl<<"  --> "<<localTruth<<endl;
                    }
                    if (s==0) {
                        value = localTruth;
                    }
                    else {
                        if (value != localTruth) {
                            change = true;
                            break;
                        }
                    }
                }
                if (change)
                    break;
            }
            if (DEBUG>2) {cout<<"  Changed? "<<change<<endl;}
            if (change)
                break;
        }
        if (!change) {
            stupidPredicates.append(TL::logicObjectManager::p_derived(p));
            if (DEBUG>2) {cout<<TL::logicObjectManager::p_derived(p)->name<<" is a stupid predicate."<<endl;}
        }
    }
    
    TL::logicObjectManager::p_derived.memMove = true;
    TL::logicObjectManager::f_derived.memMove = true;
    if (deleteFromlogicReasoning) {
        FOR1D_DOWN(stupidPredicates, p) {
            TL::logicObjectManager::p_derived.removeValueSafe(stupidPredicates(p));
        }
        // check which functions use some of the stupidPredicates and delete them as well
        FOR1D_DOWN(TL::logicObjectManager::f_derived, p) {
            PredL p_base;
            FuncL f_base;
            baseConcepts(TL::logicObjectManager::f_derived(p), p_base, f_base);
            PredL stupidPredicates_mask;
            uint r;
            FOR1D(stupidPredicates, r) {stupidPredicates_mask.append(stupidPredicates(r));}
            if (numberSharedElements(p_base, stupidPredicates_mask) > 0) {
                TL::logicObjectManager::f_derived.removeValueSafe(TL::logicObjectManager::f_derived(p));
            }
        }
    }
    
    // reset dependency graph
    PredL ps_all;
    ps_all.append(TL::logicObjectManager::p_prim);
    FOR1D(TL::logicObjectManager::p_derived, p) ps_all.append(TL::logicObjectManager::p_derived(p));
    FuncL fs_all;
    fs_all.append(TL::logicObjectManager::f_prim);
    fs_all.append(TL::logicObjectManager::f_derived);
    TL::logicObjectManager::dependencyGraph.setNodes(ps_all, fs_all);

    if (DEBUG>0) {
        cout<<"Stupid predicates:"<<endl;
        writeNice(stupidPredicates);
        if (deleteFromlogicReasoning) {
            cout<<"This leaves the logicReasoning with the following TL::logicObjectManager::p_derived predicates:"<<endl;
            writeNice(TL::logicObjectManager::p_derived);
        }
    }
    
    if (DEBUG>0) {cout<<"determineStupidDerivedPredicates [END]"<<endl;}
}



void TL::logicReasoning::killBaseConcepts(LitL& lits) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"killBaseConcepts[START]"<<endl;}
  if (DEBUG>0) {cout<<"in ["<<lits.N<<"]: ";write(lits);cout<<endl;}
  uint i, k, d, t;
  LitL removed;
  FOR1D(lits, i) {
    if (lits(i)->positive  &&  lits(i)->atom->pred->type == TL::Predicate::predicate_conjunction) {
      TL::ConjunctionPredicate* scp = dynamic_cast<TL::ConjunctionPredicate*>(lits(i)->atom->pred);
      CHECK(scp!=NULL, "cast failed");
      uintA freeVars_pos, freeVars_neg;
      calcFreeVars(*scp, freeVars_pos, freeVars_neg);
      if (freeVars_pos.N==0 && freeVars_neg.N==0) {
        if (DEBUG>1) {cout<<"Removing base preds of ";scp->writeNice(cout);cout<<endl;}
        TL::Substitution sub;
        for (k=0; k<scp->d; k++) {
          sub.addSubs(k, lits(i)->atom->args(k));
        }
        t=0;
        FOR1D(scp->basePreds, k) {
          uintA basePred_args(scp->basePreds(k)->d);
          FOR1D(basePred_args, d) {
            basePred_args(d) = sub.getSubs(scp->basePreds_mapVars2conjunction(t++));
          }
          removed.append(TL::logicObjectManager::getLiteral(scp->basePreds(k), scp->basePreds_positive(k), basePred_args));
        }
      }
    }
  }
  if (DEBUG>1) {cout<<"Removed guys: ";write(removed);cout<<endl;}
  LitL filtered;
  FOR1D(lits, i) {
    if (removed.findValue(lits(i))<0)
      filtered.setAppend(lits(i));
  }
  lits = filtered;
  if (DEBUG>0) {cout<<"out ["<<lits.N<<"]: ";write(lits);cout<<endl;}
  if (DEBUG>0) {cout<<"killBaseConcepts [END]"<<endl;}
}







// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
//    T R A N S I T I O N S
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------






void TL::logicReasoning::changes(const TL::State& pre, const TL::State& post, uintA& changedConstants, LitL& holdOnlyPre, LitL& holdOnlyPost) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"changes [START]" << endl;
  if (DEBUG>1) {
    cout << "Pre: "; pre.write(cout); cout << endl;
    cout << "Post: "; post.write(cout); cout << endl;
  }
  
  // only look at TL::logicObjectManager::p_prim
  changedConstants.clear();
  holdOnlyPre.clear();
  holdOnlyPost.clear();
  uint i;
  // pre+, post-
  FOR1D(pre.lits_prim, i) {
    if (post.lits_prim.findValue(pre.lits_prim(i)) < 0) {
      holdOnlyPre.append(pre.lits_prim(i));
      changedConstants.setAppend(pre.lits_prim(i)->atom->args);
    }
  }
  // pre-, post+
  FOR1D(post.lits_prim, i) {
    if (pre.lits_prim.findValue(post.lits_prim(i)) < 0) {
      holdOnlyPost.append(post.lits_prim(i));
      changedConstants.setAppend(post.lits_prim(i)->atom->args);
    }
  }

  if (DEBUG>0) {
    cout<<"only in pre: "; write(holdOnlyPre); cout<<endl;
    cout<<"only in post: "; write(holdOnlyPost); cout<<endl;
    PRINT(changedConstants)
  }
  if (DEBUG>0) cout<<"changes [END]" << endl;
}










// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
//     S U B S T I T U T I O N S
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------


TL::Atom* TL::logicReasoning::applyOriginalSub(TL::Substitution& sub, TL::Atom* a) {
  return TL::logicObjectManager::getAtomOrig(sub.apply(a));
}

TL::Literal* TL::logicReasoning::applyOriginalSub(TL::Substitution& sub, TL::Literal* lit) {
  return TL::logicObjectManager::getLiteralOrig(sub.apply(lit));
}

void TL::logicReasoning::applyOriginalSub(TL::Substitution& sub, const LitL& unsub_lits, LitL& sub_lits) {
  sub_lits.clear();
  uint i;
  FOR1D(unsub_lits, i)
    sub_lits.append(applyOriginalSub(sub, unsub_lits(i)));
}




