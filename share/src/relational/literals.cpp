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

#include <stdlib.h>
#include "symbols.h"
#include "reason.h" // for derive in state

#define NOISE_MARKER 'N'

namespace relational {

  
/************************************************
 * 
 *     LiteralStorage
 * 
 ************************************************/
  

static const uint LOGIC__MAX_LIMIT_CONSTANT_ID = 80;

struct LiteralStorage {
  Symbol* symbol;
  MT::Array< LitL > mem;
  LiteralStorage(Symbol* s, uint arity) : symbol(s) {
    if      (arity == 0) {mem.resize(1);}
    else if (arity == 1) {mem.resize(LOGIC__MAX_LIMIT_CONSTANT_ID);}
    else if (arity == 2) {mem.resize(LOGIC__MAX_LIMIT_CONSTANT_ID, LOGIC__MAX_LIMIT_CONSTANT_ID);}
    else NIY;
    LitL empty(0);
    mem.setUni(empty);
  }
  LiteralStorage() {} // only for array_t.cpp
  ~LiteralStorage() {
    uint i, k;
    FOR_ALL(mem, i) {
      FOR1D(mem.elem(i), k) {
        if (mem.elem(i)(k) != NULL) delete mem.elem(i)(k);
      }
    }
  }
};


struct LiteralStorage_Container {
  MT::Array< LiteralStorage* > literal_storages;
  LiteralStorage_Container() {}
  ~LiteralStorage_Container() {listDelete(literal_storages);}
};


LiteralStorage_Container lsc;



/************************************************
 * 
 *     Literal
 * 
 ************************************************/

Literal::Literal() {}


Literal* Literal::get(Symbol* s, const uintA& args, double value, ComparisonType comparison_type) {
  if (s->arity != args.N) {HALT("s->arity!=args.N  s="<<*s<<"    args="<<args);}
  uint i;
  FOR1D(lsc.literal_storages, i) {
    if (lsc.literal_storages(i)->symbol == s) break;
  }
  if (i == lsc.literal_storages.N) {
    lsc.literal_storages.append(new LiteralStorage(s, s->arity));
  }
  uintA args_memIndex = args;
  if (s->arity == 0) {
    CHECK(args.N == 0, "");
    args_memIndex.append(0);
  }
  LitL& literal_list = lsc.literal_storages(i)->mem(args_memIndex); 
  FOR1D(literal_list, i) {
    if (TL::areEqual(literal_list(i)->value, value)) {
      return literal_list(i);
    }
  }
  // insert
  Literal* new_literal = new Literal;
  new_literal->s = s;
  new_literal->args = args;
  new_literal->value = value;
  new_literal->comparison_type = comparison_type;
  literal_list.append(new_literal);
  return new_literal;
}


Literal* Literal::get(const char* text) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"Literal::get [START]"<<endl;}
  MT::String mt_string(text);
  if (DEBUG>0) {PRINT(mt_string);}
  
  double value = 1.;
  
  // Special negation with "-" for binary predicates
  if (MT::peerNextChar(mt_string) == '-') {
    value = 0.;
    skipOne(mt_string);
  }
  
  // Symbol
  MT::String name;
  name.read(mt_string, NULL, "(");
  if (DEBUG>0) PRINT(name);
  Symbol* s = Symbol::get(name);
  if (DEBUG>0) {PRINT(*s);}
  
  // Arguments
  if (DEBUG>0) {cout<<"Reading arguments"<<endl;}
  uintA args(s->arity);
  uint i;
  FOR1D(args, i) {
    MT::String arg;
    arg.read(mt_string, NULL, "/, )");
    if (DEBUG>0) {PRINT(arg);}
    if (isdigit(MT::peerNextChar(arg))) {
      arg >> args(i);
    }
    else {  // for string variables
      if (arg.N != 1) HALT("non-digit argument in bad format: "<<arg);
      if (arg(0) == 'X') args(i) = 0;
      else if (arg(0) == 'Y') args(i) = 1;
      else if (arg(0) == 'Z') args(i) = 2;
      else if (arg(0) == 'V') args(i) = 3;
      else if (arg(0) == 'W') args(i) = 4;
      else if (arg(0) == 'U') args(i) = 5;
      else HALT("unknown variable "<<arg);
    }
  }
  
  // Value
  Literal::ComparisonType comp_type;
  if (MT::peerNextChar(mt_string) == -1  &&  s->range_type == Symbol::binary) {
    comp_type = Literal::comparison_equal;
  }
  else {
    char op;
    mt_string >> op;
    if (DEBUG>0) {PRINT(op);}
    if (op == '>') {
      if (MT::peerNextChar(mt_string) == '=') {
        mt_string >> op;
        comp_type = Literal::comparison_greaterEqual;
      }
      else
        comp_type = Literal::comparison_greater;
    }
    else if (op == '<') {
      if (MT::peerNextChar(mt_string) == '=') {
        mt_string >> op;
        comp_type = Literal::comparison_lessEqual;
      }
      else
        comp_type = Literal::comparison_less;
    }
    else
      comp_type = Literal::comparison_equal;
    mt_string >> value;
    if (DEBUG>0) {PRINT(value);}
  }

  Literal* l = Literal::get(s, args, value, comp_type);
  if (DEBUG>0) {cout<<"==> "<<*l<<endl;}
  if (DEBUG>0) {cout<<"Literal::get [END]"<<endl;}
  return l;
}


void Literal::get(LitL& lits, const char* text) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"Literal::get(LitL) [START]"<<endl;}
  if (DEBUG>0) {PRINT(text);}
  lits.clear();
  MT::String mt_string(text);
  MT::String name;
  while (MT::skip(mt_string) != -1) {
    MT::String lit_text;
    lit_text.read(mt_string, NULL, ")", 1);
    if (MT::peerNextChar(mt_string) == ',') MT::skipOne(mt_string);
//     MT::skip(std::istream& is,const char *skipchars=" \n\r\t",bool skipCommentLines=true);
    char c = MT::peerNextChar(mt_string);
    if (c == '='  ||  c == '>'  ||  c == '<') {
      MT::String comparison;
      comparison.read(mt_string, NULL, " \n", 1);
      lit_text << ")" << comparison;
    }
    if (DEBUG>0) {PRINT(lit_text);}
    lits.append(Literal::get(lit_text));
  }
  if (DEBUG>0) {relational::write(lits); cout<<endl;}
  if (DEBUG>0) {cout<<"Literal::get(LitL) [END]"<<endl;}
}


bool Literal::operator==(Literal& l) const {
  if (s != l.s)
    return false;
  if (comparison_type != l.comparison_type)
    return false;
  if (value != l.value)
    return false;
  if (this->args.N==0 && l.args.N==0) // special case if no slot assignments
    return true;
  if (args == l.args)
    return true;
  else
    return false;
}

bool Literal::operator!=(Literal& lit) const {
  return !(*this == lit);
}


void Literal::write(ostream& os, bool withTypes) const {
  if (s->range_type == Symbol::binary  &&  TL::isZero(value))
    os << "-";
  os << s->name << "(";
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
      if (s->arg_types.N > 0) {
        os<<"/"<<s->arg_types(i)->name;
      }
    }
    if (i < args.N-1)
      os << " ";
  }
  os << ")";
  if (s->range_type != Symbol::binary) {
    switch(comparison_type) {
      case Literal::comparison_equal: os << "="; break;
      case Literal::comparison_less: os << "<"; break;
      case Literal::comparison_lessEqual: os << "<="; break;
      case Literal::comparison_greater: os << ">"; break;
      case Literal::comparison_greaterEqual: os << ">="; break;
      default: HALT("Unknown comparison type")
    }
    os<<value;
  }
}


bool Literal::compareValueTo(Literal::ComparisonType compType, double a) {
  return compare(this->value, compType, a);
}


bool Literal::compare(double a, Literal::ComparisonType compType, double b) {
  switch(compType) {
    case Literal::comparison_equal: return TL::areEqual(a,b);
    case Literal::comparison_less: return a<b;
    case Literal::comparison_lessEqual: return a<=b;
    case Literal::comparison_greater: return a>b;
    case Literal::comparison_greaterEqual: return a>=b;
    default: HALT("Undefined comparison:  "<<compType)
  }
  return false;
}


bool Literal::isNegated() const {
  return s->range_type == Symbol::binary  &&  TL::isZero(value);
}


Literal* Literal::getNegated() {
  CHECK(s->range_type == Symbol::binary, "only defined for binary symbols")
  return Literal::get(s, args, (TL::isZero(value) ? 1. : 0.));
}


bool Literal::equivalent(const LitL& p1, const LitL& p2) {
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


bool Literal::nonContradicting(const LitL& l1, const LitL& l2) {
  uint i, k;
  FOR1D(l1, i) {
    FOR1D(l2, k) {
      if (l1(i)->s == l2(k)->s  &&  l1(i)->args == l2(k)->args) {
        if (!TL::areEqual(l1(i)->value, l2(k)->value))
          return false;
      }
    }
  }
  return true;
}


void Literal::getLiterals(LitL& lits, Symbol* s, const uintA& constants, double value) {
  lits.clear();
  MT::Array< uintA > args_lists;
  TL::allPermutations(args_lists, constants, s->arity, true, true);
  uint i;
  FOR1D(args_lists, i) {
    lits.append(Literal::get(s, args_lists(i), value));
  }
}


void Literal::getLiterals_state(LitL& lits, const uintA& constants, double value, bool binaryOnly) {
  lits.clear();
  SymL syms_state;
  Symbol::get_state(syms_state);
  uint i;
  FOR1D(syms_state, i) {
    if (syms_state(i)->range_type != Symbol::binary) continue;
    LitL lits_local;
    getLiterals(lits_local, syms_state(i), constants, value);
    lits.setAppend(lits_local);
  }
}


void Literal::getLiterals_state(LitL& lits, const uintA& constants, const uintA& constants_mustBeContained, double value, bool binaryOnly) {
  uint DEBUG=0;
  if (DEBUG>0) cout<<"getLiterals_state [START]"<<endl;
  lits.clear();
  LitL lits_total;
  Literal::getLiterals_state(lits_total, constants, value, binaryOnly);
  uint i;
  FOR1D(lits_total, i) {
    if (numberSharedElements(lits_total(i)->args, constants_mustBeContained) == 0) {
      lits.append(lits_total(i));
    }
  }
  if (DEBUG>1) relational::write(lits);
  if (DEBUG>0) cout<<"getLiterals_state [END]"<<endl;
}


void Literal::getLiterals_actions(LitL& lits, const uintA& arguments) {
  lits.clear();
  SymL syms_action;
  Symbol::get_action(syms_action);
  uint i;
  FOR1D(syms_action, i) {
    LitL lits2;
    getLiterals(lits2, syms_action(i), arguments, 1.);
    lits.setAppend(lits2);
  }
}


double Literal::getValue(const LitL& literals, const Symbol* s, const uintA& args) {
  uint i;
  FOR1D(literals, i) {
    if (literals(i)->s == s  &&  literals(i)->args == args)
      return literals(i)->value;
  }
  HALT("value cannot be determined:  s="<<*s<<"  args="<<args<<"   in  literals="<<literals);
}


uint Literal::getArguments(uintA& args, const LitL& lits) {
  uint i;
  FOR1D(lits, i) {
    args.setAppend(lits(i)->args);
  }
  TL::sort_asc(args);
  return args.N;
}


bool Literal::negativeBinaryLiteralsLast(const LitL& lits) {
  bool negative_binary_started = false;
  uint i;
  FOR1D(lits, i) {
    bool is_positive = !lits(i)->isNegated();
    if (is_positive && negative_binary_started)
      return false;
    if (!is_positive && !negative_binary_started)
      negative_binary_started = true;
  }
  return true;
}


uint Literal::numberLiterals(const MT::Array< LitL >& LitLs) {
  uint no = 0;
  uint i;
  FOR1D(LitLs, i) {
    no += LitLs(i).N;
  }
  return no;
}

// positives first
void Literal::sort(LitL& lits) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"Literal::sort [START]"<<endl;}
  if (DEBUG>0) {PRINT(lits);}
  uint i;
  uintA keys(lits.N);
  SymL symbols;
  Symbol::get(symbols);
  // literals with small keys first
  // Stellen 1-4: arguments
  // Stellen 5-6: symbol
  // Stelle 7: negative
  FOR1D(lits, i) {
    uint key = 0;
    // Stellen 1-4: arguments
    if (lits(i)->s->arity > 0) {
      CHECK(lits(i)->args(0) < 100, "");
      key += 10e2 * lits(i)->args(0);
    }
    if (lits(i)->s->arity > 1) {
      CHECK(lits(i)->args(1) < 100, "");
      key += lits(i)->args(1);
    }
    CHECK(lits(i)->s->arity <= 2, "");
    // Stellen 5-6: symbols
    key += 10e4 * symbols.findValue(lits(i)->s);
    // Stelle 7: negated literals last
    if (lits(i)->isNegated()) {
      key += 10e7;
    }
    keys(i) = key;
    if (DEBUG>1) {cout<<"("<<i<<") "<<*lits(i)<<"\t"<<keys(i)<<endl;}
  }
  uintA sortedIndices;
//   PRINT(sortedIndices);
  TL::sort_asc_keys(sortedIndices, keys);
//   PRINT(sortedIndices);
  LitL lits_sorted;
  FOR1D(sortedIndices, i) {
    lits_sorted.append(lits(sortedIndices(i)));
  }
  if (DEBUG>0) {PRINT(lits_sorted);}
  lits = lits_sorted;
  if (DEBUG>0) {cout<<"Literal::sort [END]"<<endl;}
}


void Literal::negate(const LitL& lits, LitL& predTs_negated) {
  predTs_negated.clear();
  uint i;
  Literal* lit;
  FOR1D(lits, i) {
    lit = Literal::get(lits(i)->s, lits(i)->args, TL::isZero(lits(i)->value));
    predTs_negated.append(lit);
  }
}


int Literal::findPattern(const LitL& actions, uint minRepeats) {
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


Literal* Literal::getLiteral_default_action() {
  uintA empty;
  return get(Symbol::get(MT::String("default"), 0, Symbol::action), empty, 1.);
}


Literal* Literal::getLiteral_doNothing() {
  uintA empty;
  return get(Symbol::get(MT::String("doNothing"), 0, Symbol::action), empty, 1.);
}


void write(const LitL& lits, ostream& os) {
  uint k;
  FOR1D(lits, k) {
    if (lits(k)!=NULL)
      lits(k)->write(os);
    else
      os<<"NULL";
    os << " ";
  }
}


void write(const MT::Array< LitL >& outcomes, ostream& os) {
  uint k;
  FOR1D(outcomes, k) {
    os << "(" << k << ") ";
    write(outcomes(k), os);
    os << endl;
  }
}



/************************************************
 * 
 *     SymbolicState
 * 
 ************************************************/

SymbolicState::SymbolicState() {
  including_derived_literals = false;
}


SymbolicState::SymbolicState(const MT::Array<Literal*>& _lits) {
  including_derived_literals = false;
  this->lits = _lits;
  reason::derive(this);
  Literal::getArguments(state_constants, lits);
}


void SymbolicState::write(ostream& os, bool primOnly) const {
  uint i;
  FOR1D(lits, i) {
    if (primOnly && lits(i)->s->symbol_type != Symbol::primitive)
      continue;
    os << *lits(i) << " ";
  }
}


void SymbolicState::read(ifstream& in, bool read_constants) {
  if (read_constants) {
    in >> state_constants;
  }
  MT::String line;
  line.read(in, NULL, "\n");
  Literal::get(lits, line);
  if (!read_constants) {
    Literal::getArguments(state_constants, lits);
  }
  uint i;
  FOR1D_DOWN(lits, i) {
    if (lits(i)->s->symbol_type != Symbol::primitive)
      lits.remove(i);
  }
  including_derived_literals = false;
  reason::derive(this);
}


bool SymbolicState::operator==(const SymbolicState& s) const {
  LitL this__lits_prim, other__lits_prim;
  uint i;
  FOR1D(lits, i) {
    if (lits(i)->s->symbol_type == Symbol::primitive)
      this__lits_prim.append(lits(i));
  }
  FOR1D(s.lits, i) {
    if (s.lits(i)->s->symbol_type == Symbol::primitive)
      other__lits_prim.append(s.lits(i));
  }
  return Literal::equivalent(this__lits_prim, other__lits_prim);
}


bool SymbolicState::operator!=(const SymbolicState& s) const {
  return !(*this == s);
}


SymbolicState::~SymbolicState() {
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


void SymbolicState::calcDifferences(LitL& lits_diff_1to2, LitL& lits_diff_2to1, uintA& changedConstants, const SymbolicState& state1, const SymbolicState& state2) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calcDifferences [START]"<<endl;}
  
  if (DEBUG>0) {
    cout<<"SymbolicState 1:"<<endl;  state1.write();  cout<<endl;
    cout<<"SymbolicState 2:"<<endl;  state2.write();  cout<<endl;
  }
  
  lits_diff_1to2.clear();
  lits_diff_2to1.clear();
  changedConstants.clear();
  
  uint i;
  FOR1D(state1.lits, i) {
    if (state2.lits.findValue(state1.lits(i)) < 0) {
      lits_diff_1to2.append(state1.lits(i));
      changedConstants.setAppend(state1.lits(i)->args);
    }
  }
  FOR1D(state2.lits, i) {
    if (state1.lits.findValue(state2.lits(i)) < 0) {
      lits_diff_2to1.append(state2.lits(i));
      changedConstants.setAppend(state2.lits(i)->args);
    }
  }
  
  if (DEBUG>0) {
    cout<<"lits_diff_1to2:    "<<lits_diff_1to2<<endl;
    cout<<"lits_diff_2to1:    "<<lits_diff_2to1<<endl;
    PRINT(changedConstants);
  }
  
  if (DEBUG>0) {cout<<"calcDifferences [END]"<<endl;}
}


void SymbolicState::filterState_full(SymbolicState& s_filtered, const SymbolicState& s_full, const uintA& filter_constants, bool primOnly) {
  uint i, k;
  // lits
  s_filtered.lits.clear();
  FOR1D(s_full.lits, i) {
    FOR1D(s_full.lits(i)->args, k) {
      if (filter_constants.findValue(s_full.lits(i)->args(k))<0)
        break;
    }
    if (k==s_full.lits(i)->args.N)
      s_filtered.lits.append(s_full.lits(i));
  }
}


void SymbolicState::filterState_atleastOne(SymbolicState& s_filtered, const SymbolicState& s_full, const uintA& filter_constants, bool primOnly) {
  uint i, k;
  // lits
  s_filtered.lits.clear();
  FOR1D(s_full.lits, i) {
    if (s_full.lits(i)->args.N == 0)
      s_filtered.lits.append(s_full.lits(i));
    FOR1D(s_full.lits(i)->args, k) {
      if (filter_constants.findValue(s_full.lits(i)->args(k))>=0) {
        s_filtered.lits.append(s_full.lits(i));
        break;
      }
    }
  }
}


uint SymbolicState::getArgument(const SymbolicState& state, const Symbol& s) {
  CHECK(s.arity == 1, "");
  uintA args;
  getArguments(args, state, s);
  if (args.N == 1)
    return args(0);
  else
    return UINT_MAX;
}


void SymbolicState::getArguments(uintA& args, const SymbolicState& state, const Symbol& s) {
  args.clear();
  uint i;
  FOR1D(state.lits, i) {
    if (*state.lits(i)->s == s)
      args.setAppend(state.lits(i)->args);
  }
  TL::sort_asc(args);
}


double SymbolicState::getValue(const Symbol* s, const SymbolicState& state) {
  if (s->arity != 0) HALT("only defined for 0-ary symbol");
  uintA empty;
  return Literal::getValue(state.lits, s, empty);
}


void SymbolicState::getValues(arr& values, const SymbolicState& state, const Symbol& s, const uintA& objs) {
  uint i;
  FOR1D(state.lits, i) {
    if (state.lits(i)->s != &s) continue;
    if (numberSharedElements(state.lits(i)->args, objs) == objs.N)
      values.append(state.lits(i)->value);
  }
}


void SymbolicState::getRelatedConstants(uintA& constants_related, uint id, bool id_covers_first, const Symbol& s, const SymbolicState& state) {
  constants_related.clear();
  uint i;
  FOR1D(state.lits, i) {
    if (*state.lits(i)->s == s) {
      if (id_covers_first) {
        if (state.lits(i)->args(0) == id)
          constants_related.append(state.lits(i)->args(1));
      }
      else {
        if (state.lits(i)->args(1) == id)
          constants_related.append(state.lits(i)->args(0));
      }
    }
  }
}






/************************************************
 * 
 *     StateTransition
 * 
 ************************************************/


StateTransition::StateTransition(const SymbolicState& pre, Literal* action, const SymbolicState& post) {
  this->pre = pre;
  this->action = action;
  this->post = post;
  calcChanges();
}


StateTransition::StateTransition() {}


StateTransition::~StateTransition() {}


void StateTransition::calcChanges() {
  // only look at p_prim
  changedConstants.clear();
  del.clear();
  add.clear();
  uint i;
  // pre+, post-
  FOR1D(pre.lits, i) {
    if (pre.lits(i)->s->symbol_type != Symbol::primitive) continue;
    if (post.lits.findValue(pre.lits(i)) < 0) {
      del.append(pre.lits(i));
      changedConstants.setAppend(pre.lits(i)->args);
    }
  }
  // pre-, post+
  FOR1D(post.lits, i) {
    if (post.lits(i)->s->symbol_type != Symbol::primitive) continue;
    if (pre.lits.findValue(post.lits(i)) < 0) {
      add.append(post.lits(i));
      changedConstants.setAppend(post.lits(i)->args);
    }
  }
}


void StateTransition::write(ostream& os, bool with_details) const {
  os << "ACTION: " << *action << endl;
  if (with_details) {
    os << "PRE:    ";
    this->pre.write(os, true);  os << endl;
    os << "POST:   ";
    this->post.write(os, true);  os<<endl;
    os<<"Changed constants: "<<changedConstants<<endl;
    os << "Diff: "<<(add.N + del.N)<<" (+"<<add.N<<", -"<<del.N<<")"<<endl;
  }
  os<<"ADD:    "<<add<<endl;
  os<<"DEL:    "<<del<<endl;
}


bool StateTransition::noChange() {
  return changedConstants.N == 0  &&  del.N == 0  &&  add.N == 0;
}


void StateTransition::write(const StateTransitionL& exs, ostream& os) {
  uint t;
  FOR1D(exs, t) {
    os<<"# t="<<t<<endl;
    os<<exs(t)->pre<<endl;
    os<<*exs(t)->action<<endl;
    if (t==exs.N-1) {
      os<<"# t="<<(t+1)<<endl;
      os<<exs(t)->post<<endl;
    }
  }
}


StateTransitionL& StateTransition::read(const char* filename) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"StateTransition::read [START]"<<endl;}
  if (DEBUG>0) {PRINT(filename);}
  static StateTransitionL experiences;
  ifstream in(filename);
  if (!in.is_open()) {HALT("File not found: "<<filename);}
  MT::skip(in);
  bool read_state = true;
  SymbolicState* state, *state_old = NULL;
  Literal* action = NULL;
  while (MT::skip(in) != -1) {
    if (DEBUG>2) {PRINT(MT::peerNextChar(in));}
    if (read_state) {
      state = new SymbolicState;
      state->read(in);
      if (state_old != NULL) {
        experiences.append(new StateTransition(*state_old, action, *state));
        if (DEBUG>0) {cout<<*experiences.last()<<endl;}
      }
      state_old = state;
    }
    else {
      MT::String line;
      line.read(in, NULL, "\n");
      if (DEBUG>1) {cout<<"READING ACTION:"<<endl; PRINT(line);}
      action = Literal::get(line);
    }
    read_state = !read_state;
  }
  if (DEBUG>0) {PRINT(experiences.N);}
  if (DEBUG>0) {cout<<"StateTransition::read [END]"<<endl;}
  return experiences;
}


void write(const StateTransitionL& exs, ostream& os) {
  uint k;
  FOR1D(exs, k) {
            os << "[" << k << "] ("<<(k+1)<<"/"<<exs.N<<")"<<endl;
    exs(k)->write(os);
  }
}


}  // namespace PRADA



std::ostream& operator<<(std::ostream& os, const relational::SymbolicState& s) {
  s.write(os); return os;
}


std::ostream& operator<<(std::ostream& os, const relational::Literal& l) {
  l.write(os); return os;
}


std::ostream& operator<<(std::ostream& os, const relational::StateTransition& e) {
  e.write(os); return os;
}


std::ostream& operator<<(std::ostream& os, const relational::LitL& lits) {
  write(lits, os);
  return os;
}


std::ostream& operator<<(std::ostream& os, const relational::StateTransitionL& sl) {
  write(sl, os);
  return os;
}
