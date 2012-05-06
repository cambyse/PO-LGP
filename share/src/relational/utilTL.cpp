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

#include "utilTL.h"

double REPLACE_SIZE(double val) {
  // Blocks
  if (TL::areEqual(val,0.04)) return 1.0;
  if (TL::areEqual(val,0.06)) return 2.0;
  if (TL::areEqual(val,0.08)) return 3.0;
  
  // Balls
  if (TL::areEqual(val,0.03)) return 1.0;
  if (TL::areEqual(val,0.045)) return 2.0;
  
  // Table
  if (val > 0.9)
    return 4.0;
  
  // TODO
  return 5.0;
  if (TL::areEqual(val,0.1)) return 5.0; 
  
  HALT("unknown size "<<val);
  return -100000.;
}


// with or without repeat

struct Memory_allPossibleLists {
  MT::Array< MT::Array< uintA > > list_arguments;
  MT::Array< MT::Array< MT::Array< uintA > > > list_lists;  // 0 - length,  1 - arguments,  2 - lists
  
  Memory_allPossibleLists() {
    list_arguments.resize(20);
    list_lists.resize(20);
  }
  
  bool get(MT::Array< uintA >& lists, const uintA& arguments, uint length) {
    uint i;
    FOR1D(list_arguments(length), i) {
      if (list_arguments(length)(i) == arguments) {
        lists = list_lists(length)(i);
        return true;
      }
    }
    return false;
  }
  
  void append(MT::Array< uintA >& lists, const uintA& arguments, uint length) {
    list_arguments(length).append(arguments);
    list_lists(length).append(lists);
  }
};


Memory_allPossibleLists mem__withRepeat_returnEmpty;
Memory_allPossibleLists mem__withRepeat_dontReturnEmpty;
Memory_allPossibleLists mem__withoutRepeat_returnEmpty;
Memory_allPossibleLists mem__withoutRepeat_dontReturnEmpty;



void TL::allPossibleLists(MT::Array< uintA >& lists, const uintA& arguments, uint length, bool withRepeat, bool returnEmpty) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"allPossibleListsWithRepeat [START]"<<endl;
  if (DEBUG>0) {PRINT(arguments); PRINT(length); PRINT(withRepeat);}
  
  lists.clear();
  // Try to get from memory
  bool foundInMemory = false;
  if (withRepeat && returnEmpty) {
    foundInMemory = mem__withRepeat_returnEmpty.get(lists, arguments, length);
  }
  else if (withRepeat && !returnEmpty) {
    foundInMemory = mem__withRepeat_dontReturnEmpty.get(lists, arguments, length);
  }
  else if (!withRepeat && returnEmpty) {
    foundInMemory = mem__withoutRepeat_returnEmpty.get(lists, arguments, length);
  }
  else {
    foundInMemory = mem__withoutRepeat_dontReturnEmpty.get(lists, arguments, length);
  }
  if (foundInMemory) {
    if (DEBUG>0) {
      cout<<"found in memory:"<<endl;
      uint i;
      FOR1D(lists, i) {PRINT(lists(i));}
    }
    if (DEBUG>0) cout<<"allPossibleListsWithRepeat [END]"<<endl;
    return;
  }
  
  if (DEBUG>0) {cout<<"*** Creating new lists ***"<<endl;}
  
  
  lists.clear();
  if (length == 0) {
    if (returnEmpty) {
      uintA empty(0);
      lists.append(empty);
      if (DEBUG>0) cout<<"allPossibleListsWithRepeat [END]"<<endl;
      return;
    }
    else {
      MT_MSG("No lists returned!");
      return;
    }
  }
  uint id=0;
  uint a, i, _pow;
  // beim change ganz nach hinten rutschen und wieder alle zuruecksetzen
  while (id < pow(arguments.N, length)) {
    if (DEBUG>2) {PRINT(length);  PRINT(arguments.N);  PRINT(pow(arguments.N, length));}
    uintA nextList(length);
    i = id;
    for (a=length; a>0; a--) {
      _pow = (uint) pow(arguments.N, a-1);
      nextList(a-1) = arguments(i / _pow);
      // Auf gar keinen Fall hier die Reihenfolge aendern wollen, in der Liste bestueckt wird.
      // Andere Methoden nutzen explizit die Reihenfolge aus, in der's von links nach rechts variiert.
      i = i % _pow;
    }
    id++;
    if (withRepeat)
      lists.append(nextList);
    else {
      if (!nextList.containsDoubles())  // teuer!
	lists.append(nextList);
    }
  }
  if (DEBUG>0) {
    FOR1D(lists, i) {PRINT(lists(i))}
  }
  
  // Add to memory
  if (withRepeat && returnEmpty) {
    mem__withRepeat_returnEmpty.append(lists, arguments, length);
  }
  else if (withRepeat && !returnEmpty) {
    mem__withRepeat_dontReturnEmpty.append(lists, arguments, length);
  }
  else if (!withRepeat && returnEmpty) {
    mem__withoutRepeat_returnEmpty.append(lists, arguments, length);
  }
  else {
    mem__withoutRepeat_dontReturnEmpty.append(lists, arguments, length);
  }
  
  if (DEBUG>0) cout<<"allPossibleListsWithRepeat [END]"<<endl;
}


// different arguments
void TL::allPossibleLists(MT::Array< uintA >& lists, const MT::Array< uintA >& arguments_lists, bool returnEmpty)  {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"allPossibleLists [START]"<<endl;}
  if (DEBUG>0) {PRINT(arguments_lists);  PRINT(returnEmpty);}
  lists.clear();
  if (arguments_lists.N == 0) {
    if (returnEmpty) {
      uintA empty(0);
      lists.append(empty);
//       if (DEBUG>0) cout<<"allPossibleListsWithRepeat [END]"<<endl;
      return;
    }
    else {
      MT_MSG("No lists returned!");
      return;
    }
  }
  uint id=0;
  uint a, i, k, _num_configs;
  // beim change ganz nach hinten rutschen und wieder alle zuruecksetzen
  uint num_configs = 1;
  FOR1D(arguments_lists, i) {num_configs *= arguments_lists(i).N;}
  if (DEBUG>0) {PRINT(arguments_lists.N);  PRINT(num_configs);}
  while (id < num_configs) {
    uintA nextList(arguments_lists.N);
    i = id;
    for (a=arguments_lists.N; a>0; a--) {
      _num_configs = 1;
      for (k=a; k>1; k--) {_num_configs *= arguments_lists(k-2).N;}
      nextList(a-1) = arguments_lists(a-1)(i / _num_configs);
	// Auf gar keinen Fall hier die Reihenfolge aendern wollen, in der Liste bestueckt wird.
	// Andere Methoden nutzen explizit die Reihenfolge aus, in der's von links nach rechts variiert.
      i = i % _num_configs;
    }
    lists.append(nextList);
    id++;
  }
  
  if (DEBUG>0) {PRINT(lists);}
  if (DEBUG>0) {cout<<"allPossibleLists [END]"<<endl;}
}


void TL::allSubsets(MT::Array< uintA >& lists, const uintA& elements, bool trueSubsets, bool withEmpty) {
  uint DEBUG = 0;
  uint length;
  uint max_length = elements.N;
  uint i, k;
  if (trueSubsets)
    max_length -= 1;
  // others
  for (length=1; length<=max_length; length++) {
    MT::Array< uintA > local_lists;
    allPossibleLists(local_lists, elements, length, false, false);
    FOR1D(local_lists, i) {
      for (k=0; k<local_lists(i).N-1; k++) {
        if (local_lists(i)(k) >= local_lists(i)(k+1))
          break;
      }
      if (k == local_lists(i).N-1)
        lists.append(local_lists(i));
    }
  }
  // empty list
  if (withEmpty) {
    uintA empty;
    lists.append(empty);
  }
  
  if (DEBUG > 0) {
    cout<<"allSubsets:"<<endl;
    PRINT(elements);
    PRINT(trueSubsets);
    PRINT(withEmpty);
    cout<<"Result:"<<endl;
    PRINT(max_length);
    PRINT(lists);
  }
}

void TL::allSubsets(MT::Array< uintA >& lists, const uintA& elements, uint length) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"allSubsets [START]"<<endl;}
  if (DEBUG>0) {
    PRINT(elements);
    PRINT(length);
  }
  CHECK(length<=elements.N, "too long subset size");
  if (length==0) {
    uintA empty;
    lists.append(empty);
    return;
  }
  uint i, k, l;
  uintA marker(length);
  FOR1D(marker, i) {marker(i)=i;}
  bool change;
  while(true) {
    uintA list;
    FOR1D(marker, i) {
      list.append(elements(marker(i)));
    }
    if (DEBUG>0) {PRINT(marker);cout<<"New list: "<<list<<endl;}
    lists.append(list);
    change = false;
    FOR1D_DOWN(marker, k) {
      if (marker(k) < elements.N-(length-k)) {
        marker(k)++;
        for(l=k+1;l<marker.N;l++)
          marker(l)=marker(k)+l-k;
        change = true;
        break;
      }
    }
    if (!change)
      break;
  }
  if (DEBUG>0) {cout<<"All subsets: "<<lists<<endl;}
  if (DEBUG>0) {cout<<"allSubsets [END]"<<endl;}
}

bool TL::containsAllElements(const uintA& superlist, const uintA& list) {
  uint i;
  FOR1D(list, i) {
    if (superlist.findValue(list(i)) < 0)
      return false;
  }
  return true;
}


uint TL::basic_sample(const arr& weights) {
    arr probs_normalized = weights;
    if (!TL::areEqual(sum(probs_normalized), 1.0)) {
        normalizeDist(probs_normalized);
    }
    double rndNum = rnd.uni();
    double massSoFar = 0.0;
    uint i;
    FOR1D(weights, i) {
      massSoFar += probs_normalized(i);
      if (massSoFar >= rndNum)
        break;
    }
    CHECK(weights.N > i, "sampling from dist failed");
    return i;
}


// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
//    SORT


void TL::sort_desc(arr& sorted, uintA& sortedIndices, const arr& unsorted) {
  const double LOCAL_MIN = -333333.3;
  sortedIndices.clear();
  sorted.clear();
  arr un_copy = unsorted;
  uint i, k;
  double max;
  uint max_id;
  for (i=0; i<un_copy.N; i++) {
    max = LOCAL_MIN;
    max_id = TL::UINT_NIL;
    FOR1D(un_copy, k) {
      if (un_copy(k) > max) {
        max = un_copy(k);
        max_id = k;
      }
    }
    sorted.append(max);
    sortedIndices.append(max_id);
    un_copy(max_id) = LOCAL_MIN;
  }
  CHECK(sorted.N == unsorted.N, "");
}


void TL::sort_desc(uintA& sorted, uintA& sortedIndices, const uintA& unsorted) {
  sortedIndices.clear();
  sorted.clear();
  uint rank;
  uint i;
  uint maxID;
  boolA used(unsorted.N);
  used.setUni(false);
  for(rank=0; rank<unsorted.N; rank++) {
    maxID=0;
    while(used(maxID))  // initial set of maxID (= kleinstes verbleibendes Element)
      maxID++;
    for(i=maxID+1; i<unsorted.N; i++) {
      if (used(i))
        continue;
      if (unsorted(maxID) < unsorted(i))
        maxID = i;
    }
    sortedIndices.append(maxID);
    sorted.append(unsorted(maxID));
    used(maxID) = true;
  }
  CHECK(sorted.N == unsorted.N, "");
}

void TL::sort_desc(uintA& values) {
  uintA sorted, sortedIndices;
  sort_desc(sorted, sortedIndices, values);
  values = sorted;
}

void TL::sort_desc_keys(uintA& sortedIndices, const uintA& unsorted) {
  uintA sorted;
  sort_desc(sorted, sortedIndices, unsorted);
}

void TL::sort_desc_keys(uintA& sortedIndices, const arr& unsorted) {
  arr sorted;
  sort_desc(sorted, sortedIndices, unsorted);
}



void TL::sort_asc(uintA& values) {
  sort_desc(values);
  values.reverse();
}

void TL::sort_asc(uintA& sorted, const uintA& unsorted) {
  uintA sortedIndices;
  sort_asc(sorted, sortedIndices, unsorted);
}

void TL::sort_asc(uintA& sorted, uintA& sortedIndices, const uintA& unsorted) {
  sort_desc(sorted, sortedIndices, unsorted);
  sorted.reverse();
  sortedIndices.reverse();
}

void TL::sort_asc(arr& sorted, uintA& sortedIndices, const arr& unsorted) {
  sort_desc(sorted, sortedIndices, unsorted);
  sorted.reverse();
  sortedIndices.reverse();
}

void TL::sort_asc_keys(uintA& sortedIndices, const uintA& unsorted) {
  sort_desc_keys(sortedIndices, unsorted);
  sortedIndices.reverse();
}











// some testing wouldn't be all too bad
bool TL::isAcyclic(boolA adjMatrix) {
  // eliminate successively nodes without outgoing or without incoming 
  // edges since these cannot be part of a cycle
  CHECK(adjMatrix.nd==2, "adjMatrix must be square")
  CHECK(adjMatrix.d0==adjMatrix.d1, "adjMatrix must be square")
  boolA endangeredNodes(adjMatrix.d0);
  endangeredNodes.setUni(true);
  bool change;
  uint n_from, n_to;
  do {
    change=false;
//         PRINT(endangeredNodes);
      // without outgoing
      for(n_from=0; n_from<endangeredNodes.d0; n_from++) {
          if (!endangeredNodes(n_from)) continue;
          for (n_to=0; n_to<endangeredNodes.d0; n_to++) {
              if (!endangeredNodes(n_to)) continue;
              else if (adjMatrix(n_from, n_to)) break;
          }
          if (n_to == endangeredNodes.d0) {
              endangeredNodes(n_from) = false;
              change = true;
          }
      }
      // without incoming
      for(n_to=0; n_to<endangeredNodes.d0; n_to++) {
          if (!endangeredNodes(n_to)) continue;
          for (n_from=0; n_from<endangeredNodes.d0; n_from++) {
              if (!endangeredNodes(n_from)) continue;
              else if (adjMatrix(n_from, n_to)) break;
          }
          if (n_from == endangeredNodes.d0) {
              endangeredNodes(n_to) = false;
              change = true;
          }
      }
    } while (change);
    bool cyclic = sum(endangeredNodes);
    return !cyclic;
}



double TL::getcputime() {
  struct timeval tim;
  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  tim=ru.ru_utime;
  double t=(double)tim.tv_sec + (double)tim.tv_usec / 1000000.0;
  tim=ru.ru_stime;
  t+=(double)tim.tv_sec + (double)tim.tv_usec / 1000000.0;
  return t;
}

uint TL::getIndex(const uintA& constants, const uintA& args) {
  uint args_idx=0;
  uint i;
  FOR1D(args, i) {
    args_idx += ((uint) pow(constants.N, i)) * constants.findValue(args(i));
  }
//   cout<<"getIndex: constants="<<constants<<"  args="<<args<<"    args_idx="<<args_idx<<endl;
  return args_idx;
}






//===========================================================================
//
// Rprop
//

int _sgn(double x){ if (x > 0) return 1; if (x < 0) return -1; return 0; }
double _mymin(double x,double y){ return x < y ? x : y; }
double _mymax(double x,double y){ return x > y ? x : y; }


Rprop::Rprop(){
  incr   = 1.2;
  decr   = .33;
  dMax = 50;
  dMin = 1e-6;
  rMax = 0;
  delta0 = 1.;
}

void Rprop::init(double _delta0){
  stepSize.resize(0);
  lastGrad.resize(0);
  delta0 = _delta0;
}

bool Rprop::done(){
  double maxStep = stepSize(stepSize.maxIndex());
  return maxStep < incr*dMin;
}

void Rprop::step(double& w,const double& grad){
  static arr W,GRAD;
  W.referTo(&w,1); GRAD.referTo(&grad,1);
  step(W,GRAD);
}

void Rprop::step(arr& w,const arr& grad,uint *singleI){
  if(!stepSize.N){ //initialize
    stepSize.resize(w.N);
    lastGrad.resize(w.N);
    lastGrad.setZero();
    stepSize = delta0;
  }
  CHECK(grad.N==stepSize.N,"Rprop: gradient dimensionality changed!");
  CHECK(w.N==stepSize.N   ,"Rprop: parameter dimensionality changed!");

  uint i=0,I=w.N;
  if(singleI){ i=*(singleI); I=i+1; }
  for(;i<I;i++){
    if(grad.elem(i) * lastGrad(i) > 0){        //same direction as last time
      if(rMax) dMax=fabs(rMax*w.elem(i));
      stepSize(i) = _mymin(dMax, incr * stepSize(i)); //increase step size
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = grad.elem(i);                    //memorize gradient
    }else if(grad.elem(i) * lastGrad(i) < 0){  //change of direction
      stepSize(i) = _mymax(dMin, decr * stepSize(i)); //decrease step size
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = 0;                               //memorize to continue below next time
    }else{                                     //after change of direcion
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction 
      lastGrad(i) = grad.elem(i);                    //memorize gradient
    }
  }
}


