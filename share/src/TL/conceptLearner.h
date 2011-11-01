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

#ifndef TL__CONCEPT_LEARNER
#define TL__CONCEPT_LEARNER

#define NEW_PREDS_PER_ROUND 5
#define NEW_FUNCS_PER_ROUND 2


#include <TL/ruleLearner.h>


class PredicateInventor {
    
    public:
        virtual ~PredicateInventor() {}

        virtual void invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates) = 0;
};


class Combine_0_0 : public PredicateInventor {
    public:
        void invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates);
};

class Combine_0_01 : public PredicateInventor {
    public:
        void invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates);
};

class Combine_0_10 : public PredicateInventor {
    public:
        void invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates);
};

class Combine_01_01 : public PredicateInventor {
    public:
        void invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates);
};

class Combine_01_10 : public PredicateInventor {
    public:
        void invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates);
};

class Combine_01_00 : public PredicateInventor {
    public:
        void invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates);
};

class Combine_01_11 : public PredicateInventor {
    public:
        void invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates);
};



class CheckColleagueUnary : public PredicateInventor {
    public:
        void invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates);
};

class CheckColleagueBinary_front : public PredicateInventor {
    public:
        void invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates);
};

class CheckColleagueBinary_back : public PredicateInventor {
    public:
        void invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates);
};



class BuildTransitiveClosure : public PredicateInventor {
    public:
        void invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates);
};




class FunctionInventor {
    
    public:
        virtual ~FunctionInventor() {}

        virtual void invent(const PredL& predicates, const FuncL& functions, FuncL& inventedFunctions) = 0;
};


class BuildCountFunction : public FunctionInventor {
    public:
        void invent(const PredL& predicates, const FuncL& functions, FuncL& inventedFunctions);
};





class ConceptLearner {
    
    double rl_alpha_PEN;
    double rl_p_min;
    uint rl_param_opt_type;
    double rl_weight_c1;
    double rl_weight_c2;
    uint rl_so_choice_type;
    
    MT::Array< TL::Trial* > data;
    MT::Array<PredicateInventor*> p_inventors;
    MT::Array<FunctionInventor*> f_inventors;
    
    uint noConcepts(TL::RuleSet& rs, FuncL& usedDerivedFunctions, PredL& usedDerivedPredicates);
    
    public:
        ConceptLearner(double rl_alpha_PEN, double rl_p_min, uint rl_param_opt_type, double rl_weight_c1, double rl_weight_c2, uint rl_so_choice_type);
    
        void addData(MT::Array< TL::Trial* >& data);
    
        double learnLanguage(PredL& actions, PredL& predicates_prim, PredL& predicates_comp, FuncL& functions_prim, PredL& predicates_out, FuncL& functions_out, TL::RuleSet& rs);
    
//     double score(Ruleset& rs);
};


#endif
