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

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//  ATTENTION
//  This file needs significant revision!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#if 0


#include "conceptLearner.h"
#include "TL/ruleReasoning.h"
#include <float.h>
#define PEN_ALPHA_CONCEPTS 0.5


bool my_local_contains(const PredL& pa, const TL::Predicate& p) {
    uint i;
    CHECK(p.category == category_derived, "");
    FOR1D(pa, i) {
        if ((*pa(i)) == p)
            return true;
    }
    return false;
}

bool my_local_contains(const FuncL& fa, const TL::Function& f) {
    uint i;
    FOR1D(fa, i) {
        if ((*fa(i)) == f)
            return true;
    }
    return false;
}


ConceptLearner::ConceptLearner(double rl_alpha_PEN, double rl_p_min, uint rl_param_opt_type, double rl_weight_c1, double rl_weight_c2, uint rl_so_choice_type) {
    this->rl_alpha_PEN = rl_alpha_PEN;
    this->rl_p_min = rl_p_min;
    this->rl_param_opt_type = rl_param_opt_type;
    this->rl_weight_c1 = rl_weight_c1;
    this->rl_weight_c2 = rl_weight_c2;
    this->rl_so_choice_type = rl_so_choice_type;
    
    p_inventors.append(new Combine_0_0());
    p_inventors.append(new Combine_0_01());
    p_inventors.append(new Combine_0_10());
    p_inventors.append(new Combine_01_01());
    p_inventors.append(new Combine_01_10());
    p_inventors.append(new Combine_01_00());
    p_inventors.append(new Combine_01_11());
    p_inventors.append(new CheckColleagueUnary());
    p_inventors.append(new CheckColleagueBinary_front());
    p_inventors.append(new CheckColleagueBinary_back());
    p_inventors.append(new BuildTransitiveClosure());

    
    f_inventors.append(new BuildCountFunction());
}


    
void ConceptLearner::addData(MT::Array< TL::Trial* >& data1) {
    this->data.append(data1);
}

uint ConceptLearner::noConcepts(TL::RuleSet& rs, FuncL& usedDerivedFunctions, PredL& usedDerivedPredicates) {
    uint r, p;
    uintA ids_p;
    FOR1D_(rs, r) {
        FOR1D(rs.elem(r)->preconditions, p) {
//             rs.elem(r)->preconditions(p)->writeNice(cout);cout<<"   ";rs.elem(r)->preconditions(p)->write(cout);cout<<endl;
            if (rs.elem(r)->preconditions(p)->pred->category == category_derived) {
                TL::DerivedPredicate* dp = dynamic_cast<TL::DerivedPredicate*>(rs.elem(r)->preconditions(p)->pred);
                CHECK(dp!=NULL, "cast failed");
                usedDerivedPredicates.setAppend(dp);
            }
            else if (rs.elem(r)->preconditions(p)->pred->type == predicate_comparison) {
                TL::ComparisonPredicateInstance* pt = dynamic_cast<TL::ComparisonPredicateInstance*>(rs.elem(r)->preconditions(p));
//                 cout<<"Used function: ";pt->writeNice(cout);cout<<endl;
                CHECK(pt!=NULL, "cast failed");
                if (pt->f->category == category_derived) {
                    usedDerivedFunctions.setAppend(pt->f);
                }
            }
        }
    }
    return usedDerivedFunctions.N + usedDerivedPredicates.N;
}



double ConceptLearner::learnLanguage(PredL& actions, PredL& predicates_prim, PredL & predicates_comp, FuncL& functions_prim, PredL& predicates_out, FuncL& functions_out, TL::RuleSet& rs) {
    uint DEBUG = 2;
    if (DEBUG>0) {cout<<"learnLanguage [START]"<<endl;}
    uint i;
    
    FuncL fs_best;
    fs_best.append(functions_prim);
    PredL ps_best;
    ps_best.append(predicates_prim);
    FOR1D(predicates_comp, i) {ps_best.append(predicates_comp(i));}
    
    FuncL fs_d_best;
    PredL ps_d_best;
    
    PredL ps_discarded;
    MT::Array< TL::Function* > fs_discarded;
    
    double bestScore = -1e10;
    double score;
    int round = 0;
    
    // determine stard ID for derived concepts
    uint p_derivedStartID=0, f_derivedStartID=0;
    FOR1D(predicates_prim, i) {
        if (predicates_prim(i)->id >= p_derivedStartID)
            p_derivedStartID = predicates_prim(i)->id+1;
    }
    FOR1D(predicates_comp, i) {
        if (predicates_comp(i)->id >= p_derivedStartID)
            p_derivedStartID = predicates_comp(i)->id+1;
    }
    FOR1D(functions_prim, i) {
        if (functions_prim(i)->id >= f_derivedStartID)
            f_derivedStartID = functions_prim(i)->id+1;
    }
    
    ofstream scores_log;
    scores_log.open("scores.log");
    
    do {
        if (DEBUG>1) {
            cout<<endl<<endl;
            cout << "==================================================="<<endl;
            cout << "==================================================="<<endl;
            cout << "==================================================="<<endl;
            cout << "      CONCEPT LEARNING ROUND " << round << endl;
            cout << "---------------------------------------------------"<<endl;
        }
        
        if (DEBUG>1) {
            cout << "--- So far best derived ---"<<endl;
            cout<<"Predicates:"<<endl;
            TL::writeNice(ps_d_best);
            cout << "Functions:"<<endl;
            TL::writeNice(fs_d_best);
        }
        
        FuncL fs_d_current = fs_d_best;
        PredL ps_d_current = ps_d_best;

        // don't invent in first round
        if (round>0) { 
//         if (true) { // TODO das hier zuruecknehmen!
            // Invent concepts
            PredL ps_invented, ps_invented_all;
            FuncL fs_invented, fs_invented_all;
            FOR1D(p_inventors, i) {
                ps_invented.clear();
                p_inventors(i)->invent(ps_best, fs_best, ps_invented);
                ps_invented_all.append(ps_invented);
            }
            FOR1D(f_inventors, i) {
                fs_invented.clear();
                f_inventors(i)->invent(ps_best, fs_best, fs_invented);
                fs_invented_all.append(fs_invented);
            }
            // Filter for (1) discarded ones and
            //  (2) ones which are already in best and thus are already used.
            //  (in each round, concepts are invented from scratch; after the rule-learning,
            //  invented concepts will either land in discarded or in best)
            FuncL fs_invented_all_filtered;
            FOR1D_DOWN(fs_invented_all, i) {
                if (!my_local_contains(fs_discarded, *fs_invented_all(i))
                     &&  !my_local_contains(fs_d_best, *fs_invented_all(i)) )
                    fs_invented_all_filtered.append(fs_invented_all(i));
                else
                    delete fs_invented_all(i);
            }
            PredL ps_invented_all_filtered;
            FOR1D(ps_invented_all, i) {
                if (!my_local_contains(ps_discarded, *ps_invented_all(i))
                    &&  !my_local_contains(ps_d_best, *ps_invented_all(i)) )
                    ps_invented_all_filtered.append(ps_invented_all(i));
                else
                    delete ps_invented_all(i);
//                 else {cout << "not ";ps_invented_all(i)->writeNice(cout);cout<<endl;}
            }
            if (DEBUG>2) {
                if (DEBUG>3) {
                    cout<<"--- Discarded predicates "<<ps_discarded.N<<" ---"<<endl;
                    TL::writeNice(ps_discarded);
                    cout<<"--- All invented predicates "<<ps_invented_all.N<<" ---"<<endl;
                    TL::writeNice(ps_invented_all);
                }
                cout<<"--- Newly invented predicates "<<ps_invented_all_filtered.N<<" ---"<<endl;
                TL::writeNice(ps_invented_all_filtered);
                cout<<endl;
                if (DEBUG>3) {
                    cout<<"--- Discarded functions "<<fs_discarded.N<<" ---"<<endl;
                    TL::writeNice(fs_discarded);
                    cout<<"--- All invented functions "<<fs_invented_all.N<<" ---"<<endl;
                    TL::writeNice(fs_invented_all);
                }
                cout<<"--- Newly invented functions "<<fs_invented_all_filtered.N<<" ---"<<endl;
                TL::writeNice(fs_invented_all_filtered);
            }
            
            if (ps_invented_all_filtered.N==0 && fs_invented_all_filtered.N==0) {
                if (DEBUG>1) cout<<"Uuuh, I lost all creativity and can't find any new concepts. So I'd rather stop that business now."<<endl;
                break;
            }
            
            // Randomly choose some invented concepts
            uint randNum;
            i=0;
            if (ps_invented_all_filtered.N < NEW_PREDS_PER_ROUND) {
                FOR1D(ps_invented_all_filtered, i) {ps_d_current.append(ps_invented_all_filtered(i));}
            }
            else {
                while (i<NEW_PREDS_PER_ROUND) {
                    randNum = rnd.num(ps_invented_all_filtered.N);
                    if (ps_d_current.findValue(ps_invented_all_filtered(randNum)) < 0) {
                        ps_d_current.append(ps_invented_all_filtered(randNum));
                        i++;
                    }
                }
            }
            i=0;
            if (fs_invented_all_filtered.N < NEW_FUNCS_PER_ROUND) {
                FOR1D(fs_invented_all_filtered, i) {fs_d_current.append(fs_invented_all_filtered(i));}
            }
            else {
                while (i<NEW_FUNCS_PER_ROUND) {
                    randNum = rnd.num(fs_invented_all_filtered.N);
                    if (fs_d_current.findValue(fs_invented_all_filtered(randNum)) < 0) {
                        fs_d_current.append(fs_invented_all_filtered(randNum));
                        i++;
                    }
                }
            }
            
            // set ids sensibly
            FOR1D(ps_d_current, i) {
                ps_d_current(i)->id = p_derivedStartID + i;
            }
            FOR1D(fs_d_current, i) {
                fs_d_current(i)->id = f_derivedStartID + i;
            }
        }
        
        if (DEBUG>1) {
            cout << "--- Now learning with ---"<<endl;
            cout<<"Predicates: "<<endl;
            TL::writeNice(predicates_prim);
            TL::writeNice(predicates_comp);
            TL::writeNice(ps_d_current);
            cout<<"Functions: "<<endl;
            TL::writeNice(functions_prim);
            TL::writeNice(fs_d_current);
        }
        
        // set up LOGIC ENGINE
        TL::LogicEngine le(data(0)->constants, predicates_prim, ps_d_current, predicates_comp, functions_prim, fs_d_current, actions);

        // tell states to be newly derived
        uint s;
        FOR1D(data, i) {
            FOR1D(data(i)->states, s)
                TL::LogicEngine::dederive(data(i)->states(s));
        }
        
        // filter predicates that are always either true or false
        PredL stupidPredicates;
        FuncL stupidFunctions;
        // and deletes them!
        le.determineStupidDerivedConcepts(data, stupidPredicates, stupidFunctions, true);
        ps_discarded.append(stupidPredicates);
        fs_discarded.append(stupidFunctions);
        if (DEBUG>1) {
            cout << "--- Stupid and abandoned predicates ---"<<endl;
            TL::writeNice(stupidPredicates);
            cout << "--- Stupid and abandoned functions ---"<<endl;
            TL::writeNice(stupidFunctions);
        }
        if (stupidPredicates.N>0) {
            // tell states to be newly derived
            uint s;
            FOR1D(data, i) {
                FOR1D(data(i)->states, s) {
                    TL::LogicEngine::dederive(data(i)->states(s));
                }
            }
        }
        
        // prepare its own copy of the data
        MT::Array< TL::Trial* > data_copy;
        FOR1D(data, i) {
          NIY;
//             data_copy.append(data(i)->deep_copy());
            le.makeOriginal(*data_copy.last());
            // --> data will be deleted by the destructor of logic engine
        }

        // RULE LEARNING
        MT_MSG("Using the same noise outcome successor state probability for both normal and noisy-default rule!!");
        RuleLearner rl(&le, rl_alpha_PEN, rl_p_min, rl_p_min, rl_param_opt_type, rl_weight_c1, rl_weight_c2, rl_so_choice_type);
        FOR1D(data_copy, i) {
            rl.addExamples(data_copy(i)->states, data_copy(i)->actions);
        }
        TL::RuleSetContainer rulesC;
        MT::String logfile_name;
        logfile_name << "rl." << round << ".log";
        rl.learn_rules(rulesC, logfile_name);
        TL::RuleSet ruleSet = rulesC.rules;
        PredL ps_d_used;
        FuncL fs_d_used; 
        score = rl.score(rulesC, TL::TL_DOUBLE_MIN) - PEN_ALPHA_CONCEPTS * noConcepts(ruleSet, fs_d_used, ps_d_used);
        scores_log<<round<<" "<<score<<endl;
        if (DEBUG>1) {
            cout << "--- Learned rule-set ---"<<endl;
            TL::RuleEngine::writeNice(ruleSet, cout);
            cout << "--- Used derived ---"<<endl;
            cout<<"Predicates: "<<endl;
            TL::writeNice(ps_d_used);
            cout<<"Functions: "<<endl;
            TL::writeNice(fs_d_used);
            cout << "--- Score ---"<<endl;
            cout<<"Achieved score: "<<score;
            if (score > bestScore) cout<<" --> Improvement!";
            cout<<endl;
        }
        
        MT::String rule_file_name;
        rule_file_name<< "rules."<<round<<".dat";
        writeRules(ruleSet, rule_file_name);
        
        MT::String rule_file_nice_name;
        rule_file_nice_name<<"rules."<<round<<".nice.dat";
        ofstream ruleset_nice;
        ruleset_nice.open(rule_file_nice_name);
        ruleSet.writeNice(ruleset_nice);
        ruleset_nice.close();
        
        
        // POST-PROCESSING
        if (score > bestScore) {
            bestScore = score;
            rs = ruleSet;
//             // TODO
//             rs.writeNice();
            // derived best arrays
            fs_d_best.clear();
            fs_d_best.append(fs_d_used);
            ps_d_best.clear();
            ps_d_best.append(ps_d_used);
            FOR1D(ps_d_used, i) {
                PredL ps_pre;
                FuncL fs_pre;
                le.getAllPrecessors(*ps_d_used(i), ps_pre, fs_pre);
                uint k;
                FOR1D(fs_pre, k) {
                    if (fs_pre(k)->category == category_derived) {
                        fs_d_best.setAppend(fs_pre(k));
                    }
                }
                FOR1D(ps_pre, k) {
                    if (ps_pre(k)->category == category_derived) {
                        TL::DerivedPredicate* dp = dynamic_cast<TL::DerivedPredicate*>(ps_pre(k));
                        CHECK(dp!=NULL,"cast failed");
                        ps_d_best.setAppend(dp);
                    }
                }
            }
            // all best arrays
            fs_best.clear();//
            fs_best.append(functions_prim);
            fs_best.append(fs_d_best);
            ps_best.clear();
            ps_best.append(predicates_prim);
            FOR1D(predicates_comp, i) {ps_best.append(predicates_comp(i));}
            FOR1D(ps_d_best, i) {ps_best.append(ps_d_best(i));}
        }
        // TODO hard break for debugging
//         if (round >= 0)//
//           break;
        
        // Filter out those concepts to discarded which are not in best now.
        FOR1D(ps_d_current, i) {
            if (ps_d_best.findValue(ps_d_current(i)) < 0) {
                ps_discarded.setAppend(ps_d_current(i));
            }
        }
        FOR1D(fs_d_current, i) {
            if (fs_d_best.findValue(fs_d_current(i)) < 0) {
                fs_discarded.setAppend(fs_d_current(i));
            }
        }
        
        if (DEBUG>1) {
            if (score > bestScore) {
                cout<<"New best derived concepts:"<<endl;
                TL::writeNice(ps_d_best);
                TL::writeNice(fs_d_best);
            }
            cout<<endl;
            if (DEBUG>2) {
                cout<<"Discarded concepts:"<<endl;
                TL::writeNice(ps_discarded);
                TL::writeNice(fs_discarded);
            }
        }
        
    } while (++round);
    
    scores_log.close();
    
    predicates_out = ps_best;
    functions_out = fs_best;
    
    if (DEBUG>0) {cout<<"learnLanguage [END]"<<endl;}
    
    // TODO DAS FUNKTIONIERT NICHT!!!
    // Der loescht irgendwo schon rum, obwohl er gar nicht soll, der depp!
    // Das urspruengliche RuleSet ruleSet wird geloescht im Block oben
    // und dabei wird der Inhalt von rs mitgeloescht. Keine Ahnung, wieso.
//     rs.writeNice();
    
    return bestScore;
}




// =======================================================
// =======================================================
//  PredicateInventors
// =======================================================
// =======================================================


TL::ConjunctionPredicate* generate_single_SCP_twoBases(uint arity, uintA& basePreds_mapVars2conjunction, TL::Predicate* p1, TL::Predicate* p2, bool true1, bool true2, bool freeAllQuantified, const char* prefix) {
    TL::ConjunctionPredicate* scp  = new TL::ConjunctionPredicate;
    scp->d = arity;
    MT::String new_name;
    new_name << prefix;
    new_name << "[";
    if (!true1)
        new_name << "N";
    new_name << p1->name;
    new_name << "]";
    new_name << "[";
    if (!true2)
        new_name << "N";
    new_name << p2->name;
    new_name << "]";
    scp->name = new_name;
    scp->basePreds.resize(2);
    scp->basePreds(0) = p1;
    scp->basePreds(1) = p2;
    scp->basePreds_positive.resize(2);
    scp->basePreds_positive(0) = true1;
    scp->basePreds_positive(1) = true2;
    scp->basePreds_mapVars2conjunction = basePreds_mapVars2conjunction;
    scp->freeVarsAllQuantified = freeAllQuantified;
//     cout<<"Created: ";scp->writeNice(cout);cout<<endl;
    return scp;
}


void generateSCP_twoBases(PredL& ps, uint arity, TL::Predicate* p1, TL::Predicate* p2, uintA& baseArgs, bool freeAllQuantified, const char* prefix) {
    ps.clear();
    // pos_pos
    ps.append(generate_single_SCP_twoBases(arity, baseArgs, p1, p2, true, true, freeAllQuantified, prefix));
    // pos_neg
    ps.append(generate_single_SCP_twoBases(arity, baseArgs, p1, p2, true, false, freeAllQuantified, prefix));
    // neg_pos
    ps.append(generate_single_SCP_twoBases(arity, baseArgs, p1, p2, false, true, freeAllQuantified, prefix));
    // ALT:
//     // we want positives always first, so change predicate order
//     uintA baseArgs_rev;
//     uint i;
//     for (i=0; i<p2->d; i++) {baseArgs_rev.append(baseArgs(p1->d+i));}
//     for (i=0; i<p1->d; i++) {baseArgs_rev.append(baseArgs(i));}
//     ps.append(generate_single_SCP_twoBases(arity, baseArgs_rev, p2, p1, true, false, freeAllQuantified, prefix));

    // neg_neg
    ps.append(generate_single_SCP_twoBases(arity, baseArgs, p1, p2, false, false, freeAllQuantified, prefix));
    
}


void generateSCP_twoBases(PredL& ps_invented, uint arity, uint arity1, uint arity2, const PredL& ps_old, uintA& baseArgs, bool freeAllQuantified, const char* prefix) {
    CHECK(baseArgs.N == arity1+arity2, "insufficient arguments")
            PredL pos_pos, neg_pos, pos_neg, neg_neg;
    uint i, k;
    boolA usedCombos(ps_old.N, ps_old.N);
    usedCombos.setUni(false);
    FOR1D(ps_old, i) {
        if (ps_old(i)->d != arity1 || ps_old(i)->type == predicate_comparison)
            continue;
        FOR1D(ps_old, k) {
            if (i==k) continue;
            if (usedCombos(i,k)) continue;
            if (ps_old(k)->d != arity2 || ps_old(k)->type == predicate_comparison)
                continue;
            PredL ps;
            generateSCP_twoBases(ps, arity, ps_old(i), ps_old(k), baseArgs, freeAllQuantified, prefix);
            pos_pos.append(ps(0));
            pos_neg.append(ps(1));
            neg_pos.append(ps(2));
            neg_neg.append(ps(3));
            usedCombos(i,k)=true;usedCombos(k,i)=true;
        }
    }
    ps_invented.append(pos_pos);
    ps_invented.append(pos_neg);
    ps_invented.append(neg_pos);
    ps_invented.append(neg_neg);
}




void Combine_0_0::invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates) {
    MT::String prefix("comb_0_0__");
    uintA baseArgs(2);
    baseArgs(0) = 0;
    baseArgs(1) = 0;
    generateSCP_twoBases(inventedPredicates, 1, 1, 1, predicates, baseArgs, true, prefix);
}


void Combine_0_01::invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates) {
    MT::String prefix("comb_0_01__");
    uintA baseArgs(3);
    baseArgs(0) = 0;
    baseArgs(1) = 0;
    baseArgs(2) = 1;
    generateSCP_twoBases(inventedPredicates, 2, 1, 2, predicates, baseArgs, true, prefix);
}


void Combine_0_10::invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates) {
    MT::String prefix("comb_0_10__");
    uintA baseArgs(3);
    baseArgs(0) = 0;
    baseArgs(1) = 1;
    baseArgs(2) = 0;
    generateSCP_twoBases(inventedPredicates, 2, 1, 2, predicates, baseArgs, true, prefix);
}

void Combine_01_01::invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates) {
  MT::String prefix("comb_01_01__");
    uintA baseArgs(4);
    baseArgs(0) = 0;
    baseArgs(1) = 1;
    baseArgs(2) = 0;
    baseArgs(3) = 1;
    generateSCP_twoBases(inventedPredicates, 2, 2, 2, predicates, baseArgs, true, prefix);
}

void Combine_01_10::invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates) {
  MT::String prefix("comb_01_10__");
    uintA baseArgs(4);
    baseArgs(0) = 0;
    baseArgs(1) = 1;
    baseArgs(2) = 1;
    baseArgs(3) = 0;
    generateSCP_twoBases(inventedPredicates, 2, 2, 2, predicates, baseArgs, true, prefix);
}

void Combine_01_00::invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates) {
    MT::String prefix("comb_01_00__");
    uintA baseArgs(4);
    baseArgs(0) = 0;
    baseArgs(1) = 1;
    baseArgs(2) = 0;
    baseArgs(3) = 0;
    generateSCP_twoBases(inventedPredicates, 2, 2, 2, predicates, baseArgs, true, prefix);
}

void Combine_01_11::invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates) {
    MT::String prefix("comb_01_11__");
    uintA baseArgs(4);
    baseArgs(0) = 0;
    baseArgs(1) = 1;
    baseArgs(2) = 1;
    baseArgs(3) = 1;
    generateSCP_twoBases(inventedPredicates, 2, 2, 2, predicates, baseArgs, true, prefix);
}




void generateColleaguePredicates(PredL& ps_invented, uint arity, uintA baseArgs, const PredL& ps_old, const char* prefix) {
    uint i;
    FOR1D(ps_old, i) {
        if (ps_old(i)->d != baseArgs.N || ps_old(i)->type==predicate_comparison)
            continue;
        // forall +
        TL::ConjunctionPredicate* p_all_pos = new TL::ConjunctionPredicate;
        p_all_pos->d = arity;
        p_all_pos->basePreds.resize(1);
        p_all_pos->basePreds(0) = ps_old(i);
        p_all_pos->basePreds_mapVars2conjunction = baseArgs;
        p_all_pos->basePreds_positive.resize(1);
        p_all_pos->basePreds_positive(0) = true;
        p_all_pos->freeVarsAllQuantified = true;
        ps_invented.append(p_all_pos);
        
        // forall -
        TL::ConjunctionPredicate* p_all_neg = new TL::ConjunctionPredicate;
        *p_all_neg = *p_all_pos;
        p_all_neg->basePreds_positive(0) = false;
        ps_invented.append(p_all_neg);
        
        // exists +
        TL::ConjunctionPredicate* p_ex_pos = new TL::ConjunctionPredicate;
        *p_ex_pos = *p_all_pos;
        p_ex_pos->freeVarsAllQuantified = false;
        ps_invented.append(p_ex_pos);
        
        // exists -
        TL::ConjunctionPredicate* p_ex_neg = new TL::ConjunctionPredicate;
        *p_ex_neg = *p_all_pos;
        p_ex_neg->basePreds_positive(0) = true;
        p_ex_neg->freeVarsAllQuantified = false;
        ps_invented.append(p_ex_neg);
    }
    
    // setting names
    FOR1D(ps_invented, i) {
        TL::ConjunctionPredicate* p = dynamic_cast<TL::ConjunctionPredicate*>(ps_invented(i));
        MT::String name;
        name << prefix << "_";
        if (p->freeVarsAllQuantified) name<<"all_";
        else name<<"ex_";
        if (!p->basePreds_positive(0)) name<<"N";
        name<<p->basePreds(0)->name;
        p->name = name;
    }
}


void CheckColleagueUnary::invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates) {
    const char* prefix = "quant_";
    uintA baseArgs;
    baseArgs.append(0);
    generateColleaguePredicates(inventedPredicates, 0, baseArgs, predicates, prefix);
}

void CheckColleagueBinary_back::invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates) {
    const char* prefix = "quant_1";
    uintA baseArgs;
    baseArgs.append(1);
    baseArgs.append(0);
    generateColleaguePredicates(inventedPredicates, 1, baseArgs, predicates, prefix);
}

void CheckColleagueBinary_front::invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates) {
    const char* prefix = "quant_0";
    uintA baseArgs;
    baseArgs.append(0);
    baseArgs.append(1);
    generateColleaguePredicates(inventedPredicates, 1, baseArgs, predicates, prefix);
}



void BuildTransitiveClosure::invent(const PredL& predicates, const FuncL& functions, PredL& inventedPredicates) {
    uint i;
    const char* prefix = "trans";
    FOR1D(predicates, i) {
        if (predicates(i)->d != 2 || predicates(i)->type==predicate_comparison)
            continue;
        TL::TransClosurePredicate* p = new TL::TransClosurePredicate;
        p->d = 2;
        p->basePred = predicates(i);
        MT::String name;
        name << prefix << "_" << p->basePred->name;
        p->name = name;
        inventedPredicates.append(p);
    }
}



void BuildCountFunction::invent(const PredL& predicates, const FuncL& functions, FuncL& inventedFunctions) {
    inventedFunctions.clear();
    uint i, k, l;
    const char* prefix = "f_count";
    FOR1D(predicates, i) {
        if (predicates(i)->d != 2 || predicates(i)->type==predicate_comparison)
            continue;
        uintA elements;
        for (k=0;k<predicates(i)->d;k++) elements.append(k);
        MT::Array< uintA > countedIDs_sets;
        TL::allSubsets(countedIDs_sets, elements, true);
        // for all subsets
        FOR1D(countedIDs_sets, k) {
            TL::CountFunction* f = new TL::CountFunction;
            f->d = countedIDs_sets(k).N;
            f->countedPred = predicates(i);
            f->countedPred_mapVars2derived.resize(f->countedPred->d);
            uint argID = 0;
            uint addID = f->d;
//             PRINT(countedIDs_sets(k))
            for (l=0; l<predicates(i)->d; l++) {
                if (countedIDs_sets(k).findValue(l)<0)
                    f->countedPred_mapVars2derived(l) = argID++;
                else
                    f->countedPred_mapVars2derived(l) = addID++;
            }
//             PRINT(f->countedPred_mapVars2derived)
            MT::String name;
            name << prefix << "_" << f->countedPred->name;
            for (l=0; l<predicates(i)->d; l++) {
                if (countedIDs_sets(k).findValue(l)>=0)
                    name << "_" << l;
            }
            f->name = name;
            inventedFunctions.append(f);
        }
    }
}





#endif


