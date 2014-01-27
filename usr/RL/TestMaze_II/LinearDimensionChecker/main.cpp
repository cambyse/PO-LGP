#include <vector>
#include <deque>
#include <map>
#include <tuple>
#include <algorithm> // std::prev_permutation, std::next_permutation, std::sort
#include <iomanip> // std::setprecision

#include <omp.h>

//#define ARMA_NO_DEBUG
#include <armadillo>

#include "../util.h"

#define DEBUG_LEVEL 0
#define DEBUG_STRING ""
#include "../debug.h"

#define USE_OMP

using std::vector;
using std::deque;
using std::map;
using std::tuple;
using std::make_tuple;
using std::next_permutation;
using std::prev_permutation;
using std::sort;
using std::cout;
using std::endl;

using arma::mat;
using arma::vec;
using arma::mat44;
using arma::zeros;

using util::Range;

enum OBSERVATION { O1, O2};
typedef deque<OBSERVATION> sequence_t;
std::ostream& operator<<(std::ostream &out, const OBSERVATION& o) {
    if(o==O1) {
        out << "o1";
    } else {
        out << "o2";
    }
    return out;
}
std::ostream& operator<<(std::ostream &out, const sequence_t& s) {
    if(s.size()==0) {
        out << "phi";
    } else {
        for(auto o : s) {
            out << o;
        }
    }
    return out;
}

class Predictor {
public:
    map<sequence_t, double> o1_weights, o2_weights;
    Predictor() {
        // |h| = 0
        o1_weights[{}]         = 1; o2_weights[{}]         = 1;
        // |h| = 1
        o1_weights[{O1}]       = 1; o2_weights[{O1}]       = 1;
        o1_weights[{O2}]       = 1; o2_weights[{O2}]       = 1;
        // |h| = 2
        o1_weights[{O1,O1}]    = 1; o2_weights[{O1,O1}]    = 1;
        o1_weights[{O1,O2}]    = 1; o2_weights[{O1,O2}]    = 1;
        o1_weights[{O2,O1}]    = 1; o2_weights[{O2,O1}]    = 1;
        o1_weights[{O2,O2}]    = 1; o2_weights[{O2,O2}]    = 1;
        // |h| = 3
        o1_weights[{O1,O1,O1}] = 1; o2_weights[{O1,O1,O1}] = 1;
        o1_weights[{O1,O1,O2}] = 1; o2_weights[{O1,O1,O2}] = 1;
        o1_weights[{O1,O2,O1}] = 1; o2_weights[{O1,O2,O1}] = 1;
        o1_weights[{O1,O2,O2}] = 1; o2_weights[{O1,O2,O2}] = 1;
        o1_weights[{O2,O1,O1}] = 1; o2_weights[{O2,O1,O1}] = 1;
        o1_weights[{O2,O1,O2}] = 1; o2_weights[{O2,O1,O2}] = 1;
        o1_weights[{O2,O2,O1}] = 1; o2_weights[{O2,O2,O1}] = 1;
        o1_weights[{O2,O2,O2}] = 1; o2_weights[{O2,O2,O2}] = 1;
    }
    Predictor(const vector<double> & o1_W, const vector<double> & o2_W) {
        if(o1_W.size()<15||o2_W.size()<15) {
            *this = Predictor();
        } else {
            // |h| = 0
            o1_weights[{}]         = o1_W[ 0]; o2_weights[{}]         = o2_W[ 0];
            // |h| = 1
            o1_weights[{O1}]       = o1_W[ 1]; o2_weights[{O1}]       = o2_W[ 1];
            o1_weights[{O2}]       = o1_W[ 2]; o2_weights[{O2}]       = o2_W[ 2];
            // |h| = 2
            o1_weights[{O1,O1}]    = o1_W[ 3]; o2_weights[{O1,O1}]    = o2_W[ 3];
            o1_weights[{O1,O2}]    = o1_W[ 4]; o2_weights[{O1,O2}]    = o2_W[ 4];
            o1_weights[{O2,O1}]    = o1_W[ 5]; o2_weights[{O2,O1}]    = o2_W[ 5];
            o1_weights[{O2,O2}]    = o1_W[ 6]; o2_weights[{O2,O2}]    = o2_W[ 6];
            // |h| = 3
            o1_weights[{O1,O1,O1}] = o1_W[ 7]; o2_weights[{O1,O1,O1}] = o2_W[ 7];
            o1_weights[{O1,O1,O2}] = o1_W[ 8]; o2_weights[{O1,O1,O2}] = o2_W[ 8];
            o1_weights[{O1,O2,O1}] = o1_W[ 9]; o2_weights[{O1,O2,O1}] = o2_W[ 9];
            o1_weights[{O1,O2,O2}] = o1_W[10]; o2_weights[{O1,O2,O2}] = o2_W[10];
            o1_weights[{O2,O1,O1}] = o1_W[11]; o2_weights[{O2,O1,O1}] = o2_W[11];
            o1_weights[{O2,O1,O2}] = o1_W[12]; o2_weights[{O2,O1,O2}] = o2_W[12];
            o1_weights[{O2,O2,O1}] = o1_W[13]; o2_weights[{O2,O2,O1}] = o2_W[13];
            o1_weights[{O2,O2,O2}] = o1_W[14]; o2_weights[{O2,O2,O2}] = o2_W[14];
        }
    }


    //  p(o0,o1,...|...,o-1) = p(o1,...|...,o-1,o0) p(o0|...,o-1)

    double operator()(const sequence_t test, const sequence_t history) {
        if(test.size()<2) {
            if(test.size()==0) {
                DEBUG_ERROR("Test of zero length not well defined");
                return 0;
            } else {
                double normalization = o1_weights[history]+o2_weights[history];
                switch(test.back()) {
                case O1:
                    return o1_weights[history]/normalization;
                case O2:
                    return o2_weights[history]/normalization;
                default:
                    DEBUG_DEAD_LINE;
                    return 0;
                }
            }
        } else {
            OBSERVATION element = test.front();
            sequence_t short_test(test); short_test.pop_front();
            sequence_t long_history(history); long_history.push_back(element);
            return (*this)(short_test,long_history)*(*this)({element},history);
        }
    }
};

template <typename T>
T vector_sum(const vector<T> & v) {
    T ret{};
    for(T t : v) {
        ret += t;
    }
    return ret;
}

template <typename T>
void print_vector(const vector<T> & v) {
    bool first = true;
    for(auto elem : v) {
        if(first) {
            first = false;
        } else {
            cout << " ";
        }
        cout << elem;
    }
    cout << endl;
}

int main(int, char **) {

    // define all possible sequences of length N
    vector<sequence_t> histories;
    vector<sequence_t> tests;
    {
        int N = 2;
        vector<sequence_t> tmp_sequences;
        for(int length=0; length<=N; ++length) {
            for(int o1_part=length; o1_part>=0; --o1_part) {
                tmp_sequences.push_back(sequence_t());
                int length_idx = 1;
                for(;length_idx<=o1_part;++length_idx) {
                    tmp_sequences.back().push_back(O1);
                }
                for(;length_idx<=length;++length_idx) {
                    tmp_sequences.back().push_back(O2);
                }
            }
        }
        for(auto seq : tmp_sequences) {
            sort(seq.begin(),seq.end());
            do {
                histories.push_back(sequence_t());
                tests.push_back(sequence_t());
                for(auto o : seq) {
                    histories.back().push_back(o);
                    tests.back().push_back(o);
                }
                if(tests.back().size()==0) tests.pop_back();
            } while(next_permutation(seq.begin(),seq.end()));
        }
    }

    // define allowed weights
    int max_rank = -1;
    {
        vector<double> weights = {1,2,4,8};
        vector<vector<int> > count_vectors;
        int max_counts = 30;
        int w1,w2,w3,w4;
        DEBUG_OUT(1,"Count vectors:");
        for(w1=0;;++w1) {
            if(w1>max_counts) break;
            for(w2=0;;++w2) {
                if(w1+w2>max_counts) break;
                for(w3=0;;++w3) {
                    if(w1+w2+w3>max_counts) break;
                    for(w4=0;;++w4) {
                        if(w1+w2+w3+w4>max_counts) break;
                        if(w1+w2+w3+w4<max_counts) continue;
                        count_vectors.push_back({w1,w2,w3,w4});
                        DEBUG_OUT(1,w1 << " " << w2 << " " << w3 << " " << w4);
                    }
                }
            }
        }
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic,1) collapse(1)
#endif
        for(int c_vec_idx=0; c_vec_idx<(int)count_vectors.size(); ++c_vec_idx) {
            vector<sequence_t> local_histories;
            vector<sequence_t> local_tests;
#ifdef USE_OMP
#pragma omp critical
#endif
            {
                local_histories = histories;
                local_tests = tests;
            }
            auto c_vec = count_vectors[c_vec_idx];
            vector<double> weight_vector;
            for(int w_idx=0; w_idx<(int)c_vec.size(); ++w_idx) {
                for(int w_count=0; w_count<c_vec[w_idx]; ++w_count) {
                    weight_vector.push_back(weights[w_idx]);
                }
            }
            sort(weight_vector.begin(),weight_vector.end());
            do {
                //--------------------------------------------------//
                vector<double> o1_weights, o2_weights;
                for(int w_idx=0; w_idx<(int)weight_vector.size(); ++w_idx) {
                    if(w_idx<(int)weight_vector.size()/2) {
                        o1_weights.push_back(weight_vector[w_idx]);
                    } else {
                        o2_weights.push_back(weight_vector[w_idx]);
                    }
                }
                // print_vector(o1_weights);
                // print_vector(o2_weights);
                int h_size = local_histories.size();
                int t_size = local_tests.size();
                mat D = zeros(h_size,t_size);
                Predictor predictor(o1_weights,o2_weights);
                for(int history_idx=0; history_idx<h_size; ++history_idx) {
                    for(int test_idx=0; test_idx<t_size; ++test_idx) {
                        D(history_idx,test_idx) = predictor(local_tests[test_idx],local_histories[history_idx]);
                    }
                }

                // print D
                // cout << "	";
                // for(int test_idx : Range(t_size)) {
                //     if(test_idx>0) cout << "	";
                //     cout << local_tests[test_idx];
                // }
                // cout << endl;
                // for(int history_idx : Range(h_size)) {
                //     cout << local_histories[history_idx];
                //     for(int test_idx : Range(t_size)) {
                //         cout << "	" << std::setprecision(2) << D(history_idx,test_idx);
                //     }
                //     cout << endl;
                // }

                // print rank
                // DEBUG_OUT(0,"Rank of D = " << rank(D) );
                // DEBUG_OUT(0,"--------------------------------------------------");
#ifdef USE_OMP
#pragma omp critical
#endif
                {
                    int this_rank = rank(D);
                    if(this_rank>max_rank) {
                        max_rank = this_rank;
                        print_vector(o1_weights);
                        print_vector(o2_weights);
                        DEBUG_OUT(0,"    --> rank = " << this_rank);
                    }
                }

                //--------------------------------------------------//
            } while(next_permutation(weight_vector.begin(),weight_vector.end()));
        }
    }

    return 0;
}
