#include "BatchMaze.h"

#include "util.h"
#include "Maze.h"
#include "KMarkovCRF.h"
#include "LookAheadSearch.h"
#include "UTree.h"
#include "LinearQ.h"
#include "SmoothingKernelSigmoid.h"

#include <omp.h>

#include "qcustomplot.h"
#include <QApplication>
#include <QDateTime>

#include <algorithm>
#include <float.h> // for DBL_MAX
#include <set>
#include <tuple>
#include <iomanip> // for std::setw

#include <QString>
#include "QtUtil.h" // for << operator

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#include "debug.h"

#define LOG_COMMENT(x) DEBUG_OUT(2,x); log_file << "# " << x << std::endl;
#define LOG(x) DEBUG_OUT(2,x); log_file << x << std::endl;

#define USE_OMP

using std::set;
using std::tuple;
using std::make_tuple;
using std::get;
using std::vector;

using util::clamp;
using util::min;
using util::max;

const vector<QString> BatchMaze::mode_vector = {
    "RANDOM",
    "OPTIMAL",
    "SPARSE",
    "UTREE_PROB",
    "UTREE_VALUE",
    "LINEAR_Q",
    "SEARCH_TREE",
    "TRANSITIONS"
};

const vector<QString> BatchMaze::sample_method_vector = {
    "ACTIVE",
    "RANDOM",
    "UNIFORM",
    "EXP"
};

const vector<BatchMaze::switch_t> BatchMaze::switch_vector = {
    switch_t("-mode",         "char",   "",      "major mode BatchMaze runs in"),
    switch_t("-sample",       "char",   "",      "the way samples are selected"),
    switch_t("-nEp",          "int",    "100",   "number of episodes to run"),
    switch_t("-minTran",      "int",    "10",    "minimum number of transitions (mode 'TRANSITIONS' only)"),
    switch_t("-maxTran",      "int",    "1000",  "maximum number of transitions"),
    switch_t("-e",            "double", "0.0",   "epsilon (probability for unintended random actions)"),
    switch_t("-d",            "double", "0.5",   "discount"),
    switch_t("-minTrain",     "int",    "100",   "minimum length of traning data"),
    switch_t("-maxTrain",     "int",    "10000", "maximum length of traning data"),
    switch_t("-minTree",      "int",    "1000",  "minimum size of search tree (mode 'SEARCH_TREE' only)"),
    switch_t("-maxTree",      "int",    "10000", "maximum size of search tree"),
    switch_t("-f",            "int",    "2",     "number of feature conjunctions"),
    switch_t("-l1",           "double", "0.001", "L1-regularization factor"),
    switch_t("-pruneTree",    "bool",   "true",  "whether to prune the search tree"),
    switch_t("-kWidth",       "double", "0",     "kernel width for 'ACTIVE' sampling (0 for auto)"),
    switch_t("-incr",         "int",    "0",     "increment for 'UNIFORM' and 'EXP' sampling (0 for auto, negative for resampling)"),
    switch_t("-exp",          "double", "0",     "factor for 'EXP' sampling"),
    switch_t("-printSamples", "bool",   "false", "don't run, only print sampling locations"),
    switch_t("-optTran",      "double", "0",     "probability of optimal training transitions (vs. random)")
};

BatchMaze::BatchMaze() {
    // date and time
    QDateTime dateTime = QDateTime::currentDateTime();
    date_time_string = dateTime.toString("yyyy-MM-dd_hh:mm:ss");
}

BatchMaze::~BatchMaze() {}

int BatchMaze::run(int argn, char ** argarr) {
    // start a qApplication if an X server is available
#ifdef X_SERVER
    QApplication a(argn,argarr);
#endif

    // parse command line switches
    bool ok = parse_switches(argn, argarr);
    if(!ok) {
        DEBUG_ERROR("invalid command line arguments");
	print_help();
        return 1;
    }

    // check for valid mode
    QString mode_str = switch_char("-mode");
    mode = "";
    for(auto m : mode_vector) {
        if(mode_str==m) {
            mode=m;
        }
    }

    // check for valid sample method
    QString sample_str = switch_char("-sample");
    sample_method = "";
    for(auto s : sample_method_vector) {
        if(sample_str==s) {
            sample_method=s;
        }
    }

    // print help and exit in case of errors
    if(mode=="") {
        DEBUG_ERROR("No valid mode given ('" << switch_char("-mode") << "')" );
    }
    if(sample_method=="") {
        DEBUG_ERROR("No valid sample method given ('" << switch_char("-sample") << "')" );
    }
    if(mode=="" || sample_method=="") {
        print_help();
        return 1;
    }

    // initialize log file
    initialize_log_file();

    // start data acquisition
    return run_active();
}

bool BatchMaze::parse_switches(int argn, char ** argarr) {
    bool errors = false;

    // assign default values
    for(switch_t sw : switch_vector) {
        QString switch_value = sw.default_value;
        bool ok;
        if(sw.type=="int") {
            int_switches[sw.switch_string] = switch_value.toInt(&ok);
        } else if(sw.type=="double") {
            double_switches[sw.switch_string] = switch_value.toDouble(&ok);
        } else if(sw.type=="bool") {
            if(switch_value=="true" || switch_value=="t") {
                bool_switches[sw.switch_string] = true;
                ok = true;
            } else if(switch_value=="false" || switch_value=="f") {
                bool_switches[sw.switch_string] = false;
                ok = true;
            } else {
                ok = false;
            }
        } else if(sw.type=="char") {
            char_switches[sw.switch_string] = switch_value;
            ok = true;
        } else {
            DEBUG_DEAD_LINE;
        }
        if(!ok) {
            DEBUG_ERROR("Switch '" << sw.switch_string << "' has invalid default value '" << sw.default_value << "'");
            errors = true;
        }
    }

    // parse command line arguments
    for(int arg_idx=1; arg_idx<argn; ++arg_idx) {

        bool match_found = false;
        for(switch_t sw : switch_vector) {

            // look for match
            QString switch_value = sw.default_value;
            if(sw.switch_string!=argarr[arg_idx]) {
                continue;
            } else {
                ++arg_idx;
                switch_value = argarr[arg_idx];
                match_found = true;
            }

            // assign value to switch
	    bool ok;
            if(sw.type=="int") {
	      int_switches[sw.switch_string] = switch_value.toInt(&ok);
            } else if(sw.type=="double") {
	      double_switches[sw.switch_string] = switch_value.toDouble(&ok);
            } else if(sw.type=="bool") {
	      if(switch_value=="true" || switch_value=="t") {
		bool_switches[sw.switch_string] = true;
		ok = true;
	      } else if(switch_value=="false" || switch_value=="f") {
		bool_switches[sw.switch_string] = false;
		ok = true;
	      } else {
		ok = false;
	      }
            } else if(sw.type=="char") {
	      char_switches[sw.switch_string] = switch_value;
	      ok = true;
            } else {
	      DEBUG_DEAD_LINE;
            }

	    // check for success
	    if(!ok) {
	      DEBUG_ERROR("could not parse value for switch '" <<
			  sw.switch_string << "' ('" <<
			  switch_value << "')"
			  );
	      errors = true;
	    }
        }

        if(!match_found) {
            DEBUG_ERROR("Command line parameter '" << argarr[arg_idx] << "' not recognized");
            errors = true;
        }
    }

    return !errors;
}

int BatchMaze::switch_int(QString s) const {
    auto elem = int_switches.find(s);
    if(elem==int_switches.end()) {
        DEBUG_ERROR("Could not find <int> switch '" << s << "'");
    }
    return elem->second;
}

double BatchMaze::switch_double(QString s) const {
    auto elem = double_switches.find(s);
    if(elem==double_switches.end()) {
        DEBUG_ERROR("Could not find <double> switch '" << s << "'");
    }
    return elem->second;
}

bool BatchMaze::switch_bool(QString s) const {
    auto elem = bool_switches.find(s);
    if(elem==bool_switches.end()) {
        DEBUG_ERROR("Could not find <bool> switch '" << s << "'");
    }
    return elem->second;
}

QString BatchMaze::switch_char(QString s) const {
    auto elem = char_switches.find(s);
    if(elem==char_switches.end()) {
        DEBUG_ERROR("Could not find <char> switch '" << s << "'");
    }
    return elem->second;
}

BatchMaze::switch_t::switch_t(QString sstr,
                              QString t,
                              QString dv,
                              QString hdesc):
    switch_string(sstr),
    type(t),
    default_value(dv),
    help_description(hdesc) {}

int BatchMaze::run_active() {

    //---------------------//
    // setup sampling data //
    //---------------------//
    // "ACTIVE"
    SmoothingKernelSigmoid * sks = nullptr;
    set<tuple<int,int,double> > virtual_data; // virtual data points for parallelization
    double k_width = switch_double("-kWidth"), k_exp = 2, min_sample = 0, max_sample = 1;
    // "UNIFORM" and "EXP"
    vector<int> sample_vector;
    if(sample_method=="ACTIVE" || sample_method=="RANDOM") {
        // smoothing kernel containing the actual data points
        if(mode=="OPTIMAL" || mode=="RANDOM") {
            // nothing to do
        } else if(mode=="SEARCH_TREE") {
            if(k_width==0) {
                k_width = (switch_int("-maxTree")-switch_int("-minTree"))/10;
            }
            min_sample = switch_int("-minTree");
            max_sample = switch_int("-maxTree");
        } else if(mode=="TRANSITIONS") {
            if(k_width==0) {
                k_width = (switch_int("-maxTran")-switch_int("-minTran"))/10;
            }
            min_sample = switch_int("-minTran");
            max_sample = switch_int("-maxTran");
        } else {
            if(k_width==0) {
                k_width = (switch_int("-maxTrain")-switch_int("-minTrain"))/10;
            }
            min_sample = switch_int("-minTrain");
            max_sample = switch_int("-maxTrain");
        }
        sks = new SmoothingKernelSigmoid(k_width, k_exp, min_sample, max_sample);
    } else if(sample_method=="UNIFORM") {
        generate_uniform_samples(sample_vector);
        if(switch_bool("-printSamples")) {
            for(int s : sample_vector) {
                LOG(s);
            }
            return 0;
        }
    } else if(sample_method=="EXP") {
        generate_exp_samples(sample_vector);
        if(switch_bool("-printSamples")) {
            for(int s : sample_vector) {
                LOG(s);
            }
            return 0;
        }
    } else {
        DEBUG_DEAD_LINE;
    }

    //--------------------------//
    // run episodes in parallel //
    //--------------------------//
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic,1) collapse(1)
#endif
    for(int episode_counter=1; episode_counter<=switch_int("-nEp"); ++episode_counter) {

        // use pointers to serialize initialization
        Maze                   * maze                 = nullptr;
        instance_t             * current_instance     = nullptr;
        LookAheadSearch        * look_ahead_search    = nullptr;
        KMarkovCRF             * crf                  = nullptr;
        UTree                  * utree                = nullptr;
        LinearQ                * linQ                 = nullptr;
        SmoothingKernelSigmoid * virtual_sks          = nullptr;

        // virtual (preliminary) point
        int virtual_x;
        double virtual_reward;

        // the quantities that may be varied
        int training_length = 0, search_tree_size = switch_int("-maxTree"), transition_length = switch_int("-maxTran");

#ifdef USE_OMP
#pragma omp critical
#endif
        {
            // initialize maze
            state_t start_state = state_t::random_state();
            maze = new Maze(switch_double("-e"));
            maze->set_current_state(start_state);
            current_instance = instance_t::create(action_t::STAY,start_state,reward_t(0));

            // initialize look ahead search
            look_ahead_search = new LookAheadSearch(switch_double("-d"));

            // initialize learners
            crf = new KMarkovCRF();
            utree = new UTree(switch_double("-d"));
            linQ = new LinearQ(switch_double("-d"));

            // compute varying quantity
            if(sample_method=="ACTIVE") {
                // initialize virtual sks, get sample location, update virtual data
                if(mode!="OPTIMAL" && mode!="RANDOM") {
                    // insert old points into virtual sks
                    virtual_sks = new SmoothingKernelSigmoid(*sks);
                    for(auto data_point : virtual_data) {
                        virtual_sks->add_new_point(get<1>(data_point),get<2>(data_point));
                    }
                    // get sample location
                    virtual_x = round(virtual_sks->get_max_uncertain(max_sample-min_sample));
                    /*virtual_x += rand()%5-2; // +/- two steps of noise*/
                    /*virtual_x = clamp<int>(min_sks,max_sks,virtual_x);*/
                    // get virtual reward and add point to virtual data
                    virtual_sks->mean_dev_weights(virtual_x,virtual_reward);
                    virtual_data.insert(make_tuple(episode_counter,virtual_x,virtual_reward));
                }

                // set varying quantity
                if(mode=="OPTIMAL" || mode=="RANDOM") {
                    // nothing to do
                } else if(mode=="SEARCH_TREE") {
                    search_tree_size = virtual_x;
                } else if(mode=="TRANSITIONS") {
                    transition_length = virtual_x;
                } else {
                    training_length = virtual_x;
                }
            } else if(sample_method=="UNIFORM" || sample_method=="EXP") {
                // set varying quantity
                if(mode=="OPTIMAL" || mode=="RANDOM") {
                    // nothing to do
                } else if(mode=="SEARCH_TREE") {
                    search_tree_size = sample_vector[episode_counter-1];
                } else if(mode=="TRANSITIONS") {
                    transition_length = sample_vector[episode_counter-1];
                } else {
                    training_length = sample_vector[episode_counter-1];
                }
            } else if(sample_method=="RANDOM") {
                // set varying quantity
                int sample = (int)round(drand48()*(max_sample-min_sample)+min_sample);
                if(mode=="OPTIMAL" || mode=="RANDOM") {
                    // nothing to do
                } else if(mode=="SEARCH_TREE") {
                    search_tree_size = sample;
                } else if(mode=="TRANSITIONS") {
                    transition_length = sample;
                } else {
                    training_length = sample;
                }
            } else {
                DEBUG_DEAD_LINE;
            }

        }
// end omp critical

	if(mode=="OPTIMAL" || mode=="RANDOM" || mode=="SEARCH_TREE" || mode=="TRANSITIONS") {
	  // nothing more to do
	} else {
	  // generate training data
	  for(int train_step=0; train_step<training_length; ++train_step) {
	    action_t action;
	    state_t state;
	    reward_t reward;
	    if(drand48()<switch_double("-optTran")) {
	      look_ahead_search->clear_tree();
	      look_ahead_search->build_tree<Maze>(current_instance, *maze, switch_int("-maxTree"));
	      action = look_ahead_search->get_optimal_action();
	    } else {
	      action = action_t::random_action();
	    }
	    maze->perform_transition(action,state,reward);
	    current_instance = current_instance->append_instance(action,state,reward);
	    if(mode=="SPARSE") {
	      crf->add_action_state_reward_tripel(action,state,reward,false);
	    } else if(mode=="UTREE_VALUE" || mode=="UTREE_PROB") {
	      utree->add_action_state_reward_tripel(action,state,reward,false);
	    } else if(mode=="LINEAR_Q") {
	      linQ->add_action_state_reward_tripel(action,state,reward,false);
	    } else {
	      DEBUG_DEAD_LINE;
	    }
	    DEBUG_OUT(2,"Learning: (" << action << "," << state << "," << reward << ")");
	  }
	}


        //-------------------//
        // train the learner //
        //-------------------//
        if(mode=="OPTIMAL" || mode=="RANDOM" || mode=="SEARCH_TREE" || mode=="TRANSITIONS") {
            // nothing to train
        } else if(mode=="SPARSE") {
            for(int complx=1; complx<=switch_int("-f"); ++complx) {
                crf->score_features_by_gradient(1);
                crf->sort_scored_features(false);
                crf->add_candidate_features_to_active(0);
		if(complx==1) {
		  // no l1 in first run
		  crf->optimize_model(0,0,nullptr);
		} else {
		  // // sparsify features to speed up optimization
		  // crf->optimize_model(switch_double("-l1")/2,10);
		  // crf->erase_zero_features();
		  // crf->optimize_model(switch_double("-l1")/2,20);
		  // crf->erase_zero_features();
		  // crf->optimize_model(switch_double("-l1"),100);
		  crf->optimize_model(switch_double("-l1"),500);
		}
		crf->erase_zero_features();
            }
            // finalize
            crf->optimize_model(0,0,nullptr);
        } else if(mode=="UTREE_PROB") {
            utree->set_expansion_type(UTree::STATE_REWARD_EXPANSION);
            double score_threshold = 1e-5;
            while(score_threshold <= utree->expand_leaf_node(score_threshold)) {}
        } else if(mode=="UTREE_VALUE") {
            utree->set_expansion_type(UTree::UTILITY_EXPANSION);
            double score_threshold = 1e-5;
            while(score_threshold <= utree->expand_leaf_node(score_threshold)) {}
        } else if(mode=="LINEAR_Q") {
            for(int complx=1; complx<=switch_int("-f"); ++complx) {
                linQ->add_candidates(1);
                linQ->erase_zero_features();
		if(complx==1) {
		  // no l1 in first run
		  linQ->optimize_ridge(1e-10);
		} else {
		  // // sparsify features to speed up optimization
		  // linQ->optimize_l1(switch_double("-l1")/2,10);
		  // linQ->erase_zero_weighted_features();
		  // linQ->optimize_l1(switch_double("-l1")/2,20);
		  // linQ->erase_zero_weighted_features();
		  // linQ->optimize_l1(switch_double("-l1"),100);
		  linQ->optimize_l1(switch_double("-l1"),500);
		}
		linQ->erase_zero_weighted_features();
            }
            // finalize
            linQ->optimize_ridge(1e-10);
        } else {
            DEBUG_DEAD_LINE;
        }

        //---------------------//
        // perform transitions //
        //---------------------//
        double reward_sum = 0;
        int transition_counter;
        for(transition_counter=1; transition_counter<=transition_length; ++transition_counter) {

            // transition variables
            action_t action;
            state_t state;
            reward_t reward;

            // choose the action
            if(mode=="OPTIMAL" || mode=="TRANSITIONS") {
                if(look_ahead_search->get_number_of_nodes()==0 || !switch_bool("-pruneTree")) {
                    look_ahead_search->clear_tree();
                    look_ahead_search->build_tree<Maze>(current_instance, *maze, switch_int("-maxTree"));
                } else {
                    look_ahead_search->fully_expand_tree<Maze>(*maze, switch_int("-maxTree"));
                }
                action = look_ahead_search->get_optimal_action();
            } else if(mode=="RANDOM") {
                action = action_t::random_action();
            } else if(mode=="SPARSE") {
                if(look_ahead_search->get_number_of_nodes()==0 || !switch_bool("-pruneTree")) {
                    look_ahead_search->clear_tree();
                    look_ahead_search->build_tree<KMarkovCRF>(current_instance, *crf, switch_int("-maxTree"));
                } else {
                    look_ahead_search->fully_expand_tree<KMarkovCRF>(*crf, switch_int("-maxTree"));
                }
                action = look_ahead_search->get_optimal_action();
            } else if(mode=="UTREE_PROB") {
                look_ahead_search->clear_tree();
                if(look_ahead_search->get_number_of_nodes()==0 || !switch_bool("-pruneTree")) {
                    look_ahead_search->build_tree<UTree>(current_instance, *utree, switch_int("-maxTree"));
                } else {
                    look_ahead_search->fully_expand_tree<UTree>(*utree, switch_int("-maxTree"));
                }
                action = look_ahead_search->get_optimal_action();
            } else if(mode=="UTREE_VALUE") {
                action = utree->get_max_value_action(current_instance);
            } else if(mode=="LINEAR_Q") {
                action = linQ->get_max_value_action(current_instance);
            } else if(mode=="SEARCH_TREE") {
                if(look_ahead_search->get_number_of_nodes()==0 || !switch_bool("-pruneTree")) {
                    look_ahead_search->build_tree<Maze>(current_instance, *maze, search_tree_size);
                } else {
                    look_ahead_search->fully_expand_tree<Maze>(*maze, search_tree_size);
                }
                action = look_ahead_search->get_optimal_action();
            } else {
                DEBUG_DEAD_LINE;
            }

            // perform transition
            maze->perform_transition(action,state,reward);
            current_instance = current_instance->append_instance(action,state,reward);

            // prune search tree
            if(switch_bool("-pruneTree")) {
                if(mode=="OPTIMAL" || mode=="SEARCH_TREE" || mode=="TRANSITIONS") {
                    look_ahead_search->prune_tree(action,current_instance,*maze);
                } else if(mode=="SPARSE") {
                    look_ahead_search->prune_tree(action,current_instance,*crf);
                } else if(mode=="UTREE_PROB") {
                    look_ahead_search->prune_tree(action,current_instance,*utree);
                } else if(mode=="RANDOM" || mode=="UTREE_VALUE" || mode=="LINEAR_Q") {
                    // no search tree
                } else {
                    DEBUG_DEAD_LINE;
                }
            }

            // increment reward
            reward_sum += reward;

            DEBUG_OUT(1, "Episode	" << episode_counter <<
                      ",	training length " << training_length <<
                      ",	tree size " << search_tree_size <<
                      ",	transition " << transition_counter <<
                      ",	current mean reward = " << reward_sum/transition_counter <<
                      ",	(a,s,r) = (" << action << "," << state << "," << reward << ")"
                );
        }
#ifdef USE_OMP
#pragma omp critical
#endif
        {

            // write data to log file
            LOG(episode_counter << " 	" <<
                training_length << "	" <<
                transition_length << "	" <<
                search_tree_size << "	" <<
                (mode=="SPARSE" ? crf->get_number_of_features() : 0) << "	" <<
                ( (mode=="UTREE_VALUE" || mode=="UTREE_PROB") ? utree->get_tree_size() : 0) << "	" <<
                reward_sum/transition_length
                );

            // for "ACTIVE" sampling: update smoothing kernel and virtual data,
            // print data if X server available
            if(sample_method=="ACTIVE") {
                // update smoothing kernel and virtual data
                if(mode!="OPTIMAL" && mode!="RANDOM") {
                    if(mode=="SEARCH_TREE") {
                        bool success = virtual_data.erase(make_tuple(episode_counter,search_tree_size,virtual_reward));
                        if(!success) {
                            DEBUG_ERROR("Could not remove virtual data point (" << episode_counter << "," <<
                                      search_tree_size << "," <<
                                      virtual_reward << ")"
                                );
                        }
                        sks->add_new_point(search_tree_size,reward_sum/transition_length);
                    } else if(mode=="TRANSITIONS") {
                        bool success = virtual_data.erase(make_tuple(episode_counter,transition_length,virtual_reward));
                        if(!success) {
                            DEBUG_ERROR("Could not remove virtual data point (" << episode_counter << "," <<
                                      transition_length << "," <<
                                      virtual_reward << ")"
                                );
                        }
                        sks->add_new_point(transition_length,reward_sum/transition_length);
                    } else {
                        bool success = virtual_data.erase(make_tuple(episode_counter,training_length,virtual_reward));
                        if(!success) {
                            DEBUG_ERROR("Could not remove virtual data point (" << episode_counter << "," <<
                                      training_length << "," <<
                                      virtual_reward << ")"
                                );
                        }
                        sks->add_new_point(training_length,reward_sum/transition_length);
                    }


                    // print graph to file (only for more than 3 data points)
#ifdef X_SERVER
                    if(episode_counter>3*omp_get_num_threads()) {
                        QCustomPlot * plotter = new QCustomPlot();
                        sks->print_to_QCP(plotter);
                        QString plot_file_name = date_time_string;
                        plot_file_name.append("_");
                        plot_file_name.append(mode);
                        plot_file_name.append("_");
                        plot_file_name.append(sample_method);
                        plot_file_name.append("_print_file_");
                        plot_file_name.append(QString("%1.png").arg(QString::number(episode_counter),(int)floor(log10(switch_int("-nEp"))+1),QChar('0')));
                        DEBUG_OUT(1,"Plotting to file " << plot_file_name );
                        plotter->savePng(plot_file_name,1000,700,1,-1);
                        // plot_file_name.append(QString("%1.pdf").arg(QString::number(episode_counter),(int)floor(log10(switch_int("-nEp"))+1),QChar('0')));
                        // plotter->savePdf(plot_file_name,true,1000,700);
                        delete plotter;
                    }
#endif
                } else {
                    // nothing to do for other sampling methods
                }
            }

            // delete pointers
            delete maze;
            delete current_instance;
            delete look_ahead_search;
            delete crf;
            delete utree;
            delete linQ;
            delete virtual_sks;
        }
// end omp critical
    }
    delete sks;

    return 0;
}

void BatchMaze::print_help() {
    DEBUG_OUT(0,"----------------------------BatchMaze----------------------------");
    DEBUG_OUT(0,"switch            type      default_value description"                       );
    DEBUG_OUT(0,"-----------------------------------------------------------------");
    for(auto sw : switch_vector) {
        DEBUG_OUT(0, std::setw(18) << std::left << sw.switch_string <<
                  std::setw(10) << std::left << sw.type <<
                  std::setw(14) << std::left << sw.default_value <<
                  sw.help_description
            );
    }
    DEBUG_OUT(0,"-----------------------------------------------------------------");
    DEBUG_OUT(0,"Valid modes are");
    for(QString m : mode_vector) {
        DEBUG_OUT(0,"    " << m);
    }
    DEBUG_OUT(0,"-----------------------------------------------------------------");
    DEBUG_OUT(0,"Valid sample methods are");
    for(QString s : sample_method_vector) {
        DEBUG_OUT(0,"    " << s);
    }
    DEBUG_OUT(0,"-----------------------------------------------------------------");
}

void BatchMaze::initialize_log_file() {
    QString log_file_name = date_time_string;
    log_file_name.append("_");
    log_file_name.append(mode);
    log_file_name.append("_");
    log_file_name.append(sample_method);
    log_file_name.append("_log_file.txt");
    log_file.open((const char*)log_file_name.toLatin1());

    std::string tmp_reward_str = Maze::get_rewards(), reward_str;
    std::string tmp_wall_str = Maze::get_walls(), wall_str;
    std::string tmp_door_str = Maze::get_doors(), door_str;
    for( auto c : tmp_reward_str ) {
        if(c!='\n') {
            reward_str += c;
        } else {
            reward_str += "\n# ";
        }
    }
    for( auto c : tmp_wall_str ) {
        if(c!='\n') {
            wall_str += c;
        } else {
            wall_str += "\n# ";
        }
    }
    for( auto c : tmp_door_str ) {
        if(c!='\n') {
            door_str += c;
        } else {
            door_str += "\n# ";
        }
    }

    LOG_COMMENT("Switches:");
    for(switch_t sw : switch_vector) {
        QString ss = sw.switch_string;
        QString st = sw.type;
        if(st=="int") {
            LOG_COMMENT(std::setw(18) << std::left << ss << " = " << switch_int(ss) );
        } else if(st=="double") {
            LOG_COMMENT(std::setw(18) << std::left << ss << " = " << switch_double(ss) );
        } else if(st=="bool") {
            LOG_COMMENT(std::setw(18) << std::left << ss << " = " << switch_bool(ss) );
        } else if(st=="char") {
            LOG_COMMENT(std::setw(18) << std::left << ss << " = " << switch_char(ss) );
        } else {
            DEBUG_DEAD_LINE;
        }
    }
    // LOG_COMMENT("mode                   = " << mode );
    // LOG_COMMENT("sample method          = " << sample_method );
    // LOG_COMMENT("epsilon                = " << switch_double("-e") );
    // LOG_COMMENT("discount               = " << switch_double("-d") );
    // LOG_COMMENT("episodes               = " << switch_int("-nEp") );
    // LOG_COMMENT("min transitions        = " << switch_int("-minTran") );
    // LOG_COMMENT("(max) transitions      = " << switch_int("-maxTran") );
    // LOG_COMMENT("min training length    = " << switch_int("-minTrain") );
    // LOG_COMMENT("max training length    = " << switch_int("-maxTrain") );
    // LOG_COMMENT("L1 coefficient         = " << switch_double("-l1") );
    // LOG_COMMENT("min tree size          = " << switch_int("-minTree") );
    // LOG_COMMENT("(max) tree size        = " << switch_int("-maxTree") );
    // LOG_COMMENT("feature complexity     = " << switch_int("-f") );
    // LOG_COMMENT("prune search tree      = " << (switch_bool("-pruneTree")?"Yes":"No") );
    // LOG_COMMENT("print samples          = " << (switch_bool("-printSamples")?"Yes":"No") );
    LOG_COMMENT("");
    LOG_COMMENT("Maze size: " << Config::maze_x_size << "x" << Config::maze_y_size);
    LOG_COMMENT("");
    LOG_COMMENT(reward_str);
    LOG_COMMENT(wall_str);
    LOG_COMMENT(door_str);
    LOG_COMMENT("");
    LOG_COMMENT("Episode	training_length	transition_length	search_tree_size	feature_n	utree_size	episode_mean_reward");
    LOG_COMMENT("");
}

void BatchMaze::generate_uniform_samples(vector<int>& sample_vector) const {
    // get minimum and maximum
    int min = 0, max = 0;
    if(mode=="OPTIMAL" || mode=="RANDOM") {
        // nothing to do
    } else if(mode=="SEARCH_TREE") {
        min = switch_int("-minTree");
        max = switch_int("-maxTree");
    } else if(mode=="TRANSITIONS") {
        min = switch_int("-minTran");
        max = switch_int("-maxTran");
    } else {
        min = switch_int("-minTrain");
        max = switch_int("-maxTrain");
    }
    // get number of episodes
    int episodes = switch_int("-nEp");
    sample_vector.resize(episodes);
    // get increment
    int incr = switch_int("-incr");
    if(incr==0) {
        incr = (int)ceil((double)(max-min)/(episodes-1));
    } else if(incr<0) {
        incr = (int)ceil((double)(max-min)/((-episodes/incr)-1));
    }
    for(int i=0; i<episodes; ++i) {
        int incr_factor = i%(1+(int)ceil((double)(max-min)/incr));
        sample_vector[i] = util::min<int>(max,min+incr_factor*incr);
    }
}

void BatchMaze::generate_exp_samples(vector<int>& sample_vector) const {
    // get minimum and maximum
    int min = 0, max = 0;
    if(mode=="OPTIMAL" || mode=="RANDOM") {
        // nothing to do
    } else if(mode=="SEARCH_TREE") {
        min = switch_int("-minTree");
        max = switch_int("-maxTree");
    } else if(mode=="TRANSITIONS") {
        min = switch_int("-minTran");
        max = switch_int("-maxTran");
    } else {
        min = switch_int("-minTrain");
        max = switch_int("-maxTrain");
    }

    // get number of episodes
    int episodes = switch_int("-nEp");
    sample_vector.resize(episodes);

    // get factor
    double exp_factor = switch_double("-exp");

    // get increment
    int incr = switch_int("-incr");

    // get maximum idx
    int max_i = 0;
    while((int)round(min + max_i*incr + pow(exp_factor,max_i) - 1) < max) {
        ++max_i;
    }

    // generate samples
    for(int i=0; i<episodes; ++i) {
        int idx = i%(max_i+1);
        sample_vector[i] = util::min<int>((int)round(min + idx*incr + pow(exp_factor,idx) - 1),max);
    }
}
