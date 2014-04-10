#include "testmaze_ii.h"

#include "Config.h"
#include "util/util.h"
#include "util/QtUtil.h"

#include "SmoothingKernelSigmoid.h"

#include "Visualizer.h"
#include "PredictiveEnvironment.h"
#include "Maze/Maze.h"
#include "CheeseMaze/CheeseMaze.h"
#include "Planning/LookAheadPolicy.h"
#include "Planning/RandomPolicy.h"
#include "Planning/GoalIteration.h"
#include "Representation/DoublyLinkedInstance.h"

#include <float.h>  // for DBL_MAX
#include <vector>
#include <map>
#include <algorithm> // for min, max

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#include "util/debug.h"

using std::vector;
using std::map;
using std::cout;
using std::endl;
using std::get;
using std::min;
using std::max;
using std::dynamic_pointer_cast;
using std::shared_ptr;
using std::make_shared;

using util::arg_int;
using util::arg_double;
using util::arg_string;
using util::INVALID;

TestMaze_II::TestMaze_II(QWidget *parent):
    QWidget(parent),
    planner_type(GOAL_ITERATION),
    environment(nullptr),
    current_instance(INVALID),
    record(false), plot(false), start_new_episode(false), save_png_on_transition(false), color_maze(true),
    png_counter(0),
    action_timer(nullptr),
    console_history(1,"END OF HISTORY"),
    history_position(0),
    history_file("console_history.txt"),
    discount(0.7),
    epsilon(0.0),
    l1_factor(0),
    crf(new KMarkovCRF()),
    utree(new UTree(discount)),
    linQ(new LinearQ(discount)),
    policy(nullptr),
    max_tree_size(10000),
    prune_search_tree(true),
    goal_activated(false)
{

    // initialize UI
    ui.setupUi(this);

    // disable graph view
    ui._wGraphDockWidget->setVisible(false);

    // open console history file
    if(!history_file.open(QIODevice::ReadWrite | QIODevice::Text)) {
        DEBUG_ERROR("Could not open console history file");
    } else {
        QTextStream history_file_stream(&history_file);
        while ( !history_file_stream.atEnd() ) {
            QString line = history_file_stream.readLine();
            console_history.push_back(line);
            history_position = console_history.size();
        }
    }

    // add graph widget
    plotter = new QCustomPlot(ui._wGraphDockWidgetContent);
    plotter->setObjectName(MY_QT_STR("PlotWidget"));
    ui._lGraphWidgetLayout->addWidget(plotter, 0, 0, 1, 1);

    // focus on command line
    ui._wConsoleInput->setFocus();

    // set console welcome message
    ui._wConsoleOutput->setPlainText("    Please enter your commands (type 'help' for an overview)");

    // initialize timers
    action_timer = new QTimer(this);
    connect(action_timer, SIGNAL(timeout()), this, SLOT(choose_action()));

    // initiate delayed render action
    QTimer::singleShot(0, this, SLOT(render_update()));

    // install event filter
    MoveByKeys *moveByKeys = new MoveByKeys(this);
    ui.graphicsView->installEventFilter(moveByKeys);

    // select environment
    change_environment(make_shared<Maze>(epsilon,"Markov"));
    // change_environment(make_shared<CheeseMaze>());

    // set all commands
    typedef Commander::ReturnType ret_t;
    command_center.add_command({"help","h"},[this]()->ret_t{
            to_console("");
            for(QString s : command_center.get_help()) {
                to_console(s,4);
            }
            return{true,""};
        },"Print help");
    //---------------------------------General-----------------------------------
    command_center.add_command({"exit","quit","q"}, [this]()->ret_t{return{true,""};}, "quit application");
    command_center.add_command({"set record"}, [this]()->ret_t{return{true,""};}, "start recording movements");
    command_center.add_command({"unset record"}, [this]()->ret_t{return{true,""};}, "stop recording movements");
    command_center.add_command({"set plot"}, [this]()->ret_t{return{true,""};}, "start writing transitions into data file for plotting");
    command_center.add_command({"unset plot"}, [this]()->ret_t{return{true,""};}, "stop writing transitions into data file for plotting");
    command_center.add_command({"set planner o","optimal"}, [this]()->ret_t{return{true,""};}, "use optimal predictions (give by maze)");
    command_center.add_command({"set planner s","sparse"}, [this]()->ret_t{return{true,""};}, "use sparse predictions (given by CRF)");
    command_center.add_command({"set planner u","utree"}, [this]()->ret_t{return{true,""};}, "use UTree as predictive model");
    command_center.add_command({"set planner uv","utree-value"}, [this]()->ret_t{return{true,""};}, "use UTree as value function");
    command_center.add_command({"set planner lq","linear-q"}, [this]()->ret_t{return{true,""};}, "use linear Q-function approximation");
    command_center.add_command({"set planner g","goal"}, [this]()->ret_t{return{true,""};}, "use goal-guided value iteration");
    command_center.add_command({"set planner r","random"}, [this]()->ret_t{return{true,""};}, "use random policy");
    command_center.add_command({"set goal","s g"}, [this]()->ret_t{return{true,""};}, "activate goal state at current location");
    command_center.add_command({"set goal random","s g r"}, [this]()->ret_t{return{true,""};}, "activate goal state at random location");
    command_center.add_command({"set prune tree"}, [this]()->ret_t{return{true,""};}, "prune search tree");
    command_center.add_command({"unset prune tree"}, [this]()->ret_t{return{true,""};}, "don't prune search tree");
    command_center.add_command({"set png"}, [this]()->ret_t{return{true,""};}, "save a png image of the maze on transition");
    command_center.add_command({"unset png"}, [this]()->ret_t{return{true,""};}, "don't save a png image of the maze on transition");
    command_center.add_command({"set maze"}, [this]()->ret_t{return{true,""};}, "display available maze names");
    command_center.add_command({"set maze"}, [this](QString)->ret_t{return{true,""};}, "load maze with name <string>");
    command_center.add_command({"set cheese"}, [this]()->ret_t{return{true,""};}, "load cheese maze");
    command_center.add_command({"set color"}, [this]()->ret_t{return{true,""};}, "color maze (if applicable)");
    command_center.add_command({"unset color"}, [this]()->ret_t{return{true,""};}, "don't color maze");
    //-----------------------------------Maze-----------------------------------
    command_center.add_command({"left","l"}, [this]()->ret_t{return{true,""};}, "move left");
    command_center.add_command({"right","r"}, [this]()->ret_t{return{true,""};}, "move right");
    command_center.add_command({"up","u"}, [this]()->ret_t{return{true,""};}, "move up");
    command_center.add_command({"down","d"}, [this]()->ret_t{return{true,""};}, "move down");
    command_center.add_command({"stay","s"}, [this]()->ret_t{return{true,""};}, "stay-action");
    command_center.add_command({"move"}, [this]()->ret_t{return{true,""};}, "move one step");
    command_center.add_command({"move"}, [this](int)->ret_t{return{true,""};}, "start moving every <int> milliseconds");
    command_center.add_command({"move stop"}, [this]()->ret_t{return{true,""};}, "stop moving");
    command_center.add_command({"epsilon"}, [this]()->ret_t{return{true,""};}, "get random transition probability");
    command_center.add_command({"epsilon"}, [this](double)->ret_t{return{true,""};}, "set random transition probability");
    command_center.add_command({"reward activation","ra"}, [this](int)->ret_t{return{true,""};}, "print mean reward activation probability for length-<int> random walk");
    command_center.add_command({"random distribution","rd"}, [this](int)->ret_t{return{true,""};}, "run <int> random transitions and display relative counts for all states");
    //------------------------------Model Learning------------------------------
    command_center.add_command({"episode","e"}, [this](int)->ret_t{return{true,""};}, "record length <int> episode");
    command_center.add_command({"episode clear","e c"}, [this]()->ret_t{return{true,""};}, "clear episode data");
    //=== CRF ===
    command_center.add_command({"crf optimize","co"}, [this]()->ret_t{return{true,""};}, "optimize CRF");
    command_center.add_command({"crf optimize","co"}, [this](int)->ret_t{return{true,""};}, "optimize CRF with a maximum of <int> iterations");
    command_center.add_command({"crf check"}, [this]()->ret_t{return{true,""};}, "CRF check derivatives");
    command_center.add_command({"score-crf","scrf"}, [this](int)->ret_t{return{true,""};}, "score candidate features with distance <int> by gradient");
    command_center.add_command({"score1D-crf","s1Dcrf"}, [this](int)->ret_t{return{true,""};}, "score candidate features with distance <int> by 1D optimization");
    command_center.add_command({"add-crf"}, [this](int)->ret_t{return{true,""};}, "add <int> highest scored candidate features to active (0 for all non-zero scored)");
    command_center.add_command({"crf-erase","ce"}, [this]()->ret_t{return{true,""};}, "erase features with zero weight");
    command_center.add_command({"l1 "}, [this](double)->ret_t{return{true,""};}, "coefficient for L1 regularization");
    command_center.add_command({"evaluate"}, [this]()->ret_t{return{true,""};}, "evaluate features at current point");
    command_center.add_command({"crf-f"}, [this]()->ret_t{return{true,""};}, "print CRF features and weights");
    command_center.add_command({"apply-old-features","aof"}, [this]()->ret_t{return{true,""};}, "re-apply the old featues stored at the last erase");
    //=== UTree ===
    command_center.add_command({"expand","ex"}, [this](int)->ret_t{return{true,""};}, "expand <int> leaf nodes");
    command_center.add_command({"expand","ex"}, [this](double)->ret_t{return{true,""};}, "expand leaves until a score of <double> is reached");
    command_center.add_command({"print utree"}, [this]()->ret_t{return{true,""};}, "print the current UTree");
    command_center.add_command({"print leaves"}, [this]()->ret_t{return{true,""};}, "print leaves of the current UTree");
    command_center.add_command({"clear utree"}, [this]()->ret_t{return{true,""};}, "clear UTree");
    command_center.add_command({"v-iteration","vi"}, [this]()->ret_t{return{true,""};}, "run one step of Value-Iteration");
    command_center.add_command({"v-iteration","vi"}, [this](int)->ret_t{return{true,""};}, "run one <int> steps of Value-Iteration");
    command_center.add_command({"ex-type","ext"}, [this]()->ret_t{return{true,""};}, "get expansion type for UTree");
    command_center.add_command({"ex-type-utility","ext u"}, [this]()->ret_t{return{true,""};}, "set expansion type for UTree");
    command_center.add_command({"ex-type-observation","ext o"}, [this]()->ret_t{return{true,""};}, "set expansion type for UTree");
    command_center.add_command({"ex-type-reward","ext r"}, [this]()->ret_t{return{true,""};}, "set expansion type for UTree");
    command_center.add_command({"utree-f"}, [this]()->ret_t{return{true,""};}, "print UTree features");
    //=== Linear-Q ===
    command_center.add_command({"lq-optimize-ridge","lqor"}, [this](double)->ret_t{return{true,""};}, "optimize Linear-Q (TD Error) with L2-regularization coefficient <double>");
    command_center.add_command({"lq-optimize-l1","lqol1"}, [this](double)->ret_t{return{true,""};}, "optimize Linear-Q (TD Error) with L1-regularization coefficient <double>");
    command_center.add_command({"lq-optimize-l1","lqol1"}, [this](double,int)->ret_t{return{true,""};}, "maximum of <int> iterations");
    command_center.add_command({"lq-optimize-l1-check","lqol1-c"}, [this]()->ret_t{return{true,""};}, "check derivatives");
    command_center.add_command({"lq-optimize","lqo"}, [this](double)->ret_t{return{true,""};}, "optimize Linear-Q (Bellman Error) with L1-regularization coefficient <double>");
    command_center.add_command({"lq-optimize","lqo"}, [this](double,int)->ret_t{return{true,""};}, "maximum of <int> iterations");
    command_center.add_command({"lq-optimize-check","lqo-c"}, [this]()->ret_t{return{true,""};}, "check derivatives");
    command_center.add_command({"construct","con"}, [this](int)->ret_t{return{true,""};}, "construct candidate features with distance <int>");
    command_center.add_command({"score-lq","slq"}, [this](int)->ret_t{return{true,""};}, "score candidate features with distance <int> by gradient");
    command_center.add_command({"add-lq"}, [this](int)->ret_t{return{true,""};}, "add <int> highest scored candidate features to active (0 for all non-zero scored)");
    command_center.add_command({"lq-erase","lqe"}, [this]()->ret_t{return{true,""};}, "erase features with zero weight");
    command_center.add_command({"lq-erase","lqe"}, [this](double)->ret_t{return{true,""};}, "erase features with weight below or equal to <double>");
    command_center.add_command({"lq-alpha"}, [this]()->ret_t{return{true,""};}, "get alpha for Soft-Max");
    command_center.add_command({"lq-alpha"}, [this](double)->ret_t{return{true,""};}, "set alpha for Soft-Max");
    //---------------------------------Planning----------------------------------
    command_center.add_command({"discount"}, [this]()->ret_t{return{true,""};}, "get discount");
    command_center.add_command({"discount"}, [this](double)->ret_t{return{true,""};}, "set discount");
    command_center.add_command({"print tree"}, [this]()->ret_t{return{true,""};}, "print Look-Ahead-Tree statistics");
    command_center.add_command({"print tree graphic","p t g"}, [this]()->ret_t{return{true,""};}, "print Look-Ahead-Tree statistics with additional graphical output");
    command_center.add_command({"print tree text","p t t"}, [this]()->ret_t{return{true,""};}, "print Look-Ahead-Tree statistics with additional long text output");
    command_center.add_command({"max-tree-size"}, [this](int)->ret_t{return{true,""};}, "set maximum size of Look-Ahead-Tree (zero for infinite)");
    //---------------------------------New Stuff----------------------------------
    command_center.add_command({"col-states"}, [this]()->ret_t{return{true,""};}, "color states (random)");
    command_center.add_command({"fixed-dt-dist","fdd"}, [this](int)->ret_t{return{true,""};}, "show probability for a state to occur <int> steps after current state");
    command_center.add_command({"pair-delay-dist","pdd"}, [this]()->ret_t{return{true,""};}, "show temporal delay distribution from current state to goal state");
    command_center.add_command({"pair-delay-dist","pdd"}, [this](int)->ret_t{return{true,""};}, "restrict to time window of width <int>");
    command_center.add_command({"mediator-probability","mp"}, [this](int)->ret_t{return{true,""};}, "show probability for a state to occurr between current state and goal state given a time window of width <int>");

    // set_s += "\n" + option_1_s;
    // set_s += "\n" + option_2_s;
    // set_s += "\n" + option_3_s;
    // set_s += "\n" + option_3a_s;
    // set_s += "\n" + option_3b_s;
    // set_s += "\n" + option_3c_s;
    // set_s += "\n" + option_3d_s;
    // set_s += "\n" + option_3e_s;
    // set_s += "\n" + option_3f_s;
    // set_s += "\n" + option_3g_s;
    // set_s += "\n" + option_4_s;
    // set_s += "\n" + option_5_s;
    // set_s += "\n" + option_6_s;
    // set_s += "\n" + option_7_s;
    // set_s += "\n" + option_8_s;
    // set_s += "\n" + option_9_s;

    // QString invalid_args_s( "    invalid arguments" );

    // // getting input arguments
    // vector<QString> str_args;
    // QString tmp_s;
    // vector<int> int_args;
    // vector<bool> int_args_ok;
    // int tmp_i;
    // vector<double> double_args;
    // vector<bool> double_args_ok;
    // double tmp_d;
    // for(int arg_idx=0; arg_string(input,arg_idx,tmp_s) && tmp_s!=""; ++arg_idx) {

    //     str_args.push_back(tmp_s);

    //     int_args_ok.push_back( arg_int(input,arg_idx,tmp_i) );
    //     int_args.push_back( tmp_i );

    //     double_args_ok.push_back( arg_double(input,arg_idx,tmp_d) );
    //     double_args.push_back( tmp_d );
    // }
    // int str_args_n = str_args.size();

    // process input
//    if(str_args_n>0) {
//        if(str_args[0]=="help" || str_args[0]=="h") { // help
//            // Headline
//            to_console( headline_s );
//            // General
//            to_console( general_s );
//            to_console( help_s );
//            to_console( exit_s );
//            to_console( set_s );
//            to_console( test_s );
//            // Maze
//            to_console( maze_s );
//            to_console( left_s );
//            to_console( right_s );
//            to_console( up_s );
//            to_console( down_s );
//            to_console( stay_s );
//            to_console( move_s );
//            to_console( epsilon_s );
//            to_console( reward_activation_s );
//            to_console( random_distribution_s );
//            // Learning
//            to_console( learning_s );
//            to_console( episode_s );
//            to_console( learning_crf_s ); // CRF
//            to_console( optimize_crf_s );
//            to_console( score_crf_s );
//            to_console( score1D_crf_s );
//            to_console( add_crf_s );
//            to_console( crf_erase_s );
//            to_console( l1_s );
//            to_console( evaluate_s );
////            to_console( validate_s );
//            to_console( examine_crf_features_s );
//            to_console( apply_old_crf_features_s );
//            to_console( learning_utree_s ); // UTree
//            to_console( expand_leaf_nodes_s );
//            to_console( print_utree_s );
//            to_console( print_leaves_s );
//            to_console( clear_utree_s );
//            to_console( utree_value_iteration_s );
//            to_console( utree_expansion_type_s );
//            to_console( examine_utree_features_s );
//            to_console( learning_linQ_s ); // linear-Q
//            to_console( optimize_linQ_ridge_s );
//            to_console( optimize_linQ_l1_s );
//            to_console( optimize_linQ_s );
//            to_console( construct_s );
//            to_console( score_lq_s );
//            to_console( add_lq_s );
//            to_console( lq_erase_zero_weight_s );
//            to_console( lq_alpha_s );
//            // Planning
//            to_console( planning_s );
//            to_console( discount_s );
//            to_console( print_look_ahead_tree_s );
//            to_console( max_tree_size_s );
//            // New
//            to_console( new_s );
//            to_console( color_states_s );
//            to_console( fixed_dt_distribution_s );
//            to_console( pair_delay_distribution_s );
//            to_console( mediator_probability_s );
//        } else if(str_args[0]=="left" || str_args[0]=="l") { // left
//            if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
//                perform_transition(action_ptr_t(new MazeAction("left")));
//            } else if(dynamic_pointer_cast<CheeseMaze>(environment)!=nullptr) {
//                perform_transition(action_ptr_t(new CheeseMazeAction("west")));
//            } else {
//                to_console("    Not defined in current environment");
//            }
//        } else if(str_args[0]=="right" || str_args[0]=="r") { // right
//            if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
//                perform_transition(action_ptr_t(new MazeAction("right")));
//            } else if(dynamic_pointer_cast<CheeseMaze>(environment)!=nullptr) {
//                perform_transition(action_ptr_t(new CheeseMazeAction("east")));
//            } else {
//                to_console("    Not defined in current environment");
//            }
//        } else if(str_args[0]=="up" || str_args[0]=="u") { // up
//            if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
//                perform_transition(action_ptr_t(new MazeAction("up")));
//            } else if(dynamic_pointer_cast<CheeseMaze>(environment)!=nullptr) {
//                perform_transition(action_ptr_t(new CheeseMazeAction("north")));
//            } else {
//                to_console("    Not defined in current environment");
//            }
//        } else if(str_args[0]=="down" || str_args[0]=="d") { // down
//            if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
//                perform_transition(action_ptr_t(new MazeAction("down")));
//            } else if(dynamic_pointer_cast<CheeseMaze>(environment)!=nullptr) {
//                perform_transition(action_ptr_t(new CheeseMazeAction("south")));
//            } else {
//                to_console("    Not defined in current environment");
//            }
//        } else if(str_args[0]=="stay" || str_args[0]=="s") { // stay
//            if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
//                perform_transition(action_ptr_t(new MazeAction("stay")));
//            } else {
//                to_console("    Not defined in current environment");
//            }
//        } else if(str_args[0]=="move") { // start/stop moving
//            if(str_args_n==1) {
//                choose_action();
//            } else if(str_args[1]=="stop") {
//                action_timer->stop();
//            } else if(int_args_ok[1] && int_args[1]>=0){
//                action_timer->stop();
//                action_timer->start(int_args[1]);
//            } else {
//                to_console( invalid_args_s );
//                to_console( move_s );
//            }
//        } else if(str_args[0]=="episode" || str_args[0]=="e") { // record episode
//            if(str_args_n==1) {
//                to_console( invalid_args_s );
//                to_console( episode_s );
//            } else if( str_args[1]=="clear" || str_args[1]=="c" ) {
//                clear_data();
//            } else if(int_args_ok[1] && int_args[1]>=0){
//                collect_episode(int_args[1]);
//            } else {
//                to_console( invalid_args_s );
//                to_console( episode_s );
//            }
//        } else if(str_args[0]=="crf-optimize" || str_args[0]=="co") { // optimize CRF
//            if(str_args_n==1 || int_args_ok[1] ) {
//                if(int_args_ok[1]) {
//                    if(str_args_n==4 && double_args_ok[2] && double_args_ok[3] ) {
//                        crf->optimize_model(l1_factor, int_args[1], nullptr, true, double_args[2], double_args[3]);
//                    } else {
//                        crf->optimize_model(l1_factor, int_args[1]);
//                    }
//                } else {
//                    crf->optimize_model(l1_factor, 0);
//                }
//            } else if(str_args[1]=="check" || str_args[1]=="c") {
//                crf->check_derivatives(3,10,1e-6,1e-3);
//            } else {
//                to_console( invalid_args_s );
//                to_console( optimize_crf_s );
//            }
//        } else if(str_args[0]=="utree-f") {
//            utree->print_features();
//        } else if(str_args[0]=="apply-old-features" || str_args[0]=="aof") {
//            crf->apply_features();
//        } else if(str_args[0]=="lq-optimize-ridge" || str_args[0]=="lqor") { // optimize linear-Q
//            if(str_args_n==1 || double_args_ok[1] ) {
//                if(double_args_ok[1]) {
//                    linQ->set_optimization_type_TD_RIDGE()
//                        .set_regularization(double_args[1])
//                        .optimize();
//                } else {
//                    linQ->set_optimization_type_TD_RIDGE()
//                        .set_regularization(0)
//                        .optimize();
//                }
//            } else {
//                to_console( invalid_args_s );
//                to_console( optimize_linQ_ridge_s );
//            }
//        } else if(str_args[0]=="lq-optimize-l1" || str_args[0]=="lqol1") { // optimize linear-Q
//            if(str_args_n>1 && double_args_ok[1] ) {
//                if(str_args_n>2 && int_args_ok[2] ) {
//                    linQ->set_optimization_type_TD_L1()
//                        .set_regularization(double_args[1])
//                        .set_maximum_iterations(int_args[2])
//                        .optimize();
//                } else {
//                    linQ->set_optimization_type_TD_L1()
//                        .set_regularization(double_args[1])
//                        .optimize();
//                }
//            } else if(str_args_n>1 && (str_args[1]=="check" || str_args[1]=="c") ) {
//                linQ->check_derivatives(3,10,1e-6,1e-3);
//            } else {
//                to_console( invalid_args_s );
//                to_console( optimize_linQ_l1_s );
//            }
//        } else if(str_args[0]=="lq-optimize" || str_args[0]=="lqo") { // optimize linear-Q
//            if(str_args_n>1 && double_args_ok[1] ) {
//                if(str_args_n>2 && int_args_ok[2] ) {
//                    linQ->set_optimization_type_BELLMAN()
//                        .set_regularization(double_args[1])
//                        .set_maximum_iterations(int_args[2])
//                        .optimize();
//                } else {
//                    linQ->set_optimization_type_BELLMAN()
//                        .set_regularization(double_args[1])
//                        .set_maximum_iterations(0)
//                        .optimize();
//                }
//            } else if(str_args_n>1 && (str_args[1]=="check" || str_args[1]=="c") ) {
//                linQ->check_derivatives(10,10,1e-6,1e-3);
//            } else {
//                to_console( invalid_args_s );
//                to_console( optimize_linQ_s );
//            }
//        } else if(str_args[0]=="lq-erase" || str_args[0]=="lqe") {
//            if(str_args_n==1 || double_args_ok[1]) {
//                if(str_args_n>1) {
//                    linQ->erase_features_by_weight(double_args[1]);
//                } else {
//                    linQ->erase_features_by_weight();
//                }
//            } else {
//                to_console( invalid_args_s );
//                to_console( lq_erase_zero_weight_s );
//            }
//        } else if(str_args[0]=="lq-alpha") {
//            if(str_args_n==1 || double_args_ok[1]) {
//                if(str_args_n>1) {
//                    linQ->set_alpha(double_args[1]);
//                } else {
//                    double alpha = linQ->get_alpha();
//                    to_console( QString("    alpha = %1").arg(alpha) );
//                }
//            } else {
//                to_console( invalid_args_s );
//                to_console( lq_alpha_s );
//            }
//        } else if(str_args[0]=="epsilon") {
//            shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
//            if(maze==nullptr) {
//                to_console("    Allowed for maze-environments only");
//            } else {
//                if(str_args_n==1) {
//                    if(maze->get_epsilon()!=epsilon) {
//                        DEBUG_ERROR("Maze epsion (" << maze->get_epsilon() << ") is different from stored one (" << epsilon << ")");
//                    }
//                    to_console( QString("    maze epsilon is %1").arg(epsilon) );
//                } else if(double_args_ok[1] && double_args[1]>=0 && double_args[1]<=1) {
//                    epsilon = double_args[1];
//                    maze->set_epsilon(epsilon);
//                } else {
//                    to_console( invalid_args_s );
//                    to_console( epsilon_s );
//                }
//            }
//        } else if(str_args[0]=="reward-activation" || str_args[0]=="ra") {
//            shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
//            if(maze==nullptr) {
//                to_console("    Allowed for maze-environments only");
//            } else {
//                if(str_args_n==2 && int_args_ok[1]) {
//                    maze->print_reward_activation_on_random_walk(int_args[1]);
//                    maze->render_update();
//                } else {
//                    to_console(invalid_args_s);
//                    to_console(reward_activation_s);
//                }
//            }
//        } else if(str_args[0]=="random-distribution" || str_args[0]=="rd") { // test
//            shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
//            if(maze==nullptr) {
//                to_console("    Allowed for maze-environments only");
//            } else {
//                if(str_args_n==2 && int_args_ok[1]) {
//                    // initialize state counts to zero
//                    map<observation_ptr_t,int> state_counts;
//                    for(observation_ptr_t observation : observation_space) {
//                        state_counts[observation] = 0;
//                    }
//                    // get state counts
//                    int n = int_args[1];
//                    int max_count = 1;
//                    for(int idx=0; idx<n; ++idx) {
//                        action_ptr_t action = (action_ptr_t)(action_space->random_element());
//                        observation_ptr_t observation_to;
//                        reward_ptr_t reward;
//                        maze->perform_transition(action,observation_to,reward);
//                        ++state_counts[observation_to];
//                        max_count = max(state_counts[observation_to],max_count);
//                    }
//                    // transform into colors
//                    Maze::color_vector_t cols;
//                    for(observation_ptr_t observation : observation_space) {
//                        double p = state_counts[observation];
//                        DEBUG_OUT(0,"State " << observation << ": p = " << p/max(n,1) );
//                        p /= max_count;
//                        cols.push_back( std::make_tuple(1,1-p,1-p) );
//                    }
//                    maze->set_state_colors(cols);
//                    maze->render_update();
//                } else {
//                    to_console( invalid_args_s );
//                    to_console( random_distribution_s );
//                }
//            }
//        } else if(str_args[0]=="expand" || str_args[0]=="ex") {
//            if(str_args_n==1) {
//                double score = utree->expand_leaf_node();
//                to_console( QString("    Score was %1").arg(score) );
//            } else if(int_args_ok[1] && int_args[1]>=0 ) {
//                double score = 0;
//                for(int i=0; i<int_args[1]; ++i) {
//                    score = utree->expand_leaf_node();
//                }
//                to_console( QString("    Last score was %1").arg(score) );
//            } else if(double_args_ok[1]) {
//                while( double_args[1] <= utree->expand_leaf_node(double_args[1]) ) {}
//            } else {
//                to_console( invalid_args_s );
//                to_console( expand_leaf_nodes_s );
//            }
//        } else if(str_args[0]=="print-utree") {
//            utree->print_tree();
//        } else if(str_args[0]=="print-leaves") {
//            utree->print_leaves();
//        } else if(str_args[0]=="clear-utree") {
//            utree->clear_tree();
//        } else if(str_args[0]=="v-iteration" || str_args[0]=="vi") {
//            if( str_args_n==1 || (str_args_n>1 && int_args_ok[1] && int_args[1]>0) ) {
//                double max_diff;
//                int rep = str_args_n==1 ? 1 : int_args[1];
//                repeat(rep) {
//                    max_diff = utree->value_iteration();
//                }
//                to_console( QString(    "run %1 iteration(s), last maximum update was %2").arg(rep).arg(max_diff) );
//            } else {
//                to_console( invalid_args_s );
//                to_console( utree_value_iteration_s );
//            }
//        } else if(str_args[0]=="ex-type" || str_args[0]=="ext") {
//            if(str_args_n==1) {
//                switch(utree->get_expansion_type()) {
//                case UTree::UTILITY_EXPANSION:
//                    to_console("    expansion type: UTILITY_EXPANSION");
//                    break;
//                case UTree::OBSERVATION_REWARD_EXPANSION:
//                    to_console("    expansion type: OBSERVATION_REWARD_EXPANSION");
//                    break;
//                default:
//                    DEBUG_DEAD_LINE;
//                }
//            } else if(str_args[1]=="utility" || str_args[1]=="u") {
//                utree->set_expansion_type(UTree::UTILITY_EXPANSION);
//                to_console("    expansion type: UTILITY_EXPANSION");
//            } else if(str_args[1]=="observationreward" ||str_args[1]=="or") {
//                utree->set_expansion_type(UTree::OBSERVATION_REWARD_EXPANSION);
//                to_console("    expansion type: OBSERVATION_REWARD_EXPANSION");
//            } else {
//                to_console( invalid_args_s );
//                to_console( utree_expansion_type_s );
//            }
//        } else if(str_args[0]=="discount") {
//            if(str_args_n==1) {
//                to_console( QString("    discount is %1").arg(discount) );
//            } else if(double_args_ok[1] && double_args[1]>=0 && double_args[1]<=1) {
//                discount = double_args[1];
//                utree->set_discount(discount);
//                linQ->set_discount(discount);
//                shared_ptr<LookAheadPolicy> look_ahead_policy = dynamic_pointer_cast<LookAheadPolicy>(policy);
//                if(look_ahead_policy!=nullptr) {
//                    look_ahead_policy->set_discount(discount);
//                }
//            } else {
//                to_console( invalid_args_s );
//                to_console( discount_s );
//            }
//        } else if(str_args[0]=="evaluate") {
//            crf->evaluate_features();
//        // } else if(str_args[0]=="validate" || str_args[0]=="v") {
//        //     if(str_args_n==1 ) {
//        //         to_console( invalid_args_s );
//        //         to_console( validate_s );
//        //     } else if(str_args[1]=="crf") {
//        //         if(str_args[2]=="mc") {
//        //             if( str_args_n>3 && int_args_ok[3] && int_args[3]>0 ) {
//        //                 probability_t model_l, maze_l;
//        //                 probability_t kl = maze.validate_model(
//        //                         crf,
//        //                         int_args[3],
//        //                         &model_l,
//        //                         &maze_l
//        //                 );
//        //                 to_console(QString("    MC KL-Divergence = %1 (%2 samples)").arg(kl).arg(int_args[3]));
//        //                 to_console(QString("    Mean Likelihood: model = %1, maze = %2").arg(model_l).arg(maze_l));
//        //             } else {
//        //                 to_console( "    Please specify a valid sample size" );
//        //             }
//        //         } else {
//        //             to_console( invalid_args_s );
//        //             to_console( validate_s );
//        //         }
//        //     } else if(str_args[1]=="kmdp") {
//        //         to_console( "    Sorry, not implemented" );
//        //     } else {
//        //         to_console( invalid_args_s );
//        //         to_console( validate_s );
//        //     }
//        } else if(str_args[0]=="crf-f") {
//            crf->print_all_features();
//        } else if(str_args[0]=="l1") {
//            if(str_args_n==1) {
//                to_console(QString("    %1").arg(l1_factor));
//            } else if(double_args_ok[1] && double_args[1]>=0) {
//                l1_factor = double_args[1];
//            } else {
//                to_console( invalid_args_s );
//                to_console( l1_s );
//            }
//        } else if(str_args[0]=="score-crf" || str_args[0]=="scrf") {
//            if(str_args_n==1) {
//                to_console(score_crf_s);
//            } else if(int_args_ok[1] && int_args[1]>=0 ) {
//                crf->construct_candidate_features(int_args[1]);
//                crf->score_candidates_by_gradient();
//                crf->print_scores();
//            } else {
//                to_console( invalid_args_s );
//                to_console( score_crf_s );
//            }
//        } else if(str_args[0]=="score1D-crf" || str_args[0]=="s1Dcrf") {
//            if(str_args_n==1) {
//                to_console(score1D_crf_s);
//            } else if(int_args_ok[1] && int_args[1]>=0 ) {
//                crf->construct_candidate_features(int_args[1]);
//                crf->score_candidates_by_1D_optimization();
//            } else {
//                to_console( invalid_args_s );
//                to_console( score1D_crf_s );
//            }
//        } else if(str_args[0]=="add-crf") {
//            if(str_args_n==1) {
//                to_console( invalid_args_s );
//                to_console( add_crf_s );
//            } else if(int_args_ok[1] && int_args[1]>=0 ) {
//                crf->add_candidate_features_to_active(int_args[1]);
//            } else {
//                to_console( invalid_args_s );
//                to_console( add_crf_s );
//            }
//        } else if(str_args[0]=="score-lq" || str_args[0]=="slq") {
//            if(str_args_n==1) {
//                to_console(score_lq_s);
//            } else if(int_args_ok[1] && int_args[1]>=0 ) {
//                linQ->construct_candidate_features(int_args[1]);
//                linQ->score_candidates_by_gradient();
//            } else {
//                to_console( invalid_args_s );
//                to_console( score_lq_s );
//            }
//        } else if(str_args[0]=="add-lq") {
//            if(str_args_n==1) {
//                to_console( invalid_args_s );
//                to_console( add_lq_s );
//            } else if(int_args_ok[1] && int_args[1]>=0 ) {
//                linQ->add_candidates_by_score(int_args[1]);
//            } else {
//                to_console( invalid_args_s );
//                to_console( add_lq_s );
//            }
//        } else if(str_args[0]=="crf-erase" || str_args[0]=="ce") {
//            crf->erase_zero_features();
//        } else if(str_args[0]=="exit" || str_args[0]=="quit" || str_args[0]=="q") { // quit application
//            QApplication::quit();
//        } else if(str_args[0]=="print-tree") { // print tree
//            bool graphic = false, text = false;
//            if(str_args_n>1) {
//                if(str_args[1]=="g" || str_args[1]=="graphic") {
//                    graphic = true;
//                } else if(str_args[1]=="t" || str_args[1]=="text") {
//                    text = true;
//                } else {
//                    to_console( invalid_args_s );
//                    to_console( print_look_ahead_tree_s );
//                }
//            }
//            shared_ptr<LookAheadPolicy> look_ahead_policy = dynamic_pointer_cast<LookAheadPolicy>(policy);
//            if(look_ahead_policy!=nullptr) {
//                const LookAheadSearch & s = look_ahead_policy->get_look_ahead_search();
//                s.print_tree_statistics();
//                s.print_tree(text, graphic);
//            }
//        } else if(str_args[0]=="max-tree-size") { // set tree size
//            if(str_args_n==1) {
//                to_console( QString( "    max tree size is %1" ).arg(max_tree_size) );
//            } else if(int_args_ok[1] && int_args[1]>=0) {
//                max_tree_size = int_args[1];
//                shared_ptr<LookAheadPolicy> look_ahead_policy = dynamic_pointer_cast<LookAheadPolicy>(policy);
//                if(look_ahead_policy!=nullptr) {
//                    look_ahead_policy->set_max_tree_size(max_tree_size);
//                }
//            } else {
//                to_console( "    Please specify a valid tree size" );
//            }
//        } else if(str_args[0]=="set" || str_args[0]=="unset") { // set option
//            if(str_args_n==1) {
//                to_console( invalid_args_s );
//                to_console( set_s );
//            } else if(str_args[1]=="record") {
//                if(str_args[0]=="set") {
//                    record = true;
//                    start_new_episode = true;
//                    to_console( "    record on" );
//                } else if(str_args[0]=="unset") {
//                    record = false;
//                    to_console( "    record off" );
//                } else {
//                    DEBUG_DEAD_LINE;
//                }
//            } else if(str_args[1]=="plot") {
//                plot = str_args[0]=="set";
//                if(plot) {
//                    // open plot file
//                    plot_file.open("plot_file.txt");
//                    plot_file << "# action observation reward" << endl;
//                    to_console( "    plot on" );
//                } else {
//                    // close plot file
//                    plot_file.close();
//                    to_console( "    plot off" );
//                }
//            } else if( (str_args[1]=="p" || str_args[1]=="planner") ) {
//                if(str_args_n>2) {
//                    // set planner
//                    if(str_args[0]=="unset") {
//                        to_console( "    set different planner to unset current" );
//                    } else if(str_args[2]=="optimal" || str_args[2]=="o") {
//                        planner_type = OPTIMAL_LOOK_AHEAD;
//                        to_console( "    using optimal planner" );
//                    } else if(str_args[2]=="sparse" || str_args[2]=="s") {
//                        planner_type = SPARSE_LOOK_AHEAD;
//                        to_console( "    using sparse planner" );
//                    } else if(str_args[2]=="utree" || str_args[2]=="u") {
//                        planner_type = UTREE_LOOK_AHEAD;
//                        to_console( "    using UTree planner" );
//                    } else if(str_args[2]=="uv" || str_args[2]=="utree-value") {
//                        planner_type = UTREE_VALUE;
//                        to_console( "    using UTree-value for action selection" );
//                    } else if(str_args[2]=="lq" || str_args[2]=="linear-q") {
//                        planner_type = LINEAR_Q_VALUE;
//                        to_console( "    using linear Q-approximation for action selection" );
//                    } else if(str_args[2]=="g" || str_args[2]=="goal") {
//                        planner_type = GOAL_ITERATION;
//                        to_console( "    using goal-guided value iteration" );
//                    } else if(str_args[2]=="r" || str_args[2]=="random") {
//                        planner_type = RANDOM;
//                        to_console( "    using random policy" );
//                    } else {
//                        to_console( "    unknown planner" );
//                    }
//                    set_policy();
//                } else {
//                    to_console( "    please supply a planner to use" );
//                }
//            } else if(str_args[1]=="maze") {
//                bool print_list = false;
//                if(str_args[0]=="unset") {
//                    to_console( "    set different environment to unset current" );
//                } else {
//                    if(str_args_n>2) {
//                        shared_ptr<Maze> maze(new Maze(epsilon));
//                        bool success = maze->set_maze(str_args[2]);
//                        if(!success) {
//                            to_console("    No maze named '"+str_args[2]+"'");
//                            print_list = true;
//                        }
//                        change_environment(maze);
//                    } else {
//                        print_list = true;
//                    }
//                }
//                if(print_list) {
//                    to_console( "    Available mazes:" );
//                    for(QString name : Maze::get_maze_list()) {
//                        to_console("        "+name);
//                    }
//                }
//            } else if(str_args[1]=="cheese") {
//                if(str_args[0]=="unset") {
//                    to_console( "    set different environment to unset current" );
//                } else {
//                    change_environment(make_shared<CheeseMaze>());
//                }
//            } else if(str_args[1]=="c" || str_args[1]=="color") {
//                if(str_args[0]=="unset") {
//                    color_maze = false;
//                    to_console( "    don't color maze" );
//                } else {
//                    color_maze = true;
//                    to_console( "    color maze" );
//                }
//            } else if(str_args[1]=="g" || str_args[1]=="goal") {
//                shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
//                if(maze==nullptr) {
//                    to_console("    Allowed for maze-environments only");
//                } else {
//                    if(str_args[0]=="set") {
//                        // set flag
//                        goal_activated = true;
//                        // get state
//                        if(str_args_n>2 && (str_args[2]=="r" || str_args[2]=="random")) {
//                            goal_state = observation_space->random_element();
//                        } else {
//                            goal_state = current_instance->observation;
//                        }
//                        // color maze
//                        Maze::color_vector_t cols;
//                        for(auto observation : observation_space) {
//                            if(observation==goal_state) {
//                                cols.push_back( Maze::color_t(0,1,0) );
//                            } else {
//                                cols.push_back( Maze::color_t(1,1,1) );
//                            }
//                        }
//                        maze->set_state_colors(cols);
//                        maze->render_update();
//                        // set goal in GoalIteration policy
//                        shared_ptr<GoalIteration> goal_iteration = dynamic_pointer_cast<GoalIteration>(policy);
//                        if(goal_iteration!=nullptr) {
//                            goal_iteration->set_goal(goal_state);
//                        }
//                        // print message
//                        to_console( "    goal set" );
//                    } else {
//                        if(!goal_activated) {
//                            to_console( "    goal already inactive" );
//                        } else {
//                            goal_activated = false;
//                            to_console( "    goal inactive" );
//                        }
//                    }
//                }
//            } else if(str_args[1]=="prune-tree") {
//                if(str_args[0]=="set") {
//                    prune_search_tree=true;
//                    to_console( "    prune search tree" );
//                } else {
//                    prune_search_tree=false;
//                    to_console( "    don't prune search tree" );
//                }
//                shared_ptr<LookAheadPolicy> look_ahead_policy = dynamic_pointer_cast<LookAheadPolicy>(policy);
//                if(look_ahead_policy!=nullptr) {
//                    look_ahead_policy->set_pruning(prune_search_tree);
//                    look_ahead_policy->invalidate_search_tree();
//                }
//            } else if(str_args[1]=="png") {
//                if(str_args[0]=="set") {
//                    save_png_on_transition=true;
//                    to_console( "    save png on transition" );
//                } else {
//                    save_png_on_transition=false;
//                    to_console( "    don't save png on transition" );
//                }
//            } else {
//                to_console( invalid_args_s );
//                to_console( set_s );
//            }
//        } else if(str_args[0]=="construct" || str_args[0]=="con") {
//            if(str_args_n==1) {
//                to_console(construct_s);
//            } else if(int_args_ok[1] && int_args[1]>=0 ) {
//                linQ->add_all_candidates(int_args[1]);
//            } else {
//                to_console( invalid_args_s );
//                to_console( construct_s );
//            }
//        } else if(str_args[0]=="test") { // test
//            // if(str_args_n==2 && int_args_ok[1]) {
//            //     maze.print_reward_activation_on_random_walk(int_args[1]);
//            //     maze.render_update();
//            // }
//
//            // crf->print_all_features();
//            // utree->print_features();
//
//            probability_t like = crf->cross_validation(10);
//            to_console( QString("    Mean Data Likelihood: %1").arg(like) );
//
//            //to_console( "    currently no test function implemented" );
//        } else if(str_args[0]=="col-states") { // color states
//            shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
//            if(maze==nullptr) {
//                to_console("    Allowed for maze-environments only");
//            } else {
//                Maze::color_vector_t cols;
//                for(observation_ptr_t observation : observation_space) {
//                    cols.push_back( std::make_tuple(drand48(),drand48(),drand48()) );
//                }
//                maze->set_state_colors(cols);
//                maze->render_update();
//            }
//        } else if(str_args[0]=="fixed-dt-dist" || str_args[0]=="fdd") { // show delay probability
//            shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
//            if(maze==nullptr) {
//                to_console("    Allowed for maze-environments only");
//            } else {
//                if(str_args_n!=2 || !int_args_ok[1]) {
//                    to_console( invalid_args_s );
//                    to_console( fixed_dt_distribution_s );
//                } else {
//                    // get probabilites for all states
//                    observation_ptr_t s1 = current_instance->observation;
//                    idx_t delay = int_args[1];
//                    vector<probability_t> probs = delay_dist.get_fixed_delay_probability_distribution(s1,delay);
//
//                    // get maximum probability for rescaling and goal state idx
//                    double max_prob = -DBL_MAX;
//                    idx_t goal_idx = 0, state_idx = 0;
//                    for(observation_ptr_t observation : observation_space) {
//                        if(probs[state_idx]>max_prob) {
//                            max_prob = probs[state_idx];
//                        }
//                        if(observation==goal_state) {
//                            goal_idx=state_idx;
//                        }
//                    }
//
//                    // rescale and define colors
//                    Maze::color_vector_t cols;
//                    for( double p : probs ) {
//                        if(max_prob!=0) {
//                            cols.push_back( std::make_tuple(1,1-p/max_prob,1-p/max_prob) );
//                        } else {
//                            cols.push_back( std::make_tuple(1,1,1) );
//                        }
//                    }
//                    if(goal_activated) {
//                        cols[goal_idx]=std::make_tuple(0,1,0);
//                    }
//
//                    // render
//                    maze->set_state_colors(cols);
//                    maze->render_update();
//                }
//            }
//        } else if(str_args[0]=="pair-delay-dist" || str_args[0]=="pdd") { // show delay distribution
//            if(str_args_n>1 && !int_args_ok[1]) {
//                to_console( invalid_args_s );
//                to_console( pair_delay_distribution_s );
//            } else {
//                if(!goal_activated) {
//                    to_console( "    Goal state must be activated to calculate delay distribution" );
//                } else {
//
//                    // get distribution
//                    int time_window = -1;
//                    if(int_args_ok[1]) {
//                        time_window = int_args[1];
//                    }
//                    DelayDistribution::pair_dist_map_t dist_map;
//                    delay_dist.get_pairwise_delay_distribution(dist_map,time_window);
//
//                    // apply to current states
//                    observation_ptr_t s1 = current_instance->observation;
//                    observation_ptr_t s2 = goal_state;
//                    vector<double> forward;
//                    vector<double> backward;
//                    for( auto el : dist_map ) {
//                        observation_ptr_t s1_map = get<0>(el.first);
//                        observation_ptr_t s2_map = get<1>(el.first);
//                        int dt = get<2>(el.first);
//                        if(dt==0) {
//                            continue;
//                        }
//                        if(s1_map==s1 && s2_map==s2) {
//                            if(dt>(int)forward.size()-1) {
//                                forward.resize(dt+1,0);
//                            }
//                            forward[dt]=el.second;
//                            DEBUG_OUT(3,s1_map << " --> " << s2_map << " (" << dt << ") : " << el.second);
//                        }
//                        if(s1_map==s2 && s2_map==s1) {
//                             if(dt>(int)backward.size()-1) {
//                                backward.resize(dt+1,0);
//                            }
//                            backward[dt]=el.second;
//                            DEBUG_OUT(3,s1_map << " --> " << s1_map << " (" << dt << ") : " << el.second);
//                        }
//                    }
//
//                    // prepare for plotting
//                    large_size_t back_size = backward.size();
//                    large_size_t forw_size = forward.size();
//                    large_size_t point_n = forw_size + back_size - 1;
//                    QVector<double> x(point_n), y(point_n);
//                    probability_t max_y = -DBL_MAX;
//                    for (int i=0; i<(int)point_n; ++i) {
//                        x[i] = i - back_size + 1;
//                        y[i] = i<(int)back_size ? backward[back_size-i-1] : forward[i-back_size];
//                        if(y[i]>max_y) {
//                            max_y = y[i];
//                        }
//                    }
//                    // assign data to graph
//                    plotter->graph(0)->setData(x, y);
//                    // give the axes some labels:
//                    plotter->xAxis->setLabel("delay/steps");
//                    plotter->yAxis->setLabel("probability");
//                    // set axes ranges, so we see all data:
//                    plotter->xAxis->setRange(-(int)backward.size(),forward.size());
//                    plotter->yAxis->setRange(0,max_y);
//                    DEBUG_OUT(3,"Ranges: [" << -(int)backward.size() << ":" << forward.size() << "][" << 0 << ":" << max_y << "]");
//                    plotter->replot();
//                }
//            }
//        } else if(str_args[0]=="mediator-probability" || str_args[0]=="mp") { // show mediator probability
//            shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
//            if(maze==nullptr) {
//                to_console("    Allowed for maze-environments only");
//            } else {
//                if(str_args_n==2 && int_args_ok[1]) {
//                    if(!goal_activated) {
//                        to_console( "    Goal state must be activated to calculate mediator probabilities" );
//                    } else {
//                        // get probabilites for all states
//                        DEBUG_OUT(2,"Calculating mediator distribution...");
//                        observation_ptr_t s1 = current_instance->observation;
//                        observation_ptr_t s3 = goal_state;
//                        vector<double> probs;
//                        double max_prob = -DBL_MAX;
//                        idx_t goal_idx = 0, state_idx = 0;
//                        for(observation_ptr_t observation : observation_space) {
//                            probability_t prob = delay_dist.get_mediator_probability(s1,observation,s3,int_args[1]);
//                            probs.push_back(prob);
//                            if(prob>max_prob) {
//                                max_prob = prob;
//                            }
//                            if(observation==goal_state) {
//                                goal_idx=state_idx;
//                            }
//                            DEBUG_OUT(3,"    " << s1 << " --> " << observation << " --> " << s3 << " : " << prob);
//                            ++state_idx;
//                        }
//                        DEBUG_OUT(2,"DONE");
//                        // rescale and define colors
//                        Maze::color_vector_t cols;
//                        for( double prob : probs ) {
//                            if(max_prob!=0) {
//                                cols.push_back( std::make_tuple(1,1-prob/max_prob,1-prob/max_prob) );
//                            } else {
//                                cols.push_back( std::make_tuple(1,1,1) );
//                            }
//                        }
//                        cols[goal_idx]=std::make_tuple(0,1,0);
//                        // render
//                        maze->set_state_colors(cols);
//                        maze->render_update();
//                    }
//                } else {
//                    to_console( invalid_args_s );
//                    to_console( mediator_probability_s );
//                }
//            }
//        } else {
//            to_console("    unknown command");
//        }
//    }
}

TestMaze_II::~TestMaze_II() {
    delete action_timer;
    current_instance->detach_reachable();
    plot_file.close();
}

void TestMaze_II::collect_episode(const int& length) {
    if(policy==nullptr) {
        to_console("    No policy available");
    } else {
        start_new_episode = true;
        for(int idx=0; idx<length; ++idx) {
            action_ptr_t action = policy->get_action(current_instance);
            observation_ptr_t observation_to;
            reward_ptr_t reward;
            environment->perform_transition(action,observation_to,reward);
            update_current_instance(action,observation_to,reward);
            add_action_observation_reward_tripel(action,observation_to,reward);
        }
        render_update();
    }
}

void TestMaze_II::update_current_instance(action_ptr_t action, observation_ptr_t observation, reward_ptr_t reward, bool invalidate_search_tree) {
    if(current_instance==INVALID) {
        DEBUG_OUT(3,"Current instance INVALID. Creating.");
        current_instance = DoublyLinkedInstance::create(action,observation,reward);
    } else {
        DEBUG_OUT(3,"Current instance is " << current_instance << ". Appending.");
        current_instance = current_instance->append(action,observation,reward);
    }
    if(plot) {
        plot_file << action << " " << observation << " " << reward << endl;
    }
    if(invalidate_search_tree) {
        shared_ptr<LookAheadPolicy> look_ahead_policy = dynamic_pointer_cast<LookAheadPolicy>(policy);
        if(look_ahead_policy!=nullptr) {
            look_ahead_policy->invalidate_search_tree();
        }
    }
}

void TestMaze_II::add_action_observation_reward_tripel(
    const action_ptr_t& action,
    const observation_ptr_t& observation,
    const reward_ptr_t& reward
    ) {
           crf->add_action_observation_reward_tripel(action,observation,reward,start_new_episode);
         utree->add_action_observation_reward_tripel(action,observation,reward,start_new_episode);
          linQ->add_action_observation_reward_tripel(action,observation,reward,start_new_episode);
    delay_dist.add_action_observation_reward_tripel(action,observation,reward,start_new_episode);
    DEBUG_OUT(1,"Add (" << action << ", " << observation << ", " << reward << ")" <<
              (start_new_episode?" new episode":""));
    start_new_episode = false;
}

void TestMaze_II::clear_data() {
    crf->clear_data();
    utree->clear_data();
    linQ->clear_data();
    delay_dist.clear_data();
}

void TestMaze_II::save_to_png(QString file_name) const {
    DEBUG_OUT(1,"Saving Maze to file '" << file_name << "'");
    QGraphicsScene * scene = ui.graphicsView->scene();
    QImage img(2000,2000,QImage::Format_ARGB32_Premultiplied);
    QPainter p(&img);
    scene->render(&p);
    p.end();
    bool ok = img.save(file_name);
    if(!ok) {
        DEBUG_OUT(1,"    Error: could not save maze");
    }
}

void TestMaze_II::perform_transition(const action_ptr_t& action) {
    observation_ptr_t s;
    reward_ptr_t r;
    perform_transition(action, s, r);
}

void TestMaze_II::perform_transition(const action_ptr_t& action, observation_ptr_t& observation_to, reward_ptr_t& reward) {
    environment->perform_transition(action,observation_to,reward);
    update_current_instance(action,observation_to,reward);
    render_update();
    if(record) {
        add_action_observation_reward_tripel(action,observation_to,reward);
    }
    if(save_png_on_transition) {
        QString file_name = QString("Environment_%1.png").arg(QString::number(png_counter++),(int)4,QChar('0'));
        save_to_png(file_name);
    }
}

void TestMaze_II::change_environment(shared_ptr<Environment> new_environment) {

    // clear old environment
    if(environment!=nullptr) {
        // tear down visualization
        shared_ptr<Visualizer> vis = dynamic_pointer_cast<Visualizer>(environment);
        if(vis!=nullptr) {
            vis->render_tear_down();
        }
        // destroy current environment
        environment.reset();
    }

    // clear learners
    clear_all_learners();

    // set new environment
    if(new_environment!=nullptr) {
        // set new environment
        environment = new_environment;
        // initialize visualization
        shared_ptr<Visualizer> vis = dynamic_pointer_cast<Visualizer>(environment);
        if(vis!=nullptr) {
            vis->render_initialize(ui.graphicsView);
            vis->render_update();
        }
        // get/set all spaces
        environment->get_spaces(action_space,observation_space,reward_space);
        utree->set_spaces(*environment);
        utree->set_features(*environment);
        crf->set_spaces(*environment);
        crf->set_features(*environment);
        linQ->set_spaces(*environment);
        linQ->set_features(*environment);
        // set current instance
        current_instance->detach_reachable();
        current_instance = INVALID;
        shared_ptr<PredictiveEnvironment> pred = dynamic_pointer_cast<PredictiveEnvironment>(environment);
        if(pred!=nullptr) {
            const_instance_ptr_t env_instance = pred->get_current_instance();
            current_instance = DoublyLinkedInstance::create(env_instance->action,env_instance->observation,env_instance->reward,env_instance->const_prev(),INVALID);
        }
    }

    // set policy
    set_policy();
}

void TestMaze_II::clear_all_learners() {
    crf.reset(new KMarkovCRF());
    utree.reset(new UTree(discount));
    linQ.reset(new LinearQ(discount));
}

void TestMaze_II::set_policy() {
    switch(planner_type) {
    case RANDOM:
        policy.reset(new RandomPolicy(action_space));
        break;
    case OPTIMAL_LOOK_AHEAD: {
        shared_ptr<Predictor> pred = dynamic_pointer_cast<Predictor>(environment);
        if(pred!=nullptr) {
            policy.reset(new LookAheadPolicy(discount,pred,prune_search_tree,max_tree_size));
        } else {
            DEBUG_ERROR("No optimal look-ahead planning possible because environment does not provide predictions.");
        }
        break;
    }
    case SPARSE_LOOK_AHEAD: {
        shared_ptr<Predictor> pred = dynamic_pointer_cast<Predictor>(crf);
        if(pred!=nullptr) {
            policy.reset(new LookAheadPolicy(discount,pred,prune_search_tree,max_tree_size));
        } else {
            DEBUG_DEAD_LINE;
        }
        break;
    }
    case UTREE_LOOK_AHEAD: {
        shared_ptr<Predictor> pred = dynamic_pointer_cast<Predictor>(utree);
        if(pred!=nullptr) {
            policy.reset(new LookAheadPolicy(discount,pred,prune_search_tree,max_tree_size));
        } else {
            DEBUG_DEAD_LINE;
        }
        break;
    }
    case UTREE_VALUE: {
        shared_ptr<Policy> p = dynamic_pointer_cast<Policy>(utree);
        if(p==nullptr) { DEBUG_DEAD_LINE; }
        policy = p;
        break;
    }
    case LINEAR_Q_VALUE: {
        shared_ptr<Policy> p = dynamic_pointer_cast<Policy>(linQ);
        if(p==nullptr) { DEBUG_DEAD_LINE; }
        policy = p;
        break;
    }
    case GOAL_ITERATION: {
        shared_ptr<Predictor> pred = dynamic_pointer_cast<Predictor>(environment);
        if(pred!=nullptr) {
            GoalIteration * p = new GoalIteration(discount,*pred);
            if(goal_activated) {
                p->set_goal(goal_state);
            }
            policy.reset(p);
        } else {
            DEBUG_ERROR("No goal iteration possible because environment does not provide predictions.");
        }
        break;
    }
    case NONE:
    case KMDP_LOOK_AHEAD:
    default:
        policy.reset();
    }
}

void TestMaze_II::render_update() {
    shared_ptr<Visualizer> vis = dynamic_pointer_cast<Visualizer>(environment);
    if(vis!=nullptr) {
        vis->render_update();
    }
}

void TestMaze_II::choose_action() {
    // get action
    action_ptr_t action;
    if(policy==nullptr) {
        action = action_space;
        to_console("    No policy available");
    } else {
        action = policy->get_action(current_instance);
    }
    // color maze
    if(color_maze) {
        shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
        shared_ptr<GoalIteration> gi = dynamic_pointer_cast<GoalIteration>(policy);
        if(maze!=nullptr && gi!=nullptr) {
            maze->set_state_colors(gi->get_value_as_color());
        }
    } else {
        shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
        if(maze!=nullptr) {
            maze->set_state_colors();
        }
    }
    // perform action
    observation_ptr_t observation_to;
    reward_ptr_t reward;
    perform_transition(action,observation_to,reward);
}

void TestMaze_II::process_console_input() {

    // get input from console and process
    QString input = ui._wConsoleInput->text();
    ui._wConsoleInput->setText("");
    to_console(input);
    console_history.push_back(input);
    history_position = console_history.size();
    QTextStream history_file_stream(&history_file);
    history_file_stream << input << "\n";

    // execute command and print response message
    to_console(command_center.execute(input),4);

    return;
}

void TestMaze_II::back_in_history() {
    if(history_position>0) {
        --history_position;
    }
    ui._wConsoleInput->setText(console_history[history_position]);
}

void TestMaze_II::forward_in_history() {
    if(history_position==console_history.size()) {
        return;
    } else if(history_position<console_history.size()-1) {
        ++history_position;
    }
    ui._wConsoleInput->setText(console_history[history_position]);
}

bool MoveByKeys::eventFilter(QObject *obj, QEvent *event)
{
    // process some key events
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
	switch(keyEvent->key()) {
	case Qt::Key_Left: // left
            maze->command_center.execute(QString('l'));
            return true;
	case Qt::Key_Right: // right
            maze->command_center.execute(QString('r'));
            return true;
	case Qt::Key_Up: // up
            maze->command_center.execute(QString('u'));
            return true;
	case Qt::Key_Down: // down
            maze->command_center.execute(QString('d'));
            return true;
	case Qt::Key_Space: // space
            maze->command_center.execute(QString('s'));
            return true;
	case Qt::Key_Return:
	case Qt::Key_Enter:
	default:
            break;
	}
    }

    // standard event processing
    return QObject::eventFilter(obj, event);
}
