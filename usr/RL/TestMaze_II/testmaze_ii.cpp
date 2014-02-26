#include "testmaze_ii.h"

#include "Config.h"
#include "util.h"
#include "util/QtUtil.h"

#include "SmoothingKernelSigmoid.h"

#include "Visualizer.h"
#include "PredictiveEnvironment.h"
#include "Maze/Maze.h"
#include "CheeseMaze/CheeseMaze.h"
#include "Planning/LookAheadPolicy.h"
#include "Planning/GoalIteration.h"

#include <float.h>  // for DBL_MAX
#include <vector>
#include <map>
#include <algorithm> // for min, max

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#include "debug.h"

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


#define TO_CONSOLE(x) { ui._wConsoleOutput->appendPlainText(x); }

TestMaze_II::TestMaze_II(QWidget *parent):
    QWidget(parent),
    planner_type(OPTIMAL_LOOK_AHEAD),
    environment(nullptr),
    current_instance(nullptr),
    record(false), plot(false), start_new_episode(false), save_png_on_transition(false),
    png_counter(0),
    random_timer(nullptr), action_timer(nullptr),
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
    random_timer = new QTimer(this);
    connect(random_timer, SIGNAL(timeout()), this, SLOT(random_action()));
    action_timer = new QTimer(this);
    connect(action_timer, SIGNAL(timeout()), this, SLOT(choose_action()));

    // initiate delayed render action
    QTimer::singleShot(0, this, SLOT(render_update()));

    // install event filter
    MoveByKeys *moveByKeys = new MoveByKeys(this);
    ui.graphicsView->installEventFilter(moveByKeys);

    // select environment
    change_environment(make_shared<Maze>(epsilon,"Minimal"));
    // change_environment(make_shared<CheeseMaze>());
}

TestMaze_II::~TestMaze_II() {
    delete random_timer;
    delete action_timer;
    delete current_instance;
    plot_file.close();
}

void TestMaze_II::collect_episode(const int& length) {
    start_new_episode = true;
    for(int idx=0; idx<length; ++idx) {
        action_ptr_t action = action_space->random_element();
        observation_ptr_t observation_to;
        reward_ptr_t reward;
        environment->perform_transition(action,observation_to,reward);
        update_current_instance(action,observation_to,reward);
        add_action_observation_reward_tripel(action,observation_to,reward);
    }
    render_update();
}

void TestMaze_II::update_current_instance(action_ptr_t action, observation_ptr_t observation, reward_ptr_t reward, bool invalidate_search_tree) {
    if(current_instance==nullptr) {
        current_instance = instance_t::create(action,observation,reward);
    } else {
        current_instance = current_instance->append_instance(action,observation,reward);
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
        delete current_instance;
        current_instance = nullptr;
        shared_ptr<PredictiveEnvironment> pred = dynamic_pointer_cast<PredictiveEnvironment>(environment);
        if(pred!=nullptr) {
            const_instanceIt_t env_instance(pred->get_current_instance());
            current_instance = instance_t::create(env_instance->action,env_instance->observation,env_instance->reward,env_instance-1);
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
            policy.reset(new GoalIteration(discount,*pred));
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

void TestMaze_II::random_action() {
    perform_transition(action_space->random_element());
}

void TestMaze_II::choose_action() {
    action_ptr_t action;
    if(policy==nullptr) {
        action = action_space;
        TO_CONSOLE("    No policy available");
    } else {
        action = policy->get_action(current_instance);
    }
    observation_ptr_t observation_to;
    reward_ptr_t reward;
    perform_transition(action,observation_to,reward);
}

void TestMaze_II::process_console_input(QString sequence_input, bool sequence) {

    QString input;

    // deal with sequences
    if(sequence) {
        input = sequence_input;
    } else {
        input = ui._wConsoleInput->text();
        ui._wConsoleInput->setText("");
        TO_CONSOLE(input);
        console_history.push_back(input);
        history_position = console_history.size();
        QTextStream history_file_stream(&history_file);
        history_file_stream << input << "\n";

        // check for sequence
        QStringList command_list = input.split(";");
        if(command_list.length()==0) {
            DEBUG_ERROR("Empty command list");
            return;
        } else if(command_list.length()>1) {
            for(int command_idx=0; command_idx<command_list.length(); ++command_idx) {
                process_console_input(command_list.at(command_idx),true);
            }
            return;
        } else {
            input = command_list.at(0);
        }
    }

    // remove leading and trailing space
    while(input.startsWith(' ')) {
        input.remove(0,1);
    }
    while(input.endsWith(' ')) {
        input.chop(1);
    }

    // help strings
    QString headline_s( "Available commands:\n    COMMAND . . . . ARGUMENTS. . . . . . . . . . . . . . . . . . . . .-> ACTION");

    QString general_s(                     "\n    ---------------------------------General-----------------------------------");
    QString help_s(                          "    help  / h. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> this help");
    QString exit_s(                          "    exit/quit/q. . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> quit application");
    QString set_s(                           "    set/unset. . . . . . . . . <string>. . . . . . . . . . . . . . . . . .-> set/unset options:");
    QString option_1_s(                      "                               record. . . . . . . . . . . . . . . . . . .-> start/stop recording movements");
    QString option_2_s(                      "                               plot. . . . . . . . . . . . . . . . . . . .-> write transitions into data file for plotting");
    QString option_3_s(                      "                               p/planner .<string> . . . . . . . . . . . .-> set planner to use");
    QString option_3a_s(                     "                                          o/optimal. . . . . . . . . . . .-> use optimal predictions (give by maze)");
    QString option_3b_s(                     "                                          s/sparse . . . . . . . . . . . .-> use sparse predictions (given by CRF)");
    QString option_3c_s(                     "                                          u/utree. . . . . . . . . . . . .-> use UTree as predictive model");
    QString option_3d_s(                     "                                          uv/utree-value . . . . . . . . .-> use UTree as value function");
    QString option_3e_s(                     "                                          lq/linear-q. . . . . . . . . . .-> use linear Q-function approximation");
    QString option_3f_s(                     "                                          g/goal . . . . . . . . . . . . .-> use goal-guided value iteration");
    QString option_4_s(                      "                               goal. . . . . . . . . . . . . . . . . . . .-> activate a goal state");
    QString option_5_s(                      "                               prune-tree. . . . . . . . . . . . . . . . .-> prune search tree");
    QString option_6_s(                      "                               png . . . . . . . . . . . . . . . . . . . .-> save a png image of the maze on transition");
    QString option_7_s(                      "                               maze [<string>] . . . . . . . . . . . . . .-> display available maze names [load maze with name <string>]");
    QString option_8_s(                      "                               cheese. . . . . . . . . . . . . . . . . . .-> load cheese maze");
    QString test_s(                          "    test . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> test");

    QString maze_s(                        "\n    -----------------------------------Maze-----------------------------------");
    QString left_s(                          "    left  / l. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> move left");
    QString right_s(                         "    right / r. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> move right");
    QString up_s(                            "    up    / u. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> move up");
    QString down_s(                          "    down  / d. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> move down");
    QString stay_s(                          "    stay  / s. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> stay-action");
    QString move_s(                          "    move . . . . . . . . . . . [<int>|stop]. . . . . . . . . . . . . . . .-> start/stop moving using planner");
    QString random_s(                        "    random . . . . . . . . . . [<int>|stop]. . . . . . . . . . . . . . . .-> start/stop moving randomly");
    QString epsilon_s(                       "    epsilon. . . . . . . . . . [<double>]. . . . . . . . . . . . . . . . .-> get [set] random transition probability");
    QString reward_activation_s(             "    reward-activation / ra . . <int> . . . . . . . . . . . . . . . . . . .-> print mean reward activation probability for length-<int> random walk");
    QString random_distribution_s(           "    random-distribution / rd . <int> . . . . . . . . . . . . . . . . . . .-> run <int> random transitions and display relative counts for all states");

    QString learning_s(                    "\n    ------------------------------Model Learning------------------------------");
    QString episode_s(                       "    episode / e. . . . . . . . [<int>|clear,c] . . . . . . . . . . . . . .-> record length <int> episode or clear data");
    QString learning_crf_s(                  "    === CRF ===");
    QString optimize_crf_s(                  "    crf-optimize / co. . . . . [<int> [<d> <d>]|check, c]. . . . . . . . .-> optimize CRF [max_iterations [ alpha beta ] | check derivatives]");
    QString score_crf_s(                     "    score-crf / scrf . . . . . <int> . . . . . . . . . . . . . . . . . . .-> score candidate features with distance <int> by gradient");
    QString score1D_crf_s(                   "    score1D-crf / s1Dcrf . . . <int> . . . . . . . . . . . . . . . . . . .-> score candidate features with distance <int> by 1D optimization");
    QString add_crf_s(                       "    add-crf. . . . . . . . . . <int> . . . . . . . . . . . . . . . . . . .-> add <int> highest scored candidate features to active (0 for all non-zero scored)");
    QString crf_erase_s(                     "    crf-erase / ce . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> erase features with zero weight");
    QString l1_s(                            "    l1 . . . . . . . . . . . . <double>. . . . . . . . . . . . . . . . . .-> coefficient for L1 regularization");
    QString evaluate_s(                      "    evaluate . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> evaluate features at current point");
//    QString validate_s(                      "    validate / v . . . . . . . {crf,kmdp}[exact|mc <int>]. . . . . . . . .-> validate CRF or k-MDP model using exact (default) or Monte Carlo (with <int> samples) computation of the KL-divergence");
    QString examine_crf_features_s(          "    crf-f. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> print CRF features and weights");
    QString apply_old_crf_features_s(        "    apply-old-features / aof . . . . . . . . . . . . . . . . . . . . . . .-> re-apply the old featues stored at the last erase");
    QString learning_utree_s(                "    === UTree ===");
    QString expand_leaf_nodes_s(             "    expand / ex. . . . . . . . [<int>|<double] . . . . . . . . . . . . . .-> expand <int> leaf nodes / expand leaves until a score of <double> is reached");
    QString print_utree_s(                   "    print-utree. . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> print the current UTree");
    QString print_leaves_s(                  "    print-leaves . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> print leaves of the current UTree");
    QString clear_utree_s(                   "    clear-utree. . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> clear UTree");
    QString utree_value_iteration_s(         "    v-iteration / vi . . . . . [<int>] . . . . . . . . . . . . . . . . . .-> run one [<int>] iteration(s) of Value-Iteration");
    QString utree_expansion_type_s(          "    ex-type / ext. . . . . . . [u(tility)|o(bservation)r(eward)] . . . . .-> get/set expansion type for UTree");
    QString examine_utree_features_s(        "    utree-f. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> print UTree features");
    QString learning_linQ_s(                 "    === Linear-Q ===");
    QString optimize_linQ_ridge_s(           "    lq-optimize-ridge / lqor . [<double>]. . . . . . . . . . . . . . . . .-> optimize Linear-Q (TD Error) [ with L2-regularization coefficient <double> ]");
    QString optimize_linQ_l1_s(              "    lq-optimize-l1 / lqol1 . . [<double> [<int>] | check | c ] . . . . . .-> optimize Linear-Q (TD Error) with L1-regularization coefficient <double> [ and max <int> iterations ] or check derivatives");
    QString optimize_linQ_s(                 "    lq-optimize / lqo. . . . . [<double> [<int>] | check | c ] . . . . . .-> optimize Linear-Q (Bellman Error) with L1-regularization coefficient <double> [ and max <int> iterations ] or check derivatives");
    QString construct_s(                     "    construct / con. . . . . . <int> . . . . . . . . . . . . . . . . . . .-> construct candidate features with distance <int>");
    QString score_lq_s(                      "    score-lq / slq . . . . . . <int> . . . . . . . . . . . . . . . . . . .-> score candidate features with distance <int> by gradient");
    QString add_lq_s(                        "    add-lq . . . . . . . . . . <int> . . . . . . . . . . . . . . . . . . .-> add <int> highest scored candidate features to active (0 for all non-zero scored)");
    QString lq_erase_zero_weight_s(          "    lq-erase / lqe . . . . . . [<double>]. . . . . . . . . . . . . . . . .-> erase features with zero weight [ weight below or equal to <double> ]");
    QString lq_alpha_s(                      "    lq-alpha . . . . . . . . . [<double>]. . . . . . . . . . . . . . . . .-> get [set] alpha for Soft-Max");

    QString planning_s(                    "\n    ---------------------------------Planning----------------------------------");
    QString discount_s(                      "    discount . . . . . . . . . [<double>]. . . . . . . . . . . . . . . . .-> get [set] discount");
    QString print_look_ahead_tree_s(         "    print-tree . . . . . . . . [g|graphic|t|text]. . . . . . . . . . . . .-> print Look-Ahead-Tree statistics [with additional graphical or long text output]");
    QString max_tree_size_s(                 "    max-tree-size. . . . . . . <int> . . . . . . . . . . . . . . . . . . .-> set maximum size of Look-Ahead-Tree (zero for infinite)");

    QString new_s(                         "\n    ---------------------------------New Stuff----------------------------------");
    QString color_states_s(                  "    col-states . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> color states (random)");
    QString fixed_dt_distribution_s(         "    fixed-dt-dist / fdd. . . . <int> . . . . . . . . . . . . . . . . . . .-> show probability for a state to occur <int> steps after current state");
    QString pair_delay_distribution_s(       "    pair-delay-dist / pdd. . . [<int>] . . . . . . . . . . . . . . . . . .-> show temporal delay distribution from current state to goal state (restrict to time window of width <int>");
    QString mediator_probability_s(          "    mediator-probability / mp. <int> . . . . . . . . . . . . . . . . . . .-> show probability for a state to occurr between current state and goal state given a time window of width <int>");

    set_s += "\n" + option_1_s;
    set_s += "\n" + option_2_s;
    set_s += "\n" + option_3_s;
    set_s += "\n" + option_3a_s;
    set_s += "\n" + option_3b_s;
    set_s += "\n" + option_3c_s;
    set_s += "\n" + option_3d_s;
    set_s += "\n" + option_3e_s;
    set_s += "\n" + option_3f_s;
    set_s += "\n" + option_4_s;
    set_s += "\n" + option_5_s;
    set_s += "\n" + option_6_s;
    set_s += "\n" + option_7_s;
    set_s += "\n" + option_8_s;

    QString invalid_args_s( "    invalid arguments" );

    // getting input arguments
    vector<QString> str_args;
    QString tmp_s;
    vector<int> int_args;
    vector<bool> int_args_ok;
    int tmp_i;
    vector<double> double_args;
    vector<bool> double_args_ok;
    double tmp_d;
    for(int arg_idx=0; arg_string(input,arg_idx,tmp_s) && tmp_s!=""; ++arg_idx) {

        str_args.push_back(tmp_s);

        int_args_ok.push_back( arg_int(input,arg_idx,tmp_i) );
        int_args.push_back( tmp_i );

        double_args_ok.push_back( arg_double(input,arg_idx,tmp_d) );
        double_args.push_back( tmp_d );
    }
    int str_args_n = str_args.size();

    // process input
    if(str_args_n>0) {
        if(str_args[0]=="help" || str_args[0]=="h") { // help
            // Headline
            TO_CONSOLE( headline_s );
            // General
            TO_CONSOLE( general_s );
            TO_CONSOLE( help_s );
            TO_CONSOLE( exit_s );
            TO_CONSOLE( set_s );
            TO_CONSOLE( test_s );
            // Maze
            TO_CONSOLE( maze_s );
            TO_CONSOLE( left_s );
            TO_CONSOLE( right_s );
            TO_CONSOLE( up_s );
            TO_CONSOLE( down_s );
            TO_CONSOLE( stay_s );
            TO_CONSOLE( move_s );
            TO_CONSOLE( random_s );
            TO_CONSOLE( epsilon_s );
            TO_CONSOLE( reward_activation_s );
            TO_CONSOLE( random_distribution_s );
            // Learning
            TO_CONSOLE( learning_s );
            TO_CONSOLE( episode_s );
            TO_CONSOLE( learning_crf_s ); // CRF
            TO_CONSOLE( optimize_crf_s );
            TO_CONSOLE( score_crf_s );
            TO_CONSOLE( score1D_crf_s );
            TO_CONSOLE( add_crf_s );
            TO_CONSOLE( crf_erase_s );
            TO_CONSOLE( l1_s );
            TO_CONSOLE( evaluate_s );
//            TO_CONSOLE( validate_s );
            TO_CONSOLE( examine_crf_features_s );
            TO_CONSOLE( apply_old_crf_features_s );
            TO_CONSOLE( learning_utree_s ); // UTree
            TO_CONSOLE( expand_leaf_nodes_s );
            TO_CONSOLE( print_utree_s );
            TO_CONSOLE( print_leaves_s );
            TO_CONSOLE( clear_utree_s );
            TO_CONSOLE( utree_value_iteration_s );
            TO_CONSOLE( utree_expansion_type_s );
            TO_CONSOLE( examine_utree_features_s );
            TO_CONSOLE( learning_linQ_s ); // linear-Q
            TO_CONSOLE( optimize_linQ_ridge_s );
            TO_CONSOLE( optimize_linQ_l1_s );
            TO_CONSOLE( optimize_linQ_s );
            TO_CONSOLE( construct_s );
            TO_CONSOLE( score_lq_s );
            TO_CONSOLE( add_lq_s );
            TO_CONSOLE( lq_erase_zero_weight_s );
            TO_CONSOLE( lq_alpha_s );
            // Planning
            TO_CONSOLE( planning_s );
            TO_CONSOLE( discount_s );
            TO_CONSOLE( print_look_ahead_tree_s );
            TO_CONSOLE( max_tree_size_s );
            // New
            TO_CONSOLE( new_s );
            TO_CONSOLE( color_states_s );
            TO_CONSOLE( fixed_dt_distribution_s );
            TO_CONSOLE( pair_delay_distribution_s );
            TO_CONSOLE( mediator_probability_s );
        } else if(str_args[0]=="left" || str_args[0]=="l") { // left
            if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
                perform_transition(action_ptr_t(new MazeAction("left")));
            } else if(dynamic_pointer_cast<CheeseMaze>(environment)!=nullptr) {
                perform_transition(action_ptr_t(new CheeseMazeAction("west")));
            } else {
                TO_CONSOLE("    Not defined in current environment");
            }
        } else if(str_args[0]=="right" || str_args[0]=="r") { // right
            if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
                perform_transition(action_ptr_t(new MazeAction("right")));
            } else if(dynamic_pointer_cast<CheeseMaze>(environment)!=nullptr) {
                perform_transition(action_ptr_t(new CheeseMazeAction("east")));
            } else {
                TO_CONSOLE("    Not defined in current environment");
            }
        } else if(str_args[0]=="up" || str_args[0]=="u") { // up
            if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
                perform_transition(action_ptr_t(new MazeAction("up")));
            } else if(dynamic_pointer_cast<CheeseMaze>(environment)!=nullptr) {
                perform_transition(action_ptr_t(new CheeseMazeAction("north")));
            } else {
                TO_CONSOLE("    Not defined in current environment");
            }
        } else if(str_args[0]=="down" || str_args[0]=="d") { // down
            if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
                perform_transition(action_ptr_t(new MazeAction("down")));
            } else if(dynamic_pointer_cast<CheeseMaze>(environment)!=nullptr) {
                perform_transition(action_ptr_t(new CheeseMazeAction("south")));
            } else {
                TO_CONSOLE("    Not defined in current environment");
            }
        } else if(str_args[0]=="stay" || str_args[0]=="s") { // stay
            if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
                perform_transition(action_ptr_t(new MazeAction("stay")));
            } else {
                TO_CONSOLE("    Not defined in current environment");
            }
        } else if(str_args[0]=="move") { // start/stop moving
            if(str_args_n==1) {
                choose_action();
            } else if(str_args[1]=="stop") {
                action_timer->stop();
            } else if(int_args_ok[1] && int_args[1]>=0){
                action_timer->stop();
                action_timer->start(int_args[1]);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( move_s );
            }
        } else if(str_args[0]=="random") { // start/stop moving
            if(str_args_n==1) {
                random_action();
            } else if(str_args[1]=="stop") {
                random_timer->stop();
            } else if(int_args_ok[1] && int_args[1]>=0){
                random_timer->stop();
                random_timer->start(int_args[1]);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( random_s );
            }
        } else if(str_args[0]=="episode" || str_args[0]=="e") { // record episode
            if(str_args_n==1) {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( episode_s );
            } else if( str_args[1]=="clear" || str_args[1]=="c" ) {
                clear_data();
            } else if(int_args_ok[1] && int_args[1]>=0){
                collect_episode(int_args[1]);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( episode_s );
            }
        } else if(str_args[0]=="crf-optimize" || str_args[0]=="co") { // optimize CRF
            if(str_args_n==1 || int_args_ok[1] ) {
                if(int_args_ok[1]) {
                    if(str_args_n==4 && double_args_ok[2] && double_args_ok[3] ) {
                        crf->optimize_model(l1_factor, int_args[1], nullptr, true, double_args[2], double_args[3]);
                    } else {
                        crf->optimize_model(l1_factor, int_args[1]);
                    }
                } else {
                    crf->optimize_model(l1_factor, 0);
                }
            } else if(str_args[1]=="check" || str_args[1]=="c") {
                crf->check_derivatives(3,10,1e-6,1e-3);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( optimize_crf_s );
            }
        } else if(str_args[0]=="utree-f") {
            utree->print_features();
        } else if(str_args[0]=="apply-old-features" || str_args[0]=="aof") {
            crf->apply_features();
        } else if(str_args[0]=="lq-optimize-ridge" || str_args[0]=="lqor") { // optimize linear-Q
            if(str_args_n==1 || double_args_ok[1] ) {
                if(double_args_ok[1]) {
                    linQ->set_optimization_type_TD_RIDGE()
                        .set_regularization(double_args[1])
                        .optimize();
                } else {
                    linQ->set_optimization_type_TD_RIDGE()
                        .set_regularization(0)
                        .optimize();
                }
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( optimize_linQ_ridge_s );
            }
        } else if(str_args[0]=="lq-optimize-l1" || str_args[0]=="lqol1") { // optimize linear-Q
            if(str_args_n>1 && double_args_ok[1] ) {
                if(str_args_n>2 && int_args_ok[2] ) {
                    linQ->set_optimization_type_TD_L1()
                        .set_regularization(double_args[1])
                        .set_maximum_iterations(int_args[2])
                        .optimize();
                } else {
                    linQ->set_optimization_type_TD_L1()
                        .set_regularization(double_args[1])
                        .optimize();
                }
            } else if(str_args_n>1 && (str_args[1]=="check" || str_args[1]=="c") ) {
                linQ->check_derivatives(3,10,1e-6,1e-3);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( optimize_linQ_l1_s );
            }
        } else if(str_args[0]=="lq-optimize" || str_args[0]=="lqo") { // optimize linear-Q
            if(str_args_n>1 && double_args_ok[1] ) {
                if(str_args_n>2 && int_args_ok[2] ) {
                    linQ->set_optimization_type_BELLMAN()
                        .set_regularization(double_args[1])
                        .set_maximum_iterations(int_args[2])
                        .optimize();
                } else {
                    linQ->set_optimization_type_BELLMAN()
                        .set_regularization(double_args[1])
                        .set_maximum_iterations(0)
                        .optimize();
                }
            } else if(str_args_n>1 && (str_args[1]=="check" || str_args[1]=="c") ) {
                linQ->check_derivatives(10,10,1e-6,1e-3);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( optimize_linQ_s );
            }
        } else if(str_args[0]=="lq-erase" || str_args[0]=="lqe") {
            if(str_args_n==1 || double_args_ok[1]) {
                if(str_args_n>1) {
                    linQ->erase_features_by_weight(double_args[1]);
                } else {
                    linQ->erase_features_by_weight();
                }
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( lq_erase_zero_weight_s );
            }
        } else if(str_args[0]=="lq-alpha") {
            if(str_args_n==1 || double_args_ok[1]) {
                if(str_args_n>1) {
                    linQ->set_alpha(double_args[1]);
                } else {
                    double alpha = linQ->get_alpha();
                    TO_CONSOLE( QString("    alpha = %1").arg(alpha) );
                }
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( lq_alpha_s );
            }
        } else if(str_args[0]=="epsilon") {
            shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
            if(maze==nullptr) {
                TO_CONSOLE("    Allowed for maze-environments only");
            } else {
                if(str_args_n==1) {
                    if(maze->get_epsilon()!=epsilon) {
                        DEBUG_ERROR("Maze epsion (" << maze->get_epsilon() << ") is different from stored one (" << epsilon << ")");
                    }
                    TO_CONSOLE( QString("    maze epsilon is %1").arg(epsilon) );
                } else if(double_args_ok[1] && double_args[1]>=0 && double_args[1]<=1) {
                    epsilon = double_args[1];
                    maze->set_epsilon(epsilon);
                } else {
                    TO_CONSOLE( invalid_args_s );
                    TO_CONSOLE( epsilon_s );
                }
            }
        } else if(str_args[0]=="reward-activation" || str_args[0]=="ra") {
            shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
            if(maze==nullptr) {
                TO_CONSOLE("    Allowed for maze-environments only");
            } else {
                if(str_args_n==2 && int_args_ok[1]) {
                    maze->print_reward_activation_on_random_walk(int_args[1]);
                    maze->render_update();
                } else {
                    TO_CONSOLE(invalid_args_s);
                    TO_CONSOLE(reward_activation_s);
                }
            }
        } else if(str_args[0]=="random-distribution" || str_args[0]=="rd") { // test
            shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
            if(maze==nullptr) {
                TO_CONSOLE("    Allowed for maze-environments only");
            } else {
                if(str_args_n==2 && int_args_ok[1]) {
                    // initialize state counts to zero
                    map<observation_ptr_t,int> state_counts;
                    for(observation_ptr_t observation : observation_space) {
                        state_counts[observation] = 0;
                    }
                    // get state counts
                    int n = int_args[1];
                    int max_count = 1;
                    for(int idx=0; idx<n; ++idx) {
                        action_ptr_t action = (action_ptr_t)(action_space->random_element());
                        observation_ptr_t observation_to;
                        reward_ptr_t reward;
                        maze->perform_transition(action,observation_to,reward);
                        ++state_counts[observation_to];
                        max_count = max(state_counts[observation_to],max_count);
                    }
                    // transform into colors
                    Maze::color_vector_t cols;
                    for(observation_ptr_t observation : observation_space) {
                        double p = state_counts[observation];
                        DEBUG_OUT(0,"State " << observation << ": p = " << p/max(n,1) );
                        p /= max_count;
                        cols.push_back( std::make_tuple(1,1-p,1-p) );
                    }
                    maze->set_state_colors(cols);
                    maze->render_update();
                } else {
                    TO_CONSOLE( invalid_args_s );
                    TO_CONSOLE( random_distribution_s );
                }
            }
        } else if(str_args[0]=="expand" || str_args[0]=="ex") {
            if(str_args_n==1) {
                double score = utree->expand_leaf_node();
                TO_CONSOLE( QString("    Score was %1").arg(score) );
            } else if(int_args_ok[1] && int_args[1]>=0 ) {
                double score = 0;
                for(int i=0; i<int_args[1]; ++i) {
                    score = utree->expand_leaf_node();
                }
                TO_CONSOLE( QString("    Last score was %1").arg(score) );
            } else if(double_args_ok[1]) {
                while( double_args[1] <= utree->expand_leaf_node(double_args[1]) ) {}
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( expand_leaf_nodes_s );
            }
        } else if(str_args[0]=="print-utree") {
            utree->print_tree();
        } else if(str_args[0]=="print-leaves") {
            utree->print_leaves();
        } else if(str_args[0]=="clear-utree") {
            utree->clear_tree();
        } else if(str_args[0]=="v-iteration" || str_args[0]=="vi") {
            if( str_args_n==1 || (str_args_n>1 && int_args_ok[1] && int_args[1]>0) ) {
                double max_diff;
                int rep = str_args_n==1 ? 1 : int_args[1];
                repeat(rep) {
                    max_diff = utree->value_iteration();
                }
                TO_CONSOLE( QString(    "run %1 iteration(s), last maximum update was %2").arg(rep).arg(max_diff) );
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( utree_value_iteration_s );
            }
        } else if(str_args[0]=="ex-type" || str_args[0]=="ext") {
            if(str_args_n==1) {
                switch(utree->get_expansion_type()) {
                case UTree::UTILITY_EXPANSION:
                    TO_CONSOLE("    expansion type: UTILITY_EXPANSION");
                    break;
                case UTree::OBSERVATION_REWARD_EXPANSION:
                    TO_CONSOLE("    expansion type: OBSERVATION_REWARD_EXPANSION");
                    break;
                default:
                    DEBUG_DEAD_LINE;
                }
            } else if(str_args[1]=="utility" || str_args[1]=="u") {
                utree->set_expansion_type(UTree::UTILITY_EXPANSION);
                TO_CONSOLE("    expansion type: UTILITY_EXPANSION");
            } else if(str_args[1]=="observationreward" ||str_args[1]=="or") {
                utree->set_expansion_type(UTree::OBSERVATION_REWARD_EXPANSION);
                TO_CONSOLE("    expansion type: OBSERVATION_REWARD_EXPANSION");
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( utree_expansion_type_s );
            }
        } else if(str_args[0]=="discount") {
            if(str_args_n==1) {
                TO_CONSOLE( QString("    discount is %1").arg(discount) );
            } else if(double_args_ok[1] && double_args[1]>=0 && double_args[1]<=1) {
                discount = double_args[1];
                utree->set_discount(discount);
                linQ->set_discount(discount);
                shared_ptr<LookAheadPolicy> look_ahead_policy = dynamic_pointer_cast<LookAheadPolicy>(policy);
                if(look_ahead_policy!=nullptr) {
                    look_ahead_policy->set_discount(discount);
                }
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( discount_s );
            }
        } else if(str_args[0]=="evaluate") {
            crf->evaluate_features();
        // } else if(str_args[0]=="validate" || str_args[0]=="v") {
        //     if(str_args_n==1 ) {
        //         TO_CONSOLE( invalid_args_s );
        //         TO_CONSOLE( validate_s );
        //     } else if(str_args[1]=="crf") {
        //         if(str_args[2]=="mc") {
        //             if( str_args_n>3 && int_args_ok[3] && int_args[3]>0 ) {
        //                 probability_t model_l, maze_l;
        //                 probability_t kl = maze.validate_model(
        //                         crf,
        //                         int_args[3],
        //                         &model_l,
        //                         &maze_l
        //                 );
        //                 TO_CONSOLE(QString("    MC KL-Divergence = %1 (%2 samples)").arg(kl).arg(int_args[3]));
        //                 TO_CONSOLE(QString("    Mean Likelihood: model = %1, maze = %2").arg(model_l).arg(maze_l));
        //             } else {
        //                 TO_CONSOLE( "    Please specify a valid sample size" );
        //             }
        //         } else {
        //             TO_CONSOLE( invalid_args_s );
        //             TO_CONSOLE( validate_s );
        //         }
        //     } else if(str_args[1]=="kmdp") {
        //         TO_CONSOLE( "    Sorry, not implemented" );
        //     } else {
        //         TO_CONSOLE( invalid_args_s );
        //         TO_CONSOLE( validate_s );
        //     }
        } else if(str_args[0]=="crf-f") {
            crf->print_all_features();
        } else if(str_args[0]=="l1") {
            if(str_args_n==1) {
                TO_CONSOLE(QString("    %1").arg(l1_factor));
            } else if(double_args_ok[1] && double_args[1]>=0) {
                l1_factor = double_args[1];
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( l1_s );
            }
        } else if(str_args[0]=="score-crf" || str_args[0]=="scrf") {
            if(str_args_n==1) {
                TO_CONSOLE(score_crf_s);
            } else if(int_args_ok[1] && int_args[1]>=0 ) {
                crf->construct_candidate_features(int_args[1]);
                crf->score_candidates_by_gradient();
                crf->print_scores();
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( score_crf_s );
            }
        } else if(str_args[0]=="score1D-crf" || str_args[0]=="s1Dcrf") {
            if(str_args_n==1) {
                TO_CONSOLE(score1D_crf_s);
            } else if(int_args_ok[1] && int_args[1]>=0 ) {
                crf->construct_candidate_features(int_args[1]);
                crf->score_candidates_by_1D_optimization();
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( score1D_crf_s );
            }
        } else if(str_args[0]=="add-crf") {
            if(str_args_n==1) {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( add_crf_s );
            } else if(int_args_ok[1] && int_args[1]>=0 ) {
                crf->add_candidate_features_to_active(int_args[1]);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( add_crf_s );
            }
        } else if(str_args[0]=="score-lq" || str_args[0]=="slq") {
            if(str_args_n==1) {
                TO_CONSOLE(score_lq_s);
            } else if(int_args_ok[1] && int_args[1]>=0 ) {
                linQ->construct_candidate_features(int_args[1]);
                linQ->score_candidates_by_gradient();
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( score_lq_s );
            }
        } else if(str_args[0]=="add-lq") {
            if(str_args_n==1) {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( add_lq_s );
            } else if(int_args_ok[1] && int_args[1]>=0 ) {
                linQ->add_candidates_by_score(int_args[1]);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( add_lq_s );
            }
        } else if(str_args[0]=="crf-erase" || str_args[0]=="ce") {
            crf->erase_zero_features();
        } else if(str_args[0]=="exit" || str_args[0]=="quit" || str_args[0]=="q") { // quit application
            QApplication::quit();
        } else if(str_args[0]=="print-tree") { // print tree
            bool graphic = false, text = false;
            if(str_args_n>1) {
                if(str_args[1]=="g" || str_args[1]=="graphic") {
                    graphic = true;
                } else if(str_args[1]=="t" || str_args[1]=="text") {
                    text = true;
                } else {
                    TO_CONSOLE( invalid_args_s );
                    TO_CONSOLE( print_look_ahead_tree_s );
                }
            }
            shared_ptr<LookAheadPolicy> look_ahead_policy = dynamic_pointer_cast<LookAheadPolicy>(policy);
            if(look_ahead_policy!=nullptr) {
                const LookAheadSearch & s = look_ahead_policy->get_look_ahead_search();
                s.print_tree_statistics();
                s.print_tree(text, graphic);
            }
        } else if(str_args[0]=="max-tree-size") { // set tree size
            if(str_args_n==1) {
                TO_CONSOLE( QString( "    max tree size is %1" ).arg(max_tree_size) );
            } else if(int_args_ok[1] && int_args[1]>=0) {
                max_tree_size = int_args[1];
                shared_ptr<LookAheadPolicy> look_ahead_policy = dynamic_pointer_cast<LookAheadPolicy>(policy);
                if(look_ahead_policy!=nullptr) {
                    look_ahead_policy->set_max_tree_size(max_tree_size);
                }
            } else {
                TO_CONSOLE( "    Please specify a valid tree size" );
            }
        } else if(str_args[0]=="set" || str_args[0]=="unset") { // set option
            if(str_args_n==1) {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( set_s );
            } else if(str_args[1]=="record") {
                if(str_args[0]=="set") {
                    record = true;
                    start_new_episode = true;
                    TO_CONSOLE( "    record on" );
                } else if(str_args[0]=="unset") {
                    record = false;
                    TO_CONSOLE( "    record off" );
                } else {
                    DEBUG_DEAD_LINE;
                }
            } else if(str_args[1]=="plot") {
                plot = str_args[0]=="set";
                if(plot) {
                    // open plot file
                    plot_file.open("plot_file.txt");
                    plot_file << "# action observation reward" << endl;
                    TO_CONSOLE( "    plot on" );
                } else {
                    // close plot file
                    plot_file.close();
                    TO_CONSOLE( "    plot off" );
                }
            } else if( (str_args[1]=="p" || str_args[1]=="planner") ) {
                if(str_args_n>2) {
                    // set planner
                    if(str_args[0]=="unset") {
                        TO_CONSOLE( "    set different planner to unset current" );
                    } else if(str_args[2]=="optimal" || str_args[2]=="o") {
                        planner_type = OPTIMAL_LOOK_AHEAD;
                        TO_CONSOLE( "    using optimal planner" );
                    } else if(str_args[2]=="sparse" || str_args[2]=="s") {
                        planner_type = SPARSE_LOOK_AHEAD;
                        TO_CONSOLE( "    using sparse planner" );
                    } else if(str_args[2]=="utree" || str_args[2]=="u") {
                        planner_type = UTREE_LOOK_AHEAD;
                        TO_CONSOLE( "    using UTree planner" );
                    } else if(str_args[2]=="uv" || str_args[2]=="utree-value") {
                        planner_type = UTREE_VALUE;
                        TO_CONSOLE( "    using UTree-value for action selection" );
                    } else if(str_args[2]=="lq" || str_args[2]=="linear-q") {
                        planner_type = LINEAR_Q_VALUE;
                        TO_CONSOLE( "    using linear Q-approximation for action selection" );
                    } else if(str_args[2]=="g" || str_args[2]=="goal") {
                        planner_type = GOAL_ITERATION;
                        TO_CONSOLE( "    using goal-guided value iteration" );
                    } else {
                        TO_CONSOLE( "    unknown planner" );
                    }
                    set_policy();
                } else {
                    TO_CONSOLE( "    please supply a planner to use" );
                }
            } else if(str_args[1]=="maze") {
                bool print_list = false;
                if(str_args[0]=="unset") {
                    TO_CONSOLE( "    set different environment to unset current" );
                } else {
                    if(str_args_n>2) {
                        shared_ptr<Maze> maze(new Maze(epsilon));
                        bool success = maze->set_maze(str_args[2]);
                        if(!success) {
                            TO_CONSOLE("    No maze named '"+str_args[2]+"'");
                            print_list = true;
                        }
                        change_environment(maze);
                    } else {
                        print_list = true;
                    }
                }
                if(print_list) {
                    TO_CONSOLE( "    Available mazes:" );
                    for(QString name : Maze::get_maze_list()) {
                        TO_CONSOLE("        "+name);
                    }
                }
            } else if(str_args[1]=="cheese") {
                if(str_args[0]=="unset") {
                    TO_CONSOLE( "    set different environment to unset current" );
                } else {
                    change_environment(make_shared<CheeseMaze>());
                }
            } else if(str_args[1]=="goal") {
                shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
                if(maze==nullptr) {
                    TO_CONSOLE("    Allowed for maze-environments only");
                } else {
                    if(str_args[0]=="set") {
                        if(goal_activated) {
                            TO_CONSOLE( "    goal already active" );
                        } else {
                            TO_CONSOLE( "    goal active" );
                        }
                        goal_activated = true;
                        goal_state = current_instance->observation;
                        // color maze
                        Maze::color_vector_t cols;
                        for(auto observation : observation_space) {
                            if(observation==goal_state) {
                                cols.push_back( Maze::color_t(0,1,0) );
                            } else {
                                cols.push_back( Maze::color_t(1,1,1) );
                            }
                        }
                        maze->set_state_colors(cols);
                        maze->render_update();
                        // set goal in GoalIteration policy
                        shared_ptr<GoalIteration> goal_iteration = dynamic_pointer_cast<GoalIteration>(policy);
                        if(goal_iteration!=nullptr) {
                            goal_iteration->set_goal(goal_state);
                        }
                    } else {
                        if(!goal_activated) {
                            TO_CONSOLE( "    goal already inactive" );
                        } else {
                            goal_activated = false;
                            TO_CONSOLE( "    goal inactive" );
                        }
                    }
                }
            } else if(str_args[1]=="prune-tree") {
                if(str_args[0]=="set") {
                    prune_search_tree=true;
                    TO_CONSOLE( "    prune search tree" );
                } else {
                    prune_search_tree=false;
                    TO_CONSOLE( "    don't prune search tree" );
                }
                shared_ptr<LookAheadPolicy> look_ahead_policy = dynamic_pointer_cast<LookAheadPolicy>(policy);
                if(look_ahead_policy!=nullptr) {
                    look_ahead_policy->set_pruning(prune_search_tree);
                    look_ahead_policy->invalidate_search_tree();
                }
            } else if(str_args[1]=="png") {
                if(str_args[0]=="set") {
                    save_png_on_transition=true;
                    TO_CONSOLE( "    save png on transition" );
                } else {
                    save_png_on_transition=false;
                    TO_CONSOLE( "    don't save png on transition" );
                }
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( set_s );
            }
        } else if(str_args[0]=="construct" || str_args[0]=="con") {
            if(str_args_n==1) {
                TO_CONSOLE(construct_s);
            } else if(int_args_ok[1] && int_args[1]>=0 ) {
                linQ->add_all_candidates(int_args[1]);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( construct_s );
            }
        } else if(str_args[0]=="test") { // test
            // if(str_args_n==2 && int_args_ok[1]) {
            //     maze.print_reward_activation_on_random_walk(int_args[1]);
            //     maze.render_update();
            // }

            // crf->print_all_features();
            // utree->print_features();

            probability_t like = crf->cross_validation(10);
            TO_CONSOLE( QString("    Mean Data Likelihood: %1").arg(like) );

            //TO_CONSOLE( "    currently no test function implemented" );
        } else if(str_args[0]=="col-states") { // color states
            shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
            if(maze==nullptr) {
                TO_CONSOLE("    Allowed for maze-environments only");
            } else {
                Maze::color_vector_t cols;
                for(observation_ptr_t observation : observation_space) {
                    cols.push_back( std::make_tuple(drand48(),drand48(),drand48()) );
                }
                maze->set_state_colors(cols);
                maze->render_update();
            }
        } else if(str_args[0]=="fixed-dt-dist" || str_args[0]=="fdd") { // show delay probability
            shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
            if(maze==nullptr) {
                TO_CONSOLE("    Allowed for maze-environments only");
            } else {
                if(str_args_n!=2 || !int_args_ok[1]) {
                    TO_CONSOLE( invalid_args_s );
                    TO_CONSOLE( fixed_dt_distribution_s );
                } else {
                    // get probabilites for all states
                    observation_ptr_t s1 = current_instance->observation;
                    idx_t delay = int_args[1];
                    vector<probability_t> probs = delay_dist.get_fixed_delay_probability_distribution(s1,delay);

                    // get maximum probability for rescaling and goal state idx
                    double max_prob = -DBL_MAX;
                    idx_t goal_idx = 0, state_idx = 0;
                    for(observation_ptr_t observation : observation_space) {
                        if(probs[state_idx]>max_prob) {
                            max_prob = probs[state_idx];
                        }
                        if(observation==goal_state) {
                            goal_idx=state_idx;
                        }
                    }

                    // rescale and define colors
                    Maze::color_vector_t cols;
                    for( double p : probs ) {
                        if(max_prob!=0) {
                            cols.push_back( std::make_tuple(1,1-p/max_prob,1-p/max_prob) );
                        } else {
                            cols.push_back( std::make_tuple(1,1,1) );
                        }
                    }
                    if(goal_activated) {
                        cols[goal_idx]=std::make_tuple(0,1,0);
                    }

                    // render
                    maze->set_state_colors(cols);
                    maze->render_update();
                }
            }
        } else if(str_args[0]=="pair-delay-dist" || str_args[0]=="pdd") { // show delay distribution
            if(str_args_n>1 && !int_args_ok[1]) {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( pair_delay_distribution_s );
            } else {
                if(!goal_activated) {
                    TO_CONSOLE( "    Goal state must be activated to calculate delay distribution" );
                } else {

                    // get distribution
                    int time_window = -1;
                    if(int_args_ok[1]) {
                        time_window = int_args[1];
                    }
                    DelayDistribution::pair_dist_map_t dist_map;
                    delay_dist.get_pairwise_delay_distribution(dist_map,time_window);

                    // apply to current states
                    observation_ptr_t s1 = current_instance->observation;
                    observation_ptr_t s2 = goal_state;
                    vector<double> forward;
                    vector<double> backward;
                    for( auto el : dist_map ) {
                        observation_ptr_t s1_map = get<0>(el.first);
                        observation_ptr_t s2_map = get<1>(el.first);
                        int dt = get<2>(el.first);
                        if(dt==0) {
                            continue;
                        }
                        if(s1_map==s1 && s2_map==s2) {
                            if(dt>(int)forward.size()-1) {
                                forward.resize(dt+1,0);
                            }
                            forward[dt]=el.second;
                            DEBUG_OUT(3,s1_map << " --> " << s2_map << " (" << dt << ") : " << el.second);
                        }
                        if(s1_map==s2 && s2_map==s1) {
                             if(dt>(int)backward.size()-1) {
                                backward.resize(dt+1,0);
                            }
                            backward[dt]=el.second;
                            DEBUG_OUT(3,s1_map << " --> " << s1_map << " (" << dt << ") : " << el.second);
                        }
                    }

                    // prepare for plotting
                    size_t back_size = backward.size();
                    size_t forw_size = forward.size();
                    size_t point_n = forw_size + back_size - 1;
                    QVector<double> x(point_n), y(point_n);
                    probability_t max_y = -DBL_MAX;
                    for (int i=0; i<(int)point_n; ++i) {
                        x[i] = i - back_size + 1;
                        y[i] = i<(int)back_size ? backward[back_size-i-1] : forward[i-back_size];
                        if(y[i]>max_y) {
                            max_y = y[i];
                        }
                    }
                    // assign data to graph
                    plotter->graph(0)->setData(x, y);
                    // give the axes some labels:
                    plotter->xAxis->setLabel("delay/steps");
                    plotter->yAxis->setLabel("probability");
                    // set axes ranges, so we see all data:
                    plotter->xAxis->setRange(-(int)backward.size(),forward.size());
                    plotter->yAxis->setRange(0,max_y);
                    DEBUG_OUT(3,"Ranges: [" << -(int)backward.size() << ":" << forward.size() << "][" << 0 << ":" << max_y << "]");
                    plotter->replot();
                }
            }
        } else if(str_args[0]=="mediator-probability" || str_args[0]=="mp") { // show mediator probability
            shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
            if(maze==nullptr) {
                TO_CONSOLE("    Allowed for maze-environments only");
            } else {
                if(str_args_n==2 && int_args_ok[1]) {
                    if(!goal_activated) {
                        TO_CONSOLE( "    Goal state must be activated to calculate mediator probabilities" );
                    } else {
                        // get probabilites for all states
                        DEBUG_OUT(2,"Calculating mediator distribution...");
                        observation_ptr_t s1 = current_instance->observation;
                        observation_ptr_t s3 = goal_state;
                        vector<double> probs;
                        double max_prob = -DBL_MAX;
                        idx_t goal_idx = 0, state_idx = 0;
                        for(observation_ptr_t observation : observation_space) {
                            probability_t prob = delay_dist.get_mediator_probability(s1,observation,s3,int_args[1]);
                            probs.push_back(prob);
                            if(prob>max_prob) {
                                max_prob = prob;
                            }
                            if(observation==goal_state) {
                                goal_idx=state_idx;
                            }
                            DEBUG_OUT(3,"    " << s1 << " --> " << observation << " --> " << s3 << " : " << prob);
                            ++state_idx;
                        }
                        DEBUG_OUT(2,"DONE");
                        // rescale and define colors
                        Maze::color_vector_t cols;
                        for( double prob : probs ) {
                            if(max_prob!=0) {
                                cols.push_back( std::make_tuple(1,1-prob/max_prob,1-prob/max_prob) );
                            } else {
                                cols.push_back( std::make_tuple(1,1,1) );
                            }
                        }
                        cols[goal_idx]=std::make_tuple(0,1,0);
                        // render
                        maze->set_state_colors(cols);
                        maze->render_update();
                    }
                } else {
                    TO_CONSOLE( invalid_args_s );
                    TO_CONSOLE( mediator_probability_s );
                }
            }
        } else {
            TO_CONSOLE("    unknown command");
        }
    }
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
            maze->process_console_input(QString('l'), true);
            return true;
	case Qt::Key_Right: // right
            maze->process_console_input(QString('r'), true);
            return true;
	case Qt::Key_Up: // up
            maze->process_console_input(QString('u'), true);
            return true;
	case Qt::Key_Down: // down
            maze->process_console_input(QString('d'), true);
            return true;
	case Qt::Key_Space: // space
            maze->process_console_input(QString('s'), true);
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
