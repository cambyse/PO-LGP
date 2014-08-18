#include "testmaze_ii.h"

#include "Config.h"
#include "util/util.h"
#include "util/QtUtil.h"

#include "SmoothingKernelSigmoid.h"

#include "Visualizer.h"
#include "PredictiveEnvironment.h"
#include "Maze/Maze.h"
#include "CheeseMaze/CheeseMaze.h"
#include "ButtonWorld/ButtonWorld.h"
#include "Planning/LookAheadPolicy.h"
#include "Planning/RandomPolicy.h"
#include "Planning/GoalIteration.h"
#include "Representation/DoublyLinkedInstance.h"

#include <float.h>  // for DBL_MAX
#include <vector>
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
using std::pair;
using std::tuple;
using std::make_tuple;

using util::arg_int;
using util::arg_double;
using util::arg_string;
using util::INVALID;

static TestMaze_II::instance_ptr_t learning_episode_begin;
static TestMaze_II::instance_ptr_t learning_episode_end;

TestMaze_II::TestMaze_II(QWidget *parent):
    QWidget(parent),
    planner_type(RANDOM),
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
    utree(new UTree(discount)),
    N_plus_TEL(new ConjunctiveAdjacency()),
    tel(new TemporallyExtendedLinearQ(N_plus_TEL,discount)),
    N_plus_TEM(new ConjunctiveAdjacency()),
    tem(new TemporallyExtendedModel(N_plus_TEM)),
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

    // print console welcome message
    to_console("----Please enter your commands (type 'help' for an overview)----");

    // initialize timers
    action_timer = new QTimer(this);
    connect(action_timer, SIGNAL(timeout()), this, SLOT(choose_action()));

    // initiate delayed render action
    QTimer::singleShot(0, this, SLOT(render_update()));

    // install event filter
    MoveByKeys *moveByKeys = new MoveByKeys(this);
    ui.graphicsView->installEventFilter(moveByKeys);

    // preliminarily set an environment
    change_environment(make_shared<Maze>(epsilon,"Default"));
    // set some properties of N+ (TEL)
    N_plus_TEL->set_horizon_extension(2);
    N_plus_TEL->set_max_horizon(-1);
    N_plus_TEL->set_min_horizon(-2);
    N_plus_TEL->set_combine_features(false);
    N_plus_TEL->set_t_zero_features(ConjunctiveAdjacency::T_ZERO_FEATURES::ACTION);
    // set some properties of N+ (TEM)
    N_plus_TEM->set_horizon_extension(2);
    N_plus_TEM->set_max_horizon(0);
    N_plus_TEM->set_min_horizon(-2);
    N_plus_TEM->set_combine_features(false);
    N_plus_TEM->set_t_zero_features(ConjunctiveAdjacency::T_ZERO_FEATURES::OBSERVATION_REWARD);

    // select final environmen
    // change_environment(make_shared<Maze>(epsilon,"Markov"));
    // change_environment(make_shared<CheeseMaze>());
    // change_environment(make_shared<ButtonWorld>(5));

    // set l1 factor for tem
    tem->set_l1_factor(l1_factor);
    tel->set_l1_factor(l1_factor);

    // set all commands
    initialize_commands();
}

void TestMaze_II::initialize_commands() {
    typedef Commander::ReturnType ret_t;
    pair<double,QString> top_todo(1000,"Not Ported Yet =============================================");
    {
        pair<double,QString> top_general(1,"General ================================================");
        command_center.add_command(top_general,{"help","h"},[this]()->ret_t{
                to_console("");
                for(QString s : command_center.get_help()) {
                    to_console(s,NORMAL_STYLE,4);
                }
                return {true,""};
            },"Print help");
        command_center.add_command(top_general,{"help","h"},[this](QString filter)->ret_t{
                to_console("");
                for(QString s : command_center.get_help(filter)) {
                    to_console(s,NORMAL_STYLE,4);
                }
                return {true,""};
            },"Print help for commands containing <QString>");
        command_center.add_command(top_general,{"exit","quit","q"}, [this]()->ret_t{
                QApplication::quit();
                return {true,""};
            }, "quit application");
        command_center.add_command(top_general,{"set record"}, [this]()->ret_t{
                record = true;
                start_new_episode = true;
                return {true,"record on" };
            }, "start recording movements");
        command_center.add_command(top_general,{"unset record"}, [this]()->ret_t{
                record = false;
                return {true,"record off" };
            }, "stop recording movements");
        command_center.add_command(top_general,{"set plot"}, [this]()->ret_t{
                plot = true;
                plot_file.open("plot_file.txt");
                plot_file << "# action observation reward" << endl;
                return {true,"plot on" };
            }, "start writing transitions into data file for plotting");
        command_center.add_command(top_general,{"unset plot"}, [this]()->ret_t{
                plot = false;
                plot_file.close();
                return {true,"plot off" };
            }, "stop writing transitions into data file for plotting");
        command_center.add_command(top_todo,{"set png"}, [this]()->ret_t{
                return {true,"...to be ported"};
            }, "save a png image of the maze on transition");
        command_center.add_command(top_todo,{"unset png"}, [this]()->ret_t{
                return {true,"...to be ported"};
            }, "don't save a png image of the maze on transition");
        command_center.add_command(top_general,{"set color"}, [this]()->ret_t{
                color_maze = true;
                return {true,"color maze"};
            }, "color maze (if applicable)");
        command_center.add_command(top_general,{"unset color"}, [this]()->ret_t{
                color_maze = false;
                shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
                if(maze!=nullptr) {
                    maze->set_state_colors();
                    maze->render_update();
                }
                return {true,"don't color maze" };
            }, "don't color maze");
    }
    {
        pair<double,QString> top_maze(2,"Maze ===================================================");
        command_center.add_command(top_maze,{"set maze"}, [this]()->ret_t{
                to_console("Available mazes:",NORMAL_STYLE,4);
                for(QString name : Maze::get_maze_list()) {
                    to_console(name,NORMAL_STYLE,8);
                }
                return {true,""};
            }, "display available maze names");
        command_center.add_command(top_maze,{"set button world", "set bw"}, [this](int size)->ret_t{
                change_environment(make_shared<ButtonWorld>(size));
                return {true,"set button world"};
            }, "set button world with <int> buttons");

        command_center.add_command(top_maze,{"set button world", "set bw"}, [this](int size, double alpha)->ret_t{
                if(alpha<=0) {
                    return {false,"alpha <double> must be greater than zero"};
                } else {
                    change_environment(make_shared<ButtonWorld>(size, alpha));
                    return {true,"set button world"};
                }
            }, "set button world with <int> buttons and probabilites drawn independently from a Beta with a=b=<double>");
        command_center.add_command(top_maze,{"set maze"}, [this](QString name)->ret_t{
                shared_ptr<Maze> maze(new Maze(epsilon));
                bool success = maze->set_maze(name);
                change_environment(maze);
                if(!success) {
                    command_center.execute("set maze");
                    return {false,"no maze named '"+name+"'"};
                } else {
                    return {true,"set maze '"+name+"'"};
                }
            }, "load maze with name <string>");
        command_center.add_command(top_maze,{"set cheese"}, [this]()->ret_t{
                change_environment(make_shared<CheeseMaze>());
                return {true,"set cheese maze"};
            }, "load cheese maze");
        command_center.add_command(top_maze,{"move left","l"}, [this]()->ret_t{
                if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
                    perform_transition(action_ptr_t(new MazeAction("left")));
                } else if(dynamic_pointer_cast<CheeseMaze>(environment)!=nullptr) {
                    perform_transition(action_ptr_t(new CheeseMazeAction("west")));
                } else {
                    return {false,"Not defined in current environment"};
                }
                return {true,"moved left"};
            }, "move left");
        command_center.add_command(top_maze,{"move right","r"}, [this]()->ret_t{
                if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
                    perform_transition(action_ptr_t(new MazeAction("right")));
                } else if(dynamic_pointer_cast<CheeseMaze>(environment)!=nullptr) {
                    perform_transition(action_ptr_t(new CheeseMazeAction("east")));
                } else {
                    return {false,"Not defined in current environment"};
                }
                return {true,"moved right"};
            }, "move right");
        command_center.add_command(top_maze,{"move up","u"}, [this]()->ret_t{
                if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
                    perform_transition(action_ptr_t(new MazeAction("up")));
                } else if(dynamic_pointer_cast<CheeseMaze>(environment)!=nullptr) {
                    perform_transition(action_ptr_t(new CheeseMazeAction("north")));
                } else {
                    return {false,"Not defined in current environment"};
                }
                return {true,"moved up"};
            }, "move up");
        command_center.add_command(top_maze,{"move down","d"}, [this]()->ret_t{
                if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
                    perform_transition(action_ptr_t(new MazeAction("down")));
                } else if(dynamic_pointer_cast<CheeseMaze>(environment)!=nullptr) {
                    perform_transition(action_ptr_t(new CheeseMazeAction("south")));
                } else {
                    return {false,"Not defined in current environment"};
                }
                return {true,"moved down"};
            }, "move down");
        command_center.add_command(top_maze,{"move stay","s"}, [this]()->ret_t{
                if(dynamic_pointer_cast<Maze>(environment)!=nullptr) {
                    perform_transition(action_ptr_t(new MazeAction("stay")));
                } else {
                    return {false,"Not defined in current environment"};
                }
                return {true,"stayed"};
            }, "stay-action");
        command_center.add_command(top_maze,{"move"}, [this]()->ret_t{
                choose_action();
                return {true,"move one step"};
            }, "move one step");
        command_center.add_command(top_maze,{"move"}, [this](int n)->ret_t{
                action_timer->stop();
                action_timer->start(n);
                return {true,QString("move every %1 ms").arg(n)};
            }, "start moving every <int> milliseconds");
        command_center.add_command(top_maze,{"move stop"}, [this]()->ret_t{
                action_timer->stop();
                return {true,"stop moving"};
            }, "stop moving");
        command_center.add_command(top_maze,{"epsilon"}, [this]()->ret_t{
                shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
                if(maze==nullptr) {
                    return {false,"Allowed for maze-environments only"};
                } else {
                    if(maze->get_epsilon()!=epsilon) {
                        return {false,QString("Maze epsion (%1) is different from stored one (%2)").arg(maze->get_epsilon()).arg(epsilon)};
                    }
                    return {true,QString("epsilon = %1").arg(epsilon)};
                }
            }, "get random transition probability");
        command_center.add_command(top_maze,{"epsilon"}, [this](double d)->ret_t{
                shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
                if(maze==nullptr) {
                    return {false,"Allowed for maze-environments only"};
                } else {
                    if(0<=d && d<=1) {
                        epsilon = d;
                        maze->set_epsilon(epsilon);
                    } else {
                        return {false,"epsilon must be in [0,1]"};
                    }
                    return {true,""};
                }
            }, "set random transition probability");
        command_center.add_command(top_maze,{"reward activation"}, [this](int n)->ret_t{
                shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
                if(maze==nullptr) {
                    return {false,"Allowed for maze-environments only"};
                } else {
                    maze->print_reward_activation_on_random_walk(n);
                    maze->render_update();
                }
                return {true,""};
            }, "print mean reward activation probability for length-<int> random walk");
        command_center.add_command(top_maze,{"random distribution"}, [this](int n)->ret_t{
                shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
                if(maze==nullptr) {
                    return {false,"Allowed for maze-environments only"};
                } else {
                    // initialize state counts to zero
                    map<observation_ptr_t,int> state_counts;
                    for(observation_ptr_t observation : observation_space) {
                        state_counts[observation] = 0;
                    }
                    // get state counts
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
                }
                return {true,""};
            }, "run <int> random transitions and display relative counts for all states");
    }
    {
        pair<double,QString> top_model_learn(3, "Model Learning =========================================");
        command_center.add_command(top_model_learn,{"l1"}, [this]()->ret_t{
                return {true,QString("L1 coefficient is %1").arg(l1_factor)};
            }, "print current coefficient for L1 regularization");
        command_center.add_command(top_model_learn,{"l1"}, [this](double d)->ret_t{
                if(d>=0) {
                    l1_factor = d;
                    tem->set_l1_factor(l1_factor);
                    tel->set_l1_factor(l1_factor);
                    return {true,QString("set L1 coefficient to %1").arg(l1_factor)};
                } else {
                    return {false,"L1 coefficient must be non-negative"};
                }
            }, "set coefficient for L1 regularization");
        command_center.add_command(top_model_learn,{"episode","e"}, [this](int n)->ret_t{
                if(n<0) {
                    return {false,"Expecting positive integer"};
                } else {
                    collect_episode(n);
                }
                return {true,QString("collected episode of length %1").arg(n)};
            }, "record length <int> episode");
        command_center.add_command(top_model_learn,{"episode clear","ec"}, [this]()->ret_t{
                clear_data();
                return {true,"cleared episode data"};
            }, "clear episode data");
        command_center.add_command(top_model_learn,{"validate tem random"}, [this](int n)->ret_t{
                if(n<=0) {
                    return {false,"expecting positive integer"};
                } else {
                    return {true,QString("random length-%1 episode has mean liklihood of %2").arg(n).arg(validate_predictor_on_random_episode(n,*tem))};
                }
            }, "validate TEM on length <int> random episode");
        command_center.add_command(top_model_learn,{"validate utree random"}, [this](int n)->ret_t{
                if(n<=0) {
                    return {false,"expecting positive integer"};
                } else {
                    return {true,QString("random length-%1 episode has mean liklihood of %2").arg(n).arg(validate_predictor_on_random_episode(n,*utree))};
                }
            }, "validate UTree on length <int> random episode");
        command_center.add_command(top_model_learn,{"validate tem episode"}, [this]()->ret_t{
                return {true,QString("model assignes a mean liklihood of %1 to training episode").arg(validate_predictor_on_training_episode(*tem))};
            }, "validate TEM on training episode");
        command_center.add_command(top_model_learn,{"validate utree episode"}, [this]()->ret_t{
                return {true,QString("model assignes a mean liklihood of %1 to training episode").arg(validate_predictor_on_training_episode(*utree))};
            }, "validate UTree on training episode");
    }
    {
        pair<double,QString> top_model_learn_utree(3.2,"UTree --------------------------------------------------");
        command_center.add_command(top_model_learn_utree,{"expand","ex"}, [this]()->ret_t{
                double score = utree->expand_leaf_node();
                return {true, QString("score was %1").arg(score) };
            }, "expand one leaf node");
        command_center.add_command(top_model_learn_utree,{"expand","ex"}, [this](int n)->ret_t{
                if(n<=0 ) {
                    return {false,"Expecting a positive integer"};
                } else {
                    double score = 0;
                    for(int i=0; i<n; ++i) {
                        score = utree->expand_leaf_node();
                    }
                    return {true,QString("last score was %1").arg(score) };
                }
            }, "expand <int> leaf nodes");
        command_center.add_command(top_model_learn_utree,{"expand","ex"}, [this](double d)->ret_t{
                double score = DBL_MAX;
                while(score>d) {
                    score=utree->expand_leaf_node(d);
                }
                return {true,QString("last score was %1").arg(score) };
            }, "expand leaves until a score of <double> is reached");
        command_center.add_command(top_model_learn_utree,{"print utree"}, [this]()->ret_t{
                utree->print_tree();
                return {true,""};
            }, "print the current UTree");
        command_center.add_command(top_model_learn_utree,{"print leaves"}, [this]()->ret_t{
                utree->print_leaves();
                return {true,""};
            }, "print leaves of the current UTree");
        command_center.add_command(top_model_learn_utree,{"clear utree"}, [this]()->ret_t{
                utree->clear_tree();
                return {true,""};
            }, "clear UTree");
        command_center.add_command(top_model_learn_utree,{"v-iteration","vi"}, [this]()->ret_t{
                double max_diff = utree->value_iteration();
                return {true,QString("ran one iteration, maximum update was %1").arg(max_diff) };
            }, "run one step of Value-Iteration");
        command_center.add_command(top_model_learn_utree,{"v-iteration","vi"}, [this](int n)->ret_t{
                if(n<=0) {
                    return {false,"Expecting a positive integer"};
                } else {
                    double max_diff;
                    repeat(n) {
                        max_diff = utree->value_iteration();
                    }
                    return {true,QString("ran %1 iteration(s), last maximum update was %2").arg(n).arg(max_diff) };
                }
            }, "run one <int> steps of Value-Iteration");
        command_center.add_command(top_model_learn_utree,{"ex-type","ext"}, [this]()->ret_t{
                switch(utree->get_expansion_type()) {
                case UTree::UTILITY_EXPANSION:
                    return {true,"expansion type: UTILITY_EXPANSION"};
                    break;
                case UTree::OBSERVATION_REWARD_EXPANSION:
                    return {true,"expansion type: OBSERVATION_REWARD_EXPANSION"};
                    break;
                default:
                    return {false,"unkown error"};
                    DEBUG_DEAD_LINE;
                }
            }, "get expansion type for UTree");
        command_center.add_command(top_model_learn_utree,{"ex-type-utility","ext u"}, [this]()->ret_t{
                utree->set_expansion_type(UTree::UTILITY_EXPANSION);
                return {true,"set expansion type to UTILITY_EXPANSION"};
            }, "set expansion type for UTree");
        command_center.add_command(top_model_learn_utree,{"ex-type-observation-reward","ext or"}, [this]()->ret_t{
                utree->set_expansion_type(UTree::OBSERVATION_REWARD_EXPANSION);
                return {true,"set expansion type to OBSERVATION_REWARD_EXPANSION"};
            }, "set expansion type for UTree");
        command_center.add_command(top_model_learn_utree,{"utree-f"}, [this]()->ret_t{
                utree->print_features();
                return {true,"printed UTree features"};
            }, "print UTree features");
    }
    {
        pair<double,QString> top_model_learn_tem(3.5,"TEM ----------------------------------------------------");
        command_center.add_command(top_model_learn_tem,{"tem grow","temg"}, [this]()->ret_t{
                tem->grow_feature_set();
                return {true,"grew TEM features"};
            }, "grow TEM features");
        command_center.add_command(top_model_learn_tem,{"tem optimize","temo"}, [this]()->ret_t{
                tem->optimize_weights_LBFGS();
                return {true,"optimized TEM"};
            }, "optimize TEM");
        command_center.add_command(top_model_learn_tem,{"tem shrink","tems"}, [this]()->ret_t{
                tem->shrink_feature_set();
                return {true,"shrank TEM features"};
            }, "shrink TEM features");
        command_center.add_command(top_model_learn_tem,{"tem print","temp"}, [this]()->ret_t{
                tem->print_features();
                return {true,"printed TEM features"};
            }, "print TEM features");
        command_center.add_command(top_model_learn_tem,{"tem cycle","temc"}, [this]()->ret_t{
                tem->grow_feature_set();
                tem->optimize_weights_LBFGS();
                tem->shrink_feature_set();
                tem->print_features();
                return {true,"did one learning cycle of TEM"};
            }, "do a complete grow-optimize-shrink cycle of TEM and print features afterwards");
        command_center.add_command(top_model_learn_tem,{"tem max horizon","temmaxh"}, [this]()->ret_t{
                return {true,QString("TEM maximum horizon is %1").arg(N_plus_TEM->get_max_horizon())};
            }, "get TEM maximum horizon");
        command_center.add_command(top_model_learn_tem,{"tem max horizon","temmaxh"}, [this](int h)->ret_t{
                N_plus_TEM->set_max_horizon(h);
                return {true,QString("set TEM maximum horizon to %1").arg(N_plus_TEM->get_max_horizon())};
            }, "set TEM maximum horizon");
        command_center.add_command(top_model_learn_tem,{"tem min horizon","temminh"}, [this]()->ret_t{
                return {true,QString("TEM minimum horizon is %1").arg(N_plus_TEM->get_min_horizon())};
            }, "get TEM minimum horizon");
        command_center.add_command(top_model_learn_tem,{"tem min horizon","temminh"}, [this](int h)->ret_t{
                N_plus_TEM->set_min_horizon(h);
                return {true,QString("set TEM minimum horizon to %1").arg(N_plus_TEM->get_min_horizon())};
            }, "set TEM maximum horizon");
        command_center.add_command(top_model_learn_tem,{"tem horizon extension","temhe"}, [this]()->ret_t{
                return {true,QString("TEM horizon extension is %1").arg(N_plus_TEM->get_horizon_extension())};
            }, "get TEM horizon extension");
        command_center.add_command(top_model_learn_tem,{"tem horizon extension","temhe"}, [this](int h)->ret_t{
                N_plus_TEM->set_horizon_extension(h);
                return {true,QString("set TEM horizon extension to %1").arg(N_plus_TEM->get_horizon_extension())};
            }, "set TEM horizon extension");
        command_center.add_command(top_model_learn_tem,{"tem combine features","temcf"}, [this]()->ret_t{
                if(N_plus_TEM->get_combine_features()) {
                    return {true,"TEM does combines features"};
                } else {
                    return {true,"TEM does not combines features"};
                }
            }, "get whether TEM combines existing features");
        command_center.add_command(top_model_learn_tem,{"tem combine features","temcf"}, [this](bool combine)->ret_t{
                N_plus_TEM->set_combine_features(combine);
                if(N_plus_TEM->get_combine_features()) {
                    return {true,"set TEM to combine features"};
                } else {
                    return {true,"set TEM to not combine features"};
                }
            }, "set whether TEM combines existing features");
    }
    {
        pair<double,QString> top_model_learn_tel(3.6,"TEL ----------------------------------------------------");
        command_center.add_command(top_model_learn_tel,{"tel grow","telg"}, [this]()->ret_t{
                tel->grow_feature_set();
                return {true,"grew TEL features"};
            }, "grow TEL features");
        command_center.add_command(top_model_learn_tel,{"tel optimize","telo"}, [this]()->ret_t{
                tel->run_policy_iteration();
                return {true,"optimized TEL"};
            }, "optimize TEL");
        command_center.add_command(top_model_learn_tel,{"tel shrink","tels"}, [this]()->ret_t{
                tel->shrink_feature_set();
                return {true,"shrank TEL features"};
            }, "shrink TEL features");
        command_center.add_command(top_model_learn_tel,{"tel print","telp"}, [this]()->ret_t{
                tel->print_features();
                return {true,"printed TEL features"};
            }, "print TEL features");
        command_center.add_command(top_model_learn_tel,{"tel cycle","telc"}, [this]()->ret_t{
                tel->grow_feature_set();
                tel->run_policy_iteration();
                tel->shrink_feature_set();
                tel->print_features();
                return {true,"did one learning cycle of TEL"};
            }, "do a complete grow-optimize-shrink cycle of TEL and print features afterwards");
        command_center.add_command(top_model_learn_tel,{"tel max horizon","telmaxh"}, [this]()->ret_t{
                return {true,QString("TEL maximum horizon is %1").arg(N_plus_TEL->get_max_horizon())};
            }, "get TEL maximum horizon");
        command_center.add_command(top_model_learn_tel,{"tel max horizon","telmaxh"}, [this](int h)->ret_t{
                N_plus_TEL->set_max_horizon(h);
                return {true,QString("set TEL maximum horizon to %1").arg(N_plus_TEL->get_max_horizon())};
            }, "set TEL maximum horizon");
        command_center.add_command(top_model_learn_tel,{"tel min horizon","telminh"}, [this]()->ret_t{
                return {true,QString("TEL minimum horizon is %1").arg(N_plus_TEL->get_min_horizon())};
            }, "get TEL minimum horizon");
        command_center.add_command(top_model_learn_tel,{"tel min horizon","telminh"}, [this](int h)->ret_t{
                N_plus_TEL->set_min_horizon(h);
                return {true,QString("set TEL minimum horizon to %1").arg(N_plus_TEL->get_min_horizon())};
            }, "set TEL maximum horizon");
        command_center.add_command(top_model_learn_tel,{"tel horizon extension","telhe"}, [this]()->ret_t{
                return {true,QString("TEL horizon extension is %1").arg(N_plus_TEL->get_horizon_extension())};
            }, "get TEL horizon extension");
        command_center.add_command(top_model_learn_tel,{"tel horizon extension","telhe"}, [this](int h)->ret_t{
                N_plus_TEL->set_horizon_extension(h);
                return {true,QString("set TEL horizon extension to %1").arg(N_plus_TEL->get_horizon_extension())};
            }, "set TEL horizon extension");
        command_center.add_command(top_model_learn_tel,{"tel combine features","telcf"}, [this]()->ret_t{
                if(N_plus_TEL->get_combine_features()) {
                    return {true,"TEL does combines features"};
                } else {
                    return {true,"TEL does not combines features"};
                }
            }, "get whether TEL combines existing features");
        command_center.add_command(top_model_learn_tel,{"tel combine features","telcf"}, [this](bool combine)->ret_t{
                N_plus_TEL->set_combine_features(combine);
                if(N_plus_TEL->get_combine_features()) {
                    return {true,"set TEL to combine features"};
                } else {
                    return {true,"set TEL to not combine features"};
                }
            }, "set whether TEL combines existing features");
    }
    {
        pair<double,QString> top_planning(4,"Planning ===============================================");
        command_center.add_command(top_planning,{"set planner o","s p o"}, [this]()->ret_t{
                planner_type = OPTIMAL_LOOK_AHEAD;
                set_policy();
                return {true,"using optimal planner" };
            }, "use optimal predictions (give by maze)");
        command_center.add_command(top_planning,{"set planner mu","s p mu"}, [this]()->ret_t{
                planner_type = UTREE_LOOK_AHEAD;
                set_policy();
                return {true,"using model-based UTree planner" };
            }, "use UTree as predictive model");
        command_center.add_command(top_planning,{"set planner vu","s p vu"}, [this]()->ret_t{
                planner_type = UTREE_VALUE;
                set_policy();
                return {true,"using value-based UTree for action selection" };
            }, "use UTree as value function");
        command_center.add_command(top_planning,{"set planner tel","s p l"}, [this]()->ret_t{
                planner_type = TEL_VALUE;
                set_policy();
                return {true,"using linear combined TEFs for Q-approximation (TEL)" };
            }, "use TEL");
        command_center.add_command(top_planning,{"set planner g","s p g"}, [this]()->ret_t{
                planner_type = GOAL_ITERATION;
                set_policy();
                return {true,"using goal-guided value iteration" };
            }, "use goal-guided value iteration");
        command_center.add_command(top_planning,{"set planner r","s p r"}, [this]()->ret_t{
                planner_type = RANDOM;
                set_policy();
                return {true,"using random policy" };
            }, "use random policy");
        command_center.add_command(top_planning,{"set planner tem","s p t"}, [this]()->ret_t{
                planner_type = TEM_LOOK_AHEAD;
                set_policy();
                return {true,"using TEM for planning" };
            }, "use TEM predictions for planning");
        command_center.add_command(top_planning,{"set goal","s g"}, [this]()->ret_t{
                shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
                if(maze==nullptr) {
                    return {true,"Allowed for maze-environments only"};
                } else {
                    // set flag
                    goal_activated = true;
                    // get state
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
                    // print message
                    return {true,"goal set to current state" };
                }
            }, "activate goal state at current location");
        command_center.add_command(top_planning,{"set goal random","s g r"}, [this]()->ret_t{
                shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
                if(maze==nullptr) {
                    return {true,"Allowed for maze-environments only"};
                } else {
                    // set flag
                    goal_activated = true;
                    // get state
                    goal_state = observation_space->random_element();
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
                    // print message
                    return {true,"goal set to random location" };
                }
            }, "activate goal state at random location");
        command_center.add_command(top_planning,{"unset goal"}, [this]()->ret_t{
                if(!goal_activated) {
                    return {true,"goal already inactive" };
                } else {
                    goal_activated = false;
                    return {true,"deactivated goal" };
                }
            }, "deactivate goal state");
        command_center.add_command(top_todo,{"set prune tree"}, [this]()->ret_t{
                return {true,"...to be ported"};
            }, "prune search tree");
        command_center.add_command(top_todo,{"unset prune tree"}, [this]()->ret_t{
                return {true,"...to be ported"};
            }, "don't prune search tree");
        command_center.add_command(top_planning,{"discount"}, [this]()->ret_t{
                return {true,QString("discount is %1").arg(discount)};
            }, "get discount");
        command_center.add_command(top_planning,{"discount"}, [this](double d)->ret_t{
                if(0<=d && d<=1) {
                    discount = d;
                    utree->set_discount(discount);
                    tel->set_discount(discount);
                    shared_ptr<LookAheadPolicy> look_ahead_policy = dynamic_pointer_cast<LookAheadPolicy>(policy);
                    if(look_ahead_policy!=nullptr) {
                        look_ahead_policy->set_discount(discount);
                    }
                    return {true,QString("set discount to %1").arg(discount)};
                } else {
                    return {false,"discount must be in [0,1]"};
                }
            }, "set discount");
        command_center.add_command(top_planning,{"print tree"}, [this](bool text = false, bool graphic = false)->ret_t{
                shared_ptr<LookAheadPolicy> look_ahead_policy = dynamic_pointer_cast<LookAheadPolicy>(policy);
                if(look_ahead_policy!=nullptr) {
                    const LookAheadSearch & s = look_ahead_policy->get_look_ahead_search();
                    s.print_tree_statistics();
                    s.print_tree(text, graphic);
                }
                return {true,"Printed tree statistics"};
            }, "print Look-Ahead-Tree statistics [optionally with text and graphic output]");
        command_center.add_command(top_planning,{"max-tree-size"}, [this]()->ret_t{
                return {true,QString( "max tree size is %1" ).arg(max_tree_size) };
            }, "get maximum size of Look-Ahead-Tree (zero corresponds to infinite)");
        command_center.add_command(top_planning,{"max-tree-size"}, [this](int n)->ret_t{
                if(n<0) {
                    return {false,"Tree size must be non-negative"};
                }
                max_tree_size = n;
                shared_ptr<LookAheadPolicy> look_ahead_policy = dynamic_pointer_cast<LookAheadPolicy>(policy);
                if(look_ahead_policy!=nullptr) {
                    look_ahead_policy->set_max_tree_size(max_tree_size);
                    return {true,"Set maximum Look-Ahead-Tree size"};
                } else {
                    return {false,"Current policy is not look-ahead-search"};
                }
            }, "set maximum size of Look-Ahead-Tree (zero for infinite)");
    }
    {
        pair<double,QString> top_new_stuff(5,"New Stuff  =============================================");
        //---------------------------------New Stuff----------------------------------
        command_center.add_command(top_new_stuff,{"col-states"}, [this]()->ret_t{
                shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
                if(maze==nullptr) {
                    return {true,"allowed for maze-environments only"};
                } else {
                    Maze::color_vector_t cols;
                    for(observation_ptr_t observation : observation_space) {
                        cols.push_back( std::make_tuple(drand48(),drand48(),drand48()) );
                    }
                    maze->set_state_colors(cols);
                    maze->render_update();
                }
                return {true,"colored states"};
            }, "color states (random)");
        command_center.add_command(top_todo,{"fixed-dt-dist","fdd"}, [this](int)->ret_t{
                return {true,"...to be ported"};
            }, "show probability for a state to occur <int> steps after current state");
        command_center.add_command(top_todo,{"pair-delay-dist","pdd"}, [this]()->ret_t{
                return {true,"...to be ported"};
            }, "show temporal delay distribution from current state to goal state");
        command_center.add_command(top_todo,{"pair-delay-dist","pdd"}, [this](int)->ret_t{
                return {true,"...to be ported"};
            }, "restrict to time window of width <int>");
        command_center.add_command(top_todo,{"mediator-probability","mp"}, [this](int)->ret_t{
                return {true,"...to be ported"};
            }, "show probability for a state to occurr between current state and goal state given a time window of width <int>");
        command_center.add_command(top_new_stuff,{"p-distribution","pd"}, [this](QString a)->ret_t{
                // check action
                if(a!="u" && a!="d" && a!="l" && a!="r" && a!="s") {
                    return {false,"expect one of 'u', 'd', 'l', 'r', 's' as action"};
                }
                // check if environment is a maze
                shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
                if(maze==nullptr) {
                    return {true,"allowed for maze-environments only"};
                }
                // get action
                action_ptr_t action;
                if(a=="u") {
                    action = action_ptr_t(new MazeAction("up"));
                } else if(a=="d") {
                    action = action_ptr_t(new MazeAction("down"));
                } else if(a=="l") {
                    action = action_ptr_t(new MazeAction("left"));
                } else if(a=="r") {
                    action = action_ptr_t(new MazeAction("right"));
                } else if(a=="s") {
                    action = action_ptr_t(new MazeAction("stay"));
                } else {
                    DEBUG_DEAD_LINE;
                    action = action_ptr_t(new MazeAction("stay"));
                }
                // get probability distribution
                auto p_map = tem->get_prediction_map(current_instance,action);
                // set colors
                vector<double> dist;
                for(observation_ptr_t observation : observation_space) {
                    double p = 0;
                    for(reward_ptr_t reward : reward_space) {
                        p += p_map[make_tuple(observation,reward)];
                    }
                    dist.push_back(p);
                }
                maze->show_distribution(dist,true);
                return {true,"displayed p-distribution"};
            }, "color maze according to (sqrt of) probability distribution for TEM and action <QString>");
        command_center.add_command(top_new_stuff,{"stationary-distribution","sd"}, [this]()->ret_t{
                using namespace arma;
                // check if environment is a maze
                shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
                if(maze==nullptr) {
                    return {true,"allowed for maze-environments only"};
                }
                // get transition matrix and stationary distribution
                mat T;
                o_r_idx_map_t o_r_idx_map;
                get_TEM_transition_matrix_and_o_r_index_map(T,o_r_idx_map);
                vec stat_dist;
                bool stat_dist_found = get_stationary_distribution(T,stat_dist);
                // debug print
                IF_DEBUG(3) {
                    cout.precision(3);
                    cout.setf(std::ios::fixed);
                    cout.width(10);
                    cout << "Transition Matrix:" << endl;
                    for(auto o_r_1 : o_r_idx_map) {
                        cout << o_r_1.first << "	";
                        for(auto o_r_2 : o_r_idx_map) {
                            cout << T(o_r_1.second,o_r_2.second) << "	";
                        }
                        cout << endl;
                    }
                    cout.unsetf(cout.flags());
                }
                // return or show distribution
                if(!stat_dist_found) {
                    return {true,"Could not find stationary distribution"};
                } else {
                    // set colors
                    vector<double> dist;
                    for(observation_ptr_t observation : observation_space) {
                        double p = 0;
                        for(reward_ptr_t reward : reward_space) {
                            p += stat_dist[o_r_idx_map[make_tuple(observation,reward)]];
                        }
                        dist.push_back(p);
                    }
                    maze->show_distribution(dist,true);
                    return {true,"showed stationary distribution"};
                }
            }, "show stationary distribution (sqrt) of TEM");
        command_center.add_command(top_new_stuff,{"inverse-distribution","id"}, [this]()->ret_t{
                using namespace arma;
                // check if environment is a maze
                shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
                if(maze==nullptr) {
                    return {true,"allowed for maze-environments only"};
                }
                // get transition matrix and stationary distribution
                mat T;
                o_r_idx_map_t o_r_idx_map;
                get_TEM_transition_matrix_and_o_r_index_map(T,o_r_idx_map);
                vec stat_dist;
                bool stat_dist_found = get_stationary_distribution(T,stat_dist);
                // return if stationary distribution could not be identified
                if(!stat_dist_found) {
                    return {true,"Could not find stationary distribution"};
                }
                // invert conditional distribution
                map<observation_ptr_t,probability_t> inv_dist;
                {
                    // current observation
                    observation_ptr_t current_obs = current_instance->observation;
                    // for normalization check
                    probability_t normalization_check;
                    // get marginal stationary observation distributions
                    normalization_check = 0;
                    map<observation_ptr_t,probability_t> marg_stat_p_o;
                    for(observation_ptr_t observation : observation_space) {
                        probability_t& p_ref = marg_stat_p_o[observation];
                        for(reward_ptr_t reward : reward_space) {
                            p_ref += stat_dist[o_r_idx_map[make_tuple(observation,reward)]];
                        }
                        normalization_check += p_ref;
                    }
                    if(fabs(normalization_check-1)>1e-3) {
                        DEBUG_WARNING("Marginal stationary distribution over observations not normalized (sum = " << normalization_check << ")");
                        for(observation_ptr_t observation : observation_space) {
                            DEBUG_OUT(0,"p(" << observation << ") = " << marg_stat_p_o[observation]);
                        }
                    }
                    // get marginal stationary reward distributions
                    normalization_check = 0;
                    map<reward_ptr_t,probability_t> marg_stat_p_r;
                    for(reward_ptr_t reward : reward_space) {
                        probability_t& p_ref = marg_stat_p_r[reward];
                        for(observation_ptr_t observation : observation_space) {
                            p_ref += stat_dist[o_r_idx_map[make_tuple(observation,reward)]];
                        }
                        normalization_check += p_ref;
                    }
                    if(fabs(normalization_check-1)>1e-3) {
                        DEBUG_WARNING("Marginal stationary distribution over rewards not normalized (sum = " << normalization_check << ")");
                        for(reward_ptr_t reward : reward_space) {
                            DEBUG_OUT(0,"p(" << reward << ") = " << marg_stat_p_r[reward]);
                        }
                    }
                    // get inverse probability
                    normalization_check = 0;
                    for(observation_ptr_t observation_t_Delta_t : observation_space) {
                        // marginalize out rewards
                        probability_t marg_conditional = 0;
                        for(reward_ptr_t reward_t : reward_space) {
                            for(reward_ptr_t reward_t_Delta_t : reward_space) {
                                marg_conditional += marg_stat_p_r[reward_t_Delta_t]*T(
                                    o_r_idx_map[make_tuple(current_obs,reward_t)],
                                    o_r_idx_map[make_tuple(observation_t_Delta_t,reward_t_Delta_t)]
                                    );
                            }
                        }
                        // update inverse probability for observation
                        inv_dist[observation_t_Delta_t] = marg_conditional;
                        normalization_check += marg_conditional;
                    }
                    if(fabs(normalization_check-1)>1e-3) {
                        DEBUG_WARNING("Marginal conditional distribution over observation not normalized (sum = " << normalization_check << ")");
                        for(observation_ptr_t observation : observation_space) {
                            DEBUG_OUT(0,"p(" << observation << ") = " << inv_dist[observation]);
                        }
                    }

                    // warning if state space does not match
                    bool observation_found_in_space = false;
                    for(observation_ptr_t observation : observation_space) {
                        if(observation==current_obs) {
                            observation_found_in_space = true;
                            break;
                        }
                    }
                    if(!observation_found_in_space) {
                        DEBUG_WARNING("Current state of agent is not in current state space");
                    }

                    // observation_ptr_t current_obs = current_instance->observation;
                    // bool observation_found_in_space = false;
                    // map<observation_ptr_t,probability_t> p_stat_o_t_Delta_t;
                    // probability_t p_stat_o_t = 0;
                    // map<observation_ptr_t,probability_t> T_p_stat;
                    // probability_t normalization_check_1 = 0;
                    // probability_t normalization_check_2 = 0;
                    // // get probability of current observation according to
                    // // stationary distribution
                    // for(reward_ptr_t reward : reward_space) {
                    //     p_stat_o_t += stat_dist[o_r_idx_map[make_tuple(current_obs,reward)]];
                    // }
                    // // get stationary probability distribution over past
                    // // observation (at time t - Delta t)
                    // normalization_check_1 = 0;
                    // for(observation_ptr_t observation : observation_space) {
                    //     probability_t& p_stat_ref = p_stat_o_t_Delta_t[observation];
                    //     for(reward_ptr_t reward : reward_space) {
                    //         probability_t tmp_p = stat_dist[o_r_idx_map[make_tuple(observation,reward)]];
                    //         p_stat_ref += tmp_p;
                    //         normalization_check_1 += tmp_p;
                    //     }
                    // }
                    // IF_DEBUG(1) {
                    //     if(fabs(normalization_check_1-1)>1e-3) {
                    //         DEBUG_WARNING("Stationary distribution over past state not normalized (sum = " << normalization_check_1 << ")");
                    //         for(observation_ptr_t observation : observation_space) {
                    //             DEBUG_OUT(0,"p(" << observation << ") = " << p_stat_o_t_Delta_t[observation]);
                    //         }
                    //     }
                    // }
                    // // get T*p
                    // normalization_check_1 = 0;
                    // normalization_check_2 = 0;
                    // {
                    //     map<observation_ptr_t,probability_t> T_dist, p_dist;
                    //     for(observation_ptr_t observation_t_Delta_t : observation_space) {
                    //         probability_t& Tp_ref = T_p_stat[observation_t_Delta_t];
                    //         probability_t& T_dist_ref = T_dist[observation_t_Delta_t];
                    //         probability_t& p_dist_ref = p_dist[observation_t_Delta_t];
                    //         for(reward_ptr_t reward_t : reward_space) {
                    //             for(reward_ptr_t reward_t_Delta_t : reward_space) {
                    //                 probability_t T_tmp = T(
                    //                     o_r_idx_map[make_tuple(observation_t_Delta_t,reward_t_Delta_t)],
                    //                     o_r_idx_map[make_tuple(current_obs,reward_t)]
                    //                     );
                    //                 T_dist_ref += T_tmp;
                    //                 probability_t p_tmp = stat_dist[
                    //                     o_r_idx_map[make_tuple(observation_t_Delta_t,reward_t_Delta_t)]
                    //                     ];
                    //                 p_dist_ref += p_tmp;
                    //                 Tp_ref += T_tmp*p_tmp;
                    //             }
                    //         }
                    //         normalization_check_1 += T_dist_ref;
                    //         normalization_check_2 += p_dist_ref;
                    //     }
                    //     IF_DEBUG(1) {
                    //         if(fabs(normalization_check_1-1)>1e-3) {
                    //             DEBUG_WARNING("Transition distribution not normalized (sum = " << normalization_check_1 << ")");
                    //             for(observation_ptr_t observation : observation_space) {
                    //                 DEBUG_OUT(0,"p(" << observation << ") = " << T_dist[observation]);
                    //             }
                    //         }
                    //         if(fabs(normalization_check_2-1)>1e-3) {
                    //             DEBUG_WARNING("Stationary distribution not normalized (sum = " << normalization_check_2 << ")");
                    //             for(observation_ptr_t observation : observation_space) {
                    //                 DEBUG_OUT(0,"p(" << observation << ") = " << p_dist[observation]);
                    //             }
                    //         }
                    //     }
                    // }
                    // // get inverse
                    // normalization_check_1 = 0;
                    // for(observation_ptr_t observation : observation_space) {
                    //     if(observation==current_obs) {
                    //         observation_found_in_space = true;
                    //     }
                    //     probability_t tmp_p = p_stat_o_t_Delta_t[observation]*T_p_stat[observation]/p_stat_o_t;
                    //     inv_dist[observation] = tmp_p;
                    //     normalization_check_1 += tmp_p;
                    // }
                    // IF_DEBUG(1) {
                    //     if(fabs(normalization_check_1-1)>1e-3) {
                    //         DEBUG_WARNING("Inverse distribution not normalized (sum = " << normalization_check_1 << ")");
                    //         for(observation_ptr_t observation : observation_space) {
                    //             DEBUG_OUT(0,"p(" << observation << ") = " << inv_dist[observation]);
                    //         }
                    //     }
                    // }

                }
                // set colors
                vector<double> dist;
                for(observation_ptr_t observation : observation_space) {
                    dist.push_back(inv_dist[observation]);
                }
                maze->show_distribution(dist,true);
                return {true,"showed inverse distribution"};
            }, "show inverse observation distribution (sqrt) of TEM");
    }

//        // } else if(str_args[0]=="validate" || str_args[0]=="v") {
//        //     if(str_args_n==1 ) {
//        //         return {, invalid_args_s };
//        //         return {, validate_s };
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
//        //                 return {,QString("    MC KL-Divergence = %1 (%2 samples)").arg(kl).arg(int_args[3])};
//        //                 return {,QString("    Mean Likelihood: model = %1, maze = %2").arg(model_l).arg(maze_l)};
//        //             } else {
//        //                 return {, "    Please specify a valid sample size" };
//        //             }
//        //         } else {
//        //             return {, invalid_args_s };
//        //             return {, validate_s };
//        //         }
//        //     } else if(str_args[1]=="kmdp") {
//        //         return {, "    Sorry, not implemented" };
//        //     } else {
//        //         return {, invalid_args_s };
//        //         return {, validate_s };
//        //     }
//        } else if(str_args[0]=="score-lq" || str_args[0]=="slq") {
//            if(str_args_n==1) {
//                return {,score_lq_s};
//            } else if(int_args_ok[1] && int_args[1]>=0 ) {
//                linQ->construct_candidate_features(int_args[1]);
//                linQ->score_candidates_by_gradient();
//            } else {
//                return {, invalid_args_s };
//                return {, score_lq_s };
//            }
//        } else if(str_args[0]=="add-lq") {
//            if(str_args_n==1) {
//                return {, invalid_args_s };
//                return {, add_lq_s };
//            } else if(int_args_ok[1] && int_args[1]>=0 ) {
//                linQ->add_candidates_by_score(int_args[1]);
//            } else {
//                return {, invalid_args_s };
//                return {, add_lq_s };
//            }
//        } else if(str_args[0]=="crf-erase" || str_args[0]=="ce") {

//        } else if(str_args[0]=="exit" || str_args[0]=="quit" || str_args[0]=="q") { // quit application
//            QApplication::quit();
//        } else if(str_args[0]=="max-tree-size") { // set tree size

//        } else if(str_args[0]=="set" || str_args[0]=="unset") { // set option

//            } else if(str_args[1]=="prune-tree") {
//                if(str_args[0]=="set") {
//                    prune_search_tree=true;
//                    return {, "    prune search tree" };
//                } else {
//                    prune_search_tree=false;
//                    return {, "    don't prune search tree" };
//                }
//                shared_ptr<LookAheadPolicy> look_ahead_policy = dynamic_pointer_cast<LookAheadPolicy>(policy);
//                if(look_ahead_policy!=nullptr) {
//                    look_ahead_policy->set_pruning(prune_search_tree);
//                    look_ahead_policy->invalidate_search_tree();
//                }
//            } else if(str_args[1]=="png") {
//                if(str_args[0]=="set") {
//                    save_png_on_transition=true;
//                    return {, "    save png on transition" };
//                } else {
//                    save_png_on_transition=false;
//                    return {, "    don't save png on transition" };
//                }
//            } else {
//                return {, invalid_args_s };
//                return {, set_s };
//            }
//        } else if(str_args[0]=="construct" || str_args[0]=="con") {
//            if(str_args_n==1) {
//                return {,construct_s};
//            } else if(int_args_ok[1] && int_args[1]>=0 ) {
//                linQ->add_all_candidates(int_args[1]);
//            } else {
//                return {, invalid_args_s };
//                return {, construct_s };
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
//            return {, QString("    Mean Data Likelihood: %1").arg(like) };
//
//            //return {, "    currently no test function implemented" };
//        } else if(str_args[0]=="col-states") { // color states

//        } else if(str_args[0]=="fixed-dt-dist" || str_args[0]=="fdd") { // show delay probability
//            shared_ptr<Maze> maze = dynamic_pointer_cast<Maze>(environment);
//            if(maze==nullptr) {
//                return {,"    Allowed for maze-environments only"};
//            } else {
//                if(str_args_n!=2 || !int_args_ok[1]) {
//                    return {, invalid_args_s };
//                    return {, fixed_dt_distribution_s };
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
//                return {, invalid_args_s };
//                return {, pair_delay_distribution_s };
//            } else {
//                if(!goal_activated) {
//                    return {, "    Goal state must be activated to calculate delay distribution" };
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
//                return {,"    Allowed for maze-environments only"};
//            } else {
//                if(str_args_n==2 && int_args_ok[1]) {
//                    if(!goal_activated) {
//                        return {, "    Goal state must be activated to calculate mediator probabilities" };
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
//                    return {, invalid_args_s };
//                    return {, mediator_probability_s };
//                }
//            }
//        } else {
//            return {,"    unknown command"};
//        }
//    }
}

void TestMaze_II::to_console(QString x, TEXT_STYLE style, int indentation) {
    // set cursor to the end
    ui._wConsoleOutput->moveCursor(QTextCursor::End);
    // construct HTML line
    QString line;
    {
        // indentation string
        QString indent_str = QString("&nbsp;").repeated(indentation);
        // escape special characters
        x = x.toHtmlEscaped();
        // replace spaces
        x.replace(" ","&nbsp;");
        // replace new lines with correct html plus indentation
        x.replace("\n","<br>"+indent_str);
        // opening
        switch(style) {
        case INPUT_STYLE:
            line = line + "<font color=\"#00a\">";
            break;
        case OK_RESPONSE_STYLE:
            line = line + "<font color=\"#0a0\">";
            break;
        case ERROR_RESPONSE_STYLE:
            line = line + "<font color=\"#a00\">";
            break;
        case NORMAL_STYLE:
        default:
            line = line + "<font color=\"#000\">";
            break;
        }
        // actual text
        line = line + indent_str + x;
        // closing
        switch(style) {
        case INPUT_STYLE:
            line = line + "</font><br>";
            break;
        case OK_RESPONSE_STYLE:
            line = line + "</font><br>";
            break;
        case ERROR_RESPONSE_STYLE:
            line = line + "</font><br>";
            break;
        case NORMAL_STYLE:
        default:
            line = line + "</font><br>";
            break;
        }
    }
    // insert line
    ui._wConsoleOutput->insertHtml(line);
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
        learning_episode_begin = current_instance;
        for(int idx=0; idx<length; ++idx) {
            action_ptr_t action = policy->get_action(current_instance);
            observation_ptr_t observation_to;
            reward_ptr_t reward;
            environment->perform_transition(action,observation_to,reward);
            update_current_instance(action,observation_to,reward);
            add_action_observation_reward_tripel(action,observation_to,reward);
        }
        learning_episode_end = current_instance;
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
         utree->add_action_observation_reward_tripel(action,observation,reward,start_new_episode);
          tel->add_action_observation_reward_tripel(action,observation,reward,start_new_episode);
          tem->add_action_observation_reward_tripel(action,observation,reward,start_new_episode);
    delay_dist.add_action_observation_reward_tripel(action,observation,reward,start_new_episode);
    DEBUG_OUT(1,"Add (" << action << ", " << observation << ", " << reward << ")" <<
              (start_new_episode?" new episode":""));
    start_new_episode = false;
}

void TestMaze_II::clear_data() {
    utree->clear_data();
    tel->clear_data();
    tem->clear_data();
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
        utree->adopt_spaces(*environment);
        utree->set_features(*environment);
        tel->adopt_spaces(*environment);
        N_plus_TEL->adopt_spaces(*environment);
        tem->adopt_spaces(*environment);
        N_plus_TEM->adopt_spaces(*environment);
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
    utree.reset(new UTree(discount));
    tel.reset(new TemporallyExtendedLinearQ(N_plus_TEL,discount));
    tem.reset(new TemporallyExtendedModel(N_plus_TEM));
    tem->set_l1_factor(l1_factor);
    tel->set_l1_factor(l1_factor);
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
    case UTREE_LOOK_AHEAD: {
        shared_ptr<Predictor> pred = dynamic_pointer_cast<Predictor>(utree);
        if(pred!=nullptr) {
            policy.reset(new LookAheadPolicy(discount,pred,prune_search_tree,max_tree_size));
        } else {
            DEBUG_DEAD_LINE;
        }
        break;
    }
    case TEM_LOOK_AHEAD: {
        shared_ptr<Predictor> pred = dynamic_pointer_cast<Predictor>(tem);
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
    case TEL_VALUE: {
        shared_ptr<Policy> p = dynamic_pointer_cast<Policy>(tel);
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
    default:
        policy.reset();
    }
}

double TestMaze_II::validate_predictor_on_random_episode(int n,  const Predictor& pred) {
    double log_prob = 0;
    repeat(n) {
        action_ptr_t action = action_space->random_element();
        observation_ptr_t observation;
        reward_ptr_t reward;
        perform_transition(action,observation,reward);
        DEBUG_OUT(3,current_instance->const_prev() << " --> " << current_instance);
        double p;
        IF_DEBUG(4) {
            auto p_map = pred.get_prediction_map(current_instance->const_prev(),action);
            p = p_map[make_tuple(observation,reward)];
            for(auto x : p_map) {
                DEBUG_OUT(0,get<0>(x) << " : " << get<1>(x));
            }
        } else {
            p = pred.get_prediction(current_instance->const_prev(),action,observation,reward);
        }
        DEBUG_OUT(3,"   --> p = " << p);
        log_prob += log(p);
    }
    log_prob /= n;
    return exp(log_prob);
}

double TestMaze_II::validate_predictor_on_training_episode(const Predictor& pred) {
    double log_prob = 0;
    int counter = 0;
    for(instance_ptr_t ins = learning_episode_begin; ins!=learning_episode_end; ++counter, ++ins) {
        DEBUG_OUT(3,ins << " --> " << ins->const_next());
        double p;
        IF_DEBUG(4) {
            auto p_map = pred.get_prediction_map(ins,ins->const_next()->action);
            p = p_map[make_tuple(ins->const_next()->observation,ins->const_next()->reward)];
            for(auto x : p_map) {
                DEBUG_OUT(4,get<0>(x) << " : " << get<1>(x));
            }
        } else {
            p = pred.get_prediction(ins,ins->const_next()->action,ins->const_next()->observation,ins->const_next()->reward);
        }
        DEBUG_OUT(3,"   --> p = " << p);
        log_prob += log(p);
    }
    log_prob /= counter;
    return exp(log_prob);
}

void TestMaze_II::get_TEM_transition_matrix_and_o_r_index_map(arma::mat& T, o_r_idx_map_t& o_r_idx_map) const {
    using namespace arma;
    // memory check
    bool check_memory = AbstractInstance::memory_check_request();
    int memory_count = 0;
    if(check_memory) {
        memory_count = AbstractInstance::memory_check(false);
    }
    // get all action-observation-reward triplets
    vector<tuple<action_ptr_t,observation_ptr_t,reward_ptr_t>> aor_triplets;
    o_r_idx_map.clear();
    {
        int o_r_idx = 0;
        for(observation_ptr_t observation : observation_space) {
            for(reward_ptr_t reward : reward_space) {
                for(action_ptr_t action : action_space) {
                    aor_triplets.push_back(make_tuple(action,observation,reward));
                }
                o_r_idx_map[make_tuple(observation,reward)] = o_r_idx;
                ++o_r_idx;
            }
        }
    }
    // initialize transition matrix
    T.zeros(o_r_idx_map.size(), o_r_idx_map.size());
    {
        // construct sequence
        instance_ptr_t first_instance = DoublyLinkedInstance::create(action_space,observation_space,reward_space); // first instance (time=0) from sequence back to min_h-1
        auto last_instance = DoublyLinkedInstance::get_shared_ptr(first_instance,true);                            // last instance in sequence (time=min_h-1)
        {
            DEBUG_OUT(3,"Constructing sequence:");
            DEBUG_OUT(3,"    " << *last_instance);
            for(int t_idx=1; t_idx<N_plus_TEM->get_max_horizon(); ++t_idx) {
                last_instance->set_non_const_predecessor(DoublyLinkedInstance::create(action_space,observation_space,reward_space));
                last_instance = DoublyLinkedInstance::get_shared_ptr(last_instance->non_const_prev(),true);
                DEBUG_OUT(3,"    " << *last_instance);
            }
        }
        // compute transition maxtrix
        IF_DEBUG(1) {ProgressBar::init("Computing T-matrix:");}
        int progress_idx = 0, max_progress = aor_triplets.size();
        int n_actions = action_space->space_size();
        for(auto from_aor_triplet : aor_triplets) { // go through all possible histories (that matter)
            int col_idx = o_r_idx_map[make_tuple(get<1>(from_aor_triplet),get<2>(from_aor_triplet))];
            last_instance->set_non_const_predecessor(DoublyLinkedInstance::create(get<0>(from_aor_triplet), get<1>(from_aor_triplet), get<2>(from_aor_triplet)));
            for(action_ptr_t action : action_space) {
                auto p_map = tem->get_prediction_map(first_instance->const_prev(),action);
                for(auto o_r : o_r_idx_map) {
                    double p = p_map[o_r.first];
                    int row_idx = o_r.second;
                    T(row_idx,col_idx) += p/(n_actions*n_actions);
                    // DEBUG_OUT(4,"Instance: " << make_tuple(action,get<0>(o_r.first),get<1>(o_r.first)) << "			--> " << p);
                    // for(const_instance_ptr_t ins = first_instance->const_prev(); ins!=INVALID; --ins) {
                    //     DEBUG_OUT(4,"	" << ins);
                    // }
                    IF_DEBUG(1) {
                        if(p==0) { // cannot happen for log-linear model
                            DEBUG_WARNING("    p " << from_aor_triplet << " ---> " << make_tuple(action,get<0>(o_r.first),get<1>(o_r.first)) << " = " << p );
                        }
                    }
                }
            }
            ++progress_idx;
            IF_DEBUG(1) {ProgressBar::print(progress_idx,max_progress);}
        }
        IF_DEBUG(1) {ProgressBar::terminate();}
        // make sure memory is freed
        //first_instance->detach_all(); // not needed, don't know why
    }
    if(check_memory) {
        int memory_count_now = AbstractInstance::memory_check(false);
        if(memory_count_now!=memory_count) {
            DEBUG_WARNING(QString("Memory mismatch: was %1, now is %2").arg(memory_count).arg(memory_count_now));
        }
    }
}

bool TestMaze_II::get_stationary_distribution(const arma::mat& T, arma::vec& stat_dist) const {
    using namespace arma;
    // compute stationary distribution (via eigenvalues and
    // eigenvectors)
    cx_vec eig_vals;
    cx_mat eig_vecs;
    eig_gen(eig_vals,eig_vecs,T);
    int dim = eig_vals.size();
    vec eig_p_norms(dim);
    vec eig_L1_norms(dim);
    mat norm_eig_vecs(dim,dim);
    for(int i=0; i<dim; ++i) {
        norm_eig_vecs.col(i) = real(eig_vecs.col(i));
        eig_p_norms(i) = sum(norm_eig_vecs.col(i));
        eig_L1_norms(i) = sum(abs(norm_eig_vecs.col(i)));
        norm_eig_vecs.col(i) = norm_eig_vecs.col(i)/eig_p_norms(i);
    }
    IF_DEBUG(3) {
        // set stream properties
        cout.precision(3);
        cout.setf(std::ios::fixed);
        cout.width(10);
        // print
        T.raw_print("Transition Matrix:");
        cout.unsetf(cout.flags());
    }
    // eigenmodes
    int eig_idx = -1;
    IF_DEBUG(3) {cout << "Stationary distributions:" << endl;}
    for(int i=0; i<dim; ++i) {
        if(fabs(real(eig_vals(i))-1)<1e-3 && fabs(eig_p_norms(i))>1e-3) {
            IF_DEBUG(3) {
                cout << "EV = " << real(eig_vals(i)) << endl;
                norm_eig_vecs.col(i).raw_print();
            }
            IF_DEBUG(1) {
                if(eig_idx!=-1) {
                    DEBUG_WARNING("Already found multiple eigenvalues of 1");
                }
            }
            eig_idx = i;
        }
    }
    IF_DEBUG(3) {
        cout.precision(3);
        cout.setf(std::ios::fixed);
        cout.width(10);
        cout << "Orthogonal modes:" << endl;
        for(int i=0; i<dim; ++i) {
            if(fabs(eig_p_norms(i))<1e-3) {
                cout << "EV = " << real(eig_vals(i)) << endl;
                (real(eig_vecs.col(i))/eig_L1_norms(i)).raw_print();
            }
        }
        cout << "Invalid modes:" << endl;
        for(int i=0; i<dim; ++i) {
            if(!(fabs(real(eig_vals(i))-1)<1e-3 && fabs(eig_p_norms(i))>1e-3) && !(fabs(eig_p_norms(i))<1e-3)) {
                cout << "EV = " << real(eig_vals(i)) << endl;
                (real(eig_vecs.col(i))/eig_L1_norms(i)).raw_print();
            }
        }
        // eig_vals.raw_print("Eigenvalues:");
        // eig_vecs.raw_print("Eigenvectors:");
        // norm_eig_vecs.raw_print("Normalized eigenvectors:");
        // reset
        cout.unsetf(cout.flags());
    }
    if(eig_idx==-1) { // make sure eigenvalue 1 was found
        stat_dist.zeros(dim);
        return false;
    } else {
        stat_dist = norm_eig_vecs.col(eig_idx);
        IF_DEBUG(1) {stat_dist.print("Stationary distribution");}
        return true;
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
    to_console(input,INPUT_STYLE);
    console_history.push_back(input);
    history_position = console_history.size();
    QTextStream history_file_stream(&history_file);
    history_file_stream << input << "\n";

    // execute command and print response message
    if(input!="") {
        bool ok;
        QString msg = command_center.execute(input,ok);
        if(msg!="") {
            if(ok) {
                to_console(msg,OK_RESPONSE_STYLE,4);
            } else {
                to_console(msg,ERROR_RESPONSE_STYLE,4);
            }
        }
    }

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
