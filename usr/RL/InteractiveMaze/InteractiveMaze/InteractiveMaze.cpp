#include "InteractiveMaze.h"
#include "ui_InteractiveMaze.h"

#include <algorithm> // for std::max
#include <iostream>
#include <assert.h>
#include <float.h>
#include <math.h>

using namespace std;

static const double field_size = 50;          ///< Nominal size of maze fields.
static const double field_down_scale = 0.95;  ///< Factor to scale nominal size to et effective size.
static const double wall_ignore_area = 0.0;   ///< Center part of a field that does not activate a wall (max 0.5).
static const double reward_scale = 10;        ///< Factor to scale reward-discs with.
static const double wall_pen_width = 6;       ///< Line width of walls.
static const double reward_pen_width = 3;     ///< Line width of reward circles.
static const double agent_size = 20;          ///< Size of the agent.
static const QPen wall_off_pen(QBrush(Qt::transparent),wall_pen_width,Qt::SolidLine,Qt::RoundCap);
static const QPen wall_on_pen(QBrush(Qt::black),wall_pen_width,Qt::SolidLine,Qt::RoundCap);
static const QPen positive_reward_pen(QBrush(QColor(200,0,0)),reward_pen_width,Qt::SolidLine,Qt::RoundCap);
static const QPen negativ_reward_pen(QBrush(QColor(0,0,200)),reward_pen_width,Qt::SolidLine,Qt::RoundCap);

InteractiveMaze::InteractiveMaze(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::InteractiveMaze),
    animation_timer(new QTimer(this)),
    value_timer(new QTimer(this)),
    agent_timer(new QTimer(this)),
    input_filter(new InputFilter())
{
    // set up GUI
    ui->setupUi(this);
    // input filter
    ui->_graphics_view->setScene(new QGraphicsScene());
    ui->_graphics_view->setRenderHint(QPainter::Antialiasing);
    ui->_graphics_view->scene()->installEventFilter(input_filter);
    ui->_graphics_view->setFocus();
    connect(input_filter,SIGNAL(right_click(double,double)),this,SLOT(right_click(double,double)));
    connect(input_filter,SIGNAL(left_click(double,double)),this,SLOT(left_click(double,double)));
    connect(input_filter,SIGNAL(scroll_up(double,double)),this,SLOT(scroll_up(double,double)));
    connect(input_filter,SIGNAL(scroll_down(double,double)),this,SLOT(scroll_down(double,double)));
    connect(input_filter,SIGNAL(north_key()),this,SLOT(north_key()));
    connect(input_filter,SIGNAL(east_key()),this,SLOT(east_key()));
    connect(input_filter,SIGNAL(south_key()),this,SLOT(south_key()));
    connect(input_filter,SIGNAL(west_key()),this,SLOT(west_key()));
    connect(input_filter,SIGNAL(stay_key()),this,SLOT(stay_key()));
    // read some values (now that GUI is set up) and initialize maze and model
    // use call back functions but initialize only once after all values are correctly set
    discount_changed(ui->_discount_slider->value(), false);
    random_changed(ui->_random_slider->value(), false);
    x_dim_changed(ui->_horizontal_size->value(), false);
    y_dim_changed(ui->_vertical_size->value(), false);
    agent_check_box_changed(ui->_agent_check_box->isChecked(), false);
    init_maze();
    init_model();
    // connect start timers
    connect(animation_timer, SIGNAL(timeout()), this, SLOT(animate()));
    connect(value_timer, SIGNAL(timeout()), this, SLOT(value_update()));
    connect(agent_timer, SIGNAL(timeout()), this, SLOT(move_agent()));
    animation_timer->start(20);
    init_timers();
}

InteractiveMaze::~InteractiveMaze()
{
    delete ui;
    delete animation_timer;
    delete input_filter;
}

double InteractiveMaze::value_update()
{
    // update Q
    Q = p * (R + discount*V);
    // update V
    double max_diff = 0;
    double min_val = DBL_MAX;
    double max_val = -DBL_MAX;
    for(int x=0; x<x_dim; ++x) {
        for(int y=0; y<y_dim; ++y) {
            double old_value = V(state_idx(x,y));
            double & new_val = V(state_idx(x,y));
            new_val = -DBL_MAX;
            for(int action_idx = 0; action_idx<ACTION_N; ++action_idx) {
                new_val = max(new_val,Q(state_action_idx(x,y,action_idx)));
            }
            max_diff = max(max_diff,fabs(old_value-new_val));
            max_val = max(max_val,new_val);
            min_val = min(min_val,new_val);
        }
    }
    for(int x=0; x<x_dim; ++x) {
        for(int y=0; y<y_dim; ++y) {
            fields[x][y]->setBrush(color_from_value(V(state_idx(x,y)),min_val,max_val));
        }
    }
    return max_diff;
}

void InteractiveMaze::move_agent()
{
    ACTION a = get_action(agent_x_pos,agent_y_pos);
    execute_action(a);
}

void InteractiveMaze::init_model()
{
    Q.resize(ACTION_N*x_dim*y_dim);
    R.zeros(x_dim*y_dim);
    V.resize(x_dim*y_dim);
    p.zeros(ACTION_N*x_dim*y_dim,x_dim*y_dim);
    for(int x1=0; x1<x_dim; ++x1) {
        for(int y1=0; y1<y_dim; ++y1) {
            // reward
            R(state_idx(x1,y1)) = rewards_double[x1][y1];
            // blocked transitions from (x1,y1)
            bool north_blocked = y1==0 || h_walls_bool[x1][y1];
            bool east_blocked = x1==x_dim-1 || v_walls_bool[x1+1][y1];
            bool south_blocked = y1==y_dim-1 || h_walls_bool[x1][y1+1];
            bool west_blocked = x1==0 || v_walls_bool[x1][y1];
            // indices for target states
            int stay_idx = state_idx(x1,y1);
            int north_idx = state_idx(x1,y1-1);
            int east_idx = state_idx(x1+1,y1);
            int south_idx = state_idx(x1,y1+1);
            int west_idx = state_idx(x1-1,y1);
            // random actions (with probability epsilon, i.e., epsilon/ACTION_N per action)
            {
                for(ACTION intended_action = (ACTION)0; intended_action<ACTION_N; intended_action=(ACTION)(intended_action+1)) {
                    int s_a_idx = state_action_idx(x1,y1,intended_action);
                    // random stay
                    p(s_a_idx,stay_idx) += epsilon/ACTION_N;
                    // random north
                    if(north_blocked) {
                        p(s_a_idx,stay_idx) += epsilon/ACTION_N;
                    } else {
                        p(s_a_idx,north_idx) += epsilon/ACTION_N;
                    }
                    // random east
                    if(east_blocked) {
                        p(s_a_idx,stay_idx) += epsilon/ACTION_N;
                    } else {
                        p(s_a_idx,east_idx) += epsilon/ACTION_N;
                    }
                    // random south
                    if(south_blocked) {
                        p(s_a_idx,stay_idx) += epsilon/ACTION_N;
                    } else {
                        p(s_a_idx,south_idx) += epsilon/ACTION_N;
                    }
                    // random west
                    if(west_blocked) {
                        p(s_a_idx,stay_idx) += epsilon/ACTION_N;
                    } else {
                        p(s_a_idx,west_idx) += epsilon/ACTION_N;
                    }
                }
            }
            // intended action (with probability 1-epsilon)
            {
                // NORTH
                if(north_blocked) {
                    p(state_action_idx(x1,y1,NORTH),stay_idx) += 1 - epsilon;
                } else {
                    p(state_action_idx(x1,y1,NORTH),north_idx) += 1 - epsilon;
                }
                // EAST
                if(east_blocked) {
                    p(state_action_idx(x1,y1,EAST),stay_idx) += 1 - epsilon;
                } else {
                    p(state_action_idx(x1,y1,EAST),east_idx) += 1 - epsilon;
                }
                // SOUTH
                if(south_blocked) {
                    p(state_action_idx(x1,y1,SOUTH),stay_idx) += 1 - epsilon;
                } else {
                    p(state_action_idx(x1,y1,SOUTH),south_idx) += 1 - epsilon;
                }
                // WEST
                if(west_blocked) {
                    p(state_action_idx(x1,y1,WEST),stay_idx) += 1 - epsilon;
                } else {
                    p(state_action_idx(x1,y1,WEST),west_idx) += 1 - epsilon;
                }
                // STAY
                p(state_action_idx(x1,y1,STAY),stay_idx) += 1 - epsilon;
            }
            // check normalization
            {
                for(ACTION action = (ACTION)0; action<ACTION_N; action=(ACTION)(action+1)) {
                    double prob_sum = 0;
                    for(int x2=0; x2<x_dim; ++x2) {
                        for(int y2=0; y2<y_dim; ++y2) {
                            prob_sum += p(state_action_idx(x1,y1,action),state_idx(x2,y2));
                        }
                    }
                    if(fabs(prob_sum-1)>1e-10) {
                        cout << "unnormalized probabilities for ";
                        switch(action) {
                        case NORTH:
                            cout << "NORTH";
                            break;
                        case SOUTH:
                            cout << "SOUTH";
                            break;
                        case WEST:
                            cout << "WEST";
                            break;
                        case EAST:
                            cout << "EAST";
                            break;
                        case STAY:
                            cout << "STAY";
                            break;
                        default:
                            break;
                        }
                        cout << " in (" << x1 << "," << y1 << ") : " << prob_sum << endl;
                        for(int x=0; x<x_dim; ++x) {
                            for(int y=0; y<y_dim; ++y) {
                                double& this_p = p(state_action_idx(x1,y1,action),state_idx(x,y));
                                if(this_p!=0) {
                                    cout << "    (" << x << "," << y << ") : " << this_p << endl;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void InteractiveMaze::init_timers()
{
    // value iteration
    value_timer->start(ui->_value_update_speed->value());
    // agent
    double agent_speed_ms = ui->_agent_speed->value();
    if(move_agent_automatically) {
        agent_timer->start(agent_speed_ms);
    } else {
        agent_timer->stop();
    }
    // adapt visual speed
    alpha = 100./agent_speed_ms;
    if(alpha>0.9) {
        // if too fast, just jump. that's easier for the eye.
        alpha = 1;
    }
}

QColor InteractiveMaze::color_from_value(double val, double min_val, double max_val) const
{
    double scale = max<double>({1,fabs(max_val),fabs(min_val)});
    bool positive = val>=0;
    double rescaled = fabs(val/scale);
    if(positive) {
        return QColor(255,255*(1-rescaled),255*(1-rescaled));
    } else {
        return QColor(255*(1-rescaled),255*(1-rescaled),255);
    }
}

InteractiveMaze::ACTION InteractiveMaze::get_action(int x, int y) const
{
    // get action (with randomization)
    double max_value = -DBL_MAX;
    vector<ACTION> optimal_actions;
    for(int action = 0; action<ACTION_N; ++action) {
        double value = Q(state_action_idx(x,y,action));
        if(value > max_value) {
            optimal_actions.assign(1,(ACTION)action);
            max_value = value;
        } else if(value == max_value) {
            optimal_actions.push_back((ACTION)action);
        }
    }
    assert(optimal_actions.size()>0);
    return optimal_actions[rand()%optimal_actions.size()];
}

void InteractiveMaze::set_wall_by_index(int x_in, int y_in, bool set)
{
    double x_idx, y_idx, x_rel, y_rel;
    floating_index_and_relative_coords(x_in,y_in,x_idx,y_idx,x_rel,y_rel);
    if(fabs(x_rel-y_rel)>wall_ignore_area) {
        if(x_rel>y_rel) {
            int x = ceil(x_idx);
            int y = round(y_idx);
            if(x>0&&x<x_dim&&y>=0&&y<y_dim) {
                set_v_wall(x,y,set);
            }
        }
        if(y_rel>x_rel) {
            int x = round(x_idx);
            int y = ceil(y_idx);
            if(y>0&&y<y_dim&&x>=0&&x<x_dim) {
                set_h_wall(x,y,set);
            }
        }
    }
}

void InteractiveMaze::set_h_wall(int x_idx, int y_idx, bool set)
{
    if(!set) {
        h_walls[x_idx][y_idx]->setPen(wall_off_pen);
        h_walls_bool[x_idx][y_idx] = false;
    } else {
        h_walls[x_idx][y_idx]->setPen(wall_on_pen);
        h_walls_bool[x_idx][y_idx] = true;
    }
    init_model();
}

void InteractiveMaze::set_v_wall(int x_idx, int y_idx, bool set)
{
    if(!set) {
        v_walls[x_idx][y_idx]->setPen(wall_off_pen);
        v_walls_bool[x_idx][y_idx] = false;
    } else {
        v_walls[x_idx][y_idx]->setPen(wall_on_pen);
        v_walls_bool[x_idx][y_idx] = true;
    }
    init_model();
}

void InteractiveMaze::change_reward(int x_idx, int y_idx, double delta_reward)
{
    double r = rewards_double[x_idx][y_idx] + delta_reward;
    rewards_double[x_idx][y_idx] = r;
    rewards[x_idx][y_idx]->setPen(r>0?positive_reward_pen:negativ_reward_pen);
    r *= reward_scale;
    rewards[x_idx][y_idx]->setRect(QRectF(field_size*x_idx-r/2,field_size*y_idx-r/2,r,r));
    init_model();
}

void InteractiveMaze::floating_index_and_relative_coords(double x_in, double y_in, double &x_idx, double &y_idx, double &x_rel, double &y_rel) const
{
    x_idx = x_in/(double)field_size;
    y_idx = y_in/(double)field_size;
    x_rel = fabs(x_idx-round(x_idx));
    y_rel = fabs(y_idx-round(y_idx));
}

void InteractiveMaze::execute_action(InteractiveMaze::ACTION a)
{
    arma::vec tran_prob = p.row(state_action_idx(agent_x_pos,agent_y_pos,a)).t();
    double prob = drand48();
    for(int x=0; x<x_dim&&prob>=0; ++x) {
        for(int y=0; y<y_dim&&prob>=0; ++y) {
            prob -= tran_prob(state_idx(x,y));
            if(prob<0) {
                agent_x_pos = x;
                agent_y_pos = y;
            }
        }
    }
    if(prob>=0) {
        tran_prob.print("probs:");
    }
}

void InteractiveMaze::random_changed(int val, bool init)
{
    ui->_random_value->setText(QString("%1%").arg(val));
    epsilon = (double)val/100;
    if(init) {
        init_model();
    }
}

void InteractiveMaze::agent_check_box_changed(bool val, bool init)
{
    move_agent_automatically = val;
    ui->_agent_speed->setEnabled(val);
    if(init) {
        init_timers();
    }
}

void InteractiveMaze::discount_changed(int val, bool init)
{
    ui->_discount_value->setText(QString("%1%").arg(val));
    discount = 1. - (double)val/100;
    if(init) {
        init_model();
    }
}

void InteractiveMaze::x_dim_changed(int x, bool init)
{
    x_dim = x;
    if(init) {
        init_maze();
        init_model();
    }
}

void InteractiveMaze::y_dim_changed(int y, bool init)
{
    y_dim = y;
    if(init) {
        init_maze();
        init_model();
    }
}

void InteractiveMaze::right_click(double x_in, double y_in)
{
    set_wall_by_index(x_in,y_in,false);
}

void InteractiveMaze::left_click(double x_in, double y_in)
{
    set_wall_by_index(x_in,y_in,true);
}

void InteractiveMaze::scroll_up(double x_in, double y_in)
{
    double x_idx, y_idx, x_rel, y_rel;
    floating_index_and_relative_coords(x_in,y_in,x_idx,y_idx,x_rel,y_rel);
    double x = round(x_idx);
    double y = round(y_idx);
    if(x>=0&&x<x_dim&&y>=0&&y<y_dim) {
        change_reward(x,y,1);
    }
}

void InteractiveMaze::scroll_down(double x_in, double y_in)
{
    double x_idx, y_idx, x_rel, y_rel;
    floating_index_and_relative_coords(x_in,y_in,x_idx,y_idx,x_rel,y_rel);
    double x = round(x_idx);
    double y = round(y_idx);
    if(x>=0&&x<x_dim&&y>=0&&y<y_dim) {
        change_reward(x,y,-1);
    }
}

void InteractiveMaze::north_key()
{
    execute_action(NORTH);
}

void InteractiveMaze::east_key()
{
    execute_action(EAST);
}

void InteractiveMaze::south_key()
{
    execute_action(SOUTH);
}

void InteractiveMaze::west_key()
{
    execute_action(WEST);
}

void InteractiveMaze::stay_key()
{
    execute_action(STAY);
}

void InteractiveMaze::init_maze()
{
    // get dimensions
    x_dim = ui->_horizontal_size->value();
    y_dim = ui->_vertical_size->value();
    // clear old scene
    ui->_graphics_view->scene()->clear();
    // fields
    fields.assign(x_dim,vector<QGraphicsRectItem*>(y_dim,nullptr));
    rewards.assign(x_dim,vector<QGraphicsEllipseItem*>(y_dim,nullptr));
    rewards_double.assign(x_dim,vector<double>(y_dim,0));
    h_walls.assign(x_dim,vector<QGraphicsLineItem*>(y_dim,nullptr));
    h_walls_bool.assign(x_dim,vector<bool>(y_dim,false));
    v_walls.assign(x_dim,vector<QGraphicsLineItem*>(y_dim,nullptr));
    v_walls_bool.assign(x_dim,vector<bool>(y_dim,false));
    // fields
    double eff_size = field_size*field_down_scale;
    for(int x=0; x<x_dim; ++x) {
        for(int y=0; y<y_dim; ++y) {
            fields[x][y] = ui->_graphics_view->scene()->addRect(x*field_size-eff_size/2,
                                                                y*field_size-eff_size/2,
                                                                eff_size,
                                                                eff_size);
        }
    }
    // walls
    for(int x=0; x<x_dim; ++x) {
        for(int y=0; y<y_dim; ++y) {
            h_walls[x][y] = ui->_graphics_view->scene()->addLine(x*field_size-field_size/2,
                                                                 y*field_size-field_size/2,
                                                                 x*field_size+field_size/2,
                                                                 y*field_size-field_size/2,
                                                                 wall_off_pen);
            v_walls[x][y] = ui->_graphics_view->scene()->addLine(x*field_size-field_size/2,
                                                                 y*field_size-field_size/2,
                                                                 x*field_size-field_size/2,
                                                                 y*field_size+field_size/2,
                                                                 wall_off_pen);
        }
    }
    // agent
    agent = ui->_graphics_view->scene()->addEllipse(-agent_size/2,-agent_size/2,agent_size,agent_size,QPen(Qt::transparent),QBrush(Qt::black));
    agent_x_pos = 0;
    agent_y_pos = 0;
    // rewards
    for(int x=0; x<x_dim; ++x) {
        for(int y=0; y<y_dim; ++y) {
            rewards[x][y] = ui->_graphics_view->scene()->addEllipse(x*field_size,
                                                                    y*field_size,
                                                                    0,
                                                                    0);
        }
    }
    // initialize model
    init_model();
}

void InteractiveMaze::animate()
{
    // move agent
    agent->setPos((1-alpha)*agent->pos()+alpha*QPointF(agent_x_pos*field_size,agent_y_pos*field_size));
    // adjust display size
    QGraphicsView * view = ui->_graphics_view;
    QGraphicsScene * scene = view->scene();
    scene->setSceneRect(scene->itemsBoundingRect());
    view->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
    view->scale(0.95,0.95);
}
