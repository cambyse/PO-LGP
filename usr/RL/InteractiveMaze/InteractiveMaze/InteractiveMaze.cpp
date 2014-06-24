#include "InteractiveMaze.h"
#include "ui_InteractiveMaze.h"

#include <algorithm> // for std::max
#include <iostream>
#include <assert.h>
#include <float.h>
#include <math.h>

using namespace std;

static double alpha = 1e-1;             ///< Fraction of convergence per time step in animation.
static double field_size = 50;          ///< Nominal size of maze fields.
static double field_down_scale = 0.95;  ///< Factor to scale nominal size to et effective size.
static double wall_activation = 0.25;   ///< Percentage of a field that activates a wall.
static double reward_scale = 10;        ///< Factor to scale reward-discs with.
static double wall_width = 6;           ///< Line width of walls.
static double agent_size = 20;          ///< Size of the agent.
static QPen wall_off_pen(QBrush(Qt::transparent),wall_width,Qt::SolidLine,Qt::RoundCap);
static QPen wall_on_pen(QBrush(Qt::black),wall_width,Qt::SolidLine,Qt::RoundCap);

InteractiveMaze::InteractiveMaze(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::InteractiveMaze),
    animation_timer(new QTimer(this)),
    value_timer(new QTimer(this)),
    agent_timer(new QTimer(this)),
    mouse_filter(new MouseFilter())
{
    ui->setupUi(this);
    // mouse filter
    ui->_graphics_view->setScene(new QGraphicsScene());
    ui->_graphics_view->setRenderHint(QPainter::Antialiasing);
    ui->_graphics_view->setMouseTracking(true);
    ui->_graphics_view->scene()->installEventFilter(mouse_filter);
    connect(mouse_filter,SIGNAL(right_click(double,double)),this,SLOT(right_click(double,double)));
    connect(mouse_filter,SIGNAL(left_click(double,double)),this,SLOT(left_click(double,double)));
    connect(mouse_filter,SIGNAL(scroll_up(double,double)),this,SLOT(scroll_up(double,double)));
    connect(mouse_filter,SIGNAL(scroll_down(double,double)),this,SLOT(scroll_down(double,double)));
//    connect(mouse_filter,SIGNAL(right_mouse_move(double,double)),this,SLOT(right_mouse_move(double,double)));
//    connect(mouse_filter,SIGNAL(left_mouse_move(double,double)),this,SLOT(left_mouse_move(double,double)));
    // maze
    init_maze();
    // start timers
    init_timers();
}

InteractiveMaze::~InteractiveMaze()
{
    delete ui;
    delete animation_timer;
    delete mouse_filter;
}

double InteractiveMaze::value_update()
{
    // update Q
    Q = p * (R + discount*V);
    // update V
    double max_diff = 0;
    for(int x=0; x<x_dim; ++x) {
        for(int y=0; y<y_dim; ++y) {
            double old_value = V(state_idx(x,y));
            V(state_idx(x,y)) = 0;
            for(int action_idx = 0; action_idx<ACTION_N; ++action_idx) {
                V(state_idx(x,y)) = max(V(state_idx(x,y)),Q(state_action_idx(x,y,action_idx)));
            }
            fields[x][y]->setBrush(color_from_value(V(state_idx(x,y))));
            max_diff = max(max_diff,fabs(old_value-V(state_idx(x,y))));
        }
    }
    return max_diff;
}

void InteractiveMaze::move_agent()
{
    ACTION a = get_action(agent_x_pos,agent_y_pos);
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

void InteractiveMaze::init_model()
{
    Q.zeros(ACTION_N*x_dim*y_dim);
    R.zeros(x_dim*y_dim);
    V.zeros(x_dim*y_dim);
    p.zeros(ACTION_N*x_dim*y_dim,x_dim*y_dim);
    for(int x1=0; x1<x_dim; ++x1) {
        for(int y1=0; y1<y_dim; ++y1) {
            R(state_idx(x1,y1)) = rewards_double[x1][y1];
            for(ACTION action = (ACTION)0; action<ACTION_N; action=(ACTION)(action+1)) {
                double prob = 0;
                for(int x2=0; x2<x_dim; ++x2) {
                    for(int y2=0; y2<y_dim; ++y2) {
                        double& this_p = p(state_action_idx(x1,y1,action),state_idx(x2,y2));
                        switch(action) {
                        case NORTH:
                            if(x2==x1 && y2==y1-1 && !h_walls_bool[x1][y1]) {
                                this_p = 1;
                            } else if(x2==x1 && y2==y1 && (y1==0 || h_walls_bool[x1][y1])) {
                                this_p = 1;
                            }
                            break;
                        case SOUTH:
                            if(x2==x1 && y2==y1+1 && !h_walls_bool[x1][y1+1]) {
                                this_p = 1;
                            } else if(x2==x1 && y2==y1 && (y1==y_dim-1 || h_walls_bool[x1][y1+1])) {
                                this_p = 1;
                            }
                            break;
                        case WEST:
                            if(x2==x1-1 && y2==y1 && !v_walls_bool[x1][y1]) {
                                this_p = 1;
                            } else if(x2==x1 && y2==y1 && (x1==0 || v_walls_bool[x1][y1])) {
                                this_p = 1;
                            }
                            break;
                        case EST:
                            if(x2==x1+1 && y2==y1 && !v_walls_bool[x1+1][y1]) {
                                this_p = 1;
                            } else if(x2==x1 && y2==y1 && (x1==x_dim-1 || v_walls_bool[x1+1][y1])) {
                                this_p = 1;
                            }
                            break;
                        case STAY:
                            if(x2==x1 && y2==y1) {
                                this_p = 1;
                            }
                            break;
                        default:
                            break;
                        }
                        prob += this_p;
                    }
                }
                if(prob!=1) {
                    cout << "prob (" << x1 << "," << y1 << ") ";
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
                    case EST:
                        cout << "EST";
                        break;
                    case STAY:
                        cout << "STAY";
                        break;
                    default:
                        break;
                    }
                    cout << " (" << prob << ")" << endl;
                }
            }
        }
    }
}

void InteractiveMaze::init_timers()
{
    // animation
    connect(animation_timer, SIGNAL(timeout()), this, SLOT(animate()));
    animation_timer->start(ui->_animation_speed->value());
    // value iteration
    connect(value_timer, SIGNAL(timeout()), this, SLOT(value_update()));
    value_timer->start(ui->_value_update_speed->value());
    // agent
    connect(agent_timer, SIGNAL(timeout()), this, SLOT(move_agent()));
    agent_timer->start(ui->_agent_speed->value());
}

QColor InteractiveMaze::color_from_value(double v) const
{
    double e_plus = exp(v/ui->_value_scale->value());
    double e_minus = 1./e_plus;
    double rescaled = (e_plus-e_minus)/(e_plus+e_minus);
    if(rescaled>0) {
        return QColor(255,255*(1-rescaled),255*(1-rescaled));
    } else {
        return QColor(255*(1+rescaled),255,255);
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
    r *= reward_scale;
    rewards[x_idx][y_idx]->setRect(QRectF(field_size*x_idx-r/2,field_size*y_idx-r/2,r,r));
    init_model();
}

void InteractiveMaze::floating_index_and_relative_coords(double x_in, double y_in, double &x_idx, double &y_idx, double &x_rel, double &y_rel) const
{
    x_idx = x_in;
    y_idx = y_in;
    x_idx /= field_size;
    y_idx /= field_size;
    x_rel = x_idx-round(x_idx);
    y_rel = y_idx-round(y_idx);
}

void InteractiveMaze::right_click(double x_in, double y_in)
{

}

void InteractiveMaze::left_click(double x_in, double y_in)
{
    cout << "left click" << endl;
    double x_idx, y_idx, x_rel, y_rel;
    floating_index_and_relative_coords(x_in,y_in,x_idx,y_idx,x_rel,y_rel);
    if(fabs(x_rel)>wall_activation && fabs(y_rel)<wall_activation) {
        int x = ceil(x_idx);
        int y = round(y_idx);
        if(x>0&&x<x_dim-1&&y>=0&&y<y_dim-1) {
        set_v_wall(x,y,true);
        }
    }
}

void InteractiveMaze::scroll_up(double x_in, double y_in)
{
}

void InteractiveMaze::scroll_down(double x_in, double y_in)
{
}

//void InteractiveMaze::right_mouse_move(double x_in, double y_in)
//{
//}

//void InteractiveMaze::left_mouse_move(double x_in, double y_in)
//{
//    const double x_idx = x_in/field_size;
//    const double y_idx = y_in/field_size;
//    const double x_rel = x_idx-round(x_idx);
//    const double y_rel = y_idx-round(y_idx);
//    //if(fabs(x_rel)>wall_activation && fabs(y_rel)>wall_activation) return;
//    //if(fabs(x_rel)<wall_activation && fabs(y_rel)<wall_activation) return;
//    //point->setPos(ceil(x_idx)*field_size,round(y_idx)*field_size);
//    if(drand48()<wall_activation && drand48()>wall_activation) {
//        point->setPos(x_idx*field_size,y_idx*field_size);
//    }
//}

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
    double eff_size = field_size*field_down_scale;
    for(int x=0; x<x_dim; ++x) {
        for(int y=0; y<y_dim; ++y) {
            fields[x][y] = ui->_graphics_view->scene()->addRect(x*field_size-eff_size/2,
                                                                y*field_size-eff_size/2,
                                                                eff_size,
                                                                eff_size);
        }
    }
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
    for(int x=0; x<x_dim; ++x) {
        for(int y=0; y<y_dim; ++y) {
            rewards[x][y] = ui->_graphics_view->scene()->addEllipse(x*field_size,
                                                                    y*field_size,
                                                                    0,
                                                                    0);
        }
    }
    // agent
    agent = ui->_graphics_view->scene()->addEllipse(-agent_size/2,-agent_size/2,agent_size,agent_size,QPen(Qt::transparent),QBrush(Qt::black));
    point = ui->_graphics_view->scene()->addEllipse(-agent_size/2,-agent_size/2,agent_size,agent_size,QPen(Qt::red),QBrush(Qt::black));
    agent_x_pos = 0;
    agent_y_pos = 0;
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
