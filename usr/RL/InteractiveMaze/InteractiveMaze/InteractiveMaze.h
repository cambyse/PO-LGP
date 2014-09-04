#ifndef INTERACTIVEMAZE_H
#define INTERACTIVEMAZE_H

#include <QMainWindow>
#include <QTimer>
#include <QGraphicsEllipseItem>
#include <QGraphicsRectItem>
#include <QGraphicsLineItem>

#include <vector>

//#define ARMA_NO_DEBUG
#include <armadillo>

#include "InputFilter.h"

namespace Ui {
class InteractiveMaze;
}

class InteractiveMaze : public QMainWindow
{
    Q_OBJECT

public:
    explicit InteractiveMaze(QWidget *parent = 0);
    ~InteractiveMaze();

private:
    enum ACTION { NORTH, EAST, SOUTH, WEST, STAY, ACTION_N };

    Ui::InteractiveMaze *ui;
    QTimer *animation_timer, *value_timer, *agent_timer;
    InputFilter * input_filter;
    int x_dim, y_dim;
    std::vector<std::vector<QGraphicsRectItem*> > fields;
    std::vector<std::vector<QGraphicsEllipseItem*> > rewards;
    std::vector<std::vector<double> > rewards_double;
    std::vector<std::vector<QGraphicsLineItem*> > h_walls;
    std::vector<std::vector<QGraphicsLineItem*> > v_walls;
    std::vector<std::vector<bool> > h_walls_bool;
    std::vector<std::vector<bool> > v_walls_bool;
    QGraphicsEllipseItem *agent;
    double alpha;
    bool move_agent_automatically, display_value_plus_reward;
    int agent_x_pos, agent_y_pos;
    double discount, epsilon;
    arma::vec Q, R, V;
    arma::mat p;

    void init_model();
    int state_idx(int x, int y) const { return y*x_dim+x; }
    int state_action_idx(int x, int y, int a) const { return state_idx(x,y)*ACTION_N+a; }
    QColor color_from_value(double val, double min_val, double max_val) const;
    ACTION get_action(int x, int y) const;
    void set_wall_by_index(int x_in, int y_in, bool set);
    void set_h_wall(int x_idx, int y_idx, bool set);
    void set_v_wall(int x_idx, int y_idx, bool set);
    void change_reward(int x_idx, int y_idx, double delta_reward);
    void floating_index_and_relative_coords(double x_in, double y_in, double & x_idx, double & y_idx, double & x_rel, double & y_rel) const;
    void execute_action(ACTION);

private slots:
    void display_value_plus_reward_changed(bool);
    void random_changed(int, bool init = true);
    void agent_check_box_changed(bool, bool init = true);
    void discount_changed(int, bool init = true);
    void x_dim_changed(int x, bool init = true);
    void y_dim_changed(int y, bool init = true);
    void right_click(double x_in, double y_in);
    void left_click(double x_in, double y_in);
    void scroll_up(double x_in, double y_in);
    void scroll_down(double x_in, double y_in);
    void north_key();
    void east_key();
    void south_key();
    void west_key();
    void stay_key();
    void init_timers();
    void init_maze();
    void animate();
    double value_update();
    void move_agent();
};

#endif // INTERACTIVEMAZE_H
