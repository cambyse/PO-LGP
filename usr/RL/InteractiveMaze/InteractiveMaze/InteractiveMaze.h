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

#include "MouseFilter.h"

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
    enum ACTION { NORTH, EST, SOUTH, WEST, STAY, ACTION_N };

    Ui::InteractiveMaze *ui;
    QTimer *animation_timer, *value_timer, *agent_timer;
    MouseFilter * mouse_filter;
    int x_dim, y_dim;
    std::vector<std::vector<QGraphicsRectItem*> > fields;
    std::vector<std::vector<QGraphicsEllipseItem*> > rewards;
    std::vector<std::vector<double> > rewards_double;
    std::vector<std::vector<QGraphicsLineItem*> > h_walls;
    std::vector<std::vector<QGraphicsLineItem*> > v_walls;
    std::vector<std::vector<bool> > h_walls_bool;
    std::vector<std::vector<bool> > v_walls_bool;
    QGraphicsEllipseItem *agent, *point;
    int agent_x_pos, agent_y_pos;
    double discount = 0.9;
    arma::vec Q, R, V;
    arma::mat p;

    void init_model();
    void init_timers();
    int state_idx(int x, int y) const { return y*x_dim+x; }
    int state_action_idx(int x, int y, int a) const { return state_idx(x,y)*ACTION_N+a; }
    QColor color_from_value(double) const;
    ACTION get_action(int x, int y) const;
    void set_h_wall(int x_idx, int y_idx, bool set);
    void set_v_wall(int x_idx, int y_idx, bool set);
    void change_reward(int x_idx, int y_idx, double delta_reward);
    void floating_index_and_relative_coords(double x_in, double y_in, double & x_idx, double & y_idx, double & x_rel, double & y_rel) const;

private slots:
    void right_click(double x_in, double y_in);
    void left_click(double x_in, double y_in);
    void scroll_up(double x_in, double y_in);
    void scroll_down(double x_in, double y_in);
//    void right_mouse_move(double x_in, double y_in);
//    void left_mouse_move(double x_in, double y_in);
    void init_maze();
    void animate();
    double value_update();
    void move_agent();
};

#endif // INTERACTIVEMAZE_H
