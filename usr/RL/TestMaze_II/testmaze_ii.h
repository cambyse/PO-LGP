#ifndef TESTMAZE_II_H
#define TESTMAZE_II_H

#include "ui_testmaze_ii.h"
#include "Maze.h"
#include "KMarkovCRF.h"
#include "UTree.h"
#include "LookAheadSearch.h"
#include "Data.h"
#include "Representation/Representation.h"

#include <QWidget>
#include <QTimer>

#include <deque>
#include <iostream>
#include <fstream>

class TestMaze_II : public QWidget
{
    Q_OBJECT

public:
    TestMaze_II(QWidget *parent = 0);
    ~TestMaze_II();

private:

    friend class MoveByKeys; // event filter class

    //===============================//
    // Typedefs and Member Variables //
    //===============================//

    typedef Data::size_t        size_t;
    typedef Data::probability_t probability_t;

    //---------------//
    // Maze GUI etc. //
    //---------------//

    enum PLANNER_TYPE {
        NONE,
        OPTIMAL_PLANNER,
        SPARSE_PLANNER,
        KMDP_PLANNER,
        UTREE_PLANNER
    } planner_type;

    Ui::TestMaze_IIClass ui;
    Maze maze;

    bool record, plot;
    instance_t * current_instance;
    std::ofstream plot_file;

    QTimer * random_timer, * action_timer;

    std::vector<QString> console_history;
    size_t history_position;

    //--------//
    // Models //
    //--------//

    // CRF
    KMarkovCRF crf;
    double l1_factor;

    // UTree
    UTree utree;

    //----------//
    // Planners //
    //----------//

    double discount;
    LookAheadSearch look_ahead_search;
    size_t max_tree_size;

    //==================//
    // Member Functions //
    //==================//

    void collect_episode(const int& length);
    void update_current_instance(action_t,state_t,reward_t);


private slots:
    void render();
    void random_action();
    void choose_action();
    void process_console_input(QString sequence_input = QString(), bool sequence = false);
    void back_in_history();
    void forward_in_history();

};

// event filter for arrow keys
class MoveByKeys: public QObject {
    Q_OBJECT
public:
    MoveByKeys(TestMaze_II * m): maze(m) {}
protected:
    TestMaze_II * maze;
    bool eventFilter(QObject *obj, QEvent *event);
};

#endif // TESTMAZE_II_H
