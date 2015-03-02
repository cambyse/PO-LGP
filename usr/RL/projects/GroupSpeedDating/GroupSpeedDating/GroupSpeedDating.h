#ifndef GROUPSPEEDDATING_H
#define GROUPSPEEDDATING_H

#include <QMainWindow>
#include <QTimer>

//#define ARMA_NO_DEBUG
#include <util/ND_vector.h>
using namespace ND_vector;

namespace Ui {
class GroupSpeedDating;
}

class GroupSpeedDating : public QMainWindow
{
    Q_OBJECT

    // TYPES //
    // none...//

    // DATA MEMBERS //

private:
    Ui::GroupSpeedDating *ui;
    QTimer * permute_timer;
    int groupN, memberN, tableSize, tableN, roundN, peopleN;
    double score, max_score, temperature;
    vec_int_3D round_table_seat;
    vec_double_2D weights;

    // METHODS //

public:
    explicit GroupSpeedDating(QWidget *parent = 0);
    ~GroupSpeedDating();

private:
    int objective(const vec_int_3D talk_table_group) const;
    void permute(vec_int_3D & talk_table_group) const;
    double compute_score() const;
    void swap(int round, int table_1, int seat_1, int table_2, int seat_2);
private slots:
    void try_permutation();
    void update_table();
    void temperature_changed(int);
    void weight_changed(int,int);
    void name_changed(int,int);
    void start_stop(bool);
};

#endif // GROUPSPEEDDATING_H
