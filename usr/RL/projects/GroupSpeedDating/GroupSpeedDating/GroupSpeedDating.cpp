#include "GroupSpeedDating.h"
#include "ui_GroupSpeedDating.h"

#include <time.h>
#include <vector>
#include <set>
#include <tuple>

#include <util/util.h>
#include <util/QtUtil.h>
#include <util/pretty_printer.h>

#define DEBUG_LEVEL 1
#include <util/debug.h>

using std::vector;
using std::set;
using std::tuple;
using std::make_tuple;
using std::cout;
using std::endl;
using util::Range;

GroupSpeedDating::GroupSpeedDating(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GroupSpeedDating),
    permute_timer(new QTimer(this))
{
    ui->setupUi(this);

    // connect timer for optimization loop
    connect(permute_timer, SIGNAL(timeout()), this, SLOT(try_permutation()));

    // initialize everything
    update_table();
    temperature_changed(ui->_wTemperature->value());

    // seed random generators
    srand(time(nullptr));
    srand48(time(nullptr));
}

GroupSpeedDating::~GroupSpeedDating()
{
    delete ui;
    delete permute_timer;
}

int GroupSpeedDating::objective(const vec_int_3D talk_table_group) const {
    int counts = 0;
    set<tuple<int,int>> partners[groupN][memberN];
    for(int talk_idx : Range(roundN)) {
        for(int table_idx : Range(memberN)) {
            for(int group_idx_from : Range(groupN)) {
                for(int group_idx_to : Range(groupN)) {
                    if(group_idx_from==group_idx_to) continue;
                    partners[group_idx_from][talk_table_group[talk_idx][table_idx][group_idx_from]].insert(make_tuple(group_idx_to, talk_table_group[talk_idx][table_idx][group_idx_to]));
                }
            }
        }
    }
    for(int group_idx : Range(groupN)) {
        for(int member_idx : Range(memberN)) {
            counts += partners[group_idx][member_idx].size();
        }
    }
    return counts;
}

void GroupSpeedDating::permute(vec_int_3D & talk_table_group) const {
    int talk_idx = rand()%roundN;
    int group_idx = rand()%groupN;
    int table_idx_1 = rand()%memberN;
    int table_idx_2 = rand()%memberN;
    int tmp = talk_table_group[talk_idx][table_idx_1][group_idx];
    talk_table_group[talk_idx][table_idx_1][group_idx] = talk_table_group[talk_idx][table_idx_2][group_idx];
    talk_table_group[talk_idx][table_idx_2][group_idx] = tmp;
}

double GroupSpeedDating::compute_score() const {
    double score = 0;
    for(auto round : round_table_seat) {
        for(auto table : round) {
            for(auto seat_1 : table) {
                for(auto seat_2 : table) {
                    if(seat_1>=0 && seat_2>=0) {
                        score += weights[seat_1][seat_2];
                    }
                }
            }
        }
    }
    return score;
}

void GroupSpeedDating::swap(int round, int table_1, int seat_1, int table_2, int seat_2) {
    int tmp = round_table_seat[round][table_1][seat_1];
    round_table_seat[round][table_1][seat_1] = round_table_seat[round][table_2][seat_2];
    round_table_seat[round][table_2][seat_2] = tmp;
}

void GroupSpeedDating::try_permutation() {

    int round = rand()%roundN;
    int table_1 = rand()%tableN;
    int table_2 = rand()%tableN;
    int seat_1 = rand()%tableSize;
    int seat_2 = rand()%tableSize;

    swap(round, table_1, seat_1, table_2, seat_2);
    double new_score = compute_score();

    // keep / don't keep
    bool keep = false;
    if(new_score>=score) keep = true;
    else if(temperature==0) keep = false;
    else if(drand48()<exp((new_score-score)/temperature)) keep = true;

    if(keep) {
        score = new_score;
        max_score = std::max(score,max_score);
    } else {
        swap(round, table_1, seat_1, table_2, seat_2);
    }

    DEBUG_OUT(0, (keep?"KEEP":"DROP") << ": dates = " << score << " / " << score/peopleN);

    return;






    vec_int_3D talk_table_group = make_vec_int_3D(roundN, memberN, groupN);

    for(int talk_idx : Range(roundN)) {
        for(int table_idx : Range(memberN)) {
            for(int group_idx : Range(groupN)) {
                talk_table_group[talk_idx][table_idx][group_idx] = table_idx;
            }
        }
    }

    int objective_value = objective(talk_table_group);
    int max_objective_value = objective(talk_table_group);
    vec_int_3D max_solution = talk_table_group;
    double temperature = 10000;
    repeat(10000) {
        auto copy = talk_table_group;
        permute(talk_table_group);
        int new_val = objective(talk_table_group);
        bool select = true;
        if(new_val<objective_value) {
            select = drand48()<exp(new_val-objective_value);
            if(!select) {
                talk_table_group = copy;
            } else {
                objective_value = new_val;
            }
        } else {
            objective_value = new_val;
        }
        if(objective_value>max_objective_value) {
            max_objective_value = objective_value;
            max_solution = talk_table_group;
        }
        cout << "T = " << temperature << ", Obj = " << objective_value << (select?" (SELECT)":" (DROP)") << " max = " << max_objective_value << endl;
        temperature *= 0.9995;
        if(temperature<1000) temperature = 1000;
    }

    for(int talk_idx : Range(roundN)) {
        cout << "Talk " << talk_idx << endl;
        for(int table_idx : Range(memberN)) {
            cout << "    Table " << table_idx << endl;
            cout << "        ";
            for(int group_idx : Range(groupN)) {
                cout << max_solution[talk_idx][table_idx][group_idx] << " ";
            }
            cout << endl;
        }
    }

    cout << "Informative Talks:" << endl;
    cout << "    Total: " << max_objective_value << " / " << groupN*memberN << endl;
    cout << "     p.p.: " << (double)max_objective_value/(groupN*memberN) << endl;
}

void GroupSpeedDating::update_table() {
    // update numbers
    groupN = ui->_wGroupN->value();
    memberN = ui->_wMemberN->value();
    peopleN = groupN*memberN;
    tableSize = ui->_wTableN->value();
    tableN = ceil((double)(peopleN)/tableSize);
    roundN = ui->_wRoundN->value();

    // resize tables and arrays
    ui->_wTable->setColumnCount(peopleN);
    ui->_wTable->setRowCount(peopleN);
    ui->_wTableNames->setColumnCount(peopleN);
    ui->_wTableNames->setRowCount(1);
    ui->_wTableDates->setColumnCount(tableN);
    ui->_wTableDates->setRowCount(roundN);
    weights = make_vec_double_2D(peopleN, peopleN);
    round_table_seat = make_vec_int_3D(roundN, tableN, tableSize);
    for(auto& table_seat_array : round_table_seat) {
        int count = 0;
        for(auto& seat_array : table_seat_array) {
            for(auto& seat : seat_array) {
                seat = count++;
            }
        }
    }

    // set weights to one/zero for people from different/same group
    for(int col_idx : Range(peopleN)) {
        for(int row_idx : Range(peopleN)) {
            auto item = new QDoubleSpinBox();
            item->setValue(row_idx/memberN==col_idx/memberN?0:1);
            ui->_wTable->setCellWidget(row_idx, col_idx, item);
        }
    }

    // set default names
    int idx = 0;
    DEBUG_OUT(1, "Setting names for " << groupN << " groups with " << memberN << " members");
    for(int group_idx : Range(1,groupN)) {
        for(int member_idx : Range(1,memberN)) {
            auto name = QString("G%1M%2").arg(group_idx).arg(member_idx);
            auto item = new QTableWidgetItem(name);
            DEBUG_OUT(2, "Set name " << idx << " to " << name);
            ui->_wTableNames->setItem(0, idx, item);
            ++idx;
        }
    }

    // get score
    score = compute_score();
    max_score = score;
}

void GroupSpeedDating::temperature_changed(int val) {
    ui->_wTemperatureValue->setText(QString("%1").arg(val));
    temperature = val;
}

void GroupSpeedDating::weight_changed(int x, int y) {
    QDoubleSpinBox * spin_box = static_cast<QDoubleSpinBox*>(ui->_wTable->cellWidget(x, y));
    DEBUG_EXPECT(0,spin_box!=nullptr);
    double weight = spin_box->value();
    weights[x][y] = weight;
    weights[y][x] = weight;
}

void GroupSpeedDating::name_changed(int x, int y) {
    QString name = ui->_wTableNames->item(x, y)->text();
    QTableWidgetItem *h_header = new QTableWidgetItem();
    h_header->setText(name);
    ui->_wTable->setHorizontalHeaderItem(y, h_header);
    QTableWidgetItem *v_header = new QTableWidgetItem();
    v_header->setText(name);
    ui->_wTable->setVerticalHeaderItem(y, v_header);
}

void GroupSpeedDating::start_stop(bool val) {
    ui->_wStartStop->setText(val?"Stop":"Start");
    if(val) {
        permute_timer->start(0);
    } else {
        permute_timer->stop();
    }
}
