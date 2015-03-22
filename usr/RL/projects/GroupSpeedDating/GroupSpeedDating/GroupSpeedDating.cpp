#include "GroupSpeedDating.h"
#include "ui_GroupSpeedDating.h"

#include <time.h>
#include <vector>
#include <set>
#include <tuple>

#include <util/util.h>
#include <util/QtUtil.h>
#include <util/pretty_printer.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using std::vector;
using std::set;
using std::tuple;
using std::make_tuple;
using std::cout;
using std::endl;
using util::Range;
using util::enumerate;

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
    messages_toggled(ui->_wMessageButton->isChecked());

    // seed random generators
    srand(time(nullptr));
    srand48(time(nullptr));
}

GroupSpeedDating::~GroupSpeedDating()
{
    delete ui;
    delete permute_timer;
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
    vector<set<int>> old_partners(peopleN);
    for(auto round : enumerate(round_table_seat)) {
        for(auto table : enumerate(round.second)) {
            for(auto seat_1 : enumerate(table.second)) {
                for(auto seat_2 : enumerate(table.second)) {
                    if(seat_1.second>=0 && seat_2.second>=0) {
                        // weight
                        double weight = weights[seat_1.second][seat_2.second];
                        // check for old dates
                        auto& old_partner_list = old_partners[seat_1.second];
                        auto it = old_partner_list.find(seat_2.second);
                        if(it==old_partner_list.end()) {
                            old_partner_list.insert(seat_2.second);
                        } else {
                            weight = 0;
                        }
                        DEBUG_OUT(2, "Weight for: Round=" << round.first <<
                                  ", Table=" << table.first <<
                                  ", Seat/Name=" << seat_1.first << "/" << ui->_wTableNames->item(0, seat_1.second)->text() <<
                                  "<-->" << seat_2.first << "/" << ui->_wTableNames->item(0, seat_2.second)->text() << " : " << weight);
                        score += weight;
                    }
                }
            }
        }
    }
    DEBUG_OUT(1, "Score = " << score);
    return score;
}

void GroupSpeedDating::swap(int round, int table_1, int seat_1, int table_2, int seat_2) {
    int tmp = round_table_seat[round][table_1][seat_1];
    round_table_seat[round][table_1][seat_1] = round_table_seat[round][table_2][seat_2];
    round_table_seat[round][table_2][seat_2] = tmp;
}

void GroupSpeedDating::print_weights() const {
    cout << "Weights..." << endl;
    for(int idx_1 : Range(peopleN)) {
        cout << "    ";
        for(int idx_2 : Range(peopleN)) {
            cout << weights[idx_1][idx_2] << " ";
        }
        cout << endl;
    }
}

void GroupSpeedDating::print_dates() const {
    cout << "Dates..." << endl;
    for(auto round : enumerate(round_table_seat)) {
        cout << "    Round " << round.first << endl;
        for(auto table : enumerate(round.second)) {
            cout << "        Table " << table.first << endl;
            cout << "            ";
            for(auto seat : enumerate(table.second)) {
                cout << seat.second << " ";
            }
            cout << endl;
        }
    }
}

void GroupSpeedDating::remember_schedule() {
    DEBUG_OUT(1,"Remember Schedule");
    IF_DEBUG(2) {
        print_dates();
    }
    ui->_wScore->setText(QString("%1/%2").arg(score).arg(score/peopleN));
    for(auto round : enumerate(round_table_seat)) {
        for(auto table : enumerate(round.second)) {
            QString names;
            bool first = true;
            for(auto seat : table.second) {
                if(seat<0) continue;
                if(first) {
                    first = false;
                } else {
                    names += " & ";
                }
                names += ui->_wTableNames->item(0,seat)->text();
            }
            ui->_wTableDates->item(round.first, table.first)->setText(names);
        }
    }
}

void GroupSpeedDating::messages_toggled(bool m) {
    messages_on = m;
}

void GroupSpeedDating::clear_dates() {
    DEBUG_OUT(1,"Clear Dates");
    score = compute_score();
    max_score = score;
}

void GroupSpeedDating::try_permutation() {

    int round = rand()%roundN;
    int table_1 = rand()%tableN;
    int table_2 = rand()%tableN;
    int seat_1 = rand()%tableSize;
    int seat_2 = rand()%tableSize;

    IF_DEBUG(1) {
        QString name_1 = "N/N";
        if(round_table_seat[round][table_1][seat_1]>=0) {
            name_1 = ui->_wTableNames->item(0, round_table_seat[round][table_1][seat_1])->text();
        }
        QString name_2 = "N/N";
        if(round_table_seat[round][table_2][seat_2]>=0) {
            name_2 = ui->_wTableNames->item(0, round_table_seat[round][table_2][seat_2])->text();
        }
        cout << "Try swapping:" << endl;
        cout << "    Round " << round << endl;
        cout << "    Table/Seat/Name " << endl;
        cout << "    " << table_1 << "/" << seat_1 << "/" << name_1 << endl;
        cout << "    " << table_2 << "/" << seat_2 << "/" << name_2 << endl;
    }

    swap(round, table_1, seat_1, table_2, seat_2);
    double new_score = compute_score();

    // keep / don't keep
    bool keep = false;
    if(new_score>=score) keep = true;
    else if(temperature==0) keep = false;
    else if(drand48()<exp((new_score-score)/temperature)) keep = true;

    if(keep) {
        score = new_score;
    } else {
        swap(round, table_1, seat_1, table_2, seat_2);
    }

    if(score>=max_score) {
        max_score = score;
        remember_schedule();
    }

    if(messages_on) {
        ui->_wMessages->setText(QString("%1/%2 (%3)").arg(score).arg(score/peopleN).arg(keep?"KEEP":"DROP"));
    }
}

void GroupSpeedDating::update_table() {
    // update numbers
    int oldPeopleN = peopleN;
    int oldTableSize = tableSize;
    int oldTableN = tableN;
    int oldRoundN = roundN;
    groupN = ui->_wGroupN->value();
    memberN = ui->_wMemberN->value();
    peopleN = groupN*memberN;
    tableSize = ui->_wTableN->value();
    tableN = ceil((double)(peopleN)/tableSize);
    roundN = ui->_wRoundN->value();

    if(oldPeopleN!=peopleN) {
        // resize name and weight tables
        ui->_wTableWeights->setColumnCount(peopleN);
        ui->_wTableWeights->setRowCount(peopleN);
        ui->_wTableNames->setColumnCount(peopleN);
        ui->_wTableNames->setRowCount(1);
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
        // resize weights and set to one/zero for people from different/same group
        weights = make_vec_double_2D(peopleN, peopleN);
        for(int col_idx : Range(peopleN)) {
            for(int row_idx : Range(peopleN)) {
                auto val = QString("%1").arg(row_idx/memberN==col_idx/memberN?0:1);
                auto item = new QTableWidgetItem(val);
                ui->_wTableWeights->setItem(row_idx, col_idx, item);
            }
        }
        IF_DEBUG(2) {
            print_weights();
        }
    }

    // resize table for schedule
    if(oldRoundN!=roundN || oldTableN!=tableN) {
        ui->_wTableDates->setColumnCount(tableN);
        ui->_wTableDates->setRowCount(roundN);
        for(int row_idx : Range(roundN)) {
            for(int col_idx : Range(tableN)) {
                ui->_wTableDates->setItem(row_idx, col_idx, new QTableWidgetItem("N/N"));
            }
        }
    }

    // resize array for schedule
    if(oldRoundN!=roundN || oldTableN!=tableN || oldTableSize!=tableSize || peopleN) {
        round_table_seat = make_vec_int_3D(roundN, tableN, tableSize, -1);
        for(auto& round : round_table_seat) {
            for(int person_idx : Range(peopleN)) {
                round[person_idx/tableSize][person_idx%tableSize] = person_idx;
            }
        }
        IF_DEBUG(2) {
            print_dates();
        }
    }

    // get score
    score = compute_score();
    max_score = score;
}

void GroupSpeedDating::temperature_changed(int val) {
    temperature = (double)val/ui->_wTemperature->maximum()*100;
    ui->_wTemperatureValue->setText(QString("%1").arg(temperature));
}

void GroupSpeedDating::weight_changed(int x, int y) {
    bool ok;
    double weight = ui->_wTableWeights->item(x,y)->text().toDouble(&ok);
    if(!ok) {
        weight = 0;
        ui->_wTableWeights->item(x,y)->setText(QString("0"));
    }
    weights[x][y] = weight;
    IF_DEBUG(2) {
        print_weights();
    }
}

void GroupSpeedDating::name_changed(int x, int y) {
    QString name = ui->_wTableNames->item(x, y)->text();
    QTableWidgetItem *h_header = new QTableWidgetItem();
    h_header->setText(name);
    ui->_wTableWeights->setHorizontalHeaderItem(y, h_header);
    QTableWidgetItem *v_header = new QTableWidgetItem();
    v_header->setText(name);
    ui->_wTableWeights->setVerticalHeaderItem(y, v_header);
}

void GroupSpeedDating::start_stop(bool val) {
    ui->_wStartStop->setText(val?"Stop":"Start");
    if(val) {
        permute_timer->start(0);
    } else {
        permute_timer->stop();
    }
}
