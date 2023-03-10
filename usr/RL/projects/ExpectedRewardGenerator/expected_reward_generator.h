#ifndef EXPECTED_REWARD_GENERATOR_H
#define EXPECTED_REWARD_GENERATOR_H

#include <QtGui/QWidget>

#include <vector>

#include "ui_expected_reward_generator.h"

class ExpectedRewardGenerator : public QWidget
{
    Q_OBJECT

public:
    ExpectedRewardGenerator(QWidget *parent = 0);
    ~ExpectedRewardGenerator();

private:
    Ui::ExpectedRewardGeneratorClass ui;

    double get_value(const std::vector<bool>& rewards, const double& minR, const double& maxR, const double& discount);
    void add_density(std::vector<double>& p_values, const double& value, const double& discount, const int& depth, const double& minR, const double& maxR, const double& prob);

private slots:
    void minR_changed(int);
    void maxR_changed(int);
    void maxR_P_changed(int);
    void resolution_changed(int);
    void depth_changed(int);
    void discount_changed(int);
    void plot();


};

#endif // EXPECTED_REWARD_GENERATOR_H
