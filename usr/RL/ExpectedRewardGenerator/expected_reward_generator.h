#ifndef EXPECTED_REWARD_GENERATOR_H
#define EXPECTED_REWARD_GENERATOR_H

#include <QtGui/QWidget>
#include "ui_expected_reward_generator.h"

class ExpectedRewardGenerator : public QWidget
{
    Q_OBJECT

public:
    ExpectedRewardGenerator(QWidget *parent = 0);
    ~ExpectedRewardGenerator();

private:
    Ui::ExpectedRewardGeneratorClass ui;

private slots:
    void minR_changed(int);
    void maxR_changed(int);
    void minR_P_changed(int);
    void maxR_P_changed(int);
    void resolution_changed(int);
    void depth_changed(int);
    void plot();


};

#endif // EXPECTED_REWARD_GENERATOR_H
