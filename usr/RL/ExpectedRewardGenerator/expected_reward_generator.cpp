#include "expected_reward_generator.h"

ExpectedRewardGenerator::ExpectedRewardGenerator(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);
}

ExpectedRewardGenerator::~ExpectedRewardGenerator()
{

}

void ExpectedRewardGenerator::minR_changed(int i) {
    double value = i - ui._wMinR->minimum();
    value /= ui._wMinR->maximum()-ui._wMinR->minimum();
    ui._wMinRValue->setText(QString::number(value));
}

void ExpectedRewardGenerator::maxR_changed(int i) {
    double value = i - ui._wMaxR->minimum();
    value /= ui._wMaxR->maximum()-ui._wMaxR->minimum();
    ui._wMaxRValue->setText(QString::number(value));
}

void ExpectedRewardGenerator::minR_P_changed(int i) {
    double value = i - ui._wPMin->minimum();
    value /= ui._wPMin->maximum()-ui._wPMin->minimum();
    ui._wPMinValue->setText(QString::number(value));
}

void ExpectedRewardGenerator::maxR_P_changed(int i) {
    double value = i - ui._wPMax->minimum();
    value /= ui._wPMax->maximum()-ui._wPMax->minimum();
    ui._wPMaxValue->setText(QString::number(value));
}

void ExpectedRewardGenerator::resolution_changed(int i) {
    ui._wResolutionValue->setText(QString::number(i));
}

void ExpectedRewardGenerator::depth_changed(int i) {
    ui._wDepthValue->setText(QString::number(i));
}

void ExpectedRewardGenerator::plot() {

}
