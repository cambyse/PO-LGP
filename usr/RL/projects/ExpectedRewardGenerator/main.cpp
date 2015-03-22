#include "expected_reward_generator.h"

#include <QtGui>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ExpectedRewardGenerator w;
    w.show();
    return a.exec();
}
