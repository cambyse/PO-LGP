/********************************************************************************
** Form generated from reading UI file 'testmaze.ui'
**
** Created: Thu Jun 21 15:30:26 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TESTMAZE_H
#define UI_TESTMAZE_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TestMazeClass
{
public:

    void setupUi(QWidget *TestMazeClass)
    {
        if (TestMazeClass->objectName().isEmpty())
            TestMazeClass->setObjectName(QString::fromUtf8("TestMazeClass"));
        TestMazeClass->resize(400, 300);

        retranslateUi(TestMazeClass);

        QMetaObject::connectSlotsByName(TestMazeClass);
    } // setupUi

    void retranslateUi(QWidget *TestMazeClass)
    {
        TestMazeClass->setWindowTitle(QApplication::translate("TestMazeClass", "TestMaze", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class TestMazeClass: public Ui_TestMazeClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TESTMAZE_H
