/********************************************************************************
** Form generated from reading UI file 'testmaze.ui'
**
** Created: Thu Jun 28 15:25:22 2012
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
#include <QtGui/QGraphicsView>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TestMazeClass
{
public:
    QGridLayout *gridLayout;
    QGraphicsView *_wDisplay;

    void setupUi(QWidget *TestMazeClass)
    {
        if (TestMazeClass->objectName().isEmpty())
            TestMazeClass->setObjectName(QString::fromUtf8("TestMazeClass"));
        TestMazeClass->resize(461, 419);
        gridLayout = new QGridLayout(TestMazeClass);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        _wDisplay = new QGraphicsView(TestMazeClass);
        _wDisplay->setObjectName(QString::fromUtf8("_wDisplay"));

        gridLayout->addWidget(_wDisplay, 0, 0, 1, 1);


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
