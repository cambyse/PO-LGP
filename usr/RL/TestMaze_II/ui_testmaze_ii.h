/********************************************************************************
** Form generated from reading UI file 'testmaze_ii.ui'
**
** Created: Wed Oct 10 16:20:03 2012
**      by: Qt User Interface Compiler version 4.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TESTMAZE_II_H
#define UI_TESTMAZE_II_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGraphicsView>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TestMaze_IIClass
{
public:
    QGridLayout *gridLayout;
    QGraphicsView *graphicsView;

    void setupUi(QWidget *TestMaze_IIClass)
    {
        if (TestMaze_IIClass->objectName().isEmpty())
            TestMaze_IIClass->setObjectName(QString::fromUtf8("TestMaze_IIClass"));
        TestMaze_IIClass->resize(513, 417);
        gridLayout = new QGridLayout(TestMaze_IIClass);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        graphicsView = new QGraphicsView(TestMaze_IIClass);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));

        gridLayout->addWidget(graphicsView, 0, 0, 1, 1);


        retranslateUi(TestMaze_IIClass);

        QMetaObject::connectSlotsByName(TestMaze_IIClass);
    } // setupUi

    void retranslateUi(QWidget *TestMaze_IIClass)
    {
        TestMaze_IIClass->setWindowTitle(QApplication::translate("TestMaze_IIClass", "TestMaze_II", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class TestMaze_IIClass: public Ui_TestMaze_IIClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TESTMAZE_II_H
