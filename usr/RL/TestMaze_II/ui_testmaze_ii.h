/********************************************************************************
** Form generated from reading UI file 'testmaze_ii.ui'
**
** Created: Mon Nov 5 17:42:07 2012
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
#include <QtGui/QDockWidget>
#include <QtGui/QGraphicsView>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLineEdit>
#include <QtGui/QPlainTextEdit>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TestMaze_IIClass
{
public:
    QGridLayout *gridLayout;
    QGraphicsView *graphicsView;
    QDockWidget *_wConsoleWidget;
    QWidget *dockWidgetContents;
    QGridLayout *gridLayout_3;
    QWidget *widget;
    QGridLayout *gridLayout_2;
    QPlainTextEdit *_wConsoleOutput;
    QLineEdit *_wConsoleInput;

    void setupUi(QWidget *TestMaze_IIClass)
    {
        if (TestMaze_IIClass->objectName().isEmpty())
            TestMaze_IIClass->setObjectName(QString::fromUtf8("TestMaze_IIClass"));
        TestMaze_IIClass->resize(1183, 852);
        gridLayout = new QGridLayout(TestMaze_IIClass);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        graphicsView = new QGraphicsView(TestMaze_IIClass);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));

        gridLayout->addWidget(graphicsView, 0, 0, 1, 1);

        _wConsoleWidget = new QDockWidget(TestMaze_IIClass);
        _wConsoleWidget->setObjectName(QString::fromUtf8("_wConsoleWidget"));
        _wConsoleWidget->setFeatures(QDockWidget::DockWidgetFloatable|QDockWidget::DockWidgetMovable);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        gridLayout_3 = new QGridLayout(dockWidgetContents);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        widget = new QWidget(dockWidgetContents);
        widget->setObjectName(QString::fromUtf8("widget"));
        gridLayout_2 = new QGridLayout(widget);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        _wConsoleOutput = new QPlainTextEdit(widget);
        _wConsoleOutput->setObjectName(QString::fromUtf8("_wConsoleOutput"));
        QFont font;
        font.setFamily(QString::fromUtf8("Monospace"));
        _wConsoleOutput->setFont(font);
        _wConsoleOutput->setReadOnly(true);

        gridLayout_2->addWidget(_wConsoleOutput, 0, 0, 2, 2);

        _wConsoleInput = new QLineEdit(widget);
        _wConsoleInput->setObjectName(QString::fromUtf8("_wConsoleInput"));

        gridLayout_2->addWidget(_wConsoleInput, 2, 0, 1, 1);


        gridLayout_3->addWidget(widget, 0, 0, 1, 1);

        _wConsoleWidget->setWidget(dockWidgetContents);

        gridLayout->addWidget(_wConsoleWidget, 1, 0, 1, 1);


        retranslateUi(TestMaze_IIClass);
        QObject::connect(_wConsoleInput, SIGNAL(returnPressed()), TestMaze_IIClass, SLOT(process_console_input()));

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
