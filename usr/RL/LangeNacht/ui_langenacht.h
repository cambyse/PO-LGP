/********************************************************************************
** Form generated from reading UI file 'langenacht.ui'
**
** Created: Mon Jun 18 15:25:12 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LANGENACHT_H
#define UI_LANGENACHT_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include "WorldModel/MyDisplay.h"
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LangeNachtClass
{
public:
    QGridLayout *gridLayout;
    MyDisplay *_wDisplay;
    QPushButton *_wDeleteAllRewards;
    QSlider *_wRandom;
    QLineEdit *_wRandomValue;
    QLabel *label;
    QLabel *label_2;
    QSlider *_wDiscount;
    QLineEdit *_wDiscountValue;
    QPushButton *_wDeleteAllWalls;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QRadioButton *_wShowActions;
    QRadioButton *radioButton_4;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_3;
    QRadioButton *_wShowRewards;
    QRadioButton *radioButton_2;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout_4;
    QRadioButton *radioButton_5;
    QRadioButton *_wLoopValueIteration;
    QGroupBox *groupBox_4;
    QGridLayout *gridLayout_5;
    QRadioButton *radioButton_7;
    QRadioButton *_wLoopActions;
    QLabel *label_3;
    QSlider *_wSpeed;
    QLineEdit *_wSpeedValue;
    QSpinBox *_wXSize;
    QSpinBox *_wYSize;
    QPushButton *_wReset;
    QLabel *label_4;
    QLabel *label_5;
    QSpacerItem *verticalSpacer_2;
    QGroupBox *groupBox_5;
    QGridLayout *gridLayout_6;
    QRadioButton *radioButton;
    QRadioButton *_wAutoRewards;

    void setupUi(QWidget *LangeNachtClass)
    {
        if (LangeNachtClass->objectName().isEmpty())
            LangeNachtClass->setObjectName(QString::fromUtf8("LangeNachtClass"));
        LangeNachtClass->resize(746, 603);
        gridLayout = new QGridLayout(LangeNachtClass);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        _wDisplay = new MyDisplay(LangeNachtClass);
        _wDisplay->setObjectName(QString::fromUtf8("_wDisplay"));

        gridLayout->addWidget(_wDisplay, 1, 0, 11, 3);

        _wDeleteAllRewards = new QPushButton(LangeNachtClass);
        _wDeleteAllRewards->setObjectName(QString::fromUtf8("_wDeleteAllRewards"));

        gridLayout->addWidget(_wDeleteAllRewards, 6, 3, 1, 4);

        _wRandom = new QSlider(LangeNachtClass);
        _wRandom->setObjectName(QString::fromUtf8("_wRandom"));
        _wRandom->setMinimumSize(QSize(150, 0));
        _wRandom->setMaximum(100);
        _wRandom->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(_wRandom, 12, 1, 1, 1);

        _wRandomValue = new QLineEdit(LangeNachtClass);
        _wRandomValue->setObjectName(QString::fromUtf8("_wRandomValue"));
        _wRandomValue->setMaximumSize(QSize(50, 16777215));

        gridLayout->addWidget(_wRandomValue, 12, 2, 1, 1);

        label = new QLabel(LangeNachtClass);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 12, 0, 1, 1);

        label_2 = new QLabel(LangeNachtClass);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 13, 0, 1, 1);

        _wDiscount = new QSlider(LangeNachtClass);
        _wDiscount->setObjectName(QString::fromUtf8("_wDiscount"));
        _wDiscount->setMinimumSize(QSize(150, 0));
        _wDiscount->setMaximum(100);
        _wDiscount->setSliderPosition(10);
        _wDiscount->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(_wDiscount, 13, 1, 1, 1);

        _wDiscountValue = new QLineEdit(LangeNachtClass);
        _wDiscountValue->setObjectName(QString::fromUtf8("_wDiscountValue"));
        _wDiscountValue->setMaximumSize(QSize(50, 16777215));

        gridLayout->addWidget(_wDiscountValue, 13, 2, 1, 1);

        _wDeleteAllWalls = new QPushButton(LangeNachtClass);
        _wDeleteAllWalls->setObjectName(QString::fromUtf8("_wDeleteAllWalls"));

        gridLayout->addWidget(_wDeleteAllWalls, 7, 3, 1, 4);

        groupBox = new QGroupBox(LangeNachtClass);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        _wShowActions = new QRadioButton(groupBox);
        _wShowActions->setObjectName(QString::fromUtf8("_wShowActions"));
        _wShowActions->setChecked(false);

        gridLayout_2->addWidget(_wShowActions, 0, 0, 1, 1);

        radioButton_4 = new QRadioButton(groupBox);
        radioButton_4->setObjectName(QString::fromUtf8("radioButton_4"));
        radioButton_4->setChecked(true);

        gridLayout_2->addWidget(radioButton_4, 0, 1, 1, 1);


        gridLayout->addWidget(groupBox, 2, 3, 1, 4);

        groupBox_2 = new QGroupBox(LangeNachtClass);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        gridLayout_3 = new QGridLayout(groupBox_2);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        _wShowRewards = new QRadioButton(groupBox_2);
        _wShowRewards->setObjectName(QString::fromUtf8("_wShowRewards"));

        gridLayout_3->addWidget(_wShowRewards, 0, 0, 1, 1);

        radioButton_2 = new QRadioButton(groupBox_2);
        radioButton_2->setObjectName(QString::fromUtf8("radioButton_2"));
        radioButton_2->setChecked(true);

        gridLayout_3->addWidget(radioButton_2, 0, 1, 1, 1);


        gridLayout->addWidget(groupBox_2, 1, 3, 1, 4);

        groupBox_3 = new QGroupBox(LangeNachtClass);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        gridLayout_4 = new QGridLayout(groupBox_3);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        radioButton_5 = new QRadioButton(groupBox_3);
        radioButton_5->setObjectName(QString::fromUtf8("radioButton_5"));

        gridLayout_4->addWidget(radioButton_5, 0, 0, 1, 1);

        _wLoopValueIteration = new QRadioButton(groupBox_3);
        _wLoopValueIteration->setObjectName(QString::fromUtf8("_wLoopValueIteration"));
        _wLoopValueIteration->setChecked(true);

        gridLayout_4->addWidget(_wLoopValueIteration, 0, 1, 1, 1);


        gridLayout->addWidget(groupBox_3, 3, 3, 1, 4);

        groupBox_4 = new QGroupBox(LangeNachtClass);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        gridLayout_5 = new QGridLayout(groupBox_4);
        gridLayout_5->setSpacing(6);
        gridLayout_5->setContentsMargins(11, 11, 11, 11);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        radioButton_7 = new QRadioButton(groupBox_4);
        radioButton_7->setObjectName(QString::fromUtf8("radioButton_7"));
        radioButton_7->setChecked(true);

        gridLayout_5->addWidget(radioButton_7, 0, 0, 1, 1);

        _wLoopActions = new QRadioButton(groupBox_4);
        _wLoopActions->setObjectName(QString::fromUtf8("_wLoopActions"));

        gridLayout_5->addWidget(_wLoopActions, 0, 1, 1, 1);


        gridLayout->addWidget(groupBox_4, 4, 3, 1, 4);

        label_3 = new QLabel(LangeNachtClass);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 14, 0, 1, 1);

        _wSpeed = new QSlider(LangeNachtClass);
        _wSpeed->setObjectName(QString::fromUtf8("_wSpeed"));
        _wSpeed->setMinimumSize(QSize(150, 0));
        _wSpeed->setMinimum(1);
        _wSpeed->setMaximum(100);
        _wSpeed->setValue(30);
        _wSpeed->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(_wSpeed, 14, 1, 1, 1);

        _wSpeedValue = new QLineEdit(LangeNachtClass);
        _wSpeedValue->setObjectName(QString::fromUtf8("_wSpeedValue"));
        _wSpeedValue->setMaximumSize(QSize(50, 16777215));

        gridLayout->addWidget(_wSpeedValue, 14, 2, 1, 1);

        _wXSize = new QSpinBox(LangeNachtClass);
        _wXSize->setObjectName(QString::fromUtf8("_wXSize"));
        _wXSize->setMaximumSize(QSize(50, 16777215));
        _wXSize->setValue(10);

        gridLayout->addWidget(_wXSize, 9, 4, 1, 1);

        _wYSize = new QSpinBox(LangeNachtClass);
        _wYSize->setObjectName(QString::fromUtf8("_wYSize"));
        _wYSize->setMaximumSize(QSize(50, 16777215));
        _wYSize->setValue(10);

        gridLayout->addWidget(_wYSize, 10, 4, 1, 1);

        _wReset = new QPushButton(LangeNachtClass);
        _wReset->setObjectName(QString::fromUtf8("_wReset"));

        gridLayout->addWidget(_wReset, 9, 5, 2, 1);

        label_4 = new QLabel(LangeNachtClass);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setMaximumSize(QSize(17, 16777215));

        gridLayout->addWidget(label_4, 9, 3, 1, 1);

        label_5 = new QLabel(LangeNachtClass);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setMaximumSize(QSize(17, 16777215));

        gridLayout->addWidget(label_5, 10, 3, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer_2, 8, 4, 1, 1);

        groupBox_5 = new QGroupBox(LangeNachtClass);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        gridLayout_6 = new QGridLayout(groupBox_5);
        gridLayout_6->setSpacing(6);
        gridLayout_6->setContentsMargins(11, 11, 11, 11);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        radioButton = new QRadioButton(groupBox_5);
        radioButton->setObjectName(QString::fromUtf8("radioButton"));
        radioButton->setChecked(true);

        gridLayout_6->addWidget(radioButton, 0, 0, 1, 1);

        _wAutoRewards = new QRadioButton(groupBox_5);
        _wAutoRewards->setObjectName(QString::fromUtf8("_wAutoRewards"));

        gridLayout_6->addWidget(_wAutoRewards, 0, 1, 1, 1);


        gridLayout->addWidget(groupBox_5, 5, 3, 1, 4);


        retranslateUi(LangeNachtClass);
        QObject::connect(_wDeleteAllRewards, SIGNAL(clicked()), LangeNachtClass, SLOT(delete_all_rewards()));
        QObject::connect(_wDeleteAllWalls, SIGNAL(clicked()), LangeNachtClass, SLOT(delete_all_walls()));
        QObject::connect(_wRandom, SIGNAL(valueChanged(int)), LangeNachtClass, SLOT(random_changed(int)));
        QObject::connect(_wDiscount, SIGNAL(valueChanged(int)), LangeNachtClass, SLOT(discount_changed(int)));
        QObject::connect(_wShowRewards, SIGNAL(toggled(bool)), LangeNachtClass, SLOT(show_rewards_changed(bool)));
        QObject::connect(_wShowActions, SIGNAL(toggled(bool)), LangeNachtClass, SLOT(show_actions_changed(bool)));
        QObject::connect(_wSpeed, SIGNAL(valueChanged(int)), LangeNachtClass, SLOT(speed_changed(int)));
        QObject::connect(_wReset, SIGNAL(clicked()), LangeNachtClass, SLOT(reset_grid_world()));
        QObject::connect(_wAutoRewards, SIGNAL(toggled(bool)), LangeNachtClass, SLOT(auto_rewards_changed(bool)));

        QMetaObject::connectSlotsByName(LangeNachtClass);
    } // setupUi

    void retranslateUi(QWidget *LangeNachtClass)
    {
        LangeNachtClass->setWindowTitle(QApplication::translate("LangeNachtClass", "LangeNacht", 0, QApplication::UnicodeUTF8));
        _wDeleteAllRewards->setText(QApplication::translate("LangeNachtClass", "Alle Belohnungen L\303\266schen", 0, QApplication::UnicodeUTF8));
        _wRandomValue->setText(QApplication::translate("LangeNachtClass", "0%", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("LangeNachtClass", "Zufall in %", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("LangeNachtClass", "Vergessen in %", 0, QApplication::UnicodeUTF8));
        _wDiscountValue->setText(QApplication::translate("LangeNachtClass", "10%", 0, QApplication::UnicodeUTF8));
        _wDeleteAllWalls->setText(QApplication::translate("LangeNachtClass", "Alle W\303\244nde L\303\266schen", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("LangeNachtClass", "Aktionen anzeigen", 0, QApplication::UnicodeUTF8));
        _wShowActions->setText(QApplication::translate("LangeNachtClass", "anzeigen", 0, QApplication::UnicodeUTF8));
        radioButton_4->setText(QApplication::translate("LangeNachtClass", "nicht anzeigen", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("LangeNachtClass", "Farbwert der Felder", 0, QApplication::UnicodeUTF8));
        _wShowRewards->setText(QApplication::translate("LangeNachtClass", "Belohnungen", 0, QApplication::UnicodeUTF8));
        radioButton_2->setText(QApplication::translate("LangeNachtClass", "Erwartungen", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("LangeNachtClass", "Value Iteration", 0, QApplication::UnicodeUTF8));
        radioButton_5->setText(QApplication::translate("LangeNachtClass", "mit 'Enter'", 0, QApplication::UnicodeUTF8));
        _wLoopValueIteration->setText(QApplication::translate("LangeNachtClass", "Loop", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("LangeNachtClass", "Aktionen w\303\244hlen", 0, QApplication::UnicodeUTF8));
        radioButton_7->setText(QApplication::translate("LangeNachtClass", "manuell (Pfeiltasten,\n"
"'Space', 'r', 'o')", 0, QApplication::UnicodeUTF8));
        _wLoopActions->setText(QApplication::translate("LangeNachtClass", "Loop\n"
"(optimal)", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("LangeNachtClass", "Speed (f/s)", 0, QApplication::UnicodeUTF8));
        _wSpeedValue->setText(QApplication::translate("LangeNachtClass", "30", 0, QApplication::UnicodeUTF8));
        _wReset->setText(QApplication::translate("LangeNachtClass", "Reset", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("LangeNachtClass", "X", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("LangeNachtClass", "Y", 0, QApplication::UnicodeUTF8));
        groupBox_5->setTitle(QApplication::translate("LangeNachtClass", "Auto Rewards", 0, QApplication::UnicodeUTF8));
        radioButton->setText(QApplication::translate("LangeNachtClass", "manuell", 0, QApplication::UnicodeUTF8));
        _wAutoRewards->setText(QApplication::translate("LangeNachtClass", "automatisch", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class LangeNachtClass: public Ui_LangeNachtClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LANGENACHT_H
