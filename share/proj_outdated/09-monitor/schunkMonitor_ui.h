/********************************************************************************
** Form generated from reading ui file 'schunkMonitor.ui'
**
** Created: Tue Sep 1 18:17:57 2009
**      by: Qt User Interface Compiler version 4.5.0
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef SCHUNKMONITOR_UI_H
#define SCHUNKMONITOR_UI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QFrame>
#include <QtGui/QGraphicsView>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SchunkMonitor
{
public:
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QSlider *p3;
    QSlider *t3;
    QFrame *line;
    QSlider *p4;
    QSlider *t4;
    QFrame *line_2;
    QSlider *p5;
    QSlider *t5;
    QFrame *line_3;
    QSlider *p6;
    QSlider *t6;
    QFrame *line_4;
    QSlider *p7;
    QSlider *t7;
    QFrame *line_5;
    QSlider *p8;
    QSlider *t8;
    QFrame *line_6;
    QSlider *p9;
    QSlider *t9;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLabel *v8;
    QLabel *m8;
    QLabel *m9;
    QLabel *v9;
    QLabel *v7;
    QLabel *v6;
    QLabel *v5;
    QLabel *v4;
    QLabel *v3;
    QLabel *m3;
    QLabel *m5;
    QLabel *m6;
    QLabel *m4;
    QLabel *m7;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_2;
    QLabel *v8_2;
    QLabel *m8_2;
    QLabel *m9_2;
    QLabel *v9_2;
    QLabel *m7_2;
    QLabel *v7_2;
    QLabel *m6_2;
    QLabel *v6_2;
    QLabel *m5_2;
    QLabel *v5_2;
    QLabel *m4_2;
    QLabel *v4_2;
    QLabel *m3_2;
    QLabel *v3_2;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_2;
    QSlider *p3_2;
    QSlider *t3_2;
    QFrame *line_7;
    QSlider *p4_2;
    QSlider *t4_2;
    QFrame *line_8;
    QSlider *p5_2;
    QSlider *t5_2;
    QFrame *line_9;
    QSlider *p6_2;
    QSlider *t6_2;
    QFrame *line_10;
    QSlider *p7_2;
    QSlider *t7_2;
    QFrame *line_11;
    QSlider *p8_2;
    QSlider *t8_2;
    QFrame *line_12;
    QSlider *p9_2;
    QSlider *t9_2;
    QGraphicsView *graphicsView;
    QPushButton *homeOffset;

    void setupUi(QDialog *SchunkMonitor)
    {
        if (SchunkMonitor->objectName().isEmpty())
            SchunkMonitor->setObjectName(QString::fromUtf8("SchunkMonitor"));
        SchunkMonitor->resize(859, 517);
        QPalette palette;
        QBrush brush(QColor(255, 255, 255, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Base, brush);
        QBrush brush1(QColor(170, 170, 127, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        SchunkMonitor->setPalette(palette);
        horizontalLayoutWidget = new QWidget(SchunkMonitor);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(20, 240, 381, 171));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        p3 = new QSlider(horizontalLayoutWidget);
        p3->setObjectName(QString::fromUtf8("p3"));
        p3->setMinimum(-1000);
        p3->setMaximum(1000);
        p3->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(p3);

        t3 = new QSlider(horizontalLayoutWidget);
        t3->setObjectName(QString::fromUtf8("t3"));
        t3->setMinimum(-1000);
        t3->setMaximum(1000);
        t3->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(t3);

        line = new QFrame(horizontalLayoutWidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(line);

        p4 = new QSlider(horizontalLayoutWidget);
        p4->setObjectName(QString::fromUtf8("p4"));
        p4->setMinimum(-1000);
        p4->setMaximum(1000);
        p4->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(p4);

        t4 = new QSlider(horizontalLayoutWidget);
        t4->setObjectName(QString::fromUtf8("t4"));
        t4->setMinimum(-1000);
        t4->setMaximum(1000);
        t4->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(t4);

        line_2 = new QFrame(horizontalLayoutWidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::VLine);
        line_2->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(line_2);

        p5 = new QSlider(horizontalLayoutWidget);
        p5->setObjectName(QString::fromUtf8("p5"));
        p5->setMinimum(-1000);
        p5->setMaximum(1000);
        p5->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(p5);

        t5 = new QSlider(horizontalLayoutWidget);
        t5->setObjectName(QString::fromUtf8("t5"));
        t5->setMinimum(-1000);
        t5->setMaximum(1000);
        t5->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(t5);

        line_3 = new QFrame(horizontalLayoutWidget);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(line_3);

        p6 = new QSlider(horizontalLayoutWidget);
        p6->setObjectName(QString::fromUtf8("p6"));
        p6->setMinimum(-1000);
        p6->setMaximum(1000);
        p6->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(p6);

        t6 = new QSlider(horizontalLayoutWidget);
        t6->setObjectName(QString::fromUtf8("t6"));
        t6->setMinimum(-1000);
        t6->setMaximum(1000);
        t6->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(t6);

        line_4 = new QFrame(horizontalLayoutWidget);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setFrameShape(QFrame::VLine);
        line_4->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(line_4);

        p7 = new QSlider(horizontalLayoutWidget);
        p7->setObjectName(QString::fromUtf8("p7"));
        p7->setMinimum(-1000);
        p7->setMaximum(1000);
        p7->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(p7);

        t7 = new QSlider(horizontalLayoutWidget);
        t7->setObjectName(QString::fromUtf8("t7"));
        t7->setMinimum(-1000);
        t7->setMaximum(1000);
        t7->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(t7);

        line_5 = new QFrame(horizontalLayoutWidget);
        line_5->setObjectName(QString::fromUtf8("line_5"));
        line_5->setFrameShape(QFrame::VLine);
        line_5->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(line_5);

        p8 = new QSlider(horizontalLayoutWidget);
        p8->setObjectName(QString::fromUtf8("p8"));
        p8->setMinimum(-1000);
        p8->setMaximum(1000);
        p8->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(p8);

        t8 = new QSlider(horizontalLayoutWidget);
        t8->setObjectName(QString::fromUtf8("t8"));
        t8->setMinimum(-1000);
        t8->setMaximum(1000);
        t8->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(t8);

        line_6 = new QFrame(horizontalLayoutWidget);
        line_6->setObjectName(QString::fromUtf8("line_6"));
        line_6->setFrameShape(QFrame::VLine);
        line_6->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(line_6);

        p9 = new QSlider(horizontalLayoutWidget);
        p9->setObjectName(QString::fromUtf8("p9"));
        p9->setMinimum(-1000);
        p9->setMaximum(1000);
        p9->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(p9);

        t9 = new QSlider(horizontalLayoutWidget);
        t9->setObjectName(QString::fromUtf8("t9"));
        t9->setMinimum(-1000);
        t9->setMaximum(1000);
        t9->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(t9);

        gridLayoutWidget = new QWidget(SchunkMonitor);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(20, 420, 381, 42));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        v8 = new QLabel(gridLayoutWidget);
        v8->setObjectName(QString::fromUtf8("v8"));
        v8->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(v8, 1, 5, 1, 1);

        m8 = new QLabel(gridLayoutWidget);
        m8->setObjectName(QString::fromUtf8("m8"));
        m8->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(m8, 0, 5, 1, 1);

        m9 = new QLabel(gridLayoutWidget);
        m9->setObjectName(QString::fromUtf8("m9"));
        m9->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(m9, 0, 6, 1, 1);

        v9 = new QLabel(gridLayoutWidget);
        v9->setObjectName(QString::fromUtf8("v9"));
        v9->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(v9, 1, 6, 1, 1);

        v7 = new QLabel(gridLayoutWidget);
        v7->setObjectName(QString::fromUtf8("v7"));
        v7->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(v7, 1, 4, 1, 1);

        v6 = new QLabel(gridLayoutWidget);
        v6->setObjectName(QString::fromUtf8("v6"));
        v6->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(v6, 1, 3, 1, 1);

        v5 = new QLabel(gridLayoutWidget);
        v5->setObjectName(QString::fromUtf8("v5"));
        v5->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(v5, 1, 2, 1, 1);

        v4 = new QLabel(gridLayoutWidget);
        v4->setObjectName(QString::fromUtf8("v4"));
        v4->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(v4, 1, 1, 1, 1);

        v3 = new QLabel(gridLayoutWidget);
        v3->setObjectName(QString::fromUtf8("v3"));
        v3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(v3, 1, 0, 1, 1);

        m3 = new QLabel(gridLayoutWidget);
        m3->setObjectName(QString::fromUtf8("m3"));
        m3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(m3, 0, 0, 1, 1);

        m5 = new QLabel(gridLayoutWidget);
        m5->setObjectName(QString::fromUtf8("m5"));
        m5->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(m5, 0, 2, 1, 1);

        m6 = new QLabel(gridLayoutWidget);
        m6->setObjectName(QString::fromUtf8("m6"));
        m6->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(m6, 0, 3, 1, 1);

        m4 = new QLabel(gridLayoutWidget);
        m4->setObjectName(QString::fromUtf8("m4"));
        m4->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(m4, 0, 1, 1, 1);

        m7 = new QLabel(gridLayoutWidget);
        m7->setObjectName(QString::fromUtf8("m7"));
        m7->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(m7, 0, 4, 1, 1);

        v8->raise();
        m8->raise();
        m9->raise();
        v9->raise();
        v7->raise();
        v6->raise();
        v5->raise();
        v4->raise();
        v3->raise();
        m7->raise();
        m4->raise();
        m6->raise();
        m5->raise();
        m3->raise();
        gridLayoutWidget_2 = new QWidget(SchunkMonitor);
        gridLayoutWidget_2->setObjectName(QString::fromUtf8("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(410, 420, 430, 42));
        gridLayout_2 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        v8_2 = new QLabel(gridLayoutWidget_2);
        v8_2->setObjectName(QString::fromUtf8("v8_2"));

        gridLayout_2->addWidget(v8_2, 1, 5, 1, 1);

        m8_2 = new QLabel(gridLayoutWidget_2);
        m8_2->setObjectName(QString::fromUtf8("m8_2"));

        gridLayout_2->addWidget(m8_2, 0, 5, 1, 1);

        m9_2 = new QLabel(gridLayoutWidget_2);
        m9_2->setObjectName(QString::fromUtf8("m9_2"));

        gridLayout_2->addWidget(m9_2, 0, 6, 1, 1);

        v9_2 = new QLabel(gridLayoutWidget_2);
        v9_2->setObjectName(QString::fromUtf8("v9_2"));

        gridLayout_2->addWidget(v9_2, 1, 6, 1, 1);

        m7_2 = new QLabel(gridLayoutWidget_2);
        m7_2->setObjectName(QString::fromUtf8("m7_2"));

        gridLayout_2->addWidget(m7_2, 0, 4, 1, 1);

        v7_2 = new QLabel(gridLayoutWidget_2);
        v7_2->setObjectName(QString::fromUtf8("v7_2"));

        gridLayout_2->addWidget(v7_2, 1, 4, 1, 1);

        m6_2 = new QLabel(gridLayoutWidget_2);
        m6_2->setObjectName(QString::fromUtf8("m6_2"));

        gridLayout_2->addWidget(m6_2, 0, 3, 1, 1);

        v6_2 = new QLabel(gridLayoutWidget_2);
        v6_2->setObjectName(QString::fromUtf8("v6_2"));

        gridLayout_2->addWidget(v6_2, 1, 3, 1, 1);

        m5_2 = new QLabel(gridLayoutWidget_2);
        m5_2->setObjectName(QString::fromUtf8("m5_2"));

        gridLayout_2->addWidget(m5_2, 0, 2, 1, 1);

        v5_2 = new QLabel(gridLayoutWidget_2);
        v5_2->setObjectName(QString::fromUtf8("v5_2"));

        gridLayout_2->addWidget(v5_2, 1, 2, 1, 1);

        m4_2 = new QLabel(gridLayoutWidget_2);
        m4_2->setObjectName(QString::fromUtf8("m4_2"));

        gridLayout_2->addWidget(m4_2, 0, 1, 1, 1);

        v4_2 = new QLabel(gridLayoutWidget_2);
        v4_2->setObjectName(QString::fromUtf8("v4_2"));

        gridLayout_2->addWidget(v4_2, 1, 1, 1, 1);

        m3_2 = new QLabel(gridLayoutWidget_2);
        m3_2->setObjectName(QString::fromUtf8("m3_2"));

        gridLayout_2->addWidget(m3_2, 0, 0, 1, 1);

        v3_2 = new QLabel(gridLayoutWidget_2);
        v3_2->setObjectName(QString::fromUtf8("v3_2"));

        gridLayout_2->addWidget(v3_2, 1, 0, 1, 1);

        horizontalLayoutWidget_2 = new QWidget(SchunkMonitor);
        horizontalLayoutWidget_2->setObjectName(QString::fromUtf8("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(410, 240, 381, 171));
        horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        p3_2 = new QSlider(horizontalLayoutWidget_2);
        p3_2->setObjectName(QString::fromUtf8("p3_2"));
        p3_2->setMinimum(-1000);
        p3_2->setMaximum(1000);
        p3_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(p3_2);

        t3_2 = new QSlider(horizontalLayoutWidget_2);
        t3_2->setObjectName(QString::fromUtf8("t3_2"));
        t3_2->setMinimum(-1000);
        t3_2->setMaximum(1000);
        t3_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(t3_2);

        line_7 = new QFrame(horizontalLayoutWidget_2);
        line_7->setObjectName(QString::fromUtf8("line_7"));
        line_7->setFrameShape(QFrame::VLine);
        line_7->setFrameShadow(QFrame::Sunken);

        horizontalLayout_2->addWidget(line_7);

        p4_2 = new QSlider(horizontalLayoutWidget_2);
        p4_2->setObjectName(QString::fromUtf8("p4_2"));
        p4_2->setMinimum(-1000);
        p4_2->setMaximum(1000);
        p4_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(p4_2);

        t4_2 = new QSlider(horizontalLayoutWidget_2);
        t4_2->setObjectName(QString::fromUtf8("t4_2"));
        t4_2->setMinimum(-1000);
        t4_2->setMaximum(1000);
        t4_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(t4_2);

        line_8 = new QFrame(horizontalLayoutWidget_2);
        line_8->setObjectName(QString::fromUtf8("line_8"));
        line_8->setFrameShape(QFrame::VLine);
        line_8->setFrameShadow(QFrame::Sunken);

        horizontalLayout_2->addWidget(line_8);

        p5_2 = new QSlider(horizontalLayoutWidget_2);
        p5_2->setObjectName(QString::fromUtf8("p5_2"));
        p5_2->setMinimum(-1000);
        p5_2->setMaximum(1000);
        p5_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(p5_2);

        t5_2 = new QSlider(horizontalLayoutWidget_2);
        t5_2->setObjectName(QString::fromUtf8("t5_2"));
        t5_2->setMinimum(-1000);
        t5_2->setMaximum(1000);
        t5_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(t5_2);

        line_9 = new QFrame(horizontalLayoutWidget_2);
        line_9->setObjectName(QString::fromUtf8("line_9"));
        line_9->setFrameShape(QFrame::VLine);
        line_9->setFrameShadow(QFrame::Sunken);

        horizontalLayout_2->addWidget(line_9);

        p6_2 = new QSlider(horizontalLayoutWidget_2);
        p6_2->setObjectName(QString::fromUtf8("p6_2"));
        p6_2->setMinimum(-1000);
        p6_2->setMaximum(1000);
        p6_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(p6_2);

        t6_2 = new QSlider(horizontalLayoutWidget_2);
        t6_2->setObjectName(QString::fromUtf8("t6_2"));
        t6_2->setMinimum(-1000);
        t6_2->setMaximum(1000);
        t6_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(t6_2);

        line_10 = new QFrame(horizontalLayoutWidget_2);
        line_10->setObjectName(QString::fromUtf8("line_10"));
        line_10->setFrameShape(QFrame::VLine);
        line_10->setFrameShadow(QFrame::Sunken);

        horizontalLayout_2->addWidget(line_10);

        p7_2 = new QSlider(horizontalLayoutWidget_2);
        p7_2->setObjectName(QString::fromUtf8("p7_2"));
        p7_2->setMinimum(-1000);
        p7_2->setMaximum(1000);
        p7_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(p7_2);

        t7_2 = new QSlider(horizontalLayoutWidget_2);
        t7_2->setObjectName(QString::fromUtf8("t7_2"));
        t7_2->setMinimum(-1000);
        t7_2->setMaximum(1000);
        t7_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(t7_2);

        line_11 = new QFrame(horizontalLayoutWidget_2);
        line_11->setObjectName(QString::fromUtf8("line_11"));
        line_11->setFrameShape(QFrame::VLine);
        line_11->setFrameShadow(QFrame::Sunken);

        horizontalLayout_2->addWidget(line_11);

        p8_2 = new QSlider(horizontalLayoutWidget_2);
        p8_2->setObjectName(QString::fromUtf8("p8_2"));
        p8_2->setMinimum(-1000);
        p8_2->setMaximum(1000);
        p8_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(p8_2);

        t8_2 = new QSlider(horizontalLayoutWidget_2);
        t8_2->setObjectName(QString::fromUtf8("t8_2"));
        t8_2->setMinimum(-1000);
        t8_2->setMaximum(1000);
        t8_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(t8_2);

        line_12 = new QFrame(horizontalLayoutWidget_2);
        line_12->setObjectName(QString::fromUtf8("line_12"));
        line_12->setFrameShape(QFrame::VLine);
        line_12->setFrameShadow(QFrame::Sunken);

        horizontalLayout_2->addWidget(line_12);

        p9_2 = new QSlider(horizontalLayoutWidget_2);
        p9_2->setObjectName(QString::fromUtf8("p9_2"));
        p9_2->setMinimum(-1000);
        p9_2->setMaximum(1000);
        p9_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(p9_2);

        t9_2 = new QSlider(horizontalLayoutWidget_2);
        t9_2->setObjectName(QString::fromUtf8("t9_2"));
        t9_2->setMinimum(-1000);
        t9_2->setMaximum(1000);
        t9_2->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(t9_2);

        graphicsView = new QGraphicsView(SchunkMonitor);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setGeometry(QRect(300, 20, 256, 192));
        homeOffset = new QPushButton(SchunkMonitor);
        homeOffset->setObjectName(QString::fromUtf8("homeOffset"));
        homeOffset->setGeometry(QRect(50, 80, 231, 27));

        retranslateUi(SchunkMonitor);

        QMetaObject::connectSlotsByName(SchunkMonitor);
    } // setupUi

    void retranslateUi(QDialog *SchunkMonitor)
    {
        SchunkMonitor->setWindowTitle(QApplication::translate("SchunkMonitor", "SchunkMonitor", 0, QApplication::UnicodeUTF8));
        v8->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m8->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m9->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        v9->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        v7->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        v6->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        v5->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        v4->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        v3->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m3->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m5->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m6->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m4->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m7->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        v8_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m8_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m9_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        v9_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m7_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        v7_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m6_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        v6_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m5_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        v5_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m4_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        v4_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m3_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        v3_2->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        homeOffset->setText(QApplication::translate("SchunkMonitor", "set this pos as HomeOffset", 0, QApplication::UnicodeUTF8));
        Q_UNUSED(SchunkMonitor);
    } // retranslateUi

};

namespace Ui {
    class SchunkMonitor: public Ui_SchunkMonitor {};
} // namespace Ui

QT_END_NAMESPACE

#endif // SCHUNKMONITOR_UI_H
