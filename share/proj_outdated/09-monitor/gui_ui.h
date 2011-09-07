/********************************************************************************
** Form generated from reading ui file 'gui.ui'
**
** Created: Tue May 4 11:22:12 2010
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef GUI_UI_H
#define GUI_UI_H

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
    QLabel *m7;
    QLabel *m4;
    QLabel *m6;
    QLabel *m5;
    QLabel *m3;
    QGraphicsView *graphicsView;
    QPushButton *homeOffset;
    QPushButton *reportAll;

    void setupUi(QDialog *SchunkMonitor)
    {
        if (SchunkMonitor->objectName().isEmpty())
            SchunkMonitor->setObjectName(QString::fromUtf8("SchunkMonitor"));
        SchunkMonitor->resize(478, 517);
        QFont font;
        font.setFamily(QString::fromUtf8("AlArabiya"));
        font.setPointSize(10);
        SchunkMonitor->setFont(font);
        horizontalLayoutWidget = new QWidget(SchunkMonitor);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(20, 240, 400, 171));
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
        gridLayoutWidget->setGeometry(QRect(20, 420, 401, 56));
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

        m7 = new QLabel(gridLayoutWidget);
        m7->setObjectName(QString::fromUtf8("m7"));
        m7->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(m7, 0, 4, 1, 1);

        m4 = new QLabel(gridLayoutWidget);
        m4->setObjectName(QString::fromUtf8("m4"));
        m4->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(m4, 0, 1, 1, 1);

        m6 = new QLabel(gridLayoutWidget);
        m6->setObjectName(QString::fromUtf8("m6"));
        m6->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(m6, 0, 3, 1, 1);

        m5 = new QLabel(gridLayoutWidget);
        m5->setObjectName(QString::fromUtf8("m5"));
        m5->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(m5, 0, 2, 1, 1);

        m3 = new QLabel(gridLayoutWidget);
        m3->setObjectName(QString::fromUtf8("m3"));
        m3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(m3, 0, 0, 1, 1);

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
        graphicsView = new QGraphicsView(SchunkMonitor);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setGeometry(QRect(200, 20, 256, 192));
        homeOffset = new QPushButton(SchunkMonitor);
        homeOffset->setObjectName(QString::fromUtf8("homeOffset"));
        homeOffset->setGeometry(QRect(20, 140, 171, 27));
        reportAll = new QPushButton(SchunkMonitor);
        reportAll->setObjectName(QString::fromUtf8("reportAll"));
        reportAll->setGeometry(QRect(20, 180, 171, 27));

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
        m7->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m4->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m6->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m5->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        m3->setText(QApplication::translate("SchunkMonitor", "0", 0, QApplication::UnicodeUTF8));
        homeOffset->setText(QApplication::translate("SchunkMonitor", "set this pos as HomeOffset", 0, QApplication::UnicodeUTF8));
        reportAll->setText(QApplication::translate("SchunkMonitor", "report all", 0, QApplication::UnicodeUTF8));
        Q_UNUSED(SchunkMonitor);
    } // retranslateUi

};

namespace Ui {
    class SchunkMonitor: public Ui_SchunkMonitor {};
} // namespace Ui

QT_END_NAMESPACE

#endif // GUI_UI_H
