#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::display_raw()
{
    ui->display_plain->blockSignals(true);
    ui->display_plain->setPlainText(ui->display_html->toHtml());
    ui->display_plain->blockSignals(false);
}

void MainWindow::display_html()
{
    ui->display_html->blockSignals(true);
    ui->display_html->setHtml(ui->display_plain->toPlainText());
    ui->display_html->blockSignals(false);
}
