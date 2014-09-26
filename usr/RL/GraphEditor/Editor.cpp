#include "Editor.h"
#include "ui_Editor.h"

#include <QtCore>
#include <QFileDialog>

#include <list>

using namespace std;

Editor::Editor(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Editor)
{
    ui->setupUi(this);
    parser.set_console(ui->console);
    parse_file(ui->file_name->text());
}

Editor::~Editor()
{
    delete ui;
}

void Editor::parse_file(QString file_name)
{
    // open and parse input file
    QFile input_file(file_name);
    QString output = "<font face=\"monospace\">\n";
    if(!input_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        ui->graph_editor->setText(QString("Error: Could not open input file '%1'").arg(file_name));
    } else {
        // parse line by line
        QTextStream input_file_stream(&input_file);
        //input_file_stream.setCodec("UTF-8");
        while(!input_file_stream.atEnd()) {
            parser.parse_input(input_file_stream.readLine());
            output += parser.get_output()+"<br>\n";
        }
        if(ui->raw_display->isChecked()) {
            ui->graph_editor->setPlainText(output.replace("&nbsp;"," "));
        } else {
            ui->graph_editor->setHtml(output);
        }
    }
}

void Editor::parse_file()
{
    parse_file(ui->file_name->text());
}

void Editor::open_file()
{
    QString old_file_name = ui->file_name->text();
    QString base_path = QFileInfo(old_file_name).absolutePath();
    QString file_name = QFileDialog::getOpenFileName(this, tr("Parse file:"), base_path, tr("Text (*.txt);; Other (*)"));
    if(file_name!="") {
        ui->file_name->setText(file_name);
    }
}

void Editor::quit()
{
    QApplication::quit();
}
