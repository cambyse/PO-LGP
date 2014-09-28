#include "Editor.h"
#include "ui_Editor.h"

#include "util.h"

#include <QtCore>
#include <QFileDialog>

#include <list>

using namespace std;

Editor::Editor(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Editor)
{
    ui->setupUi(this);
    parse_content(read_file(ui->file_name->text()));
}

Editor::~Editor()
{
    delete ui;
}

QString Editor::read_file(QString file_name)
{
    // open and read input file
    QFile input_file(file_name);
    QString input;
    if(!input_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        block_editor_signals();
        ui->graph_editor->setPlainText(QString("Error: Could not open input file '%1'").arg(file_name));
        unblock_editor_signals();
    } else {
        QTextStream input_file_stream(&input_file);
        while(!input_file_stream.atEnd()) {
            input += input_file_stream.readLine()+"\n";
        }
        input.chop(1); // remove last new-line
    }
    return input;
}

void Editor::parse_content(QString input)
{
    // highlight
    editor_content = "";
    Parser::parse_graph(input, editor_content);
    editor_content = R"(<span style="font-family:monospace;font-size:10pt">)" + editor_content + R"(</span>)";

    // display result (raw/rich text)
    int old_pos = ui->graph_editor->textCursor().position();
    block_editor_signals();
    ui->graph_editor->setHtml(editor_content);
    unblock_editor_signals();
    QTextCursor new_cursor = ui->graph_editor->textCursor();
    new_cursor.setPosition(old_pos);
    ui->graph_editor->setTextCursor(new_cursor);
}

void Editor::block_editor_signals()
{
    editor_blocked_state = ui->graph_editor->blockSignals(true);
}

void Editor::unblock_editor_signals()
{
    ui->graph_editor->blockSignals(editor_blocked_state);
}

void Editor::raw_display(bool raw)
{
    block_editor_signals();
    if(raw) {
        ui->graph_editor->setReadOnly(true);
        ui->graph_editor->setPlainText(ui->graph_editor->toHtml());
    } else {
        ui->graph_editor->setReadOnly(false);
        ui->graph_editor->setHtml(editor_content);
    }
    unblock_editor_signals();
}

void Editor::update_content()
{
    DEBUG_OUT("UPDATE");
    parse_content(ui->graph_editor->toPlainText());
}

void Editor::open_file()
{
    QString old_file_name = ui->file_name->text();
    QString base_path = QFileInfo(old_file_name).absolutePath();
    QString file_name = QFileDialog::getOpenFileName(this, tr("Parse file:"), base_path, tr("Text (*.txt);; Other (*)"));
    if(file_name!="") {
        ui->file_name->setText(file_name);
        parse_content(read_file(file_name));
    }
}

void Editor::save_file()
{
    QString file_name = ui->file_name->text();
    QFile outfile;
    outfile.setFileName(file_name);
    outfile.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&outfile);
    out << ui->graph_editor->toPlainText() << endl;
}

void Editor::quit()
{
    QApplication::quit();
}
