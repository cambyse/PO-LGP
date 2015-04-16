#include "PdfPresentation.h"
#include "ui_PdfPresentation.h"
#include <QFileInfo>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>

#include <set>
#include <iostream>
#include <functional>

#define INFO(x) cout << x << endl;
//#define INFO(x)

using namespace std;

static std::ostream& operator<<(std::ostream &out, const QString& s) {
    out << (const char*)s.toLatin1();
    return out;
}

PdfPresentation::PdfPresentation(QWidget *parent) :
    QMainWindow(parent),
    extraction_timer(new QTimer()),
    ui(new Ui::PdfPresentation)
{
    ui->setupUi(this);
    // set home folder
    QString home_folder = QStandardPaths::locate(QStandardPaths::HomeLocation,"",QStandardPaths::LocateDirectory);
    ui->_wFolderPath->setText(home_folder);
    ui->_wZipFilePath->setText(home_folder);
    // set files in tmp folder
    QString tmp_folder = QStandardPaths::locate(QStandardPaths::TempLocation,"",QStandardPaths::LocateDirectory);
    tmp_file_1.setFileName(tmp_folder+"tmp_1_"+random_string(20));
    tmp_file_2.setFileName(tmp_folder+"tmp_2_"+random_string(20));
    tmp_file_3.setFileName(tmp_folder+"tmp_3_"+random_string(20));
    // connect file watcher and extraction timer
    connect(&file_watcher, &QFileSystemWatcher::fileChanged,
            this, &PdfPresentation::zip_file_changed);
    connect(extraction_timer, &QTimer::timeout,
            this, &PdfPresentation::delayed_extraction);
}

PdfPresentation::~PdfPresentation() {
    tmp_file_1.remove();
    tmp_file_2.remove();
    tmp_file_3.remove();
    delete extraction_timer;
    delete ui;
}

void PdfPresentation::read_directory() {
    QDir dir(ui->_wFolderPath->text());
    if(!dir.exists()) {
        QMessageBox::information(this, "", "Folder does not exist.", QMessageBox::Ok);
        return;
    }

    // iterate through files
    dir.setSorting(QDir::LocaleAware);
    dir.setFilter(QDir::Files);
    set<pair<vector<int>,QString>> file_list;
    for(auto file_info : dir.entryInfoList()) {
        QString file_name = file_info.fileName();
        // skip non-pdf files
        if(!file_name.endsWith(".pdf") && !file_name.endsWith(".PDF")) continue;
        // get id
        auto list = file_name.split(QRegExp(R"([^\d-])")); // split at non-digits
        vector<int> id;
        bool ok;
        for(QString number : list) {
            int i = number.toInt(&ok);
            if(ok) {
                id.push_back(i);
            } else {
                break;
            }
        }
        // add to file list
        file_list.insert(make_pair(id,file_name));
    }

    // build file hierarchy
    file_hierarchy.clear();
    vector<pair<vector<int>,QString>> file_stack;
    for(auto it=file_list.begin(); it!=file_list.end(); ++it) {
        // remove files from stack that don't have common id prefix
        while(!file_stack.empty()) {
            if(!same_id_prefix(file_stack.back().first,it->first)) file_stack.pop_back();
            else break;
        }
        // add new file to stack
        file_stack.push_back(*it);
        // add new entry in file hierarchy ONLY if it is not a background file
        // (i.e. not if id is prefix of next id)
        auto next = it;
        ++next;
        if(next==file_list.end() || !same_id_prefix(it->first,next->first)) {
            file_hierarchy.push_back({});
            // write list
            auto & list = file_hierarchy.back();
            INFO("New hierarchy entry:");
            for(auto file : file_stack) {
                list.push_back(file.second);
                INFO("    " << file.second)
            }
        }
    }

    // print file hierarchy to info
    ui->_wInfo->clear();
    int slide_counter = 1;
    for(auto stack : file_hierarchy) {
        ui->_wInfo->appendPlainText(QString("Slide %1:").arg(slide_counter));
        for(auto file : stack) {
            ui->_wInfo->appendPlainText(QString("    %1").arg(file));
        }
        ++slide_counter;
    }
}

bool PdfPresentation::same_id_prefix(const std::vector<int> &v1, const std::vector<int> &v2) const {
    if(v1.size()>v2.size()) return false;
    for(int idx=0; idx<(int)std::min(v1.size(),v2.size()); ++idx) {
        if(v1[idx]!=v2[idx]) return false;
    }
    return true;
}

void PdfPresentation::generate_slides() {
    QDir source_dir(ui->_wFolderPath->text());
    bool first_slide = true;
    for(auto stack : file_hierarchy) {
        // generate slide
        cout << "Generate slide from stack:" << endl;
        for(auto file_name : stack) {
            cout << "    " << file_name << endl;
        }
        bool first_layer = true;
        for(auto file_name : stack) {
            QFile file(source_dir.filePath(file_name));
            if(!file.exists()) {
                cout << "ERROR: Expected file " << file.fileName() << " to exist" << endl;
                return;
            }
            if(first_layer) {
                // just copy first layer
                first_layer = false;
                copy_overwrite(file, tmp_file_2);
            } else {
                // overlay other layers
                QString command = ui->_wPdfOverlayCommand->text().
                        arg(QFileInfo(tmp_file_1).filePath()).
                        arg(QFileInfo(file).filePath()).
                        arg(QFileInfo(tmp_file_2).filePath());
                system(command.toLatin1());
                INFO(command);
            }
            copy_overwrite(tmp_file_2, tmp_file_1);
        }
        // append slide
        if(first_slide) {
            first_slide = false;
            copy_overwrite(tmp_file_1, tmp_file_3);
        } else {
            copy_overwrite(tmp_file_3, tmp_file_1);
            QString command = ui->_wPdfAppendCommand->text().
                    arg(QFileInfo(tmp_file_1).filePath()).
                    arg(QFileInfo(tmp_file_2).filePath()).
                    arg(QFileInfo(tmp_file_3).filePath());
            system(command.toLatin1());
            INFO(command);
        }
    }
}

QString PdfPresentation::random_string(int length) {
    QString rnd_str;
    QString possible_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    for(int i=0; i<length; ++i) {
        int j = rand()%possible_chars.length();
        rnd_str += possible_chars[j];
    }
    return rnd_str;
}

void PdfPresentation::copy_overwrite(QFile & file, QString path) {
    QFile new_file(path);
    if(new_file.exists()) {
        new_file.remove();
    }
    bool ok = file.copy(path);
    if(!ok) {
        INFO("copy " << file.fileName() << " to " << path << " failed");
    }
    INFO("cp " << file.fileName() << " --> " << path);
}

void PdfPresentation::copy_overwrite(QFile & file, QFile & new_file) {
    copy_overwrite(file,QFileInfo(new_file).filePath());
}

void PdfPresentation::choose() {
    QString old_path;
    if(ui->_wUseFolder->isChecked()) {
        old_path = ui->_wFolderPath->text();
        QString folder_name = QFileDialog::getExistingDirectory(this, tr("Choose folder:"), old_path);
        if(folder_name!="") {
            ui->_wFolderPath->setText(folder_name);
            read_directory();
        }
    } else {
        old_path = ui->_wZipFilePath->text();
        QString base_path = QFileInfo(old_path).absolutePath();
        QString file_name = QFileDialog::getOpenFileName(this, tr("Choose file:"), base_path, "*.zip");
        if(file_name!="") {
            ui->_wZipFilePath->setText(file_name);
            file_name.chop(4);
            ui->_wFolderPath->setText(file_name);
        }
    }
}

void PdfPresentation::extract() {
    // get paths
    QString file_path = ui->_wZipFilePath->text();
    QString folder_path = ui->_wFolderPath->text();
    QDir dir(folder_path);
    // create folder if necessary
    if(!dir.exists()) {
        auto reply = QMessageBox::question(this,
                                           "",
                                           QString("Folder %1 does not exist. Create?").arg(folder_path),
                                           QMessageBox::Yes|QMessageBox::No);
        if(reply==QMessageBox::Yes) {
            dir.mkpath(".");
        } else {
            QMessageBox::information(this, "", "Extraction has been canceled.", QMessageBox::Ok);
            return;
        }
    }
    // clear all pdf files in folder
    dir.setFilter(QDir::Files);
    for(auto file_info : dir.entryInfoList()) {
        QString file_name = file_info.fileName();
        // skip non-pdf files
        if(!file_name.endsWith(".pdf") && !file_name.endsWith(".PDF")) continue;
        QFile(file_info.absoluteFilePath()).remove();
        INFO("rm " << file_name);
    }
    // extract to directory
    system(ui->_wZipCommand->text().arg(file_path).arg(folder_path).toLatin1());
    // create the file hierarchy
    read_directory();
    // save if auto save is checked
    if(ui->_wAutoSaveFile->isChecked()) {
        generate_slides();
        copy_overwrite(tmp_file_3, save_file);
    }
}

bool PdfPresentation::get_save_file_name() {
    // get sensible initial path
    QString path = QFileInfo(save_file).absolutePath();
    if(path=="") {
        path = ui->_wFolderPath->text();
    }
    QString file_name = QFileDialog::getSaveFileName(this, "Save slides as:", path,"*.pdf;; all (*)");
    if(file_name!="") {
        ui->_wSaveFileName->setText(file_name);
        save_file.setFileName(file_name);
    }
    return save_file.fileName()!="";
}

void PdfPresentation::save_as() {
    if(get_save_file_name()) {
        generate_slides();
        copy_overwrite(tmp_file_3, save_file);
    }
}

void PdfPresentation::auto_save(bool do_auto_save) {
    if(do_auto_save && save_file.fileName()=="") {
        bool valid_file = get_save_file_name();
        if(!valid_file) {
            ui->_wAutoSaveFile->setChecked(false);
        }
    }
}

void PdfPresentation::zip_file_name_changed(QString file_name) {
    // clear all (should be only one though)
    file_watcher.removePaths(file_watcher.files());
    if(file_name!="") {
        // add new name
        file_watcher.addPath(file_name);
        // call slot
        zip_file_changed(file_name);
    }
}

void PdfPresentation::zip_file_changed(QString file_name) {
    if(file_name!=ui->_wZipFilePath->text()) {
        cout << "ERROR: watched file (" << file_name << ") is not the zip file (" << ui->_wZipFilePath->text() << ")" << endl;
        return;
    }
    if(ui->_wAutoExtract->isChecked()) {
        zip_file_changed_since_last_call = true;
        if(!extraction_timer->isActive()) {
            INFO("Activating delayed extraction");
            extraction_timer->setSingleShot(false);
            extraction_timer->setInterval(1000);
            extraction_timer->start();
        } else {
            INFO("Delayed extraction already activated");
        }
    }
}

void PdfPresentation::delayed_extraction() {
    if(!zip_file_changed_since_last_call) {
        INFO("File changes settled --> extracting");
        extraction_timer->stop();
        extract();
    } else {
        INFO("Waiting for file changes to settle...");
        zip_file_changed_since_last_call = false;
    }
}
