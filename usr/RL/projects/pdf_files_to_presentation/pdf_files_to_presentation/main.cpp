#include <iostream>
#include <vector>
#include <tuple>

#include <QString>
#include <QDir>

using namespace std;

QString usage_string = "Usage: pdf_files_to_presentation <directory> <output.pdf>";

//#define COMMAND(x) cout << x << endl;
#define COMMAND(x) system(x.toLatin1());

std::ostream& operator<<(std::ostream &out, const QString& s) {
    out << (const char*)s.toLatin1();
    return out;
}

int main(int argn, char ** args) {

    // check number of args
    if(argn<3) {
        cout << usage_string << endl;
        return 0;
    }

    // get directory and output file
    QDir dir(args[1]);
    if(!dir.exists()) {
        cout << "First argument must be a valid directory" << endl;
        cout << usage_string << endl;
        return 0;
    }
    QString working_dir(args[1]);
    QString output_file(args[2]);
    cout << "Working in directory: " << dir.path() << endl;
    cout << "Output file: " << output_file << endl;
    if(dir.exists(output_file)) {
        cout << "First removing output file" << endl;
        COMMAND(QString("rm %1/%2").arg(working_dir).arg(output_file));
    }

    // iterate through files
    dir.setSorting(QDir::LocaleAware);
    dir.setFilter(QDir::Files);
    vector<pair<QString,QString>> active_files;
    auto file_list = dir.entryInfoList();
    for(int idx=0; idx<file_list.size(); ++idx) {
        // get file name
        QString file_name = file_list[idx].fileName();
        // skip file not ending on *.pdf
        if(!file_name.endsWith(".pdf")) {
            cout << "Skipping file " << file_name << endl;
            continue;
        } else {
            cout << "Processing file " << file_name << endl;
        }
        // get prefix
        QString file_prefix = file_name.split("_")[0];
        // remove all stack files that don't have common prefix
        while(!active_files.empty()) {
            if(!file_prefix.startsWith(active_files.back().first)) {
                active_files.pop_back();
            } else {
                break;
            }
        }
        // add file to queue
        active_files.push_back(make_pair(file_prefix,file_name));
        // peek ahead to see if it's purely a background file
        if(idx<file_list.size()-1 && file_list[idx+1].fileName().startsWith(file_prefix)) {
            cout << "--> skipping background file" << endl;
            continue;
        }
        // combine
        bool first = true;
        for(auto stack_file : active_files) {
            if(first) {
                first = false;
                COMMAND(QString("pdftk %1/%2 output %1/%3.new").arg(working_dir).arg(stack_file.second).arg(output_file));
            } else {
                COMMAND(QString("pdftk %3/%1 background %3/%2.old output %3/%2.new").arg(stack_file.second).arg(output_file).arg(working_dir));
            }
            cout << "        " << stack_file.second << " (prefix: " << stack_file.first << ")" << endl;
            COMMAND(QString("cp %3/%1.new %3/%1.old").arg(output_file).arg(working_dir));
        }
        // append to output
        if(dir.exists(output_file)) {
            COMMAND(QString("pdftk %3/%1 %3/%1.old output %3/%1.new").arg(output_file).arg(working_dir));
        }
        COMMAND(QString("cp %3/%1.new %3/%1").arg(output_file).arg(working_dir));
    }
    COMMAND(QString("rm %3/%1.old %3/%1.new").arg(output_file).arg(working_dir));

    return 0;
}
