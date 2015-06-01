#ifndef PDFPRESENTATION_H
#define PDFPRESENTATION_H

#include <QMainWindow>
#include <QFile>
#include <QFileSystemWatcher>
#include <QTimer>

#include <vector>

namespace Ui {
class PdfPresentation;
}

class PdfPresentation : public QMainWindow
{
    Q_OBJECT

public:
    explicit PdfPresentation(QWidget *parent = 0);
    ~PdfPresentation();

private:
    QFile tmp_file_1, tmp_file_2, tmp_file_3, save_file;
    QFileSystemWatcher file_watcher;
    bool zip_file_changed_since_last_call = false;
    QTimer * extraction_timer;
    Ui::PdfPresentation *ui;
    std::vector<std::vector<QString>> file_hierarchy;
    void read_directory();
    bool same_id_prefix(const std::vector<int> & v1, const std::vector<int> & v2) const;
    void generate_slides();
    static QString random_string(int length);
    static void copy_overwrite(QFile & file, QString path);
    static void copy_overwrite(QFile & file, QFile & new_file);
private slots:
    void choose();
    void extract();
    bool get_save_file_name();
    void save_as();
    void auto_save(bool);
    void zip_file_name_changed(QString file_name);
    void zip_file_changed(QString file_name);
    void delayed_extraction();
};

#endif // PDFPRESENTATION_H
