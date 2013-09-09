#ifndef BATCHMAZE_H_
#define BATCHMAZE_H_

#include "Config.h"

#include <tuple>
#include <map>
#include <vector>
#include <fstream>

class QString;

class BatchMaze {
public:
    USE_CONFIG_TYPEDEFS;
    BatchMaze();
    virtual ~BatchMaze();
    int run(int argn, char ** argarr);
private:

    struct switch_t {
        switch_t(QString, QString, QString, QString);
        QString switch_string;
        QString type;
        QString default_value;
        QString help_description;
    };

    /** \brief Available modes for running BatchMaze. */
    static const std::vector<QString> mode_vector;

    /** \brief The mode BatchMaze is run in. */
    QString mode;

    /** \brief Available sampling methods. */
    static const std::vector<QString> sample_method_vector;

    /** \brief The method how BatchMaze draws samples. */
    QString sample_method;

    /** \brief Contains the command line switches with default value, type, and
     * help description. */
    static const std::vector<switch_t> switch_vector;

    /** \brief Stores command line switches of type <int> with their actual value. */
    std::map<QString,int> int_switches;

    /** \brief Stores command line switches of type <double> with their actual value. */
    std::map<QString,double> double_switches;

    /** \brief Stores command line switches of type <bool> with their actual value. */
    std::map<QString,bool> bool_switches;

    /** \brief Stores command line switches of type <char> with their actual value. */
    std::map<QString,QString> char_switches;

    /** \brief String holding the date an time for unique output file naming. */
    QString date_time_string;

    /** \brief The output file data is written to. */
    std::ofstream log_file;

    /** \brief Parse the command line switches. */
    void parse_switches(int argn, char ** argarr);

    /** \brief Return the value of a command line switch of type int. */
    int switch_int(QString) const;

    /** \brief Return the value of a command line switch of type double. */
    double switch_double(QString) const;

    /** \brief Return the value of a command line switch of type bool. */
    bool switch_bool(QString) const;

    /** \brief Return the value of a command line switch of type char*. */
    QString switch_char(QString) const;

    /** \brief Run BatchMaze by actively sampling data. */
    int run_active();

    /** \brief Print a help text. */
    void print_help();

    /** \brief Set the log file name and write the header with general information. */
    void initialize_log_file();

    /** \brief Generate samples with uniform spacing. */
    void generate_uniform_samples(std::vector<int>& sample_vector) const;

    /** \brief Generate samples with exponential spacing. */
    void generate_exp_samples(std::vector<int>& sample_vector) const;
};

#endif /* BATCHMAZE_H_ */
