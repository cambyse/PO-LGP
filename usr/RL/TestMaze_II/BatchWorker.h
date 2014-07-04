#ifndef BATCHWORKER_H_
#define BATCHWORKER_H_

#include "../Config.h"

#include <tclap/CmdLine.h>

#include <memory> // for shared_ptr
#include <fstream> // for ofstream

class HistoryObserver;
class Environment;
class HistoryObserver;

class BatchWorker {

public:

    // use standard typedefs
    USE_CONFIG_TYPEDEFS;

    static const std::vector<std::string> mode_vector;

    BatchWorker(int argc, char ** argv);

    virtual ~BatchWorker() = default;

    void collect_data();

private:

    // the command line arguments
    std::string mode;
    TCLAP::ValueArg<std::string> mode_arg;
    int minT;
    TCLAP::ValueArg<int> minT_arg;
    int maxT;
    TCLAP::ValueArg<int> maxT_arg;
    int incT;
    TCLAP::ValueArg<int> incT_arg;
    TCLAP::ValueArg<int> eval_arg;
    TCLAP::ValueArg<int> repeat_arg;
    TCLAP::ValueArg<double> discount_arg;
    TCLAP::ValueArg<int> tree_arg;
    TCLAP::ValueArg<double> l1_arg;
    TCLAP::SwitchArg pruningOff_arg;
    TCLAP::ValueArg<int> minH_arg;
    TCLAP::ValueArg<int> maxH_arg;
    TCLAP::ValueArg<int> extH_arg;
    TCLAP::ValueArg<double> delta_arg;
    TCLAP::ValueArg<int> maxCycles_arg;

    bool args_ok = false;

    bool post_process_args();

    void collect_random_data(
        std::shared_ptr<Environment> env,
        std::shared_ptr<HistoryObserver> obs,
        const int& length,
        instance_ptr_t i);

    void train_TEM(std::shared_ptr<HistoryObserver> learner, double& likelihood, int& features);
    void train_TEL(std::shared_ptr<HistoryObserver> learner, double& TD_error, int& features);
    void train_value_based_UTree(std::shared_ptr<HistoryObserver> learner, int& size, double& score);
    void train_model_based_UTree(std::shared_ptr<HistoryObserver> learner, int& size, double& score);

    /** \brief Set the log file name and write the header with general information. */
    void initialize_log_file(std::ofstream& log_file);
};

#endif /* BATCHWORKER_H_ */
