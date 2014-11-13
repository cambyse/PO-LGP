#ifndef BATCHWORKER_H_
#define BATCHWORKER_H_

#include "../Config.h"

#include <tclap/CmdLine.h>

#include <memory> // for shared_ptr
#include <fstream> // for ofstream

#include <omp.h>

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

    /** Perform multiple runs specified by command line arguments. */
    void collect_data();

private:

    // the command line arguments
    std::string mode;
    TCLAP::ValueArg<std::string> mode_arg;         ///< mode to use
    TCLAP::ValueArg<std::string> environment_arg;  ///< environment to use
    int minT;
    TCLAP::ValueArg<int> minT_arg;                 ///< minimum number of training samples
    int maxT;
    TCLAP::ValueArg<int> maxT_arg;                 ///< maximum number of training samples
    int incT;
    TCLAP::ValueArg<int> incT_arg;                 ///< increment of training samples
    TCLAP::ValueArg<int> eval_arg;                 ///< length of evaluation episode
    TCLAP::ValueArg<int> repeat_arg;               ///< how many times to repeat everything
    TCLAP::ValueArg<double> discount_arg;          ///< discount
    TCLAP::ValueArg<int> tree_arg;                 ///< maximum size of search tree
    TCLAP::ValueArg<double> l1_arg;                ///< L1-regularization factor
    TCLAP::SwitchArg pruningOff_arg;               ///< whether to turn off pruning the search tree
    TCLAP::ValueArg<int> minH_arg;                 ///< minimum horizon
    TCLAP::ValueArg<int> maxH_arg;                 ///< maximum horizon
    TCLAP::ValueArg<int> extH_arg;                 ///< horizon extension
    TCLAP::ValueArg<double> delta_arg;             ///< minimum change of data likelihood/TD-error
    TCLAP::ValueArg<int> maxCycles_arg;            ///< maximum number of grow-shrinc cycles
    TCLAP::ValueArg<int> minCycles_arg;            ///< minimum number of grow-shrinc cycles (negative values to force immediate full expansion
    TCLAP::ValueArg<double> epsilon_arg;           ///< epsilon / randomness of transitions
    TCLAP::ValueArg<int> button_n_arg;             ///< number of buttons in button world
    TCLAP::ValueArg<double> button_alpha_arg;      ///< for beta dist. per button in button world
    TCLAP::ValueArg<double> utree_threshold_arg;   ///< threshold for expansion of UTree
    double utree_threshold;

    bool args_ok = false;

    /** Check if arguments are in range etc. */
    bool post_process_args();

    /** Perform random transition with given environment and observer. */
    void collect_random_data(
        std::shared_ptr<Environment> env,
        std::shared_ptr<HistoryObserver> obs,
        const int& length,
        instance_ptr_t& i);

    /** Train TemporallyExtendedModel */
    void train_TEM(std::shared_ptr<HistoryObserver> learner, double& likelihood, int& features, int& cycles);

    /** Train TemporallyExtendedLinearQ */
    void train_TEL(std::shared_ptr<HistoryObserver> learner, double& TD_error, int& features, int& cycles);

    /** Train value-based UTree */
    void train_value_based_UTree(std::shared_ptr<HistoryObserver> learner, int& size, double& score);

    /** Train model-based UTree */
    void train_model_based_UTree(std::shared_ptr<HistoryObserver> learner, int& size, double& score);

    /** \brief Set the log file name and write the header with general information. */
    void initialize_log_file(std::ofstream& log_file);


    /** Function for coordinating learning and evaluation. Initialize all locks
     * to number of threads if vector has zero size. */
    void init_all_learn_locks(std::vector<omp_lock_t> & locks);

    /** Function for coordinating learning and evaluation. Lock all threads for
     * learning. */
    void set_all_learn_locks(std::vector<omp_lock_t> & locks);

    /** Function for coordinating learning and evaluation. Unlock all threads
     * after learning. */
    void unset_all_learn_locks(std::vector<omp_lock_t> & locks);

    /** Function for coordinating learning and evaluation. Lock this thread for
     * a planning step so learners must wait. This function waits 100ms before
     * locking so a learner can grab the thread. */
    void set_this_learn_lock(std::vector<omp_lock_t> & locks);

    /** Function for coordinating learning and evaluation. Unlock this thread
     * after a planning step. */
    void unset_this_learn_lock(std::vector<omp_lock_t> & locks);

    /** Function for coordinating learning and evaluation. Destroy all lock and
     * clear vector. */
    void destroy_all_learn_locks(std::vector<omp_lock_t> & locks);
};

#endif /* BATCHWORKER_H_ */
