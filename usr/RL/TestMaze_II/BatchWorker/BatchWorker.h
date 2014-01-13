#ifndef BATCHWORKER_H_
#define BATCHWORKER_H_

#include <tclap/CmdLine.h>

#include <QString>

class BatchWorker {

public:

    BatchWorker(int argc, char ** argv);

    virtual ~BatchWorker() = default;

    void collect_data();

private:

    // the command line arguments
    TCLAP::ValueArg<std::string> learner_arg;
    TCLAP::ValueArg<int> minT_arg;
    TCLAP::ValueArg<int> maxT_arg;
    TCLAP::ValueArg<int> incT_arg;
    TCLAP::ValueArg<double> epsilon_arg;
    TCLAP::ValueArg<double> discount_arg;
    TCLAP::ValueArg<int> tree_arg;
    TCLAP::ValueArg<double> l1_arg;
    TCLAP::SwitchArg pruningOff_arg;
    TCLAP::ValueArg<int> incF_arg;
    TCLAP::ValueArg<double> delta_arg;

};

#endif /* BATCHWORKER_H_ */
