#include "BatchWorker.h"

#include "../debug.h"

BatchWorker::BatchWorker(int argc, char ** argv):
    learner_arg(       "l", "learner"      , "learner to use"                               , true,        "",  "string"),
    minT_arg(           "", "minT"         , "minimum number of training samples"           , true,        10,     "int"),
    maxT_arg(           "", "maxT"         , "maximum number of training samples"           ,false,        -1,     "int"),
    incT_arg(           "", "incT"         , "increment of training samples"                ,false,        -1,     "int"),
    epsilon_arg(       "e", "epsilon"      , "epsilon (randomness)"                         ,false,       0.0,  "double"),
    discount_arg(      "d", "discount"     , "discount"                                     ,false,       0.5,  "double"),
    tree_arg(          "t", "tree"         , "maximum size of search tree"                  ,false,     10000,     "int"),
    l1_arg(             "", "l1"           , "L1-regularization factor"                     ,false,     0.001,  "double"),
    pruningOff_arg(    "p", "pruningOff"   , "whether to turn off pruning the search tree"  ,           false           ),
    incF_arg(          "f", "incF"         , "how many candidate features to include"       ,false,        50,     "int"),
    delta_arg(         "D", "delta"        , "minimum change of data likelihood"            ,false,     0.001,  "double")
{
    try {
	TCLAP::CmdLine cmd("This program is BatchWorker. It collects data.", ' ', "");

        cmd.add(delta_arg);
        cmd.add(incF_arg);
        cmd.add(pruningOff_arg);
        cmd.add(l1_arg);
        cmd.add(tree_arg);
        cmd.add(discount_arg);
        cmd.add(epsilon_arg);
        cmd.add(incT_arg);
        cmd.add(maxT_arg);
        cmd.add(minT_arg);
        cmd.add(learner_arg);

	// TCLAP::ValueArg<std::string> nameArg("n","name","Name to print",false,"homer","string");
	// cmd.add( nameArg );
	// TCLAP::SwitchArg reverseSwitch("r","reverse","Print name backwards", cmd, false);

	// Parse the argv array (throws execption in case of failure)
	cmd.parse( argc, argv );

    } catch (TCLAP::ArgException &e) {
        // catch any exceptions
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    }
}

void BatchWorker::collect_data() {

}
