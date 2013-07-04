#ifndef BATCHMAZE_H_
#define BATCHMAZE_H_

#include "Config.h"

class BatchMaze {
public:
    USE_CONFIG_TYPEDEFS;
    BatchMaze();
    virtual ~BatchMaze();
    int run(int argn, char ** argarr);
private:
    int file_id;
    void print_help();
    void parse_command_line_arguments(int argn, char ** argarr);
    void initialize_log_file();
    void precompute_training_lengths(int * & training_lengths, int & training_steps);
};

#endif /* BATCHMAZE_H_ */
