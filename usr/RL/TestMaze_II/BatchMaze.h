#ifndef BATCHMAZE_H_
#define BATCHMAZE_H_

#include "Data.h"
#include "Representation/Representation.h"

class BatchMaze {
public:
    USE_DATA_TYPEDEFS;
    USE_REPRESENTATION_TYPEDEFS;
    BatchMaze();
    virtual ~BatchMaze();
    int run(int argn, char ** argarr);
private:
    int file_id;
};

#endif /* BATCHMAZE_H_ */
