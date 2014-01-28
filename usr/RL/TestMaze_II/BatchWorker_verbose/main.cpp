#include <time.h>
#include <stdlib.h>

#include "../BatchWorker.h"

int main(int argc, char *argv[]) {

    // seed random generator
#ifdef NO_RANDOM
    srand(0);
    srand48(0);
#else
    srand(time(nullptr));
    srand48(time(nullptr));
#endif

    BatchWorker batch_worker(argc,argv);
    batch_worker.collect_data();

    return 0;

}
