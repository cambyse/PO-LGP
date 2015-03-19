#ifndef VALUEHEURISTIC_H_
#define VALUEHEURISTIC_H_

namespace value_heuristic {

    /**
     * Abstract basis class for heuristics that initialize the return of new
     * leaf nodes. For problems with a single reward at a terminal state (like
     * the games of chess or go) this typically is a CompleteRollout. In the
     * case of intermediate reward this may also be a PartialRollout of fixed
     * length \e k or OneStep rollout. */
    class ValueHeuristic {
        //----typedefs/classes----//

        //----members----//
    private:
        //----methods----//
    public:
    ValueHeuristic() {}
    };

} // end namespace value_heuristic

#endif /* VALUEHEURISTIC_H_ */
