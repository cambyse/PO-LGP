##IMPLEMENTATION Notes

## 15.12.2014

* there is no notion of UCT tree. we are using a list caching all visited states. This will be very inefficient in large problems.

* How to build the set {D \union w}.  w is the decision to call termination. It's easy to find the probability of w. At each step, we're choosing d or w, we compute all possible combinatoric set $M$ of non-mutex actions. Then, w is the case when we add no action into the current list, then probability of w is 1/M.

## 14.12
- I commented this line: rules_ground.sanityCheck(); in rules.cpp


* 12.12.2014
- pickup(X Y) can be added later in the code? as we would have 5*3*4 (4 is the number of actions) = 48 terms
- should we assume that all functions are initialized to 0.
