## some note
- Y is the end-eff trajectory (which is at the tip of the peg)
- however the coordinate base of the object "peg" is at its center.

## 27-09: Ask Marc
1. whether planeconstraint is also collision contraint? because the peg-in-hole does not want to penetrate that plane constraint as well



#questions
1. How to set optimization without planeconstraint
2. stickyness behaviour is too sticky in online-pomdp 
3. g is gradient information, not sensed forces.
4. out of memory in peg-in-hole experiment.



##########################################333

1. dual-execution: method from MARC's IROS paper 
2. online-pomdp: replace Marc's method by a POMDP
3. peg-in-hole-dual: using Marc's method for peg-in-hole
4. peg-like-pomdp: using POMDP for peg-like domain
5. all start with pr2_xxxx: those algorithms on PR2.


* Submodularity:
1. online-submodular: submodularity approach on the 10-cm-point on table

