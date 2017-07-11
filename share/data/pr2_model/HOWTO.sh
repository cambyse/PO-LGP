cp $1 z.urdf
../../bin/urdf2ors.py > z1-raw.ors
sed 's/package:\/\/pr2_description\/meshes\///g' z1-raw.ors > z2-paths.ors
sed 's/\.dae/\.stl/g' z2-paths.ors > z2-paths2.ors
sed 's/\.STL/\.stl/g' z2-paths2.ors > z2-paths3.ors
cat pr2_before.ors z2-paths3.ors pr2_coll.g pr2_after.ors > z3-augmented.ors
../../bin/kinEdit -file z3-augmented.ors -cleanOnly
mv z.ors z4-clean.ors

