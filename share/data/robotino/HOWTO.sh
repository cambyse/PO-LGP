#rosrun xacro xacro.py --inorder /opt/ros/indigo/share/baxter_description/urdf/baxter.urdf.xacro > baxter.urdf
cp robotino.urdf z.urdf
../../bin/urdf2ors.py > z.1.ors
sed 's#package://robotino_description/meshes/\(.*\)\.dae#robotino_description/meshes/\1\.STL#g' z.1.ors > z.2.ors
sed 's#package://##g' z.2.ors > z.3.ors
#cat pr2_before.ors z1-raw.ors pr2_after.ors > z2-augmented.ors
ors_editor -file z.3.ors -cleanOnly
mv z.ors robotino-clean.ors


