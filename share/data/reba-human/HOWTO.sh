cp human.urdf z.urdf
../../bin/urdf2ors.py > z.1.ors
ors_editor -file z.1.ors -cleanOnly
mv z.ors human-clean.ors


