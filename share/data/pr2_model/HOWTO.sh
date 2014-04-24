cp $1 z.urdf
./urdf2ors.py > z1-raw.ors
#sed 's/mesh="pr2_model/mesh="/g' z.ors > pr2-1-org.ors
ors_editor -file z1-raw.ors -cleanOnly
mv z.ors z2-clean.ors

