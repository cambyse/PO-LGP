"""
Simple script to decompose a ply file in its convex parts.

Make sure meshconv and hacd_decomposer are in your PATH.

What happens:
- convert the file into the .off format
- decompose the off file and spit out a wrl file
- convert wrt to ply
- parse wrt and extract infos about sub-grops for the ors file
"""

import sys
import os
import envoy
import wrlparse


def main(f):
    cmds = ["meshconv %s -c off" % f,
            "hacd_decomposer %s 2 100 0 1 1 30 2000" % (f[:-4] + '.off'),
            "meshconv %s -c ply -kd" % (f[:-4] + '_hacd.wrl')
            ]
    for cmd in cmds:
        print cmd
        envoy.run(cmd)
        print '=' * 79

    f_wrl = f[:-4] + "_hacd.wrl"
    wrlparse.wrlparse(f_wrl)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "Specify the file to decompose."
        sys.exit()
    if not os.path.exists(sys.argv[1]):
        print "File does not exist."
        sys.exit()

    main(sys.argv[1])
