#!/usr/bin/env python
# encoding: utf-8

"""
Decompose non convex meshes into convex sub meshes.
"""

from __future__ import print_function
import argparse
import envoy


def main(args):
    filename = args.filename
    filename_off = filename[:-4] + ".off"

    cmds = [
        # convert to .off
        "./meshconv {} -c off".format(filename),
        # convex decomposition of .off file into .obj file
        "./HACD {}".format(filename_off),
    ]

    for cmd in cmds:
        print("=" * 70)
        print(cmd)
        r = envoy.run(cmd)
        if args.verbose:
            print(r.std_out)
        print("Status Code: ", r.status_code)


if __name__ == '__main__':
    # ARGPARSE
    descr = """Chaining a few function calls to convexify a mesh file, i.e., to
               convert a mesh file into a mesh file which contists only of
               convex sub meshes which then can be read by ors."""
    parser = argparse.ArgumentParser(description=descr)
    parser.add_argument('filename', metavar='filename', type=str,
                        help='the mesh file to convert')
    parser.add_argument("-v", "--verbose", dest="verbose",
                        action="store_true", default=False,
                        help="Verbose printing.")
    args = parser.parse_args()

    # MAIN
    main(args)
