"""
wrl2ply.py parses wrl files which contain convex sub meshes
(as produced by HACD http://sourceforge.net/projects/hacd/)
and spits out the size of the convex sub meshes.
"""

import sys


def wrlparse(wrlfile):
    data = open(wrlfile).readlines()

    print 'Parsing:'
    vertices = []
    i = 0
    while i < len(data):
        counter = 0
        sys.stdout.write(".")
        if "point [" in data[i]:
            i += 1
            while not data[i].strip().endswith("]"):
                counter += 1
                i += 1
            else:
                vertices.append(counter)
        i += 1

    print ' DONE'
    print "body XXX{ type=3 mesh='%s' submeshsizes=[%s] }" % \
        (wrlfile[:-3] + "ply", " ".join(str(e) for e in vertices))
    return vertices


if __name__ == '__main__':
    if len(sys.argv) == 2:
        print "Specify the wrl file as parameter"
        sys.exit(-1)

    wrlfile = sys.argv[1]
    if not wrlfile.endswith(".wrl"):
        print "Wrong file format. I can only parse wrl files."
        sys.exit(-1)

    vertices = wrlparse(wrlfile)
    print vertices
