python

def qdump__MT__String(d, value):
    p = value["p"]
    N = value["N"]
    s = ""
    for i in xrange(N):
        s += "%c" % int(p.dereference())
        p += 1
    d.putValue("'%s' [%i]" % (s,N))

    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("N", N)

def qdump__MT__Array(d, value):
    p = value["p"]
    N = value["N"]
    nd = value["nd"]
    d0 = value["d0"]
    d1 = value["d1"]
    d2 = value["d2"]
    s = "[ "
    for i in xrange(N):
        s += str(p.dereference()) + " "
        p += 1
    s += "]"
    d.putValue(s)
    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            if nd==0:
                d.putSubItem("<>", N)
            if nd==1:
                d.putSubItem("<%i>"%d0, N)
            if nd==2:
                d.putSubItem("<%i %i>"%(d0,d1), N)
            

end
