python

def qdump__MT__String(d, value):
    p = value["p"]
    N = value["N"]
    s = ""
    for i in xrange(N):
        s += "%c" % int(p.dereference())
        p += 1
    d.putValue("'%s' [%i]" % (s,N))

    d.putNumChild(2)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("N", N)
            d.putSubItem("p", p)

def qdump__MT__Array(d, value):
    p = value["p"]
    N = value["N"]
    nd = value["nd"]
    d0 = value["d0"]
    d1 = value["d1"]
    d2 = value["d2"]
    if nd==0:
        s = "<>"
    if nd==1:
        s = "<%i>" %d0
    if nd==2:
        s = "<%i %i>"%(d0,d1)
    if nd==3:
        s = "<%i %i %i>"%(d0,d1,d2)
    # for i in xrange(N):
    #     s += str(p.dereference()) + " "
    #     p += 1
    # s += "]"
    d.putValue(s)
    m=N
    if m>10:
        m=10
    d.putNumChild(m+4)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("N", N)
            for i in xrange(m):
                if nd==1:
                    s = "(%i)" %(i)
                if nd==2:
                    s = "(%i,%i)"%(i/d1,i%d1)
                if nd==3:
                    s = "(%i,%i,%i)"%(i/d1,(i/d2)%d1,i%d2)
                d.putSubItem(s, (p+i).dereference())
            d.putSubItem("p", p)
            d.putSubItem("reference", value["reference"])
            d.putSubItem("special", value["special"])
            d.putSubItem("aux", value["aux"])
            

end
