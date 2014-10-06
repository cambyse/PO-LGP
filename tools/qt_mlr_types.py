python

import math

def qdump__MT__String(d, value):
    p = value["p"]
    N = value["N"]
    s = "'"
    i = 0
    while(i<N):
        s += "%c" % int((p+i).dereference())
        i = i+1
    s += "' [%i]" % N
    d.putValue(s)
    d.putNumChild(2)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("N", N)
            d.putSubItem("p", p)


def qdump__LIST(d, value):
    p = value["p"]
    N = value["N"]
    s = "<%i>" %N
    d.putValue(s)
    m=N
    if m>10:
        m=10
    d.putNumChild(m+1)
    if d.isExpanded():
        with Children(d):
            i=0
            while (i<m):
                s = "(%i)" %i
                d.putSubItem(s, (p+i).dereference().dereference())
                i = i+1
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
    d.putValue(s)
    m=N
    if m>10:
        m=10
    d.putNumChild(m+4)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("N", N)
            i=0
            while (i<m):
                if nd==1:
                    s = "(%i)" %(i)
                if nd==2:
                    s = "(%i,%i)"%(i/d1,i%d1)
                if nd==3:
                    s = "(%i,%i,%i)"%(i/d1,(i/d2)%d1,i%d2)
                d.putSubItem(s, (p+i).dereference())
                i = i+1
            d.putSubItem("p", p)
            d.putSubItem("reference", value["reference"])
            d.putSubItem("special", value["special"])
            d.putSubItem("aux", value["aux"])
            
def qdump__Item_typed(d, value):
    keys_N = value["keys"]["N"]
    keys_p = value["keys"]["p"]
    s = "'"
    i = 0
    while(i<keys_N):
        string = (keys_p+i).dereference()
        string_N = string["N"]
        string_p = string["p"]
        j = 0
        while(j<string_N):
            s += "%c" % int((string_p+j).dereference())
            j = j+1
        i = i+1
        if(i<keys_N):
            s += " "
    s += "'"
    d.putValue(s)
    d.putNumChild(4)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("value", value["value"].dereference())
            d.putSubItem("parents", value["parents"])
            d.putSubItem("parentOf", value["parentOf"])
            d.putSubItem("index", value["index"])

def qdump__ItemL(d, value):
    qdump__LIST(d,value)

def qdump__KeyValueGraph(d, value):
    p = value["p"]
    N = value["N"]
    s = "<%i>" %N
    d.putValue(s)
    m=N
    if m>10:
        m=10
    d.putNumChild(m+1)
    if d.isExpanded():
        with Children(d):
            i=0
            while (i<m):
                s = "(%i)" %i
                d.putSubItem(s, (p+i).dereference().dereference())
                i = i+1
            d.putSubItem("p", p)
            d.putSubItem("isItemOfParentKvg", value["isItemOfParentKvg"])

def qdump__BodyL(d, value):
    qdump__LIST(d,value)

def qdump__ShapeL(d, value):
    qdump__LIST(d,value)

def qdump__JointL(d, value):
    qdump__LIST(d,value)

def qdump__ProxyL(d, value):
    qdump__LIST(d,value)

def qdump__ors__Vector(d, value):
    x=float(value["x"]); y=float(value["y"]); z=float(value["z"])
    s = "[%f %f %f]" % (x,y,z)
    d.putValue(s)
    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            with SubItem(d, "length"):
                d.putValue(math.sqrt(x*x+y*y+z*z))
                d.putType("float")
                d.putNumChild(0)

def qdump__ors__Quaternion(d, value):
    w=float(value["w"]); x=float(value["x"]);
    y=float(value["y"]); z=float(value["z"])
    s = "[%f %f %f %f]" % (w,x,y,z)
    d.putValue(s)
    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            with SubItem(d, "degrees"):
                d.putValue(360.0/math.pi*math.acos(w))
                d.putType("float")
                d.putNumChild(0)

end
