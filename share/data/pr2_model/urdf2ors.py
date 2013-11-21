#!/usr/bin/python

from lxml import etree

inFile = "pr2.urdf" 
xmlData = etree.parse(inFile)

links = xmlData.findall("/link") 
for link in links:
    name = link.attrib['name']
    print 'body %s {'%name

    elem = link.find("inertial/mass")
    if elem != None:
        print '\tmass=%s'%elem.attrib['value']

    elem = link.find("collision/origin")
    if elem != None:
        print '\trel=<T t(%s) E(%s)>'%(elem.attrib['xyz'],elem.attrib['rpy'])

    elem = link.find("collision/geometry/box")
    if elem != None:
        print '\ttype=0\n\tsize=[%s 0]'%elem.attrib['size']

    elem = link.find("collision/geometry/sphere")
    if elem != None:
        print '\ttype=1 size=[0 0 0 %s]'%elem.attrib['radius']

    elem = link.find("collision/geometry/cylinder")
    if elem != None:
        print '\ttype=2 size=[0 0 %s %s]'%(elem.attrib['length'],elem.attrib['radius'])

    elem = link.find("collision/geometry/mesh")
    if elem != None:
        meshfile = elem.attrib['filename']
        meshfile = meshfile.replace("package://pr2_description/meshes","pr2_model");
        print '\ttype=3\n\tmesh="%s"'%meshfile

    print  '}\n'

joints = xmlData.findall("/joint") 
for joint in joints:
    name = joint.attrib['name']
    if joint.find("child")!=None:
        print 'joint %s (%s %s) {'%(name, joint.find("parent").attrib['link'],joint.find("child").attrib['link'])

        # figure out joint type
        att = joint.attrib.get('type')
        if att=="revolute" or att=="continuous":
            print '\ttype=0'
        if att=="prismatic":
            print '\ttype=3'
        if att=="fixed":
            print '\ttype=10'

        elem = joint.find("mimic")
        if elem!=None:
            print '\tmimic=%s'%elem.attrib['joint']

        elem = joint.find("axis")
        if elem!=None:
            print '\taxis=[%s]'%elem.attrib['xyz']

        elem = joint.find("origin")
        if elem!=None:
            att=elem.attrib.get('rpy')
            if att!=None:
                print '\tA=<T t(%s) E(%s)>'%(elem.attrib['xyz'],att)
            else:
                print '\tA=<T t(%s)>'%(elem.attrib['xyz'])

        print  '}\n'

#print(etree.tostring(links[22]))
#print(etree.tostring(joints[0]))
