from lxml import etree

inFile = "pr2.urdf" 
xmlData = etree.parse(inFile)

links = xmlData.findall("//link") 

for link in links:
    name = link.attrib['name']
    mass = link.find("inertial/mass")
    if mass!=None:
        mass_val = mass.attrib['value']
    else:
        mass_val = -1
    origin = link.find("collision/origin")
    if origin!=None:
        pos = origin.attrib['xyz']
        ori = origin.attrib['rpy']
    else:
        pos = -1
        ori = -1
    mesh = link.find("collision/geometry/mesh")
    if mesh != None:
        meshfile = mesh.attrib['filename']
    else:
        meshfile = "NONE"
    box = link.find("collision/geometry/box")
    if box != None:
        box_size = box.attrib['size']
    else:
        box_size = -1
    sphere = link.find("collision/geometry/sphere")
    if sphere != None:
        sphere_rad = sphere.attrib['radius']
    else:
        sphere_rad = -1
 
    print  'link %s {\n\tmass=%s\n\torigin=[%s]\n\torientation=[%s]\n\tmesh="%s"\n\tbox=[%s]\n\tsphere=%s\n}\n'%(name, mass_val, pos, ori, meshfile, box_size, sphere_rad)

joints = xmlData.findall("//joint") 
for joint in joints:
    name = joint.attrib['name']
    typ = joint.attrib.get('type')
    origin = joint.find("origin")
    if origin!=None:
        pos = origin.attrib['xyz']
        ori = origin.attrib.get('rpy')
    if joint.find("child")!=None:
        fr = joint.find("child").attrib['link']
        to = joint.find("parent").attrib['link']
    print  'joint %s (%s %s) {\n\ttype=%s\n\tpos=[%s]\n\tori=[%s]\n}\n'%(name, fr, to, typ, pos, ori)

#print(etree.tostring(links[22]))
#print(etree.tostring(joints[0]))
