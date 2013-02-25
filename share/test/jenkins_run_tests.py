#!/usr/bin/env python2

"""
Build all tests in test/ and create a JUnit compatible result
file which then can be parsed by Jenkins' junit publisher.


Dependencies
============
- envoy: https://github.com/kennethreitz/envoy


Resources
=========
- http://stackoverflow.com/questions/4922867/junit-xml-format-specification-that-hudson-supports
- http://pymotw.com/2/xml/etree/ElementTree/create.html
"""

from xml.etree.ElementTree import Element
from xml.etree.ElementTree import SubElement
from xml.etree.ElementTree import tostring
from xml.dom import minidom

import os
import envoy


def prettyprintXML(elem):
    """ Return a pretty-printed XML string for the Element. """
    rough_string = tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def main():
    root = Element("testsuite")
    for filename in os.listdir('.'):
        if os.path.isdir(filename):
            print filename
            r = envoy.run("make -C %s" % filename)
            testcase = SubElement(root, "testcase",
                                  {"classname": "test", "name": filename})
            if r.status_code != 0:
                failure = SubElement(testcase, "failure", {"type": "error"})
                print '\t', "status_code:", r.status_code
                # failure.text =  r.std_err
                failure.text = "TODO add compile error"

    s = prettyprintXML(root)
    open("test_run.xml", "w").write(s)


if __name__ == '__main__':
    main()
