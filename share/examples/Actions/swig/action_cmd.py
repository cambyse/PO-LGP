#!/usr/bin/env python

from __future__ import print_function
import cmd
import random
import sys
import os
import time
sys.path.append(os.path.abspath('../../../src/Actions'))
import swig


class ActionCmd(cmd.Cmd):
    prompt = "pr2> "
    last_output = ''
    intro = "Pr2 cmd start"

    def __init__(self):
        cmd.Cmd.__init__(self)
        self.shapeL = S.getShapeList()
        self.bodyL = S.getBodyList()
        self.jointL = S.getJointList()
        self.parameters =({"type":"pos", "ref1":"endeffL", "target":"[0.5, 0.0, 0.0]","PD" :"[.5, .9, .1, 10.]"})
        self.facts =  S.getFacts()
        self.actions = []


    def cmdloop (self):
        cmd.Cmd.cmdloop(self)
        return "true"

    def do_close(self, para):
        self.parameters = ({"type":"qItself", "ref1":"l_gripper_joint", "target":"[0.05]", "PD" :"[.5, .9, .1, 10.]"})
        self.do_newTask("close")
        self.do_startAction("close")

    def do_open(self, para):
        self.parameters = ({"type":"qItself", "ref1":"l_gripper_joint", "target":"[0.2]", "PD" :"[.5, .9, .1, 10.]"})
        self.do_newTask("open")
        self.do_startAction("open")



    def do_changePara(self, par):
        p = par.split()
        if len(p) == 2:       
            self.parameters[p[0]] = p[1]


    def do_getShapeByName (self, obj):
        """get literal of shape by name"""
        shape = S.getShapeByName(obj)
        print("name: " + shape["name"])
        print("pos: " + shape["pos"])
        print("Quaterion: " + shape["Q"])
        print("Type: " + shape["type"])
        #return body

    def do_getBodyByName (self, obj):
        """get literal of body by name"""
        body = S.getBodyByName(obj)
        print("name: " + body["name"])
        print("pos: " + body["pos"])
        print("Quaterion: " + body["Q"])
        print("Type: " + body["type"])
        #return body

    def do_getShapeList (self, xxx):
        """get list of available shapes"""
        shapeL = S.getShapeList()
        print (shapeL)

    def do_getBodyList (self, xxx):
        """get list of available bodies"""
        bodyL = S.getBodyList()
        print(bodyL)


    def do_getJointList (self, xxx):
        """get list of available joints"""
        jointL = S.getJointList()
        print(jointL)

    def do_print(self, xxx):
        print (self.parameters)
        
    def do_newTask (self, name):
        """define new task space control action
        argument order:
        1.: type 2.: name, 3.: effector, 4.: target."""
        p = name.split()
        name = "task" 
        reference = ["FollowReferenceActivity"]

        for i in range(0,len(p) ):
            if i == 0:
                name = p[i]
            elif i == 1:
                self.parameters["type"] = p[i]
            elif i == 2:
                self.parameters["ref1"] = p[i]
            elif i == 3:
                do_changePara("target" + p[i])
            elif i == 4:
                self.parameters["PD"] = p[i]
            elif i == 5:
                self.parameters["ref2"] = p[i]
        
        S.defineNewTaskSpaceControlAction(name, reference, self.parameters)
        #S.startActivity([name])
        self.actions.append(name)

    def do_startAction (self, name):
        """start defined task"""
        if name == "":
            name = "task"
        S.startActivity([name])


    def do_stopAction (self, name):
        """stop defined task"""
        if name == "":
            name = "task"
        S.stopActivity([name])

    def do_start(self,name):
        for i in range(0,len(self.actions)):
            self.do_startAction(self.actions[i])

    def do_stop(self, name):
        """stop everything"""
        self.do_getFacts("")
        for i in range(0,len(self.facts)):
            hString = self.facts[i]
            hString = hString.replace("(", "")
            hString = hString.replace("),", "")
            self.do_stopAction(hString)

    def do_getFacts(self, name):
        """get facts"""
        self.facts = S.getFacts()
        print (S.getFacts())


    def complete_newTask(self, text, line, begidx, endidx):
        if not text:
            completions = self.shapeL[:]
        else:
            completions = [f for f in self.shapeL if f.startswith(text)]
        return completions



    # CMD stuff
    def do_exit(self, line):
        """End the program."""

        print("Exiting...")
        return "ters"

    def do_EOF(self, line):
        """End the program."""
        print("Exiting...")
        return "tart"

if __name__ == '__main__':
    S = swig.ActionSwigInterface(1)
    
    C = ActionCmd()

    C.cmdloop()
    

