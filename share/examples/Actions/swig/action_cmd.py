#!/usr/bin/env python

from __future__ import print_function
import cmd
import random
import sys
import os
import time
sys.path.append(os.path.join(os.path.expanduser("~"),'git/mlr/share/src/Actions/'))
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
        self.parameters =({"type":"pos", "ref1":"endeffL" , "target":"[0.5, 0.1, 0.8]","PD" :"[.5, .9, .1, 10.]"})


    def cmdloop (self):
        cmd.Cmd.cmdloop(self)
        return "true"

    def do_changePara(self, par):
        p = par.split()
        if len(p) == 2:
            if p[0]=="target" and list(p[1])[0] != "[":
                self.parameters[p[0]] = self.do_getBodyByName(p[1])["pos"]
            else:          
                self.parameters[p[0]] = p[1]

    def do_getBodyByName (self, obj):
        """get literal of body by name"""
        body = S.getBodyByName(obj)
        print("name: " + body["name"])
        print("pos: " + body["pos"])
        print("Quaterion: " + body["Q"])
        return body

    def do_getShapeList (self, xxx):
        """get list of available shapes"""
        shapeL = S.getShapeList()
        print (shapeL)

    def do_getBodyList (self, xxx):
        """get list of available bodies"""
        bodyL = S.getBodyList()
        print(bodyL)

    def do_getLit (self, name):
        """get literal"""
        print (S.lit([name]))

    def newTask (self, nae, ref, target):
        print("testerer")

    def do_print(self, xxx):
        print (self.parameters)
        
    def do_newTask (self, name):
        """define new task space control action
        argument order:
        1.: type 2.: name, 3.: effector, 4.: target."""
        p = name.split()
        name = "test" 

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
        
        S.defineNewTaskSpaceControlAction(name, self.parameters)
        S.startActivity(S.lit([name]),self.parameters)

    def do_startAction (self, name):
        """start defined task"""
        S.startActivity(S.lit(["posHand"]), self.parameters)

    def complete_newTask(self, text, line, begidx, endidx):
        if not text:
            completions = self.bodyL[:]
        else:
            completions = [f for f in self.bodyL if f.startswith(text)]
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
    S = swig.ActionSwigInterface(0)
    
    C = ActionCmd()

    C.cmdloop()
    

