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
    S = swig.ActionSwigInterface(0)
    prompt = "pr2> "

    shapeL = S.getShapeList()
    bodyL = S.getBodyList()
    jointL = S.getJointList()


    def do_getBodyByName (self, obj):
        """get literal of body by name"""
        body = self.S.getBodyByName(obj)
        print(body["pos"])
        return body

    def do_getShapeList (self, xxx):
        """get list of available shapes"""
        shapeL = self.S.getShapeList()
        print (shapeL)

    def do_getBodyList (self, xxx):
        """get list of available bodies"""
        bodyL = self.S.getBodyList()
        print(bodyL)#

    def do_getLit (self, name):
        """get literal"""
        print (self.S.lit([name]))

    def do_newTask (self, name):
        """define new task space control action
        3 arguments needed:
        1.: name, 2.: effector, 3.: target."""
        p = name.split()
        if len(p)!= 3:
            print ("invalid number of arguments")
            return
        else:
            parameters =({"type":"pos", "ref1":p[1] , "target":self.S.getBodyByName(p[2])["pos"],"PD" :"[.5, .9, .1, 10.]"})
            self.S.defineNewTaskSpaceControlAction(p[0], parameters)

    def do_startAction (self, name):
        """start defined task"""
        parameters =({"type":"pos", "ref1":"endeffL" , "target":self.S.getBodyByName("handle")["pos"],"PD" :"[.5, .9, .1, 10.]"})

        self.S.startActivity(self.S.lit(["posHand"]), parameters)

    def complete_newTask(self, text, line, begidx, endidx):
        if not text:
            completions = self.bodyL[:]
        else:
            completions = [f for f in self.bodyL if f.startswith(text)]
        return completions


    # CMD stuff
    def do_exit(self, line):
        """End the program."""

        del self.S
        print("Exiting...")
        return True

    def do_EOF(self, line):
        """End the program."""
        print("Exiting...")
        return True


if __name__ == '__main__':
    ActionCmd().cmdloop()
