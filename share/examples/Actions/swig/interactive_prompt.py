#!/usr/bin/env python
"""
This module contains a draft for the interactive prompt of the action
interface.

It's just meant as a demo and has no actual functionality yet.

Start it with:

>>> python action_cmd.py

Note:
- http://pymotw.com/2/cmd/
- https://pythonhosted.org/cmd2/

"""
from __future__ import print_function
import cmd
import random


class ActionCmd(cmd.Cmd):
    prompt = "mlr> "

    objects = ["mug", "table", "flower"]

    def do_ls(self, xxx):
        """Print all shapes"""
        print(self.objects)

    def do_get_state(self, xxx):
        """Print all shapes"""
        print([random.random() for i in range(5)])

    # TOUCH
    def do_touch(self, obj):
        """Touch the given object
        """
        if obj:
            print("touching {}".format(obj))
        else:
            print("no object specified")

    def complete_touch(self, text, line, begidx, endidx):
        if not text:
            completions = self.objects[:]
        else:
            completions = [f for f in self.objects if f.startswith(text)]
        return completions

    # MISC
    def do_close_gripper(self, side):
        """Close the gripper - this is the documentation."""
        print("closing gripper {}".format(side))

    def do_open_gripper(self, side):
        """Open the gripper - this is the documentation."""
        print("opening gripper {}".format(side))

    # CMD stuff
    def preloop(self):
        print("Type 'help' to see all actions.")
        print("Type 'help action' to get the help for the given action.")
        print()

    def do_exit(self, line):
        """End the program."""
        print("Exiting...")
        return True

    def do_EOF(self, line):
        """End the program."""
        print("Exiting...")
        return True


if __name__ == '__main__':
    ActionCmd().cmdloop()
