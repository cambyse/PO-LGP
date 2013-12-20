__author__ = 'ingo'

from morse.builder import *

# A 'naked' PR2 robot to the scene
bigbird = BasePR2()
bigbird.add_interface("ros")
bigbird.translate(x=2.5, y=3.2, z=0.0)

# laser scanner
scan = Hokuyo()
scan.translate(x=0.275, z=0.252)
bigbird.append(scan)
scan.properties(Visible_arc=False)
scan.properties(laser_range=30.0)
scan.properties(resolution=1.0)
scan.properties(scan_window=270.0)
scan.create_laser_arc()

scan.add_interface('ros', topic='/scan')

# An odometry sensor to get odometry information
odometry = Odometry()
bigbird.append(odometry)
odometry.add_interface('ros', topic="/odom")

# Keyboard control
keyboard = Keyboard()
bigbird.append(keyboard)

# Set the environment
env = Environment('tum_kitchen/tum_kitchen')
env.aim_camera([1.0470, 0, 0.7854])