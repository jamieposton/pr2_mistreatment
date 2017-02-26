#!/usr/bin/env python

"""
PR2 Mistreatment
"""
import argparse
import sys

from copy import copy

import rospy

import actionlib

from trajectory_msgs.msg import JointTrajectoryPoint


from control_msgs.msg import (
    FollowJointTrajectoryAction, 
    FollowJointTrajectoryGoal,
    PointHeadAction,
    PointHeadGoal
) 

from sensor_msgs.msg import (
    Image,
)

from geometry_msgs.msg import PointStamped

import os
import sys

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

import time
import socket

# Quit flag
quit = False

# Dictionary of items
items = {1 : "a shaving mirror", 2 : "a five gallon can of", 3 : "a case of army rations", 4 : "an atlas of the pacific", 5 : "a floating seat cushion", 6 : "a small transistor radio", 7 : "the shark repellent", 8 : "fifteen feet of nylon", 9 : "two boxes of chocolate bars", 10 : "a fishing kit"}

def main():
    """
    PR2 Mistreatment
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=False, choices=['l', 'r'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb
    if limb == None: limb = 'l'

    print("Initializing node... ")
    rospy.init_node("pr2_mistreatment")

    print("Initializing sound... ")
    soundhandle = SoundClient()

    print("Running. Ctrl-c to quit")

    otherLimb = ""
    if limb == 'l':
        otherLimb = 'r'
    else:
        otherLimb = 'l'

    traj = Trajectory(limb)
    trajOther = Trajectory(otherLimb)
    trajHead = HeadTrajectory()

    rospy.on_shutdown(traj.stop)
    rospy.on_shutdown(trajOther.stop)
    rospy.on_shutdown(trajHead.stop)

    moveArmsToStart(traj, trajOther, limb, otherLimb)
    moveHeadToStart(trajHead)


    # set up socket for TCP connection to get external IP
    s = socket.socket ( socket.AF_INET, socket.SOCK_STREAM ) # TCP

    # get external IP
    s.bind ( ( '', 0 ) )
    s.connect ( ( 'google.com', 80 ) )
    ThisIP = s.getsockname()[0]
    s.close()

    # Create the welcome port
    welcome = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    welcome.bind((ThisIP, 0))
    welcome.listen(5)

    # Get the port number
    ThisPort = int ( welcome.getsockname()[1] )

    size = 1024

    socket.setdefaulttimeout(180)

    # Show the IP and port
    print "IP: " + ThisIP
    print "Port: " + str(ThisPort)

    # Accept the connection
    oz, addr = welcome.accept()

    # Loop forever
    while not quit:
        attempt = 0
        while attempt < 10 and attempt != -1:
            # Get the command
            try:
                data = oz.recv(size)
                attempt = -1
            except:
                attempt +=1
                print("what") #wat
                time.sleep(3)
        if attempt >= 10:
            print("Connection Dropped\nCLOSING...")
            oz.close()
            exit()
        print data
        # Get the command and content
        divide = data.split()
        command = divide[0]
        content = divide[1]

        # Excecute based on the command

        if command == "wave":
            wave(soundhandle, traj, 'l')

        # Start message
        if command == "start":
            tts(soundhandle, "Hello you have two minutes")
        time.sleep(2)

        # Item prompt
        elif command == "prompt":
            # Give the prompt phrase
            tts(soundhandle, "What is your choice for")
        time.sleep(3)
            tts(soundhandle, content)

        # Item confimation
        elif command == "confirm":
            # Ask for confrimation
            tts(soundhandle, "Did you choose")
        time.sleep(1)
            tts(soundhandle, items[int(content)])

        # Happy face
        elif command == "happy":
            tts(soundhandle, "Yipeeeee")
            nod(trajHead, 1)
            time.sleep(1)

        # Sad face
        elif command == "sad":
            # Make phrase based on the item number
            if content == '5':
                slump(traj, trajOther, 'l', 'r')
                tts(soundhandle, "I am sorry I know that")
        shake(trajHead, 3)
                time.sleep(1)
            else:
        lookAt(trajHead, 5,0,-1,3)
                slump(traj, trajOther, 'l', 'r')
                tts(soundhandle, "I am sorry I am still")
                time.sleep(1)
            moveArmsToStart(traj, trajOther, 'l', 'r', 3)
            moveHeadToStart(trajHead)

            time.sleep(1)

        # Failure
        elif command == "fail":
            tts(soundhandle, "I am sorry I do not")
        time.sleep(4)

        # Good bye
        elif command == "bye":
            tts(soundhandle, "That is all five items")
        time.sleep(2)

        oz.send("continue")
    oz.close()
    

# Speaks the given string
def tts(soundhandle,text):
    print "Playing " + text
    throttle = 3 #seconds
    global allow_yak
    allow_yak = rospy.Time.now() + rospy.Duration.from_sec(throttle)
    soundAssets = '/home/mercedesa/catkin_ws/src/pr2_mistreatment/sounds/'

    if rospy.Time.now() >= allow_yak: # Throttles yak to avoid
        print("Sound throttled")      # SoundClient segfault
        return
    soundhandle.playWave(soundAssets + text + ".wav" )

def translateCoords(coords):
    translate = [-1,1,-1,1,-1,1,1]
    newCoords = []
    for i in range(7):
        newCoords.append(coords[i]*translate[i])
    return newCoords

class HeadTrajectory():
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
           "/head_traj_controller/point_head_action",
           PointHeadAction
        )

        self._goal = PointHeadGoal()

        server_up = self._client.wait_for_server()
        if not server_up:
            rospy.logerr("Timed out waiting for HeadTrajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, x, y, z):
        point = PointStamped()
        point.header.frame_id = "base_link"
        point.point.x = x
        point.point.y = y
        point.point.z = z
        self._goal.target = point

        self._goal.pointing_frame = "high_def_frame"
        self._goal.pointing_axis.x = 1
        self._goal.pointing_axis.y = 0
        self._goal.pointing_axis.z = 0
        self._goal.min_duration = rospy.Duration(0.5)
        self._goal.max_velocity = 1.0

    def start(self):
        self._goal.target.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = PointHeadGoal()

class Trajectory(object):
    def __init__(self, limb):

        ns = "_arm_controller/follow_joint_trajectory"

        self._client = actionlib.SimpleActionClient(
           limb + ns, FollowJointTrajectoryAction
        )
        self._goal = FollowJointTrajectoryGoal()
        # self._client.wait_for_server()
        server_up = self._client.wait_for_server()
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.trajectory.joint_names = [limb + joint for joint in \
            ['_shoulder_pan_joint',
             '_shoulder_lift_joint',
             '_upper_arm_roll_joint',
             '_elbow_flex_joint',
             '_forearm_roll_joint',
             '_wrist_flex_joint',
             '_wrist_roll_joint']]

def lookAt(traj, x,y,z, t=5.0):
    traj.add_point(x,y,z)
    traj.start()
    traj.wait(t)
    traj.clear()

def nod(traj, t=5.0):
    for i in range(2):
        lookAt(traj, 5,0,2, t/3)
        lookAt(traj, 5,0,-1, t/3)
    moveHeadToStart(traj)

def shake(traj, t=5.0):
    for i in range(3):
        lookAt(traj, 5,2,-0.5, t/3)
        lookAt(traj, 5,-2,-0.5, t/3)

def moveHeadToStart(traj, t=5.0):
    lookAt(traj, 10,0,0, t)

def moveArmsToStart(traj, trajOther, limb, otherLimb, t=5.0):
    startPos = {
      'l': [0.75, 0.0, 1.6, -1.5, -0.0, 0.0, -1.5],
        'r': [-0.75, 0.0, -1.6, -1.5, 0.0, 1.0, 1.5]
    }
    traj.add_point(startPos[limb], t)
    trajOther.add_point(startPos[otherLimb], t)
    traj.start()
    trajOther.start()
    traj.wait(t)
    trajOther.wait(t)
    traj.clear(limb)
    trajOther.clear(otherLimb)

def moveArmToStart(traj, limb, t=5.0):
#Original start position values
    startPos = {
        'l': [0.75, 0.0, 1.6, -1.5, -0.0, 0.0, -1.5],
        'r': [-0.75, 0.0, -1.6, -1.5, 0.0, 1.0, 1.5]
    }
    # startPos = {
    #   'l': [0.75, 0.25, 1.75, -1.5, -0.0, 0.0, -1.5],
    #     'r': [-0.75, 0.25, -1.75, -1.5, 0.0, 1.0, 1.5]
    # }
    traj.add_point(startPos[limb], t)
    traj.start()
    traj.wait(t)
    traj.clear(limb)
    

def wave(soundhandle, traj, limb, t=3.0):
    #WAVE POSITIONS
    newWave1 = [0.75, 0.0, 0.0, -2.0, 0.0, 0.0, -1.5]
    #for right -0.75, 0.0, 0.0, -2.0, 0.0, 1.0, 1.5
    newWave2 = [0.75, 0.0, 0.0, -1.5, 0.0, 0.0, -1.5]
    #for right -0.75, 0.0, 0.0, -1.5, 0.0, 1.0, 1.5

    if limb != "l":
        newWave1 = translateCoords(newWave1)
        newWave2 = translateCoords(newWave2)
   
    dt = 1.0
    traj.add_point(newWave1, t)
    t+= dt
    traj.add_point(newWave2, t)
    t+=dt
    traj.add_point(newWave1, t)
    t+= dt
    traj.add_point(newWave2, t)
    t+=dt + .5
   
    traj.start()
    traj.wait(t/2)
    traj.wait(t/2)    
    traj.clear(limb)
    moveArmToStart(traj, limb, 2.5)

def slump(traj, trajOther, limb, otherLimb, t=3.0):

    sadPos = {
        'l': [0.75, 0.25, 1.75, -1.5, -0.0, 0.0, -1.5],
        'r': [-0.75, 0.25, -1.75, -1.5, 0.0, 1.0, 1.5]
    }


    traj.add_point(sadPos[limb], t)
    trajOther.add_point(sadPos[otherLimb], t)
    traj.start()
    trajOther.start()
    traj.wait(t)
    trajOther.wait(t)
    traj.clear(limb)
    trajOther.clear(otherLimb)
    
if __name__ == "__main__":
    main()
