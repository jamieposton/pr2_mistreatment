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
import random

# Quit flag
quit = False

# Dictionary of items
items = {1 : "a shaving mirror", 2 : "a five gallon can of", 3 : "a case of army rations", 4 : "an atlas of the pacific", 5 : "a floating seat cushion", 6 : "a small transistor radio", 7 : "the shark repellent", 8 : "fifteen feet of nylon", 9 : "two boxes of chocolate bars", 10 : "a fishing kit"}

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

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

    print("Initializing Arm/Head Positions. Ctrl-d to quit")

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
    #wave(soundhandle, traj, 'l')
    moveHeadToStart(trajHead)

    # Loop forever
    while not quit:
        print "New experiment"

        used = []

        # Press enter when it is time to prompt for answers (3 min mark)
        print "Press enter to wave"
        ignore = raw_input()
        wave(soundhandle, traj, 'l')

        ignore = raw_input("Press enter to prompt for answers at the three minute mark")
        tts(soundhandle, "Hello you have two minutes")
        time.sleep(2)

        ignore = raw_input("Press enter to start recording answers")

        for i in range(1, 6):
            itemNum = -1
            print "This is item number " + str (i)

            # Failure loop
            loop = True
            while loop:
                # Send command for item prompt
                tts(soundhandle, "What is your choice for")
                time.sleep(3)
                tts(soundhandle, str(i))

                # Get the item number
                valid = False
                while not valid:
                    itemNum = -1
                    userInput = raw_input("\nEnter item number, or enter 0 to prompt again: ")
                    if is_number(userInput):
                        itemNum = int(userInput.strip())

                    if itemNum >= 0 and itemNum < 11:
                        valid = True

                    else:
                        print "That wasn't a correct item, the item must be between zero and ten"

                # Check for failure
                if itemNum == 0:
                    # Send the failure message
                    tts(soundhandle, "I am sorry I do not")
                    time.sleep(4)
                    print "Fail message given"
                    # Make a brief pause
                    time.sleep(1)
                    continue
                else:
                    loop = False

                used.append(itemNum)

                # If this is test 1,2 or 4, give the right answer
                if i == 1 or i == 2 or i == 4:
                    #used.append(itemNum)
                    print "Correct item used"

                # Else pick a random wrong answer
                else:
                    flag = True
                    while flag:
                        itemNum = random.randrange(1,10)
                        if itemNum not in used:
                            flag = False

                    used.append(itemNum)
                    print "Wrong item used"

            loop = True
            while loop:
                # Send confirm message
                tts(soundhandle, "Did you choose")
                time.sleep(1)
                tts(soundhandle, items[int(itemNum)])

                # Wait to give response
                check = raw_input("\nEnter when response given, or 0 to prompt again: ") .strip()

                # Check for failure
                if check == '0':
                    # Send the failure message
                    tts(soundhandle, "I am sorry I do not")
                    time.sleep(4)
                    print "Fail message given"
                    # Make a brief pause
                    time.sleep(1)
                    continue
                else:
                    loop = False

                # If this is test 1,2,4 then give the happy response
                if i == 1 or i == 2 or i == 4:
                    tts(soundhandle, "Yipeeeee")
                    nod(trajHead, 1)
                    time.sleep(1)

                # Else give the sad response based on the trial number
                else:
                    if i == 5:
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

        # Say goodbye
        tts(soundhandle, "That is all five items")
        time.sleep(2)
    

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

    def add_point(self, x, y, z, vel):
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
        self._goal.max_velocity = vel

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

def lookAt(traj, x,y,z, vel=1.0, t=5.0):
    traj.add_point(x,y,z, vel)
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
    startPos = {
        'l': [0.75, 0.0, 1.6, -1.5, -0.0, 0.0, -1.5],
        'r': [-0.75, 0.0, -1.6, -1.5, 0.0, 1.0, 1.5]
    }
    traj.add_point(startPos[limb], t)
    traj.start()
    traj.wait(t)
    traj.clear(limb)
    

def wave(soundhandle, traj, limb, t=2.25):

            # '_shoulder_pan_joint',
            #  '_shoulder_lift_joint',
            #  '_upper_arm_roll_joint',
            #  '_elbow_flex_joint',
            #  '_forearm_roll_joint',
            #  '_wrist_flex_joint',
            #  '_wrist_roll_joint'

    #WAVE POSITIONS
    # newWave = [0.9, -0.05, 0.0, -1.75, 0.0, -0.5, -1.5]
    newWave1 = [0.9, -0.075, 0.0, -2.0, 0.1, -0.4, -1.5]
    #for right -0.75, 0.0, 0.0, -2.0, 0.0, 1.0, 1.5
    newWave2 = [0.9, 0.075, -0.1, -1.6, 0., 0.0, -1.5]
    #for right -0.75, 0.0, 0.0, -1.5, 0.0, 1.0, 1.5

    if limb != "l":
        newWave1 = translateCoords(newWave1)
        newWave2 = translateCoords(newWave2)
   
    dt = 1.5
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

def idle(trajhead, t=3.0):

    headPos = [
        [5,     2,      0       ],
        [5,     -2,     0       ],
        [10,    0,      0       ],
        [10,    0,      -1      ],
    ]
    
    currentIndex = random.randint(0,len(headPos)-1)
    print "random: "
    print currentIndex
    lookAt(trajhead, headPos[currentIndex][0], 
        headPos[currentIndex][1], headPos[currentIndex][2], 0.5)
    
if __name__ == "__main__":
    main()
