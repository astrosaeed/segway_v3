#!/usr/bin/env python
"""--------------------------------------------------------------------
COPYRIGHT 2018 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company") 
for its licensed SI Vector Platform is intended and supplied to you, 
the Company's customer, for use solely and exclusively with Stanley Innovation 
products. The software is owned by the Company and/or its supplier, and is 
protected under applicable copyright laws.  All rights are reserved. Any use in 
violation of the foregoing restrictions may subject the user to criminal 
sanctions under applicable laws, as well as to civil liability for the 
breach of the terms and conditions of this license. The Company may 
immediately terminate this Agreement upon your use of the software with 
any products that are not Stanley Innovation products.

The software was written using Python programming language.  Your use 
of the software is therefore subject to the terms and conditions of the 
OSI- approved open source license viewable at http://www.python.org/.  
You are solely responsible for ensuring your compliance with the Python 
open source license.

You shall indemnify, defend and hold the Company harmless from any claims, 
demands, liabilities or expenses, including reasonable attorneys fees, incurred 
by the Company as a result of any claim or proceeding against the Company 
arising out of or based upon: 

(i) The combination, operation or use of the software by you with any hardware, 
    products, programs or data not supplied or approved in writing by the Company, 
    if such claim or proceeding would have been avoided but for such combination, 
    operation or use.
 
(ii) The modification of the software by or on behalf of you 

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
 \file   gripper_action_test.py

 \brief  Node for testing the gripper command action server

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import sys
from copy import copy
import rospy
import actionlib
import math
import random

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

from sensor_msgs.msg import JointState


class GripperActionTest(object):
    def __init__(self):
        
        self._client = actionlib.SimpleActionClient(
            '/kinova/gripper_controller/gripper_cmd',
            GripperCommandAction,
        )
        self._goal = GripperCommandGoal()
        server_up = self._client.wait_for_server()
        if not server_up:
            rospy.logerr("Timed out waiting for Gripper Command"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def command(self, position, block=False, timeout=15.0):
        self._goal.command.position = position
        self._goal.command.max_effort = -1.0
        self._client.send_goal(self._goal)
        if block:
            self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()


def main():
    rospy.init_node('gripper_action_test')

    g_test = GripperActionTest()

    g_test.command(0.0)
    g_test.wait()
       
    g_test.command(0.96)
    g_test.wait()
    
    g_test.command(0.0)
    g_test.wait()
    
    g_test.command(0.96)
    g_test.wait()

    g_test.command(0.0)
    g_test.wait()
    
    print("Gripper Action Test Example Complete")
    
if __name__ == "__main__":
    main()

        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
