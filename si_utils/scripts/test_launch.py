#!/usr/bin/env python
import subprocess
import os
import rospy
import rosnode
import signal
import psutil




if __name__ == "__main__":
    env_tags = os.environ.copy()
    print env_tags

    p = subprocess.Popen(['roslaunch','si_utils','test_launch.launch'],env=env_tags)

    rospy.sleep(10)
    
    rosnode.kill_nodes(['sound_play', 'movo_voice', 'voice_test','subtest_bringup'])


