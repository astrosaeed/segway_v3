"""--------------------------------------------------------------------
COPYRIGHT 2018 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company") 
for its licensed segway RMP Robotic Platforms is intended and supplied to you, 
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
 
 \file   segway_teleop.py

 \brief  This module contains a class for teleoperating the segway
         platform with a joystick controller

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from utils import *
from system_defines import *
from segway_msgs.msg import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,Float64,UInt32
from ds4drv_msgs.msg import DS4_ConnectionStatus, DS4_Indication
import threading
import rospy
import sys
import math

DS4_CTRL_MAP =     dict({'momentary': {'dead_man'     : {'is_button':False,'index':3,'invert_axis':True,'set_thresh':0.9},
                                       'man_ovvrd'    : {'is_button':False,'index':4,'invert_axis':True,'set_thresh':0.9},
                                       'standby'      : {'is_button':True,'index':8,'set_val':1},
                                       'tractor'      : {'is_button':True,'index':9,'set_val':1},
                                       'stop_robot'   : {'is_button':True,'index':5,'set_val':1},
                                       'start_wp'     : {'is_button':True,'index':0,'set_val':1},
                                       'stop_wp'      : {'is_button':True,'index':1,'set_val':1},
                                       'reset_wp'     : {'is_button':True,'index':2,'set_val':1},
                                       'rec_wp'       : {'is_button':True,'index':3,'set_val':1},
                                       'clear_wp'     : {'is_button':True,'index':4,'set_val':1}},
                         'axis'     : {'left_right'   : {'index' :0, 'invert_axis':False},
                                       'for_aft'      : {'index' :1, 'invert_axis':False},
                                       'twist'        : {'index' :2, 'invert_axis':False},
                                       'jog_x'        : {'index' :10, 'invert_axis':False},
                                       'jog_y'        : {'index' :9, 'invert_axis':False}}})

FX10_CTRL_MAP =    dict({'momentary': {'dead_man'     : {'is_button':False,'index':2,'invert_axis':True,'set_thresh':0.9},
                                       'man_ovvrd'    : {'is_button':False,'index':5,'invert_axis':True,'set_thresh':0.9},
                                       'standby'      : {'is_button':True,'index':6,'set_val':1},
                                       'tractor'      : {'is_button':True,'index':7,'set_val':1},
                                       'stop_robot'   : {'is_button':True,'index':5,'set_val':1},
                                       'start_wp'     : {'is_button':True,'index':0,'set_val':1},
                                       'stop_wp'      : {'is_button':True,'index':1,'set_val':1},
                                       'reset_wp'     : {'is_button':True,'index':3,'set_val':1},
                                       'rec_wp'       : {'is_button':True,'index':2,'set_val':1},
                                       'clear_wp'     : {'is_button':True,'index':4,'set_val':1}},
                         'axis'     : {'left_right'   : {'index' :0, 'invert_axis':False},
                                       'for_aft'      : {'index' :1, 'invert_axis':False},
                                       'twist'        : {'index' :3, 'invert_axis':False},
                                       'jog_x'        : {'index' :7, 'invert_axis':False},
                                       'jog_y'        : {'index' :6, 'invert_axis':False}}})

"""                                       
X3DP_CTRL_MAP =    dict({'momentary': {'dead_man'     : {'is_button':False,'index':3,'invert_axis':True,'set_thresh':0.9},
                                       'man_ovvrd'    : {'is_button':False,'index':4,'invert_axis':True,'set_thresh':0.9},
                                       'standby'      : {'is_button':True,'index':8,'set_val':1},
                                       'tractor'      : {'is_button':True,'index':9,'set_val':1},
                                       'stop_robot'   : {'is_button':True,'index':5,'set_val':1},
                                       'start_wp'     : {'is_button':True,'index':0,'set_val':1},
                                       'stop_wp'      : {'is_button':True,'index':1,'set_val':1},
                                       'reset_wp'     : {'is_button':True,'index':2,'set_val':1},
                                       'rec_wp'       : {'is_button':True,'index':3,'set_val':1},
                                       'clear_wp'     : {'is_button':True,'index':4,'set_val':1}},
                         'axis'     : {'left_right'   : {'index' :0, 'invert_axis':False},
                                       'for_aft'      : {'index' :1, 'invert_axis':False},
                                       'twist'        : {'index' :2, 'invert_axis':False},
                                       'jog_x'        : {'index' :10, 'invert_axis':False},
                                       'jog_y'        : {'index' :9, 'invert_axis':False}}})
                                       
CTRL_MAPPING = dict({'ds4' : DS4_CTRL_MAP,
                     'f710': FX10_CTRL_MAP,
                     'f310': FX10_CTRL_MAP,
                     'x3dp': X3DP_CTRL_MAP})
"""

"""
Leaving out extreme 3d pro for now
"""
CTRL_MAPPING = dict({'ds4' : DS4_CTRL_MAP,
                     'f710': FX10_CTRL_MAP,
                     'f310': FX10_CTRL_MAP})

class SegwayTeleop(object):
    def __init__(self):
         
        self.is_sim = rospy.get_param('~sim',False)
        self.js_type = rospy.get_param('~js_type','ds4')
        
        if not self.js_type in CTRL_MAPPING:
            rospy.logerr("No such control mapping %s; try ds4, f710, f310 or x3dp"%self.js_type)
            sys.exit(0)
            
        
        """
        Get the mapping for the various commands, defaults are xbox360 wireless
        """
        self.ctrl_map = CTRL_MAPPING[self.js_type]
        
        """
        Initialize the debounce logic states
        """
        self.db_cnt = dict()
        self.axis_value = dict()
        self.button_state = dict()

        for key, value in self.ctrl_map.iteritems():
            if key == 'momentary':
                for key, value2 in value.iteritems():
                    self.db_cnt[key]=0
                    self.button_state[key]=False
            else:
                self.db_cnt[key]=0
                self.axis_value[key]=0.0 
        
        self.zero_joy_commands = False
        self.good_frames = 6
        self.frames_of_zero_command = 0
                
        
        self._subs = []
        self._pubs = []
        
        if (False == self.is_sim):
        
            """
            Subscribe to the configuration message
            """
            self.config_updated = False
            try:
                config = rospy.wait_for_message("/segway/feedback/configuration", Configuration, timeout=10.0)
                self.x_vel_limit_mps = config.teleop_vel_limit_mps
                self.y_vel_limit_mps = config.teleop_vel_limit_mps
                self.yaw_rate_limit_rps = config.teleop_yaw_rate_limit_rps
                self.accel_lim = config.teleop_accel_limit_mps2
                self.yaw_accel_lim = config.teleop_yaw_accel_limit_rps2
                self.jog_velocity_lim = 0.1
                self.jog_yaw_lim = 0.2
            except:
                rospy.logerr("Timed out waiting for RMP feedback topics make sure the driver is running")
                rospy.signal_shutdown(0)
                return
        else:
            self.x_vel_limit_mps = rospy.get_param('~sim_teleop_x_vel_limit_mps',0.5)
            self.y_vel_limit_mps = rospy.get_param('~sim_teleop_y_vel_limit_mps',0.5)
            self.yaw_rate_limit_rps = rospy.get_param('~sim_teleop_yaw_rate_limit_rps',0.5)
            self.accel_lim = rospy.get_param('~sim_teleop_accel_lim',0.5)
            self.yaw_accel_lim = rospy.get_param('~sim_teleop_yaw_accel_lim',1.0)
            self.jog_velocity_lim = rospy.get_param('~sim_jog_velocity_lim',0.1)
            self.jog_yaw_lim = rospy.get_param('~sim_jog_yaw_lim',0.2)
        
        """
        Initialize the flags and pubs/subs
        """
        self._subs.append(rospy.Subscriber("/segway/feedback/configuration", Configuration, self._update_limits))
        
        self.send_cmd_none = False
        self.no_motion_commands = True
        self.last_motion_command_time = 0.0
        self.last_joy = rospy.get_time()
        self.cmd_last = rospy.get_time()
            
        self.cfg_cmd = ConfigCmd()
        self.cfg_pub = rospy.Publisher('/segway/gp_command', ConfigCmd, queue_size=10)
        self._pubs.append(self.cfg_pub)
        self.nav_cmd_pub = rospy.Publisher('/segway/waypoint_cmd',UInt32, queue_size=10)
        self._pubs.append(self.nav_cmd_pub)
        
        self.motion_cmd = Twist()
        self.limited_cmd = Twist()
        self.motion_pub = rospy.Publisher('/segway/teleop/cmd_vel', Twist, queue_size=10)
        self._pubs.append(self.motion_pub)
        self.override_pub = rospy.Publisher("/segway/manual_override/cmd_vel",Twist, queue_size=10)
        self._pubs.append(self.override_pub)
                
        self._max_rates = [self.accel_lim,self.accel_lim,self.yaw_accel_lim]
        self._rate_cmds = [0.0,0.0,0.0]
        self._cmd_rate_limit = RateLimitSignals(self._max_rates,3,self._rate_cmds)

        self.cfg_cmd.header.stamp = rospy.get_rostime()
        self.cfg_cmd.gp_cmd = "GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE"
        self.cfg_cmd.gp_param = STANDBY_REQUEST
        self.cfg_pub.publish(self.cfg_cmd)
        self.update_stop_state = True
        self.stop_robot = 0
        self.prev_nav_button_state = 0
        self.no_input_frames = 11
        
        self._lock = threading.Lock()
        
        if ('ds4' == self.js_type):
            joy_online = False
            while not joy_online and not rospy.is_shutdown():
                
                try:
                    msg = rospy.wait_for_message("/joy/connection_status", DS4_ConnectionStatus, timeout=10.0)
                    if 1 == msg.index:
                        joy_online = msg.connected
                except:
                    rospy.logwarn ('Waiting for joystick to come online,\n make sure it is on an connected via bluetooth.....')
                    
            rospy.logwarn("Joystick is connected and ready.....")
            
            self._ctl_msg = DS4_Indication()
            self._ctl_msg.index = 1
            self._pub = rospy.Publisher('/joy/indication',DS4_Indication,queue_size=1)
            rospy.sleep(0.1)
            self._ctl_msg.big_rumble = 0xff
            self._ctl_msg.small_rumble = 0
            self._ctl_msg.led_red = 0
            self._ctl_msg.led_green = 0xff
            self._ctl_msg.led_blue = 0
            self._ctl_msg.flash_on = 0
            self._ctl_msg.flash_off = 0
            self._pub.publish(self._ctl_msg)
            rospy.sleep(0.5)
            self._ctl_msg.big_rumble = 0
            self._pub.publish(self._ctl_msg)
            rospy.sleep(0.5)
            
            self._subs.append(rospy.Subscriber("/joy/connection_status", DS4_ConnectionStatus, self._update_joy_status)) 
        rospy.Subscriber('/joy', Joy, self._parse_joy_input)
        
        rospy.sleep(1.0)
        self.cfg_cmd.header.stamp = rospy.get_rostime()
        self.cfg_cmd.gp_cmd = "GENERAL_PURPOSE_CMD_SET_AUDIO_COMMAND"
        self.cfg_cmd.gp_param = MOTOR_AUDIO_PLAY_ENTER_ALARM_SONG
        self.cfg_pub.publish(self.cfg_cmd)
        rospy.sleep(1.0)
        
        self._t1 = rospy.Timer(rospy.Duration(0.05),self._run_teleop)


        
    def shutdown(self):
        try:
            self._t1.shutdown()
        except:
            pass
        try:
            for sub in self._subs:
                sub.unregister()
        except:
            pass
        try:
            for pub in self._pubs:
                pub.unregister()
        except:
            pass
            
    def _update_limits(self,config):
        self.x_vel_limit_mps = config.teleop_vel_limit_mps
        self.y_vel_limit_mps = config.teleop_vel_limit_mps
        self.yaw_rate_limit_rps = config.teleop_yaw_rate_limit_rps
        self.accel_lim = config.teleop_accel_limit_mps2
        self.yaw_accel_lim = config.teleop_yaw_accel_limit_rps2              
    
    def _update_joy_status(self,msg):
  
        if (1 == msg.index): 
            if not msg.connected:
                if not self.zero_joy_commands:
                    rospy.logwarn("Joystick reports not connected....zeroing commands")    
                self.zero_joy_commands = True
            elif (msg.reports_per_second < 30):
                if not self.zero_joy_commands:
                    rospy.logwarn("Joystick reports weak signal....zeroing commands")   
                self.zero_joy_commands = True
                self.good_frames = 0
            elif self.zero_joy_commands and (msg.reports_per_second > 100) and (self.good_frames < 6):
                self.good_frames += 1
                if (self.good_frames > 6):
                    self.good_frames = 0
                    self.zero_joy_commands = False
                    rospy.logwarn("Joystick reports good signal....resuming teleop") 
            elif msg.connected and self.zero_joy_commands:
                rospy.logwarn("Joystick connected....starting teleop")
                self.zero_joy_commands = False
        
    def _parse_joy_input(self,joyMessage):
        with self._lock:
       
            raw_button_states = dict()
            self.button_state = dict()
            
            for key, value in self.ctrl_map.iteritems():
                if key == 'momentary':
                    for key2, value2 in value.iteritems():
                        raw_button_states[key2]=True
                        self.button_state[key2]=False
                else:
                    for key2, value2 in value.iteritems():  
                        self.axis_value[key2] = 0.0            
             
            
            for key, value in self.ctrl_map.iteritems():
                if key == 'momentary':
                    for key2, item in value.iteritems():
                        if item['is_button']:
                            if item['set_val'] == joyMessage.buttons[item['index']]:
                                raw_button_states[key2] &= True
                            else:
                                raw_button_states[key2] = False
                        else:
                            temp = joyMessage.axes[item['index']]
                            if (item['invert_axis']):
                                temp *= -1.0
                            if (temp >= item['set_thresh']):
                                raw_button_states[key2] &= True
                            else:
                                raw_button_states[key2] = False
                         
                
                        if (True == raw_button_states[key2]):
                            self.db_cnt[key2]+=1
                            if (self.db_cnt[key2] > 10):
                                self.db_cnt[key2] = 10
                                self.button_state[key2] = True
                        else:
                            self.button_state[key2] = False
                            self.db_cnt[key2] = 0
                if key == 'axis':
                    for key2, item in value.iteritems():
                        temp = joyMessage.axes[item['index']]
                        if (item['invert_axis']):
                            temp *= -1.0
                        self.axis_value[key2] = temp
            self.last_joy = rospy.get_time()

    def _run_teleop(self, event):
    
        with self._lock:
            dt = rospy.get_time() - self.cmd_last
            
            nav_button_state = 0
            nav_button_state |= self.button_state['rec_wp']      << 0 #0x0001
            nav_button_state |= self.button_state['start_wp']    << 1 #0x0002
            nav_button_state |= self.button_state['stop_wp']     << 2 #0x0004      
            nav_button_state |= self.button_state['reset_wp']    << 3 #0x0004
            nav_button_state |= self.button_state['clear_wp']    << 4 #0x0010
            #nav_button_state |= self.button_state['load_wp_rec'] << 5 #0x0011
            #nav_button_state |= self.button_state['save_wp_rec'] << 6 #0x0012
            
            if (self.prev_nav_button_state != nav_button_state):
                if nav_button_state:
                    temp = UInt32()
                    temp.data = nav_button_state
                    self.nav_cmd_pub.publish(temp)
                
            self.prev_nav_button_state = nav_button_state 

                                      
            if self.button_state['standby']:
                self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
                self.cfg_cmd.gp_param = STANDBY_REQUEST
            elif self.button_state['tractor']:
                self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
                self.cfg_cmd.gp_param = TRACTOR_REQUEST
            else:
                self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_NONE'
                self.cfg_cmd.gp_param = 0
                
            if self.button_state['stop_robot']:
                if self.update_stop_state:
                    self.stop_robot ^= 1
                    self.update_stop_state = False
            else:
                self.update_stop_state = True
                
            
            if (self.stop_robot):
                tmp = Twist()
                self.override_pub.publish(tmp)
                    
            
            if ('GENERAL_PURPOSE_CMD_NONE' != self.cfg_cmd.gp_cmd):
                self.cfg_cmd.header.stamp = rospy.get_rostime()
                self.cfg_pub.publish(self.cfg_cmd)
                self.cfg_cmd.header.seq
                self.send_cmd_none = True
            elif (True == self.send_cmd_none):
                self.cfg_cmd.header.stamp = rospy.get_rostime()
                self.cfg_pub.publish(self.cfg_cmd)
                self.cfg_cmd.header.seq
                self.send_cmd_none = False
            elif (False == self.send_cmd_none):
                if (self.stop_robot):
                    self.motion_cmd.linear.x = 0.0
                    self.motion_cmd.linear.y = 0.0
                    self.motion_cmd.angular.z = 0.0
                    self.limited_cmd = self.motion_cmd
                    self.override_pub.publish(self.motion_cmd)
                    self._cmd_rate_limit.Reset([0.0,0.0,0.0])
                elif (self.zero_joy_commands) and (self.frames_of_zero_command < 10):
                    self.motion_cmd.linear.x = 0.0
                    self.motion_cmd.linear.y = 0.0
                    self.motion_cmd.angular.z = 0.0
                    self.limited_cmd = self.motion_cmd
                    self.override_pub.publish(self.motion_cmd)
                    self.motion_pub.publish(self.motion_cmd)
                    self.frames_of_zero_command += 1
                    self._cmd_rate_limit.Reset([0.0,0.0,0.0])
                elif self.button_state['dead_man'] and not self.zero_joy_commands:
                    self.no_input_frames = 0 
                    self.frames_of_zero_command = 0
                    if (abs(self.axis_value['jog_x']) > 0.0) or (abs(self.axis_value['jog_y']) > 0.0):
                        self.motion_cmd.linear.x =  (self.axis_value['jog_x'] * self.jog_velocity_lim)
                        self.motion_cmd.linear.y =  (self.axis_value['jog_y'] * self.jog_velocity_lim)
                        self.motion_cmd.angular.z = 0.0
                    else:                    
                        self.motion_cmd.linear.x =  (self.axis_value['for_aft'] * self.x_vel_limit_mps)
                        self.motion_cmd.linear.y =  (self.axis_value['left_right'] * self.y_vel_limit_mps)
                        self.motion_cmd.angular.z = (self.axis_value['twist'] * self.yaw_rate_limit_rps)
                    
                    self._rate_cmds = [self.motion_cmd.linear.x,self.motion_cmd.linear.y,self.motion_cmd.angular.z]
                    rate_limited_cmds = self._cmd_rate_limit.Update(self._rate_cmds)
                    
                    self.limited_cmd.linear.x = rate_limited_cmds[0]
                    self.limited_cmd.linear.y = rate_limited_cmds[1]
                    self.limited_cmd.angular.z = rate_limited_cmds[2]
                    
                    if (self.button_state['man_ovvrd']):
                        self.override_pub.publish(self.limited_cmd)
                    else:
                        self.motion_pub.publish(self.limited_cmd)
                else:

                    self.motion_cmd.linear.x = 0.0
                    self.motion_cmd.linear.y = 0.0
                    self.motion_cmd.angular.z = 0.0
                    self._cmd_rate_limit.Reset([0.0,0.0,0.0])
                    self.limited_cmd = self.motion_cmd
                    if (self.no_input_frames < 10): 
                        self.override_pub.publish(self.limited_cmd)
                        self.motion_pub.publish(self.limited_cmd)
                        self.no_input_frames += 1
                    
                    
            self.cmd_last = rospy.get_time()  
            
            
