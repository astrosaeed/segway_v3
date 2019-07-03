"""--------------------------------------------------------------------
COPYRIGHT 2018 Waypoint Robotics Inc.

Software License Agreement:

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 \file   rmp_data_classes.py

 \brief  a collection of RMP data classes

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from utils import *
from segway_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu, MagneticField,NavSatFix,NavSatStatus,JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32, Bool
import rospy
import tf
import math
import os

HAS_2_WHEELS = [2,3,5]

header_stamp = 0
platform_identifier = 0
machine_id = 1
has_imu = 0
has_brakes = 0
has_segway_bsa = 0
num_wheels = 2
wheel_circum = 0.542391 * math.pi

M_PI = math.pi

def deg_to_rad(degrees):
    return degrees * (M_PI / 180.0)

def rad_to_deg(rads):
    return rads * (180.0 / M_PI)
    
def wrap_angle(angle_rad):
    ret = ( angle_rad + math.pi) % (2 * math.pi ) - math.pi
    return ret


class RMP_Status:
    def __init__(self):
        self._MsgData = Status()
        self._MsgPub = rospy.Publisher('/segway/feedback/status', Status, queue_size=10)
        self._MsgData.header.frame_id = ''
        self._seq = 0
        self.init = True
        self.op_mode = 0

    def Shutdown(self):
        self._MsgPub.unregister()

    def parse(self,data):
        global header_stamp
        global platform_identifier
        global machine_id
        global has_imu
        global has_brakes
        global has_segway_bsa
        global num_wheels
        
        rmp_timestamp = round(convert_u32_to_float(data[8]),2)
        
        if self.init:
            self.platform_identifier = data[12]
             
            machine_id = self.platform_identifier & 0x1F
            has_imu = (self.platform_identifier & 0x00000020) >> 5
            has_brakes = (self.platform_identifier & 0x40) >> 6
            has_segway_bsa = (self.platform_identifier & 0x80) >> 7
            self.start_time_machine = rmp_timestamp
            self.start_time_local = rospy.Time.now().to_sec()
            self.init = False
            
            if machine_id in HAS_2_WHEELS:
                num_wheels = 2
            else:
                num_wheels = 4
            
        """
        causes drift with linux time (need ntpd) 
        
        if (rmp_timestamp < self.start_time_machine):
            self.start_time_machine = rmp_timestamp
            self.start_time_local = rospy.Time.now().to_sec()        
        header_stamp = rospy.Time.from_sec(self.start_time_local + (rmp_timestamp - self.start_time_machine))
        """

        header_stamp = rospy.Time.now()-rospy.Duration(0.02)
        
        self._MsgData.header.stamp = header_stamp
        self._MsgData.header.seq = self._seq
        temp = [data[0],data[1],data[2],data[3]]
        self._MsgData.fault_status_words = temp
        temp = [data[4],data[5],data[6],data[7]]
        self._MsgData.mcu_fault_status = temp
        self._MsgData.operational_time = rmp_timestamp
        self._MsgData.operational_state = data[9]
        self.op_mode = data[9]
        self._MsgData.dynamic_response = data[10]
        self._MsgData.machine_id = self.platform_identifier
        
        if not rospy.is_shutdown():
            self._MsgPub.publish(self._MsgData)
            self._seq += 1
        
        return header_stamp
        
class RMP_Propulsion:
    def __init__(self):
        self._MsgData = Propulsion()
        self._MsgPub = rospy.Publisher('/segway/feedback/propulsion', Propulsion, queue_size=10)
        self._MsgData.header.frame_id = ''
        self._seq = 0

    def Shutdown(self):
        self._MsgPub.unregister()

    def parse(self,data):
        self._MsgData.header.stamp = header_stamp
        self._MsgData.header.seq = self._seq
        
        self._MsgData.min_propulsion_battery_soc = convert_u32_to_float(data[0]) 
        temp = [convert_u32_to_float(data[1]),
                convert_u32_to_float(data[2]),
                convert_u32_to_float(data[3]),
                convert_u32_to_float(data[4])]
        self._MsgData.mcu_battery_soc = temp
        temp = [convert_u32_to_float(data[5]),
                convert_u32_to_float(data[6]),
                convert_u32_to_float(data[7]),
                convert_u32_to_float(data[8])]
        self._MsgData.mcu_battery_temp_degC = temp

        temp = [convert_u32_to_float(data[9]),
                convert_u32_to_float(data[10]),
                convert_u32_to_float(data[11]),
                convert_u32_to_float(data[12])]
        self._MsgData.mcu_inst_power_W = temp
        temp = [convert_u32_to_float(data[13]),
                convert_u32_to_float(data[14]),
                convert_u32_to_float(data[15]),
                convert_u32_to_float(data[16])]
        self._MsgData.mcu_total_energy_Wh = temp
        temp = [convert_u32_to_float(data[17]),
                convert_u32_to_float(data[18]),
                convert_u32_to_float(data[19]),
                convert_u32_to_float(data[20])]
        self._MsgData.motor_current_A0pk = temp               
        
        self._MsgData.max_motor_current_A0pk = convert_u32_to_float(data[21])

        temp = [convert_u32_to_float(data[22]),
                convert_u32_to_float(data[23]),
                convert_u32_to_float(data[24]),
                convert_u32_to_float(data[25])]
        self._MsgData.motor_current_limit_A0pk = temp 

        self._MsgData.min_motor_current_limit_A0pk = convert_u32_to_float(data[26])

        
        if not rospy.is_shutdown():
            self._MsgPub.publish(self._MsgData)
            self._seq += 1

class RMP_AuxPower:
    def __init__(self):
        self._MsgData = AuxPower()
        self._MsgPub = rospy.Publisher('/segway/feedback/aux_power', AuxPower, queue_size=10)
        self._MsgData.header.frame_id = ''
        self._seq = 0

    def Shutdown(self):
        self._MsgPub.unregister()

    def parse(self,data):
        self._MsgData.header.stamp = header_stamp
        self._MsgData.header.seq = self._seq

        temp = [convert_u32_to_float(data[0]),
                convert_u32_to_float(data[1])]
        self._MsgData.aux_soc = temp

        temp = [convert_u32_to_float(data[2]),
                convert_u32_to_float(data[3])]
        self._MsgData.aux_voltage_V = temp
                
        temp = [convert_u32_to_float(data[4]),
                convert_u32_to_float(data[5])]
        self._MsgData.aux_current_A = temp

        temp = [convert_u32_to_float(data[6]),
                convert_u32_to_float(data[7])]
        self._MsgData.aux_temp_degC = temp
        
        temp = [data[8], data[9]]
        self._MsgData.aux_sys_status = temp
        
        temp = [data[10], data[11]]
        self._MsgData.aux_bcu_status = temp

        temp = [data[12], data[13]]
        self._MsgData.aux_bcu_faults = temp
        
        self._MsgData.startup_7p2v_batt_voltage_V = convert_u32_to_float(data[14])
        
        if not rospy.is_shutdown():
            self._MsgPub.publish(self._MsgData)
            self._seq += 1
        
class IMU_Packet(object):
    def __init__(self):
        self.accel_mps2       = [0.0]*3
        self.gyro_rps         = [0.0]*3
        self.mag_T            = [0.0]*3
        self.rpy_rad          = [0.0]*3
        self.timestamp_sec    = 0.0
        self.linear_accel_covariance = 0.098 * 0.098
        self.angular_velocity_covariance = 0.012 * 0.012
        self.orientation_covariance = 0.035 * 0.035
        self.magnetic_field_covariance = 0.000002 * 0.000002
        
        self.publish_data = False
        self._MsgData = Imu()
        self._MsgPub = rospy.Publisher('/segway/feedback/imu', Imu, queue_size=10)
        self._MsgData.header.frame_id = 'imu_frame'
        self._seq = 0
        
        self._MagMsgData = MagneticField()
        self._MagMsgPub = rospy.Publisher('/segway/feedback/mag_feild', MagneticField, queue_size=10)
        self._MagMsgData.header.frame_id = 'imu_frame'
        self._Magseq = 0

    def Shutdown(self):
        self._MsgPub.unregister() 
        self._MagMsgPub.unregister() 

    def parse_data(self,data):
        index = 0
        for i in range(0,3):
            self.accel_mps2[i] = convert_u32_to_float(data[index]) * 9.81
            index+=1
        for i in range(0,3):
            self.gyro_rps[i] = convert_u32_to_float(data[index])
            index+=1
        for i in range(0,3):
            self.mag_T[i] = convert_u32_to_float(data[index]) * 0.0001
            index+=1
        for i in range(0,3):
            self.rpy_rad[i] = convert_u32_to_float(data[index])
            index+=1
        
        self._MsgData.header.stamp = header_stamp
        self._MagMsgData.header.stamp = header_stamp
        self._MsgData.header.seq = self._seq
        self._MagMsgData.header.seq = self._seq      
        
        roll = self.rpy_rad[0] 
        pitch = self.rpy_rad[1]
        yaw = self.rpy_rad[2]
        
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw, 'sxyz')
        
        self._MsgData.orientation.x = q[0]
        self._MsgData.orientation.y = q[1]
        self._MsgData.orientation.z = q[2]
        self._MsgData.orientation.w = q[3]
        
        self._MsgData.orientation_covariance[0] = self.orientation_covariance
        self._MsgData.orientation_covariance[4] = self.orientation_covariance
        self._MsgData.orientation_covariance[8] = self.orientation_covariance
        
        self._MsgData.linear_acceleration.x = self.accel_mps2[0]
        self._MsgData.linear_acceleration.y = self.accel_mps2[1]
        self._MsgData.linear_acceleration.z = self.accel_mps2[2]
        self._MsgData.linear_acceleration_covariance[0] = self.linear_accel_covariance
        self._MsgData.linear_acceleration_covariance[4] = self.linear_accel_covariance
        self._MsgData.linear_acceleration_covariance[8] = self.linear_accel_covariance
        
        self._MsgData.angular_velocity.x = self.gyro_rps[0]
        self._MsgData.angular_velocity.y = self.gyro_rps[1]
        self._MsgData.angular_velocity.z = self.gyro_rps[2]
        self._MsgData.angular_velocity_covariance[0] = self.angular_velocity_covariance
        self._MsgData.angular_velocity_covariance[4] = self.angular_velocity_covariance
        self._MsgData.angular_velocity_covariance[8] = self.angular_velocity_covariance
        
        self._MagMsgData.magnetic_field.x = self.mag_T[0]
        self._MagMsgData.magnetic_field.y = self.mag_T[1]
        self._MagMsgData.magnetic_field.z = self.mag_T[2]
        self._MagMsgData.magnetic_field_covariance[0] = self.magnetic_field_covariance
        self._MagMsgData.magnetic_field_covariance[4] = self.magnetic_field_covariance
        self._MagMsgData.magnetic_field_covariance[8] = self.magnetic_field_covariance
        
                
        if not rospy.is_shutdown():
            self._MsgPub.publish(self._MsgData)
            self._MagMsgPub.publish(self._MagMsgData)
            self._seq += 1

class BSA_Packet(object):
    def __init__(self):
        self.accel_mps2       = [0.0]*3
        self.gyro_rps         = [0.0]*3
        self.rpy_rad          = [0.0]*3
        self.linear_accel_covariance = 0.098 * 0.098
        self.angular_velocity_covariance = 0.012 * 0.012
        self.orientation_covariance = 0.035 * 0.035
        
        self.publish_data = False
        self._MsgData = Imu()
        self._MsgPub = rospy.Publisher('/segway/feedback/segway_bsa', Imu, queue_size=10)
        self._StatData = Int32()
        self._StatPub = rospy.Publisher('/segway/feedback/segway_bsa/stat', Int32, queue_size=10)
        self._MsgData.header.frame_id = 'segway_bsa_frame'
        self._seq = 0

    def Shutdown(self):
        self._MsgPub.unregister()
        self._StatPub.unregister()

    def parse_data(self,data):
        self.accel_mps2[0] = convert_u32_to_float(data[0]) * 9.81
        self.accel_mps2[1] = convert_u32_to_float(data[1]) * 9.81
        self.accel_mps2[2] = 0.0
        self.rpy_rad[1] = convert_u32_to_float(data[5]) * (math.pi/180.0)
        self.gyro_rps[1] = convert_u32_to_float(data[6]) * (math.pi/180.0)
        self.rpy_rad[0] = convert_u32_to_float(data[7]) * (math.pi/180.0)
        self.gyro_rps[0] = convert_u32_to_float(data[8]) * (math.pi/180.0)
        self.gyro_rps[2] = -convert_u32_to_float(data[9]) * (math.pi/180.0)
        self.rpy_rad[2] = 0.0
        self.pse_data_valid = data[10]

        self._MsgData.header.stamp = header_stamp
        self._MsgData.header.seq = self._seq
        
        q = tf.transformations.quaternion_from_euler(self.rpy_rad[0], self.rpy_rad[1], self.rpy_rad[2], 'sxyz')
        
        self._MsgData.orientation.x = q[0]
        self._MsgData.orientation.y = q[1]
        self._MsgData.orientation.z = q[2]
        self._MsgData.orientation.w = q[3]
        
        self._MsgData.orientation_covariance[0] = self.orientation_covariance
        self._MsgData.orientation_covariance[4] = self.orientation_covariance
        self._MsgData.orientation_covariance[8] = 999999.0
        
        self._MsgData.linear_acceleration.x = self.accel_mps2[0]
        self._MsgData.linear_acceleration.y = self.accel_mps2[1]
        self._MsgData.linear_acceleration.z = self.accel_mps2[2]
        self._MsgData.linear_acceleration_covariance[0] = self.linear_accel_covariance
        self._MsgData.linear_acceleration_covariance[4] = self.linear_accel_covariance
        self._MsgData.linear_acceleration_covariance[8] = 999999.0
        
        self._MsgData.angular_velocity.x = self.gyro_rps[0]
        self._MsgData.angular_velocity.y = self.gyro_rps[1]
        self._MsgData.angular_velocity.z = self.gyro_rps[2]
        self._MsgData.angular_velocity_covariance[0] = self.angular_velocity_covariance
        self._MsgData.angular_velocity_covariance[4] = self.angular_velocity_covariance
        self._MsgData.angular_velocity_covariance[8] = self.angular_velocity_covariance
        
        self._StatData = self.pse_data_valid 
        
                
        if not rospy.is_shutdown():
            self._StatPub.publish(self._StatData)
            self._MsgPub.publish(self._MsgData)
            self._seq += 1            
        
        
class RMP_IMU(object):
    def __init__(self):
        self._init_packets = True

    def Shutdown(self):
        if (True == has_segway_bsa):
            self.bsa.Shutdown()
        if (True == has_imu):
            self.imu.Shutdown()
    
    def parse_data(self,data):
        if (True == has_segway_bsa):
            if (True == self._init_packets):
                self.bsa = BSA_Packet() 
            self.bsa.parse_data(data[0:11])
        if (True == has_imu):
            if (True == self._init_packets):
                self.imu = IMU_Packet()
            self.imu.parse_data(data[11:])
                
        self._init_packets = False

class RMP_Dynamics:
    def __init__(self):
        self._use_platform_odometry = rospy.get_param('~use_platform_odometry',False)      
        self._MsgData = Dynamics()
        self._MsgPub = rospy.Publisher('/segway/feedback/dynamics', Dynamics, queue_size=10)
        self._jointStatePub = rospy.Publisher('rmp_joint_states', JointState, queue_size=10)
        self._jointStateMsg = JointState()
        
        
        if 2 == num_wheels:
            names = ['left_wheel','right_wheel']
        else:
            names = ['left_front_wheel','right_front_wheel','left_rear_wheel','right_rear_wheel']
            
        self._jointStateMsg.name = names
        
        self._MsgData.header.frame_id = ''
        self._jointStateMsg.header.frame_id = ''
  
        self._OdomData = Odometry()
        #self._OdomPub = rospy.Publisher('/segway/odometry/local_filtered', Odometry, queue_size=10)
        if (False == self._use_platform_odometry):
            self._OdomPub = rospy.Publisher('/segway/feedback/wheel_odometry', Odometry, queue_size=10)
        #    rospy.Subscriber('/segway/odometry/local_filtered', Odometry, self._update_odom_yaw)
        else:
            self._OdomPub = rospy.Publisher('/segway/odometry/local_filtered', Odometry, queue_size=10)
        self._OdomData.header.frame_id = 'odom'
        self._OdomData.child_frame_id  = 'base_link'
        
        
        self._OdomData.pose.covariance = [0.0017,0.0,0.0,0.0,0.0,0.0,
                                          0.0,0.0017,0.0,0.0,0.0,0.0,
                                          0.0,0.0,0.0017,0.0,0.0,0.0,
                                          0.0,0.0,0.0,0.00000,0.0,0.0,
                                          0.0,0.0,0.0,0.0,0.00000,0.0,
                                          0.0,0.0,0.0,0.0,0.0,0.0017]
             
        self._OdomData.twist.covariance = [0.0017,0.0,0.0,0.0,0.0,0.0,
                                           0.0,0.0017,0.0,0.0,0.0,0.0,
                                           0.0,0.0,0.0017,0.0,0.0,0.0,
                                           0.0,0.0,0.0,0.00000,0.0,0.0,
                                           0.0,0.0,0.0,0.0,0.00000,0.0,
                                           0.0,0.0,0.0,0.0,0.0,0.0017]
        
        self._seq = 0
        
    def Shutdown(self):
        self._MsgPub.unregister() 
        self._jointStatePub.unregister()
        self._OdomPub.unregister() 

    def parse(self,data):
        self._MsgData.header.stamp = header_stamp
        self._MsgData.header.seq = self._seq
        
        self._OdomData.header.stamp = header_stamp
        self._OdomData.header.seq = self._seq
        
        self._jointStateMsg.header.stamp = header_stamp
        self._jointStateMsg.header.seq = self._seq

        self._MsgData.x_vel_target_mps = convert_u32_to_float(data[0])
        self._MsgData.y_vel_target_mps = convert_u32_to_float(data[1])
        self._MsgData.yaw_rate_target_rps = convert_u32_to_float(data[2])
        self._MsgData.yaw_rate_limit_rps = convert_u32_to_float(data[3])
        self._MsgData.vel_limit_mps = convert_u32_to_float(data[4])
        
        temp = [convert_u32_to_float(data[5]),
                convert_u32_to_float(data[6]),
                convert_u32_to_float(data[7]),
                convert_u32_to_float(data[8])]
        self._MsgData.wheel_vel_mps = temp
        joint_vel = [(-1.0*temp[i]) for i in range(num_wheels)]
        temp = [convert_u32_to_float(data[9]),
                convert_u32_to_float(data[10]),
                convert_u32_to_float(data[11]),
                convert_u32_to_float(data[12])]
        self._MsgData.wheel_pos_m = temp
        joint_pos = [(-1.0*((temp[i]/wheel_circum) % 1.0) * (2 * math.pi)) for i in range(num_wheels)]
        self._MsgData.x_accel_mps2 = convert_u32_to_float(data[13])
        self._MsgData.y_accel_mps2 = convert_u32_to_float(data[14])
        self._MsgData.yaw_accel_mps2 = convert_u32_to_float(data[15])
        self._OdomData.twist.twist.linear.x = convert_u32_to_float(data[16])
        self._OdomData.twist.twist.linear.y = convert_u32_to_float(data[17])
        self._OdomData.twist.twist.linear.z = 0.0
        self._OdomData.twist.twist.angular.x = 0.0
        self._OdomData.twist.twist.angular.y = 0.0
        self._OdomData.twist.twist.angular.z = convert_u32_to_float(data[18])
        self._OdomData.pose.pose.position.x = convert_u32_to_float(data[19])
        self._OdomData.pose.pose.position.y = convert_u32_to_float(data[20])
        self._OdomData.pose.pose.position.z = 0.0
        self._MsgData.yaw_angle_rad = convert_u32_to_float(data[21])
        
        rot = tf.transformations.quaternion_from_euler(0,0,self._MsgData.yaw_angle_rad)
        pos = (self._OdomData.pose.pose.position.x,self._OdomData.pose.pose.position.y,0.0)
        
        self._OdomData.pose.pose.orientation.x = rot[0]
        self._OdomData.pose.pose.orientation.y = rot[1]
        self._OdomData.pose.pose.orientation.z = rot[2]
        self._OdomData.pose.pose.orientation.w = rot[3]      

        self._jointStateMsg.velocity = joint_vel
        self._jointStateMsg.position = joint_pos

        if not rospy.is_shutdown():
            self._OdomPub.publish(self._OdomData) 
            self._MsgPub.publish(self._MsgData)
            self._jointStatePub.publish(self._jointStateMsg)
            
            br = tf.TransformBroadcaster()
            br.sendTransform(pos,
                             rot,
                             header_stamp,
                             "base_link",
                             "odom")
            self._seq += 1


class RMP_Configuration:
    def __init__(self):
        self._MsgData = Configuration()
        self._MsgPub = rospy.Publisher('/segway/feedback/configuration', Configuration, queue_size=10)
        self._MsgData.header.frame_id = ''
        self._seq = 0 
        self.configuration_feedback = [0]*16

    def Shutdown(self):
        self._MsgPub.unregister()

    def SetTeleopConfig(self,data):
        self._MsgData.teleop_vel_limit_mps = data[0]
        self._MsgData.teleop_accel_limit_mps2 = data[1]
        self._MsgData.teleop_yaw_rate_limit_rps = data[2]
        self._MsgData.teleop_yaw_accel_limit_rps2 = data[3]           

    def parse(self,data):
        global wheel_circum
        self._MsgData.header.stamp = header_stamp
        self._MsgData.header.seq = self._seq
        
        self.configuration_feedback = data
        self._MsgData.vel_limit_mps = convert_u32_to_float(data[0])
        self._MsgData.accel_limit_mps2 = convert_u32_to_float(data[1])
        self._MsgData.decel_limit_mps2 = convert_u32_to_float(data[2])
        self._MsgData.dtz_decel_limit_mps2 = convert_u32_to_float(data[3])
        self._MsgData.yaw_rate_limit_rps = convert_u32_to_float(data[4])
        self._MsgData.yaw_accel_limit_rps2 = convert_u32_to_float(data[5])
        self._MsgData.lateral_accel_limit_mps2 = convert_u32_to_float(data[6])
        self._MsgData.tire_diameter_m = convert_u32_to_float(data[7])
        wheel_circum = self._MsgData.tire_diameter_m * math.pi
        self._MsgData.wheelbase_length_m = convert_u32_to_float(data[8])
        self._MsgData.omni_yaw_correction_factor = convert_u32_to_float(data[9])
        self._MsgData.omni_straffe_correction_factor = convert_u32_to_float(data[10])
        self._MsgData.wheel_track_width_m = convert_u32_to_float(data[11])
        self._MsgData.gear_ratio = convert_u32_to_float(data[12])
        self._MsgData.config_bitmap = data[13]
        self._MsgData.eth_ip_address = numToDottedQuad(data[14])
        self._MsgData.eth_port_number = data[15]
        self._MsgData.eth_subnet_mask = numToDottedQuad(data[16])
        self._MsgData.eth_gateway = numToDottedQuad(data[17])
        
        if not rospy.is_shutdown():
            self._MsgPub.publish(self._MsgData)
            self._seq += 1
            
class RMP_DATA:
    def __init__(self):
        self.status = RMP_Status()
        self.propulsion = RMP_Propulsion()
        self.auxiliary_power = RMP_AuxPower()
        self.config_param = RMP_Configuration()
        self.dynamics = RMP_Dynamics()
        self.imu = RMP_IMU()
        
    def unregister_topics(self):
        self.status.Shutdown()
        self.propulsion.Shutdown()
        self.auxiliary_power.Shutdown()
        self.config_param.Shutdown()
        self.dynamics.Shutdown()
        self.imu.Shutdown()       


        
    
