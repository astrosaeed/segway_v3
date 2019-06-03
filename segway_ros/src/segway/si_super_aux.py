"""--------------------------------------------------------------------
COPYRIGHT 2014 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company") 
for its licensed Segway RMP Robotic Platforms is intended and supplied to you, 
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
 
 \file   si_super_aux.py

 \brief  runs the driver for the high capacity 24V auxiliary power
         system

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from system_defines import *
from utils import *
from io_eth import IoEthThread
from segway_msgs.msg import SuperAux
import multiprocessing
import threading
import select
import rospy
import operator

SUPER_AUX_PACKET_SIZE = 36

class SuperAuxDriver(object):
    def __init__(self,rmp_ip='10.66.171.5'):       
        self.init_success = False
        
        """
        Create the thread to run the super aux interface
        """
        self._cmd_buffer = multiprocessing.Queue()
        self.tx_queue_ = multiprocessing.Queue()
        self.rx_queue_ = multiprocessing.Queue()
        self.comm = IoEthThread((rmp_ip,6236),
                                self.tx_queue_,
                                self.rx_queue_,
                                max_packet_size=SUPER_AUX_PACKET_SIZE)
                                    
        
        if (False == self.comm.link_up):
            rospy.logerr("Could not open socket for super aux...exiting")
            self.Shutdown()
            return

        """
        Initialize the publishers and subscribers for the node
        """
        self.battery_data = SuperAux()
        self.battery_pub = rospy.Publisher("/segway/feedback/super_aux", SuperAux, queue_size=10)

        """
        Start the receive handler thread
        """
        self.need_to_terminate = False
        self.terminate_mutex = threading.RLock()
        self.last_rsp_rcvd = rospy.get_time()
        self._rcv_thread   = threading.Thread(target = self._run)
        self._rcv_thread.start()

        rospy.loginfo("Super Aux interface is up and running")
        self.init_success = True
        
    def Shutdown(self):
        with self.terminate_mutex:
            self.need_to_terminate = True
        rospy.loginfo("Super Aux interface has called the Shutdown method, terminating")
        self.battery_pub.unregister()
        self.comm.Close()
        self.tx_queue_.close()
        self.rx_queue_.close()    
    
    def _run(self):
        while not self.need_to_terminate:
            """
            Run until signaled to stop
            Perform the actions defined based on the flags passed out
            """
            result = select.select([self.rx_queue_._reader],[],[],0.02)
            if len(result[0]) > 0:
                data = result[0][0].recv()
                if (SUPER_AUX_PACKET_SIZE == len(data)):
                    with self.terminate_mutex:
                        if not self.need_to_terminate:
                            self._handle_rsp(data)

    def _handle_rsp(self,data_bytes):
        valid_data,rsp_data = validate_response(data_bytes)
        
        if (False == valid_data):
            self.last_rsp_rcvd = rospy.get_time()
            rospy.logerr("bad super aux data packet")
            return
        
        self.last_rsp_rcvd = rospy.get_time()
        
        rsp = [convert_u32_to_float(i) for i in rsp_data]
        rsp[7] = rsp_data[7]

        self.battery_data.header.stamp = rospy.get_rostime()
        self.battery_data.header.seq +=1
        
        self.battery_data.battery_state_of_charge_percent = rsp[0]
        self.battery_data.battery_current_A0pk = rsp[1]
        self.battery_data.battery_voltage_V = rsp[2]
        self.battery_data.max_cell_temp_degc = rsp[3]
        self.battery_data.max_pcba_temp_degc = rsp[4]
        self.battery_data.max_cell_voltage_V = rsp[5]
        self.battery_data.min_cell_voltage_V = rsp[6]
        self.battery_data.battery_interface_status_bits = rsp[7]
       
        if not rospy.is_shutdown():
            self.battery_pub.publish(self.battery_data)
            
        rospy.logdebug("feedback received from super aux")
        
