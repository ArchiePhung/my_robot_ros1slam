#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
App su dung cho viec test mach 
Developer: Phung Quy Duong
Company: STI
Latest Modify: 07/06/2023
    
Chức năng của node (back_end):
   + chuyển dữ liệu ROS sang frame truyền CAN
   + chuyển đổi frame truyền CAN sang dữ liệu ROS 
"""

from message_pkg.msg import CAN_send, CAN_received
# from sti_msgs.msg import *
# from geometry_msgs.msg import Twist
import time
import rospy
from std_msgs.msg import Int8
from ros_canbus.msg import *

class CAN_ROS():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('CAN_ROS', anonymous=False)
		self.rate = rospy.Rate(50)

		# ------------- PARAMETER ------------- #
		self.ID_RTC_ORIGIN = rospy.get_param('ID_RTC_ORIGIN', 1)
		self.ID_RTC        = rospy.get_param('ID_RTC', 2)
		self.ID_MAIN       = rospy.get_param('ID_MAIN', 3)
		self.ID_OC         = rospy.get_param('ID_OC', 4)
		self.ID_HC         = rospy.get_param('ID_HC', 5)
		self.ID_CPD        = rospy.get_param('ID_CPD', 6)
		# ------------- ROS ------------- #
		# - PUBLISH
		# --
		self.pub_statusRTC = rospy.Publisher("/RTC_info", Status_rtc, queue_size = 10)
		self.status_RTC = Status_rtc()
		# --
		self.pub_statusMain = rospy.Publisher("/MAIN_info", Status_main, queue_size = 10)
		self.status_Main = Status_main()
		# --
		self.pub_statusOC = rospy.Publisher("/OC_info", Status_oc, queue_size= 10)      
		self.status_oc = Status_oc()
		# -- 
		self.pub_statusHC = rospy.Publisher("/HC_info", Status_hc, queue_size = 20)
		self.status_hc = Status_hc()
		# -- 
		self.pub_statusCPD = rospy.Publisher("/CPD_info", Status_cpd, queue_size = 10)
		self.status_cpd = Status_cpd()
		
		# -- 

		self.pub_sendCAN = rospy.Publisher("/CAN_send", CAN_send, queue_size= 40)
		self.data_sendCan = CAN_send()
		self.frequence_sendCAN = 14. # - Hz
		self.saveTime_sendCAN = time.time()
		self.sort_send = 0

		# - SUBCRIBER
		# ------------- CAN ------------- #
		rospy.Subscriber("/request_rtc", Control_rtc, self.requestRTC_callback)
		self.data_requestRTC = Control_rtc()

		rospy.Subscriber("/request_main", Control_main, self.requestMAIN_callback)
		self.data_requestMAIN = Control_main()

		rospy.Subscriber("/request_oc", Control_oc, self.requestOC_callback)
		self.data_requestOC = Control_oc()

		rospy.Subscriber("/request_hc", Control_hc, self.requestHC_callback)
		self.data_requestHC = Control_hc()

		rospy.Subscriber("/request_cpd", Control_cpd, self.requestCPD_callback)
		self.data_requestCPD = Control_cpd()

		rospy.Subscriber("/CAN_received", CAN_received, self.CAN_callback)                        
		self.data_receivedCAN = CAN_received()

		# ------------- VAR ------------- #

	# -------------------
	
	def requestRTC_callback(self, data):
		self.data_requestRTC = data

	def requestMAIN_callback(self, data):
		self.data_requestMAIN = data

	def requestOC_callback(self, data):
		self.data_requestOC = data

	def requestHC_callback(self, data):
		self.data_requestHC = data

	def requestCPD_callback(self, data):
		self.data_requestCPD = data

	# ------------------- 
	def CAN_callback(self, data):
		self.data_receivedCAN = data
		# -- RECEIVED CAN
		self.analysisFrame_receivedCAN()

	def statusCAN_callback(self, data):
		self.CAN_status = data

	def convert_4byte_int(self, byte0, byte1, byte2, byte3):
		int_out = 0
		int_out = byte0 + byte1*256 + byte2*256*256 + byte3*256*256*256
		if int_out > pow(2, 32)/2.:
			int_out = int_out - pow(2, 32)
		return int_out

	def convert_16bit_int(self, bitArr):
		int_out = 0
		for i in range(16):
			int_out += bitArr[i]*pow(2, i)
		return int_out

	def getByte_fromInt16(self, valueIn, pos):
		byte1 = int(valueIn/256)
		byte0 =  valueIn - byte1*256
		if (pos == 0):
			return byte0
		else:
			return byte1

	def getBit_fromInt8(self, value_in, pos):
		bit_out = 0
		value_now = value_in
		for i in range(8):
			bit_out = value_now%2
			value_now = value_now//2
			if (i == pos):
				return bit_out

			if (value_now < 1):
				return 0		
		return 0

	def getBit_fromInt16(self, value_in, pos):
		bit_out = 0
		value_now = value_in
		for i in range(16):
			bit_out = value_now%2
			value_now = value_now//2
			if (i == pos):
				return bit_out

			if (value_now < 1):
				return 0		
		return 0
	
	def syntheticFrame_sendCAN(self):

		# -- RTC 
		if (self.sort_send == 0):
			self.data_sendCan.id = self.ID_RTC_ORIGIN
			self.data_sendCan.byte0 = self.ID_RTC
			self.data_sendCan.byte1 = self.data_requestRTC.rtc_value_writeCan
			self.data_sendCan.byte2 = self.data_requestRTC.rtc_can_sec
			self.data_sendCan.byte3 = self.data_requestRTC.rtc_can_sen
			self.data_sendCan.byte4 = self.data_requestRTC.rtc_ros_sec
			self.data_sendCan.byte5 = self.data_requestRTC.rtc_ros_sen
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 1

		# -- Main
		elif (self.sort_send == 1):
			self.data_sendCan.id = self.ID_RTC_ORIGIN
			self.data_sendCan.byte0 = self.ID_MAIN
			self.data_sendCan.byte1 = self.data_requestMAIN.main_charge
			self.data_sendCan.byte2 = self.data_requestMAIN.main_emc_write
			self.data_sendCan.byte3 = self.data_requestMAIN.main_emc_reset
			
			bitMain = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
			bitMain[0] = self.data_requestMAIN.main_sound1
			bitMain[1] = self.data_requestMAIN.main_sound2
			bitMain[2] = self.data_requestMAIN.main_sound3
			bitMain[3] = self.data_requestMAIN.main_sound4

			self.data_sendCan.byte4 = self.convert_16bit_int(bitMain)
			self.data_sendCan.byte5 = self.data_requestMAIN.main_value_writeCan
			self.data_sendCan.byte6 = self.data_requestMAIN.main_off_power
			self.sort_send = 2

		# -- OC 
		elif (self.sort_send == 2):
			self.data_sendCan.id = self.ID_RTC_ORIGIN
			self.data_sendCan.byte0 = self.ID_OC
			self.data_sendCan.byte1 = self.data_requestOC.oc_value_writeCan
			self.data_sendCan.byte2 = self.data_requestOC.manipulate_oc_bt_ccw
			self.data_sendCan.byte3 = self.data_requestOC.manipulate_oc_bt_cw
			self.data_sendCan.byte4 = self.data_requestOC.manipulate_oc_bn_ccw
			self.data_sendCan.byte5 = self.data_requestOC.manipulate_oc_bn_cw
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 3

		# -- HC
		elif (self.sort_send == 3):
			self.data_sendCan.id = self.ID_RTC_ORIGIN
			self.data_sendCan.byte0 = self.ID_HC
			self.data_sendCan.byte1 = self.data_requestHC.hc_value_writeCan

			bitHc = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
			bitHc[0] = self.data_requestHC.hc_led_red
			bitHc[1] = self.data_requestHC.hc_led_green
			bitHc[2] = self.data_requestHC.hc_led_blue

			self.data_sendCan.byte2 = self.convert_16bit_int(bitHc)
			self.data_sendCan.byte3 = self.data_requestHC.hc_sickzone1
			self.data_sendCan.byte4 = self.data_requestHC.hc_sickzone2
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 4

		# -- CPD
		elif (self.sort_send == 4):
			bitArr = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
			self.data_sendCan.id = self.ID_RTC_ORIGIN
			self.data_sendCan.byte0 = self.ID_CPD
			self.data_sendCan.byte1 = self.data_requestCPD.cpd_value_writeCan

			bitArr[0] = self.data_requestCPD.cpd_output1
			bitArr[1] = self.data_requestCPD.cpd_output2
			bitArr[2] = self.data_requestCPD.cpd_output3
			bitArr[3] = self.data_requestCPD.cpd_output4
			bitArr[4] = self.data_requestCPD.cpd_output5
			bitArr[5] = self.data_requestCPD.cpd_output6
			bitArr[6] = self.data_requestCPD.cpd_output7
			bitArr[7] = self.data_requestCPD.cpd_output8
			bitArr[8] = self.data_requestCPD.cpd_output9
			bitArr[9] = self.data_requestCPD.cpd_output10
			bitArr[10] = self.data_requestCPD.cpd_output11
			bitArr[11] = self.data_requestCPD.cpd_output12

			int16bit = self.convert_16bit_int(bitArr)
			byte0 = self.getByte_fromInt16(int16bit, 0)
			byte1 = self.getByte_fromInt16(int16bit, 1)

			# # print ("int: " + str(int16bit) + " Byte: " + str(byte0) + " | " + str(byte1) )
			self.data_sendCan.byte2 = byte0
			self.data_sendCan.byte3 = byte1
			self.data_sendCan.byte4 = 0
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			
			self.sort_send = 0
		
	def analysisFrame_receivedCAN(self):
		# -- RTC 
		if self.data_receivedCAN.idSend == self.ID_RTC:
			self.status_RTC.rtc_upload_status = self.data_receivedCAN.byte0
			self.status_RTC.rtc_value_canRead = self.data_receivedCAN.byte1
			self.pub_statusRTC.publish(self.status_RTC)

		# -- Main
		elif self.data_receivedCAN.idSend == self.ID_MAIN:
			self.status_Main.voltages_analog    = self.convert_4byte_int(self.data_receivedCAN.byte0, self.data_receivedCAN.byte1, 0, 0)
			self.status_Main.charge_analog      = self.convert_4byte_int(self.data_receivedCAN.byte2, self.data_receivedCAN.byte3, 0, 0)
			self.status_Main.stsButton_reset    = self.data_receivedCAN.byte4
			self.status_Main.stsButton_power    = self.data_receivedCAN.byte5//2
			self.status_Main.EMC_status 	    = self.data_receivedCAN.byte6
			self.status_Main.main_value_canRead = self.data_receivedCAN.byte7
			self.status_Main.main_upload_status = self.data_receivedCAN.byte5 - (self.data_receivedCAN.byte5//2)*2

			self.pub_statusMain.publish(self.status_Main)
			# print ("--- --- Main")

		# -- OC No.1 - CONVEYOR_No12
		elif (self.data_receivedCAN.idSend == self.ID_OC):
			self.status_oc.oc_upload_status = self.data_receivedCAN.byte0
			self.status_oc.oc_value_canRead = self.data_receivedCAN.byte1
			self.status_oc.sensor_s1 = self.data_receivedCAN.byte2
			self.status_oc.sensor_s3 = self.data_receivedCAN.byte3
			self.status_oc.sensor_s4 = self.data_receivedCAN.byte4

			self.status_oc.sensor_s5 = self.data_receivedCAN.byte5
			self.status_oc.sensor_s6 = self.data_receivedCAN.byte6
			self.status_oc.sensor_s7 = self.data_receivedCAN.byte7

			self.pub_statusOC.publish(self.status_oc)
		
		# -- HC
		elif self.data_receivedCAN.idSend == self.ID_HC:
			self.status_hc.hc_upload_status = self.data_receivedCAN.byte0
			self.status_hc.hc_value_canRead = self.data_receivedCAN.byte1

			self.status_hc.hc_sick_ahead0  = self.getBit_fromInt8(self.data_receivedCAN.byte2, 0)
			self.status_hc.hc_sick_ahead1  = self.getBit_fromInt8(self.data_receivedCAN.byte2, 1)
			self.status_hc.hc_sick_ahead2  = self.getBit_fromInt8(self.data_receivedCAN.byte2, 2)
			self.status_hc.hc_sick_ahead3  = self.getBit_fromInt8(self.data_receivedCAN.byte2, 3)

			self.status_hc.hc_sick_behind0 = self.getBit_fromInt8(self.data_receivedCAN.byte3, 0)
			self.status_hc.hc_sick_behind1 = self.getBit_fromInt8(self.data_receivedCAN.byte3, 1)
			self.status_hc.hc_sick_behind2 = self.getBit_fromInt8(self.data_receivedCAN.byte3, 2)
			self.status_hc.hc_sick_behind3 = self.getBit_fromInt8(self.data_receivedCAN.byte3, 3)

			self.status_hc.hc_badersock   = self.data_receivedCAN.byte4
			self.pub_statusHC.publish(self.status_hc)
			# print ("--- HC")

		# -- CPD
		elif (self.data_receivedCAN.idSend == self.ID_CPD):
			self.status_cpd.cpd_upload_status = self.data_receivedCAN.byte0
			self.status_cpd.cpd_value_canRead = self.data_receivedCAN.byte1
			int16Bit = self.convert_4byte_int(self.data_receivedCAN.byte2, self.data_receivedCAN.byte3, 0, 0)
			self.status_cpd.cpd_input1  = self.getBit_fromInt16(int16Bit, 0)
			self.status_cpd.cpd_input2  = self.getBit_fromInt16(int16Bit, 1)
			self.status_cpd.cpd_input3  = self.getBit_fromInt16(int16Bit, 2)
			self.status_cpd.cpd_input4  = self.getBit_fromInt16(int16Bit, 3)
			self.status_cpd.cpd_input5  = self.getBit_fromInt16(int16Bit, 4)
			self.status_cpd.cpd_input6  = self.getBit_fromInt16(int16Bit, 5)
			self.status_cpd.cpd_input7  = self.getBit_fromInt16(int16Bit, 6)
			self.status_cpd.cpd_input8  = self.getBit_fromInt16(int16Bit, 7)
			self.status_cpd.cpd_input9  = self.getBit_fromInt16(int16Bit, 8)
			self.status_cpd.cpd_input10 = self.getBit_fromInt16(int16Bit, 9)
			self.status_cpd.cpd_input11 = self.getBit_fromInt16(int16Bit, 10)
			self.status_cpd.cpd_input12 = self.getBit_fromInt16(int16Bit, 11)

			self.pub_statusCPD.publish(self.status_cpd)
			# print ("--- --- --- CPD")

	def try_run(self):
		val = 100
		print ("Bit0", self.getBit_fromInt8(val, 0))
		print ("Bit1", self.getBit_fromInt8(val, 1))
		print ("Bit2", self.getBit_fromInt8(val, 2))
		print ("Bit3", self.getBit_fromInt8(val, 3))
		print ("Bit4", self.getBit_fromInt8(val, 4))
		print ("Bit5", self.getBit_fromInt8(val, 5))
		print ("Bit6", self.getBit_fromInt8(val, 6))
		print ("Bit7", self.getBit_fromInt8(val, 7))

	def string_arry(self):
		arr_str = "0123456"
		print ("OUT0: ", arr_str[2:4])
		print ("OUT1: ", arr_str[0:2])
		print ("OUT2: ", arr_str[:2])
		print ("OUT3: ", arr_str[:-3])
		print ("OUT4: ", arr_str[-2:0])
		print ("OUT5: ", arr_str[2:0])

	def run(self):
		while not rospy.is_shutdown():
			# -- SEND CAN
			delta_time = (time.time() - self.saveTime_sendCAN)%60 
			if (delta_time > 1/self.frequence_sendCAN):
				self.saveTime_sendCAN = time.time()
				self.syntheticFrame_sendCAN()
				self.pub_sendCAN.publish(self.data_sendCan)

			self.rate.sleep()

def main():
	print('Program starting')
	program = CAN_ROS()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()



