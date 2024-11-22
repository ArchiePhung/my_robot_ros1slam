#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dev: Archie Phung
Date Modify: 06/03/2024
"""
import rospy
from message_pkg.msg import *

class KeyBoard_Command():
	def __init__(self):
		print("ROS Initial: KeyBoard_Command!")
		rospy.init_node('keyboard_command', anonymous = False, disable_signals=True) # False

		self.rate = rospy.Rate(10)
		# -
		self.pub_keyboard_command = rospy.Publisher("/Keyboard_cmd", Keyboard_command, queue_size = 20)	# Dieu khien Toyo.
		self.Keyboard_command = Keyboard_command()

		# -- CONST --
		self.TIME_SAVEDATA = 2
		# -- VAR -- 
		self.input = ""
		self.ct_sendcmd = rospy.get_time()
		self.step = 0
		self.is_exit = 0

	def shutdown(self):
		self.is_exit = 1

	def run(self):
		try:
			if self.is_exit == 0:
				while not rospy.is_shutdown():
					if self.input == "":
						self.input = input("Please enter command: ")
					
					else:
						if self.step == 0:
							time_curr = rospy.get_time()
							self.Keyboard_command.value = self.input
							self.step = 1

						elif self.step == 1:
							d = (time_curr - self.ct_sendcmd)
							if (d > float(1/self.TIME_SAVEDATA)): # < 20hz 
								# self.pre_timeBoard = time_curr
								self.Keyboard_command.value = ""
								self.input = ""
								self.step = 0
					
						self.pub_keyboard_command.publish(self.Keyboard_command)

					self.rate.sleep()
		except KeyboardInterrupt:
			rospy.on_shutdown(self.shutdown)
			self.is_exit = 1
			print('!!FINISH!!')

def main():
	print('Starting main program')
	program = KeyBoard_Command()
	program.run()
	print('Exiting main program')	

if __name__ == '__main__':
    main()
    