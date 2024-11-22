#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dev: Archie Phung
Date Modify: 16/8/2024

Advantage:
  From tf coef -> position tolerance < 1 cm

Function:
  - Spin AGV to precicise angle with odometry data
  - Exec command from keyboard
  - Intergrate with remote function of AGV

"""

import rospy
from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs, acos, degrees

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Twist, PoseWithCovarianceStamped

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from sick_lidar_localization import OdometryMessage0105
from message_pkg.msg import *
from sti_msgs.msg import *
from ros_canBus.msg import *

class GetTFCoef():
    def __init__(self):
        print("ROS Initial: GetTFCoef !")
        rospy.init_node('get_tf_coef', anonymous = False, disable_signals=True) # False

        self.rate = rospy.Rate(40)

        # self.tolerance_rot_step1 = rospy.get_param('~tolerance_rot_step1',0.02)
        # self.vel_rot_step1 = rospy.get_param('~vel_rot_step1',0.32)      # 0.45
        self.vel_rot_step1 = 0.2

        # -- SUBSCRIBER NODE -- 
        # -- Keyboard command
        rospy.Subscriber("/Keyboard_cmd", Keyboard_command, self.callback_Keyboard) # 
        self.keyboard_cmd = Keyboard_command()	

        # -- App
        rospy.Subscriber("/app_button", App_button, self.callback_appButton) # lay thong tin trang thai nut nhan tren man hinh HMI.
        self.app_button = App_button()

        # -- Safety zone
        rospy.Subscriber("/HC_info", HC_info, self.zone_callback)
        self.zone_lidar = HC_info()

        # rospy.Subscriber('/robot_pose', PoseStamped, self.getPose, queue_size = 20)
        # self.is_pose_robot = False
        # self.poseRbMa = Pose()
        # self.theta_rb_ht = 0.0

        rospy.Subscriber('/raw_odom', Odometry, self.getPose_Odometry, queue_size = 20)
        self.poseRbMa_odom = Pose()
        self.theta_rb_odom = 0.0

        rospy.Subscriber('/odom_combined', Odometry, self.getPose_OdometryEkf, queue_size = 20)
        self.poseRbMa_odomekf = Pose()
        self.theta_rb_odomekf = 0.0

        rospy.Subscriber('/NN_infoRespond', NN_infoRespond, self.callback_NNinfoRespond, queue_size = 20)
        self.NN_infoRespond = NN_infoRespond()

        # -- PUBLISH TOPIC -- 
        # self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # -- CONST --
        self.ANGLE1 = 358.20*PI/180.0
        self.ANGLE2 = 88.20*PI/180.0
        self.ANGLE3 = 178.20*PI/180.0
        self.ANGLE4 = 268.2*PI/180.0
        self.DELTA_ANGLE_TARGET = 0.015
        self.VEL_RB_MIN = 0.05
        
        # -- VAR --
        self.total_x = 0.0
        self.total_y = 0.0
        self.total_z = 0.0
        self.step_simple = 0
        self.number_coorSample = 0
        self.coorAverage_x = 0.0
        self.coorAverage_y = 0.0
        self.coorAverage_z = 0.0
        self.coor_deviation_x = 0.0
        self.coor_deviation_y = 0.0

        self.reference_point_x = 0
        self.reference_point_y = 0
        self.number_spin_simple = 0
        self.allow_spin_simple = 0

        self.angle_target = 0
        self.pre_angle_target = 0
        self.angle_giam_toc = 45.0*PI/180.0

        self.flag_reset = 0
        self.flag_restart = 0
        self.flag_spin = 0
        self.flag_stop = 0
        self.flag_get_coor_simple = 0
        self.flag_auto = 0
        self.flag_get_pos = 0
        self.is_exit = 0

        self.rate_cmdvel = 30
        self.time_tr = rospy.get_time()

        # --- for get pos of AGV when it run straight
        self.NUMBER_GET_POS = 5
        self.TIME_FOR_RUN_LINE = 30  # s
        self.TIME_FOR_EACH_GET_POS = int(self.TIME_FOR_RUN_LINE/ self.NUMBER_GET_POS)

        self.list_posX_line = []
        self.list_posY_line = []
        self.list_posZ_line = []

        self.ct_get_pos = 0
        self.ct_time_get_pos = 0

        self.VEL_LINEAR_X = 0.2

        self.step_run_line = 0


    ############################################ CALLBACK FUNCTION ########################################################################3
    def callback_Keyboard(self, data):
        self.keyboard_cmd = data
        if self.keyboard_cmd.value == "":
            self.flag_reset = 0	

        elif self.keyboard_cmd.value == "get_pos":     # spin forward
            self.flag_get_pos = 1

        elif self.keyboard_cmd.value == "reset":
            self.flag_reset = 1

        elif self.keyboard_cmd.value == "restart":
            self.flag_restart = 1

    def callback_appButton(self, dat):
        self.app_button = dat

    def zone_callback(self, data):
        self.zone_lidar = data

    def callback_NNinfoRespond(self, data):
        self.NN_infoRespond = data

    def getPose_Odometry(self, data):
        self.poseRbMa_odom = data.pose.pose
        quata = ( self.poseRbMa_odom.orientation.x,\
                self.poseRbMa_odom.orientation.y,\
                self.poseRbMa_odom.orientation.z,\
                self.poseRbMa_odom.orientation.w )
        euler = euler_from_quaternion(quata)

        self.theta_rb_odom = euler[2]
        if self.theta_rb_odom <= 0:
            self.theta_rb_odom = self.theta_rb_odom + 2*PI

    def getPose_OdometryEkf(self, data):
        self.poseRbMa_odomekf = data.pose.pose
        quata = ( self.poseRbMa_odomekf.orientation.x,\
                self.poseRbMa_odomekf.orientation.y,\
                self.poseRbMa_odomekf.orientation.z,\
                self.poseRbMa_odomekf.orientation.w )
        euler = euler_from_quaternion(quata)

        self.theta_rb_odomekf = euler[2]
        if self.theta_rb_odomekf <= 0:
            self.theta_rb_odomekf = self.theta_rb_odomekf + 2*PI        

    ############################################ DEF FUNCTION ########################################################################3
    def pub_cmdVel(self, twist , rate):

        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass
            
    def log_mess(self, mess):
        if self.enb_log == 1:
            if self.step_log == 0:
                self.ct_log = rospy.get_time()
                self.step_log = 1
                print(mess)
            else:
                if rospy.get_time() - self.ct_log >= self.TIME_LOG:
                    print(mess)
                    self.ct_log = rospy.get_time()
        else:
            self.step_log = 0

    def turn_ar(self, theta, target_theta, vel_rot):
        delta_theta = theta - target_theta
        # print("Góc và độ lệch hiện tại là: {x}, {y}".format(x=round(degrees(theta), 3), y = round(degrees(delta_theta), 3)))

        if -self.DELTA_ANGLE_TARGET/2 > delta_theta or delta_theta > self.DELTA_ANGLE_TARGET/2: # +- 10 do
            if target_theta >= 0: #quay trai
                # print "b"
                if fabs(delta_theta) <= self.angle_giam_toc:
                    # print('hhhhhhhhhhh')
                    vel_th = (fabs(delta_theta)/self.angle_giam_toc)*vel_rot
                else:
                    vel_th = vel_rot

                if vel_th < self.VEL_RB_MIN:
                    vel_th = self.VEL_RB_MIN

                # vel_th = fabs(theta) + 0.1
                # if vel_th > vel_rot : vel_th = vel_rot
                return vel_th

            elif target_theta < 0: #quay phai , vel_z < 0
                # print "a"
                if fabs(delta_theta) <= self.angle_giam_toc:
                    # print('hhhhhhhhhhhh')
                    vel_th = (fabs(delta_theta)/self.angle_giam_toc)*(-vel_rot)
                else:
                    vel_th = -vel_rot

                if vel_th > -self.VEL_RB_MIN:
                    vel_th = -self.VEL_RB_MIN

                # vel_th = -fabs(theta) - 0.1
                # if vel_th < -vel_rot : vel_th = -vel_rot
                return vel_th
                # buoc = 1

        else : 
            return -10

    def stop(self):
        for i in range(2):
            self.pub_cmd_vel.publish(Twist())

    def reset(self):
        pass

    def restart(self):
        pass	

    def shutdown(self):
        self.is_exit = 1
        
    ############################################ LOOP FUNCTION ########################################################################3
    def run(self):
        try:
            if self.is_exit == 0:
                while not rospy.is_shutdown():
                    if self.flag_get_pos == 1:
                        self.flag_get_pos = 0
                        print("Tọa độ AGV odom là: {x}, {y}, {r}".format(x = self.poseRbMa_odomekf.position.x, y = self.poseRbMa_odomekf.position.y, r = round(degrees(self.theta_rb_odomekf), 3)))
                        print("Tọa độ AGV từ lidar là: {x}, {y}, {r}".format(x = self.NN_infoRespond.x, y = self.NN_infoRespond.y, r = round(degrees(self.NN_infoRespond.z), 3)))
                        print("Tọa độ AGV raw odom là: {x}, {y}, {r}".format(x = self.poseRbMa_odom.position.x, y = self.poseRbMa_odom.position.y, r = round(degrees(self.theta_rb_odom), 3)))
                        print("--------------------------------------------------------------------")

                    self.rate.sleep()
        except KeyboardInterrupt:
            rospy.on_shutdown(self.shutdown)
            self.is_exit = 1
            print('!!FINISH!!')

def main():
	print('Starting main program')
	program = GetTFCoef()
	program.run()
	print('Exiting main program')	

if __name__ == '__main__':
    main()
    