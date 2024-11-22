#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import các thư viện cần thiết
import rospy
from geometry_msgs.msg import Twist           # Thư viện ROS để nhận lệnh vận tốc
from std_msgs.msg import Int16      # Để gửi dữ liệu dạng mảng số nguyên
import time

from math import pi                           # Sử dụng số pi cho tính toán góc và vận tốc

class ControlMotorByKinematic():
    def __init__(self):
        # Khởi tạo node ROS tên 'control_motor' 
        rospy.init_node('control_motor', anonymous=False)
        self.rate = rospy.Rate(30)  # Tần số thực hiện vòng lặp là 30 Hz

        # Hướng mặc định của động cơ trái và phải
        self.dir_motorLeft = 0
        self.dir_motorRight = 1

        # Các tham số vật lý của hệ thống
        self.r_banh = rospy.get_param("r_banh", 0.034)           # Bán kính bánh xe
        self.kc_hai_banh = rospy.get_param("kc_hai_banh", 0.26)   # Khoảng cách giữa hai bánh xe
        self.v_max_dong_co = rospy.get_param("v_max_dong_co", 10000) # Tốc độ tối đa của động cơ
        self.pwm_max = rospy.get_param("pwm_max", 255)     
        self.pwm_min = 40       # PWM tối thiểu
        self.hop_giam_toc = rospy.get_param("hop_giam_toc", 78) # Tỉ số giảm tốc của hộp số

        # Đăng ký callback để nhận lệnh vận tốc từ topic /cmd_vel
        rospy.Subscriber("/cmd_vel", Twist, self.cmdVel_callback)	
        self.data_cmdVel = Twist() 
        self.is_cmdVel = 0 

        self.pub_queryMotor_left = rospy.Publisher('/left_wheel_query', Int16, queue_size=10)
        self.pub_queryMotor_right = rospy.Publisher('/right_wheel_query', Int16, queue_size=10)

    # Hàm callback khi có dữ liệu mới từ topic /cmd_vel
    def cmdVel_callback(self, data):
        self.data_cmdVel = data
        self.is_cmdVel = 1

    # Hàm chuyển vận tốc góc bánh xe thành giá trị PWM
    def convert_pwm(self, v_banh):
        pwm = v_banh

        # pwm = (pwm / self.v_max_dong_co) * self.pwm_max
        return int(pwm)

    # Hàm tính toán vận tốc góc của các bánh từ vận tốc và góc quay của xe
    def donghocnghich(self, v_dai, v_goc):
        vtgoc_trai = (1.0 / (2.0 * self.r_banh)) * (2.0 * v_dai - self.kc_hai_banh * v_goc)
        vtgoc_phai = (1.0 / (2.0 * self.r_banh)) * (2.0 * v_dai + self.kc_hai_banh * v_goc)

        v_banh_trai = self.convert_pwm(vtgoc_trai * (60 / (2 * pi)))  # Chuyển đổi thành vòng/phút
        v_banh_phai = self.convert_pwm(vtgoc_phai * (60 / (2 * pi)))

        return v_banh_trai, v_banh_phai

    def run_test(self):
        pwm_v_trai, pwm_v_phai = self.donghocnghich(0.1, 0.0)
        print(pwm_v_trai, pwm_v_phai)
    
    # Hàm điều khiển chính
    def run(self):
        while not rospy.is_shutdown():
            if self.is_cmdVel == 1:
                # Tính toán giá trị PWM cho bánh trái và bánh phải
                pwm_v_trai, pwm_v_phai = self.donghocnghich(self.data_cmdVel.linear.x, self.data_cmdVel.angular.z)

                print(pwm_v_trai, pwm_v_phai)

                msg_left = Int16()
                msg_left.data = pwm_v_trai

                msg_right = Int16()
                msg_right.data = pwm_v_phai

                # -- 
                self.pub_queryMotor_left.publish(msg_left)
                self.pub_queryMotor_right.publish(msg_right)

            self.rate.sleep()

# Hàm main để khởi động chương trình
def main():
    print('Starting main program')
    controlMT = ControlMotorByKinematic()
    controlMT.run()

# Chạy chương trình nếu script được gọi trực tiếp
if __name__ == '__main__':
    main()
