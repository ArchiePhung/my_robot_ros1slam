#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from tf import TransformListener
from tf.transformations import euler_from_quaternion

class PurePursuitPlanner:
    def __init__(self):
        # Khởi tạo node ROS
        rospy.init_node('pure_pursuit_global_planner', anonymous=True)

        # Đăng ký subscriber để nhận quỹ đạo từ global planner
        self.path_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.path_callback)

        # Publisher điều khiển robot
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Tham số Pure Pursuit
        self.lookahead_distance = 0.7  # Khoảng cách nhìn trước (m)
        self.linear_velocity = 0.2    # Vận tốc tuyến tính cố định (m/s)

        # TF listener để lấy tọa độ của robot
        self.tf_listener = TransformListener()

        # Quỹ đạo hiện tại
        self.path = []
        self.path_updated = False

    def path_callback(self, msg):
        """Callback nhận quỹ đạo từ global planner."""
        self.path = msg.poses
        self.path_updated = True
        rospy.loginfo("Received new global plan with %d points.", len(self.path))

    def calculate_control_command(self):
        """Tính toán vận tốc tuyến tính (v) và góc (w) dựa trên Pure Pursuit."""
        if not self.path:
            rospy.logwarn("No global plan available.")
            return Twist()

        # Lấy tọa độ hiện tại của robot
        try:
            (trans, rot) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
        except Exception as e:
            rospy.logwarn("TF lookup failed: %s", str(e))
            return Twist()

        robot_x, robot_y = trans[0], trans[1]
        _, _, robot_yaw = self.quaternion_to_euler(rot)

        # Tìm điểm "lookahead" trên quỹ đạo
        lookahead_point = self.get_lookahead_point(robot_x, robot_y)
        if not lookahead_point:
            rospy.logwarn("No lookahead point found.")
            return Twist()

        # Tính toán điều khiển
        cmd_vel = Twist()
        target_x, target_y = lookahead_point.pose.position.x, lookahead_point.pose.position.y

        # Chuyển đổi điểm "lookahead" sang hệ tọa độ của robot
        dx = target_x - robot_x
        dy = target_y - robot_y
        lookahead_distance = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_target - robot_yaw)

        # Điều khiển theo Pure Pursuit
        cmd_vel.linear.x = self.linear_velocity
        cmd_vel.angular.z = 2 * self.linear_velocity * math.sin(angle_error) / lookahead_distance

        print(cmd_vel.linear.x, cmd_vel.angular.z)

        return cmd_vel

    def get_lookahead_point(self, robot_x, robot_y):
        """Tìm điểm 'lookahead' dựa trên khoảng cách."""
        for point in self.path:
            path_x = point.pose.position.x
            path_y = point.pose.position.y
            distance = math.sqrt((path_x - robot_x)**2 + (path_y - robot_y)**2)
            if distance >= self.lookahead_distance:
                return point
        return None

    def quaternion_to_euler(self, quat):
        """Chuyển đổi quaternion thành Euler angles."""
        return euler_from_quaternion([quat[0], quat[1], quat[2], quat[3]])

    def normalize_angle(self, angle):
        """Chuẩn hóa góc về khoảng [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def run(self):
        """Vòng lặp chính."""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.path:
                cmd_vel = self.calculate_control_command()
                self.cmd_pub.publish(cmd_vel)
            rate.sleep()

if __name__ == "__main__":
    try:
        # Khởi tạo Pure Pursuit Planner
        planner = PurePursuitPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass