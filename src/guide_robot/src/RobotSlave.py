#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import json
import numpy as np
from scipy.spatial.transform import Rotation as R


class RobotSlave():
    def __init__(self):
        
        rospy.init_node("robot_s_node", log_level=rospy.INFO)
        self.comm_sub = rospy.Subscriber(
            "/comm", String, self.comm_callback, queue_size=10)
        self.comm_pub = rospy.Publisher("/comm", String, queue_size=10)
        self.tag_detections_sub = rospy.Subscriber(
            "/robot_m/tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10)
        self.vel_pub = rospy.Publisher("/robot_m/cmd_vel", Twist, queue_size=10)
        self.guidance = True
        self.MAX_VEL = 0.26
        self.rate = rospy.Rate(5)
        self.recent_time = rospy.get_time()

    def tag_callback(self, msg):
        """
        Updates the pose for detected apriltags

        Args:
            msg (AprilTagDetectionArray): detected april tags in current view
        """
        self.recent_time = rospy.get_time()
        for tag in msg.detections:
            tag_id = tag.id

            #TODO: follow tag if its the leader
            if tag_id == 1 and self.guidance:
                vel_cmd = Twist()
                
                #parse tag position
                tag_pose = tag.pose.pose
                pos = np.array([tag_pose.position.x, tag_pose.position.y, tag_pose.position.z])
                dist = np.linalg.norm(pos)
                maxDist = 5
                angle = np.arctan2(pos[0], pos[1])

                imu = rospy.wait_for_message("/robot_m/imu", Imu)
                quat = np.array([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
                r = R.from_quat(quat)
                rpy = r.as_euler('XYZ', degrees=False)
                heading = rpy[2]

                vel_cmd.linear.x = self.MAX_VEL * (dist/maxDist)
                vel_cmd.linear.y = 0
                vel_cmd.linear.z = 0

                vel_cmd.angular.x = 0
                vel_cmd.angular.y = 0
                vel_cmd.angular.z = (heading - angle) * 0.025

                self.vel_pub.publish(vel_cmd)
                # self.rate.sleep()
                # self.vel_pub.publish(Twist())
                

    def comm_callback(self, msg):
        """
        Comm callback to set guidance mode.

        Args:
            msg (String): a dict containing from_slave, guidance, and lost booleans
        """
        comm = json.loads(msg.data)

        if not comm['from_slave']:
            if comm['guidance'] == True:
                self.guidance = True
            else:
                self.guidance = False

    def run(self):
        """
        Run common loop for slave following master. If it loses track stop moving.
        """
        
        if rospy.get_time() - self.recent_time < 0.05:
            self.vel_pub.publish(Twist())
            comm = {
                'from_slave': True,
                'lost':  True
            }
            str_cmd = String()
            str_cmd.data = json.dumps(comm)
            self.comm_pub.publish(str_cmd)


def main():
    slave = RobotSlave()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        slave.run()
        rate.sleep()


if __name__=="__main__":
    main()

    