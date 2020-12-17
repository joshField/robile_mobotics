#! /usr/bin/env python

import rospy
import tf2_ros
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
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(1.0)
        
        self.comm_sub = rospy.Subscriber(
            "/comm", String, self.comm_callback, queue_size=10)
        self.comm_pub = rospy.Publisher("/comm", String, queue_size=10)
        self.tag_detections_sub = rospy.Subscriber(
            "tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.guidance = True
        self.target_id = None
        self.MAX_lin_VEL = 1.00                         #set max follow velocity
        self.MAX_ang_VEL = 0.5
        self.last_detection = rospy.get_time()
        self.id = 0  # TODO: Implement ROS param for getting this and passing from launch file
        self.kP = 0.5 #PID gains for angular velocity controller
        self.kI = 0.0
        self.kD = 0.01
        self.last_heading_error = 0
        self.error_sum = 0
        self.master_id = 7

    def tag_callback(self, msg):
        """
        Updates the pose for detected apriltags

        Args:
            msg (AprilTagDetectionArray): detected april tags in current view
        """
        for tag in msg.detections:
            tag_id = tag.id
            rospy.loginfo_throttle(5.0, "Detected tag: %d" % tag_id)

            #Follow tag if its the leader and in guidance mode
            if tag_id == self.master_id and self.guidance:
                
                self.last_detection = rospy.get_time()
                rospy.loginfo_throttle(5.0, "Following Master")
                vel_cmd = Twist()

                # Grab tag position relative to robot
                try:
                    trans = self.tfBuffer.lookup_transform(f'{rospy.get_namespace().split("/")[1]}/base_footprint', f'{rospy.get_namespace().split("/")[1]}/tag_{self.master_id:03d}', rospy.Time(0))
                    pos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]) 
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(e)
                    continue
                
                #parse tag position
                dist = np.linalg.norm(pos)
                maxDist = 5
                heading_error = np.arctan2(pos[1], pos[0]) #angle between plane of tag on m and the front plane of slave robot

                if abs(heading_error) < 10*np.pi/180:        #only send linear velocity when within a specified heading
                    vel_cmd.linear.x = self.MAX_lin_VEL * (dist/maxDist)
                else:
                    #Adjust heading using PID control
                    vel_cmd.angular.z = (heading_error * self.kP) + (self.last_heading_error * self.kD) + (self.error_sum * self.kI)
                    self.last_heading_error = heading_error #previous heading error for derivative control
                    self.error_sum += heading_error #sum of error for integral control
                vel_cmd.linear.y = 0
                vel_cmd.linear.z = 0

                vel_cmd.angular.x = 0
                vel_cmd.angular.y = 0

                self.vel_pub.publish(vel_cmd)

            elif tag_id == self.target_id:
                self.guidance = False
                #todo: goto target tag
                

    def comm_callback(self, msg):
        """
        Comm callback to set guidance mode.

        Args:
            msg (String): a dict containing from_slave, guidance, and lost booleans
        """
        comm = json.loads(msg.data)

        if not comm['from_slave'] and self.id == comm['slave_id']:
            if comm['guidance'] == True:
                self.guidance = True
                self.target_id = comm['target_id']
            else:
                self.guidance = False

    def run(self):
        """
        Run common loop for slave following master. If it loses track stop moving.
        """
        #check time since last detection, if lost, kill velocity commands, send lost message over \comm
        if rospy.get_time() - self.last_detection > 0.5:
            rospy.loginfo_throttle(5.0, 'Stopping Slave')
            self.vel_pub.publish(Twist())
            comm = {
                'guidance': self.guidance,
                'from_slave': True,
                'slave_id': self.id,
                'targe_id': self.target_id,
                'lost':  True
            }
            str_cmd = String()
            str_cmd.data = json.dumps(comm)
            self.comm_pub.publish(str_cmd)


def main():
    slave = RobotSlave()
    while not rospy.is_shutdown():
        slave.run()


if __name__=="__main__":
    main()

    