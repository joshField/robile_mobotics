
import rospy
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Twist
import json


class RobotSlave():
    def __init__(self):
        
        rospy.init_node("robot_s_node", log_level=rospy.INFO)
        self.comm_pub = rospy.Subscriber(
            "/comm", String, self.comm_callback, queue_size=10)
        self.tag_detections_sub = rospy.Subscriber(
            "tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.guidance = False

    def tag_callback(self, msg):
        """
        Updates the pose for detected apriltags

        Args:
            msg (AprilTagDetectionArray): detected april tags in current view
        """

        for tag in msg:
            tag_id = tag.id[0]

            #TODO: follow tag if its the leader
            if tag_id == "8" and self.guidance:
                vel_cmd = Twist()

                self.tag_dict[tag_id] = tag.pose

                vel_cmd.linear.x = 
                vel_cmd.linear.y = 0
                vel_cmd.linear.z = 0

                vel_cmd.angular.x = 0
                vel_cmd.angular.y = 0
                vel_cmd.angular.z = 

                self.vel_cmd.publish(vel_cmd)
                # self.rate.sleep()

    def comm_callback(self, msg):
        """
        Comm callback to set guidance mode.

        Args:
            msg (String): a boolean ("True" or "False")
        """

        if msg.data == "True":
            self.guidance = True
        else:
            self.guidance = False


def main():
    slave = RobotSlave()
    rospy.spin()

if name == "__main__":
    main()