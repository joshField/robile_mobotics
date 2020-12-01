
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
        self.tag_dict = {}

    def tag_callback(self, msg):
        """
        Updates the pose for detected apriltags

        Args:
            msg (AprilTagDetectionArray): detected april tags in current view
        """

        for tag in msg:
            tag_id = tag.id[0]
            self.tag_dict[tag_id] = tag.pose

    def comm_callback(self, msg):
        """
        Twist velocity command in the form of a String

        Args:
            msg (String): a Twist velocity command in a String
        """

        v = json.loads(msg.data)

        vel_cmd = Twist()
        vel_cmd.linear.x = v["linear"]["x"]
        vel_cmd.linear.y = v["linear"]["y"]
        vel_cmd.linear.z = v["linear"]["z"]
        vel_cmd.angular.x = v["angular"]["x"]
        vel_cmd.angular.y = v["angular"]["y"]
        vel_cmd.angular.z = v["angular"]["z"]

        self.vel_pub.publish(vel_cmd)


def main():
    slave = RobotSlave()
    rospy.spin()

if name == "__main__":
    main()