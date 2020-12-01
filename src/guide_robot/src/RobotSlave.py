
import rospy
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray


class RobotSlave():
    def __init__(self):
        
        rospy.init_node("robot_s_node", log_level=rospy.INFO)
        self.comm_pub = rospy.Subscriber(
            "/comm", String, self.comm_callback, queue_size=10)
        self.tag_detections_sub = rospy.Subscriber(
            "tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10)
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
        pass

def main():
    slave = RobotSlave()
    rospy.spin()

if name == "__main__":
    main()