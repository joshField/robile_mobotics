
import rospy
import actionlib
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class RobotMaster():
    def __init__(self):

        rospy.init_node("robot_m_node", log_level=rospy.INFO)
        self.comm_pub = rospy.Publisher("/comm", String, queue_size=10)
        self.tag_detections_sub = rospy.Subscriber(
            "tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10)
        self.target_sub = rospy.Subscriber("/target", String, self.target_callback, queue_size=10)
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
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

    def target_callback(self, msg):
        """
        When an SOI id is given the robot will navigate to the known position.

        Args:
            msg (String): SOI id as a String
        """

        tag_id = msg.data
        goal_pose = self.tag_dict[tag_id]
        
        # send Nav 2D goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = goal_pose

        self.action_client.send_goal(goal)
        wait = self.action_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return selfclient.get_result()

def main():
    master = RobotMaster()
    rospy.spin()

if name == "__main__":
    main()


