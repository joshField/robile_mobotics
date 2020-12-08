
import rospy
import actionlib
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
import json
import yaml


class RobotMaster():
    def __init__(self):

        rospy.init_node("robot_m_node", log_level=rospy.INFO)
        self.comm_pub = rospy.Publisher("/comm", String, queue_size=10)
        self.tag_detections_sub = rospy.Subscriber(
            "tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10)
        self.target_sub = rospy.Subscriber("/target", String, self.target_callback, queue_size=10)
        self.vel_sub = rospy.Subscriber("cmd_vel", Twist, self.vel_callback, queue_size=10)
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        x = rospy.get_param("home_x")
        y = rospy.get_param("home_y")
        z = rospy.get_param("home_z")
        self.home = (x,y,z)
        self.guidance_mode = False
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
            msg (String): SOI id as a String (ex. "0,1,2")
        """
        sois = msg.data.split(",")
        
        for tag_id in sois:
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

    def vel_callback(self, msg):
        """
        Listen on cmd_vel topic and republishes to the comm topic for the slave robot.vel_callback if in guidance mode.

        Args:
            msg (Twist): Velocity command
        """

        if self.guidance_mode: 
            str_cmd = String()
            data = yaml.load(str(msg))
            str_cmd.data = json.dumps(data)
            self.comm_pub.publish(str_cmd)


def main():
    master = RobotMaster()
    rospy.spin()

if name == "__main__":
    main()


