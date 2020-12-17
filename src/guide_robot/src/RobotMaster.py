#! /usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import String, Header
from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Quaternion, Point, PoseStamped
import tf2_ros
import json
import yaml
import rospkg
from collections import deque


class RobotMaster():
    def __init__(self):
        rospy.init_node("robot_m_node", log_level=rospy.INFO)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(1.0)

        self.rospack = rospkg.RosPack()
        
        self.namespace = "robot_m"

        self.comm_pub = rospy.Publisher("/comm", String, queue_size=10)
        self.comm_sub = rospy.Subscriber("/comm", String, self.comm_callback, queue_size=10)
        self.tag_detections_sub = rospy.Subscriber(
            f"tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10)
        self.target_sub = rospy.Subscriber("/target", String, self.target_callback, queue_size=10)
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # x = rospy.get_param("home_x")
        # y = rospy.get_param("home_y")
        # z = rospy.get_param("home_z")
        # self.home = (x,y,z)
        self.guidance = False
        self.tag_dict = {}
        self.pose_history = deque(maxlen=30)
        self.backtrack_mode = False
        self.rate = rospy.Rate(2)

        path = self.rospack.get_path('guide_robot')
        with open(path + '/config/tag_positions.yaml') as f:
            self.tag_dict = yaml.load(f, Loader=yaml.FullLoader)
            #parse loaded tag locations 
            for tag_id, point in self.tag_dict.items():
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = point[2]

                self.tag_dict[tag_id] = p
                

    def tag_callback(self, msg):
        """
        Updates the pose for detected apriltags

        Args:
            msg (AprilTagDetectionArray): detected april tags in current view
        """
        for tag in msg.detections:
            tag_id = tag.id
            rospy.loginfo_throttle(5.0, "Found tag: %d" % tag_id)
            #convert tag pose to global
            try:
                trans = self.tfBuffer.lookup_transform('map', f'{rospy.get_namespace().split("/")[1]}/tag_{tag_id:03d}', rospy.Time(0))
                self.tag_dict[tag_id] = Point(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(e)
                continue


    def target_callback(self, msg):
        """
        When an SOI id is given the robot will navigate to the known position.

        Args:
            msg (String): SOI id as a String (ex. "0,1,2")
        """
        sois = msg.data.split(",")
        sois = [int(i) for i in sois]
        
        for tag_id in sois:
            rospy.loginfo("Moving to tag %d" % tag_id)
            goal_pose = self.tag_dict[tag_id]
            
            # send Nav 2D goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = goal_pose
            goal.target_pose.pose.orientation.w = 1.0

            self.action_client.send_goal(goal)
            wait = self.action_client.wait_for_result()
            # if not wait:
            #     rospy.logerr("Action server not available!")
            #     rospy.signal_shutdown("Action server not available!")
            # else:
            #     return self.action_client.get_result()

    def comm_callback(self, msg):
        """
        """
        comm = json.loads(msg.data)
        return 

        #todo: need logic for lost and then found to clear pose history and set backtrack mode back to False, lost is never set to False
        if comm['from_slave']:
            if comm['lost']:
                #go back to slave, send most recent Nav 2D goal from pose history
                self.backtrack_mode = True
                if len(self.pose_history != 0):
                    goal_pose = self.pose_history.pop()
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position = goal_pose
                    # goal.target_pose.pose.orientation.w = 1.0

                    self.action_client.send_goal(goal)
                    wait = self.action_client.wait_for_result()
                else:
                    # out of points to backtrack
                    self.backtrack_mode = False
            else:
                # not lost
                self.backtrack_mode = False

    def run(self):
        """
        Save current position into pose history queue. To be used for backtrack in guidance mode.
        """
        
        #Save the current pose if not in backtrack mode and in guidance mode
        if not self.backtrack_mode and self.guidance:
            try:
                trans = self.tfBuffer.lookup_transform('map', f'{rospy.get_namespace().split("/")[1]}/base_footprint', rospy.Time(0))
                master_pose = PoseStamped()
                master_pose.header = Header()
                master_pose.header.stamp = rospy.Time.now()
                master_pose.header.frame_id = "world"
                master_pose.position = Point(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
                master_pose.orientation = trans.transform.rotation
                self.pose_history.append(master_pose)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(e)
                continue

        self.rate.sleep()

def main():
    master = RobotMaster()
    while not rospy.is_shutdown():
        master.run()

if __name__ == "__main__":
    main()


