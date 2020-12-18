#! /usr/bin/env python

import rospy
import actionlib
from actionlib import GoalStatus
from std_msgs.msg import String, Header
from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Quaternion, Point, Pose
import tf2_ros
import json
import yaml
import rospkg
from collections import deque
import numpy as np


class RobotMaster():
    def __init__(self):
        rospy.init_node("robot_m_node", log_level=rospy.INFO)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        

        self.rospack = rospkg.RosPack()
        
        self.namespace = "robot_m"

        self.comm_pub = rospy.Publisher("/comm", String, queue_size=10)
        self.comm_sub = rospy.Subscriber("/comm", String, self.comm_callback, queue_size=10)
        self.tag_detections_sub = rospy.Subscriber(
            f"tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10)
        self.target_sub = rospy.Subscriber("/target", String, self.target_callback, queue_size=10)
        self.target_pub = rospy.Publisher("/target", String, queue_size=10)
        self.follow_sub = rospy.Subscriber("/follow", String, self.follow_callback, queue_size=10)
        
        self.guidance = False
        self.tag_dict = {}
        self.backtrack_mode = False
        self.hz = 20
        self.rate = rospy.Rate(self.hz)
        self.last_lost_status = False
        self.curr_goal_id = None
        self.curr_slave_id = None
        self.sent_goal = False
    
        self.init_x = rospy.get_param("home_x", 2.0)
        self.init_y = rospy.get_param("home_y", 9.0)
        self.init_z = rospy.get_param("home_z", 0.0)
        self.maxlen = 60
        self.tick_count = 0
        self.return_home = False

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
        self.pose_history = None
        rospy.sleep(1.0)
        self.reset_history()

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
        rospy.loginfo(f"{sois}")
        sois = [int(i) for i in sois]
        
        #TODO: make sure doesn't return home, set a flag
        for tag_id in sois:
            self.move_target(tag_id)

    def reset_history(self):
        """
        Reset past history of positions to current position.
        """
        pose = self.get_position()
        poses = [pose]*self.maxlen
        self.pose_history = deque(poses, maxlen=self.maxlen)

    def move_target(self, tag_id, wait=True):
        """
        Move to specific target apriltag id.

        Args:
            tag_id (int): id of apriltag
            wait (bool): flag that tells function to wait for action result
        """
        rospy.loginfo("Moving to tag %d" % tag_id)
        goal_point = self.tag_dict[tag_id]
        
        # send Nav 2D goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = goal_point
        goal.target_pose.pose.orientation.w = 1.0

        self.action_client.send_goal(goal)
        rospy.loginfo("Sent goal")
        if wait:
            action_wait = self.action_client.wait_for_result()
            if not action_wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                status = self.action_client.get_state()
                if status == GoalStatus.SUCCEEDED and not self.backtrack_mode:
                    rospy.loginfo(f"Reached tag{tag_id:03d}")

                    if self.return_home:
                        # return to home position
                        goal = MoveBaseGoal()
                        home_point = Point(self.init_x, self.init_y, self.init_z)
                        goal.target_pose.header.frame_id = "map"
                        goal.target_pose.header.stamp = rospy.Time.now()
                        goal.target_pose.pose.position = home_point
                        goal.target_pose.pose.orientation.w = 1.0

                        self.action_client.send_goal(goal)
                        self.action_client.wait_for_result()
                    self.sent_goal = False
                    self.curr_slave_id = None
                    self.curr_goal_id = None

    def comm_callback(self, msg):
        """
        Comm callback to handle all communication coming from slave robot. Contains logic to backtrack if slave if lost.

        Args:
            msg (String): Comm msg to be loaded into a dict
        """
        comm = json.loads(msg.data)

        if comm['from_slave']:
            curr_lost_status = False  #comm['lost']  # TODO THIS DISABLES BACKTRACKING
            # rospy.loginfo_throttle_identical(5.0, f"Comm from slave: {comm}")

            # Slave was lost, but now now found tag again
            if self.last_lost_status and not curr_lost_status:
                rospy.loginfo("Slave is not lost anymore")
                self.reset_history()
                self.backtrack_mode = False
                self.sent_goal = False

            # Slave currently lost
            elif curr_lost_status:

                #go back to slave, send most recent Nav 2D goal from pose history
                self.backtrack_mode = True
                if len(self.pose_history) != 0:
                    goal_pose = self.pose_history.pop()

                    rospy.loginfo_throttle(5.0, "Backtracking to(%f, %f)" %(goal_pose.position.x, goal_pose.position.y))
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose = goal_pose
                    goal.target_pose.pose.orientation.w = 1.0


                    self.action_client.send_goal(goal)
                    # self.action_client.
                    wait = self.action_client.wait_for_result()
                else:
                    rospy.logerr_throttle(5.0, "Ran out of points for master to backtrack...")
            
            # save last slave lost status
            self.last_lost_status = curr_lost_status

    def get_position(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', f'{rospy.get_namespace().split("/")[1]}/base_footprint', rospy.Time(0))
            master_pose = Pose()
            master_pose.position = Point(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
            master_pose.orientation = trans.transform.rotation
            return master_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)

    def follow_callback(self, msg):
        """
        Callback for telling master what slaves should follow it and where.

        Args:
            msg (String): Assigns multiple slave_ids to goal_ids (ex. "0,1 1,2")
        """
        pairs = msg.data.split()
        
        self.return_home = True
        for p in pairs:
            slave_id, goal_id = p.split(",")
            self.curr_goal_id = int(goal_id)
            self.curr_slave_id = int(slave_id)
            while self.curr_slave_id is not None and self.curr_goal_id is not None:
                continue
        self.return_home = False
        self.curr_goal_id = None
        self.curr_slave_id = None


    def run(self):
        """
        Save current position into pose history queue. To be used for backtrack in guidance mode.
        """
        if self.pose_history is None:
            return

        # Tell slave to go into guided if slave id and goal id are set
        if self.curr_slave_id is not None and self.curr_goal_id is not None and not self.sent_goal:
            self.guidance = True
            comm = {
                'guidance': True,
                'from_slave': False,
                'slave_id': self.curr_slave_id,
                'target_id': self.curr_goal_id,
                'lost':  False
            }
            str_cmd = String()
            str_cmd.data = json.dumps(comm)
            self.comm_pub.publish(str_cmd)
            
            self.target_pub.publish(f"{self.curr_goal_id}")
            rospy.loginfo_throttle(5.0, "Sent move to tag: %d" % self.curr_goal_id)
            self.sent_goal = True

        #Save the current pose if not in backtrack mode and in guidance mode
        if not self.backtrack_mode and self.guidance and (self.tick_count % self.hz == 0):
            self.pose_history.append(self.get_position())
        
        self.tick_count += 1
        self.rate.sleep()

def main():
    master = RobotMaster()
    while not rospy.is_shutdown():
        master.run()

if __name__ == "__main__":
    main()


