#!/usr/bin/env python

"""
OPEN TWO TERMINALS WITH:
roslaunch robotics_project gazebo_project.launch
roslaunch robotics_project launch_project.launch
"""

import py_trees as pt, py_trees_ros as ptr, rospy
from numpy import linalg as LA
from reactive_sequence import RSequence
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead
from std_srvs.srv import Empty, SetBool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelState

class tuckarm(pt.behaviour.Behaviour):

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):

        # Initialise again if tree has to be repeated
        global flag_repeat_tree_1
        if flag_repeat_tree_1:
            self.sent_goal = False
            self.finished = False
            flag_repeat_tree_1 = False

        # already tucked the arm
        if self.finished:
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class movehead(pt.behaviour.Behaviour):

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):

        # Initialise again if tree has to be repeated
        global flag_repeat_tree_2
        if flag_repeat_tree_2:
            self.tried = False
            self.done = False
            flag_repeat_tree_2 = False

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class localize(pt.behaviour.Behaviour):

    def __init__(self):

        rospy.loginfo("Initialising localize behaviour.")

        # action space
        self.cmd_vel_top = "/key_vel"
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.amcl_pose_top = "/amcl_pose"
        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = 0
        self.move_msg.angular.z = -1

        # services
        global_loc_nm = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.global_loc_srv = rospy.ServiceProxy(global_loc_nm, Empty)
        rospy.wait_for_service(global_loc_nm, timeout=20)

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(localize, self).__init__("Localize!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.amcl_pose = rospy.wait_for_message(self.amcl_pose_top, PoseWithCovarianceStamped, 5)
            self.K_factor = LA.norm(self.amcl_pose.pose.covariance)
            self.localization_req = self.global_loc_srv()
            rate = rospy.Rate(10)
            while not rospy.is_shutdown() and self.K_factor > 0.02:
                self.amcl_pose = rospy.wait_for_message(self.amcl_pose_top, PoseWithCovarianceStamped, 5)
                self.K_factor = LA.norm(self.amcl_pose.pose.covariance)
                self.cmd_vel_pub.publish(self.move_msg)
                rate.sleep()
            self.tried = True

			# tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.localization_req:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.localization_req:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class pickup(pt.behaviour.Behaviour):

    def __init__(self):

        rospy.loginfo("Initialising pick up behaviour.")

        # server
        pick_srv = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.pick_srv = rospy.ServiceProxy(pick_srv, SetBool)
        rospy.wait_for_service(pick_srv, timeout=30)

        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(pickup, self).__init__("Pick up!")

    def update(self):

        # Initialise again if tree has to be repeated
        global flag_repeat_tree_3
        if flag_repeat_tree_3:
            self.tried = False
            self.done = False
            flag_repeat_tree_3 = False
       
        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv("down")
            self.pick_req = self.pick_srv()
            self.move_head_req = self.move_head_srv("up")
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.pick_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class place(pt.behaviour.Behaviour):

    def __init__(self):

        rospy.loginfo("Initialising place behaviour.")

        # server
        place_srv = rospy.get_param(rospy.get_name() + '/place_srv')
        self.place_srv = rospy.ServiceProxy(place_srv, SetBool)
        rospy.wait_for_service(place_srv, timeout=30)

        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(place, self).__init__("Place!")

    def update(self):

        # Initialise again if tree has to be repeated
        global flag_repeat_tree_4
        if flag_repeat_tree_4:
            self.tried = False
            self.done = False
            flag_repeat_tree_4 = False

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv("down")
            self.place_req = self.place_srv()
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.place_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_req.success:
            return pt.common.Status.SUCCESS

        # if still trying
        else:
            return pt.common.Status.RUNNING

class navigate_pickup(pt.behaviour.Behaviour):

    def __init__(self):

        rospy.loginfo("Initialising navigate behaviour to pick up pose.")

        # get parameters and set up Action Client
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        self.pick_pos = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        destination_pick = rospy.wait_for_message(self.pick_pos, PoseStamped, 5)

        # send position to action service
        self.goal = MoveBaseGoal()
        self.goal.target_pose = destination_pick

        # KIDNAPPING
        self.cmd_vel_top = "/key_vel"
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.amcl_pose_top = "/amcl_pose"
        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = 0
        self.move_msg.angular.z = -1
        # services
        global_loc_nm = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        clear_srv = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        self.clear_srv = rospy.ServiceProxy(clear_srv, Empty)
        self.global_loc_srv = rospy.ServiceProxy(global_loc_nm, Empty)
        rospy.wait_for_service(global_loc_nm, timeout=20)
        rospy.wait_for_service(clear_srv, timeout=20)

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(navigate_pickup, self).__init__("Navigate to pick up pose!")

    def update(self):

        # Initialise again if tree has to be repeated
        global flag_repeat_tree_6
        if flag_repeat_tree_6:
            self.sent_goal = False
            self.finished = False
            flag_repeat_tree_6 = False

        if not self.finished:
            self.amcl_pose = rospy.wait_for_message(self.amcl_pose_top, PoseWithCovarianceStamped, 5)
            self.K_factor = LA.norm(self.amcl_pose.pose.covariance)

        # KIDNAPPING
        rate = rospy.Rate(10)
        if self.K_factor > 0.04:
            self.localization_req = self.global_loc_srv()
            while not rospy.is_shutdown() and self.K_factor > 0.02:
                self.amcl_pose = rospy.wait_for_message(self.amcl_pose_top, PoseWithCovarianceStamped, 5)
                self.K_factor = LA.norm(self.amcl_pose.pose.covariance)
                self.cmd_vel_pub.publish(self.move_msg)
                rate.sleep()
            self.clear_req = self.clear_srv()
            self.sent_goal = False

        if self.finished:
            return pt.common.Status.SUCCESS
        
        elif not self.sent_goal:
            # send the goal
            self.move_base_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.move_base_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_base_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class navigate_place(pt.behaviour.Behaviour):

    def __init__(self):

        rospy.loginfo("Initialising navigate behaviour to place pose.")

        # get parameters and set up Action Client
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        self.place_pos = rospy.get_param(rospy.get_name() + '/place_pose_topic')
        destination_place = rospy.wait_for_message(self.place_pos, PoseStamped, 5)

        # send position to action service
        self.goal = MoveBaseGoal()
        self.goal.target_pose = destination_place

        # KIDNAPPING
        self.cmd_vel_top = "/key_vel"
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.amcl_pose_top = "/amcl_pose"
        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = 0
        self.move_msg.angular.z = -1
        # services
        global_loc_nm = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        clear_srv = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        self.clear_srv = rospy.ServiceProxy(clear_srv, Empty)
        self.global_loc_srv = rospy.ServiceProxy(global_loc_nm, Empty)
        rospy.wait_for_service(global_loc_nm, timeout=20)
        rospy.wait_for_service(clear_srv, timeout=20)

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(navigate_place, self).__init__("Navigate to place pose!")

    def update(self):

        # Initialise again if tree has to be repeated
        global flag_repeat_tree_7
        if flag_repeat_tree_7:
            self.sent_goal = False
            self.finished = False
            flag_repeat_tree_7 = False

        if not self.finished:
            self.amcl_pose = rospy.wait_for_message(self.amcl_pose_top, PoseWithCovarianceStamped, 5)
            self.K_factor = LA.norm(self.amcl_pose.pose.covariance)

        # KIDNAPPING
        rate = rospy.Rate(10)
        if self.K_factor > 0.04:
            self.localization_req = self.global_loc_srv()
            while not rospy.is_shutdown() and self.K_factor > 0.02:
                self.amcl_pose = rospy.wait_for_message(self.amcl_pose_top, PoseWithCovarianceStamped, 5)
                self.K_factor = LA.norm(self.amcl_pose.pose.covariance)
                self.cmd_vel_pub.publish(self.move_msg)
                rate.sleep()
            self.clear_req = self.clear_srv()
            self.sent_goal = False

        if self.finished:
            return pt.common.Status.SUCCESS
        
        elif not self.sent_goal:
            # send the goal
            self.move_base_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.move_base_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_base_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class cubecheck(pt.behaviour.Behaviour):

    def __init__(self):

        rospy.loginfo("Checking cube is there.")
   
        self.done = False
        self.pause_cnt = 0

        super(cubecheck, self).__init__("Cube is checked")
       
    def update(self):

        # Initialise again if tree has to be repeated
        global flag_repeat_tree_1
        global flag_repeat_tree_2
        global flag_repeat_tree_3
        global flag_repeat_tree_4
        global flag_repeat_tree_5
        global flag_repeat_tree_6
        global flag_repeat_tree_7

        if flag_repeat_tree_5:
            self.aruco_pose_subs = None
            self.done = False
            flag_repeat_tree_5 = False

        self.aruco_pose_subs = rospy.Subscriber("/marker_pose_topic", PoseStamped, self.aruco_pose_cb)

        if self.done:
            return pt.common.Status.SUCCESS
        else:
            message = ModelState()
            message.model_name = "aruco_cube"
            message.pose.position.x = -1.130530
            message.pose.position.y = -6.653650
            message.pose.position.z = 0.86250
            message.pose.orientation.x = 0
            message.pose.orientation.y = 0
            message.pose.orientation.z = 0
            message.pose.orientation.w = 1
            message.twist.linear.x = 0
            message.twist.linear.y = 0
            message.twist.linear.z = 0
            message.twist.angular.x = 0
            message.twist.angular.y = 0
            message.twist.angular.z = 0
            message.reference_frame = "map"

            self.respawn_top = "/gazebo/set_model_state"
            self.respawn_pub = rospy.Publisher(self.respawn_top, ModelState, queue_size=10)
            self.respawn_pub.publish(message)

            # Use this pause, otherwise the resetting will never work
            self.pause_cnt = self.pause_cnt + 1
            if self.pause_cnt == 1000:
                flag_repeat_tree_1 = True
                flag_repeat_tree_2 = True
                flag_repeat_tree_3 = True
                flag_repeat_tree_4 = True
                flag_repeat_tree_5 = True
                flag_repeat_tree_6 = True
                flag_repeat_tree_7 = True
                self.pause_cnt = 0

            return pt.common.Status.FAILURE

    def aruco_pose_cb(self,aruco_pose_msg):
        self.done = True

class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):

        rospy.loginfo("Initialising behaviour tree")

        global flag_repeat_tree_1
        global flag_repeat_tree_2
        global flag_repeat_tree_3
        global flag_repeat_tree_4
        global flag_repeat_tree_5
        global flag_repeat_tree_6
        global flag_repeat_tree_7

        flag_repeat_tree_1 = False
        flag_repeat_tree_2 = False
        flag_repeat_tree_3 = False
        flag_repeat_tree_4 = False
        flag_repeat_tree_5 = False
        flag_repeat_tree_6 = False
        flag_repeat_tree_7 = False

        b0 = movehead("up")
        b1 = tuckarm()
        b2 = localize()
        b3 = navigate_pickup()
        b4 = pickup()
        b5 = navigate_place()
        b6 = place()
        b7 = cubecheck()

        tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, b5, b6, b7])

        super(BehaviourTree, self).__init__(tree)

        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown(): self.tick_tock(1)

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()