#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros 
import tf2_geometry_msgs
import math

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1


def move(path):

  global control_client, robot_frame_id, pub

  # Call service client (collision avoidance) with path
  try:
    service_result = control_client(path)
    setpoint = service_result.setpoint
    service_path = service_result.new_path
  except:
    print('false')

  while service_path.poses: # This loop keeps going while the service_path is not empty
    try:
      service_result = control_client(service_path)
      setpoint = service_result.setpoint
      service_path = service_result.new_path
    except:
      print('error')

    # Transform Setpoint from service client
    trans = tf_buffer.lookup_transform(robot_frame_id, setpoint.header.frame_id, setpoint.header.stamp)
    transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, trans)
    
    # Create Twist message from the transformed Setpoint
    msg = Twist()

    if math.atan2(transformed_setpoint.point.y, transformed_setpoint.point.x) < max_angular_velocity:
      msg.angular.z = math.atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
    else:
      msg.angular.z = max_angular_velocity
    
    if 0.5 * math.sqrt(transformed_setpoint.point.x ** 2 + transformed_setpoint.point.y ** 2) < max_linear_velocity:
      msg. linear.x = msg.linear.x = 0.6 * math.sqrt(transformed_setpoint.point.x ** 2 + transformed_setpoint.point.y ** 2)
    else:
      msg.linear.x = max_linear_velocity

    # Publish Twist
    pub.publish(msg)

  msg.linear.x = 0
  msg.angular.z = 0
  pub.publish(msg)

  # Get new path from action server
  get_path()


def get_path():
    global goal_client

    goal_client.wait_for_server()

    goal_client.send_goal(0)

    # Wait for the server to finish performing the action.
    goal_client.wait_for_result()

    # Prints out the result of executing the action
    result = goal_client.get_result()

    # Call move with path from action server
    move(result.path)


if __name__ == "__main__":
    # Initiate a node called controller
    rospy.init_node('controller')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Initialise publisher 
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 20)

    # Initialise the action server
    goal_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)

    # Initialise ServiceProxy for move()
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)
    
    # Call get_path function 
    get_path()
    
    # The final addition, rospy.spin() simply keeps your node from exiting until the node has been shutted down.
    rospy.spin()