#! /usr/bin/env python
import copy
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from std_msgs.msg import Header
from locobot_simulation.msg import LogicalImage
import math
import tf.transformations as tf

# Initialize moveit_commander & the ROS node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

# Initialize the robot & arm group using the robot's description and namespaces.
robot = moveit_commander.RobotCommander("locobot/robot_description", "/locobot")
group = moveit_commander.MoveGroupCommander("interbotix_arm", "locobot/robot_description", "/locobot", 10.0)

# Interface for adding/removing objects in the world.
scene = moveit_commander.PlanningSceneInterface("locobot")  

# For when a new LogicalImage message is received.
def callback(data):
    # Loop through each detected model from the camera.
    for model in data.models:
        if model.type in ["red_cube"]: 
            print(f"{model.type}") # In this case "red cube".
            
            # Calculate the down orientation for the gripper using Euler angles.
            down_orient = tf.quaternion_from_euler(0, math.pi / 2, 0)
            
            # Create a PoseStamped object for the target pose.
            pose = PoseStamped(Header(frame_id="locobot/camera_link"), Pose(model.pose.position, Quaternion(*down_orient)))
            
            # Call the move_robot function to move to the target pose.
            move_robot(pose, 0.1, 0.005, True)
            
            # Move the robot back to the sleep pose.
            sleep_pose(0.1)

# To move the robot to a specified pose.
def move_robot(pose, speed, height, lower=False):
    print("moving") 
    
    # Set the velocity scaling factor for the robot's movement.
    velocity(group, speed)
    
    # Make copy of the pose to avoid modifying the original pose.
    pose = copy.deepcopy(pose)

    # If not lowering, use the current z position robot arm.
    if not lower: 
        pose.pose.position.z = group.get_current_pose().pose.position.z
    else:
        pose.pose.position.z += height
        
    print(f"Target: {pose}")  

    # Set the target pose for the robot and execute the movement.
    group.set_pose_target(pose)
    group.plan() 
    group.go(wait=True) 
    print("After:", group.get_current_pose().pose)  

# To set velocity and acceleration limits.
def velocity(item, speed):
    item.set_max_velocity_scaling_factor(speed)  # Set maximum velocity scaler.
    item.set_max_acceleration_scaling_factor(speed)  # Set max acceleration scaler.

# Listener function to subscribe to the logical camera topic.
def listener():
    sleep_pose()  # Move the robot to sleep pose.
    
    # Subscribe to the logical camera image topic.
    rospy.Subscriber("/gazebo/locobot/camera/logical_camera_image", LogicalImage, callback)
    rospy.spin()  # Keep the node running and processing incoming messages

# Function to move the robot to the "Home" named pose (sleeping pose).
def sleep_pose(speed=1):
    print("sleeping pose")
    
    # Set the velocity and move the robot to the predefined "Home" pose.
    velocity(group, speed)
    group.set_named_target("Home")  
    group.plan()  
    group.go(wait=True)  # Execute the plan and wait for completion.

# Main entry point for the script.
if __name__ == '__main__':
    try:
        listener()  
    except rospy.ROSInterruptException:
        pass  
