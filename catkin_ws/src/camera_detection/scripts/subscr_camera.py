#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from locobot_simulation.msg import LogicalImage

class PublisherNode:
    def __init__(self):
        # Initialize node.
        rospy.init_node('publisher_node', anonymous=True)

        # Publishers.
        self.pan_publisher = rospy.Publisher('/locobot/pan_controller/command', Float64, queue_size=10)
        self.tilt_publisher = rospy.Publisher('/locobot/tilt_controller/command', Float64, queue_size=10)

        # Publishing rate.
        self.rate = rospy.Rate(100)

        # Commands for pan & tilt.
        self.pan_cmd = 0
        self.tilt_cmd = 0.5
        self.delta = 0.01

        rospy.Subscriber("/gazebo/locobot/camera/logical_camera_image", LogicalImage, self.camera_callback)

    def camera_callback(self, data):
        # Process the camera data.
        rospy.loginfo("Logical camera data found.")
        if data.models:
            for model in data.models:
                rospy.loginfo(f"Detected object: {model.type}, at position: {model.pose.position}")
        else:
            rospy.loginfo("No objects detected.")

    def publish_topics(self):
        while not rospy.is_shutdown():
            # Publish pan & tilt.
            rospy.loginfo(f'Publishing pan command: {self.pan_cmd}')
            self.pan_publisher.publish(self.pan_cmd)
            self.tilt_publisher.publish(self.tilt_cmd)

            # Update pan command.
            if self.pan_cmd >= 1.8 or self.pan_cmd <= -1.8:
                self.delta *= -1
            self.pan_cmd += self.delta

            # Sleep to maintain publishing rate.
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node = PublisherNode()
        publisher_node.publish_topics()
    except rospy.ROSInterruptException:
        pass