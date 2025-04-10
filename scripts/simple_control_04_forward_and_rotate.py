#!/usr/bin/env python3
# coding: utf-8

import time
import rospy
import nav_msgs.msg as nav_msgs
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion, Point, Vector3, TransformStamped, TwistWithCovarianceStamped,Twist
from gazebo_msgs.msg import ModelStates
#
from jethexa_controller_interfaces import msg as jetmsg
from jethexa_controller_interfaces.msg import Traveling
from jethexa_controller_interfaces.srv import SetPose1, SetPose1Request, SetPose1Response
from jethexa_controller_interfaces.srv import SetPose2, SetPose2Request, SetPose2Response
from jethexa_controller_interfaces.srv import PoseTransform, PoseTransformRequest, PoseTransformResponse
#
from jethexa_controller import jethexa, build_in_pose, config
from jethexa_controller.z_voltage_publisher import VoltagePublisher
from jethexa_controller.z_joint_states_publisher import JointStatesPublisher
import geometry_msgs.msg

class jetHexaBasicMotion:
    def __init__(self, output_file):
        #inital control node
        rospy.init_node("jethexa_basic_motion", anonymous=True, log_level=rospy.INFO)

        #build publisher
        self.traveling_pub = rospy.Publisher('/jethexa_controller/traveling', Traveling, queue_size=10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize recording variables
        self.recording = False
        self.output_file = output_file
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        rospy.loginfo("Subscribed to /gazebo/model_states")

    def forward(self,step):
        msg=Traveling()
        msg.gait= 1
        msg.stride=40.0
        msg.height=15.0
        msg.direction=0
        msg.rotation=0
        msg.time=0.8
        msg.steps=step
        msg.relative_height=False
        msg.interrupt=False
        self.traveling_pub.publish(msg)
        print("sending")

    def horizontal_left(self,step):
        msg= jetmsg.Traveling()
        msg.gait=2
        msg.stride=40.0
        msg.height=15.0
        msg.direction=90
        msg.rotation=0.0
        msg.time=0.8
        msg.steps=step
        msg.relative_height=False
        msg.interrupt=False
        self.traveling_pub.publish(msg)   

    def horizontal_right(self,step):
        msg= jetmsg.Traveling()
        msg.gait=2
        msg.stride=40.0
        msg.height=15.0
        msg.direction=-90
        msg.rotation=0.0
        msg.time=0.8
        msg.steps=step
        msg.relative_height=False
        msg.interrupt=False
        self.traveling_pub.publish(msg)
    
    def reverse(self,step):
        msg= jetmsg.Traveling()
        msg.gait=2
        msg.stride=40.0
        msg.height=15.0
        msg.direction=180
        msg.rotation=0.0
        msg.time=0.8
        msg.steps=step
        msg.relative_height=False
        msg.interrupt=False
        self.traveling_pub.publish(msg)

    def rotation(self,rotation,step):
        msg= jetmsg.Traveling()
        msg.gait=1
        msg.stride=40.0
        msg.height=15.0
        msg.direction=0
        msg.rotation=rotation
        msg.time=0.7
        msg.steps=step
        msg.relative_height=False
        msg.interrupt=False
        #rospy.sleep(1)
        self.traveling_pub.publish(msg)
    
    def stop(self):
        msg= jetmsg.Traveling()
        msg.gait=0
        msg.stride=0.0
        msg.height=0.0
        msg.direction=0
        msg.rotation=0.0
        msg.time=0.8
        msg.steps=1
        msg.relative_height=False
        msg.interrupt=False
        self.traveling_pub.publish(msg)


    def cmd_vel_Publisher(self, linear_x, linear_y, angular_z):
        msg= geometry_msgs.msg.Twist()
        msg.linear.x=linear_x
        msg.linear.y=linear_y
        msg.angular.z=angular_z
        rospy.sleep(1)
        self.vel_pub.publish(msg)

    def start_recording(self):
        self.recording = True
        rospy.loginfo("Recording started")

    def stop_recording(self):
        self.recording = False
        rospy.loginfo("Recording stopped")

    def model_states_callback(self, data):
        if not self.recording:
            return
        try:
            with open(self.output_file, 'a') as file:
                for i, model_name in enumerate(data.name):  # Iterate over model names
                    if model_name == "jethexa":  # Replace "robot" with your robot's model name
                        position = data.pose[i].position
                        orientation = data.pose[i].orientation
                        file.write(f"Position: {position}, \nOrientation: {orientation}\n")
                        rospy.loginfo(f"Robot Pose - Position: {position},\n Orientation: {orientation}")
                        break  # Exit loop after finding the robot
        except Exception as e:
            rospy.logerr(f"Failed to write data to file: {e}")

    def execute_motion_sequence(self):
        self.start_recording()  # Start recording
        rospy.sleep(1)
        self.forward(0)
        rospy.sleep(10)  # Simulate some process
        self.rotation(1, 0)
        rospy.sleep(21)
        self.forward(0)
        rospy.sleep(30)  # Simulate some process
        self.rotation(1, 0)
        rospy.sleep(21)
        self.forward(0)
        rospy.sleep(10)  # Simulate some process
        self.rotation(1, 0)
        rospy.sleep(21)
        self.forward(0)
        rospy.sleep(30)  # Simulate some process
        self.rotation(1, 0)
        rospy.sleep(21)
        self.stop()
        rospy.sleep(1)
        self.stop_recording()  # Stop recording
    def move_in_triangle(self):
        rospy.loginfo("Starting triangular movement")
        # Move forward
        self.cmd_vel_Publisher(linear_x=0.05, linear_y=0.0, angular_z=0.0)
        rospy.sleep(10)  # Adjust sleep time based on side length

        # Rotate 120 degrees (2π/3 radians) for the triangle
        self.cmd_vel_Publisher(linear_x=-0.025, linear_y=0.05, angular_z=0)
        rospy.sleep(20)  # Adjust sleep time based on angular speed
        self.cmd_vel_Publisher(linear_x=0, linear_y=-0.05, angular_z=0)
        rospy.sleep(20)  # Adjust sleep time based on angular speed
        self.cmd_vel_Publisher(linear_x=0.0, linear_y=0.0, angular_z=0)
        self.stop()
        rospy.loginfo("Finished triangular movement")

if __name__ == "__main__":
    output_file = "/home/hiwonder/jethexa_vm/model_states_data.txt"
    robot = jetHexaBasicMotion(output_file)
    rospy.sleep(1)
    #robot.execute_motion_sequence()
    robot.move_in_triangle()















