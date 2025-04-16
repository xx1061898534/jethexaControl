#!/usr/bin/env python3
# coding: utf-8

import math
import time
import rospy
import nav_msgs.msg as nav_msgs
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion, Point, Vector3, TransformStamped, TwistWithCovarianceStamped,Twist
from gazebo_msgs.msg import ModelStates
import csv  # Add this import for CSV handling
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
from trajectory import parabolicBlends

class jetHexaBasicMotion:
    def __init__(self, output_file):
        #inital control node
        rospy.init_node("jethexa_basic_motion", anonymous=False, log_level=rospy.INFO)

        #build publisher
        self.traveling_pub = rospy.Publisher('/jethexa_controller/traveling', Traveling, queue_size=10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize recording variables
        self.recording = False
        self.output_file = output_file
        self.start_time = rospy.get_time()  # Record the start time
        self.csv_initialized = False  # Track if the CSV header is written
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        rospy.loginfo("Subscribed to /gazebo/model_states")
        self.rate = rospy.Rate(10)  # Set rate to 25 Hz

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
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.vel_pub.publish(msg)
       # self.rate.sleep()  # Ensures publishing at 25 Hz

    def start_recording(self):
        self.recording = True
        rospy.loginfo("Recording started")

    def stop_recording(self):
        self.recording = False
        rospy.loginfo("Recording stopped")

    def model_states_callback(self, data):
        #rate = rospy.Rate(10)  # Set rate to 10 Hz
        if not self.recording:
            return
        try:
            with open(self.output_file, 'a', newline='') as file:
                writer = csv.writer(file)
                if not self.csv_initialized:
                    # Write the CSV header
                    writer.writerow(['Elapsed_Time', 'Position_X', 'Position_Y', 'Position_Z', 
                                     'Orientation_X', 'Orientation_Y', 'Orientation_Z', 'Orientation_W'])
                    self.csv_initialized = True

                for i, model_name in enumerate(data.name):  # Iterate over model names
                    if model_name == "jethexa":  # Replace "jethexa" with your robot's model name
                        position = data.pose[i].position
                        orientation = data.pose[i].orientation
                        elapsed_time = rospy.get_time() - self.start_time  # Calculate elapsed time
                        writer.writerow([
                            elapsed_time,  # Elapsed time since the script started
                            position.x, position.y, position.z,  # Position
                            orientation.x, orientation.y, orientation.z, orientation.w  # Orientation
                        ])
                       # rospy.loginfo(f"Robot Pose - Elapsed Time: {elapsed_time}, Position: {position}, Orientation: {orientation}")
                        break  # Exit loop after finding the robot
        except Exception as e:
            rospy.logerr(f"Failed to write data to file: {e}")
        #rospy.sleep(0.1)

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
        for i in range(4):
          #self.stop()
          #rospy.sleep(1)
          self.cmd_vel_Publisher(linear_x=0.05, linear_y=0.0, angular_z=0.0)
          rospy.sleep(20)  # Adjust sleep time based on side length
          #self.stop()
          #rospy.sleep(1)
          self.cmd_vel_Publisher(linear_x=0.0, linear_y=0.00, angular_z=0.2)
          rospy.sleep(14)
          
        # self.cmd_vel_Publisher(linear_x=-0.025, linear_y=0.05, angular_z=0)
        # rospy.sleep(40)  # Adjust sleep time based on angular speed
        # self.cmd_vel_Publisher(linear_x=0, linear_y=-0.05, angular_z=0)
        # rospy.sleep(40)  # Adjust sleep time based on angular speed
        # self.cmd_vel_Publisher(linear_x=0.0, linear_y=0.0, angular_z=0)
        self.stop()
        rospy.loginfo("Finished triangular movement")

if __name__ == "__main__":
    output_file = "/home/hiwonder/jethexa_vm/src/jethexa_planning/scripts/model_states_data_para.csv"
    trajectory_file = "/home/hiwonder/jethexa_vm/src/jethexa_planning/scripts/trajectory_data.csv"  # New CSV file for x1, y1
    robot = jetHexaBasicMotion(output_file)
    #x1, vx, t1 = parabolicBlends([0, 0, 0.5, 0,  0], [10,20, 20, 20], 0.01, 0.1)
    #y1, vy, t2 = parabolicBlends([0, 0, 0,  1, 0], [10, 20, 20, 20], 0.01, 0.1)
    x1,v1, t1 = parabolicBlends([0,300*math.cos(math.radians(234)), 100*math.cos(math.radians(198)),300*math.cos(math.radians(162)), 100*math.cos(math.radians(126)), 300*math.cos(math.radians(90)), 100*math.cos(math.radians(54)), 300*math.cos(math.radians(18)), 100*math.cos(math.radians(342)),300*math.cos(math.radians(306)),0 ], [10,10,10,10,10,10,10,10,10,10], 3, 0.04)
    y1,vy, t2 = parabolicBlends([-100,300*math.sin(math.radians(234)), 100*math.sin(math.radians(198)),300*math.sin(math.radians(162)), 100*math.sin(math.radians(126)), 300*math.sin(math.radians(90)), 100*math.sin(math.radians(54)), 300*math.sin(math.radians(18)), 100*math.sin(math.radians(342)),300*math.sin(math.radians(306)),-100], [10,10,10,10,10,10,10,10,10,10], 3, 0.04)  
   

    # Record x1 and y1 to a new CSV file
    try:
        with open(trajectory_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['X1', 'Y1'])  # Write header
            for x, y in zip(x1, y1):
                writer.writerow([x, y])  # Write x1 and y1 values
        rospy.loginfo(f"Trajectory data saved to {trajectory_file}")
    except Exception as e:
        rospy.logerr(f"Failed to write trajectory data to file: {e}")

    rospy.sleep(5)
    robot.start_recording()  # Ensure recording starts
    rospy.sleep(2)
    # Publish vx and vy to cmd_vel
    for vx_val, vy_val in zip(v1, vy):
        robot.cmd_vel_Publisher(linear_x=vx_val/250, linear_y=vy_val/250, angular_z=0.0)
        print(f"Publishing cmd_vel - Linear X: {vx_val/250}, Linear Y: {vy_val/250}")
        robot.rate.sleep()  # Adjust sleep time to match the parabolic blend time step
    robot.move_in_triangle()
    rospy.sleep(3)
    robot.stop_recording()  # Ensure recording stops















