#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates

class ModelStatesListener:
    def __init__(self, output_file):
        self.output_file = output_file
        rospy.init_node('model_states_listener', anonymous=True)
        self.rate = rospy.Rate(30)  # Set frequency to 30 Hz
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        rospy.loginfo("Subscribed to /gazebo/model_states")

    def callback(self, data):
        try:
            with open(self.output_file, 'a') as file:
                for i, pose in enumerate(data.pose):  # Iterate over the pose array
                    position = pose.position
                    orientation = pose.orientation
                    file.write(f"Position: {position}, Orientation: {orientation}\n")
            rospy.loginfo("Pose data written to file")
        except Exception as e:
            rospy.logerr(f"Failed to write data to file: {e}")
        self.rate.sleep()  # Maintain 30 Hz frequency

if __name__ == "__main__":
    output_file = "/home/hiwonder/jethexa_vm/model_states_data.txt"
    listener = ModelStatesListener(output_file)
    rospy.spin()
