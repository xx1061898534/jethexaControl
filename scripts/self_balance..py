#!/usr/bin/env python3

import time
import rospy
from jethexa_sdk import pid
from scipy.spatial.transform import Rotation as R
from jethexa_controller import build_in_pose, jethexa, kinematics_api
import sensor_msgs.msg
import csv

class SelfBalancingNode:

    def __init__(self):
        rospy.init_node('self_balancing_2')
        
        self.pid_pitch = pid.PID(0.2, 0.03, 0.015)
        self.pid_roll = pid.PID(0.2, 0.03, 0.015)
        self.pitch = 0
        self.roll = 0
        
        self.jethexa = jethexa.JetHexa(self, pwm=False)
        self.timestamp = time.time()
        self.start_time = rospy.Time.now()
        
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/filtered")
        self.jethexa.set_pose(build_in_pose.DEFAULT_POSE_M, (0, 0, 140), duration=1.5)
        
        self.imu_sub = rospy.Subscriber(self.imu_topic, sensor_msgs.msg.Imu, self.imu_callback, queue_size=2)

        self.csv_file = open('imu_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Elapsed_Time', 'IMU_X', 'IMU_Y', 'Pitch', 'Roll', 'PID_Pitch_Output', 'PID_Roll_Output'])

    def imu_callback(self, imu_msg):
        try:
            q = imu_msg.orientation
            r = R.from_quat((q.x, q.y, q.z, q.w))
            x, y, z = r.as_euler('xyz')
            
            self.pid_pitch.update(y)
            self.pid_roll.update(x)
            
            try:
                pitch = self.pitch - self.pid_pitch.output
                roll = self.roll - self.pid_roll.output
                rospy.loginfo("Pitch:{:>6.2f}, Roll:{:>6.2f}".format(pitch, roll))
                
                new_pose = kinematics_api.transform_euler(build_in_pose.DEFAULT_POSE_M, (0, 0, 0), 'xyz', (roll, pitch, 0), degrees=False)
                self.jethexa.set_pose_base(new_pose, 0.02)
                
                self.pitch, self.roll = pitch, roll

                # Write data to CSV
                elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
                self.csv_writer.writerow([elapsed_time, x, y, pitch, roll, self.pid_pitch.output, self.pid_roll.output])
            except Exception as e:
                rospy.logerr(str(e))
                self.pitch = 0
                self.roll = 0
        except Exception as e:
            rospy.logerr(str(e))

    def __del__(self):
        self.csv_file.close()

if __name__ == "__main__":
    try:
        self_balancing_node = SelfBalancingNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))