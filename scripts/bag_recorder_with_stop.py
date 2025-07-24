#!/usr/bin/env python
import rospy
import subprocess
import signal
import math
import os
import sys
from nav_msgs.msg import Odometry

class BagRecorder:
    def __init__(self):
        rospy.init_node('bag_recorder_with_stop', anonymous=True)

        self.fake_odom = None
        self.gt_odom = None
        self.proc = None
        self.threshold = 7.0  # meters

        rospy.Subscriber("/wamv/base_pose_ground_truth", Odometry, self.fake_odom_cb)
        rospy.Subscriber("/uav0/base_pose_ground_truth", Odometry, self.gt_odom_cb)

        self.start_bag_recording()
        rospy.Timer(rospy.Duration(0.5), self.check_distance_loop)

    def start_bag_recording(self):
        topics = [
            #"/fake_odometry",
            "/wamv/base_pose_ground_truth",
            "/uav0/base_pose_ground_truth",
            "/uav0/bpn_cmd",
            "/uav0/mavros/setpoint_raw/attitude"
        ]
        cmd = ["rosbag", "record", "-O", "output.bag"] + topics
        self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        rospy.loginfo("Started rosbag recording.")

    def stop_bag_recording(self):
        if self.proc:
            rospy.loginfo("Stopping rosbag recording.")
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except Exception as e:
                rospy.logwarn("Error stopping rosbag: %s", str(e))
            self.proc = None

    def fake_odom_cb(self, msg):
        self.fake_odom = msg.pose.pose.position

    def gt_odom_cb(self, msg):
        self.gt_odom = msg.pose.pose.position

    def compute_distance(self, p1, p2):
        return math.sqrt(
            (p1.x - p2.x)**2 +
            (p1.y - p2.y)**2 +
            (p1.z+10 - p2.z)**2
        )

    def check_distance_loop(self, event):
        if self.fake_odom and self.gt_odom:
            dist = self.compute_distance(self.fake_odom, self.gt_odom)
            rospy.loginfo("Distance between fake and ground truth: %.2f m", dist)
            if dist < self.threshold:
                self.stop_bag_recording()
                rospy.signal_shutdown("Distance threshold reached (<2m)")

def shutdown_handler(signal_num, frame):
    rospy.loginfo("Caught Ctrl+C. Cleaning up...")
    recorder.stop_bag_recording()
    rospy.signal_shutdown("User interrupt (Ctrl+C)")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, shutdown_handler)
    recorder = BagRecorder()
    rospy.spin()
