import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def publish_fake_odometry():
    rospy.init_node('fake_odometry_publisher')
    pub = rospy.Publisher('/fake_odometry', Odometry, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now()

    position_x = 0.0
    velocity_x = 10.0  # m/s

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed = (current_time - start_time).to_sec()
        position_x = velocity_x * elapsed

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Position updates only along x-axis
        odom.pose.pose.position = Point(position_x, 0.0, 8.0)
        odom.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # No rotation

        # Velocity along x
        odom.twist.twist = Twist(Vector3(velocity_x, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

        pub.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_fake_odometry()
    except rospy.ROSInterruptException:
        pass