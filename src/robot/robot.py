import rospy
from geometry_msgs.msg import Twist


class Robot:
    def __init__(self, velocity_topic="/cmd_vel"):
        self.velocity_publisher = rospy.Publisher(velocity_topic, Twist, queue_size=10)
        self.velocity = Twist()

    def accelerate(self, speed=0.25):
        self.velocity.linear.x += speed
        self.publish_velocity()

    def decelerate(self, speed=0.25):
        self.velocity.linear.x += -speed
        self.publish_velocity()

    def accelerate_rotation(self, angular_speed=0.25):
        self.velocity.angular.z += angular_speed
        self.publish_velocity()

    def accelerate_clockwise(self, speed=0.25):
        self.accelerate_rotation(angular_speed=-speed)

    def accelerate_anticlockwise(self, speed=0.25):
        self.accelerate_rotation(angular_speed=speed)

    def publish_velocity(self):
        self.velocity_publisher.publish(self.velocity)

    def publish_velocity_continuously(self):
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                self.publish_velocity()
                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Robot velocity publisher stopped")
