import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
from threading import Thread


class Robot:
    def __init__(self, velocity_topic="/pepper/cmd_vel"):
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
        while not rospy.is_shutdown():
            self.publish_velocity()
            rate.sleep()


class RobotController:
    def __init__(self, robot=Robot()):
        self.robot: Robot = robot
        self.key_pressed = None

    def on_press(self, key):
        self.key_pressed = key.char

    def start(self):
        self.robot_thread = Thread(target=self.robot.publish_velocity_continuously)
        self.robot_thread.start()

        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        while not rospy.is_shutdown():
            if self.key_pressed == "w":
                self.robot.accelerate()
            elif self.key_pressed == "s":
                self.robot.decelerate()
            elif self.key_pressed == "a":
                self.robot.accelerate_anticlockwise()
            elif self.key_pressed == "d":
                self.robot.accelerate_clockwise()
            self.key_pressed = None

        self.robot_thread.join()


if __name__ == "__main__":
    rospy.init_node("robot_controller")
    controller = RobotController()
    controller.start()
