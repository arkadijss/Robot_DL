import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard


class Robot:
    def __init__(self, velocity_topic="/pepper/cmd_vel"):
        self.velocity_publisher = rospy.Publisher(velocity_topic, Twist, queue_size=10)
        self.velocity = Twist()

    def move_forward(self, speed=1.0, duration=1.0):
        self.velocity.linear.x = speed
        self.publish_velocity()
        rospy.sleep(duration)
        self.velocity.linear.x = 0.0
        self.publish_velocity()

    def turn(self, angular_speed=1.0, duration=1.0):
        self.velocity.angular.z = angular_speed
        self.publish_velocity()
        rospy.sleep(duration)
        self.velocity.angular.z = 0.0
        self.publish_velocity()

    def turn_right(self, speed=1.0, duration=1.0):
        self.turn(angular_speed=-speed, duration=duration)

    def turn_left(self, speed=1.0, duration=1.0):
        self.turn(angular_speed=speed, duration=duration)

    def publish_velocity(self):
        self.velocity_publisher.publish(self.velocity)


class RobotController:
    def __init__(self, robot=Robot()):
        self.robot: Robot = robot
        self.key_pressed = None

    def on_press(self, key):
        self.key_pressed = key.char

    def start(self):
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        while not rospy.is_shutdown():
            if self.key_pressed == "w":
                self.robot.move_forward()
            elif self.key_pressed == "a":
                self.robot.turn_left()
            elif self.key_pressed == "d":
                self.robot.turn_right()
            self.key_pressed = None


if __name__ == "__main__":
    rospy.init_node("robot_controller")
    controller = RobotController()
    controller.start()
