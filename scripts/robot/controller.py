from threading import Thread

import rospy
from pynput import keyboard

from robot.robot import Robot


class RobotController:
    def __init__(self, robot=Robot()):
        self.robot: Robot = robot
        self.key_pressed = None

    def on_press(self, key):
        self.key_pressed = key.char

    def start(self):
        # try:
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
