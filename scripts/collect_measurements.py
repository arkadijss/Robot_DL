import rospy
from rosbag_utils.recording import BagRecorder
from robot import RobotController
from conf.rosbag_conf import topics


if __name__ == "__main__":
    rospy.init_node("collect_measurements")
    bag_path = "bagfile.bag"
    bag_recorder = BagRecorder(bag_path, topics)
    bag_recorder.setup_subscribers()
    robot_controller = RobotController()
    robot_controller.start()
