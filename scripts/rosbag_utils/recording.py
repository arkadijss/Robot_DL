from threading import Lock

import rospy
import rostopic

from rosbag_utils.base import BagManager
from conf.rosbag_conf import topics


class BagRecorder(BagManager):
    def __init__(self, bag_path, topics):
        super().__init__(bag_path, topics)
        self.write_lock = Lock()  # Create a lock for write operations

    def setup_subscribers(self):
        for topic in self.topics:
            msg_class = rostopic.get_topic_class(topic)[0]
            subscriber = rospy.Subscriber(
                topic, msg_class, self.callback, callback_args=topic
            )

    def callback(self, data, topic):
        with self.write_lock:
            rospy.loginfo(f"Writing to {topic}")
            self.write(topic, data)

    def record(self):
        self.setup_subscribers()
        rospy.spin()
        self.close_bag()


if __name__ == "__main__":
    rospy.init_node("bag_recorder")

    bag_path = "bagfile.bag"
    input_topics = topics

    bag_recorder = BagRecorder(bag_path, input_topics)
    bag_recorder.record()
