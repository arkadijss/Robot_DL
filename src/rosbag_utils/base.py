import rosbag

class BagManager():
    def __init__(self, bag_path, topics):
        self.bag_path = bag_path
        self.topics = topics
        self.create_bag()

    def create_bag(self):
        self.bag = rosbag.Bag(self.bag_path, "w")

    def close_bag(self):
        self.bag.close()

    def write(self, topic, data):
        self.bag.write(topic, data)