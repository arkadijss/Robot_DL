import os
import rospy
import rosbag
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge


class ImageSaver:
    def __init__(self, bag_file, output_directory):
        self.bridge = CvBridge()
        self.output_directory = output_directory
        self.bag_images_directory = os.path.join(output_directory, "bag_images")
        os.makedirs(self.bag_images_directory, exist_ok=True)

        with rosbag.Bag(bag_file, "r") as bag:
            for topic, msg, t in bag.read_messages(
                topics=["/naoqi_driver/camera/front/image_raw"]
            ):
                self.image_callback(msg)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(e)
            return

        timestamp = round(msg.header.stamp.to_sec(), 2)
        image_filename = f"{timestamp}.png"
        image_path = os.path.join(self.bag_images_directory, image_filename)

        cv2.imwrite(image_path, cv_image)
        print(f"Saved image: {image_path}")

    def process_bag(self):
        rospy.init_node("image_saver", anonymous=True)
        rospy.spin()


if __name__ == "__main__":
    bag_file_path = "recordings/1.0.1.bag"  # Replace with your ROS bag file path
    output_directory = "extraction/1.0.1"  # Replace with your desired output directory

    image_saver = ImageSaver(bag_file_path, output_directory)
    image_saver.process_bag()
