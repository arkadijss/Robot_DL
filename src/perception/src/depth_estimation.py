#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import numpy as np
from PIL import Image as PILImage
import scripts.depth_estimation.metric_depth.estimate as estimate


class DepthEstimation:
    def __init__(self, image_topic="~/camera/image_raw", depth_topic="~/depth_nn"):
        rospy.init_node("depth_estimation_node", anonymous=True)
        self.model = estimate.build_model_metric()
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber(
            image_topic, Image, self.image_callback
        )
        self.depth_publisher = rospy.Publisher(depth_topic, Image, queue_size=10)

    def image_callback(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            img_torch = estimate.prepare_sample(cv_image).cuda()

            predicted_depth = estimate.infer(self.model, img_torch)

            # interpolate to original size
            prediction = torch.nn.functional.interpolate(
                predicted_depth,
                size=cv_image.shape[:-1],
                mode="bicubic",
                align_corners=False,
            )

            output = prediction.squeeze().cpu().numpy()

            output = (output * 1000).astype(np.uint16)

            # Convert depth image to ROS message
            depth_msg = self.bridge.cv2_to_imgmsg(output, "16UC1")
            depth_msg.header = ros_image.header

            # Publish the depth image
            self.depth_publisher.publish(depth_msg)

        except Exception as e:
            rospy.logerr(e)


if __name__ == "__main__":
    try:
        image_topic = "/naoqi_driver/camera/front/image_raw"
        depth_node = DepthEstimation(image_topic=image_topic)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Depth estimation node terminated.")
