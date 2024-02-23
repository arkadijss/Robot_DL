import sys
import cv2
import numpy as np

import app_auth


def get_cam_link(cam_dev, name, idx, res, color_space, fps):
    cam_link = cam_dev.subscribeCamera(name, idx, res, color_space, fps)
    return cam_link


def get_depth_cam_link(cam_dev):
    cam_link = get_cam_link(cam_dev, "depth", 2, 1, 11, 10)
    return cam_link


def get_rgb_cam_link(cam_dev):
    cam_link = get_cam_link(cam_dev, "rgb", 0, 1, 13, 10)
    return cam_link


def get_image(cam_dev, cam_link):
    image_raw = cam_dev.getImageRemote(cam_link)
    image = np.frombuffer(image_raw[6], np.uint8).reshape(image_raw[1], image_raw[0], 3)
    return image


if __name__ == "__main__":
    program_path = sys.argv[0]
    qi_url = "tcps://x.x.x.x:x"
    username = "nao"
    password = "nao"
    app = app_auth.make_application(program_path, qi_url, username, password)
    app.start()

    s = app.session

    cam_dev = s.service("ALVideoDevice")

    rgb_cam_link = get_rgb_cam_link(cam_dev)
    depth_cam_link = get_depth_cam_link(cam_dev)

    show = True
    while True:
        image_rgb = get_image(cam_dev, rgb_cam_link)
        image_depth = get_image(cam_dev, depth_cam_link)
        image_depth = cv2.cvtColor(image_depth, cv2.COLOR_BGR2GRAY)

        if show:
            cv2.namedWindow("rgb", cv2.WINDOW_NORMAL)
            cv2.imshow("rgb", image_rgb)

            cv2.namedWindow("depth", cv2.WINDOW_NORMAL)
            cv2.imshow("depth", image_depth)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
