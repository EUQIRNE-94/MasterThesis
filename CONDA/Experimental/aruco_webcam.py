from aruco_camera import Camera
import cv2
import time
import numpy as np

def aruco_pose(frame, aruco_dict, aruco_params):
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

    tag_locations = {}

    if len(corners) > 0:
        ids = ids.flatten()

        for (tag_corners, tag_id) in zip(corners, ids):
            corners = tag_corners.reshape((4,2))
            (top_left, top_right, bottom_right, bottom_left) = corners

            top_right = (int(top_right[0]), int(top_right[1]))
            top_left = (int(top_left[0]), int(top_left[1]))
            bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
            bottom_left = (int(bottom_left[0]), int(bottom_left[1]))

            center_x = int((top_left[0] + bottom_right[0]) / 2.0)
            center_y = int((top_left[1] + bottom_right[1]) / 2.0)
            
            a = bottom_right[0] - bottom_left[0]
            b = bottom_right[1] - bottom_left[1]
            
            center_theta = np.arctan2(b, a)
            
            tag_locations[tag_id] = [(center_x, center_y), center_theta]

        print(tag_locations)

    return tag_locations

def resize(image, width=None, height=None, inter=cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    if width is None and height is None:
        return image

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the image
    resized = cv2.resize(image, dim, interpolation=inter)

    # return the resized image
    return resized

def main():
    ARUCO_DICT = {
    	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }

    aruco_key = "DICT_4X4_50"

    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_key])
    aruco_params = cv2.aruco.DetectorParameters_create()

    print("Starting video stream...")

    vs = Camera(src = 0).start()
    time.sleep(2.0)

    start_time = time.time()

    while time.time() - start_time < 5:
        frame = vs.read()
        frame = resize(frame, width = 600)
        tag_poses = aruco_pose(frame, aruco_dict, aruco_params)
        
        print(tag_poses)

    vs.stop()


if __name__ == '__main__':
    main()
