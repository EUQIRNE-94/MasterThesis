from threading import Thread
import cv2
import numpy as np

class Camera:
    def __init__(self, src = 0, name="Webcam"):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()

        self.name = name
        self.stopped = False

    def start(self):
        stream = Thread(target = self.update, name = self.name, args = ())
        stream.daemon = True
        stream.start()
        return self

    def update(self):
        while not self.stopped:
            (self.grabbed, self.frame) = self.stream.read()

        if self.stopped:
            self.stream.release()
            return

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True


class ArucoTracker:
    def __init__(self, aruco_key = 'DICT_4X4_100', side_length = 0.18, position = 'bottom'):
        self.aruco_dictionaries =  {
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
        self.aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dictionaries[aruco_key])
        self.params = cv2.aruco.DetectorParameters_create()
        self.calibrated = False
        self.length = side_length
        self.has_coordinate_system = False
        self.tag_locations = {}
        self.attempts = 0
        self.scale = 1.0
        self.position = position
        self.cs = {
            'x' : 0.0,
            'y' : 0.0,
            'theta': 0.0
            }


    def aruco_pose(self, frame):
        centers = self.aruco_centers(frame)
        return centers

    def calibrate_pixel2meter(self, corners):
        (top_left, top_right, bottom_right, bottom_left) = corners
        side = np.linalg.norm(top_right - top_left)
        self.scale = self.length / side # side length meter / pixel
        self.scale = np.abs(self.scale)
        pass

    def coordinate_system(self):
        tag_30 = self.tag_locations['30']
        tag_40 = self.tag_locations['40']

        cs_x = (tag_30['x'] + tag_40['x'])/2.0
        cs_y = (tag_30['y'] + tag_40['y'])/2.0

        self.cs['x'] = np.abs(cs_x)
        self.cs['y'] = np.abs(cs_y)

        pass

    def aruco_centers(self, frame):
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.params)

        if len(corners) > 0:
            ids = ids.flatten()

            if not self.calibrated and np.any(np.isin(ids, [30, 40])):
                for (tag_corners, tag_id) in zip(corners, ids):
                    if not self.calibrated and (tag_id == 30 or tag_id == 40) :
                        corners_calibration = tag_corners.reshape((4,2))
                        self.calibrate_pixel2meter(corners_calibration)
                        self.calibrated = True
                        # print('Scale : ', self.scale)

            for (tag_corners, tag_id) in zip(corners, ids):
                corners = tag_corners.reshape((4,2))
                (top_left, top_right, bottom_right, bottom_left) = corners

                # top_right = (int(top_right[0]), int(top_right[1]))
                top_left = (int(top_left[0]), int(top_left[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))

                center_x = (top_left[0] + bottom_right[0]) / 2.0
                center_y = (top_left[1] + bottom_right[1]) / 2.0

                a = bottom_right[0] - bottom_left[0]
                b = bottom_right[1] - bottom_left[1]

                center_theta = np.arctan2(b, a)

                pose = {}
                pose['x'] = center_x * self.scale - self.cs['x']
                pose['y'] =  - ((center_y * self.scale) - self.cs['y'])
                pose['theta'] = center_theta

                self.tag_locations[str(tag_id)] = pose

            # print(self.tag_locations)

        if self.attempts < 5 and self.calibrated and not self.has_coordinate_system:
            print('t-cal')
            try:
                self.coordinate_system()
                self.has_coordinate_system = True

                for tag in self.tag_locations:
                    self.tag_locations[tag]['x'] -= self.cs['x']
                    self.tag_locations[tag]['y'] += self.cs['y']


                print(self.cs)
            except:
                print('Could not find CS')
            self.attempts += 1



        return self.tag_locations
