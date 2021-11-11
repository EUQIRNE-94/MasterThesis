# -*- coding: utf-8 -*-
"""
Created on Sat Aug  7 17:48:37 2021

@author: enriq
"""

from imutils.video import VideoStream
import imutils
import cv2
import time
from matplotlib import pyplot as plt

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
arucoParams = cv2.aruco.DetectorParameters_create()

vs1 = VideoStream(src=1).start()
vs2 = VideoStream(src=2).start()
time.sleep(2.0)


while True:
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 1000 pixels
    frame1 = vs1.read()
    frame2 = vs2.read()
    
	# Show the output frame
    cv2.imshow("Cam1",frame1)
    cv2.imshow("Cam2",frame2)
    key = cv2.waitKey(1) & 0xFF
	# if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break


cv2.destroyAllWindows()
vs1.stop()
vs2.stop()