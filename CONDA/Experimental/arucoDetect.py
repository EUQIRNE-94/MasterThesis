# -*- coding: utf-8 -*-
"""
Created on Tue Mar  2 12:04:18 2021

@author: enriq
"""

print("\014")

from imutils.video import VideoStream
import imutils
import cv2
import time
from matplotlib import pyplot as plt

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
arucoParams = cv2.aruco.DetectorParameters_create()

vs = VideoStream(src=1).start()
time.sleep(2.0)

while True:
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 1000 pixels
    frame = vs.read()
    frame_color = frame.copy()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret3,frame = cv2.threshold(frame,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
	# detect ArUco markers in the input frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame,arucoDict,parameters=arucoParams)
    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
		# flatten the ArUco IDs list
        ids = ids.flatten()
		# loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
 			# extract the marker corners (which are always returned
 			# in top-left, top-right, bottom-right, and bottom-left
 			# order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRightight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeftLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            # draw the bounding box of the ArUCo detection
            # cv2.line(frame, tuple(topLeft), tuple(topRight), (0, 255, 0), 2)
            # cv2.line(frame, tuple(topRight), tuple(bottomRight), (0, 255, 0), 2)
            # cv2.line(frame, tuple(bottomRight), tuple(bottomLeft), (0, 255, 0), 2)
            # cv2.line(frame, tuple(bottomLeft), tuple(topLeft), (0, 255, 0), 2)
 			# compute and draw the center (x, y)-coordinates of the
 			# ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame_color, (cX, cY), 4, (0, 0, 255), -1)
            print([markerID,cX,cY])
 			# draw the ArUco marker ID on the frame
            cv2.putText(frame_color, str(markerID),
				(topLeft[0], topLeft[1] - 15),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)

	# Show the output frame
    cv2.imshow("Frame", frame_color)
    cv2.imshow("Binario",frame)
    key = cv2.waitKey(1) & 0xFF
	# if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

# Cleanup
# cv2.imwrite('Top.png',frame_color)
# cv2.imwrite('TopOTSU.png',frame)
cv2.destroyAllWindows()
img = cv2.cvtColor(frame_color, cv2.COLOR_BGR2GRAY)
plt.hist(img.ravel(),255,[0,255])
plt.show()
vs.stop()