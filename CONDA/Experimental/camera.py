# -*- coding: utf-8 -*-
"""
Created on Tue Mar  2 13:16:28 2021

@author: enriq
"""

# from imutils.video import WebCamVideoStream
import cv2

cam = cv2.VideoCapture(1)

while True:
    ret, frame = cam.read()
    if not ret:
        print("failed to grab frame")
        break
    cv2.imshow("test", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cam.release()

cv2.destroyAllWindows()