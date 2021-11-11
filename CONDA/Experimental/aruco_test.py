from aruco_camera import Camera, ArucoTracker
import cv2
import time


def resize(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image

    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)

    else:
        r = width / float(w)
        dim = (width, int(h * r))

    resized = cv2.resize(image, dim, interpolation=inter)

    return resized

def main():
    tracker_top = ArucoTracker()
    tracker_bottom = ArucoTracker()


    print("Starting video stream...")

    camera_top = Camera(src = 2).start()
    camera_bottom = Camera(src = 1).start()

    time.sleep(2.0)

    start_time = time.time()
    adjust_cs = False
    while time.time() - start_time < 0.1:
        frame_bottom = camera_bottom.read()
        frame_top = camera_top.read()

        frame_top = resize(frame_top, width = 600)
        frame_bottom = resize(frame_bottom, width = 600)
        
        frame_top = cv2.cvtColor(frame_top, cv2.COLOR_BGR2GRAY)
        ret3_top,frame_top = cv2.threshold(frame_top,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        frame_bottom = cv2.cvtColor(frame_bottom, cv2.COLOR_BGR2GRAY)
        ret3_bottom,frame_bottom = cv2.threshold(frame_bottom,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        tag_poses = tracker_top.aruco_pose(frame_top) # returns dictionary with values 'x', 'y', 'theta'
        tracker_bottom.calibrated = True
        tracker_bottom.scale = tracker_top.scale
        
        
        tag_poses2 = tracker_bottom.aruco_pose(frame_bottom)
        
        if not adjust_cs :
            tracker_bottom.cs['y'] -= 0.04
            adjust_cs = True

        # tag_poses.update(tag_poses2)
        

        # print(tag_poses['1']['theta'])

    # print(tracker_top.scale)
    # print(tracker_bottom.scale)
    
    camera_top.stop()
    camera_bottom.stop()
    
    print('Tag 1')
    print(tag_poses['1'])
    print(tag_poses2['1'])
    print('Tag 2')
    print(tag_poses['2'])
    print(tag_poses2['2'])
    print('Tag 3')
    print(tag_poses['3'])
    print(tag_poses2['3'])
    print('Tag 4')
    print(tag_poses['4'])
    print(tag_poses2['4'])
    print('Tag 5')
    print(tag_poses['5'])
    print(tag_poses2['5'])

    
    print('Centros')
    print(tracker_bottom.scale * tracker_top.cs['x'] / tracker_top.scale)
    print(tracker_top.cs)
    print(tracker_bottom.cs)


if __name__ == '__main__':
    main()
