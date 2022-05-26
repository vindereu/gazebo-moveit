#!/usr/bin/env python3
import sys
import rospy
import cv2 as cv
import cv_py
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from os.path import dirname

### Change these params to get the contour
hsv_init = (21, 128, 182, 45, 255, 255)
object_min_area = 1300

#### Frame Initialize ####
cap = cv.VideoCapture("/dev/video0")  # Change it if your camera isn't this port 
wait_time = 1
if not cap.isOpened():
    print("Can't open camera, start to read demo.mp4\n")
    cap.release()
    path = dirname(dirname(__file__)) + "/video/demo.mp4"
    cap = cv.VideoCapture(path)
    if not cap.isOpened():
        cap.release()
        sys.exit("Error! Can't load video.mp4")
    
    ### Do not edit these params ###
    hsv_init = 21, 128, 182, 45, 255, 255
    object_min_area = 1300
    wait_time = int(cap.get(cv.CAP_PROP_FPS))

height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
width = cap.get(cv.CAP_PROP_FRAME_WIDTH)

#### Information ####
print("\n###########################")
print("####### Camera node #######")
print("###########################")
print("Initialize success")
print("To save the HSV values, click frame and press q")
print("To reset all track bar values, click frame and press space")
print("###########################\n")

#### HSV Trackbar Create ####
hsv_name = "Hmin", "Smin", "Vmin", "Hmax", "Smax", "Vmax"
hsv_count = 180, 255, 255, 180, 255, 255
HSV = cv_py.Trackbar("TRACKBAR", hsv_name, hsv_init, hsv_count)
HSV.set_group("min", ("Hmin", "Smin", "Vmin"))
HSV.set_group("max", ("Hmax", "Smax", "Vmax"))
HSV.set_group("hsv_init", ("Hmin", "Smin", "Vmin", "Hmax", "Smax", "Vmax"))

#### ROS Init ####
rospy.init_node("camera")
pub = rospy.Publisher("/robot/robot_controller/command", Float64MultiArray, queue_size=3)
msg = Float64MultiArray()

#### Main Loop ####
count = 0
rotate = 1
while not rospy.is_shutdown():
    ret, frame = cap.read()
    if ret:
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        binary = cv.inRange(hsv, HSV.values("min"), HSV.values("max"))
        
        contours, _ = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        index = cv_py.largest_contour(contours, object_min_area, 1)
        if index:
            if cv_py.simple_line_check(contours[index[0]], 20, frame):
                rotate = cv.minAreaRect(contours[index[0]])[2] * -4 * 3.14 / 360

            center, radius = cv.minEnclosingCircle(contours[index[0]])
            cv.circle(frame, (int(center[0]), int(center[1])), int(radius), (0, 255, 0), 3)
            
            # Publish ROS messages
            dim = []
            dim.append(MultiArrayDimension("", 3, 1))
            msg.data = center[0] / width * 6.28 - 3.14, center[1] / height * 2.5 + 0.1, rotate
            msg.layout.dim = dim
            pub.publish(msg)

            print(f"rad: {rotate:.3f}", end="\r")
        
        cv.imshow("frame", frame)
        cv.imshow("binary", binary)

    else:
        cap.set(cv.CAP_PROP_POS_FRAMES, 1)

    key = cv.waitKey(wait_time)
    if key == ord('q') or key == ord('Q'):
        HSV.print_values("hsv_init")
        break
    elif key == ord(' '):
        HSV.reset()
