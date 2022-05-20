#!/usr/bin/env python3
import sys
import rospy
import cv2 as cv
import cv_py
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


cap = cv.VideoCapture("/dev/video0")
if not cap.isOpened():
    sys.exit("Camera open failed")

hsv_name = "Hmin", "Smin", "Vmin", "Hmax", "Smax", "Vmax"
hsv_init = 21, 128, 182, 45, 255, 255
hsv_count = 180, 255, 255, 180, 255, 255
HSV = cv_py.Trackbar("TRACKBAR", hsv_name, hsv_init, hsv_count)
HSV.set_group("min", ("Hmin", "Smin", "Vmin"))
HSV.set_group("max", ("Hmax", "Smax", "Vmax"))

rospy.init_node("cam")
pub = rospy.Publisher("/robot/robot_controller/command", Float64MultiArray, queue_size=3)
msg = Float64MultiArray()

count = 0
rotate = 1
while not rospy.is_shutdown():
    ret, frame = cap.read()
    if ret:
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        binary = cv.inRange(hsv, HSV.values("min"), HSV.values("max"))
        contours, _ = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        index = cv_py.largest_contour(contours, 1300, 1)
        if index:
            if cv_py.simple_line_check(contours[index[0]], 20, frame):
                rotate = cv.minAreaRect(contours[index[0]])[2] * -4 * 3.14 / 360

            center, radius = cv.minEnclosingCircle(contours[index[0]])
            cv.circle(frame, (int(center[0]), int(center[1])), int(radius), (0, 255, 0), 3)
            dim = []
            dim.append(MultiArrayDimension("", 3, 1))
            msg.data = center[0] / 640 * 6.28 - 3.14, center[1] / 480 * 2.5 + 0.1, rotate
            msg.layout.dim = dim
            pub.publish(msg)
            print(rotate)

    cv.imshow("frame", frame)
    cv.imshow("binary", binary)
    key = cv.waitKey(1)
    if key == ord('q'):
        HSV.print_values()
        break
    elif key == ord(' '):
        HSV.reset()

cap.release()
cv.destroyAllWindows()
