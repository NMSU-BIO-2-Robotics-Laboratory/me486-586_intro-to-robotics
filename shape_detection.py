import cv2
import numpy as np
from matplotlib import pyplot as plt
from image_capture_ import *
import pyrealsense2 as rs

# take an RGB image save it as shapes.png
depth_frame, depth_intrinsics, color_intrinsics = image_capture()

# reading image
img_org = cv2.imread('shapes.png')
cv2.imshow('original',img_org)

print("Shape of the image", img_org.shape)

# [rows, columns]
img = img_org[125:370, 310:640]  # [ymin:ymax, xmin:xmax] [480, 640]
y_min_offset = 125   # x_min
x_min_offset = 310   # y_min

cv2.imshow('cropped', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# converting image into blurred image
blur = cv2.GaussianBlur(img, (7,7), 0)
cv2.imshow('blur',blur)

# converting image into grayscale image
gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
cv2.imshow('gray',gray)
#cv2.waitKey(0)

# setting threshold of gray image
_, threshold = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)
cv2.imshow('threshold',threshold)

# using a findContours() function
contours, _ = cv2.findContours(
    threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#print(len(contours))

i = 0

# list for storing names of shapes
for contour in contours:

    # here we are ignoring first counter because
    # findcontour function detects whole image as shape
    if i == 0:
        i = 1
        continue

    # cv2.approxPloyDP() function to approximate the shape
    approx = cv2.approxPolyDP(
        contour, 0.035 * cv2.arcLength(contour, True), True)
    print(len(approx))

    # using drawContours() function
    cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)

    # finding the center point of shape
    M = cv2.moments(contour)

    if M['m00'] != 0.0:
        x = int(M['m10'] / M['m00'])
        y = int(M['m01'] / M['m00'])
    print("x, y = ", x + x_min_offset, y + y_min_offset)

    depth_xy = depth_frame.get_distance(int(x + x_min_offset), int(y + y_min_offset))
    shape_point_xyz = np.array(rs.rs2_deproject_pixel_to_point(color_intrinsics, [int(x + x_min_offset), int(y + y_min_offset)], depth_xy))
    print(np.dot(shape_point_xyz, 1000))

    # putting shape name at center of each shape
    if len(approx) == 3:
        cv2.putText(img, 'Triangle', (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    elif len(approx) == 4:
        cv2.putText(img, 'Quadrilateral', (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    elif len(approx) == 5:
        cv2.putText(img, 'Pentagon', (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    elif len(approx) == 6:
        cv2.putText(img, 'Hexagon', (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    elif len(approx) == 8:
        cv2.putText(img, 'Octagon', (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    else:
        cv2.putText(img, 'ellipse', (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # displaying the image after drawing contours
    cv2.imshow('shapes', img)

cv2.waitKey(0)
cv2.destroyAllWindows()
