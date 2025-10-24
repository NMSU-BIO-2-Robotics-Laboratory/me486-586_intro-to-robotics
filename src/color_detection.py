#!/usr/bin/env python3

# -- Here is how to perform color detection in Python using OpenCV: Import necessary libraries.
import os
import cv2
import numpy as np

path_to_images = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'images')
# -- load the image.
image = cv2.imread(path_to_images + '/savedImage.jpg')  # 'shapes.png')


# --Convert the image to HSV.
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# -- OpenCV reads images in BGR format by default,
# -- so converting to HSV (Hue, Saturation, Value) makes color detection easier.
# -- Define the color range.
# Yellow:
# Lower range: (20, 100, 100)
# Upper range: (40, 255, 255)
# Blue:
# Lower range: (100, 100, 100)
# Upper range: (130, 255, 255)
# Green:
# Lower range: (50, 100, 100)
# Upper range: (90, 255, 255)

# lower_bound = np.array([hue_min, saturation_min, value_min])
# upper_bound = np.array([hue_max, saturation_max, value_max])

lower_bound_red = np.array([0, 100, 100])
upper_bound_red = np.array([10, 255, 255])

lower_bound_green = np.array([50, 100, 100])
upper_bound_green = np.array([90, 255, 255])

lower_bound_blue = np.array([100, 100, 100])
upper_bound_blue = np.array([130, 255, 255])

lower_bound_yellow = np.array([20, 100, 100])
upper_bound_yellow = np.array([40, 255, 255])

# -- Replace hue_min, saturation_min, value_min, hue_max,
# saturation_max, and value_max with the appropriate values
# for the color you want to detect. You can use color picker tools or
# experiment with different values to find the correct range. Create a mask.
mask_red = cv2.inRange(hsv_image, lower_bound_red, upper_bound_red)
mask_green = cv2.inRange(hsv_image, lower_bound_green, upper_bound_green)
mask_blue = cv2.inRange(hsv_image, lower_bound_blue, upper_bound_blue)
mask_yellow = cv2.inRange(hsv_image, lower_bound_yellow, upper_bound_yellow)

# cv2.imshow('Blue Color Mask', mask_blue)
# cv2.imshow('Yellow Color Mask', mask_yellow)

# Merge the two masks using bitwise OR
merged_mask = cv2.bitwise_or(mask_red, mask_green)
merged_mask = cv2.bitwise_or(merged_mask, mask_yellow)
merged_mask = cv2.bitwise_or(merged_mask, mask_blue)

# -- This creates a binary mask where white pixels represent
# -- the detected color and black pixels represent everything else.

# -- Apply the mask to the original image:
# result_red = cv2.bitwise_and(image, image, mask=mask_red)
# result_green = cv2.bitwise_and(image, image, mask=mask_green)
result = cv2.bitwise_and(image, image, mask=merged_mask)


# -- This extracts the detected color from the original image,
# making it visible while the rest of the image is blacked out. Display the results.
cv2.imshow('Original Image', image)
cv2.imshow('Color Mask', merged_mask)
cv2.imshow('Detected Color', result)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

filename = path_to_images + '/detected_colors_mask.png'
cv2.imwrite(filename, merged_mask)

# -- Optional: Find contours.
contours, _ = cv2.findContours(merged_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(image, contours, -1, (0, 255, 0), 2)
cv2.imshow('Image with Contours', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# -- This can be used to outline the detected color regions in the original image.


