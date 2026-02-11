## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################
import os
import pyrealsense2 as rs
import numpy as np
import cv2
# from matplotlib import pyplot as plt

path_to_images = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'images')

def image_capture():

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # # Get device product line for setting a supporting resolution
    # pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    # pipeline_profile = config.resolve(pipeline_wrapper)
    # device = pipeline_profile.get_device()
    # device_product_line = str(device.get_info(rs.camera_info.product_line))

    # found_rgb = False
    # for s in device.sensors:
    #     if s.get_info(rs.camera_info.name) == 'RGB Camera':
    #         found_rgb = True
    #         break
    # if not found_rgb:
    #     print("The demo requires Depth camera with Color sensor")
    #     exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    # Get stream profile and camera intrinsics
    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    depth_intrinsics = depth_profile.get_intrinsics()
    # depth_extrinsics = depth_profile.get_extrinsics()
    color_intrinsics = color_profile.get_intrinsics()


    align_to = rs.stream.color  # align depth to color
    align = rs.align(align_to)

    try:
        for _ in range(16):

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            # if not frames: continue
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            # depth_frame = frames.get_depth_frame()
            # color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            # depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # print("Shape of the depth image", depth_image.shape)
            # print("Shape of the color image", color_image.shape)

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # depth_colormap_dim = depth_colormap.shape
            # color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            # if depth_colormap_dim != color_colormap_dim:
            #     resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            #     images = np.hstack((resized_color_image, depth_colormap))
            # else:
            #     images = np.hstack((color_image, depth_colormap))

            # Show images
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense', images)
            # cv2.imshow('RealSense', color_image)
            # cv2.waitKey(1)
            # Filename
            filename = path_to_images + '/shapes.png'

            # Using cv2.imwrite() method
            # Saving the image
            cv2.imwrite(filename, color_image)
            # pipeline.stop()



    finally:

        # Stop streaming
        pipeline.stop()
        print("stopped")

    return depth_frame, depth_intrinsics, color_intrinsics


# test the code

# depth_frame, depth_intrinsics, color_intrinsics = image_capture()
