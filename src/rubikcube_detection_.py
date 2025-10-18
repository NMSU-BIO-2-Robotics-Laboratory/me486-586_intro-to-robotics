## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.
#####################################################
## librealsense tutorial #1 - Accessing depth data ##
#####################################################

# First import the library
import pyrealsense2 as rs
import numpy as np
# from pupil_apriltags import Detector    # works with Windows
from dt_apriltags import Detector      # works with Linux
import cv2


def rubikcube_detection_func(tag_size_in_m):
    # try:
    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()

    # Configure streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    # Start streaming
    pipeline.start(config)

    # Get stream profile and camera intrinsics
    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()

    w, h = depth_intrinsics.width, depth_intrinsics.height
    fx, fy, cx, cy = depth_intrinsics.fx, depth_intrinsics.fy, depth_intrinsics.ppx, depth_intrinsics.ppy
    print("w, h, fx, fy, cx, cy are:", w, h, fx, fy, cx, cy)

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color    # depth
    align = rs.align(align_to)

    at_detector = Detector(families='tagStandard41h12',
                           nthreads=1,
                           quad_decimate=1.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)
    tags_point_xyz  = []
    for _ in range(1):
        # This call waits until a new coherent set of frames is available on a device
        # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will
        # return stable values until wait_for_frames(...) is called

        frames = pipeline.wait_for_frames()
        #if not frames: continue

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # Depth image
        depth_image = np.asanyarray(depth_frame.get_data())
        # Apriltag detection, 3D pose estimate
        color_image_apriltag = np.asanyarray(color_frame.get_data())
        cv2.imwrite('/images/apriltag_test_realtime.jpg', color_image_apriltag)
        image = cv2.imread('/images/apriltag_test_realtime.jpg', cv2.IMREAD_GRAYSCALE)
        camera_params = (fx, fy, cx, cy)
        tags = at_detector.detect(image, True, camera_params, tag_size_in_m) # unit is m
        print(tags)
        cv2.imshow('Original image', image)
        print("Shape of the image", image.shape)
        # cv2.imshow('Depth image', depth_image)
        cv2.waitKey(1)
        color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(color_image, tuple(tag.corners[idx - 1, :].astype(int)),
                         tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
            cv2.circle(color_image, (cX, cY), 5, (0, 0, 255), -1)
            print("cX, cY = ", int(cX), int(cY))
            depth_xy = depth_frame.get_distance(int(cX), int(cY))
            apriltag_point_xyz = np.array(rs.rs2_deproject_pixel_to_point(depth_intrinsics, [int(cX), int(cY)], depth_xy))
            #print(apriltag_point_xyz)
            tags_point_xyz.append(apriltag_point_xyz)


            cv2.putText(color_image, str(tag.tag_id),
                        org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.8,
                        color=(0, 0, 255))

        cv2.imshow('Detected tags', color_image)
        cv2.waitKey(0)

        # for tag in tags:
            # print(tag.pose_t)
            # print(tag.pose_R)
            # r = Rot.from_matrix(tag.pose_R)
            # print(r.as_euler('zyx', degrees=True))

        # cv2.waitKey(0)

        # pc = rs.pointcloud()
        # points = pc.calculate(depth_frame)
        # pc.map_to(color_frame)
        if not depth_frame: continue

    return tags_point_xyz, tags

# test the code (comment it when you want to use it in the robot arm's code)


# tags_point_xyz_m, tags = rubikcube_detection_func(0.017)
# pose_rot = []
# i = 0
# for tag in tags:
#     print("apriltag_pose_R = ", tag.pose_R)
#     pose_rot = np.array(tag.pose_R)
#     pose_trans = np.dot(np.array(tag.pose_t), 1000)
#     print("pose_rot = ", pose_rot)
#     print("pose_trans = ", pose_trans)
#     Rp = np.concatenate((pose_rot, pose_trans), axis=1)
#     print("Rp = ", Rp)
#     T_c_a = np.concatenate((Rp, np.array([[0, 0, 0, 1]])), axis=0)
#     print("T_c_a = ", T_c_a)
#     apriltag_point_xyz_mm = np.dot(tags_point_xyz_m[i], 1000)
#     print("apriltag_pose_xyz = ", apriltag_point_xyz_mm)
#     i = i + 1

