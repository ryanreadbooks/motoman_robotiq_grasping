"""
@ Author: ryanreadbooks
@ Time: 2021/1/5
@ File name: realsense.py
@ File description: realsense相机类
"""

import os
import time
from threading import Timer

import pyrealsense2 as rs
import cv2 as cv
import numpy as np
import glob


class RealsenseCamera:

    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)  # 格式也可以设置成rgb8
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        self.profile = None
        self.aligner = rs.align(rs.stream.color)

        self.is_on = False  # 标志位，标记pipeline是否已经启动了
        self.decimation_filter = rs.decimation_filter()
        self.spatial_filter = rs.spatial_filter()
        self.decimation_filter.set_option(rs.option.filter_magnitude, 3)
        self.spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.55)
        self.hole_filling_filter = rs.hole_filling_filter(mode=2)
        self._num_count = 0

    def capture_video(self, file_dir: str, need_depth=False, record=True):
        """
        录制一段视频，并且保存到指定路径
        :param file_dir: 保存的指定路径
        :param need_depth: 是否需要深度
        :param record: 是否需要保存视频
        :return:
        """
        existing_files = sorted(glob.glob(os.path.join(file_dir, '*.jpg')))
        if len(existing_files) != 0:
            cur_last_one = int(os.path.basename(existing_files[-1])[0:4])
            self._num_count = cur_last_one + 1
        self.start()
        video_writer_depth = None
        video_writer_color = None
        if record:
            # 一些视频录制需要的初始化
            recorded_avi_name = os.path.join(file_dir, str(time.time()) + '_color.avi')
            video_writer_color = cv.VideoWriter(recorded_avi_name, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (self.width, self.height))
            if need_depth:
                recorded_avi_name_depth = os.path.join(file_dir, str(time.time()) + '_depth.avi')
                video_writer_depth = cv.VideoWriter(recorded_avi_name_depth, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (self.width, self.height))

        try:
            # 可视化的循环
            # shows the image
            if need_depth:
                cv.namedWindow('Realsense depth', cv.WINDOW_AUTOSIZE)
            cv.namedWindow('Realsense color', cv.WINDOW_AUTOSIZE)
            while True:
                frames = self.pipeline.wait_for_frames()
                # 将color的像素和深度图的像素对齐
                aligned_frames = self.aligner.process(frames)

                color_frame = aligned_frames.get_color_frame()
                if not color_frame:
                    continue
                color_image = np.asanyarray(color_frame.get_data())
                if need_depth:
                    depth_frame = aligned_frames.get_depth_frame()
                    depth_image = np.asanyarray(depth_frame.get_data())
                    depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.05), cv.COLORMAP_JET)

                    # show the depth map if needed
                    cv.imshow('Realsense depth', depth_colormap)
                    # save the video frame
                    if record:
                        video_writer_depth.write(depth_colormap)

                cv.imshow('Realsense color', color_image)

                # save the color image
                if record:
                    video_writer_color.write(color_image)

                key_pressed = cv.waitKey(1)
                if key_pressed == ord('q') or key_pressed == ord('Q') or key_pressed == 27:  # 按q或者esc退出
                    cv.destroyAllWindows()
                    break
                elif key_pressed == ord('s') or key_pressed == ord('S'):
                    # 按s保存一帧图片
                    save_name = str(self._num_count).zfill(4)
                    self._num_count += 1
                    # print(save_name)
                    saved_time = time.time()
                    color_saved_name = os.path.join(file_dir, str(save_name) + '_color.jpg')
                    cv.imwrite(color_saved_name, color_image)
                    print('Color image saved at {}'.format(color_saved_name))
                    if need_depth:
                        depth_saved_name = os.path.join(file_dir, str(save_name) + '_depth.png')
                        cv.imwrite(depth_saved_name, depth_image)
                        print('Depth image saved at {}'.format(depth_saved_name))
        finally:
            self.stop()

    def fetch_one_frame(self):
        """
        提供图片一帧，包括RGB和深度图
        :return: RGB图像和深度图像(该深度图为原始的深度信息，需要乘以scale才得到以米为单位的深度值；
                scale通过self.profile.get_device().first_depth_sensor().get_depth_scale()获得)
        """
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.aligner.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            intrinsic = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
            depth_intrinsic = depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
            return color_image, depth_image, intrinsic, depth_intrinsic
        except Exception as e:
            print(f'ERROR:相机断开连接;{type(e)}, {str(e)}')

    def start(self):
        """
        启动pipeline
        :return:
        """
        if not self.is_on:
            self.profile = self.pipeline.start(self.config)
            self.is_on = True

    def stop(self):
        """
        关闭pipeline
        :return:
        """
        if self.is_on:
            self.pipeline.stop()
            self.is_on = False

    def get_depth_scale(self):
        return self.profile.get_device().first_depth_sensor().get_depth_scale()
