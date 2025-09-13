# WayFinder.py

import numpy as np
import cv2
import random
import logging
import time
import threading
import ctypes # 这个库用于C语言函数的调用

class WayFinder():
    """
    寻迹小车
    """
    def __init__(self, track_map_dir, tmsize=(1920, 1080), car_w=40, car_h=30, view_wb=60, view_wt=220, view_h=140, bdist2c=17, maskw=188, maskh=120):
        """
        初始化寻迹小车
        长度单位取像素
        track_map: 赛道图像
        tmsize: 赛道图像尺寸
        car_w, car_h: 小车尺寸
        view_wb, view_wt, view_h: 视野尺寸，底边宽、顶边宽、高度
        bdist2c: 视野中心距小车的距离
        """
        assert 0 < view_wb < view_wt, "视野底边宽应小于顶边宽"
        assert view_h > 0, "视野高度应大于0"
        assert car_h > 0 and car_w > 0, "小车尺寸应大于0"

        self.car_w = car_w
        self.car_h = car_h
        self.view_wb = view_wb
        self.view_wt = view_wt
        self.view_h = view_h
        self.bdist2c = bdist2c
        self.maskw = maskw
        self.maskh = maskh
        self.track_map_origin = cv2.resize(cv2.imread(track_map_dir), tmsize) # 读取赛道图用作模板

        # 小车初始位姿
        """
        x, y: 小车质心位置。以左上角为原点，向右为x轴正方向，向下为y轴正方向
        theta: 小车朝向角度。角度单位取度，0度为x轴正方向，逆时针为正方向
        speed: 小车线速度，单位像素/帧
        omega: 小车角速度，单位度/帧
        """
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.track_map = self.track_map_origin.copy()  # 确保存在
        # 随机一个白色起点
        ys, xs = np.where(np.all(self.track_map_origin == [255, 255, 255], axis=-1))
        assert xs.size > 0, "赛道图中没有白色区域"
        i = random.randrange(xs.size)
        self.x, self.y = float(xs[i]), float(ys[i])  # 用 float，方便后续计算
        self.theta = float(random.uniform(0, 360))
        self.speed = 4
        self.omega = 0.0

        # 加载循迹算法库
        Row = ctypes.c_uint8 * self.maskw
        self.Image2D = ctypes.POINTER(Row)
        self.lib = ctypes.CDLL("asset/track.dll")
        self.steer_algo = self.lib.track # 选择循迹算法函数
        self.steer_algo.argtypes = [self.Image2D]
        self.steer_algo.restype  = ctypes.c_float

    def get_view(self):
        """
        获取小车视野
        maskw, maskh: 视野图像尺寸
        """
        # Get the center of both the bottom and the top of the view trapezoid
        b_center_x, b_center_y = self.x + self.bdist2c * np.cos(np.deg2rad(self.theta)), self.y + self.bdist2c * np.sin(np.deg2rad(self.theta))
        t_center_x, t_center_y = self.x + (self.bdist2c + self.view_h) * np.cos(np.deg2rad(self.theta)), self.y + (self.bdist2c + self.view_h) * np.sin(np.deg2rad(self.theta))
        # Get the four corners of the view trapezoid
        # left top, right top, right bottom, left bottom
        lt = (int(t_center_x - self.view_wt / 2 * np.sin(np.deg2rad(self.theta))), int(t_center_y + self.view_wt / 2 * np.cos(np.deg2rad(self.theta))))
        rt = (int(t_center_x + self.view_wt / 2 * np.sin(np.deg2rad(self.theta))), int(t_center_y - self.view_wt / 2 * np.cos(np.deg2rad(self.theta))))
        rb = (int(b_center_x + self.view_wb / 2 * np.sin(np.deg2rad(self.theta))), int(b_center_y - self.view_wb / 2 * np.cos(np.deg2rad(self.theta))))
        lb = (int(b_center_x - self.view_wb / 2 * np.sin(np.deg2rad(self.theta))), int(b_center_y + self.view_wb / 2 * np.cos(np.deg2rad(self.theta))))
        src_pts = np.array([lt, rt, rb, lb], dtype=np.float32)
        dst_pts = np.array([[0, 0], [self.maskw-1, 0], [self.maskw-1, self.maskh-1], [0, self.maskh-1]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        # 获取视域掩膜并转化为灰度图
        world = self.compose_frame()
        view_bgr = cv2.warpPerspective(world, M, (self.maskw, self.maskh))
        self.view = cv2.cvtColor(view_bgr, cv2.COLOR_BGR2GRAY)[:, ::-1]  # 左右翻转，符合人类视觉习惯
        # 如需可视化视锥，放到 GUI 里统一绘制

    def update_pos(self):
        """
        更新小车位置
        """
        self.theta = float((self.theta + self.omega) % 360.0)
        self.x += float(self.speed * np.cos(np.deg2rad(self.theta)))
        self.y += float(self.speed * np.sin(np.deg2rad(self.theta)))
        H, W, _ = self.track_map_origin.shape  # 用 origin 的尺寸即可
        assert 0 <= self.x <= W and 0 <= self.y <= H
        # 仅更新状态，不在这里画红色矩形；把绘制放 GUI 统一做
        # 如必须绘制，请在 GUI 线程按状态绘制到一个frame副本

    def steer(self) -> float:
        """
        根据视野图像计算转向角
        返回一个[-1, 1]之间的值，映射到实际前轮转角
        """
        assert hasattr(self, 'view'), "视野图像未获取，请先调用 get_view() 方法"
        img = np.ascontiguousarray(self.view, dtype=np.uint8) # 保证连续且 uint8
        ptr = img.ctypes.data_as(self.Image2D) # 转为 uint8 (*)[188]
        steer_val = self.steer_algo(ptr)
        return np.clip(steer_val, -1.0, 1.0)  # 做饱和处理，防止越界
    
    def srvout2omega(self, srvout: float, max_steer_angle=60):
        """
        由舵机输出换算到角速度（单位度/帧）
        srvout: 舵机输出，[-1, 1]之间的值。实际这里会做饱和处理
        max_steer_angle: 最大转角，单位度
        """
        steer_angle = - srvout * max_steer_angle
        self.omega = 180 / np.pi * self.speed * np.tan(np.deg2rad(steer_angle)) / self.car_h

    # ----- 线程循环：采样 / 运动 / GUI -----

    def camera_loop(self):
        while not self.stop.is_set():
            with self.lock:
                self.get_view()
            time.sleep(0.1)  # ≈ 10 FPS

    def motion_loop(self):
        while not self.stop.is_set():
            with self.lock:
                self.update_pos()
                if hasattr(self, 'view'):
                    srvout = self.steer()
                    self.srvout2omega(srvout)
            time.sleep(0.05)  # ≈ 20 FPS

    def gui_loop(self):
        wtitle = "Simulation"
        cv2.namedWindow(wtitle, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(wtitle, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        while not self.stop.is_set():
            with self.lock:
                # 基于模板制作一帧副本，再叠加可视元素，避免写回共享底图
                frame = self.track_map_origin.copy()
                # 画视锥（可选）：用最新的 x/y/theta
                b_center_x = self.x + self.bdist2c * np.cos(np.deg2rad(self.theta))
                b_center_y = self.y + self.bdist2c * np.sin(np.deg2rad(self.theta))
                t_center_x = self.x + (self.bdist2c + self.view_h) * np.cos(np.deg2rad(self.theta))
                t_center_y = self.y + (self.bdist2c + self.view_h) * np.sin(np.deg2rad(self.theta))
                lt = (int(t_center_x - self.view_wt/2*np.sin(np.deg2rad(self.theta))), int(t_center_y + self.view_wt/2*np.cos(np.deg2rad(self.theta))))
                rt = (int(t_center_x + self.view_wt/2*np.sin(np.deg2rad(self.theta))), int(t_center_y - self.view_wt/2*np.cos(np.deg2rad(self.theta))))
                rb = (int(b_center_x + self.view_wb/2*np.sin(np.deg2rad(self.theta))), int(b_center_y - self.view_wb/2*np.cos(np.deg2rad(self.theta))))
                lb = (int(b_center_x - self.view_wb/2*np.sin(np.deg2rad(self.theta))), int(b_center_y + self.view_wb/2*np.cos(np.deg2rad(self.theta))))
                # 再画视锥轮廓到 frame 上
                cv2.polylines(frame, [np.array([lt, rt, rb, lb], np.int32)], True, (0,255,0), 2)

                # 画小车（红色旋转矩形）
                rect = ((float(self.x), float(self.y)), (float(self.car_w), float(self.car_h)), float(self.theta))
                box = cv2.boxPoints(rect).astype(np.int32)
                cv2.drawContours(frame, [box], 0, (0,0,255), -1)

                # 缩略显示视野（可选）
                if hasattr(self, 'view'):
                    thumb = cv2.cvtColor(self.view, cv2.COLOR_GRAY2BGR)
                    h, w = thumb.shape[:2]
                    frame[0:h, 0:w] = thumb

            cv2.imshow(wtitle, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.stop.set()
                break

    def spin(self):
        try:
            t_cam = threading.Thread(target=self.camera_loop, daemon=True)
            t_motion = threading.Thread(target=self.motion_loop, daemon=True)
            t_cam.start()
            t_motion.start()
            self.gui_loop()      # GUI放在主线程
            self.stop.set()
            t_cam.join()
            t_motion.join()
        finally:
            cv2.destroyAllWindows()

    def compose_frame(self):
        frame = self.track_map_origin.copy()
        rect = ((float(self.x), float(self.y)), (float(self.car_w), float(self.car_h)), float(self.theta))
        box = cv2.boxPoints(rect).astype(np.int32)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), -1)
        return frame

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO) # 这里配置了日志输出级别为INFO，但是并没有加实际的日志输出
    wf = WayFinder(track_map_dir="asset/map.png")

    wf.spin()
