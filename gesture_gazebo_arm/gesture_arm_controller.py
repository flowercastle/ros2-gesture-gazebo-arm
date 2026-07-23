#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import time
from collections import deque

import cv2
import mediapipe as mp
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_joint",
    "left_finger_joint",
    "right_finger_joint",
]


JOINT_LIMITS = {
    "base_joint": (-3.14, 3.14),
    "shoulder_joint": (-1.2, 1.2),
    "elbow_joint": (-1.5, 1.5),
    "wrist_joint": (-1.5, 1.5),
    "left_finger_joint": (0.0, 0.7),
    "right_finger_joint": (-0.7, 0.0),
}


class GestureArmController(Node):
    def __init__(self):
        super().__init__("gesture_arm_controller")

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10
        )

        self.joint_pos = {
            "base_joint": 0.0,
            "shoulder_joint": 0.0,
            "elbow_joint": 0.0,
            "wrist_joint": 0.0,
            "left_finger_joint": 0.5,
            "right_finger_joint": -0.5,
        }

        self.step = 0.04
        self.last_publish_time = 0.0
        self.publish_interval = 0.12

        self.gesture_buffer = deque(maxlen=7)
        self.current_stable_gesture = "NONE"

        self.cn_font = self.load_chinese_font(24)

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=0.65,
            min_tracking_confidence=0.65
        )

        # 使用英文窗口名，避免部分OpenCV/Wayland环境下中文标题导致窗口异常。
        self.window_name = "Gesture Gazebo Arm Controller"

    def load_chinese_font(self, size=24):
        font_paths = [
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        ]

        for path in font_paths:
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                pass

        return ImageFont.load_default()

    def open_camera(self):
        """
        自动扫描摄像头。
        Ubuntu下经常出现/dev/video0不是实际画面设备的情况，
        所以不能只固定用VideoCapture(0)。
        """

        candidates = []

        for i in range(8):
            if os.path.exists(f"/dev/video{i}"):
                candidates.append(i)

        # 如果/dev/video*没查到，也尝试0~5。
        if not candidates:
            candidates = list(range(6))

        self.get_logger().info(f"开始扫描摄像头编号：{candidates}")

        for index in candidates:
            self.get_logger().info(f"尝试打开摄像头 /dev/video{index}")

            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)

            if not cap.isOpened():
                cap.release()
                self.get_logger().warning(f"/dev/video{index}无法打开")
                continue

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            ok = False
            frame = None

            # 多读几帧，跳过摄像头初始化阶段的空帧。
            for _ in range(10):
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    ok = True
                    break
                time.sleep(0.05)

            if ok:
                h, w = frame.shape[:2]
                self.get_logger().info(f"成功打开摄像头 /dev/video{index}，画面尺寸：{w}x{h}")
                return cap, index

            cap.release()
            self.get_logger().warning(f"/dev/video{index}可以打开，但读不到有效画面")

        return None, None

    def clamp(self, joint, value):
        low, high = JOINT_LIMITS[joint]
        return max(low, min(high, value))

    def set_home(self):
        self.joint_pos["base_joint"] = 0.0
        self.joint_pos["shoulder_joint"] = 0.0
        self.joint_pos["elbow_joint"] = 0.0
        self.joint_pos["wrist_joint"] = 0.0
        self.open_gripper()

    def open_gripper(self):
        self.joint_pos["left_finger_joint"] = 0.5
        self.joint_pos["right_finger_joint"] = -0.5

    def close_gripper(self):
        self.joint_pos["left_finger_joint"] = 0.05
        self.joint_pos["right_finger_joint"] = -0.05

    def publish_current_pose(self):
        now = time.time()

        if now - self.last_publish_time < self.publish_interval:
            return

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [self.joint_pos[name] for name in JOINT_NAMES]
        point.time_from_start = Duration(sec=0, nanosec=250_000_000)

        msg.points.append(point)
        self.publisher.publish(msg)

        self.last_publish_time = now

    def landmark_dist(self, a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def is_finger_extended(self, lm, tip_id, pip_id, mcp_id):
        """
        方向无关的手指伸直判断。

        之前使用tip.y < pip.y判断伸直，只适合手指向上。
        当食指向下时，tip.y会大于pip.y，因此会被误判为握拳。
        这里改为根据“指尖是否明显远离手腕/掌心”判断，
        所以向上、向下、向左、向右都能识别为伸出。
        """

        wrist = lm[0]
        middle_mcp = lm[9]

        palm_size = self.landmark_dist(wrist, middle_mcp)
        if palm_size < 1e-6:
            return False

        tip = lm[tip_id]
        pip = lm[pip_id]
        mcp = lm[mcp_id]

        d_tip = self.landmark_dist(wrist, tip)
        d_pip = self.landmark_dist(wrist, pip)
        d_mcp = self.landmark_dist(wrist, mcp)

        # 伸直时，指尖应该比中间关节、掌指关节更远离手腕。
        far_from_wrist = (
            d_tip > d_pip + 0.10 * palm_size and
            d_tip > d_mcp + 0.22 * palm_size
        )

        # 再加入一个弱约束：指尖不能贴近掌心。
        not_near_palm = d_tip > 0.70 * palm_size

        return far_from_wrist and not_near_palm

    def get_non_thumb_states(self, lm):
        """
        返回食指、中指、无名指、小指是否伸直。
        这个版本不再依赖y坐标方向，因此支持食指向下。
        """

        states = {}

        finger_info = {
            "index": (8, 6, 5),
            "middle": (12, 10, 9),
            "ring": (16, 14, 13),
            "pinky": (20, 18, 17),
        }

        for name, (tip_id, pip_id, mcp_id) in finger_info.items():
            states[name] = self.is_finger_extended(lm, tip_id, pip_id, mcp_id)

        return states

    def is_ok_gesture(self, lm, states):
        thumb_tip = lm[4]
        index_tip = lm[8]
        wrist = lm[0]
        middle_mcp = lm[9]

        palm_size = self.landmark_dist(wrist, middle_mcp)
        if palm_size < 1e-6:
            return False

        thumb_index_dist = self.landmark_dist(thumb_tip, index_tip)
        ratio = thumb_index_dist / palm_size

        other_extended = int(states["middle"]) + int(states["ring"]) + int(states["pinky"])

        return ratio < 0.35 and other_extended >= 2

    def get_index_direction(self, lm):
        wrist = lm[0]
        index_tip = lm[8]

        dx = index_tip.x - wrist.x
        dy = index_tip.y - wrist.y

        if abs(dx) < 0.08 and abs(dy) < 0.08:
            return "UNKNOWN"

        if abs(dx) > abs(dy):
            if dx < 0:
                return "POINT_LEFT"
            return "POINT_RIGHT"

        if dy < 0:
            return "POINT_UP"

        return "POINT_DOWN"

    def raw_gesture_from_landmarks(self, lm):
        states = self.get_non_thumb_states(lm)

        index = states["index"]
        middle = states["middle"]
        ring = states["ring"]
        pinky = states["pinky"]

        non_thumb_count = int(index) + int(middle) + int(ring) + int(pinky)

        if non_thumb_count == 0:
            return "FIST_STOP"

        if self.is_ok_gesture(lm, states):
            return "OK_CLOSE"

        if non_thumb_count == 4:
            return "PALM_HOME"

        if index and not middle and not ring and not pinky:
            return self.get_index_direction(lm)

        if index and middle and not ring and not pinky:
            return "TWO_ELBOW_UP"

        if index and middle and ring and not pinky:
            return "THREE_ELBOW_DOWN"

        return "UNKNOWN"

    def stabilize_gesture(self, raw_gesture):
        if raw_gesture in ["UNKNOWN", "NO_HAND"]:
            return self.current_stable_gesture

        self.gesture_buffer.append(raw_gesture)

        counts = {}
        for gesture in self.gesture_buffer:
            counts[gesture] = counts.get(gesture, 0) + 1

        stable = max(counts, key=counts.get)

        if counts[stable] >= 4:
            self.current_stable_gesture = stable

        return self.current_stable_gesture

    def apply_gesture(self, gesture):
        if gesture == "FIST_STOP":
            return

        if gesture == "PALM_HOME":
            self.set_home()

        elif gesture == "POINT_LEFT":
            self.joint_pos["base_joint"] -= self.step

        elif gesture == "POINT_RIGHT":
            self.joint_pos["base_joint"] += self.step

        elif gesture == "POINT_UP":
            # Gazebo中该关节正方向与视觉上抬方向相反，因此上抬使用负方向
            self.joint_pos["shoulder_joint"] -= self.step

        elif gesture == "POINT_DOWN":
            # Gazebo中该关节负方向与视觉下压方向相反，因此下压使用正方向
            self.joint_pos["shoulder_joint"] += self.step

        elif gesture == "TWO_ELBOW_UP":
            # 小臂上抬方向同样需要反向
            self.joint_pos["elbow_joint"] -= self.step

        elif gesture == "THREE_ELBOW_DOWN":
            # 小臂下压方向同样需要反向
            self.joint_pos["elbow_joint"] += self.step

        elif gesture == "OK_CLOSE":
            self.close_gripper()

        for joint_name in self.joint_pos:
            self.joint_pos[joint_name] = self.clamp(
                joint_name,
                self.joint_pos[joint_name]
            )

    def gesture_to_chinese(self, gesture):
        mapping = {
            "NONE": "无稳定手势",
            "NO_HAND": "未检测到手",
            "UNKNOWN": "未知手势",
            "FIST_STOP": "握拳：停止",
            "PALM_HOME": "张开手掌：回到初始姿态",
            "POINT_LEFT": "单指向左：底座左转",
            "POINT_RIGHT": "单指向右：底座右转",
            "POINT_UP": "单指向上：大臂上抬",
            "POINT_DOWN": "单指向下：大臂下压",
            "TWO_ELBOW_UP": "两指：小臂上抬",
            "THREE_ELBOW_DOWN": "三指：小臂下压",
            "OK_CLOSE": "OK手势：夹爪闭合",
        }
        return mapping.get(gesture, gesture)

    def draw_chinese_text(self, frame, lines):
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_pil = Image.fromarray(image_rgb)
        draw = ImageDraw.Draw(image_pil)

        x = 20
        y = 25

        for line in lines:
            draw.text((x, y), line, font=self.cn_font, fill=(0, 255, 0))
            y += 34

        return cv2.cvtColor(np.array(image_pil), cv2.COLOR_RGB2BGR)

    def draw_info(self, frame, raw_gesture, stable_gesture, camera_index):
        lines = [
            f"当前摄像头：/dev/video{camera_index}",
            f"当前识别：{self.gesture_to_chinese(raw_gesture)}",
            f"稳定指令：{self.gesture_to_chinese(stable_gesture)}",
            "操作说明：",
            "握拳：停止",
            "张开手掌：复位",
            "单指上下左右：控制底座/大臂",
            "两指：小臂上抬",
            "三指：小臂下压",
            "OK：夹爪闭合",
            "按q退出",
        ]

        return self.draw_chinese_text(frame, lines)

    def run(self):
        cap, camera_index = self.open_camera()

        if cap is None:
            self.get_logger().error("没有找到可用摄像头。请执行：ls /dev/video* 或 v4l2-ctl --list-devices")
            return

        # 显式创建窗口，部分Ubuntu/Wayland环境下更稳。
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 960, 720)

        self.get_logger().info("手势控制节点已启动，摄像头窗口应已弹出。")

        consecutive_failures = 0

        while rclpy.ok():
            ret, frame = cap.read()

            if not ret or frame is None or frame.size == 0:
                consecutive_failures += 1

                if consecutive_failures % 30 == 0:
                    self.get_logger().warning(
                        f"连续{consecutive_failures}次读取摄像头失败，当前设备：/dev/video{camera_index}"
                    )

                rclpy.spin_once(self, timeout_sec=0.001)
                time.sleep(0.01)
                continue

            consecutive_failures = 0

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            result = self.hands.process(rgb)

            raw_gesture = "NO_HAND"
            stable_gesture = self.current_stable_gesture

            if result.multi_hand_landmarks:
                hand_landmarks = result.multi_hand_landmarks[0]
                lm = hand_landmarks.landmark

                self.mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS
                )

                raw_gesture = self.raw_gesture_from_landmarks(lm)
                stable_gesture = self.stabilize_gesture(raw_gesture)

                self.apply_gesture(stable_gesture)
                self.publish_current_pose()

            frame = self.draw_info(frame, raw_gesture, stable_gesture, camera_index)

            cv2.imshow(self.window_name, frame)

            rclpy.spin_once(self, timeout_sec=0.001)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            # 如果用户手动关闭窗口，也退出节点。
            if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                break

        cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    node = GestureArmController()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
