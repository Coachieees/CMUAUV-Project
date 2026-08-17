import sys
import os
import csv
import time
from datetime import datetime
from collections import deque
import cv2
import numpy as np
import math
import json
from sensor_msgs.msg import CompressedImage, Joy

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import CompressedImage

# PyQt5 & PyQtGraph Imports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QSlider, QPushButton, QFrame, 
                             QGridLayout, QProgressBar, QComboBox, QSizePolicy,
                             QTabWidget, QDoubleSpinBox, QStackedWidget, QLineEdit, QScrollArea)
from PyQt5.QtCore import Qt, pyqtSignal, QThread, QTimer, QSize, QPointF, QRectF
from PyQt5.QtGui import QColor, QPalette, QImage, QPixmap, QIcon, QPainter, QPolygonF, QPen, QBrush, QFont
import pyqtgraph as pg

# --- THEME CONFIGURATION ---
BG_MAIN = "#0A0A0A"
BG_CARD = "#1D1D1F"
BG_CARD2 = "#141416"
ACCENT_BLUE = "#0A84FF"
ACCENT_RED = "#FF453A"
ACCENT_GREEN = "#32D74B"
ACCENT_ORANGE = "#FF9F0A"
ACCENT_BLUE_HOV_L = "#369AFE"
ACCENT_RED_HOVER_L = "#FF635B"
ACCENT_GREEN_HOVER_L = "#55D969"
ACCENT_ORANGE_HOVER_L = "#FFB64A"
ACCENT_BLUE_HOVER_D = "#096ACB"
ACCENT_RED_HOVER_D = "#DB352C"
ACCENT_GREEN_HOVER_D = "#21B037"
ACCENT_ORANGE_HOVER_D = "#D28511"
TEXT_WHITE = "#F5F5F7"
TEXT_DIM = "#8E8E93"
FONT_MAIN = "Arial"

class CompassWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.heading = 0.0
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
    def set_heading(self, angle):
        self.heading = angle
        self.update() 
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        side = min(self.width(), self.height())
        painter.translate(self.width() / 2.0, self.height() / 2.0)
        
        scale = side / 200.0
        painter.scale(scale, scale)
        
        painter.setPen(QPen(QColor("#333333"), 3))
        painter.setBrush(QBrush(QColor("#1C1C1E")))
        painter.drawEllipse(QPointF(0, 0), 97, 97)
        
        for i in range(36):
            if i % 9 == 0:
                painter.setPen(QPen(QColor(TEXT_WHITE), 3))
                painter.drawLine(0, -97, 0, -82)
            else:
                painter.setPen(QPen(QColor(TEXT_DIM), 2))
                painter.drawLine(0, -97, 0, -88)
            painter.rotate(10)
        
        painter.setPen(QColor(TEXT_WHITE))
        painter.setFont(QFont("Arial", 16, QFont.Bold))
        painter.drawText(QRectF(-20, -78, 40, 30), Qt.AlignCenter, "N")
        painter.drawText(QRectF(-20, 48, 40, 30), Qt.AlignCenter, "S")
        painter.drawText(QRectF(48, -15, 30, 30), Qt.AlignCenter, "E")
        painter.drawText(QRectF(-78, -15, 30, 30), Qt.AlignCenter, "W")
        
        painter.rotate(self.heading)
        
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(ACCENT_RED)))
        poly_north = QPolygonF([QPointF(-7, 0), QPointF(7, 0), QPointF(0, -60)])
        painter.drawPolygon(poly_north)
        
        painter.setBrush(QBrush(QColor(TEXT_DIM)))
        poly_south = QPolygonF([QPointF(-7, 0), QPointF(7, 0), QPointF(0, 60)])
        painter.drawPolygon(poly_south)
        
        painter.setBrush(QBrush(QColor(TEXT_WHITE)))
        painter.drawEllipse(QPointF(0, 0), 6, 6)

# ==========================================
# 1. ROS 2 Worker Thread
# ==========================================
class ROS2Thread(QThread):
    telemetry_signal = pyqtSignal(str)
    camera_signal = pyqtSignal(object)
    joy_signal = pyqtSignal(object)
    pwm_feedback_signal = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.node = None
        self.pub_pwm = None
        self.pub_cam_ctrl = None 
        self.pub_setpoint = None 
        self.pub_sys = None 
        self.pub_gain = None 

    def run(self):
        rclpy.init()
        self.node = rclpy.create_node('auv_qt_dashboard')
        self.pub_pwm = self.node.create_publisher(Int32MultiArray, '/auv/cmd_pwm', 10)
        self.pub_cam_ctrl = self.node.create_publisher(String, '/auv/cmd_cam', 10)
        self.pub_setpoint = self.node.create_publisher(String, '/auv/setpoint', 10) 
        self.pub_sys = self.node.create_publisher(String, '/auv/cmd_sys', 10) 
        self.pub_gain = self.node.create_publisher(String, '/auv/tune_gains', 10)
        
        self.node.create_subscription(String, '/auv/sensors', self.sensor_cb, 10)
        self.node.create_subscription(CompressedImage, '/auv/camera/image/compressed', self.camera_cb, 10)
        self.node.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.node.create_subscription(Int32MultiArray, '/auv/cmd_pwm', self.pwm_feedback_cb, 10)
        
        rclpy.spin(self.node)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    
    def joy_cb(self, msg):
        self.joy_signal.emit(msg)

    def sensor_cb(self, msg):
        self.telemetry_signal.emit(msg.data)

    def camera_cb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_img is not None:
            self.camera_signal.emit(cv_img)

    def pwm_feedback_cb(self, msg):
        self.pwm_feedback_signal.emit(list(msg.data))

    def send_pwm_command(self, pwm_list):
        if self.pub_pwm:
            msg = Int32MultiArray()
            msg.data = [int(x) for x in pwm_list]
            self.pub_pwm.publish(msg)

    def send_cam_command(self, cmd_str: str):
        if self.pub_cam_ctrl:
            msg = String()
            msg.data = cmd_str
            self.pub_cam_ctrl.publish(msg)

    def send_setpoint(self, mode, z=0.0, r=0.0, p=0.0, y=0.0):
        if self.pub_setpoint:
            msg = String()
            if mode == "AUTO":
                msg.data = f"AUTO,{z:.2f},{r:.2f},{p:.2f},{y:.2f}"
            else:
                msg.data = "MANUAL"
            self.pub_setpoint.publish(msg)

    def send_sys_command(self, cmd_str: str):
        if self.pub_sys:
            msg = String()
            msg.data = cmd_str
            self.pub_sys.publish(msg)

    def send_gain_command(self, cmd_str: str):
        if self.pub_gain:
            msg = String()
            msg.data = cmd_str
            self.pub_gain.publish(msg)

    def stop(self):
        if rclpy.ok():
            rclpy.shutdown()
        self.wait()

# ==========================================
# 2. Main GUI Window
# ==========================================
class ModernDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CMUAUV - Mission Control")
        
        icon_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "auv_icon.png")
        self.setWindowIcon(QIcon(icon_path))
        
        self.setStyleSheet(f"""
            QMainWindow {{ background-color: {BG_MAIN}; }}
            QWidget {{ font-family: '{FONT_MAIN}'; color: {TEXT_WHITE}; }}
            QFrame#Card {{ background-color: {BG_CARD}; border-radius: 12px;}}
            QLabel#CardTitle {{ color: {TEXT_DIM}; font-size: 16px; font-weight: bold; letter-spacing: 1px; }}
            QLabel#Value {{ font-size: 24px; font-weight: bold; color: {TEXT_WHITE}; }}
            QLabel#Unit {{ font-size: 14px; color: {TEXT_DIM}; }}
            QPushButton {{ background-color: #3A3A3C; border-radius: 6px; padding: 10px; font-weight: bold; }}
            QPushButton:hover {{ background-color: #505052; }}
        """)

        self.is_logging = False
        self.log_file = None
        self.csv_writer = None
        
        self.latest_frame = None
        self.is_video_recording = False
        self.video_writer = None
        self.is_streaming = False 
        
        self.is_fullscreen_mode = False

        self.last_heartbeat = time.time()
        self.is_signal_lost = False
        self.is_critical_batt = False
        self.low_batt_msg = ""
        self.is_overheat = False
        self.is_manual_control = False
        self.is_joy_connected = False
        self.last_joy_time = time.time()
        
        self.is_auto_mode = False
        self.target_sp = {'depth': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        
        self.is_ramp_mode = True 
        self.target_osd = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0, 'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0}
        self.current_osd = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0, 'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0}
        self.was_osd_active = False

        self.depth_offset = 0.0
        self.data_labels = {} 
        self.sp_spinboxes = {} 
        self.auto_current_labels = {} 
        
        self.start_time = time.time()
        self.hist_size = 100
        self.t_data = deque(maxlen=self.hist_size)
        self.r_data = deque(maxlen=self.hist_size); self.p_data = deque(maxlen=self.hist_size); self.y_data = deque(maxlen=self.hist_size)
        self.d_data = deque(maxlen=self.hist_size)
        self.lx_data = deque(maxlen=self.hist_size); self.ly_data = deque(maxlen=self.hist_size); self.lz_data = deque(maxlen=self.hist_size)
        self.gx_data = deque(maxlen=self.hist_size); self.gy_data = deque(maxlen=self.hist_size); self.gz_data = deque(maxlen=self.hist_size)

        self.gain_config_file = os.path.expanduser("~/auv_data/gains_config.json")
        self.default_gains = {
            "q_vel": "50.0, 10.0, 200.0, 10.0, 50.0, 50.0",
            "q_pos": "100.0, 100.0, 100.0, 50.0, 50.0, 200.0",
            "q_i": "1.0",
            "r": "0.1, 0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.05",
            "qn": "0.01",
            "rn": "0.02"
        }
        if os.path.exists(self.gain_config_file):
            try:
                with open(self.gain_config_file, 'r') as f:
                    self.default_gains.update(json.load(f))
            except Exception: pass

        self.init_ui()

        self.ros_thread = ROS2Thread()
        self.ros_thread.telemetry_signal.connect(self.update_telemetry)
        self.ros_thread.camera_signal.connect(self.update_image)
        self.ros_thread.joy_signal.connect(self.process_joystick)
        self.ros_thread.pwm_feedback_signal.connect(self.update_sliders_from_feedback) 
        self.ros_thread.start() 
        
        self.watchdog = QTimer()
        self.watchdog.timeout.connect(self.check_connection)
        self.watchdog.start(1000) 
        
        self.osd_timer = QTimer()
        self.osd_timer.timeout.connect(self.update_osd_control)
        self.osd_timer.start(50) 

    def init_ui(self):
        self.central_stack = QStackedWidget()
        self.setCentralWidget(self.central_stack)

        self.page_normal = QWidget()
        main_layout = QHBoxLayout(self.page_normal) 
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)

        self.left_layout = QVBoxLayout()
        self.left_layout.setSpacing(10)
        
        right_layout = QVBoxLayout()
        right_layout.setSpacing(10)

        # --- CAMERA CARD ---
        self.cam_card = self.create_card()
        cam_layout = QVBoxLayout(self.cam_card)
        cam_layout.setContentsMargins(5, 5, 5, 5)
        
        # 1. สร้างหน้าจอกล้อง
        self.video_label = QLabel("CAMERA OFFLINE")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet(f"background-color: #000; border-radius: 8px; color: {TEXT_DIM}; font-size: 18px;")
        self.video_label.setMinimumSize(320, 180)
        cam_layout.addWidget(self.video_label, stretch=1)
        
        # 2. แถบปุ่มควบคุมกล้อง
        cam_bottom = QHBoxLayout()
        cam_bottom.setContentsMargins(5, 0, 5, 0)

        icon_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui_icon")
        
        self.btn_stream = QPushButton(" START STREAM")
        self.btn_stream.setIcon(QIcon(os.path.join(icon_dir, "play.png"))) 
        self.btn_stream.setIconSize(QSize(14, 14)) 
        self.btn_stream.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_GREEN}; color: #121212; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: #3BEA55; }}")
        self.btn_stream.clicked.connect(self.toggle_stream)
        
        self.combo_res = QComboBox()
        self.combo_res.addItems(["360p (640x360)", "720p (1280x720)", "1080p (1920x1080)", "2K (2560x1440)"])
        self.combo_res.setCurrentIndex(1) 
        self.combo_res.setStyleSheet("""
            QComboBox {
                background-color: #3A3A3C;
                color: #F5F5F7;
                border-radius: 12px; 
                padding: 4px 15px;
                font-size: 14px;
                font-weight: bold;
                border: none;
            }
            QComboBox::drop-down {
                subcontrol-origin: padding;
                subcontrol-position: top right;
                width: 25px;
                border: none; 
                background-color: transparent; 
            }
            QComboBox QAbstractItemView {
                background-color: #2C2C2E;
                color: #F5F5F7;
                border-radius: 8px;
                border: 1px solid #444;
                selection-background-color: #0A84FF; 
                outline: none; 
            }
        """)

        self.btn_snap = QPushButton(" CAPTURE")
        self.btn_snap.setIcon(QIcon(os.path.join(icon_dir, "capture.png")))
        self.btn_snap.setIconSize(QSize(15, 15))
        self.btn_snap.setStyleSheet(f"QPushButton {{ background-color: #F5F5F7; color: #121212; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: #CCCCCC; }}")
        self.btn_snap.clicked.connect(self.capture_image)
        
        self.btn_rec = QPushButton(" RECORD")
        self.btn_rec.setIcon(QIcon(os.path.join(icon_dir, "record.png")))
        self.btn_rec.setIconSize(QSize(14, 14))
        self.btn_rec.setStyleSheet(f"QPushButton {{ background-color: #F5F5F7; color: #121212; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: #CCCCCC; }}")
        self.btn_rec.clicked.connect(self.toggle_video_recording)

        self.btn_full = QPushButton(" FULL SCREEN")
        self.btn_full.setIcon(QIcon(os.path.join(icon_dir, "fullscreen.png")))
        self.btn_full.setIconSize(QSize(16, 16))
        self.btn_full.setStyleSheet(f"QPushButton {{ background-color: #3A3A3C; color: {TEXT_WHITE}; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: #505052; }}")
        self.btn_full.clicked.connect(self.toggle_fullscreen_cam)

        cam_bottom.addWidget(self.btn_stream)
        cam_bottom.addWidget(self.combo_res)
        cam_bottom.addStretch() 
        cam_bottom.addWidget(self.btn_snap)
        cam_bottom.addWidget(self.btn_rec)
        cam_bottom.addWidget(self.btn_full) 
        
        cam_layout.addLayout(cam_bottom)

        # 🟢 --- TOP RIGHT (MISSION CONTROL + BATTERY + COMPASS) ---
        # 1. Mission Control Card (Log, Restart, Power, E-Stop)
        # --- MISSION CONTROL CARD ---
        mission_card = self.create_card()
        mission_layout = QHBoxLayout(mission_card) # 🟢 ใช้เลย์เอาต์แนวนอนเป็นแกนหลัก
        mission_layout.setContentsMargins(10, 10, 10, 10)

        left_vbox = QVBoxLayout() # 🟢 กล่องซ้ายเก็บปุ่มย่อยและข้อความสถานะ

        btn_layout = QHBoxLayout()
        self.btn_log = QPushButton(" START LOGGING")
        self.btn_log.setIcon(QIcon(os.path.join(icon_dir, "record.png")))
        self.btn_log.setIconSize(QSize(14, 14))
        self.btn_log.setStyleSheet(f"QPushButton {{ background-color: #E5E5EA; color: #121212; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: #CCCCCC; }}")
        self.btn_log.clicked.connect(self.toggle_logging)

        self.btn_restart = QPushButton()
        self.btn_restart.setIcon(QIcon(os.path.join(icon_dir, "restart.png")))
        self.btn_restart.setIconSize(QSize(26, 26))
        self.btn_restart.setFixedSize(36, 36)
        self.btn_restart.setStyleSheet(f"QPushButton {{ background-color: #1A1A1A; border-radius: 18px; }} QPushButton:hover {{ background-color: #646468; }}")
        self.btn_restart.clicked.connect(self.restart_program)

        self.btn_power = QPushButton()
        self.btn_power.setIcon(QIcon(os.path.join(icon_dir, "power.png")))
        self.btn_power.setIconSize(QSize(26, 26))
        self.btn_power.setFixedSize(36, 36)
        self.btn_power.setStyleSheet(f"QPushButton {{ background-color: #1A1A1A; border-radius: 18px; }} QPushButton:hover {{ background-color: #646468; }}")
        self.btn_power.clicked.connect(self.close)

        btn_layout.addWidget(self.btn_log)
        btn_layout.addStretch()
        btn_layout.addWidget(self.btn_restart)
        btn_layout.addWidget(self.btn_power)

        status_layout = QHBoxLayout()
        self.status_lbl = QLabel("STATUS DETAILS")
        self.status_lbl.setStyleSheet(f"color: {TEXT_DIM}; font-weight: bold; font-size: 16px; text-transform: uppercase;")
        self.status_lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        status_layout.addWidget(self.status_lbl)
        status_layout.addStretch()

        left_vbox.addLayout(btn_layout)
        left_vbox.addLayout(status_layout)

        self.btn_stop = QPushButton()
        self.btn_stop.setIcon(QIcon(os.path.join(icon_dir, "emrstop.png")))
        self.btn_stop.setIconSize(QSize(55, 55))
        self.btn_stop.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding) # 🟢 ให้ยืดความสูงเต็มพื้นที่
        self.btn_stop.setFixedWidth(65) # 🟢 ล็อกความกว้างไว้
        self.btn_stop.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_ORANGE}; border-radius: 8px; }} QPushButton:hover {{ background-color: {ACCENT_ORANGE_HOVER_D}; }}")
        self.btn_stop.clicked.connect(self.emergency_stop)

        # 🟢 ประกอบฝั่งซ้ายและขวาเข้าด้วยกัน
        mission_layout.addLayout(left_vbox)
        mission_layout.addSpacing(10)
        mission_layout.addWidget(self.btn_stop)

        # 2. Battery & Temp Row (ดีไซน์ใหม่ ไม่มีหลอด บาร์)
        self.row2_layout = QHBoxLayout()
        self.row2_layout.setSpacing(10)

        def create_batt_card(name, v_key, pct_key):
            card = self.create_card()
            layout = QGridLayout(card)
            layout.setContentsMargins(15, 5, 15, 5)
            layout.setHorizontalSpacing(5)

            # 🟢 ดึงรูป battery.png จากโฟลเดอร์มาแสดงผล
            icon_lbl = QLabel()
            icon_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui_icon", "battery.png")
            icon_lbl.setPixmap(QPixmap(icon_path).scaled(40, 40, Qt.KeepAspectRatio, Qt.SmoothTransformation))

            v_lbl = QLabel("0.00 V")
            v_lbl.setStyleSheet("color: #8E8E93; font-weight: bold; font-size: 18px;")
            self.data_labels[v_key] = v_lbl

            name_lbl = QLabel(name)
            name_lbl.setStyleSheet("color: #8E8E93; font-weight: bold; font-size: 28px;")

            pct_lbl = QLabel("100%")
            pct_lbl.setStyleSheet(f"color: {ACCENT_GREEN}; font-weight: bold; font-size: 30px;")
            self.data_labels[pct_key] = pct_lbl

            layout.addWidget(icon_lbl, 0, 0)
            layout.addWidget(v_lbl, 0, 1, Qt.AlignRight)
            layout.addWidget(name_lbl, 1, 0)
            layout.addWidget(pct_lbl, 1, 1, Qt.AlignRight)
            return card

        self.ctr_batt_card = create_batt_card("CTR", "Hull Batt", "Hull %")
        self.thr_batt_card = create_batt_card("THR", "Thruster Batt", "Thruster %")

        temp_card = self.create_card()
        temp_layout = QVBoxLayout(temp_card)
        temp_layout.setContentsMargins(15, 10, 15, 10)
        temp_title = QLabel("TEMP")
        temp_title.setStyleSheet("color: #8E8E93; font-weight: bold; font-size: 16px;")
        temp_title.setAlignment(Qt.AlignCenter)
        temp_val = QLabel("25.0°C")
        temp_val.setStyleSheet("color: white; font-weight: bold; font-size: 26px;")
        temp_val.setAlignment(Qt.AlignCenter)
        self.data_labels["Temp"] = temp_val
        temp_layout.addWidget(temp_title)
        temp_layout.addWidget(temp_val)

        self.row2_layout.addWidget(self.ctr_batt_card, stretch=2)
        self.row2_layout.addWidget(self.thr_batt_card, stretch=2)
        self.row2_layout.addWidget(temp_card, stretch=1)

        # 3. Compass Card
        compass_card = self.create_card()
        compass_card.setFixedWidth(150) 
        
        compass_layout = QVBoxLayout(compass_card)
        compass_layout.setContentsMargins(5, 5, 5, 5)
        # (ลบ compass_layout.setSpacing(5) ออกด้วยก็ได้ครับ เพราะไม่มีปุ่มแล้ว)
        
        self.compass_widget = CompassWidget() 
        compass_layout.addWidget(self.compass_widget)
        
        # ประกอบ Top Right รวมกัน
        top_right_h = QHBoxLayout()
        top_right_h.setSpacing(10)
        
        top_right_v = QVBoxLayout()
        top_right_v.setSpacing(10)
        top_right_v.addWidget(mission_card)
        top_right_v.addLayout(self.row2_layout)

        top_right_h.addLayout(top_right_v, stretch=3)
        top_right_h.addWidget(compass_card, stretch=0) # stretch=0 เพื่อไม่ให้เสียรูปทรงจัตุรัส

        # --- THRUSTER SIGNAL CARD ---
        sig_card = self.create_card()
        sig_layout = QVBoxLayout(sig_card)
        sig_title = QLabel("THRUSTER SIGNAL")
        sig_title.setObjectName("CardTitle")
        sig_layout.addWidget(sig_title)
        
        sliders_grid = QGridLayout()
        sliders_grid.setVerticalSpacing(8)
        self.sliders = []
        
        for i in range(8):
            lbl_name = QLabel(f"T{i+1}")
            lbl_name.setStyleSheet("font-weight: bold; color: #8E8E93; font-size: 14px;")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(1000, 2000)
            slider.setValue(1500)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(250)
            slider.setStyleSheet(f"QSlider::groove:horizontal {{ border-radius: 4px; height: 6px; background: #2C2C2E; }} QSlider::handle:horizontal {{ background: {ACCENT_GREEN}; width: 25px; height: 20px; margin: -5px 0; border-radius: 8px; }}")
            lbl_val = QLabel("1500")
            lbl_val.setStyleSheet("background-color: #2C2C2E; color: #F5F5F7; padding: 4px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
            lbl_val.setFixedWidth(55)
            lbl_val.setAlignment(Qt.AlignCenter)
            
            btn_reset = QPushButton("⟲")
            btn_reset.setFixedSize(26, 26)
            btn_reset.setStyleSheet("QPushButton { background-color: #3A3A3C; border-radius: 13px; font-size: 14px; padding: 0px; } QPushButton:hover { background-color: #505052; }")
            
            slider.valueChanged.connect(lambda val, l=lbl_val, idx=i: self.on_slider_change(idx, val, l))
            btn_reset.clicked.connect(lambda checked, s=slider: s.setValue(1500))
            
            col_offset = (i // 4) * 4
            row = (i % 4)
            sliders_grid.addWidget(lbl_name, row, col_offset)
            sliders_grid.addWidget(slider, row, col_offset + 1)
            sliders_grid.addWidget(lbl_val, row, col_offset + 2)
            sliders_grid.addWidget(btn_reset, row, col_offset + 3)
            self.sliders.append((slider, lbl_val))
            
        sig_layout.addLayout(sliders_grid)

        # --- THRUSTER CONTROL CARD (Tabs) ---
        ctrl_card = self.create_card()
        ctrl_layout = QVBoxLayout(ctrl_card)
        
        self.tabs = QTabWidget()
        
        def update_tab_style(index):
            sel_bg = ACCENT_BLUE if index == 0 else ACCENT_ORANGE
            sel_text = "white" if index == 0 else "#121212"
            
            self.tabs.setStyleSheet(f"""
                QTabWidget::pane {{ border: 1px solid #333; border-radius: 4px; background: {BG_CARD}; top: -1px; }} 
                QTabBar::tab {{ 
                    background: #2C2C2E; 
                    color: #8E8E93; 
                    padding: 8px 25px; /* 🟢 เพิ่ม padding ซ้าย-ขวา เป็น 25px */
                    min-width: 140px;  /* 🟢 บังคับความกว้างขั้นต่ำไม่ให้ตัวหนังสือตกขอบ */
                    border-top-left-radius: 4px; 
                    border-top-right-radius: 4px; 
                    margin-right: 2px; 
                    font-weight: bold; 
                    font-size: 16px; 
                }} 
                QTabBar::tab:selected {{ background: {sel_bg}; color: {sel_text}; }}
            """)

        self.tabs.currentChanged.connect(update_tab_style)
        update_tab_style(0)

        # --- Tab 1: Manual ---
        tab_man = QWidget()
        man_layout = QVBoxLayout(tab_man)
        man_layout.setContentsMargins(0, 0, 0, 0)
        
        scroll_man = QScrollArea()
        scroll_man.setWidgetResizable(True)
        scroll_man.setStyleSheet("QScrollArea { border: none; background: transparent; } QScrollBar:vertical { background: #1C1C1E; width: 12px; } QScrollBar::handle:vertical { background: #3A3A3C; border-radius: 6px; }")
        
        content_man = QWidget()
        content_man.setStyleSheet("background: transparent;")
        m_lay = QVBoxLayout(content_man)
        m_lay.setContentsMargins(15, 15, 15, 15)
        m_lay.setSpacing(15)

        self.btn_manual = QPushButton("MANUAL MODE : OFF")
        self.btn_manual.setCheckable(True)
        self.btn_manual.setStyleSheet(f"QPushButton {{ background-color: #3A3A3C; color: white; padding: 10px; border-radius: 12px; font-size: 16px; font-weight: bold; }} QPushButton:checked {{ background-color: {ACCENT_BLUE}; }} QPushButton:hover {{ background-color: #646468; }}")
        self.btn_manual.clicked.connect(self.toggle_manual_control)
        m_lay.addWidget(self.btn_manual)

        def create_man_row(title, widgets):
            row = QHBoxLayout()
            lbl = QLabel(title)
            lbl.setStyleSheet("color: #8E8E93; font-size: 16px; font-weight: bold;")
            lbl.setFixedWidth(180)
            row.addWidget(lbl)
            for w in widgets: row.addWidget(w)
            row.addStretch()
            return row

        self.lbl_joy_status = QLabel("N/A")
        self.lbl_joy_status.setStyleSheet("color: white; font-size: 16px; font-weight: bold;")
        m_lay.addLayout(create_man_row("JOYSTICK STATUS", [self.lbl_joy_status]))

        self.btn_ramp = QPushButton("ON")
        self.btn_ramp.setCheckable(True)
        self.btn_ramp.setChecked(True)
        self.btn_ramp.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_ORANGE}; color: #121212; padding: 2px 10px; border-radius: 10px; font-size: 14px; font-weight: bold; }}")
        self.btn_ramp.clicked.connect(self.toggle_ramp_mode)
        
        self.slider_ramp = QSlider(Qt.Horizontal)
        self.slider_ramp.setRange(1, 50)
        self.slider_ramp.setValue(25)
        self.slider_ramp.setStyleSheet(f"QSlider::groove:horizontal {{ height: 6px; background: #333; border-radius: 2px; }} QSlider::handle:horizontal {{ background: {ACCENT_ORANGE}; width: 21px; height: 14px; margin: -3px 0; border-radius: 7px; }}")
        self.slider_ramp.setFixedWidth(250)
        
        self.lbl_ramp_val = QLabel("0.25")
        self.lbl_ramp_val.setStyleSheet("color: white; font-weight: bold; font-size: 16px;")
        self.slider_ramp.valueChanged.connect(lambda v: self.lbl_ramp_val.setText(f"{v/100.0:.2f}"))
        
        ramp_v = QVBoxLayout()
        ramp_v.setSpacing(5)
        rh1 = QHBoxLayout(); rh1.addWidget(QLabel("RAMP", styleSheet="color:white; font-weight:bold; font-size:16px;")); rh1.addWidget(self.btn_ramp); rh1.addStretch()
        rh2 = QHBoxLayout(); rh2.addWidget(QLabel("Value", styleSheet="color:#8E8E93; font-size:16px;")); rh2.addWidget(self.slider_ramp); rh2.addWidget(self.lbl_ramp_val)
        ramp_v.addLayout(rh1); ramp_v.addLayout(rh2)
        
        r_w = QWidget(); r_w.setLayout(ramp_v)
        m_lay.addLayout(create_man_row("PROFILE", [r_w]))

        grid_preset = QGridLayout()
        grid_preset.setSpacing(10)
        btn_p1 = QPushButton("ALL MAX"); btn_p1.setStyleSheet(f"QPushButton {{ background: {ACCENT_BLUE}; padding: 12px; border-radius: 10px; font-size: 16px; font-weight: bold; }}")
        btn_p2 = QPushButton("ALL MIN"); btn_p2.setStyleSheet(f"QPushButton {{ background: {ACCENT_BLUE}; padding: 12px; border-radius: 10px; font-size: 16px; font-weight: bold; }}")
        btn_p3 = QPushButton("SURGE MAX"); btn_p3.setStyleSheet(f"QPushButton {{ background: {ACCENT_BLUE}; padding: 12px; border-radius: 10px; font-size: 16px; font-weight: bold; }}")
        btn_p4 = QPushButton("SWAY MAX"); btn_p4.setStyleSheet(f"QPushButton {{ background: {ACCENT_BLUE}; padding: 12px; border-radius: 10px; font-size: 16px; font-weight: bold; }}")
        btn_p1.clicked.connect(self.preset_all_max); btn_p2.clicked.connect(self.preset_all_min); btn_p3.clicked.connect(self.preset_surge)
        grid_preset.addWidget(btn_p1, 0, 0); grid_preset.addWidget(btn_p2, 0, 1)
        grid_preset.addWidget(btn_p3, 1, 0); grid_preset.addWidget(btn_p4, 1, 1)
        gw = QWidget(); gw.setLayout(grid_preset)
        m_lay.addLayout(create_man_row("QUICK PRESET", [gw]))

        btn_all_stop = QPushButton("ALL STOP")
        btn_all_stop.setStyleSheet(f"QPushButton {{ background: {ACCENT_RED}; color: white; padding: 8px 130px; border-radius: 10px; font-size: 16px; font-weight: bold; }}")
        btn_all_stop.clicked.connect(self.stop_all_thrusters_manual)
        m_lay.addLayout(create_man_row("DANGER", [btn_all_stop]))
        
        m_lay.addStretch()
        scroll_man.setWidget(content_man)
        man_layout.addWidget(scroll_man)

        # --- Tab 2: Auto Control ---
        tab_auto = QWidget()
        auto_layout = QVBoxLayout(tab_auto)
        auto_layout.setContentsMargins(0, 0, 0, 0)
        
        scroll_auto = QScrollArea()
        scroll_auto.setWidgetResizable(True)
        scroll_auto.setStyleSheet("QScrollArea { border: none; background: transparent; } QScrollBar:vertical { background: #1C1C1E; width: 12px; } QScrollBar::handle:vertical { background: #3A3A3C; border-radius: 6px; }")
        
        content_auto = QWidget()
        content_auto.setStyleSheet("background: transparent;")
        a_lay = QVBoxLayout(content_auto)
        a_lay.setContentsMargins(15, 15, 15, 15)
        a_lay.setSpacing(12)

        self.btn_auto = QPushButton("AUTO MODE : OFF")
        self.btn_auto.setCheckable(True)
        self.btn_auto.setStyleSheet(f"QPushButton {{ background-color: #3A3A3C; color: white; padding: 10px; border-radius: 12px; font-size: 16px; font-weight: bold; }} QPushButton:checked {{ background-color: {ACCENT_ORANGE}; color: #121212; }} QPushButton:hover {{ background-color: #646468; }}")
        self.btn_auto.clicked.connect(self.toggle_auto_mode)
        a_lay.addWidget(self.btn_auto)

        self.current_auto_step = 0.01
        step_layout = QHBoxLayout()
        step_lbl = QLabel("SET TARGET"); step_lbl.setStyleSheet("color: #8E8E93; font-weight: bold; font-size: 16px;")
        step_layout.addWidget(step_lbl); step_layout.addStretch()
        step_layout.addWidget(QLabel("STEP", styleSheet="color:#8E8E93; font-weight:bold; font-size:14px;"))
        
        self.step_btns = {}
        
        def update_step_ui(val):
            self.current_auto_step = val
            for v, b in self.step_btns.items():
                bg_color = '#505052' if v == val else 'transparent'
                b.setStyleSheet(f"QPushButton {{ background: {bg_color}; color: white; border-radius: 12px; font-size: 14px; font-weight: bold; padding: 2px 4px; }} QPushButton:hover {{ background: #646468; }}")
        
        for val in [0.01, 0.1, 1.0, 10.0]:
            b = QPushButton(str(val))
            b.setFixedSize(45, 26)
            b.clicked.connect(lambda checked, v=val: update_step_ui(v))
            self.step_btns[val] = b
            step_layout.addWidget(b)
        update_step_ui(0.01) 
        
        a_lay.addLayout(step_layout)

        def create_12state_row(name, key, unit, min_v=-1000, max_v=1000):
            row = QHBoxLayout()
            lbl = QLabel(name)
            lbl.setStyleSheet("color: white; font-size: 16px; font-weight: bold;")
            lbl.setFixedWidth(100) 
            
            sp = QDoubleSpinBox()
            sp.setRange(min_v, max_v); sp.setDecimals(2); sp.setButtonSymbols(QDoubleSpinBox.NoButtons)
            sp.setStyleSheet("background: #2C2C2E; color: white; border: 1px solid #444; border-radius: 4px; padding: 4px; font-size: 16px;")
            sp.setFixedWidth(100)
            self.sp_spinboxes[key] = sp
            
            u_lbl = QLabel(unit)
            u_lbl.setStyleSheet("color: #8E8E93; font-size: 16px;")
            u_lbl.setFixedWidth(30)
            
            btn_minus = QPushButton("-") 
            btn_minus.setFixedSize(40, 26)
            btn_minus.setStyleSheet(f"QPushButton {{ background: #2C2C2E; color: white; border-radius: 8px; font-weight: bold; font-size: 20px; padding: 0px; }} QPushButton:hover {{ background: #FF6961; }}")
            btn_plus = QPushButton("+")
            btn_plus.setFixedSize(40, 26)
            btn_plus.setStyleSheet(f"QPushButton {{ background: #2C2C2E; color: white; border-radius: 8px; font-weight: bold; font-size: 20px; padding: 0px; }} QPushButton:hover {{ background: #FFB340; }}")

            btn_minus.clicked.connect(lambda: sp.setValue(sp.value() - self.current_auto_step))
            btn_plus.clicked.connect(lambda: sp.setValue(sp.value() + self.current_auto_step))
            
            btn_set = QPushButton("SET")
            btn_set.setFixedSize(70, 30)
            btn_set.setStyleSheet(f"QPushButton {{ background: {ACCENT_GREEN}; color: #121212; border-radius: 14px; font-weight: bold; font-size: 12px; padding: 0px; }} QPushButton:hover {{ background: #3BEA55; }}")
            btn_set.clicked.connect(lambda _, k=key, s=sp: self.set_individual_target(k, s.value()))

            curr = QLabel("Current: 0.00")
            curr.setStyleSheet("color: #8E8E93; font-size: 15px;")
            curr.setFixedWidth(100) 
            self.auto_current_labels[key] = curr
            
            row.addWidget(lbl)
            row.addWidget(sp)
            row.addWidget(u_lbl)
            row.addWidget(btn_minus)
            row.addWidget(btn_plus)
            row.addStretch() 
            row.addWidget(btn_set)
            row.addSpacing(10)
            row.addWidget(curr)
            
            return row

        states = [
            ("Position (x)", 'x', 'm'), ("Position (y)", 'y', 'm'), ("Depth (z)", 'depth', 'm'),
            ("Roll (ϕ)", 'roll', 'deg'), ("Pitch (θ)", 'pitch', 'deg'), ("Yaw (ψ)", 'yaw', 'deg'),
            ("Vel (u)", 'u', 'm/s'), ("Vel (v)", 'v', 'm/s'), ("Vel (w)", 'w', 'm/s'),
            ("Rate (p)", 'p', 'd/s'), ("Rate (q)", 'q', 'd/s'), ("Rate (r)", 'r', 'd/s')
        ]
        
        for s in states:
            if s[1] not in self.target_sp:
                self.target_sp[s[1]] = 0.0

        for i, s in enumerate(states):
            a_lay.addLayout(create_12state_row(s[0], s[1], s[2]))
            if i in [2, 5, 8]: 
                line = QFrame(); line.setFrameShape(QFrame.HLine); line.setStyleSheet("color: #333;")
                a_lay.addWidget(line)

        a_lay.addStretch()
        scroll_auto.setWidget(content_auto)
        auto_layout.addWidget(scroll_auto)

        # --- Tab 3: Gain Config ---
        tab_gain = QWidget()
        tab_gain_layout = QVBoxLayout(tab_gain)
        tab_gain_layout.setContentsMargins(0, 0, 0, 0)
        
        scroll_gain = QScrollArea()
        scroll_gain.setWidgetResizable(True)
        scroll_gain.setStyleSheet("QScrollArea { border: none; background-color: transparent; } QScrollBar:vertical { background: #1C1C1E; width: 12px; } QScrollBar::handle:vertical { background: #3A3A3C; border-radius: 6px; }")
        
        content_gain = QWidget()
        content_gain.setStyleSheet("background-color: transparent;")
        gain_layout = QVBoxLayout(content_gain)
        gain_layout.setSpacing(12)
        gain_layout.setAlignment(Qt.AlignTop)

        lbl_gain = QLabel("SDRE GAINS")
        lbl_gain.setStyleSheet(f"color: {ACCENT_ORANGE}; font-size: 16px; font-weight: bold;")
        gain_layout.addWidget(lbl_gain)
        
        self.gain_inputs = {}
        def create_gain_row(name, key, default_val):
            row = QHBoxLayout()
            lbl = QLabel(name)
            lbl.setStyleSheet(f"color: {TEXT_WHITE}; font-size: 16px; font-weight: bold;")
            lbl.setFixedWidth(55)
            
            inp = QLineEdit(default_val)
            inp.setStyleSheet(f"background-color: #2C2C2E; color: {TEXT_WHITE}; border: 1px solid #444; border-radius: 4px; padding: 6px; font-family: monospace; font-size: 16px;")
            self.gain_inputs[key] = inp
            
            row.addWidget(lbl)
            row.addWidget(inp)
            return row
            
        gain_layout.addLayout(create_gain_row("Q_vel:", "q_vel", self.default_gains["q_vel"]))
        gain_layout.addLayout(create_gain_row("Q_pos:", "q_pos", self.default_gains["q_pos"]))
        gain_layout.addLayout(create_gain_row("Q_i:", "q_i", self.default_gains["q_i"]))
        gain_layout.addLayout(create_gain_row("R:", "r", self.default_gains["r"]))
        gain_layout.addLayout(create_gain_row("Qn:", "qn", self.default_gains["qn"]))
        gain_layout.addLayout(create_gain_row("Rn:", "rn", self.default_gains["rn"]))
        
        btn_gain_row = QHBoxLayout()
        
        self.btn_update_gain = QPushButton("UPDATE GAINS")
        self.btn_update_gain.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_ORANGE}; color: black; padding: 8px 12px; border-radius: 4px; font-size: 13px; font-weight: bold; }} QPushButton:hover {{ background-color: #ffad33; }}")
        self.btn_update_gain.clicked.connect(self.update_controller_gains)
        btn_gain_row.addWidget(self.btn_update_gain)

        self.btn_save_gain = QPushButton("💾 SAVE DEFAULTS")
        self.btn_save_gain.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_GREEN}; color: black; padding: 8px 12px; border-radius: 4px; font-size: 13px; font-weight: bold; }} QPushButton:hover {{ background-color: #3BEA55; }}")
        self.btn_save_gain.clicked.connect(self.save_controller_gains)
        btn_gain_row.addWidget(self.btn_save_gain)
        
        gain_layout.addLayout(btn_gain_row)

        scroll_gain.setWidget(content_gain)
        tab_gain_layout.addWidget(scroll_gain)

        self.tabs.addTab(tab_man, "Manual Control")
        self.tabs.addTab(tab_auto, "Auto Control")
        self.tabs.addTab(tab_gain, "Gain Config")
        ctrl_layout.addWidget(self.tabs)

        # --- OSD CONTROL CARD ---
        osd_card = self.create_card()
        osd_layout = QVBoxLayout(osd_card)
        osd_title = QLabel("ON-SCREEN CONTROL PAD")
        osd_title.setObjectName("CardTitle")
        osd_layout.addWidget(osd_title)
                      
        dpad_grid = QGridLayout()
        dpad_grid.setSpacing(8) 
        
        def create_dpad_btn(text, surge=0, sway=0, heave=0, yaw=0, roll=0, pitch=0):
            btn = QPushButton(text)
            btn.setMinimumSize(35, 35) 
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding) 
            btn.setStyleSheet(f"QPushButton {{ background-color: #3A3A3C; border-radius: 12px; font-weight: bold; font-size: 13px; }} QPushButton:hover {{ background-color: #646468; }} QPushButton:pressed {{ background-color: {ACCENT_GREEN}; color: white; }}")
            btn.pressed.connect(lambda: self.set_osd_target(surge=surge, sway=sway, heave=heave, yaw=yaw, roll=roll, pitch=pitch))
            btn.released.connect(self.clear_osd_target)
            return btn

        btn_fwd = create_dpad_btn("▲\nFWD", surge=1)
        btn_rev = create_dpad_btn("▼\nREV", surge=-1)
        btn_lft = create_dpad_btn("◀\nL", sway=-1)
        btn_rgt = create_dpad_btn("▶\nR", sway=1)
        
        btn_up = create_dpad_btn("△\nUP", heave=1)
        btn_dn = create_dpad_btn("▽\nDN", heave=-1)
        
        btn_p_up = create_dpad_btn("▲\nP+", pitch=1)
        btn_p_dn = create_dpad_btn("▼\nP-", pitch=-1)
        btn_r_lft = create_dpad_btn("◀\nR+", roll=-1)
        btn_r_rgt = create_dpad_btn("▶\nR-", roll=1)
        
        btn_ccw = create_dpad_btn("↶\nCCW", yaw=-1)
        btn_cw = create_dpad_btn("↷\nCW", yaw=1)
        
        dpad_grid.addWidget(btn_fwd, 0, 1)
        dpad_grid.addWidget(btn_lft, 1, 0)
        dpad_grid.addWidget(QLabel("XY"), 1, 1, Qt.AlignCenter)
        dpad_grid.addWidget(btn_rgt, 1, 2)
        dpad_grid.addWidget(btn_rev, 2, 1)
        
        spacer1 = QLabel(""); spacer1.setFixedWidth(20); dpad_grid.addWidget(spacer1, 1, 2)
        
        dpad_grid.addWidget(btn_up, 0, 4)
        dpad_grid.addWidget(QLabel("Z"), 1, 4, Qt.AlignCenter)
        dpad_grid.addWidget(btn_dn, 2, 4)
        
        spacer2 = QLabel(""); spacer2.setFixedWidth(20); dpad_grid.addWidget(spacer2, 1, 5)
        
        dpad_grid.addWidget(btn_p_dn, 0, 6)
        dpad_grid.addWidget(QLabel("PITCH"), 0, 7, Qt.AlignCenter)
        dpad_grid.addWidget(btn_p_up, 0, 8)
        
        dpad_grid.addWidget(btn_r_lft, 1, 6)
        dpad_grid.addWidget(QLabel("ROLL"), 1, 7, Qt.AlignCenter)
        dpad_grid.addWidget(btn_r_rgt, 1, 8)
        
        dpad_grid.addWidget(btn_ccw, 2, 6)
        dpad_grid.addWidget(QLabel("YAW"), 2, 7, Qt.AlignCenter)
        dpad_grid.addWidget(btn_cw, 2, 8)

        for i in range(dpad_grid.count()):
            w = dpad_grid.itemAt(i).widget()
            if isinstance(w, QLabel) and w.text() in ["XY", "Z", "YAW", "PITCH", "ROLL"]:
                w.setStyleSheet(f"color: {TEXT_DIM}; font-size: 18px; font-weight: bold;") 
                
        # 🟢 เอาสปริงแนวนอนออก ปล่อยให้ Grid ขยายเต็มความกว้างซ้าย-ขวา
        osd_layout.addStretch()         # สปริงดันจากด้านบน
        osd_layout.addLayout(dpad_grid) # ใส่ตารางปุ่มลงไปโดยตรง
        osd_layout.addStretch()         # สปริงดันจากด้านล่าง

        # --- SENSORS & GRAPHS CARDS (Left side) ---
        row1 = QHBoxLayout()
        nav_card = self.create_card()
        nav_grid = QGridLayout(nav_card)
        self.add_dual_stat(nav_grid, "Roll", "deg", "rad", 0, 0)
        self.add_dual_stat(nav_grid, "Pitch", "deg", "rad", 0, 1)
        self.add_dual_stat(nav_grid, "Yaw", "deg", "rad", 0, 2)
        
        self.btn_tare_roll = QPushButton("TARE ROLL")
        self.btn_tare_roll.setStyleSheet("""QPushButton { background-color: #737373; color: #1A1A1A; padding: 4px; font-size: 14px; border-radius: 12px; } QPushButton:hover { background-color: #B5B5B5; }""")
        self.btn_tare_roll.setFixedHeight(24) 
        self.btn_tare_roll.clicked.connect(self.tare_roll)
        nav_grid.addWidget(self.btn_tare_roll, 1, 0)

        self.btn_tare_pitch = QPushButton("TARE PITCH")
        self.btn_tare_pitch.setStyleSheet("""QPushButton { background-color: #737373; color: #1A1A1A; padding: 4px; font-size: 14px; border-radius: 12px; } QPushButton:hover { background-color: #B5B5B5; }""")
        self.btn_tare_pitch.setFixedHeight(24) 
        self.btn_tare_pitch.clicked.connect(self.tare_pitch)
        nav_grid.addWidget(self.btn_tare_pitch, 1, 1)

        self.btn_tare_yaw = QPushButton("TARE YAW")
        self.btn_tare_yaw.setStyleSheet("""QPushButton { background-color: #737373; color: #1A1A1A; padding: 4px; font-size: 14px; border-radius: 12px; } QPushButton:hover { background-color: #B5B5B5; }""")
        self.btn_tare_yaw.setFixedHeight(24) 
        self.btn_tare_yaw.clicked.connect(self.tare_yaw)
        nav_grid.addWidget(self.btn_tare_yaw, 1, 2)
        
        row1.addWidget(nav_card, stretch=2)

        env_card = self.create_card()
        env_grid = QGridLayout(env_card)
        self.add_stat(env_grid, "Depth", "m", 0, 0); self.add_stat(env_grid, "Pressure", "kPa", 0, 1)
        
        self.btn_calib = QPushButton("TARE DEPTH")
        self.btn_calib.setStyleSheet("""QPushButton { background-color: #737373; color: #1A1A1A; padding: 4px; font-size: 14px; border-radius: 12px; } QPushButton:hover { background-color: #B5B5B5; }""")
        self.btn_calib.setFixedHeight(24) 
        self.btn_calib.clicked.connect(self.calibrate_depth)
        env_grid.addWidget(self.btn_calib, 1, 0, 1, 2)
        row1.addWidget(env_card, stretch=1)

        # UNIT QUATERNION
        imu_card = self.create_card()
        imu_layout = QHBoxLayout(imu_card)
        imu_layout.setContentsMargins(15, 8, 15, 8) 
        
        imu_title = QLabel("UNIT QUATERNION:"); imu_title.setObjectName("CardTitle")
        imu_layout.addWidget(imu_title)
        imu_layout.addSpacing(15)
        
        for q in ["W", "X", "Y", "Z"]:
            lbl = QLabel(f"{q}:")
            lbl.setStyleSheet(f"color: {TEXT_DIM}; font-size: 14px; font-weight: bold;")
            val = QLabel("0.000")
            val.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {TEXT_WHITE};")
            self.data_labels[f"Quat {q}"] = val
            imu_layout.addWidget(lbl)
            imu_layout.addWidget(val)
            imu_layout.addSpacing(15)
            
        imu_layout.addStretch()

        file_status_card = self.create_card()
        fs_layout_bottom = QHBoxLayout(file_status_card)
        fs_layout_bottom.setContentsMargins(10, 8, 10, 8) 
        self.file_status_lbl = QLabel("FILE STATUS: IDLE")
        self.file_status_lbl.setStyleSheet("color: #8E8E93; font-weight: bold; font-size: 13px;")
        
        # 🟢 นำปุ่ม TEST MODE ย้ายมาไว้ในเฟรมนี้
        self.btn_test_mode = QPushButton("TEST MODE: OFF")
        self.btn_test_mode.setCheckable(True)
        self.btn_test_mode.setStyleSheet(f"QPushButton {{ background-color: transparent; color: {TEXT_DIM}; font-size: 11px; font-weight: bold; border: none; }} QPushButton:checked {{ color: {ACCENT_ORANGE}; }}")
        self.btn_test_mode.toggled.connect(self.on_test_mode_toggled)

        fs_layout_bottom.addWidget(self.file_status_lbl)
        fs_layout_bottom.addStretch() # 🟢 ใส่สปริงเพื่อดันปุ่มด้านล่างให้ไปชิดขวาสุด
        fs_layout_bottom.addWidget(self.btn_test_mode)

        # ==========================================
        # ASSEMBLE PAGE 1 (NORMAL)
        # ==========================================
        self.left_layout.addWidget(self.cam_card, stretch=5) 
        self.left_layout.addLayout(row1)
        self.left_layout.addWidget(imu_card)

        bottom_left_h = QHBoxLayout()
        bottom_left_h.setSpacing(10)
        
        self.graph1_w, self.lines1, self.combo_g1 = self.create_dynamic_plot("Telemetry Graph:")
        self.combo_g1.setCurrentText("Angular Displacement")
        
        self.graph1_w.setMaximumHeight(260)
        osd_card.setMaximumHeight(260)
        
        bottom_left_h.addWidget(self.graph1_w, stretch=1)
        bottom_left_h.addWidget(osd_card, stretch=1)

        self.left_layout.addLayout(bottom_left_h) 
        self.left_layout.addWidget(file_status_card)

        # จัดการ Assembly ฝั่งขวาใหม่
        right_layout.addLayout(top_right_h)
        right_layout.addWidget(sig_card) 
        right_layout.addWidget(ctrl_card, stretch=1) 

        main_layout.addLayout(self.left_layout, stretch=6)
        main_layout.addLayout(right_layout, stretch=4) 
        
        # ----------------------------------------------------
        # PAGE 2: FULL SCREEN CAMERA + BATTERIES
        # ----------------------------------------------------
        self.page_fs = QWidget()
        self.fs_page_layout = QVBoxLayout(self.page_fs)
        self.fs_page_layout.setSpacing(10)
        self.fs_page_layout.setContentsMargins(10, 10, 10, 10)
        
        self.fs_batt_layout = QHBoxLayout()
        self.fs_batt_layout.setSpacing(10)
        
        self.central_stack.addWidget(self.page_normal)
        self.central_stack.addWidget(self.page_fs)

    # --- HELPERS & LOGIC ---
    def create_dynamic_plot(self, title_text):
        w = QFrame()
        w.setStyleSheet(f"background-color: {BG_CARD}; border-radius: 8px;")
        l = QVBoxLayout(w)
        l.setContentsMargins(10, 10, 10, 10)
        
        top_h = QHBoxLayout()
        lbl = QLabel(title_text)
        lbl.setStyleSheet("color: #8E8E93; font-size: 14px; font-weight: bold; border: none;")
        combo = QComboBox()
        combo.addItems(["Angular Displacement", "Linear Displacement", "Angular Velocities", "Linear Acceleration"])
        combo.setStyleSheet("""
                    QComboBox {
                        background-color: #3A3A3C;
                        color: #F5F5F7;
                        border-radius: 12px; 
                        padding: 4px 15px;
                        font-size: 14px;
                        font-weight: bold;
                        border: none;
                    }
                    QComboBox::drop-down {
                        subcontrol-origin: padding;
                        subcontrol-position: top right;
                        width: 25px;
                        border: none; 
                        background-color: transparent; 
                    }
                    QComboBox QAbstractItemView {
                        background-color: #2C2C2E;
                        color: #F5F5F7;
                        border-radius: 8px;
                        border: 1px solid #444;
                        selection-background-color: #0A84FF; 
                        outline: none; 
                    }
                """)
        top_h.addWidget(lbl)
        top_h.addWidget(combo)
        top_h.addStretch()
        
        plot = pg.PlotWidget()
        plot.setBackground(BG_CARD)
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.addLegend(offset=(5, 5))
        plot.getAxis('left').setTextPen("#8E8E93")
        plot.getAxis('bottom').setTextPen("#8E8E93")
        
        lines = {
            "Roll": plot.plot(pen=pg.mkPen(color=ACCENT_RED, width=2), name="Roll"),
            "Pitch": plot.plot(pen=pg.mkPen(color=ACCENT_GREEN, width=2), name="Pitch"),
            "Yaw": plot.plot(pen=pg.mkPen(color=ACCENT_BLUE, width=2), name="Yaw"),
            "Depth (z)": plot.plot(pen=pg.mkPen(color=ACCENT_BLUE, width=2), name="Depth (z)"),
            "p": plot.plot(pen=pg.mkPen(color=ACCENT_RED, width=2), name="p"),
            "q": plot.plot(pen=pg.mkPen(color=ACCENT_GREEN, width=2), name="q"),
            "r": plot.plot(pen=pg.mkPen(color=ACCENT_BLUE, width=2), name="r"),
            "Acc x": plot.plot(pen=pg.mkPen(color=ACCENT_RED, width=2), name="Acc x"),
            "Acc y": plot.plot(pen=pg.mkPen(color=ACCENT_GREEN, width=2), name="Acc y"),
            "Acc z": plot.plot(pen=pg.mkPen(color=ACCENT_BLUE, width=2), name="Acc z")
        }
        for line in lines.values():
            line.setVisible(False)
            
        l.addLayout(top_h)
        l.addWidget(plot)
        return w, lines, combo

    def update_dynamic_plot(self, combo, lines):
        sel = combo.currentText()
        for line in lines.values(): 
            line.setVisible(False)
        
        if sel == "Angular Displacement":
            lines["Roll"].setData(self.t_data, self.r_data); lines["Roll"].setVisible(True)
            lines["Pitch"].setData(self.t_data, self.p_data); lines["Pitch"].setVisible(True)
            lines["Yaw"].setData(self.t_data, self.y_data); lines["Yaw"].setVisible(True)
        elif sel == "Linear Displacement":
            lines["Depth (z)"].setData(self.t_data, self.d_data); lines["Depth (z)"].setVisible(True)
        elif sel == "Angular Velocities":
            lines["p"].setData(self.t_data, self.gx_data); lines["p"].setVisible(True)
            lines["q"].setData(self.t_data, self.gy_data); lines["q"].setVisible(True)
            lines["r"].setData(self.t_data, self.gz_data); lines["r"].setVisible(True)
        elif sel == "Linear Acceleration":
            lines["Acc x"].setData(self.t_data, self.lx_data); lines["Acc x"].setVisible(True)
            lines["Acc y"].setData(self.t_data, self.ly_data); lines["Acc y"].setVisible(True)
            lines["Acc z"].setData(self.t_data, self.lz_data); lines["Acc z"].setVisible(True)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape and self.is_fullscreen_mode:
            self.toggle_fullscreen_cam()
        super().keyPressEvent(event)

    def toggle_fullscreen_cam(self):
        icon_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui_icon")
        if not self.is_fullscreen_mode:
            self.fs_page_layout.addWidget(self.cam_card, stretch=1)
            
            self.fs_batt_layout.addWidget(self.ctr_batt_card)
            self.fs_batt_layout.addWidget(self.thr_batt_card)
            self.fs_page_layout.addLayout(self.fs_batt_layout)
            
            self.central_stack.setCurrentWidget(self.page_fs)
            self.showFullScreen() 
            
            self.btn_full.setText(" EXIT FULL SCREEN")
            self.btn_full.setIcon(QIcon(os.path.join(icon_dir, "exitfullscreen.png")))
            self.btn_full.setIconSize(QSize(16, 16))
            self.btn_full.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_RED}; color: {TEXT_WHITE}; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: {ACCENT_RED_HOVER_D}; }}")
            self.is_fullscreen_mode = True
        else:
            self.left_layout.insertWidget(0, self.cam_card, stretch=5)
            self.row2_layout.insertWidget(0, self.ctr_batt_card, stretch=2)
            self.row2_layout.insertWidget(1, self.thr_batt_card, stretch=2)
            
            self.central_stack.setCurrentWidget(self.page_normal)
            self.showMaximized() 
            
            self.btn_full.setText(" FULL SCREEN")
            self.btn_full.setIcon(QIcon(os.path.join(icon_dir, "fullscreen.png")))
            self.btn_full.setIconSize(QSize(16, 16))
            self.btn_full.setStyleSheet(f"QPushButton {{ background-color: #3A3A3C; color: {TEXT_WHITE}; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: #505052; }}")

            self.is_fullscreen_mode = False

    def update_controller_gains(self):
        try:
            cmd = []
            for key, inp in self.gain_inputs.items():
                cmd.append(f"{key}:{inp.text().strip()}")
            cmd_str = ";".join(cmd)
            self.ros_thread.send_gain_command(cmd_str)
            self.file_status_lbl.setText("⚙️ CONTROLLER GAINS UPDATED AND SENT TO PI!")
            self.file_status_lbl.setStyleSheet(f"color: {ACCENT_ORANGE}; font-weight: bold; font-size: 13px;")
        except Exception as e:
            print(f"Gain Update Error: {e}")

    def save_controller_gains(self):
        try:
            gains_to_save = {}
            for key, inp in self.gain_inputs.items():
                gains_to_save[key] = inp.text().strip()
            
            os.makedirs(os.path.dirname(self.gain_config_file), exist_ok=True)
            with open(self.gain_config_file, 'w') as f:
                json.dump(gains_to_save, f, indent=4)
            
            self.file_status_lbl.setText("💾 GAINS SAVED AS SYSTEM DEFAULT!")
            self.file_status_lbl.setStyleSheet(f"color: {ACCENT_GREEN}; font-weight: bold; font-size: 13px;")
        except Exception as e:
            print(f"Gain Save Error: {e}")
            self.file_status_lbl.setText("❌ ERROR SAVING GAINS!")
            self.file_status_lbl.setStyleSheet(f"color: {ACCENT_RED}; font-weight: bold; font-size: 13px;")

    def set_individual_target(self, key, val):
        if not self.is_auto_mode:
            self.file_status_lbl.setText("⚠️ PLEASE ENABLE AUTO MODE FIRST!")
            self.file_status_lbl.setStyleSheet(f"color: {ACCENT_RED}; font-weight: bold; font-size: 13px;")
            self.sp_spinboxes[key].setValue(self.target_sp[key]) 
            return
            
        self.target_sp[key] = float(val)
        self.send_current_setpoint()
        self.file_status_lbl.setText(f"🎯 SP UPDATED: {key.upper()} = {val:.2f}")
        self.file_status_lbl.setStyleSheet(f"color: {ACCENT_ORANGE}; font-weight: bold; font-size: 13px;")

    def update_sliders_from_feedback(self, pwms):
        if getattr(self, 'is_auto_mode', False):
            for i, val in enumerate(pwms):
                if i < len(self.sliders):
                    slider, label = self.sliders[i]
                    slider.blockSignals(True)
                    slider.setValue(val)
                    label.setText(str(val))
                    if val > 1520: 
                        label.setStyleSheet(f"background-color: rgba(255, 69, 58, 0.2); color: {ACCENT_RED}; border: 1px solid {ACCENT_RED}; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
                    elif val < 1480: 
                        label.setStyleSheet(f"background-color: rgba(58, 69, 255, 0.2); color: {ACCENT_BLUE}; border: 1px solid {ACCENT_BLUE}; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
                    else: 
                        label.setStyleSheet("background-color: #2C2C2E; color: #F5F5F7; border: 1px solid transparent; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
                    slider.blockSignals(False)

    def send_current_setpoint(self):
            self.ros_thread.send_setpoint(
                "AUTO",
                self.target_sp['depth'],
                self.target_sp['roll'],
                self.target_sp['pitch'],
                self.target_sp['yaw']
            )

    def toggle_auto_mode(self):
        self.is_auto_mode = self.btn_auto.isChecked()
        if self.is_auto_mode:
            self.btn_auto.setText("AUTO MODE : ON")
            self.btn_auto.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_ORANGE}; color: #121212; padding: 10px; border-radius: 12px; font-size: 16px; font-weight: bold; }} ")
            
            if self.is_manual_control:
                self.btn_manual.setChecked(False)
                self.toggle_manual_control()
            
            try:
                self.target_sp['depth'] = float(self.data_labels["Depth"].text())
                self.target_sp['roll'] = float(self.data_labels["Roll"].text())
                self.target_sp['pitch'] = float(self.data_labels["Pitch"].text())
                self.target_sp['yaw'] = float(self.data_labels["Yaw"].text())
                
                self.sp_spinboxes['depth'].setValue(self.target_sp['depth'])
                self.sp_spinboxes['roll'].setValue(self.target_sp['roll'])
                self.sp_spinboxes['pitch'].setValue(self.target_sp['pitch'])
                self.sp_spinboxes['yaw'].setValue(self.target_sp['yaw'])
            except:
                pass
            
            self.send_current_setpoint()
            self.file_status_lbl.setText(f"🎯 TARGET LOCKED: D={self.target_sp['depth']:.1f}m, Y={self.target_sp['yaw']:.1f}°")
            self.file_status_lbl.setStyleSheet(f"color: {ACCENT_ORANGE}; font-weight: bold; font-size: 13px;")
        else:
            self.btn_auto.setText("AUTO MODE : OFF")
            self.btn_auto.setStyleSheet(f"QPushButton {{ background-color: #3A3A3C; color: white; padding: 10px; border-radius: 12px; font-size: 16px; font-weight: bold; }} QPushButton:hover {{ background-color: #646468; }} QPushButton:hover {{ background-color: #646468; }}")
            
            self.ros_thread.send_setpoint("MANUAL")
            self.stop_all_thrusters_internal()
            self.file_status_lbl.setText("FILE STATUS: IDLE")
            self.file_status_lbl.setStyleSheet("color: #8E8E93; font-weight: bold; font-size: 13px;")

    def toggle_manual_control(self):
        self.is_manual_control = self.btn_manual.isChecked()
        if self.is_manual_control:
            self.btn_manual.setText("MANUAL MODE : ON")
            self.btn_manual.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_BLUE}; color: white; padding: 10px; border-radius: 12px; font-size: 16px; font-weight: bold; }}")
            
            if self.is_auto_mode:
                self.btn_auto.setChecked(False)
                self.toggle_auto_mode()
        else:
            self.btn_manual.setText("MANUAL MODE : OFF")
            self.btn_manual.setStyleSheet(f"QPushButton {{ background-color: #3A3A3C; color: white; padding: 10px; border-radius: 12px; font-size: 16px; font-weight: bold; }} QPushButton:hover {{ background-color: #646468; }}")
            self.clear_osd_target()
            self.current_osd = {k: 0.0 for k in self.current_osd}
            self.stop_all_thrusters_internal() 

    def toggle_ramp_mode(self):
        self.is_ramp_mode = self.btn_ramp.isChecked()
        if self.is_ramp_mode:
            self.btn_ramp.setText("ON")
            self.btn_ramp.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_ORANGE}; color: #121212; padding: 2px 10px; border-radius: 10px; font-size: 14px; font-weight: bold; }}")
        else:
            self.btn_ramp.setText("OFF")
            self.btn_ramp.setStyleSheet(f"QPushButton {{ background-color: #505052; color: white; padding: 2px 10px; border-radius: 10px; font-size: 14px; font-weight: bold; }}")
            
    def set_osd_target(self, surge=0, sway=0, heave=0, yaw=0, roll=0, pitch=0):
        if getattr(self, 'is_critical_batt', False) or getattr(self, 'is_signal_lost', False): return
        
        if self.is_auto_mode:
            self.target_sp['depth'] -= heave * 0.1
            self.target_sp['yaw'] += yaw * 5.0      
            self.target_sp['roll'] += roll * 5.0
            self.target_sp['pitch'] += pitch * 5.0
            
            if self.target_sp['yaw'] > 180: self.target_sp['yaw'] -= 360
            if self.target_sp['yaw'] < -180: self.target_sp['yaw'] += 360
            
            self.sp_spinboxes['depth'].setValue(self.target_sp['depth'])
            self.sp_spinboxes['roll'].setValue(self.target_sp['roll'])
            self.sp_spinboxes['pitch'].setValue(self.target_sp['pitch'])
            self.sp_spinboxes['yaw'].setValue(self.target_sp['yaw'])

            self.send_current_setpoint()
            self.file_status_lbl.setText(f"🎯 SP UPDATED: D={self.target_sp['depth']:.1f}m, Y={self.target_sp['yaw']:.1f}°, P={self.target_sp['pitch']:.1f}°, R={self.target_sp['roll']:.1f}°")
            self.file_status_lbl.setStyleSheet(f"color: {ACCENT_ORANGE}; font-weight: bold; font-size: 13px;")
            return

        if not self.is_manual_control: return
        self.target_osd['surge'] = float(surge)
        self.target_osd['sway'] = float(sway)
        self.target_osd['heave'] = float(heave)
        self.target_osd['yaw'] = float(yaw)
        self.target_osd['roll'] = float(roll)
        self.target_osd['pitch'] = float(pitch)

    def clear_osd_target(self):
        if getattr(self, 'is_auto_mode', False): return 
        self.set_osd_target(0, 0, 0, 0, 0, 0)

    def update_osd_control(self):
        if not getattr(self, 'is_manual_control', False): return
        if getattr(self, 'is_auto_mode', False): return 
        if getattr(self, 'is_critical_batt', False) or getattr(self, 'is_signal_lost', False): return

        active = False
        ramp_speed = (self.slider_ramp.value() / 100.0) if self.is_ramp_mode else 1.0 

        for axis in self.current_osd:
            diff = self.target_osd[axis] - self.current_osd[axis]
            if abs(diff) > 0.001:
                active = True
                if diff > 0:
                    self.current_osd[axis] = min(self.target_osd[axis], self.current_osd[axis] + ramp_speed)
                else:
                    self.current_osd[axis] = max(self.target_osd[axis], self.current_osd[axis] - ramp_speed)
            elif self.current_osd[axis] != 0:
                active = True
                self.current_osd[axis] = self.target_osd[axis] 

        if active:
            self.send_osd_pwm()
            self.was_osd_active = True
        elif getattr(self, 'was_osd_active', False):
            self.send_osd_pwm() 
            self.was_osd_active = False

    def send_osd_pwm(self):
        speed = 400 
        
        surge = self.current_osd['surge']
        sway = self.current_osd['sway']
        heave = self.current_osd['heave']
        yaw = self.current_osd['yaw']
        roll = self.current_osd['roll']
        pitch = self.current_osd['pitch']
        
        t1 = 1500 - (surge * speed) - (sway * speed) + (yaw * speed)
        t2 = 1500 - (surge * speed) + (sway * speed) - (yaw * speed)
        t3 = 1500 + (surge * speed) - (sway * speed) + (yaw * speed)
        t4 = 1500 + (surge * speed) + (sway * speed) - (yaw * speed)
        
        t5 = 1500 - (heave * speed) + (roll * speed) - (pitch * speed)
        t6 = 1500 - (heave * speed) - (roll * speed) - (pitch * speed)
        t7 = 1500 - (heave * speed) + (roll * speed) + (pitch * speed)
        t8 = 1500 - (heave * speed) - (roll * speed) + (pitch * speed)

        pwms = [t1, t2, t3, t4, t5, t6, t7, t8]
        pwms = [int(max(1000, min(2000, p))) for p in pwms]

        for i, val in enumerate(pwms):
            if i < len(self.sliders):
                slider, label = self.sliders[i]
                slider.blockSignals(True)
                slider.setValue(val)
                label.setText(str(val))
                if val > 1520: 
                    label.setStyleSheet(f"background-color: rgba(255, 69, 58, 0.2); color: {ACCENT_RED}; border: 1px solid {ACCENT_RED}; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
                elif val < 1480: 
                    label.setStyleSheet(f"background-color: rgba(58, 69, 255, 0.2); color: {ACCENT_BLUE}; border: 1px solid {ACCENT_BLUE}; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
                else: 
                    label.setStyleSheet("background-color: #2C2C2E; color: #F5F5F7; border: 1px solid transparent; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
                slider.blockSignals(False)
                
        self.ros_thread.send_pwm_command(pwms)

    def process_joystick(self, joy_msg):
        self.last_joy_time = time.time()
        if not self.is_joy_connected:
            self.is_joy_connected = True
            self.lbl_joy_status.setText("🎮 JOY: CONNECTED")
            self.lbl_joy_status.setStyleSheet(f"color: {ACCENT_GREEN}; font-size: 16px; font-weight: bold;")

        if not self.is_manual_control or getattr(self, 'is_critical_batt', False) or getattr(self, 'is_signal_lost', False):
            return

        if getattr(self, 'is_auto_mode', False):
            return

        osd_in_use = any(abs(v) > 0.001 for v in self.target_osd.values()) or any(abs(v) > 0.001 for v in self.current_osd.values())
        if osd_in_use:
            return

        idx_sway  = 0  
        idx_surge = 1  
        idx_yaw   = 2  
        idx_heave = 3  
        idx_roll  = 6  
        idx_pitch = 7  

        def deadband(val): 
            return val if abs(val) > 0.1 else 0.0

        sway  = deadband(joy_msg.axes[idx_sway])
        surge = deadband(joy_msg.axes[idx_surge])
        yaw   = deadband(joy_msg.axes[idx_yaw])
        heave = -deadband(joy_msg.axes[idx_heave]) 
        
        roll  = joy_msg.axes[idx_roll] if len(joy_msg.axes) > idx_roll else 0.0
        pitch = joy_msg.axes[idx_pitch] if len(joy_msg.axes) > idx_pitch else 0.0

        t1 = 1500 - (surge * 500) - (sway * 500) + (yaw * 500)
        t2 = 1500 - (surge * 500) + (sway * 500) - (yaw * 500)
        t3 = 1500 + (surge * 500) - (sway * 500) + (yaw * 500)
        t4 = 1500 + (surge * 500) + (sway * 500) - (yaw * 500)
        
        t5 = 1500 - (heave * 500) + (roll * 500) - (pitch * 500)
        t6 = 1500 - (heave * 500) - (roll * 500) - (pitch * 500)
        t7 = 1500 - (heave * 500) + (roll * 500) + (pitch * 500)
        t8 = 1500 - (heave * 500) - (roll * 500) + (pitch * 500)

        pwms = [t1, t2, t3, t4, t5, t6, t7, t8]
        pwms = [int(max(1000, min(2000, p))) for p in pwms]

        for i, val in enumerate(pwms):
            if i < len(self.sliders):
                slider, label = self.sliders[i]
                slider.blockSignals(True)
                slider.setValue(val)
                label.setText(str(val))
                if val > 1520: 
                    label.setStyleSheet(f"background-color: rgba(255, 69, 58, 0.2); color: {ACCENT_RED}; border: 1px solid {ACCENT_RED}; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
                elif val < 1480: 
                    label.setStyleSheet(f"background-color: rgba(58, 69, 255, 0.2); color: {ACCENT_BLUE}; border: 1px solid {ACCENT_BLUE}; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
                else: 
                    label.setStyleSheet("background-color: #2C2C2E; color: #F5F5F7; border: 1px solid transparent; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
                slider.blockSignals(False)
                
        self.ros_thread.send_pwm_command(pwms)

    def on_slider_change(self, idx, val, lbl):
        if getattr(self, 'is_auto_mode', False) or getattr(self, 'is_critical_batt', False) or getattr(self, 'is_signal_lost', False):
            self.sliders[idx][0].blockSignals(True)
            self.sliders[idx][0].setValue(1500)
            lbl.setText("1500")
            lbl.setStyleSheet("background-color: #2C2C2E; color: #F5F5F7; border: 1px solid transparent; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
            self.sliders[idx][0].blockSignals(False)
            if getattr(self, 'is_auto_mode', False): return
            self.ros_thread.send_pwm_command([1500] * 8)
            return

        lbl.setText(str(val))
        if val > 1520: 
            lbl.setStyleSheet(f"background-color: rgba(255, 69, 58, 0.2); color: {ACCENT_RED}; border: 1px solid {ACCENT_RED}; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
        elif val < 1480: 
            lbl.setStyleSheet(f"background-color: rgba(58, 69, 255, 0.2); color: {ACCENT_BLUE}; border: 1px solid {ACCENT_BLUE}; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
        else: 
            lbl.setStyleSheet("background-color: #2C2C2E; color: #F5F5F7; border: 1px solid transparent; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
        
        self.ros_thread.send_pwm_command([s.value() for s, _ in self.sliders])

    def stop_all_thrusters_manual(self):
        if self.is_auto_mode:
            self.btn_auto.setChecked(False)
            self.toggle_auto_mode()
        if self.is_manual_control:
            self.btn_manual.setChecked(False)
            self.toggle_manual_control()
            
        self.clear_osd_target()
        self.current_osd = {k: 0.0 for k in self.current_osd}
        self.stop_all_thrusters_internal()
        self.update_status_label()

    def stop_all_thrusters_internal(self):
        for s, l in self.sliders:
            s.blockSignals(True)
            s.setValue(1500)
            l.setText("1500")
            l.setStyleSheet("background-color: #2C2C2E; color: #F5F5F7; border: 1px solid transparent; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-family: monospace;")
            s.blockSignals(False)
        self.ros_thread.send_pwm_command([1500] * 8)

    def preset_all_max(self):
        if getattr(self, 'is_auto_mode', False) or getattr(self, 'is_critical_batt', False) or getattr(self, 'is_signal_lost', False): return
        for s, l in self.sliders: s.setValue(2000)

    def preset_all_min(self):
        if getattr(self, 'is_auto_mode', False) or getattr(self, 'is_critical_batt', False) or getattr(self, 'is_signal_lost', False): return
        for s, l in self.sliders: s.setValue(1000)

    def preset_surge(self):
        if getattr(self, 'is_auto_mode', False) or getattr(self, 'is_critical_batt', False) or getattr(self, 'is_signal_lost', False): return
        surge_pwm = [1000, 1000, 2000, 2000, 1000, 1000, 1500, 1500]
        for i, (s, l) in enumerate(self.sliders):
            if i < len(surge_pwm):
                s.setValue(surge_pwm[i])

    def create_card(self):
        f = QFrame(); f.setObjectName("Card"); return f

    def add_stat(self, layout, name, unit, row, col):
        w = QWidget(); v = QVBoxLayout(w); v.setContentsMargins(0,0,0,0); v.setSpacing(1)
        t = QLabel(name.upper()); t.setObjectName("CardTitle")
        val = QLabel("0.00"); val.setObjectName("Value")
        u = QLabel(unit); u.setObjectName("Unit")
        h = QHBoxLayout(); h.addWidget(val); h.addWidget(u); h.addStretch()
        v.addWidget(t); v.addLayout(h)
        layout.addWidget(w, row, col)
        self.data_labels[name] = val

    def add_dual_stat(self, layout, name, unit1, unit2, row, col):
        w = QWidget(); v = QVBoxLayout(w); v.setContentsMargins(0,0,0,0); v.setSpacing(1)
        t = QLabel(name.upper()); t.setObjectName("CardTitle")
        
        val1 = QLabel("0.00"); val1.setObjectName("Value")
        u1 = QLabel(unit1); u1.setObjectName("Unit")
        h1 = QHBoxLayout(); h1.addWidget(val1); h1.addWidget(u1); h1.addStretch()
        
        val2 = QLabel("0.0000")
        val2.setStyleSheet(f"color: {TEXT_DIM}; font-size: 16px; font-weight: bold;")
        u2 = QLabel(unit2)
        u2.setStyleSheet(f"color: {TEXT_DIM}; font-size: 12px;")
        h2 = QHBoxLayout(); h2.addWidget(val2); h2.addWidget(u2); h2.addStretch()
        h2.setContentsMargins(0, -5, 0, 0)
        
        v.addWidget(t); v.addLayout(h1); v.addLayout(h2)
        layout.addWidget(w, row, col)
        
        self.data_labels[name] = val1
        self.data_labels[f"{name}_rad"] = val2

    def tare_roll(self):
        self.ros_thread.send_sys_command("TARE_ROLL")
        self.file_status_lbl.setText("📍 ROLL ZEROED")
        self.file_status_lbl.setStyleSheet(f"color: {ACCENT_GREEN}; font-weight: bold; font-size: 13px;")

    def tare_pitch(self):
        self.ros_thread.send_sys_command("TARE_PITCH")
        self.file_status_lbl.setText("📍 PITCH ZEROED")
        self.file_status_lbl.setStyleSheet(f"color: {ACCENT_GREEN}; font-weight: bold; font-size: 13px;")

    def tare_yaw(self):
        self.ros_thread.send_sys_command("TARE_YAW")
        self.file_status_lbl.setText("📍 YAW ZEROED")
        self.file_status_lbl.setStyleSheet(f"color: {ACCENT_GREEN}; font-weight: bold; font-size: 13px;")


    def calibrate_depth(self):
        try:
            self.depth_offset += float(self.data_labels["Depth"].text())
            self.file_status_lbl.setText(f"📍 DEPTH CALIBRATED (Offset: {self.depth_offset:.2f}m)")
            self.file_status_lbl.setStyleSheet(f"color: {ACCENT_GREEN}; font-weight: bold; font-size: 13px;")
        except: pass

    def toggle_logging(self):
        icon_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui_icon")
        if not self.is_logging:
            save_dir = os.path.expanduser("~/auv_data/logs")
            os.makedirs(save_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = os.path.join(save_dir, f"auv_log_{ts}.csv")
            self.log_file = open(fname, 'w', newline='')
            self.csv_writer = csv.writer(self.log_file)
            
            log_header = [
                "Time", "Hull Batt", "Thruster Batt", "Temp", "Pressure", "z (depth)",
                "Roll", "Roll_rad", "Pitch", "Pitch_rad", "Yaw", "Yaw_rad",
                "p", "q", "r", "acc_x", "acc_y", "acc_z",
                "Quat W", "Quat X", "Quat Y", "Quat Z"
            ]
            self.csv_writer.writerow(log_header)
            
            self.is_logging = True
            self.btn_log.setText(" STOP LOGGING")
            self.btn_log.setIcon(QIcon(os.path.join(icon_dir, "pause.png")))
            self.btn_log.setIconSize(QSize(14, 14))                  
            self.btn_log.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_RED}; color: white; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: {ACCENT_RED_HOVER_L}; }}")
            
            self.file_status_lbl.setText(f"📝 RECORDING LOG: {os.path.basename(fname)}")
            self.file_status_lbl.setStyleSheet(f"color: {ACCENT_BLUE}; font-weight: bold; font-size: 13px;")
        else:
            self.is_logging = False
            self.log_file.close()
            self.btn_log.setText(" START LOGGING")
            self.btn_log.setIcon(QIcon(os.path.join(icon_dir, "record.png")))
            self.btn_log.setIconSize(QSize(14, 14))
            self.btn_log.setStyleSheet(f"QPushButton {{ background-color: #E5E5EA; color: #121212; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: #CCCCCC; }}")
            
            self.file_status_lbl.setText("📝 LOG SAVED")
            self.file_status_lbl.setStyleSheet(f"color: {ACCENT_GREEN}; font-weight: bold; font-size: 13px;")

    def emergency_stop(self):
        self.stop_all_thrusters_manual()

    def restart_program(self):
        self.status_lbl.setText("RESTARTING SYSTEM...")
        QApplication.processEvents()
        self.stop_all_thrusters_internal()
        time.sleep(0.1)
        if self.is_logging and self.log_file:
            self.log_file.close()
        if self.is_video_recording and self.video_writer:
            self.video_writer.release()
        if self.is_streaming:
            self.ros_thread.send_cam_command("STOP")
        self.ros_thread.stop()
        os.execl(sys.executable, sys.executable, *sys.argv)

    def update_telemetry(self, data_str):
        parts = data_str.split(',')
        if len(parts) >= 18 and parts[0] == "DATA": # เช็ค >= 18 เผื่อเวอร์ชันเก่า แต่เดี๋ยวเราเช็ค 21 ข้างใน
            try:
                self.last_heartbeat = time.time()
                if self.is_signal_lost:
                    self.is_signal_lost = False
                    
                y, p, r = parts[1:4]
                d = float(parts[4]) - self.depth_offset
                
                r_rad = math.radians(float(r))
                p_rad = math.radians(float(p))
                y_rad = math.radians(float(y))
                
                pres = (997 * 9.80665 * d) / 1000.0
                v_hull = float(parts[5]); v_thrust = float(parts[6])
                
                v_hull_calc = round(v_hull, 1)
                v_thrust_calc = round(v_thrust, 1)
                
                pct_hull = max(0.0, min(100.0, ((v_hull_calc - 6.0) / (8.4 - 6.0)) * 100.0))
                pct_thrust = max(0.0, min(100.0, ((v_thrust_calc - 18.0) / (25.2 - 18.0)) * 100.0))
                
                test_mode_on = self.btn_test_mode.isChecked()
                
                if pct_hull < 10.0 or (pct_thrust < 10.0 and not test_mode_on):
                    if not self.is_critical_batt:
                        self.is_critical_batt = True
                        self.stop_all_thrusters_internal()
                else:
                    self.is_critical_batt = False
                    
                if not self.is_critical_batt:
                    if pct_hull < 25.0:
                        self.low_batt_msg = "Battery Low! please charge the controller battery"
                    elif pct_thrust < 25.0 and not test_mode_on:
                        self.low_batt_msg = "Battery Low! please charge the thruster battery"
                    else:
                        self.low_batt_msg = ""
                else:
                    self.low_batt_msg = ""
                
                try:
                    if float(parts[17]) > 60.0:
                        self.is_overheat = True
                    else:
                        self.is_overheat = False
                except ValueError:
                    pass

                self.update_status_label()

                def get_batt_color(pct):
                    if pct >= 50: return "#32D74B" 
                    elif pct >= 25: return "#FFD60A" 
                    else: return "#FF453A" 

                if "Hull %" in self.data_labels:
                    self.data_labels["Hull %"].setStyleSheet(f"color: {get_batt_color(pct_hull)}; font-weight: bold; font-size: 30px;")
                if "Thruster %" in self.data_labels:
                    self.data_labels["Thruster %"].setStyleSheet(f"color: {get_batt_color(pct_thrust)}; font-weight: bold; font-size: 30px;")

                qw, qx, qy, qz = parts[7:11]; lx, ly, lz = parts[11:14]; gx, gy, gz = parts[14:17]; temp = parts[17]
                
                vals = {
                    "Yaw": y, "Pitch": p, "Roll": r, 
                    "Yaw_rad": f"{y_rad:.4f}", "Pitch_rad": f"{p_rad:.4f}", "Roll_rad": f"{r_rad:.4f}",
                    "Depth": f"{d:.2f}", "Pressure": f"{pres:.2f}",
                    "Hull Batt": f"{v_hull:.2f} V", "Hull %": f"{pct_hull:.0f}%",
                    "Thruster Batt": f"{v_thrust:.2f} V", "Thruster %": f"{pct_thrust:.0f}%",
                    "Temp": f"{float(temp):.1f} °C", "Quat W": qw, "Quat X": qx, "Quat Y": qy, "Quat Z": qz
                }
                for k, v in vals.items():
                    if k in self.data_labels: self.data_labels[k].setText(str(v))

                if 'depth' in self.auto_current_labels:
                    self.auto_current_labels['depth'].setText(f"Current: {d:.2f}")
                    self.auto_current_labels['roll'].setText(f"Current: {r}")
                    self.auto_current_labels['pitch'].setText(f"Current: {p}")
                    self.auto_current_labels['yaw'].setText(f"Current: {y}")

                if self.is_logging:
                    log_row = [
                        datetime.now().strftime("%H:%M:%S.%f")[:-3],
                        f"{v_hull:.2f}", f"{v_thrust:.2f}", temp, f"{pres:.2f}", f"{d:.2f}",
                        r, f"{r_rad:.4f}", p, f"{p_rad:.4f}", y, f"{y_rad:.4f}",
                        gx, gy, gz, lx, ly, lz,
                        qw, qx, qy, qz
                    ]
                    self.csv_writer.writerow(log_row)
                    
                current_t = time.time() - self.start_time
                self.t_data.append(current_t)
                self.r_data.append(float(r)); self.p_data.append(float(p)); self.y_data.append(float(y)); self.d_data.append(d)
                self.lx_data.append(float(lx)); self.ly_data.append(float(ly)); self.lz_data.append(float(lz))
                self.gx_data.append(float(gx)); self.gy_data.append(float(gy)); self.gz_data.append(float(gz))

                # 🟢 ใช้มุม Yaw ส่งให้เข็มทิศโดยตรง (เพราะ Yaw ถูก Tare มาจากบอร์ดแล้ว)
                if len(parts) >= 21:
                    heading = float(y)
                    
                    # ควบคุมให้อยู่ในช่วง 0-360 องศา
                    if heading < 0:
                        heading += 360.0
                    elif heading >= 360.0:
                        heading -= 360.0
                    
                    # สั่งให้เข็มทิศชี้ตามมุม Yaw ที่ Tare แล้ว
                    self.compass_widget.set_heading(heading)
                
                self.update_dynamic_plot(self.combo_g1, self.lines1)

            except Exception as e:
                print(f"[GUI Error] Telemetry parsing failed: {e}") 

    def on_test_mode_toggled(self, checked):
        if checked:
            self.btn_test_mode.setText("TEST MODE: ON")
        else:
            self.btn_test_mode.setText("TEST MODE: OFF")
        self.update_status_label()

    def update_status_label(self):
        if getattr(self, 'is_signal_lost', False):
            self.status_lbl.setText("SIGNAL LOSS!")
            self.status_lbl.setStyleSheet("color: #FF453A; font-weight: bold; font-size: 16px; text-transform: uppercase;")
        elif getattr(self, 'is_critical_batt', False):
            self.status_lbl.setText("CONTROL DENIED! Please charge the battery.")
            self.status_lbl.setStyleSheet("color: #FF453A; font-weight: bold; font-size: 16px; text-transform: uppercase;")
        elif getattr(self, 'is_overheat', False):
            self.status_lbl.setText("🔥 CRITICAL WARNING: HILL TEMP EXCEEDS 60°C!")
            self.status_lbl.setStyleSheet("color: #FF453A; font-weight: bold; font-size: 16px; text-transform: uppercase;")
        elif getattr(self, 'low_batt_msg', "") != "":
            self.status_lbl.setText(self.low_batt_msg)
            self.status_lbl.setStyleSheet("color: #FFD60A; font-weight: bold; font-size: 16px; text-transform: uppercase;")
        elif getattr(self, 'btn_test_mode', None) and self.btn_test_mode.isChecked():
            self.status_lbl.setText("⚠️ TEST MODE: Thruster Battery Ignored")
            self.status_lbl.setStyleSheet(f"color: {ACCENT_ORANGE}; font-weight: bold; font-size: 16px; text-transform: uppercase;")
        else:
            self.status_lbl.setText("STATUS DETAILS")
            self.status_lbl.setStyleSheet("color: #8E8E93; font-weight: bold; font-size: 16px; text-transform: uppercase;")

    def check_connection(self):
        if time.time() - self.last_heartbeat > 2.0:
            if not self.is_signal_lost:
                self.is_signal_lost = True
                self.stop_all_thrusters_internal() 
            self.update_status_label()

        if self.is_joy_connected and (time.time() - self.last_joy_time > 2.0):
            self.is_joy_connected = False
            self.lbl_joy_status.setText("🎮 JOY: DISCONNECTED")
            self.lbl_joy_status.setStyleSheet(f"color: {ACCENT_RED}; font-size: 12px; font-weight: bold;")
            
            if self.is_manual_control:
                self.stop_all_thrusters_internal()
    def toggle_fullscreen_cam(self):
        icon_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui_icon")
        if not self.is_fullscreen_mode:
            self.fs_page_layout.addWidget(self.cam_card, stretch=1)
            
            self.fs_batt_layout.addWidget(self.ctr_batt_card)
            self.fs_batt_layout.addWidget(self.thr_batt_card)
            self.fs_page_layout.addLayout(self.fs_batt_layout)
            
            self.central_stack.setCurrentWidget(self.page_fs)
            self.showFullScreen() 
            
            self.btn_full.setText(" EXIT FULL SCREEN")
            self.btn_full.setIcon(QIcon(os.path.join(icon_dir, "exitfullscreen.png")))
            self.btn_full.setIconSize(QSize(16, 16))
            self.btn_full.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_RED}; color: {TEXT_WHITE}; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: {ACCENT_RED_HOVER_D}; }}")
            self.is_fullscreen_mode = True
        else:
            self.left_layout.insertWidget(0, self.cam_card, stretch=5)
            self.row2_layout.insertWidget(0, self.ctr_batt_card, stretch=2)
            self.row2_layout.insertWidget(1, self.thr_batt_card, stretch=2)
            
            self.central_stack.setCurrentWidget(self.page_normal)
            self.showMaximized() 
            
            self.btn_full.setText(" FULL SCREEN")
            self.btn_full.setIcon(QIcon(os.path.join(icon_dir, "fullscreen.png")))
            self.btn_full.setIconSize(QSize(16, 16))
            self.btn_full.setStyleSheet(f"QPushButton {{ background-color: #3A3A3C; color: {TEXT_WHITE}; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: #505052; }}")

            self.is_fullscreen_mode = False

    def close_fullscreen(self):
        if getattr(self, 'is_fullscreen_mode', False):
            self.toggle_fullscreen_cam()

    def toggle_stream(self):
        icon_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui_icon")
        self.is_streaming = not self.is_streaming
        if self.is_streaming:
            res_text = self.combo_res.currentText()
            if "360p" in res_text: w, h = 640, 360
            elif "720p" in res_text: w, h = 1280, 720
            elif "1080p" in res_text: w, h = 1920, 1080
            elif "2K" in res_text: w, h = 2560, 1440
            self.ros_thread.send_cam_command(f"START,{w},{h}")
            
            self.combo_res.setEnabled(False) 
            self.btn_stream.setText(" STOP STREAM")
            self.btn_stream.setIcon(QIcon(os.path.join(icon_dir, "pause.png")))
            self.btn_stream.setIconSize(QSize(14, 14))
            self.btn_stream.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_RED}; color: white; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: {ACCENT_RED_HOVER_L}; }}")
            self.video_label.setText("Starting stream...")
        else:
            self.ros_thread.send_cam_command("STOP")
            if self.is_video_recording:
                self.toggle_video_recording()
            self.combo_res.setEnabled(True)
            self.btn_stream.setText(" START STREAM")
            self.btn_stream.setIcon(QIcon(os.path.join(icon_dir, "play.png")))
            self.btn_stream.setIconSize(QSize(14, 14))
            self.btn_stream.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_GREEN}; color: #121212; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: {ACCENT_GREEN_HOVER_L}; }}")
            self.video_label.clear()
            self.video_label.setText("CAMERA OFFLINE")
            self.latest_frame = None
    
    def capture_image(self):
        if self.latest_frame is not None:
            save_dir = os.path.expanduser("~/auv_data/images")
            os.makedirs(save_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = os.path.join(save_dir, f"img_{ts}.jpg")
            cv2.imwrite(fname, self.latest_frame)
            h, w, _ = self.latest_frame.shape
            
            self.file_status_lbl.setText(f"📸 IMAGE SAVED ({h}p): img_{ts}.jpg")
            self.file_status_lbl.setStyleSheet(f"color: {ACCENT_GREEN}; font-weight: bold; font-size: 13px;")
        else:
            self.file_status_lbl.setText("NO VIDEO STREAM TO CAPTURE!")
            self.file_status_lbl.setStyleSheet(f"color: {ACCENT_RED}; font-weight: bold; font-size: 13px;")

    def toggle_video_recording(self):
        icon_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui_icon")
        save_dir = os.path.expanduser("~/auv_data/videos")
        os.makedirs(save_dir, exist_ok=True)
        if not self.is_video_recording:
            if self.latest_frame is None:
                self.file_status_lbl.setText("WAITING FOR STREAM BEFORE RECORDING...")
                self.file_status_lbl.setStyleSheet(f"color: {ACCENT_RED}; font-weight: bold; font-size: 13px;")
                return
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = os.path.join(save_dir, f"vid_{ts}.mp4")
            h, w, _ = self.latest_frame.shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(fname, fourcc, 30.0, (w, h))
            self.is_video_recording = True
            
            self.btn_rec.setText(" STOP RECORD")
            self.btn_rec.setIcon(QIcon(os.path.join(icon_dir, "record_stop.png")))
            self.btn_rec.setIconSize(QSize(14, 14))
            self.btn_rec.setStyleSheet(f"QPushButton {{ background-color: {ACCENT_RED}; color: white; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: {ACCENT_RED_HOVER_D}; }}")
                    
            self.file_status_lbl.setText(f"🎥 RECORDING VIDEO ({h}p): vid_{ts}.mp4")
            self.file_status_lbl.setStyleSheet(f"color: {ACCENT_ORANGE}; font-weight: bold; font-size: 14px;")
        else:
            self.is_video_recording = False
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None
                
            self.btn_rec.setText(" RECORD")
            self.btn_rec.setIcon(QIcon(os.path.join(icon_dir, "record.png")))
            self.btn_rec.setIconSize(QSize(14, 14))
            self.btn_rec.setStyleSheet(f"QPushButton {{ background-color: #F5F5F7; color: #121212; padding: 6px 15px; border-radius: 14px; font-size: 14px; font-weight: bold; }} QPushButton:hover {{ background-color: #CCCCCC; }}")
            
            self.file_status_lbl.setText("🎥 VIDEO SAVED")
            self.file_status_lbl.setStyleSheet(f"color: {ACCENT_GREEN}; font-weight: bold; font-size: 14px;")

    def update_image(self, cv_img):
        self.latest_frame = cv_img.copy()
        if self.is_video_recording and self.video_writer is not None:
            self.video_writer.write(cv_img)
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_img = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_img)
        
        self.video_label.setPixmap(pixmap.scaled(self.video_label.width(), self.video_label.height(), Qt.KeepAspectRatio))
        if getattr(self, 'fs_window', None) and self.fs_window.isVisible():
            self.fs_label.setPixmap(pixmap.scaled(self.fs_window.width(), self.fs_window.height(), Qt.KeepAspectRatio))

    def closeEvent(self, event):
        self.stop_all_thrusters_internal()
        self.close_fullscreen() 
        try:
            time.sleep(0.1)
        except:
            pass
        if self.is_video_recording and self.video_writer:
            self.video_writer.release()
        if self.is_streaming:
            self.ros_thread.send_cam_command("STOP")
        self.ros_thread.stop()
        event.accept()

# ==============================================================
def main(args=None):
    app = QApplication(sys.argv)
    
    app.setApplicationName("auv-dashboard")
    app.setDesktopFileName("auv-dashboard.desktop")
    
    import os
    from PyQt5.QtGui import QIcon
    icon_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "auv_icon.png")
    app.setWindowIcon(QIcon(icon_path))
    
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(BG_MAIN))
    palette.setColor(QPalette.WindowText, QColor(TEXT_WHITE))
    app.setPalette(palette)
    
    window = ModernDashboard()
    window.show() 
    
    QTimer.singleShot(100, window.showMaximized) 
    
    try: sys.exit(app.exec_())
    except KeyboardInterrupt: pass

if __name__ == "__main__":
    main()
