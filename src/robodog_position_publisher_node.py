#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QSlider, QLabel, QPushButton, QGroupBox, QGridLayout, QFrame
)
from PyQt5.QtCore import Qt, QThread
import numpy as np
import time

class ROS2Publisher(Node):
    def __init__(self):
        super().__init__('robodog_position_publisher_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_position_controller/commands', 10)
        self.values = [0.0] * 12
        self.is_active = False

    def update_value(self, index, value):
        self.values[index] = value

    def publish_values(self):
        if self.is_active:
            msg = Float64MultiArray()
            msg.data = self.values
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {self.values}')

    def start_publishing(self):
        self.is_active = True

    def stop_publishing(self):
        self.is_active = False

class SliderThread(QThread):
    def __init__(self, ros_node, get_frequency_callback):
        super().__init__()
        self.ros_node = ros_node
        self.get_frequency_callback = get_frequency_callback
        self.running = False

    def run(self):
        self.running = True
        while self.running:
            hz = self.get_frequency_callback()
            interval = 1.0 / hz if hz > 0 else 0.1
            self.ros_node.publish_values()
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)
            time.sleep(interval)

    def stop(self):
        self.running = False
        self.wait()

class SliderGUI(QWidget):
    def __init__(self, ros_node, thread):
        super().__init__()
        self.ros_node = ros_node
        self.slider_thread = thread
        self.joint_names = [
            "front_left_leg_P0_R0_joint", "front_left_leg_P1_R1_joint", "front_left_leg_P2_R2_joint",
            "front_right_leg_P0_R0_joint", "front_right_leg_P1_R1_joint", "front_right_leg_P2_R2_joint",
            "back_left_leg_P0_R0_joint", "back_left_leg_P1_R1_joint", "back_left_leg_P2_R2_joint",
            "back_right_leg_P0_R0_joint", "back_right_leg_P1_R1_joint", "back_right_leg_P2_R2_joint"
        ]
        self.sliders = []
        self.labels = []
        self.value_labels = []
        self.freq_value_label = QLabel()
        self.frequency_slider = None
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("ROS2 Joint Control Panel")
        self.setGeometry(100, 100, 600, 750)
        main_layout = QVBoxLayout()

        group_titles = ['Front Left Leg', 'Front Right Leg', 'Back Left Leg', 'Back Right Leg']
        for i in range(4):
            group_box = QGroupBox(group_titles[i])
            group_box.setStyleSheet("QGroupBox { font-weight: bold; font-size: 14px; }")
            grid = QGridLayout()
            for j in range(3):
                index = i * 3 + j
                label = QLabel(self.joint_names[index])
                label.setFixedWidth(180)
                label.setStyleSheet("font-weight: bold;")
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(-3141)  # Daha yüksek çözünürlük
                slider.setMaximum(3141)
                slider.setValue(0)
                slider.setTickInterval(1)
                slider.setSingleStep(1)
                value_label = QLabel("0.000 rad / 0°")
                value_label.setFixedWidth(120)

                slider.valueChanged.connect(lambda value, idx=index: self.update_value(idx, value))

                grid.addWidget(label, j, 0)
                grid.addWidget(slider, j, 1)
                grid.addWidget(value_label, j, 2)

                self.sliders.append(slider)
                self.labels.append(label)
                self.value_labels.append(value_label)

            group_box.setLayout(grid)
            main_layout.addWidget(group_box)

        # Yayın frekansı ayarı
        freq_box = QGroupBox("Publish Frequency (Hz)")
        freq_layout = QHBoxLayout()
        self.frequency_slider = QSlider(Qt.Horizontal)
        self.frequency_slider.setMinimum(1)
        self.frequency_slider.setMaximum(100)
        self.frequency_slider.setValue(10)
        self.frequency_slider.setTickInterval(1)
        self.frequency_slider.setSingleStep(1)
        self.frequency_slider.valueChanged.connect(self.update_frequency_label)

        self.freq_value_label.setText("10 Hz")
        self.freq_value_label.setFixedWidth(60)

        freq_layout.addWidget(self.frequency_slider)
        freq_layout.addWidget(self.freq_value_label)
        freq_box.setLayout(freq_layout)
        main_layout.addWidget(freq_box)

        # Butonlar
        button_layout = QHBoxLayout()

        self.start_button = QPushButton("▶ Start Publishing")
        self.start_button.setStyleSheet("font-size: 16px; padding: 6px;")
        self.start_button.clicked.connect(self.start_publishing)
        button_layout.addWidget(self.start_button)

        self.stop_button = QPushButton("⏸ Stop Publishing")
        self.stop_button.setStyleSheet("font-size: 16px; padding: 6px;")
        self.stop_button.clicked.connect(self.stop_publishing)
        button_layout.addWidget(self.stop_button)

        self.reset_button = QPushButton("🔄 Reset All")
        self.reset_button.setStyleSheet("font-size: 16px; padding: 6px;")
        self.reset_button.clicked.connect(self.reset_all_sliders)
        button_layout.addWidget(self.reset_button)

        # Separator
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)

        main_layout.addWidget(line)
        main_layout.addLayout(button_layout)
        self.setLayout(main_layout)

    def update_frequency_label(self):
        hz = self.frequency_slider.value()
        self.freq_value_label.setText(f"{hz} Hz")

    def get_publish_frequency(self):
        return self.frequency_slider.value()

    def update_value(self, index, value):
        radian_value = value / 1000.0 * np.pi  # Daha hassas çözünürlük
        degree_value = np.degrees(radian_value)  # Radyandan dereceye dönüşüm
        self.ros_node.update_value(index, radian_value)
        self.value_labels[index].setText(f"{radian_value:.3f} rad / {degree_value:.1f}°")

    def reset_all_sliders(self):
        for i, slider in enumerate(self.sliders):
            slider.setValue(0)
            self.ros_node.update_value(i, 0.0)
            self.value_labels[i].setText("0.000 rad / 0°")

    def start_publishing(self):
        if not self.slider_thread.isRunning():
            self.slider_thread.start()
        self.ros_node.start_publishing()

    def stop_publishing(self):
        self.ros_node.stop_publishing()
        self.slider_thread.stop()

def main():
    os.environ["QT_QPA_PLATFORM"] = "xcb"
    rclpy.init()
    ros_node = ROS2Publisher()

    app = QApplication(sys.argv)

    gui = SliderGUI(ros_node, None)
    slider_thread = SliderThread(ros_node, gui.get_publish_frequency)
    gui.slider_thread = slider_thread

    gui.show()

    try:
        app.exec_()
    finally:
        gui.stop_publishing()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
