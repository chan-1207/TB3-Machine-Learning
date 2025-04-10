#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from PyQt5.QtWidgets import (
    QWidget, QProgressBar, QLabel, QLineEdit,
    QGridLayout, QApplication
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal

class Ros2Subscriber(Node):
    def __init__(self, qt_thread):
        super().__init__('progress_subscriber')
        self.qt_thread = qt_thread

        # "get_action" 토픽 구독
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'get_action',
            self.get_array_callback,
            10
        )

    def get_array_callback(self, msg):
        data = list(msg.data)

        # (1) 우선 모든 프로그레스바를 0으로 초기화
        self.qt_thread.change_value1.emit(0)
        self.qt_thread.change_value2.emit(0)
        self.qt_thread.change_value3.emit(0)
        self.qt_thread.change_value4.emit(0)
        self.qt_thread.change_value5.emit(0)

        # (2) data[0]의 값에 따라 해당 프로그레스바만 100 설정
        if data[0] == 0:
            self.qt_thread.change_value1.emit(100)
        elif data[0] == 1:
            self.qt_thread.change_value2.emit(100)
        elif data[0] == 2:
            self.qt_thread.change_value3.emit(100)
        elif data[0] == 3:
            self.qt_thread.change_value4.emit(100)
        elif data[0] == 4:
            self.qt_thread.change_value5.emit(100)

        # (3) 마지막 2개 값은 "Total reward", "Reward" 라벨에 표시 (소숫점 둘째 자리까지)
        if len(data) >= 2:
            self.qt_thread.change_value6.emit(str(round(data[-2], 2)))
            self.qt_thread.change_value7.emit(str(round(data[-1], 2)))


class Thread(QThread):
    # 프로그레스바 5개에 값을 주기 위한 시그널
    change_value1 = pyqtSignal(int)
    change_value2 = pyqtSignal(int)
    change_value3 = pyqtSignal(int)
    change_value4 = pyqtSignal(int)
    change_value5 = pyqtSignal(int)
    # "Total reward", "Reward" 표시용 시그널
    change_value6 = pyqtSignal(str)
    change_value7 = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.node = None

    def run(self):
        # 스레드가 시작되면 ROS2 Subscriber 노드를 생성 & spin
        self.node = Ros2Subscriber(self)
        rclpy.spin(self.node)
        # spin이 종료되면 노드 제거
        self.node.destroy_node()


class Form(QWidget):
    def __init__(self, qt_thread):
        super().__init__(flags=Qt.Widget)
        self.setWindowTitle("Action State")

        # GridLayout으로 구성
        layout = QGridLayout()

        # 1) 세로 프로그레스바 5개
        self.pgsb1 = QProgressBar()
        self.pgsb1.setOrientation(Qt.Vertical)
        self.pgsb1.setValue(0)
        self.pgsb1.setRange(0, 100)

        self.pgsb2 = QProgressBar()
        self.pgsb2.setOrientation(Qt.Vertical)
        self.pgsb2.setValue(0)
        self.pgsb2.setRange(0, 100)

        self.pgsb3 = QProgressBar()
        self.pgsb3.setOrientation(Qt.Vertical)
        self.pgsb3.setValue(0)
        self.pgsb3.setRange(0, 100)

        self.pgsb4 = QProgressBar()
        self.pgsb4.setOrientation(Qt.Vertical)
        self.pgsb4.setValue(0)
        self.pgsb4.setRange(0, 100)

        self.pgsb5 = QProgressBar()
        self.pgsb5.setOrientation(Qt.Vertical)
        self.pgsb5.setValue(0)
        self.pgsb5.setRange(0, 100)

        # 2) 왼쪽에 "Total reward", "Reward" 라벨과 텍스트필드
        self.label_total_reward = QLabel("Total reward")
        self.edit_total_reward = QLineEdit("")
        self.edit_total_reward.setDisabled(True)
        self.edit_total_reward.setFixedWidth(100)

        self.label_reward = QLabel("Reward")
        self.edit_reward = QLineEdit("")
        self.edit_reward.setDisabled(True)
        self.edit_reward.setFixedWidth(100)

        # 3) 하단에 "Left", "Front", "Right" 라벨
        self.label_left = QLabel("Left")
        self.label_front = QLabel("Front")
        self.label_right = QLabel("Right")

        # 4) 레이아웃 배치
        #    (참고) 세로 프로그레스바를 가로로 나열하고,
        #    그 아래에 "Left", "Front", "Right" 텍스트를 배치
        #    아래는 예시: pgsb1, pgsb2, pgsb3, pgsb4, pgsb5 가 (0,4)부터 (0,8)까지 차지하도록
        #    (원본 코드 배치를 참고)

        # 왼쪽(0,0~3,0)에 텍스트와 라벨 배치
        layout.addWidget(self.label_total_reward, 0, 0)
        layout.addWidget(self.edit_total_reward, 1, 0)
        layout.addWidget(self.label_reward, 2, 0)
        layout.addWidget(self.edit_reward, 3, 0)

        # 프로그레스바 (행 0~3, 열 4~8)
        # rowSpan=4, colSpan=1 식으로 세로로 공간을 할당
        layout.addWidget(self.pgsb1, 0, 4, 4, 1)
        layout.addWidget(self.pgsb2, 0, 5, 4, 1)
        layout.addWidget(self.pgsb3, 0, 6, 4, 1)
        layout.addWidget(self.pgsb4, 0, 7, 4, 1)
        layout.addWidget(self.pgsb5, 0, 8, 4, 1)

        # 하단 라벨(행 4, 열 4/6/8)
        layout.addWidget(self.label_left, 4, 4)
        layout.addWidget(self.label_front, 4, 6)
        layout.addWidget(self.label_right, 4, 8)

        # 최종 레이아웃 적용
        self.setLayout(layout)

        # 5) QThread 시그널 -> GUI 위젯 연결
        qt_thread.change_value1.connect(self.pgsb1.setValue)
        qt_thread.change_value2.connect(self.pgsb2.setValue)
        qt_thread.change_value3.connect(self.pgsb3.setValue)
        qt_thread.change_value4.connect(self.pgsb4.setValue)
        qt_thread.change_value5.connect(self.pgsb5.setValue)
        qt_thread.change_value6.connect(self.edit_total_reward.setText)
        qt_thread.change_value7.connect(self.edit_reward.setText)

    def closeEvent(self, event):
        # 창 종료 시에는 ROS2 노드와 스레드를 정리
        rclpy.shutdown()
        event.accept()


# -------------------------------------------------
# 4) main() 함수
# -------------------------------------------------
def main(args=None):
    rclpy.init(args=args)  # ROS2 초기화
    qt_thread = Thread()   # QThread (ROS2 spin)
    qt_thread.start()      # 스레드 시작

    # PyQt5 앱 구동
    app = QApplication(sys.argv)
    form = Form(qt_thread)
    form.show()

    exit_code = app.exec_()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
