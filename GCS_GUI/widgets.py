import cv2
import os
import subprocess
import time
import csv
from PyQt6.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, 
    QPushButton, QTableWidget, QTableWidgetItem, 
    QHeaderView, QTabWidget, QFrame
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QUrl, QTimer
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWebEngineWidgets import QWebEngineView

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(QImage)
    def __init__(self, source=0):
        super().__init__()
        self.source = source
        self._run_flag = True
    def run(self):
        cap = cv2.VideoCapture(self.source)
        while self._run_flag:
            ret, cv_img = cap.read()
            if ret:
                rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                qt_img = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                self.change_pixmap_signal.emit(qt_img.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio))
            else:
                time.sleep(1)
        cap.release()
    def stop(self):
        self._run_flag = False
        self.wait()

class VideoWidget(QFrame):
    def __init__(self, source=0, title="Camera Stream"):
        super().__init__()
        layout = QVBoxLayout()
        self.title_label = QLabel(title)
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_label.setStyleSheet("font-weight: bold; color: #38bdf8; background-color: rgba(56, 189, 248, 0.1); padding: 5px;")
        
        self.image_label = QLabel("INITIALIZING FEED...")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setStyleSheet("background-color: black; border-radius: 5px;")
        
        layout.addWidget(self.title_label)
        layout.addWidget(self.image_label)
        self.setLayout(layout)
        
        self.thread = VideoThread(source)
        self.thread.change_pixmap_signal.connect(lambda img: self.image_label.setPixmap(QPixmap.fromImage(img)))
        self.thread.start()

class WebWidget(QFrame):
    def __init__(self, url="https://google.com", title="Web View"):
        super().__init__()
        layout = QVBoxLayout()
        self.title_label = QLabel(title)
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_label.setStyleSheet("font-weight: bold; color: #38bdf8; background-color: rgba(56, 189, 248, 0.1); padding: 5px;")
        
        self.browser = QWebEngineView()
        self.browser.setUrl(QUrl(url))
        
        layout.addWidget(self.title_label)
        layout.addWidget(self.browser)
        self.setLayout(layout)

class TelemetryHubWidget(QFrame):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout()
        
        # --- Header with Counter ---
        header_layout = QHBoxLayout()
        self.title_label = QLabel("LIVE SURVIVOR TRACKING")
        self.title_label.setStyleSheet("font-weight: bold; color: #10b981; font-size: 14px;")
        
        self.counter_label = QLabel("TOTAL FOUND: 0")
        self.counter_label.setStyleSheet("""
            background-color: #059669; 
            color: white; 
            font-weight: 800; 
            padding: 2px 10px; 
            border-radius: 5px;
            border: 1px solid #34d399;
        """)
        
        header_layout.addWidget(self.title_label)
        header_layout.addStretch()
        header_layout.addWidget(self.counter_label)

        self.tabs = QTabWidget()
        self.drone1_table = self.create_table()
        self.drone2_table = self.create_table()
        
        self.tabs.addTab(self.drone1_table, "DRONE 1")
        self.tabs.addTab(self.drone2_table, "DRONE 2")

        layout.addLayout(header_layout)
        layout.addWidget(self.tabs)
        self.setLayout(layout)

        self.drone1_cache = {}
        self.drone2_cache = {}
        self.total_detected_count = 0
        self.counted_locations = set() 

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_data)
        self.timer.start(1000)

    def create_table(self):
        table = QTableWidget()
        table.setColumnCount(3)
        table.setHorizontalHeaderLabels(["COUNT", "LOCATION", "STATUS"])
        header = table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        return table

    def refresh_data(self):
        self.process_csv("drone1_telemetry.csv", self.drone1_table, self.drone1_cache)
        self.process_csv("drone2_telemetry.csv", self.drone2_table, self.drone2_cache)
        self.counter_label.setText(f"TOTAL FOUND: {self.total_detected_count}")

    def process_csv(self, filename, table, cache):
        if not os.path.exists(filename): return
        try:
            with open(filename, 'r') as f:
                reader = list(csv.reader(f))
                if len(reader) > 1:
                    for row in reader[1:]:
                        if len(row) < 3: continue
                        count_val, loc, raw_status = row[0], row[1], row[2]
                        status_text = raw_status.strip().lower()

                        # --- FILTER LOGIC ---
                        # Skip if it is just "Searching"
                        if "searching" in status_text:
                            continue 
                        
                        # Only proceed if status is Detected or Delivered
                        if "detected" in status_text or "delivered" in status_text:
                            # Update Counter only for NEW detections
                            if loc not in self.counted_locations and "detected" in status_text:
                                try:
                                    self.total_detected_count += int(count_val)
                                    self.counted_locations.add(loc)
                                except ValueError:
                                    pass

                            # Add to cache (This keeps the record on screen)
                            cache[loc] = [count_val, raw_status]

            # Update the UI table from the filtered cache
            table.setRowCount(len(cache))
            for i, (loc, info) in enumerate(cache.items()):
                table.setItem(i, 0, QTableWidgetItem(info[0]))
                table.setItem(i, 1, QTableWidgetItem(loc))
                
                status_item = QTableWidgetItem(info[1])
                curr_status = info[1].strip().lower()
                
                if "delivered" in curr_status:
                    status_item.setBackground(Qt.GlobalColor.green)
                    status_item.setForeground(Qt.GlobalColor.black)
                elif "detected" in curr_status:
                    status_item.setBackground(Qt.GlobalColor.red)
                    status_item.setForeground(Qt.GlobalColor.white)
                
                table.setItem(i, 2, status_item)
        except: pass

class QGCLauncherWidget(QFrame):
    def __init__(self, app_path="./QGroundControl.AppImage"):
        super().__init__()
        self.app_path = app_path
        self.layout = QVBoxLayout()
        
        self.status_label = QLabel("QGROUNDCONTROL SYSTEM")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.btn = QPushButton("LAUNCH MISSION CONTROL")
        self.btn.clicked.connect(self.launch)
        
        self.layout.addStretch()
        self.layout.addWidget(self.status_label)
        self.layout.addWidget(self.btn)
        self.layout.addStretch()
        self.setLayout(self.layout)

    def launch(self):
        if os.path.exists(self.app_path):
            subprocess.Popen([self.app_path])
            self.btn.setEnabled(False)
            self.status_label.setText("SYSTEM ACTIVE")