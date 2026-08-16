import sys
import os
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, 
    QGridLayout, QLabel, QGraphicsDropShadowEffect
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPixmap

# Ensure widgets.py is in the same directory
from widgets import VideoWidget, WebWidget, QGCLauncherWidget, TelemetryHubWidget

class GCSMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("ResQWings | Ground Control Station")
        self.resize(1280, 800)

        # Main Container
        central_widget = QWidget()
        central_widget.setObjectName("GridOverlay")
        self.setCentralWidget(central_widget)

        layout = QGridLayout()
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)
        central_widget.setLayout(layout)

        # ================= LOGO (TOP LEFT) =================
        self.logo_label = QLabel()
        logo_path = os.path.join(os.path.dirname(__file__), "assets", "NidarLogo.png")
        
        if os.path.exists(logo_path):
            pixmap = QPixmap(logo_path).scaled(120, 60, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            self.logo_label.setPixmap(pixmap)
        
        # White Glow Effect for Black Logo Visibility
        glow = QGraphicsDropShadowEffect()
        glow.setBlurRadius(20)
        glow.setColor(Qt.GlobalColor.white)
        glow.setOffset(0, 0)
        self.logo_label.setGraphicsEffect(glow)
        
        layout.addWidget(self.logo_label, 0, 0, alignment=Qt.AlignmentFlag.AlignLeft)

        # ================= TEAM NAME (CENTER) =================
        team_name = QLabel("RESQWINGS COMMAND")
        team_name.setAlignment(Qt.AlignmentFlag.AlignCenter)
        team_name.setStyleSheet("font-size: 30px; font-weight: 800; letter-spacing: 3px; color: #38bdf8;")
        layout.addWidget(team_name, 0, 0, 1, 3)

        # ================= INITIALIZE WIDGETS =================
        qgc_path = os.path.join(os.path.dirname(__file__), "QGroundControl.AppImage")
        self.qgc_widget = QGCLauncherWidget(app_path=qgc_path)
        
        self.rpi_drone1 = WebWidget(url="https://www.raspberrypi.com/", title="RPi CONNECT - DRONE 1")
        self.rpi_drone2 = WebWidget(url="https://www.raspberrypi.com/", title="RPi CONNECT - DRONE 2")
        self.cam_feed = VideoWidget(source=0, title="LIVE OPTICAL FEED - DRONE 1")
        
        # Survivor Telemetry Hub (Persistent Tracking)
        self.telemetry_hub = TelemetryHubWidget()

        # ================= GRID LAYOUT =================
        # Row 1: QGC (Spans 2 columns) | RPi Drone 2
        layout.addWidget(self.qgc_widget, 1, 0, 1, 2)
        layout.addWidget(self.rpi_drone2, 1, 2)
        
        # Row 2: RPi Drone 1 | Camera Feed | Telemetry Tracking
        layout.addWidget(self.rpi_drone1, 2, 0)
        layout.addWidget(self.cam_feed, 2, 1)
        layout.addWidget(self.telemetry_hub, 2, 2)

        # Stretch Factors
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 1)
        layout.setRowStretch(1, 2)
        layout.setRowStretch(2, 1)

def main():
    app = QApplication(sys.argv)

    # Load External CSS
    css_path = os.path.join(os.path.dirname(__file__), "style.css") # Ensure your css file is named style.css
    if os.path.exists(css_path):
        with open(css_path, "r") as f:
            app.setStyleSheet(f.read())

    window = GCSMainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()