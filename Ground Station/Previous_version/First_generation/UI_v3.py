import sys
import serial
from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout,
    QTabWidget, QFrame
)
from PyQt6.QtGui import QFont
from PyQt6.QtCore import Qt, QTimer

class GroundStationUI(QWidget):
    def __init__(self):
        super().__init__()
        self.ser = None
        self.initUI()
        self.initSerial()

    def initUI(self):
        self.setWindowTitle("Ground Station Control")
        self.setGeometry(100, 100, 800, 500)

        # Main Layout
        main_layout = QHBoxLayout()
        left_panel = QVBoxLayout()
        right_panel = QVBoxLayout()

        # Mission Status as Tabs
        mission_tabs = QTabWidget()
        mission_tabs.setFont(QFont("Arial", 10, QFont.Weight.Bold))

        # Define the statuses and create a corresponding tab for each
        statuses = ["IDLE", "READY", "ASCENDING", "DESCENDING", "RETRIEVAL"]
        for status in statuses:
            tab = QWidget()
            tab_layout = QVBoxLayout()
            # Example: each tab displays a simple label; you could add more widgets per page as needed
            status_label = QLabel(f"Current Status: {status}")
            status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            status_label.setFont(QFont("Arial", 12))
            tab_layout.addWidget(status_label)
            tab.setLayout(tab_layout)
            mission_tabs.addTab(tab, status)

        left_panel.addWidget(mission_tabs)

        # Control Buttons
        button_layout = QHBoxLayout()
        self.calibrate_btn = QPushButton("Calibrate")
        self.calibrate_btn.setStyleSheet("background-color: black; color: red;")
        self.calibrate_btn.clicked.connect(self.toggle_power)

        set_time_btn = QPushButton("Set Time")

        self.sim_mode_btn = QPushButton("Simulation Mode")
        self.sim_mode_btn.setStyleSheet("background-color: black; color: red;")
        self.sim_mode_btn.clicked.connect(self.toggle_power)

        command_btn = QPushButton("Command")

        self.emergency_btn = QPushButton("Emergency Stop")
        self.emergency_btn.setStyleSheet("background-color: red; color: white;")
        self.emergency_btn.clicked.connect(self.toggle_power)

        self.power_btn = QPushButton("Power")
        self.power_btn.setStyleSheet("background-color: red; color: white;")
        self.power_btn.clicked.connect(self.toggle_power)

        button_layout.addWidget(self.calibrate_btn)
        button_layout.addWidget(set_time_btn)
        button_layout.addWidget(self.sim_mode_btn)
        button_layout.addWidget(command_btn)
        button_layout.addWidget(self.emergency_btn)
        button_layout.addWidget(self.power_btn)

        left_panel.addLayout(button_layout)

        # Sensor Data Panels
        sensor_layout = QVBoxLayout()

        self.altitude_label = QLabel("Altitude (m): ---")
        self.temp_label = QLabel("Temperature (°C): ---")
        self.pressure_label = QLabel("Pressure (kPa): ---")
        self.magnet_label = QLabel("Magnetometer: ---")
        self.accel_label = QLabel("Accelerometer: ---")

        for label in [self.altitude_label, self.temp_label, self.pressure_label, self.magnet_label, self.accel_label]:
            label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            label.setFrameShape(QFrame.Shape.Box)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            sensor_layout.addWidget(label)

        gps_label = QLabel("GPS Coordinates")
        gps_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        gps_label.setFrameShape(QFrame.Shape.Box)
        gps_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        gps_coords = QLabel("Latitude: 6.048°\nLongitude: 9.341°")
        sensor_layout.addWidget(gps_label)
        sensor_layout.addWidget(gps_coords)

        right_panel.addLayout(sensor_layout)
        main_layout.addLayout(left_panel, 2)
        main_layout.addLayout(right_panel, 3)
        self.setLayout(main_layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)
        self.timer.start(100)

    def toggle_power(self):
        sender = self.sender()
        current_style = sender.styleSheet()
        if ("background-color: black" in current_style) or ("background-color: red" in current_style):
            sender.setStyleSheet("background-color: green; color: white;")
        else:
            sender.setStyleSheet("background-color: red; color: white;")

    def initSerial(self):
        try:
            self.ser = serial.Serial('COM11', 9600, timeout=1)  # Adjust COM port as needed
        except serial.SerialException as e:
            print("Serial error:", e)
            self.ser = None

    def estimate_altitude(self, pressure_hpa):
        sea_level_pressure = 1013.25
        return 44330.0 * (1.0 - (pressure_hpa / sea_level_pressure)**(1/5.255))

    def read_serial(self):
        if self.ser and self.ser.in_waiting:
            try:
                line = self.ser.readline().decode().strip()
                print("Received:", line)
                parts = line.split(',')

                if len(parts) >= 8:
                    temp = int(parts[0]) / 10.0
                    pressure = int(parts[1]) / 10.0  # in hPa
                    altitude = self.estimate_altitude(pressure)
                    mx, my, mz = int(parts[2]), int(parts[3]), int(parts[4])
                    ax, ay, az = int(parts[5]), int(parts[6]), int(parts[7])

                    self.temp_label.setText(f"Temperature (°C): {temp:.1f}")
                    self.pressure_label.setText(f"Pressure (kPa): {pressure / 10:.2f}")
                    self.altitude_label.setText(f"Altitude (m): {altitude:.1f}")
                    self.magnet_label.setText(f"Magnetometer: X={mx}, Y={my}, Z={mz}")
                    self.accel_label.setText(f"Accelerometer: X={ax}, Y={ay}, Z={az}")
            except Exception as e:
                print("Parse error:", e)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = GroundStationUI()
    ui.show()
    sys.exit(app.exec())
