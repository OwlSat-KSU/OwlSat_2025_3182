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
        self.emergency_shutdown = False  # Flag to track emergency shutdown state
        self.initUI()
        self.initSerial()

    def initUI(self):
        self.setWindowTitle("Ground Station Control")
        self.setGeometry(100, 100, 800, 500)

        # Main Layout with left (control) and right (sensors) panels
        main_layout = QHBoxLayout()
        left_panel = QVBoxLayout()
        right_panel = QVBoxLayout()

        # Mission Status as Tabs using QTabWidget
        mission_tabs = QTabWidget()
        mission_tabs.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        statuses = ["IDLE", "READY", "ASCENDING", "DESCENDING", "RETRIEVAL"]

        for status in statuses:
                # Create the Idle tab with system control and command details
                tab = QWidget()
                idle_layout = QVBoxLayout()

                # Radio connection status indicator
                self.radio_status_label = QLabel("Radio Connection: Disconnected")
                self.radio_status_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
                self.radio_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
                self.radio_status_label.setStyleSheet("background-color: red; color: white;")
                idle_layout.addWidget(self.radio_status_label)

                # Buttons for starting the system and lowering power
                btn_layout = QHBoxLayout()
                self.start_system_btn = QPushButton("Start System")
                self.start_system_btn.clicked.connect(self.start_system)
                self.lower_power_btn = QPushButton("Lower Power")
                self.lower_power_btn.clicked.connect(self.lower_power)
                btn_layout.addWidget(self.start_system_btn)
                btn_layout.addWidget(self.lower_power_btn)
                idle_layout.addLayout(btn_layout)

                # Command Details Section
                command_details_header = QLabel("Command Details:")
                command_details_header.setFont(QFont("Arial", 10, QFont.Weight.Bold))
                command_details_header.setAlignment(Qt.AlignmentFlag.AlignCenter)
                idle_layout.addWidget(command_details_header)

                # Create labels for the additional command details
                self.team_id_label = QLabel("TEAM_ID: ---")
                self.mission_time_label = QLabel("MISSION_TIME: ---")
                self.packet_count_label = QLabel("PACKET_COUNT: ---")
                self.mode_label = QLabel("MODE: ---")
                self.state_label = QLabel("STATE: ---")
                self.gps_time_label = QLabel("GPS_TIME: ---")
                self.gps_altitude_label = QLabel("GPS_ALTITUDE: ---")
                self.gps_latitude_label = QLabel("GPS_LATITUDE: ---")
                self.gps_longitude_label = QLabel("GPS_LONGITUDE: ---")
                self.gps_sats_label = QLabel("GPS_SATS: ---")
                self.cmd_echo_label = QLabel("CMD_ECHO: ---")

                # Set up each detail label and add to the idle layout
                details_labels = [
                    self.team_id_label, self.mission_time_label, self.packet_count_label,
                    self.mode_label, self.state_label, self.gps_time_label,
                    self.gps_altitude_label, self.gps_latitude_label,
                    self.gps_longitude_label, self.gps_sats_label, self.cmd_echo_label
                ]
                for label in details_labels:
                    label.setFont(QFont("Arial", 9))
                    label.setAlignment(Qt.AlignmentFlag.AlignLeft)
                    idle_layout.addWidget(label)

                tab.setLayout(idle_layout)
                mission_tabs.addTab(tab, status)

                # Simple placeholder tabs for other statuses
                #tab = QWidget()
                #tab_layout = QVBoxLayout()
                #status_label = QLabel(f"Current Status: {status}")
                #status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
                #status_label.setFont(QFont("Arial", 12))
                #tab_layout.addWidget(status_label)
                #tab.setLayout(tab_layout)
                #mission_tabs.addTab(tab, status)

        left_panel.addWidget(mission_tabs)

        # Control Buttons (outside the Idle tab)
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
        self.emergency_btn.clicked.connect(self.emergency_stop)

        # The Power button is used to re-engage the systems after an emergency stop.
        self.power_btn = QPushButton("Power")
        self.power_btn.setStyleSheet("background-color: red; color: white;")
        self.power_btn.clicked.connect(self.power_engage)

        button_layout.addWidget(self.calibrate_btn)
        button_layout.addWidget(set_time_btn)
        button_layout.addWidget(self.sim_mode_btn)
        button_layout.addWidget(command_btn)
        button_layout.addWidget(self.emergency_btn)
        button_layout.addWidget(self.power_btn)
        left_panel.addLayout(button_layout)

        # Sensor Data Panels (right panel remains as before)
        sensor_layout = QVBoxLayout()
        self.altitude_label = QLabel("Altitude (m): ---")
        self.temp_label = QLabel("Temperature (°C): ---")
        self.pressure_label = QLabel("Pressure (kPa): ---")
        self.magnet_label = QLabel("Magnetometer: ---")
        self.accel_label = QLabel("Accelerometer: ---")

        sensor_labels = [
            self.altitude_label, self.temp_label,
            self.pressure_label, self.magnet_label, self.accel_label
        ]
        for label in sensor_labels:
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

    def send_command(self, command):
        if self.ser:
            try:
                self.ser.write(command.encode() + b'\n')
                print(f"Sent command: {command}")
            except Exception as e:
                print("Error sending command:", e)
        else:
            print("Serial not connected.")

    def start_system(self):
        """Initiates a system check by sending a 'sys_check' command and updating the radio indicator."""
        self.radio_status_label.setStyleSheet("background-color: yellow; color: black;")
        self.radio_status_label.setText("Radio Connection: Checking...")
        self.send_command("sys_check")

    def lower_power(self):
        """Instructs the payload to enter low-power mode."""
        self.send_command("command-standby")

    def emergency_stop(self):
        """
        On Emergency Stop:
          - Sends 'emergency_stop' command.
          - Updates the radio indicator.
          - Disables key controls (except the Power button).
          - Stops sensor data updates.
        """
        self.send_command("emergency_stop")
        print("Emergency Stop Activated: Shutting down all systems.")

        self.radio_status_label.setStyleSheet("background-color: grey; color: white;")
        self.radio_status_label.setText("EMERGENCY STOP ACTIVATED")

        self.calibrate_btn.setEnabled(False)
        self.start_system_btn.setEnabled(False)
        self.lower_power_btn.setEnabled(False)
        self.sim_mode_btn.setEnabled(False)
        self.emergency_btn.setEnabled(False)

        self.altitude_label.setText("Altitude (m): OFF")
        self.temp_label.setText("Temperature (°C): OFF")
        self.pressure_label.setText("Pressure (kPa): OFF")
        self.magnet_label.setText("Magnetometer: OFF")
        self.accel_label.setText("Accelerometer: OFF")

        if self.timer.isActive():
            self.timer.stop()

        self.emergency_shutdown = True

    def power_engage(self):
        """
        Engages the systems back up:
          - Re-enables disabled controls.
          - Resets display labels.
          - Restarts the sensor data update timer.
          - Optionally sends a 'power_engage' command.
        """
        if self.emergency_shutdown:
            print("Power button pressed: Re-engaging systems...")
            self.calibrate_btn.setEnabled(True)
            self.start_system_btn.setEnabled(True)
            self.lower_power_btn.setEnabled(True)
            self.sim_mode_btn.setEnabled(True)
            self.emergency_btn.setEnabled(True)

            self.altitude_label.setText("Altitude (m): ---")
            self.temp_label.setText("Temperature (°C): ---")
            self.pressure_label.setText("Pressure (kPa): ---")
            self.magnet_label.setText("Magnetometer: ---")
            self.accel_label.setText("Accelerometer: ---")

            self.radio_status_label.setStyleSheet("background-color: red; color: white;")
            self.radio_status_label.setText("Radio Connection: Disconnected")

            if not self.timer.isActive():
                self.timer.start(100)

            self.send_command("power_engage")
            self.emergency_shutdown = False
        else:
            print("System is already online.")

    def read_serial(self):
        """Reads and parses a command packet from the serial port.
        If the packet contains 25 elements, sensor values update the right panel and
        additional command details update the Idle tab."""
        if self.ser and self.ser.in_waiting:
            try:
                line = self.ser.readline().decode().strip()
                print("Received:", line)
                
                # Check for radio test confirmation
                if "sys_check_ok" in line:
                    self.radio_status_label.setStyleSheet("background-color: green; color: white;")
                    self.radio_status_label.setText("Radio Connection: Connected")
                else:
                    parts = line.split(',')
                    if len(parts) >= 25:
                        # Update Idle tab "Command Details"
                        self.team_id_label.setText(f"TEAM_ID: {parts[0]}")
                        self.mission_time_label.setText(f"MISSION_TIME: {parts[1]}")
                        self.packet_count_label.setText(f"PACKET_COUNT: {parts[2]}")
                        self.mode_label.setText(f"MODE: {parts[3]}")
                        self.state_label.setText(f"STATE: {parts[4]}")
                        
                        # Update Sensor Panel (right panel)
                        self.altitude_label.setText(f"Altitude (m): {parts[5]}")
                        self.temp_label.setText(f"Temperature (°C): {parts[6]}")
                        self.pressure_label.setText(f"Pressure (kPa): {parts[7]}")
                        self.accel_label.setText(
                            f"Accelerometer: X={parts[12]}, Y={parts[13]}, Z={parts[14]}"
                        )
                        self.magnet_label.setText(
                            f"Magnetometer: X={parts[15]}, Y={parts[16]}, Z={parts[17]}"
                        )
                        
                        # Update additional idle details (GPS info and command echo)
                        self.gps_time_label.setText(f"GPS_TIME: {parts[19]}")
                        self.gps_altitude_label.setText(f"GPS_ALTITUDE: {parts[20]}")
                        self.gps_latitude_label.setText(f"GPS_LATITUDE: {parts[21]}")
                        self.gps_longitude_label.setText(f"GPS_LONGITUDE: {parts[22]}")
                        self.gps_sats_label.setText(f"GPS_SATS: {parts[23]}")
                        self.cmd_echo_label.setText(f"CMD_ECHO: {parts[24]}")

            except Exception as e:
                print("Parse error:", e)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = GroundStationUI()
    ui.show()
    sys.exit(app.exec())
