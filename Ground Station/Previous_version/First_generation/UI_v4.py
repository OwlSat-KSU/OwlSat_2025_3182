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
        self.emergency_shutdown = False  # Track whether the system is in a shutdown state
        self.initUI()
        self.initSerial()

    def initUI(self):
        self.setWindowTitle("Ground Station Control")
        self.setGeometry(100, 100, 800, 500)

        # Main Layout
        main_layout = QHBoxLayout()
        left_panel = QVBoxLayout()
        right_panel = QVBoxLayout()

        # Mission Status as Tabs using QTabWidget
        mission_tabs = QTabWidget()
        mission_tabs.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        statuses = ["IDLE", "READY", "ASCENDING", "DESCENDING", "RETRIEVAL"]

        for status in statuses:
            if status == "IDLE":
                # Create the Idle tab with specific controls
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

                tab.setLayout(idle_layout)
                mission_tabs.addTab(tab, status)
            else:
                # Placeholder tabs for other statuses
                tab = QWidget()
                tab_layout = QVBoxLayout()
                status_label = QLabel(f"Current Status: {status}")
                status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
                status_label.setFont(QFont("Arial", 12))
                tab_layout.addWidget(status_label)
                tab.setLayout(tab_layout)
                mission_tabs.addTab(tab, status)

        left_panel.addWidget(mission_tabs)

        # Control Buttons (other than the Idle tab buttons)
        button_layout = QHBoxLayout()
        self.calibrate_btn = QPushButton("Calibrate")
        self.calibrate_btn.setStyleSheet("background-color: black; color: red;")
        self.calibrate_btn.clicked.connect(self.toggle_power)

        set_time_btn = QPushButton("Set Time")

        self.sim_mode_btn = QPushButton("Simulation Mode")
        self.sim_mode_btn.setStyleSheet("background-color: black; color: red;")
        self.sim_mode_btn.clicked.connect(self.toggle_power)

        command_btn = QPushButton("Command")

        # Emergency Stop now calls the emergency_stop method
        self.emergency_btn = QPushButton("Emergency Stop")
        self.emergency_btn.setStyleSheet("background-color: red; color: white;")
        self.emergency_btn.clicked.connect(self.emergency_stop)

        # The Power button is now used to engage/restart the system.
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

        # Sensor Data Panels
        sensor_layout = QVBoxLayout()
        self.altitude_label = QLabel("Altitude (m): ---")
        self.temp_label = QLabel("Temperature (°C): ---")
        self.pressure_label = QLabel("Pressure (kPa): ---")
        self.magnet_label = QLabel("Magnetometer: ---")
        self.accel_label = QLabel("Accelerometer: ---")

        for label in [self.altitude_label, self.temp_label, self.pressure_label,
                      self.magnet_label, self.accel_label]:
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
        """
        Toggles styles for non-critical controls.
        This method is still used by other buttons (e.g. Calibrate, Simulation Mode) for a simple visual change.
        """
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
        """
        Initiates a system check.
        The radio connection indicator is updated to yellow ('Checking...'),
        and the command 'sys_check' is sent to the payload.
        """
        self.radio_status_label.setStyleSheet("background-color: yellow; color: black;")
        self.radio_status_label.setText("Radio Connection: Checking...")
        self.send_command("sys_check")

    def lower_power(self):
        """
        Transmits the 'command-standby' string to instruct the payload to enter low-power mode.
        """
        self.send_command("command-standby")

    def emergency_stop(self):
        """
        On Emergency Stop activation:
          - Sends the 'emergency_stop' command to the payload.
          - Updates the radio status indicator.
          - Disables most controls (except the Power button) to prevent further commands.
          - Stops the sensor reading timer.
        """
        self.send_command("emergency_stop")
        print("Emergency Stop Activated: Shutting down all systems.")

        # Update the radio status indicator
        self.radio_status_label.setStyleSheet("background-color: grey; color: white;")
        self.radio_status_label.setText("EMERGENCY STOP ACTIVATED")

        # Disable critical controls so that no further commands are issued
        self.calibrate_btn.setEnabled(False)
        self.start_system_btn.setEnabled(False)
        self.lower_power_btn.setEnabled(False)
        self.sim_mode_btn.setEnabled(False)
        self.emergency_btn.setEnabled(False)
        # Note: The Power button remains enabled so the system can be engaged

        # Update sensor information displays to show shutdown state
        self.altitude_label.setText("Altitude (m): OFF")
        self.temp_label.setText("Temperature (°C): OFF")
        self.pressure_label.setText("Pressure (kPa): OFF")
        self.magnet_label.setText("Magnetometer: OFF")
        self.accel_label.setText("Accelerometer: OFF")

        # Stop the timer that reads serial data
        if self.timer.isActive():
            self.timer.stop()

        self.emergency_shutdown = True

    def power_engage(self):
        """
        Engages the systems back up if an emergency stop has been activated.
        Once Power is pressed:
          - Re-enables key controls.
          - Restarts the sensor data timer.
          - Resets display labels to their default 'waiting' state.
          - Optionally, sends a command (e.g., 'power_engage') to notify the payload that systems are back online.
        """
        if self.emergency_shutdown:
            print("Power button pressed: Re-engaging systems...")

            # Re-enable critical controls (keep Power button active)
            self.calibrate_btn.setEnabled(True)
            self.start_system_btn.setEnabled(True)
            self.lower_power_btn.setEnabled(True)
            self.sim_mode_btn.setEnabled(True)
            self.emergency_btn.setEnabled(True)

            # Reset sensor data displays
            self.altitude_label.setText("Altitude (m): ---")
            self.temp_label.setText("Temperature (°C): ---")
            self.pressure_label.setText("Pressure (kPa): ---")
            self.magnet_label.setText("Magnetometer: ---")
            self.accel_label.setText("Accelerometer: ---")

            # Reset the radio connection indicator back to default (disconnected)
            self.radio_status_label.setStyleSheet("background-color: red; color: white;")
            self.radio_status_label.setText("Radio Connection: Disconnected")

            # Restart the sensor data timer if not active.
            if not self.timer.isActive():
                self.timer.start(100)
            
            # Optionally, send a command to the payload to indicate reactivation.
            self.send_command("power_engage")
            self.emergency_shutdown = False
        else:
            print("System is already online.")

    def estimate_altitude(self, pressure_hpa):
        sea_level_pressure = 1013.25
        return 44330.0 * (1.0 - (pressure_hpa / sea_level_pressure)**(1/5.255))

    def read_serial(self):
        if self.ser and self.ser.in_waiting:
            try:
                line = self.ser.readline().decode().strip()
                print("Received:", line)
                
                # Check for confirmation of the radio system test
                if "sys_check_ok" in line:
                    self.radio_status_label.setStyleSheet("background-color: green; color: white;")
                    self.radio_status_label.setText("Radio Connection: Connected")
                else:
                    # Handle sensor data if available.
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
