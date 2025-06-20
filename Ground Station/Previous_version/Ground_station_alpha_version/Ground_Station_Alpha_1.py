import sys
import serial
import pyqtgraph as pg
import csv
import os

from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout,
    QTabWidget, QFrame, QGridLayout
)
from PyQt6.QtWidgets import QLineEdit
from PyQt6.QtGui import QFont
from PyQt6.QtCore import Qt, QTimer
from datetime import datetime
import pytz


class GroundStationUI(QWidget):
    def __init__(self):
        super().__init__()
        self.ser = None
        self.emergency_shutdown = False  # Flag to track emergency shutdown state
        self.initUI()
        self.initSerial()
        # plot variable
        self.time_data = [] # time
        self.altitude_data = [] # altitude
        self.voltage_data = [] # voltage
        self.temperture_data = [] #temperture
        self.pressure_data = [] #pressure
        self.accel_data_x = [] #acceleration_x
        self.accel_data_y = [] #accelration_y
        self.accel_data_z = [] #acceleration_z
        self.gyro_data_x = [] #gyroscope_x
        self.gyro_data_y = [] #gyroscope_y
        self.gyro_data_z = [] #gyroscope_z
        self.Magnetometer_data_x = [] #magnotometer_x
        self.Magnetometer_data_y = [] #magnotometer_y
        self.Magnetometer_data_z = [] #magnotometer_z

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
                #self.start_system_btn.clicked.connect(self.toggle_power)
                self.start_system_btn.clicked.connect(lambda: self.send_command("CMD,3182,CX,ON,"))
                self.lower_power_btn = QPushButton("Lower Power")
                self.lower_power_btn.clicked.connect(self.lower_power)
                #self.lower_power_btn.clicked.connect(self.toggle_power)
                self.lower_power_btn.clicked.connect(lambda: self.send_command("CMD,3182,CX,OFF,"))
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
                self.voltage_label = QLabel("Voltage: ---")
                self.Auto_gyro_label = QLabel("Auto Gyro rotation rate (deg/sec): ---")
                self.gps_time_label = QLabel("GPS_TIME: ---")
                self.gps_altitude_label = QLabel("GPS_ALTITUDE: ---")
                self.gps_latitude_label = QLabel("GPS_LATITUDE: ---")
                self.gps_longitude_label = QLabel("GPS_LONGITUDE: ---")
                self.gps_sats_label = QLabel("GPS_SATS: ---")
                self.cmd_echo_label = QLabel("CMD_ECHO: ---")

                # Set up each detail label and add to the idle layout
                details_labels = [
                    self.team_id_label, self.mission_time_label, self.packet_count_label,
                    self.mode_label, self.state_label,self.voltage_label, self.Auto_gyro_label ,self.gps_time_label,
                    self.gps_altitude_label, self.gps_latitude_label,
                    self.gps_longitude_label, self.gps_sats_label, self.cmd_echo_label
                ]
                for label in details_labels:
                    label.setFont(QFont("Arial", 9))
                    label.setAlignment(Qt.AlignmentFlag.AlignLeft)
                    idle_layout.addWidget(label)

                tab.setLayout(idle_layout)
                

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
        self.calibrate_btn.clicked.connect(lambda: self.send_command("CMD,3182,CAL,"))
        #button_layout.addWidget(self.calibrate_btn)

        self.Sensor_btn = QPushButton("Ready Mode")
        self.Sensor_btn.setStyleSheet("background-color: black; color: red;")
        self.Sensor_btn.clicked.connect(lambda: self.send_command("CMD,3182,READY,"))

        self.set_time_btn = QPushButton("Set Time")
        self.set_time_btn.setStyleSheet("background-color: black; color: red;")
        self.set_time_btn.clicked.connect(self.send_time)

        self.sim_mode_btn = QPushButton("Simulation Mode")
        self.sim_mode_btn.setStyleSheet("background-color: black; color: red;")
        #self.sim_mode_btn.clicked.connect(self.toggle_power)
        self.sim_mode_btn.clicked.connect(self.open_simulation_window)

        self.Mechanism_btn = QPushButton("Mechanism")
        self.Mechanism_btn.setStyleSheet("background-color: black; color: red;")
        self.Mechanism_btn.clicked.connect(self.open_mechaism_window)
        
        self.Plot_btn = QPushButton("Plots")
        self.Plot_btn.setStyleSheet = ("background-color: black; color: red;")
        self.Plot_btn.clicked.connect(self.open_plot_window)
        #self.emergency_btn = QPushButton("Emergency Stop")
        #self.emergency_btn.setStyleSheet("background-color: red; color: white;")
        #self.emergency_btn.clicked.connect(self.emergency_stop)

        # The Power button is used to re-engage the systems after an emergency stop.
        #self.power_btn = QPushButton("Power")
        #self.power_btn.setStyleSheet("background-color: red; color: white;")
        #self.power_btn.clicked.connect(self.power_engage)

        button_layout.addWidget(self.calibrate_btn)
        button_layout.addWidget(self.Sensor_btn)
        button_layout.addWidget(self.set_time_btn)
        button_layout.addWidget(self.sim_mode_btn)
        button_layout.addWidget(self.Mechanism_btn)
        button_layout.addWidget(self.Plot_btn)
        #button_layout.addWidget(self.power_btn)
        left_panel.addLayout(button_layout)

        # Sensor Data Panels (right panel remains as before)
        sensor_layout = QVBoxLayout()
        self.altitude_label = QLabel("Altitude (m): ---")
        self.temp_label = QLabel("Temperature (°C): ---")
        self.pressure_label = QLabel("Pressure (kPa): ---")
        self.magnet_label = QLabel("Magnetometer: ---")
        self.accel_label = QLabel("Accelerometer: ---")
        self.gyro_label = QLabel("Gyroscope: ---")

        sensor_labels = [
            self.altitude_label, self.temp_label,
            self.pressure_label, self.magnet_label, self.accel_label, self.gyro_label
        ]
        for label in sensor_labels:
            label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            label.setFrameShape(QFrame.Shape.Box)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            sensor_layout.addWidget(label)

        #gps_label = QLabel("GPS Coordinates")
        #gps_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        #gps_label.setFrameShape(QFrame.Shape.Box)
        #gps_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        #gps_coords = QLabel("Latitude: 6.048°\nLongitude: 9.341°")
        #sensor_layout.addWidget(gps_label)
        #sensor_layout.addWidget(gps_coords)

        right_panel.addLayout(sensor_layout)
        main_layout.addLayout(left_panel, 2)
        main_layout.addLayout(right_panel, 3)
        mission_tabs.addTab(tab, status)
        self.setLayout(main_layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)
        self.timer.start(100)


    def send_time(self):
        time_str = self.utc_time()
        command = f"CMD,3182,ST,{time_str},"
        self.send_command(command)

    def open_mechaism_window(self):
        self.mechaism_window = MechanismWindow(self)
        self.mechaism_window.show()

    def open_plot_window(self):
        self.plot_window = PlotWindow(self)
        self.plot_window.show()
    
    def open_simulation_window(self):
        self.simulation_window = SimulationWindow(self)
        self.simulation_window.show()

    def utc_time(self):
        # Assume current local time (e.g., US Eastern)
        local_tz = pytz.timezone("US/Eastern")
        local_time = datetime.now(local_tz)
        utc_time = local_time.astimezone(pytz.utc)
        return utc_time.strftime("%H:%M:%S")

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
                self.ser.write(command.encode())
                print(f"Sent command: {command}")
            except Exception as e:
                print("Error sending command:", e)
                print(f"Sent command: {command}")
        else:
            print("Serial not connected.")
            print(f"Sent command: {command}")

    def start_system(self):
        """Initiates a system check by sending a 'sys_check' command and updating the radio indicator."""
        self.radio_status_label.setStyleSheet("background-color: yellow; color: black;")
        self.radio_status_label.setText("Radio Connection: Checking...")
        #self.send_command("sys_check")

    def lower_power(self):
        """Instructs the payload to enter low-power mode."""
        #self.send_command("command-standby")

    def time_str_to_seconds(self,t):
        parts = list(map(int, t.split(":")))
        if len(parts) == 2:
            minutes, seconds = parts
            return minutes * 60 + seconds
        elif len(parts) == 3:
            hours, minutes, seconds = parts
            return hours * 3600 + minutes * 60 + seconds
        else:
            return 0  # Or raise an error

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
                        self.voltage_label.setText(f"Voltage (V):{parts[8]}")
                        # Update Sensor Panel (right panel)
                        self.altitude_label.setText(f"Altitude (m): {parts[5]}")
                        self.temp_label.setText(f"Temperature (°C): {parts[6]}")
                        self.pressure_label.setText(f"Pressure (kPa): {parts[7]}")
                        
                        self.gyro_label.setText(
                            f"Gyroscope (deg/s): Roll={parts[9]}, Pitch={parts[10]}, Yaw={parts[11]}"
                        )
                        self.accel_label.setText(
                            f"Accelerometer (m/s^2): Roll={parts[12]}, Pitch={parts[13]}, Yaw={parts[14]}"
                        )
                        self.magnet_label.setText(
                            f"Magnetometer (Gauss): Roll={parts[15]}, Pitch={parts[16]}, Yaw={parts[17]}"
                        )

                        
                        # Update additional idle details (GPS info, command echo, Auto Gyro)
                        self.Auto_gyro_label.setText(f"Auto Gyro rotation rate={parts[18]}")
                        self.gps_time_label.setText(f"GPS_TIME: {parts[19]}")
                        self.gps_altitude_label.setText(f"GPS_ALTITUDE: {parts[20]}")
                        self.gps_latitude_label.setText(f"GPS_LATITUDE: {parts[21]}")
                        self.gps_longitude_label.setText(f"GPS_LONGITUDE: {parts[22]}")
                        self.gps_sats_label.setText(f"GPS_SATS: {parts[23]}")
                        self.cmd_echo_label.setText(f"CMD_ECHO: {parts[24]}")

                    
                    telemetry_data = {
                            "Altitude": float(parts[5]),
                            "Voltage" : float(parts[8]),
                            "Temperture": float(parts[6]),
                            "Pressure" : float(parts[7]),
                            "acceleration_x": float(parts[12]),
                            "acceleration_y": float(parts[13]),
                            "acceleration_z": float(parts[14]),
                            "Gyroscope_x": float(parts[9]),
                            "Gyroscope_y": float(parts[10]),
                            "Gyroscope_z": float(parts[11]),
                            "Magnetometer_x": float(parts[15]),
                            "Magnetometer_y": float(parts[16]),
                            "Magnetometer_z": float(parts[17]),
                    }
                    #raw_time = parts[1]
                    #print(f"Raw time: {raw_time}")
                    time_seconds = self.time_str_to_seconds(parts[1])

                    self.time_data.append(time_seconds)
                    self.altitude_data.append(telemetry_data["Altitude"])
                    self.voltage_data.append(telemetry_data["Voltage"])
                    self.temperture_data.append(telemetry_data["Temperture"])
                    self.pressure_data.append(telemetry_data["Pressure"])
                    self.accel_data_x.append(telemetry_data["acceleration_x"])
                    self.accel_data_y.append(telemetry_data["acceleration_y"])
                    self.accel_data_z.append(telemetry_data["acceleration_z"])
                    self.gyro_data_x.append(telemetry_data["Gyroscope_x"])
                    self.gyro_data_y.append(telemetry_data["Gyroscope_y"])
                    self.gyro_data_z.append(telemetry_data["Gyroscope_z"])
                    self.Magnetometer_data_x.append(telemetry_data["Magnetometer_x"])
                    self.Magnetometer_data_y.append(telemetry_data["Magnetometer_y"])
                    self.Magnetometer_data_z.append(telemetry_data["Magnetometer_z"])

            except Exception as e:
                print("Parse error:", e)
    def store_data(self, parts):
        filename ="CanSatt2025_3128_trial.csv"

        headers = ["TEAM_ID", "MISSION_TIME", "PACKET_COUNT", "MODE", "STATE",
               "Altitude", "Temperature", "Pressure", "Voltage","Gyro_R","Gyro_P","Gyro_Y",
               "Accel_R","Accel_P","Accel_Y","Mag_R","Mag_p","Mag_Y","Auto_gyro_rotation_rate",
               "GPS_Time","GPS_ALTITUDE","GPS_LATITUDE","GPS_LONGITUDE","GPS_SATS", "CMD_ECHO"]
        
        row_data = [parts[0],parts[1],parts[2],parts[3],parts[4],parts[5],
                    parts[6],parts[7],parts[8],parts[9],parts[10],parts[11],parts[12],
                    parts[13],parts[14],parts[15],parts[16],parts[17],parts[18],parts[19],
                    parts[20],parts[21],parts[22],parts[23],parts[24]]
        
        with open(filename, mode='w',newline='')as file:
            writer = csv.writer(file)
            file.seek(0,2)
            if file.tell() == 0:
                writer.writerow(headers)
            writer.writerow(row_data)
        
            

class MechanismWindow(QWidget):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.setWindowTitle("Mechanism Controls")
        self.setGeometry(800, 400, 400, 600)
        
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Control Options"))

        self.pinRelase_btn = QPushButton("Pin release")
        self.pinRelease_btn_state = True
        self.pinRelase_btn.clicked.connect(self.toggle_pin_release)
        #pinRelase_btn.clicked.connect(self.controller.toggle_power)
        layout.addWidget(self.pinRelase_btn)

        self.Booms_btn = QPushButton("Booms Rotate")
        self.Booms_btn_State = True
        self.Booms_btn.clicked.connect(self.toggle_boom)
        layout.addWidget(self.Booms_btn)

        self.Beeper_btn = QPushButton("Beeper button")
        self.Beeper_btn_State = True
        self.Beeper_btn.clicked.connect(self.toggle_beeper)
        layout.addWidget(self.Beeper_btn)

            #button2 = QPushButton("Retract Mechanism")
            #button2.clicked.connect(lambda: print("Retract command sent"))
            #layout.addWidget(button2)
            
        self.setLayout(layout)

    def toggle_pin_release(self):
        if self.pinRelease_btn_state:
            self.controller.send_command("CMD.3182,MEC,PinRelease,OFF,")
            #self.pinRelease_btn.setText("Pin Release: OFF")
        else:
            self.controller.send_command("CMD.3182,MEC,PinRelease,ON,")
            #self.pinRelease_btn.setText("Pin Release: ON")
        self.pinRelease_btn_state = not self.pinRelease_btn_state

    def toggle_boom(self):
        if self.Booms_btn_State:
            self.controller.send_command("CMD,3182,MEC,BoomStick,OFF,")
            #self.pinRelease_btn.setText("Pin Release: OFF")
        else:
            self.controller.send_command("CMD,3182,MEC,BoomStick,ON,")
            #self.pinRelease_btn.setText("Pin Release: ON")
        self.Booms_btn_State = not self.Booms_btn_State 

    def toggle_beeper(self):
        if self.Beeper_btn_State:
            self.controller.send_command("CMD,3182,MEC,Beep,OFF,")
            #self.pinRelease_btn.setText("Pin Release: OFF")
        else:
            self.controller.send_command("CMD,3182,MEC,Beep,ON,")
            #self.pinRelease_btn.setText("Pin Release: ON")
        self.Beeper_btn_State = not self.Beeper_btn_State

class PlotWindow(QWidget):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.setWindowTitle("Plots")
        self.setGeometry(100, 20, 1500, 1000)
        
        layout = QGridLayout()

        # Create PlotWidget (altitude vs time)
        self.plot_widget_altitude = pg.PlotWidget(title="Altitude vs Mission Time")
        self.plot_widget_altitude.setLabel('left', 'Altitude', units='m')
        self.plot_widget_altitude.setLabel('bottom', 'Mission Time')
        self.plot_widget_altitude.showGrid(x=True, y=True)
        self.altitude_curve = self.plot_widget_altitude.plot(pen='y')

        # plot (voltage vs time)
        self.plot_widget_voltage = pg.PlotWidget(title="Voltage vs Mission Time")
        self.plot_widget_voltage.setLabel('left', 'Voltage', units='V')
        self.plot_widget_voltage.setLabel('bottom', 'Mission Time')
        self.plot_widget_voltage.showGrid(x=True, y=True)
        self.voltage_curve = self.plot_widget_voltage.plot(pen='r')

        #plot (temperture vs time)
        self.plot_widget_temperture = pg.PlotWidget(title="temperture vs Mission Time")
        self.plot_widget_temperture.setLabel('left', 'Temperture', units='°C')
        self.plot_widget_temperture.setLabel('bottom', 'Mission Time')
        self.plot_widget_temperture.showGrid(x=True, y=True)
        self.temperture_curve = self.plot_widget_temperture.plot(pen='g')

        #plot (pressure vs time)
        self.plot_widget_pressure = pg.PlotWidget(title="pressure vs Mission Time")
        self.plot_widget_pressure.setLabel('left', 'Pressure', units='kPa')
        self.plot_widget_pressure.setLabel('bottom', 'Mission Time')
        self.plot_widget_pressure.showGrid(x=True, y=True)
        self.pressure_curve = self.plot_widget_pressure.plot(pen='b')

        #plot (acceleration vs time)
        self.plot_widget_acceleration = pg.PlotWidget(title="Accelration vs Mission Time")
        self.plot_widget_acceleration.setLabel('left', 'accelration', units='m/s^2')
        self.plot_widget_acceleration.setLabel('bottom', 'Mission Time')
        self.plot_widget_acceleration.showGrid(x=True, y=True)
        legend = self.plot_widget_acceleration.addLegend()
        legend.setOffset((-100,50))
        #self.pressure_curve = self.plot_widget_pressure.plot(pen='b')  
        #self.plot_widget_accleration(x,)

        #plot (Gyroscope vs time)
        self.plot_widget_Gyroscope = pg.PlotWidget(title="Gyroscope vs Mission Time")
        self.plot_widget_Gyroscope.setLabel('left', 'Gyroscope', units='m/s')
        self.plot_widget_Gyroscope.setLabel('bottom', 'Mission Time')
        self.plot_widget_Gyroscope.showGrid(x=True, y=True)
        legend2 = self.plot_widget_Gyroscope.addLegend()
        legend2.setOffset((-100,50))

        #plot (Magnetommeter vs time) 
        self.plot_widget_Magnetommeter = pg.PlotWidget(title="Magnetommeter vs Mission Time")
        self.plot_widget_Magnetommeter.setLabel('left', 'Magnetommeter', units='gauss')
        self.plot_widget_Magnetommeter.setLabel('bottom', 'Mission Time')
        self.plot_widget_Magnetommeter.showGrid(x=True, y=True)
        legend3 = self.plot_widget_Magnetommeter.addLegend()
        legend3.setOffset((-100,50))

        layout.addWidget(self.plot_widget_altitude,0,0)
        layout.addWidget(self.plot_widget_voltage,0,1)
        layout.addWidget(self.plot_widget_pressure,1,0)
        layout.addWidget(self.plot_widget_temperture,1,1)
        layout.addWidget(self.plot_widget_acceleration,2,0)
        layout.addWidget(self.plot_widget_Gyroscope,2,1)
        layout.addWidget(self.plot_widget_Magnetommeter,2,2)
        self.setLayout(layout)

        # Optional: Use a timer to update plot periodically
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(1000)  # every 1 second

    def update_plot(self):
    # Pull latest data from controller
        x_labels = self.controller.time_data
        y_altitude = self.controller.altitude_data
        y_voltage = self.controller.voltage_data
        y_temperture = self.controller.temperture_data
        y_pressure = self.controller.pressure_data
        y_acceleration_x = self.controller.accel_data_x
        y_acceleration_y = self.controller.accel_data_y
        y_acceleration_z = self.controller.accel_data_z
        y_Gyroscope_x = self.controller.gyro_data_x
        y_Gyroscope_y = self.controller.gyro_data_y
        y_Gyroscope_z = self.controller.gyro_data_z
        y_Magnetommeter_x = self.controller.Magnetometer_data_x
        y_Magnetommeter_y = self.controller.Magnetometer_data_y
        y_Magnetommeter_z = self.controller.Magnetometer_data_z
        x_ticks = list(range(len(x_labels)))

    # Ensure data is valid before updating plot
        if not x_labels or not y_altitude or not y_voltage:
            print("Warning: No data available for plotting")
            return
        # Clear plots before updating
        self.plot_widget_altitude.clear()
        self.plot_widget_voltage.clear()
        self.plot_widget_pressure.clear()
        self.plot_widget_temperture.clear()
        self.plot_widget_acceleration.clear()
        self.plot_widget_Gyroscope.clear()
        self.plot_widget_Magnetommeter.clear()

        # Update plots using the correct widget attributes
        self.plot_widget_altitude.plot(x_ticks, y_altitude, pen='y')
        self.plot_widget_voltage.plot(x_ticks, y_voltage, pen='r')
        self.plot_widget_temperture.plot(x_ticks,y_temperture, pen="g")
        self.plot_widget_pressure.plot(x_ticks,y_pressure, pen='b')
        
        self.plot_widget_acceleration.plot(x_ticks,y_acceleration_x,pen='r', name="acceleration_x")
        self.plot_widget_acceleration.plot(x_ticks,y_acceleration_y,pen='g', name="acceleration_y")
        self.plot_widget_acceleration.plot(x_ticks,y_acceleration_z,pen='b', name="acceleration_z")
        
        self.plot_widget_Gyroscope.plot(x_ticks,y_Gyroscope_x, pen='r', name="Gyroscope_x")
        self.plot_widget_Gyroscope.plot(x_ticks,y_Gyroscope_y, pen='g', name="Gyroscope_y")
        self.plot_widget_Gyroscope.plot(x_ticks,y_Gyroscope_z, pen='b', name="Gyroscope_z")
        
        self.plot_widget_Magnetommeter.plot(x_ticks,y_Magnetommeter_x, pen='r', name="Magnetommeter_x")
        self.plot_widget_Magnetommeter.plot(x_ticks,y_Magnetommeter_y, pen='g', name="Magnetommeter_y")
        self.plot_widget_Magnetommeter.plot(x_ticks,y_Magnetommeter_z, pen='b', name="Magnetommeter_z")


class SimulationWindow(QWidget):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.setWindowTitle("Sim Controls")
        self.setGeometry(800, 400, 300, 300)

        layout = QVBoxLayout()

        # Initialize transmission state as OFF
        self.transmitting = True
        self.commands = []  # Store commands from the file
        self.current_index = 0  # Track which command to send next

        self.sim_btn = QPushButton("Simulate OFF/ON") 
        self.sim_btn_state = False 
        self.sim_btn.clicked.connect(self.sim_mode) 
        layout.addWidget(self.sim_btn)

        self.simp_btn = QPushButton("Start Transmission")
        self.simp_btn.clicked.connect(self.toggle_transmission)
        layout.addWidget(self.simp_btn)

        self.setLayout(layout)

        # Timer for fixed-interval transmission
        self.timer = QTimer()
        self.timer.timeout.connect(self.transmit_next_command)
        self.transmit_interval = 1000  # Transmit every 1 second

    def sim_mode(self): 
        """Toggle simulation ON/OFF.""" 
        if self.sim_btn_state: 
            self.controller.send_command("CMD.3182,SIM,ENABLE,") 
        else: 
            self.controller.send_command("CMD.3182,SIM,DISABLE,") 
        self.sim_btn_state = not self.sim_btn_state

    def toggle_transmission(self):
        """Start or stop command transmission."""
        if  not self.sim_btn_state:
            print("echo")
            return

        if self.transmitting:
            self.timer.stop()  # Stop transmission
            self.simp_btn.setText("Start Transmission")
        else:
            self.load_commands(r"C:\python code\cansat_2023_simp.txt")  # Load commands
            if self.commands:
                self.current_index = 0  # Reset to the first command
                self.timer.start(self.transmit_interval)  # Start transmission
                self.simp_btn.setText("Stop Transmission")
            else:
                print("Error: No valid commands found!")

        self.transmitting = not self.transmitting  # Toggle state

    def transmit_next_command(self):
        """Transmit the next command in the list."""
        if self.current_index < len(self.commands):
            command = self.commands[self.current_index]
            self.controller.send_command(command)
            self.current_index += 1
        else:
            print("All commands transmitted. Stopping...")
            self.timer.stop()
            self.simp_btn.setText("Start Transmission")
            self.transmitting = True  # Ensure state resets

    def load_commands(self, filename):
        """Loads commands from a file into a list."""
        if os.path.exists(filename):
            with open(filename, 'r') as file:
                self.commands = [line.strip() + ',' for line in file if line.strip()]
        else:
            print(f"Error: File '{filename}' not found.")
            self.commands = []


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = GroundStationUI()
    ui.show()
    sys.exit(app.exec())