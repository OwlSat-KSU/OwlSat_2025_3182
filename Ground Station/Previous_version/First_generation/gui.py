import sys
#import random
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class LivePlotter(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 Live Data Plot")
        self.setGeometry(100, 100, 800, 600)
        
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        
        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)
        
        self.canvas = FigureCanvas(plt.figure())
        self.layout.addWidget(self.canvas)
        
        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.toggle_stream)
        self.layout.addWidget(self.start_button)
        
        self.ax = self.canvas.figure.add_subplot(111)
        self.x_data = []
        self.y_data = []
        self.running = False
        self.ani = animation.FuncAnimation(self.canvas.figure, self.update_plot, interval=500)
        
        # Placeholder for serial connection (update COM port & baud rate accordingly)
        # self.ser = serial.Serial('COM3', 115200, timeout=1)
    
    def toggle_stream(self):
        self.running = not self.running
        self.start_button.setText("Stop" if self.running else "Start")
    
    def update_plot(self, frame):
        if self.running:
            # Simulated data (Replace this with serial reading)
            new_data = random.uniform(0, 10)  # Placeholder for STM32 data
            #self.ser = serial.Serial('COM3', 115200, timeout=1)
            self.x_data.append(len(self.x_data))
            self.y_data.append(new_data)
            
            # Keep data window limited to last 50 points
            self.x_data = self.x_data[-50:]
            self.y_data = self.y_data[-50:]
            
            self.ax.clear()
            self.ax.plot(self.x_data, self.y_data, marker='o')
            self.ax.set_title("Live Data Stream")
            self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LivePlotter()
    window.show()
    sys.exit(app.exec())
