import sys
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QGridLayout, QWidget, QLabel, QSpinBox, QPushButton

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Circle, Arrow, Arc, ConnectionPatch

class MplCanvas(FigureCanvas):
    def __init__(self, width=8, height=8, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)

class DWA_Viz(QMainWindow):
    def __init__(self):
        super(DWA_Viz, self).__init__()
        self.setWindowTitle("Visualization Dynamic Window Approach")
        self.setGeometry(50, 50, 1000, 750)

        grid_size = 10
        self.start_x = QSpinBox()
        self.start_y = QSpinBox()
        self.goal_x = QSpinBox()
        self.goal_y = QSpinBox()
        self.start_x.setRange(0, grid_size)
        self.start_y.setRange(0, grid_size)
        self.goal_x.setRange(0, grid_size)
        self.goal_y.setRange(0, grid_size)
        self.start = QPushButton("Start")
        self.reset = QPushButton("Reset")

        self.canvas = MplCanvas(width=8, height=8, dpi=100)

        self.layout = QGridLayout()
        self.layout.addWidget(QLabel("Start: (x,y)"), 0, 0)
        self.layout.addWidget(QLabel("Goal: (x,y)"), 1, 0)
        self.layout.addWidget(self.start_x, 0, 1)
        self.layout.addWidget(self.start_y, 0, 2)
        self.layout.addWidget(self.goal_x, 1, 1)
        self.layout.addWidget(self.goal_y, 1, 2)
        self.layout.addWidget(self.start, 2, 1)
        self.layout.addWidget(self.reset, 2, 2)
        self.layout.addWidget(self.canvas, 0, 3, 6, 6)

        widget = QWidget()
        widget.setLayout(self.layout)
        self.setCentralWidget(widget)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = DWA_Viz()
    window.show()
    sys.exit(app.exec_())