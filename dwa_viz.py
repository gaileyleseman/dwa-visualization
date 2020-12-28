import sys
import random
import matplotlib
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QGridLayout, QWidget, QLabel, QSpinBox, QPushButton

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class MplCanvas(FigureCanvas):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)


class DWA_Viz(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(DWA_Viz, self).__init__(*args, **kwargs)
        self.setWindowTitle("Visualization Dynamic Window Approach")
        self.setGeometry(50, 50, 1000, 750)
        self.reached_goal = False

        # Widgets
        grid_size = 10
        self.start_x = QSpinBox()
        self.start_y = QSpinBox()
        self.goal_x = QSpinBox()
        self.goal_y = QSpinBox()
        self.start_x.setRange(0, grid_size)
        self.start_y.setRange(0, grid_size)
        self.goal_x.setRange(0, grid_size)
        self.goal_y.setRange(0, grid_size)
        self.start_btn = QPushButton("Start")
        self.reset_btn = QPushButton("Reset")
        self.start_btn.clicked.connect(self.start)
        self.reset_btn.clicked.connect(self.reset)

        self.canvas = MplCanvas(self, width=5, height=4, dpi=100)

        # Layout
        self.layout = QGridLayout()
        self.layout.addWidget(QLabel("Start (x,y):"), 0, 0)
        self.layout.addWidget(QLabel("Goal (x,y):"), 1, 0)
        self.layout.addWidget(self.start_x, 0, 1)
        self.layout.addWidget(self.start_y, 0, 2)
        self.layout.addWidget(self.goal_x, 1, 1)
        self.layout.addWidget(self.goal_y, 1, 2)
        self.layout.addWidget(self.start_btn, 2, 1)
        self.layout.addWidget(self.reset_btn, 2, 2)
        self.layout.addWidget(self.canvas, 0, 3, 6, 6)

        widget = QWidget()
        widget.setLayout(self.layout)

        self.setCentralWidget(widget)

        n_data = 50
        self.xdata = list(range(n_data))
        self.ydata = [random.randint(0, 10) for i in range(n_data)]
        self.update_plot()

        self.show()

        # Setup a timer to trigger the redraw by calling update_plot.
        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.path_planning)


    def update_plot(self):
        # Drop off the first y element, append a new one.
        self.ydata = self.ydata[1:] + [random.randint(0, 10)]
        self.canvas.axes.cla()  # Clear the canvas.
        self.canvas.axes.plot(self.xdata, self.ydata, 'r')
        # Trigger the canvas to update and redraw.
        self.canvas.draw()

    def start(self):
        self.timer.start()

    def reset(self):
        self.timer.stop()

    def path_planning(self):
        self.update_plot()


app = QtWidgets.QApplication(sys.argv)
w = DWA_Viz()
app.exec_()