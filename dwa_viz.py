import sys
import random
import matplotlib
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QGridLayout, QWidget, QLabel, QSpinBox, QPushButton

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Circle, Arrow, Arc, ConnectionPatch

from dwa import *


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
        self.p = get_params()
        self.reached_goal = False

        # Widgets
        self.start_x = QSpinBox()
        self.start_y = QSpinBox()
        self.goal_x = QSpinBox()
        self.goal_y = QSpinBox()
        self.start_x.setRange(0, self.p.grid_size)
        self.start_x.setValue(1)
        self.start_y.setValue(1)
        self.goal_x.setValue(self.p.grid_size - 1)
        self.goal_y.setValue(self.p.grid_size - 1)
        self.start_y.setRange(0, self.p.grid_size)
        self.goal_x.setRange(0, self.p.grid_size)
        self.goal_y.setRange(0, self.p.grid_size)
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

        # DWA
        self.viz = []
        self.obstacles = []
        self.paths = []

        self.update_plot()
        self.show()

        # Setup a timer to trigger the redraw by calling update_plot.
        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.path_planning)


    def update_plot(self):
        self.canvas.axes.cla()  # Clear the canvas.
        for patch in self.viz:
            self.canvas.axes.add_patch(patch)
        # Trigger the canvas to update and redraw.
        self.plot_layout()
        self.canvas.draw()

    def plot_layout(self):
        ax = self.canvas.axes
        ax.set_xlim(0, self.p.grid_size)
        ax.set_ylim(0, self.p.grid_size)
        ax.set_aspect('equal')

    def start(self):
        self.reset()
        self.init_objects()
        self.viz_objects()
        self.timer.start()

    def reset(self):
        self.obstacles = []
        self.timer.stop()

    def path_planning(self):
        if self.timer.isActive():
            if not self.reached_goal:
                window = dynamic_window(self.bot)
                self.paths = [RobotPath(self.bot, 0.01, -0.1)] #(self.bot, window, self.obstacles)
                optimal = find_optimum(self.bot, self.paths, self.goal_pos, self.p)
                self.paths.append(optimal)
                self.viz_objects()
                self.bot.update_state(optimal.v, optimal.omega)
                print(self.bot.v, self.bot.omega)
        self.update_plot()

    def init_objects(self):
        self.start_pos = (self.start_x.value(), self.start_y.value())
        self.goal_pos = (self.goal_x.value(), self.goal_y.value())
        self.bot = Robot(self.start_pos, self.p)

        for i in range(0, self.p.n_obstacles):
            x = random.randint(0, self.p.grid_size)
            y = random.randint(0, self.p.grid_size)
            self.obstacles.append(Obstacle(x, y, self.p.r_obstacle))


    def viz_objects(self):
        self.viz = []
        self.viz = generate_robot_viz(self.bot)
        self.viz.append((Circle(self.goal_pos, 0.2, color='limegreen')))
        for obstacle in self.obstacles:
            self.viz.append((Circle((obstacle.x, obstacle.y), obstacle.r, color='black')))
        for path in self.paths:
            viz_path = generate_path_viz(path, self.p.grid_size)
            self.viz.append(viz_path)



def generate_robot_viz(bot):
    r = bot.p.r_bot
    dx = r * math.cos(bot.theta)
    dy = r * math.sin(bot.theta)
    bot_body = Circle((bot.x, bot.y), r, color='cornflowerblue')
    bot_heading = Arrow(bot.x, bot.y, dx, dy, width=0.2, color='darkblue')
    return [bot_body, bot_heading]

def generate_path_viz(path, grid_size):
    if path.optimal:
        line_color = 'red'
    else:
        line_color = 'grey'

    if path.type == 'curved':
        path_viz = Arc((path.x, path.y), path.r * 2, path.r * 2, path.angle, path.start, path.end, color=line_color)
    else:
        x = min(max(path.x, 0), grid_size)
        y = min(max(path.y, 0), grid_size)
        path_viz = ConnectionPatch((path.xA, path.yA), (x, y), "data", "data", color=line_color)
    return path_viz

app = QtWidgets.QApplication(sys.argv)
w = DWA_Viz()
app.exec_()