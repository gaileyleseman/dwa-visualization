import yaml
import math

class Params:
    def __init__(self, config):
        for param, value in config.items():
            setattr(self, param, value)     # sett all config parameters as attributes

def get_params():
    # convert config yaml to dictionairy
    with open('config.yaml') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        p = Params(config)
    return p

class Robot:
    def __init__(self, start_pos, params):
        # initialize robot state
        self.x = start_pos[0]
        self.y = start_pos[1]
        self.theta = self.v = self.omega = 0
        self.p = params

    def update_state(self, v, omega):
        self.v = v
        self.omega = omega
        self.theta += self.omega * self.p.dt
        if self.omega == 0:     # straight line
            self.x += self.v * math.cos(self.theta) * self.p.dt
            self.y += self.omega + math.sin(self.theta) * self.p.dt
        else:                   # circular trajectory
            self.x += (self.v/self.omega) * (math.sin(self.theta) + math.sin(self.theta + self.omega * self.p.dt))
            self.y += -(self.v/self.omega) * (math.cos(self.theta) + math.cos(self.theta + self.omega * self.p.dt))


class Obstacle:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

