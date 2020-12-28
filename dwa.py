import yaml

class Params:
    def __init__(self, config):
        for param, value in config.items():
            setattr(self, param, value)     # sett all config parameters as attributes

def get_params():
    # convert config yaml to dictionairy
    with open('config.yaml') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        p = Params(config)
    return

