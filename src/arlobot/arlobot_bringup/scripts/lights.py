# This class is a wrapper for controlling lights on the robot.  There is no specific plan for lighting at this moment
# but with an 8-port relay on board and only three relays being used (Activity board, Left and Right motors) there is
# the capability to support some lighting

class LightsError(Exception):
    pass

class Lights:
    def __init__(self):
        pass