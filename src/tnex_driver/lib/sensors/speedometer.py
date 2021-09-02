import rospy
from std_msgs.msg import Float64


class Speedometer:
    speed = 0.0

    def __init__(self):
        rospy.Subscriber('vehicle_speed', Float64, self.set_speed)

    def set_speed(self, msg):
        self.speed = msg.data

    def get_speed(self):
        return self.speed
