import rospy
from tnex_driver.msg import GNSSMeasurement


ego_vehicle_location = None

def get_gnss(message):
    global ego_vehicle_location

    # get ego vehicle location from GNSS sensor
    ego_vehicle_location = message.location

rospy.Subscriber('gnss', GNSSMeasurement, get_gnss)

def get_ego_vehicle_location():
    return ego_vehicle_location
