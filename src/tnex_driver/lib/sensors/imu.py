import math
import rospy
from tnex_driver.msg import IMU


heading = None

def get_imu(message):
    global heading

    # get ego vehicle compass reading
    heading = message.compass

rospy.Subscriber('imu', IMU, get_imu)

def get_ego_vehicle_heading():
    return heading

def get_ego_vehicle_map_heading(compass_heading_in_radians, radius = 30):
    '''
    Get heading of ego vehicle wrt the map's north
    '''
    heading_in_degrees = math.degrees(compass_heading_in_radians)
    map_north_offset = 180 # map north is off compass north by 180 degrees
    # align map and compass norths
    if heading_in_degrees <= map_north_offset:
        heading_in_degrees = -1 * heading_in_degrees + map_north_offset
    else:
        heading_in_degrees = -1 * heading_in_degrees + 3 * map_north_offset

    if heading_in_degrees == 0 or heading_in_degrees == 360:
        return [heading_in_degrees, 0, radius]
    if heading_in_degrees == 90:
        return [heading_in_degrees, radius, 0]
    if heading_in_degrees == 180:
        return [heading_in_degrees, 0, -1 * radius]
    if heading_in_degrees == 270:
        return [heading_in_degrees, -1 * radius, 0]

    if heading_in_degrees > 0 and heading_in_degrees < 90: # 1st quadrant
        theta_x = heading_in_degrees
        theta_y = 90 - heading_in_degrees
        sign_x = 1 # positive sign
        sign_y = 1
    elif heading_in_degrees > 90 and heading_in_degrees < 180: # 4th quadrant
        theta_x = 180 - heading_in_degrees
        theta_y = heading_in_degrees - 90
        sign_x = 1
        sign_y = -1 # negative sign
    elif heading_in_degrees > 180 and heading_in_degrees < 270: # 3rd quadrant
        theta_x = heading_in_degrees - 180
        theta_y = 270 - heading_in_degrees
        sign_x = -1
        sign_y = -1
    elif heading_in_degrees > 270 and heading_in_degrees < 360: # 2nd quadrant
        theta_x = 360 - heading_in_degrees
        theta_y = heading_in_degrees - 270
        sign_x = -1
        sign_y = 1
    else:
        rospy.logerr('Invalid heading: ' + str(compass_heading_in_radians) + ' rad')
        return [heading_in_degrees, 0, radius]

    dx = radius * math.sin(math.radians(theta_x)) # sine rule
    dy = radius * math.sin(math.radians(theta_y))

    if (dx < 0 and sign_x > -1) or (dx > -1 and sign_x < 0): # i.e signs don't match
        dx = -1 * dx # change the sign
    if (dy < 0 and sign_y > -1) or (dy > -1 and sign_y < 0):
        dy = -1 * dy

    return [heading_in_degrees, dx, dy]
