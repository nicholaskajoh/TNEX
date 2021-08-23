import redis
import rospy


try:
    r = redis.Redis()
except redis.ConnectionError as e:
    rospy.logerr(e)

def clear_vehicle_state():
    for key in r.scan_iter('tnex_driver:*'):
        r.delete(key)
    rospy.loginfo('Vehicle state cleared.')
