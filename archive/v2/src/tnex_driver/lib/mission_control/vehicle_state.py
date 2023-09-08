import redis
import rospy


try:
    r = redis.Redis()
except redis.ConnectionError as e:
    rospy.logerr(e)

prefix = 'tnex_driver:'

def clear():
    for key in r.scan_iter(prefix + '*'):
        r.delete(key)
    rospy.loginfo('Vehicle state cleared.')

def get_state(key, default=None):
    try:
        value = r.get(prefix + key)
        if value is None:
            return default
        return str(value, 'utf-8')
    except Exception as e:
        rospy.logerr('Unable to get state: ' + str(e))

def set_state(key, value):
    try:
        value = str(value)
        r.set(prefix + key, value)
        rospy.loginfo('State ' + key + ' set to ' + value)
    except Exception as e:
        rospy.logerr('Unable to set state: ' + str(e))
