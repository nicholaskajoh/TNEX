#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import glob
import os
import sys

try:
    sys.path.append(glob.glob('%s/dist/carla-*%d.%d-%s.egg' % (
        os.path.dirname(os.path.realpath(__file__)),
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import numpy as np

bridge = CvBridge()


def publish(carla_image, topic):
    image = np.frombuffer(carla_image.raw_data, dtype=np.dtype('uint8'))
    image = np.reshape(image, (carla_image.height, carla_image.width, 4))
    image = image[:, :, :3]
    image = image[:, :, ::-1]
    image_message = bridge.cv2_to_imgmsg(image, 'rgb8')
    topic.publish(image_message)


def carla_video_capture():
    rospy.init_node('carla_video_capture')

    cam_left_topic = rospy.Publisher('camera_left', Image, queue_size=50)
    cam_right_topic = rospy.Publisher('camera_right', Image, queue_size=50)
    cam_full_topic = rospy.Publisher('camera_full', Image, queue_size=50) # left right combo

    # create simulator client
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    # get simulator world
    world = client.get_world()

    # get camera sensors
    cam_left = None; cam_right = None
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == 'autonomous_car_camera_left':
            cam_left = actor

        if actor.attributes.get('role_name') == 'autonomous_car_camera_right':
            cam_right = actor
    if cam_left is None or cam_right is None:
        raise Exception('Could not find camera sensors')


    cam_left.listen(lambda carla_image: publish(carla_image, cam_left_topic))
    cam_right.listen(lambda carla_image: publish(carla_image, cam_right_topic))

    rospy.loginfo('Streaming video from cameras')

    rate = rospy.Rate(24) # 24hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        carla_video_capture()
    except rospy.ROSInterruptException:
        pass
