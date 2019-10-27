from __future__ import print_function
import roslibpy
from cv_bridge import CvBridge, CvBridgeError
import cv2


def main(client):
    bridge = CvBridge()

    def display_image(image_message, window_name):
        try:
            image = bridge.imgmsg_to_cv2(image_message, 'rgb8')
        except CvBridgeError as e:
            print(e)
        image = cv2.resize(image, (854, 480))
        cv2.imshow(window_name, image)
        cv2.waitKey(1)

    # print(client.get_topics(None))

    cam_left_listener = roslibpy.Topic(client, '/camera_left', 'sensor_msgs/Image')
    cam_left_listener.subscribe(lambda msg: display_image(msg, 'Left cam'))
    # print(cam_left_listener.is_subscribed)

    cam_right_listener = roslibpy.Topic(client, '/camera_right', 'sensor_msgs/Image')
    cam_right_listener.subscribe(lambda msg: display_image(msg, 'Right cam'))

try:
    ros_client = roslibpy.Ros(host='localhost', port=9090)
    ros_client.on_ready(lambda: main(ros_client))
    ros_client.run_forever()
except KeyboardInterrupt:
    ros_client.terminate()
