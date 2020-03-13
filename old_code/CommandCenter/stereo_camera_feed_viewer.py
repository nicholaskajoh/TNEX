import redis
import numpy as np
import pickle
import time
import cv2


try:
    r = redis.Redis()
except redis.ConnectionError as e:
    print(e)

def display_image(image_message, window_name):
    if type(image_message['data']) is not int:
        image = pickle.loads(image_message['data'])
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        image = cv2.resize(image, (854, 480))
        cv2.imshow(window_name, image)
        cv2.waitKey(1)

def main():
    cam_left = r.pubsub()
    cam_left.subscribe('cam_left')

    cam_right = r.pubsub()
    cam_right.subscribe('cam_right')

    try:
        while True:
            cam_left_message = cam_left.get_message()
            cam_right_message = cam_right.get_message()

            if cam_left_message:
                display_image(cam_left_message, 'Cam left')

            if cam_right_message:
                display_image(cam_right_message, 'Cam right')

            time.sleep(0.0667)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
