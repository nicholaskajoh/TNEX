#!/usr/bin/env python

import traceback
import rospy
from controller import PID
from sensors import Speedometer
from mission_control import vehicle_state
from tnex_driver.msg import VehicleControl, CruiseControlData


rospy.init_node('ctrlr_cruise')

class CruiseControl:
    freq = 5
    throttle = 0
    steer = 0
    brake = 0
    reverse = 0

    def __init__(self):
        rospy.Subscriber('vehicle_control', VehicleControl, self.set_controls)
        self.vc_pub = rospy.Publisher('vehicle_control', VehicleControl, queue_size=10)
        self.ccd_pub = rospy.Publisher('cruise_control_data', CruiseControlData, queue_size=50)

    def set_controls(self, vehicle_control):
        self.throttle = vehicle_control.throttle
        self.steer = vehicle_control.steer
        self.brake = vehicle_control.brake
        self.reverse = vehicle_control.reverse

    def create(self):
        rate = rospy.Rate(self.freq)
        pid = PID(0.35, 0.00625, 0.035)
        speedometer = Speedometer()

        while not rospy.is_shutdown():
            cc_enabled = bool(int(vehicle_state.get_state('cruise_control_enabled', '0')))
            target_speed = int(vehicle_state.get_state('cruise_control_target_speed', 0)) # in m/s
            pid.SetPoint = target_speed

            if cc_enabled:
                speed = speedometer.get_speed()
                pid.update(speed)
                throttle_brake = pid.output

                ccd_msg = CruiseControlData()
                ccd_msg.speed = speed
                ccd_msg.target_speed = target_speed
                ccd_msg.throttle_brake = throttle_brake
                self.ccd_pub.publish(ccd_msg)

                # 0 to 1 for throttle and -1 to 0 for brake
                # so throttle + brake range is 1 to -1
                if throttle_brake > 1:
                    throttle_brake = 1
                if throttle_brake < -1:
                    throttle_brake = -1
                if throttle_brake < 0:
                    self.brake = -1 * throttle_brake
                    self.throttle = 0
                else:
                    self.throttle = throttle_brake
                    self.brake = 0

                vc_msg = VehicleControl()
                vc_msg.throttle = self.throttle
                vc_msg.steer = self.steer
                vc_msg.brake = self.brake
                vc_msg.reverse = self.reverse
                self.vc_pub.publish(vc_msg)

            rate.sleep()

    def destroy(self):
        pass


try:
    cc = CruiseControl()
    rospy.on_shutdown(cc.destroy)
    cc.create()
except Exception:
    rospy.logerr(traceback.format_exc())
