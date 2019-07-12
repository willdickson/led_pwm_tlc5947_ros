#!/usr/bin/env python
from __future__ import print_function
import rospy
import threading
import std_msgs.msg

from led_pwm_tlc5947 import PwmController
from led_pwm_tlc5947 import PwmControllerException

from led_pwm_tlc5947_ros.msg import LedPwmInfo
from led_pwm_tlc5947_ros.srv import LedPwmSet 
from led_pwm_tlc5947_ros.srv import LedPwmSetResponse
from led_pwm_tlc5947_ros.srv import LedPwmOff 
from led_pwm_tlc5947_ros.srv import LedPwmOffResponse

class LedPwmTLC5947(object):

    def __init__(self):

        self.port = rospy.get_param('port', '/dev/ttyACM0')
        rospy.init_node('led_pwm_tlc5947')
        self.lock = threading.Lock()
        with self.lock:
            self.controller = PwmController(self.port)
        self.number_of_devices = self.controller.get_number_of_devices()
        self.set_srv = rospy.Service('led_pwm_set', LedPwmSet, self.on_led_pwm_set)
        self.off_srv = rospy.Service('led_pwm_off', LedPwmOff, self.on_led_pwm_off)
        self.info_pub = rospy.Publisher('led_pwm_info', LedPwmInfo, queue_size=10) 

    def on_led_pwm_set(self,req): 
        ok = True
        msg = ''
        with self.lock:
            try:
                self.controller.set_one(req.dev_number, req.led_number, req.led_value)
            except PwmControllerException, exception:
                ok = False
                msg = str(exception)

        info_msg = LedPwmInfo()
        info_msg.header = std_msgs.msg.Header()
        info_msg.header.stamp = rospy.Time.now()
        info_msg.success = ok
        info_msg.message = msg
        info_msg.command = 'led_pwm_set'
        info_msg.dev_number = req.dev_number
        info_msg.led_number = req.led_number
        info_msg.led_value = req.led_value
        self.info_pub.publish(info_msg)
        return LedPwmSetResponse(ok,msg)

    def on_led_pwm_off(self,req):
        ok = True
        msg = ''
        for i in range(self.number_of_devices):
            with self.lock:
                try:
                    self.controller.set_all(i,0)
                except PwmControllerException, exception:
                    ok = False
                    msg = str(exception)
                    break

        info_msg = LedPwmInfo()
        info_msg.header = std_msgs.msg.Header()
        info_msg.header.stamp = rospy.Time.now()
        info_msg.success = ok
        info_msg.message = msg
        info_msg.command = 'led_pwm_off'
        info_msg.dev_number = 0
        info_msg.led_number = 0
        info_msg.led_value = 0 
        self.info_pub.publish(info_msg)
        return LedPwmOffResponse(ok,msg)


    def run(self):
        while not rospy.is_shutdown(): 
            rospy.sleep(0.1)


# ---------------------------------------------------------------------------------------

if __name__ == '__main__':

    led_pwm = LedPwmTLC5947()
    led_pwm.run()



 
