#!/usr/bin/env python
from __future__ import print_function
import rospy
import time

from led_pwm_tlc5947_ros.srv import LedPwmSet 
from led_pwm_tlc5947_ros.srv import LedPwmOff 


class LedPwmTLC5947ProxyException(Exception):
    pass


class LedPwmTLC5947Proxy(object):

    def __init__(self,namespace=None):
        self.namespace = namespace
        if self.namespace is None:
            set_srv_name = 'led_pwm_set'
            off_srv_name = 'led_pwm_off'
        else:
            set_srv_name = '/{}/led_pwm_set'.format(self.namespace)
            off_srv_name = '/{}/led_pwm_off'.format(self.namespace)
        rospy.wait_for_service(set_srv_name)
        rospy.wait_for_service(off_srv_name)
        self.set_proxy = rospy.ServiceProxy(set_srv_name,LedPwmSet)
        self.off_proxy = rospy.ServiceProxy(off_srv_name,LedPwmOff)

    def set(self,dev_number, led_number, led_value):
        self.set_proxy(dev_number, led_number, led_value)

    def off(self):
        self.off_proxy()
