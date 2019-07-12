import time
import random
from led_pwm_tlc5947_proxy import LedPwmTLC5947Proxy

dt = 0.1
dev_number = 0
number_of_led = 18

led_pwm = LedPwmTLC5947Proxy()

led_pwm.set(dev_number,22,255);
time.sleep(1)
led_pwm.off()


