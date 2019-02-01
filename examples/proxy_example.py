import time
import random
from led_pwm_tlc5947_proxy import LedPwmTLC5947Proxy

dt = 0.1
dev_number = 0
number_of_led = 18

led_pwm = LedPwmTLC5947Proxy()

# Turn on leds one at a time
pwm_value = 1000
for i in range(number_of_led):
    led_pwm.set(dev_number,i,pwm_value)
    time.sleep(dt)

# Turn off leds one at a time (reverse order)
pwm_value = 0
for i in reversed(range(number_of_led)):
    led_pwm.set(dev_number,i,0)
    time.sleep(dt)

# Turn on leds one at a time random order
pwm_value = 2000
led_list = range(number_of_led)
random.shuffle(led_list)
for i in led_list:
    led_pwm.set(dev_number,i,pwm_value)
    time.sleep(dt)

# Turn off all leds at the same time
led_pwm.off()
