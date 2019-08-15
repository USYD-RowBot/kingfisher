import sys
import time

import navio.pwm
import navio.util

navio.util.check_apm()

PWM_OUTPUT = 5
SERVO_MIN = 0.9 #ms
SERVO_MAX = 2.1 #ms

with navio.pwm.PWM(PWM_OUTPUT) as pwm:
    pwm.set_period(20)
    pwm.enable()

    while(True):
        pwm.set_duty_cycle(SERVO_MIN)
        time.sleep(1)
        pwm.set_duty_cycle(1.5)
        time.sleep(1)
        pwm.set_duty_cycle(SERVO_MAX)
        time.sleep(1)
