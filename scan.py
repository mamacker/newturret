import servo
import random
import pigpio
import time
pan = servo.Servo(12)
tilt = servo.Servo(13)
pi = pigpio.pi()
cur_pan = 0
cur_tilt = 0
tilt_direction = 1
pan_direction = 1

pi.write(23,0)
time.sleep(5)
while(1):
    pi.write(23,1)
    cur_pan = pan.value();
    cur_tilt = tilt.value();

    if (cur_pan <= -.8):
        pan_direction = 1
        cur_pan = -.8
    elif (cur_pan >= .9):
        pan_direction = -1
        cur_pan = .9

    if (cur_tilt <= -1):
        tilt_direction = 1
        cur_tilt = -1
    elif (cur_tilt >= .6):
        tilt_direction = -1
        cur_tilt =.6

    pan.value(cur_pan + (pan_direction * .01 * random.randint(1,5)))
    tilt.value(cur_tilt + (tilt_direction * .01 * random.randint(1,5)))
    time.sleep(.1);


pan.stop()
tilt.stop()
