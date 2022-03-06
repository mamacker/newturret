import pigpio
from time import sleep

# connect to the 
pi = pigpio.pi()

class Servo:
    def __init__(self, pin):
        self.pin = pin
        self.current_pos = 0
    def value(self, value = None):
        if (value == None):
            return self.current_pos;

        self.current_pos = value
        if value < 0:
            pos = 1500 + (500 * value)
        else:
            pos = 1500 + (value * 500)

        pi.set_servo_pulsewidth(self.pin, pos);
        
    def stop(self):
        pi.set_servo_pulsewidth(self.pin, 0)    # off

