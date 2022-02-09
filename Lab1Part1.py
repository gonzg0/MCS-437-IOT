import picar_4wd as picar
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
import time
import sys
import tty
import termios

servo = Servo(PWM("P0"), offset=0)
ultrasonic = Ultrasonic(Pin('D8'), Pin('D9'))
FORWARD_SPEED = 10
BACKWARD_SPEED = 10
TURN_SPEED = 40
SERVO_STEP = 18
ULTRASONIC_DETECTION_DISTANCE = 35
SERVO_MAX_ANGLE = 90
SERVO_ZERO_ANGLE = 0
SERVO_MIN_ANGLE = -90
SERVO_TIME = 0.5
servo_currentAngle = SERVO_ZERO_ANGLE
       
def set_servo_angle(angle: int):
    global servo_currentAngle
    servo_currentAngle = angle
    servo.set_angle(angle)

def get_servo_angle():
    global servo_currentAngle
    return(servo_currentAngle)

def get_status_at(angle, servo_speed=SERVO_TIME):
    set_servo_angle(angle)
    time.sleep(servo_speed)
    return ultrasonic.get_distance()

def perform_one_sweep(detection_distance=30, servo_speed=SERVO_TIME):
    global SERVO_STEP
    detection_list = []
    scan_info = []
    start_angle = get_servo_angle()
    next_angle = get_servo_angle()
    
    if(start_angle >= SERVO_MAX_ANGLE):
        servo_delta = -SERVO_STEP
    elif(start_angle <= SERVO_MIN_ANGLE):
        servo_delta = SERVO_STEP
    else:
        servo_delta = 0

    while ((start_angle == SERVO_MIN_ANGLE and next_angle <= SERVO_MAX_ANGLE) or
          (start_angle == SERVO_MAX_ANGLE and next_angle >= SERVO_MIN_ANGLE)):
        
        distance = get_status_at(next_angle, servo_speed=servo_speed)
        isDetected = 1 if distance <= detection_distance and not distance == -2 else 0
        detection_list.append(isDetected)       
        next_angle = get_servo_angle() + servo_delta
 
    if(servo_delta < 0):
        detection_list.reverse()
    return (detection_list)
   
def decide_movement(Isdetected):
    isClearAhead = not sum(Isdetected[3:7])
    isClearLeft = not sum(Isdetected[0:2])
    isClearRight = not sum(Isdetected[8:10])

    if(isClearAhead):
        picar.forward(FORWARD_SPEED)
    else:
        if(isClearRight):
            picar.turn_right(TURN_SPEED)
        elif(isClearLeft):
            picar.turn_left(TURN_SPEED)
        else:
            picar.backward(BACKWARD_SPEED)
       
if __name__ == '__main__':
    try:
        set_servo_angle(SERVO_MIN_ANGLE)
        
        #While ESC key is not pressed
        while (True):
            #get ultrasonic sweep data
            IsDetected = perform_one_sweep(ULTRASONIC_DETECTION_DISTANCE, servo_speed=0.02)               
            decide_movement(IsDetected)

    finally:
        picar.stop()
        set_servo_angle(SERVO_ZERO_ANGLE)