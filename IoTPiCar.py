import picar_4wd as picar
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
#import utils, constants
import time, math

servo = Servo(PWM("P0"), offset=0)
ultrasonic = Ultrasonic(Pin('D8'), Pin('D9'))
WHEEL_POWER = 100
ROTATION_SPEED = 30
THIRTY_FIVE_CM = 20
TEN_CM = 10
SERVO_SLEEP = 0.08
SERVO_STEP = 18
DETECTION_DISTANCE = 30
SERVO_MAX_ANGLE = 90
SERVO_ZERO_ANGLE = 0
SERVO_MIN_ANGLE = -90
SERVO_TIME = 0.5

def isDetected(distances: list):
    for distance in distances:
        if 'found' in distance:
            return True
    return False

def detectionSubset(distances: list):
    MAX_RANGE = 19
    detection_list = []
    for distance in distances:
        if 'current-angle' in distance and abs(distance['current-angle']) < MAX_RANGE:
            detection_list.append(distance)
    return detection_list

def calculate_cycles_from_distance(distance: int):
    # five centimeters
    base_distance = {'distance': 5, 'cycles': 2}
    cycleCount = math.floor(distance / base_distance["distance"])
    print('cycleCount\n', cycleCount)
    return cycleCount * base_distance["cycles"]


def move(cycles, movement_type='linear', direction='forward'):
    movement = {
        'linear': {
            'forward': picar.forward,
            'backward': picar.backward
        },
        'rotation': {
            'left': picar.turn_left,
            'right': picar.turn_right
        }
    }

    movement_power = WHEEL_POWER if movement_type == 'linear' else ROTATION_SPEED
    movement[movement_type][direction](movement_power)

    for i in range(cycles):
        time.sleep(0.1)
    picar.stop()
    return True


# def move_until_sensor_detects_at(cm: int):
#     while(picar.us.get_distance() <= cm):
#         picar.forward(constants.WHEEL_POWER)
#     picar.stop()

def get_status_at(angle, servo_speed=SERVO_TIME):
    servo.set_angle(angle)
    time.sleep(servo_speed)
    return ultrasonic.get_distance()


angle_distance = [0, 0]
current_angle = 0
current_servo_position = SERVO_STEP
max_angle = 90
min_angle = -90


def perform_stationary_scan(detection_distance=30, servo_speed=SERVO_TIME):
    global SERVO_STEP
    distances = []
    detection_list = []
    servo.set_angle(SERVO_ZERO_ANGLE)
    time.sleep(servo_speed)
    servo.set_angle(SERVO_MAX_ANGLE)
    current_angle = SERVO_MAX_ANGLE
    time.sleep(servo_speed)
    while len(distances) <= 10 and current_angle >= SERVO_MIN_ANGLE:
        distance = get_status_at(current_angle, servo_speed=servo_speed)
        isDetected = 1 if distance <= detection_distance and not distance == -2 else 0
        distances.append({'distance': distance, 'current-angle': current_angle, 'detection': isDetected})
        detection_list.append(isDetected)
        current_angle -= SERVO_STEP
    servo.set_angle(SERVO_ZERO_ANGLE)

    return (distances, detection_list)

def perform_moving_scan(detection_distance=30, servo_speed=SERVO_TIME):
    global SERVO_STEP
    servo_delta = SERVO_STEP
    detection_list = []
    servo.set_angle(SERVO_ZERO_ANGLE)
    current_angle = SERVO_ZERO_ANGLE
    time.sleep(servo_speed)

    while sum(detection_list) <= 0:
        distance = get_status_at(current_angle, servo_speed=servo_speed)
        isDetected = 1 if distance <= detection_distance and not distance == -2 else 0
        detection_list.append(isDetected)
        current_angle += servo_delta

        if(current_angle > SERVO_MAX_ANGLE):
            servo_delta = -servo_delta
            print('servo max angle')
        if(current_angle < SERVO_MIN_ANGLE):
            print('servo min angle', servo_delta)
            servo_delta = abs(servo_delta)

        print('current angle', current_angle)

    print('detection list', detection_list)
    servo.set_angle(SERVO_ZERO_ANGLE)

    return True
# def self_drive():
#     servo.set_angle(0)
#     picar.forward(constants.WHEEL_POWER)
#     isObjectFound = False
#     while not isObjectFound:
#         print('running wheels! lol')
#         scan_info = scan_step()
#         if(scan_info):
#             scan_subset = utils.detectionSubset(scan_info)

#         if(scan_info):
#             print('scaninfo %s:', scan_info)
#             print('subset %s', scan_subset)
#             wasItemDetected = utils.isDetected(scan_subset)
#             if(wasItemDetected and wasItemDetected):
#                 print('is detected halting')
#                 isObjectFound = True
#                 picar.stop()
#     picar.stop()
#     servo.set_angle(0)

def decide_movement(scan_results: list):
    detection = scan_results[1]
    forward_domain = detection[3:7]
    left_domain = detection[0:2]
    right_domain = detection[8:10]

    print(scan_results[1])
    isSomethingAhead = sum(forward_domain) >= 1
    isSomethingLeft = sum(left_domain) >= 1
    isSomethingRight = sum(right_domain) >= 1
    if(isSomethingAhead and isSomethingLeft):
        move(5, movement_type='rotation', direction='right')
    if(isSomethingAhead and isSomethingRight):
        move(5, movement_type='rotation', direction='left')

    detection_distances = []
    safe_forward_cycles = 0
    print('scan result', scan_results[0][3:8])
    for scan_result in scan_results[0][3:7]:
        if(not scan_result['distance'] == -2):
            detection_distances.append(scan_result['distance'])
            
    if(not isSomethingAhead):
        max_safe_distance = min(detection_distances) -THIRTY_FIVE_CM
        print('detection dist', detection_distances)
        safe_forward_cycles = calculate_cycles_from_distance(max_safe_distance)
        print('safe forward cyucles', safe_forward_cycles)
        if(max_safe_distance > 0 and max_safe_distance <= 80):
            print('moving forward')
            #move(safe_forward_cycles, movement_type='linear', direction='forward')
        

if __name__ == '__main__':
    # self_drive()
    #servo.set_angle(0)
    # #move(5, movement_type='rotation', direction='left')
    # distance = ultrasonic.get_distance()
    # if(distance > 35):
    #     picar.forward
    # picar.stop()
    # print()
    isSafeToMove = True
    while isSafeToMove:
        picar.forward(WHEEL_POWER)
        isSafeToMove = not perform_moving_scan(servo_speed=0.02)
        if(not isSafeToMove):
            picar.stop()
            stationary_scan = perform_stationary_scan()
            decide_movement(stationary_scan)
    picar.stop()

##todo: fix the detection on move to be less sensitive on the sides
## if there is something blocking it's forward path should check on the sides then decide