import picar_4wd as picar
#import servo
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
#import utils, constants
import time, math
from picar_4wd.utils import mapping

#Camera imports
import cv2
from object_detector import ObjectDetector
from object_detector import ObjectDetectorOptions

class Servo():
    PERIOD = 4095
    PRESCALER = 10
    MAX_PW = 2500
    MIN_PW = 500
    FREQ = 50
    ARR = 4095
    CPU_CLOCK = 72000000
    def __init__(self, pin, offset=0):
        self.pin = pin
        self.offset = offset
        self.pin.period(self.PERIOD)
        prescaler = int(float(self.CPU_CLOCK) / self.FREQ/ self.ARR)
        self.pin.prescaler(prescaler)
        self.currentAngle = 0

    def set_angle(self, angle):
        try:
            angle = int(angle)
            self.currentAngle = angle
        except:
            raise ValueError("Angle value should be int value, not %s"%angle)
        if angle < -90:
            angle = -90
        if angle > 90:
            angle = 90
        angle = angle + self.offset
        High_level_time = mapping(angle, -90, 90, self.MIN_PW, self.MAX_PW)
        pwr =  High_level_time / 20000
        value = int(pwr*self.PERIOD)
        self.pin.pulse_width(value)
        
servo = Servo(PWM("P0"), offset=0)
ultrasonic = Ultrasonic(Pin('D8'), Pin('D9'))
FORWARD_SPEED = 40
TURN_SPEED = 30
THIRTY_FIVE_CM = 20
TEN_CM = 10
SERVO_SLEEP = 0.08
SERVO_STEP = 18
DETECTION_DISTANCE = 30
SERVO_MAX_ANGLE = 90
SERVO_ZERO_ANGLE = 0
SERVO_MIN_ANGLE = -90
SERVO_TIME = 0.5

def calculate_cycles_from_distance(distance: int):
    # five centimeters
    base_distance = {'distance': 5, 'cycles': 2}
    cycleCount = math.floor(distance / base_distance["distance"])
    print('cycleCount\n', cycleCount)
    return cycleCount * base_distance["cycles"]

def get_status_at(angle, servo_speed=SERVO_TIME):
    servo.set_angle(angle)
    time.sleep(servo_speed)
    return ultrasonic.get_distance()

def perform_one_sweep(detection_distance=30, servo_speed=SERVO_TIME):
    global SERVO_STEP
    servo_delta = SERVO_STEP
    detection_list = []
    scan_info = []
    start_angle = servo.currentAngle
    current_angle = servo.currentAngle
    
    if(start_angle >= SERVO_MAX_ANGLE):
        servo_delta = -servo_delta
    elif(start_angle <= SERVO_MIN_ANGLE):
        servo_delta = abs(servo_delta)
    else:
        servo_delta = 0

    while ((start_angle == SERVO_MIN_ANGLE and current_angle <= SERVO_MAX_ANGLE) or
          (start_angle == SERVO_MAX_ANGLE and current_angle >= SERVO_MIN_ANGLE)):
        
        distance = get_status_at(current_angle, servo_speed=servo_speed)
        #print('current angle', current_angle)
        isDetected = 1 if distance <= detection_distance and not distance == -2 else 0
        scan_info.append({'distance': distance, 'current-angle': current_angle, 'detection': isDetected})
        detection_list.append(isDetected)       
        current_angle += servo_delta
 
    print('One Sweep detection list:', detection_list) 
    return (scan_info, detection_list)

def decide_movement(sweep_info, Isdetected):
    isClearAhead = not sum(Isdetected[3:7])
    isClearLeft = not sum(Isdetected[0:2])
    isClearRight = not sum(Isdetected[8:10])

    if(isClearAhead):
        picar.forward(FORWARD_SPEED)
    else:
        if(isClearLeft):
            picar.turn_left(TURN_SPEED)
        elif(isClearRight):
            picar.turn_right(TURN_SPEED)
        else:
            picar.stop()
 
#     detection_distances = []
#     safe_forward_cycles = 0
#     print('scan result', sweep_info[3:8])
#     print('Isdetected', Isdetected[3:8])
#     for scan_result in sweep_info[3:7]:
#         if(not scan_result['distance'] == -2):
#             detection_distances.append(scan_result['distance'])
#             
#     if(not isSomethingAhead):
#         max_safe_distance = min(detection_distances) -THIRTY_FIVE_CM
#         print('detection dist', detection_distances)
#         safe_forward_cycles = calculate_cycles_from_distance(max_safe_distance)
#         print('safe forward cyucles', safe_forward_cycles)
#         if(max_safe_distance > 0 and max_safe_distance <= 80):
#             print('moving forward')
#             #move(safe_forward_cycles, movement_type='linear', direction='forward')
        

if __name__ == '__main__':
    try:
        servo.set_angle(SERVO_MIN_ANGLE)
        
         #Camera setting Camera Id = 0
         camera = cv2.VideoCapture(0)
         camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
         camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
         
         #Tensorflow model settings
         tensorflow_model_options = ObjectDetectorOptions(
             num_threads=4,
             score_threshold=0.5,
             max_results=3,
             enable_edgetpu=False)
         
         detector = ObjectDetector(model_path='efficientdet_lite0.tflite', options=tensorflow_model_options)

        #While ESC key is not pressed
        while (cv2.waitKey(1) != 27):           
            #get ultrasonic sweep data
            (one_sweep_info, IsDetected) = perform_one_sweep(servo_speed=0.02)
            
            #get detected camera objects
             if(camera.isOpened()):
                 success, image = camera.read()
                 if(success):
                     image = cv2.flip(image, 1)
                     detections_info = detector.detect(image)
                     print(detections_info)
                 else:
                     print('Camera read error')                    
             else:
                 print('Camera open error')
                
            decide_movement(one_sweep_info, IsDetected)

    finally:
         camera.release()
        picar.stop()
        servo.set_angle(SERVO_ZERO_ANGLE)

##todo: fix the detection on move to be less sensitive on the sides
## if there is something blocking it's forward path should check on the sides then decide
