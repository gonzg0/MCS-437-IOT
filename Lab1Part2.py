import picar_4wd as picar
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
import RPi.GPIO as GPIO

#import utils, constants
import time, math
from picar_4wd.utils import mapping
import numpy as np

#Camera imports
import cv2 as cv
from object_detector import ObjectDetector
from object_detector import ObjectDetectorOptions

import Animation as animate
import threading

servo = Servo(PWM("P0"), offset=0)
ultrasonic = Ultrasonic(Pin('D8'), Pin('D9'))
ENABLE_PLOT=False
MAP_SIZE = 200
SPEED = 5
FORWARD_SPEED = 5
BACKWARD_SPEED = 10
TURN_SPEED = 30
THIRTY_FIVE_CM = 20
TEN_CM = 10
SERVO_SLEEP = 0.08
SERVO_STEP = 18
DETECTION_DISTANCE = 40
SERVO_MAX_ANGLE = 90
SERVO_ZERO_ANGLE = 0
SERVO_MIN_ANGLE = -90
SERVO_TIME = 0.02
TURN_LEFT_TIME_FOR_90_DEGREE = 1.75
TURN_RIGHT_TIME_FOR_90_DEGREE = 1.6
servo_currentAngle = SERVO_ZERO_ANGLE
envoriment_map_ultrasonic = np.zeros((MAP_SIZE,MAP_SIZE), dtype=np.uint8)
envoriment_map_camera = np.empty((MAP_SIZE,MAP_SIZE), dtype="S10")

#Considering the car starts from the bottom center
car_start_map_x = MAP_SIZE/2 -1
car_start_map_y = 0

#Considering the car desitnation to be last mid point of the map
car_destination_map_x = MAP_SIZE/2 -1
car_destination_map_y = MAP_SIZE -1

class Photointrrupter():
    wheelDiameter = 6.6
    PPR = 20
    def __init__(self, pin):
        self.pin = pin
        self.distancePerPulse = (2*3.14*self.wheelDiameter/2)/self.PPR
        self.distance = 0
        self.pulseCount = 0
        self.turn = 0
        
    def setup(self):
        self.pin.irq(self.RisingEdgeHandler, GPIO.RISING)
    
    def RisingEdgeHandler(self, channel):
        #if it is a turn skip the distance calcuation
        if(not self.turn):
            self.pulseCount +=1
            self.distance = self.pulseCount * self.distancePerPulse
        #print(self.distance)
    def getDistance(self):
        return(self.distance)
    def resetDistance(self):
        self.pulseCount = 0
        
def picar_forward(speed):
    photointrrupter.turn = 0
    picar.forward(speed)

def picar_reverse(speed):
    photointrrupter.turn = 0
    picar.backward(speed)

def picar_turn_left(speed):
    photointrrupter.turn = 1
    picar.turn_right(speed)
    
def picar_turn_right(speed):
    photointrrupter.turn = 1
    picar.turn_left(speed)    

def picar_stop():
    photointrrupter.turn = 0
    picar.stop()
    
def VisualizeData(x, y,cl):
    if ENABLE_PLOT:
        animate.animate_scan(x, y, cl=cl)
        
def set_servo_angle(angle: int):
    global servo_currentAngle
    servo_currentAngle = angle
    servo.set_angle(angle)

def get_servo_angle():
    global servo_currentAngle
    return(servo_currentAngle)

def calculate_cycles_from_distance(distance: int):
    # five centimeters
    base_distance = {'distance': 5, 'cycles': 2}
    cycleCount = math.floor(distance / base_distance["distance"])
    print('cycleCount\n', cycleCount)
    return cycleCount * base_distance["cycles"]

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
        #print('current angle', next_angle)
        isDetected = 1 if distance <= detection_distance and not distance == -2 else 0
        scan_info.append({'distance': distance, 'angle': next_angle, 'detection': isDetected})
        detection_list.append(isDetected)       
        next_angle = get_servo_angle() + servo_delta
 
    print('One Sweep detection list:', detection_list) 
    return (scan_info, detection_list)

def decide_movement(sweep_info, Isdetected):
    isClearAhead = not sum(Isdetected[3:7])
    isClearLeft = not sum(Isdetected[0:2])
    isClearRight = not sum(Isdetected[8:10])

    if(isClearAhead):
        picar_forward(FORWARD_SPEED)
    else:
        if(isClearLeft):
            picar_turn_left(TURN_SPEED)
        elif(isClearRight):
            picar_turn_right(TURN_SPEED)
        else:
            picar_reverse(BACKWARD_SPEED)

def create_advanced_mapping(sweep_info, cam_result):
    global envoriment_map
    for entry in sweep_info:
        if(entry['detection']):
            #(x,y) with respect to the car
            thetaInRadians = math.radians(entry['angle'])
            scanned_object_x = entry['distance']*math.sin(thetaInRadians)
            scanned_object_y = entry['distance']*math.cos(thetaInRadians)
            
            #(x,y) with respect to the ma           
            map_x = car_start_map_x + scanned_object_x
            map_y = car_start_map_y + scanned_object_y
            
            if((map_x >= MAP_SIZE) or (map_y >= MAP_SIZE)):
                print('x:',scanned_object_x, 'y:',scanned_object_y,'xx:',map_x,'yy:',map_y)
            elif(not cam_result):
                envoriment_map_ultrasonic[int(map_x), int(map_y)] = 1
                VisualizeData(int(map_x), int(map_y), cl='black')
            else:    
                envoriment_map_camera[int(map_x), int(map_y)] = cam_result
                VisualizeData(int(map_x), int(map_y), cl='black')
                
class ObjectDetection():
    CAMERA_THREAD_SLEEP_TIME = 0.02
    def __init__(self):
        self.cam_result = ""
        #Camera setting Camera Id = 0
        self.camera = cv.VideoCapture(0)
        self.camera.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
         
        #Tensorflow model settings
        tensorflow_model_options = ObjectDetectorOptions(
            num_threads=4,
            score_threshold=0.5,
            max_results=3,
            enable_edgetpu=False)
         
        self.detector = ObjectDetector(model_path='efficientdet_lite0.tflite', options=tensorflow_model_options)        
    def __del__(self):
        self.camera.release()
        
    def camera_scan(self):
        global cam_result
        while(True):
            #get detected camera objects
            if(self.camera.isOpened()):
                success, image = self.camera.read()
                if(success):
                    image = cv.flip(image, 1)
                    detections_info = self.detector.detect(image)
                    cam_result = ""
                    for detected_obect in detections_info:
                        if(detected_obect.categories[0].label == 'person'):
                            cam_result = detected_obect.categories[0].label
                            break
                        elif(detected_obect.categories[0].label == 'stop sign'):
                            cam_result = detected_obect.categories[0].label
                    self.cam_result = cam_result
                    print('camera_scan:Object list:',self.cam_result)
                else:
                    print('Camera read error1')                    
            else:
                print('Camera open error2')
            time.sleep(self.CAMERA_THREAD_SLEEP_TIME)

    def setup(self):
        self.cameraThread = threading.Thread(target=self.camera_scan, daemon=True)
        self.cameraThread.start()
        
    def getDetectedObject(self):
        return(self.cam_result)
    
def car_command(command, step=0):
    photointrrupter.resetDistance()
    while((command != 'Stop')):
        print('car_command:',command,':',photointrrupter.getDistance(),':',step,':',photointrrupter.getDistance() <= step,':',(command == 'forawrd'),':', type(command))
        if(command == 'Forward'):
            while(photointrrupter.getDistance() <= step):
                picar_forward(SPEED)
            command = 'Stop'
        elif(command == 'Reverse'):
            while(photointrrupter.getDistance() <= step):
                picar_reverse(SPEED)
            command = 'Stop'    
        elif(command == 'Left'):
            turnStartTime = time.time()
            while((time.time() - turnStartTime) <= TURN_LEFT_TIME_FOR_90_DEGREE):
                picar_turn_left(SPEED)
            command = 'Stop'
        elif(command == 'Right'):
            turnStartTime = time.time()
            while((time.time() - turnStartTime) <= TURN_RIGHT_TIME_FOR_90_DEGREE):
                picar_turn_right(SPEED)
            command = 'Stop'
        else:
            command = 'Stop'
    picar_stop()
        
if __name__ == '__main__':
    try:
        set_servo_angle(SERVO_MIN_ANGLE)
        

        photointrrupter = Photointrrupter(Pin('D6'))
        detector=ObjectDetection()
        
        #mark the start on the plot
        VisualizeData(car_start_map_x, car_start_map_y, cl='red')
        
        #mark the end on the plot
        VisualizeData(car_destination_map_x, car_destination_map_y, cl='green')
        
        photointrrupter.setup()
        detector.setup()
        car_command('Forward', 80)
        
        while(True):
              scan_info, detection_list = perform_one_sweep(40, 0.03)
              create_advanced_mapping(scan_info, detector.getDetectedObject())
#              print('Main:Object:', detector.getDetectedObject())

    finally:
        #print('Finally')
        picar_stop()
        set_servo_angle(SERVO_ZERO_ANGLE)