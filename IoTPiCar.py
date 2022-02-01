import picar_4wd as picar
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
#import utils, constants
import time, math
from picar_4wd.utils import mapping
import numpy as np

#Camera imports
import cv2 as cv
from object_detector import ObjectDetector
from object_detector import ObjectDetectorOptions
      
servo = Servo(PWM("P0"), offset=0)
ultrasonic = Ultrasonic(Pin('D8'), Pin('D9'))
FORWARD_SPEED = 40
BACKWARD_SPEED = 40
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
servo_currentAngle = SERVO_ZERO_ANGLE
MAP_SIZE = 200
envoriment_map = np.zeros((MAP_SIZE,MAP_SIZE), dtype='uint8')

#Considering the car starts from the bottom center
car_start_map_x = MAP_SIZE/2 -1
car_start_map_y = 0

#Considering the car desitnation to be last mid point of the map
car_destination_map_x = MAP_SIZE/2 -1
car_destination_map_y = MAP_SIZE -1

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
 
    #print('One Sweep detection list:', detection_list) 
    return (scan_info, detection_list)

def decide_movement(sweep_info, Isdetected):
    isClearAhead = not sum(Isdetected[3:7])
    isClearLeft = not sum(Isdetected[0:2])
    isClearRight = not sum(Isdetected[8:10])

    if(isClearAhead):
        picar.forward(FORWARD_SPEED)
    else:
        picar.stop()
        if(isClearLeft):
            picar.turn_left(TURN_SPEED)
        elif(isClearRight):
            picar.turn_right(TURN_SPEED)
        else:
            picar.backward(BACKWARD_SPEED)
 
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

def create_advanced_mapping(sweep_info):
    global envoriment_map
    for entry in sweep_info:
        if(entry['detection']):
            #(x,y) with respect to the car
            scanned_object_x = entry['distance']*math.sin(math.radians(entry['angle']))
            scanned_object_y = entry['distance']*math.cos(math.radians(entry['angle']))
                        
            #(x,y) with respect to the ma           
            map_x = car_start_map_x + scanned_object_x
            map_y = car_start_map_y + scanned_object_y
            
            if((map_x >= MAP_SIZE) or (map_y >= MAP_SIZE)):
                print('x:',scanned_object_x, 'y:',scanned_object_y,'xx:',map_x,'yy:',map_y)
            else:
                envoriment_map[int(map_x), int(map_y)] = entry['detection']
        
if __name__ == '__main__':
    try:
        set_servo_angle(SERVO_MIN_ANGLE)
        
        #Camera setting Camera Id = 0
        camera = cv.VideoCapture(0)
        camera.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
         
        #Tensorflow model settings
        tensorflow_model_options = ObjectDetectorOptions(
            num_threads=4,
            score_threshold=0.5,
            max_results=3,
            enable_edgetpu=False)
         
        detector = ObjectDetector(model_path='efficientdet_lite0.tflite', options=tensorflow_model_options)

        #While ESC key is not pressed
        while (cv.waitKey(1) != 27):           
            #get ultrasonic sweep data
            (one_sweep_info, IsDetected) = perform_one_sweep(servo_speed=0.02)
            create_advanced_mapping(one_sweep_info)
            #get detected camera objects
            if(camera.isOpened()):
                success, image = camera.read()
                if(success):
                    image = cv.flip(image, 1)
                    detections_info = detector.detect(image)
#                     result = []
#                     for detected_obect in detections_info:
#                         label = detected_obect.categories[0].label
#                         score = detected_obect.categories[0].score
#                         result.append([label, score])
#                     print('Object list:',result)
                else:
                    print('Camera read error')                    
            else:
                print('Camera open error')
                
            decide_movement(one_sweep_info, IsDetected)
        #print(envoriment_map)

    finally:
        camera.release()
        picar.stop()
        set_servo_angle(SERVO_ZERO_ANGLE)

##todo: fix the detection on move to be less sensitive on the sides
## if there is something blocking it's forward path should check on the sides then decide
