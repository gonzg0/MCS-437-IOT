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
ENABLE_PLOT=False
MAP_SIZE = 200

servo = Servo(PWM("P0"), offset=0)
ultrasonic = Ultrasonic(Pin('D8'), Pin('D9'))
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
SERVO_TIME = 0.5
servo_currentAngle = SERVO_ZERO_ANGLE
envoriment_map_ultrasonic = np.zeros((MAP_SIZE,MAP_SIZE), dtype=np.uint8)
envoriment_map_camera = np.empty((MAP_SIZE,MAP_SIZE), dtype="S10")
CAMERA_OBJECT_NO_OBJECT = 0
CAMERA_OBJECT_STOP_SIGN = 1
CAMERA_OBJECT_PERSON = 2
CAMERA_OBJECT_UNIDENTIFIED = 3

#Considering the car starts from the bottom center
car_start_map_x = MAP_SIZE/2 -1
car_start_map_y = 0

#Considering the car desitnation to be last mid point of the map
car_destination_map_x = MAP_SIZE/2 -1
car_destination_map_y = MAP_SIZE -1

class Photointrrupter():
    wheelDiameter = 2.5
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

def picar_forward(speed):
    photointrrupter.turn = 0
    picar.forward(speed)

def picar_reverse(speed):
    photointrrupter.turn = 0
    picar.backward(speed)

def picar_turn_left(speed):
    photointrrupter.turn = 1
    picar.turn_left(speed)

def picar_turn_right(speed):
    photointrrupter.turn = 1
    picar.turn_right(speed)
        
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
 
    #print('One Sweep detection list:', detection_list) 
    return (scan_info, detection_list)

def decide_movement(sweep_info, Isdetected):
    isClearAhead = not sum(Isdetected[3:7])
    isClearLeft = not sum(Isdetected[0:2])
    isClearRight = not sum(Isdetected[8:10])

    if(isClearAhead):
        picar_forward(FORWARD_SPEED)
    else:
        picar.stop()
        if(isClearLeft):
            picar_turn_left(TURN_SPEED)
        elif(isClearRight):
            picar_turn_right(TURN_SPEED)
        else:
            picar_reverse(BACKWARD_SPEED)

def advanced_mapping(sweep_info, cam_result, map_size = 30):
    map = np.zeros((map_size * 2, map_size), dtype=np.uint8)
    for entry in sweep_info:
        if(entry['detection']):
            #(x,y) with respect to the car
            thetaInRadians = math.radians(abs(entry['angle']))
            scanned_object_x = entry['distance']*math.sin(thetaInRadians)
            scanned_object_y = entry['distance']*math.cos(thetaInRadians)

            #(x,y) with respect to the ma
            map_x = map_size - scanned_object_x
            if entry['angle'] >= 0:           
                map_x = map_size + scanned_object_x
            map_y = scanned_object_y

            #if(not cam_result):
            map[int(map_x), int(map_y)] = 1

    return map

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

#   added by me below

class Node:
    """
        A node class for A* Pathfinding
        parent is parent of the current Node
        position is current position of the Node in the maze
        g is cost from start to current Node
        h is heuristic based estimated cost for current Node to end Node
        f is total cost of present node i.e. :  f = g + h
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other):
        return self.position == other.position




def search(maze, cost, start, end):
    """
        Returns a list of tuples as a path from the given start to the given end in the given maze
        :param maze:
        :param cost
        :param start:
        :param end:
        :return:
    """

    # Create start and end node with initized values for g, h and f
    start_node = Node(None, tuple(start))
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, tuple(end))
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both yet_to_visit and visited list
    # in this list we will put all node that are yet_to_visit for exploration. 
    # From here we will find the lowest cost node to expand next
    yet_to_visit_list = []  
    # in this list we will put all node those already explored so that we don't explore it again
    visited_list = [] 
    
    # Add the start node
    yet_to_visit_list.append(start_node)
    
    # Adding a stop condition. This is to avoid any infinite loop and stop 
    # execution after some reasonable number of steps
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    # what squares do we search . serarch movement is left-right-top-bottom 
    #(4 movements) from every positon

    move  =  [[-1, 0 ], # go up
              [ 0, -1], # go left
              [ 1, 0 ], # go down
              [ 0, 1 ]] # go right


    """
        1) We first get the current node by comparing all f cost and selecting the lowest cost node for further expansion
        2) Check max iteration reached or not . Set a message and stop execution
        3) Remove the selected node from yet_to_visit list and add this node to visited list
        4) Perofmr Goal test and return the path else perform below steps
        5) For selected node find out all children (use move to find children)
            a) get the current postion for the selected node (this becomes parent node for the children)
            b) check if a valid position exist (boundary will make few nodes invalid)
            c) if any node is a wall then ignore that
            d) add to valid children node list for the selected parent
            
            For all the children node
                a) if child in visited list then ignore it and try next node
                b) calculate child node g, h and f values
                c) if child in yet_to_visit list then ignore it
                d) else move the child to yet_to_visit list
    """
    #find maze has got how many rows and columns 
    no_rows, no_columns = np.shape(maze)
    
    # Loop until you find the end
    
    while len(yet_to_visit_list) > 0:
        
        # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
        outer_iterations += 1    

        
        # Get the current node
        current_node = yet_to_visit_list[0]
        current_index = 0
        for index, item in enumerate(yet_to_visit_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
                
        # if we hit this point return the path such as it may be no solution or 
        # computation cost is too high
        if outer_iterations > max_iterations:
            print ("giving up on pathfinding too many iterations")
            return return_path(current_node,maze)

        # Pop current node out off yet_to_visit list, add to visited list
        yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)

        # test if goal is reached or not, if yes then return the path
        if current_node == end_node:
            return return_path(current_node,maze)

        # Generate children from all adjacent squares
        children = []

        for new_position in move: 

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range (check if within maze boundary)
            if (node_position[0] > (no_rows - 1) or 
                node_position[0] < 0 or 
                node_position[1] > (no_columns -1) or 
                node_position[1] < 0):
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            
            # Child is on the visited list (search entire visited list)
            if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + cost
            ## Heuristic costs calculated here, this is using eucledian distance
            child.h = (((child.position[0] - end_node.position[0]) ** 2) + 
                       ((child.position[1] - end_node.position[1]) ** 2)) 

            child.f = child.g + child.h

            # Child is already in the yet_to_visit list and g cost is already lower
            if len([i for i in yet_to_visit_list if child == i and child.g > i.g]) > 0:
                continue

            # Add the child to the yet_to_visit list
            yet_to_visit_list.append(child)

# add padding around obstacles
def fix_maze(maze, boundary_thickness=5): # coordinate system matches cartesian
    obstacles = np.nonzero(maze)
    print(obstacles)
    (y_limit, x_limit) = maze.shape
    y_coords = obstacles[0]
    x_coords = obstacles[1]
    new_maze = np.zeros_like(maze)
    for i in range(len(y_coords)):
        x, y = x_coords[i], y_coords[i]
        y_start, y_end, x_start, x_end = 0, y_limit, 0, x_limit
        if y - boundary_thickness > 0 :
            y_start = y - boundary_thickness
        if y + boundary_thickness < y_limit :
            y_end = y + boundary_thickness
        if x - boundary_thickness > 0 :
            x_start = x - boundary_thickness
        if x + boundary_thickness < x_limit :
            x_end = x + boundary_thickness
        new_maze[y_start:y_end, x_start:x_end] = 1
    return new_maze
        
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
        
        photointrrupter = Photointrrupter(Pin('D6'))
        photointrrupter.setup()
        
        #mark the start on the plot
        ##VisualizeData(car_start_map_x, car_start_map_y, cl='red')
        
        #mark the end on the plot
        ##VisualizeData(car_destination_map_x, car_destination_map_y, cl='green')
        
        #While ESC key is not pressed
        while (cv.waitKey(1) != 27):           
            #get ultrasonic sweep data
            (one_sweep_info, IsDetected) = perform_one_sweep(DETECTION_DISTANCE, servo_speed=0.02)
            #get detected camera objects
            if(camera.isOpened()):
                success, image = camera.read()
                if(success):
                    image = cv.flip(image, 1)
                    detections_info = detector.detect(image)
                    cam_result = ""
                    for detected_obect in detections_info:
                        if((detected_obect.categories[0].label == 'person') or
                           (detected_obect.categories[0].label == 'stop sign')):
                            cam_result=detected_obect.categories[0].label
                            break
                    print('Object list:',cam_result)
                else:
                    print('Camera read error')                    
            else:
                print('Camera open error')
            create_advanced_mapping(one_sweep_info, cam_result)
            
            decide_movement(one_sweep_info, IsDetected)
        #print(envoriment_map)

    finally:
        camera.release()
        picar.stop()
        set_servo_angle(SERVO_ZERO_ANGLE)