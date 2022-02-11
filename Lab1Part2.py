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

np.set_printoptions(edgeitems=30, linewidth=100000, formatter=dict(float=lambda x: "%.3g" % x))

servo = Servo(PWM("P0"), offset=0)
ultrasonic = Ultrasonic(Pin('D8'), Pin('D9'))
MAP_SIZE = 200
FORWARD_SPEED = 5
REVERSE_SPEED = 10
TURN_SPEED = 30
THIRTY_FIVE_CM = 20
TEN_CM = 10
DETECTION_DISTANCE = 40
SERVO_TIME = 0.02
TURN_LEFT_TIME_FOR_90_DEGREE = 1.6
TURN_RIGHT_TIME_FOR_90_DEGREE = 1.6
servo_currentAngle = 0

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
        self.distance = 0
        
def picar_forward(speed):
    photointrrupter.turn = 0
    picar.forward(speed)

def picar_reverse(speed):
    photointrrupter.turn = 0
    picar.backward(speed)

def picar_turn_left(speed):
    photointrrupter.resetDistance()
    photointrrupter.turn = 1
    picar.turn_right(speed)
    
def picar_turn_right(speed):
    photointrrupter.resetDistance()
    photointrrupter.turn = 1
    picar.turn_left(speed)    

def picar_stop():
    photointrrupter.turn = 0
    picar.stop()
        
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
        
class SweepScan():
    SERVO_SLEEP = 0.08
    SERVO_STEP = 18
    DETECTION_DISTANCE = 30
    SERVO_MAX_ANGLE = 90
    SERVO_ZERO_ANGLE = 0
    SERVO_MIN_ANGLE = -90
    servo_speed = 0.02
    scan_info = []
    def __init__(self, detectionDistance, delay_btw_scan_points, DelayBtwnSweeps):
        self.DETECTION_DISTANCE = detectionDistance
        self.servo_speed = delay_btw_scan_points
        self.DelayBtwnSweeps = DelayBtwnSweeps
        set_servo_angle(self.SERVO_MIN_ANGLE) 
     
    def perform_one_sweep(self):
        #print('perform_one_sweep:')
        while(True):
            self.scan_info = []
            start_angle = get_servo_angle()
            next_angle = get_servo_angle()
            
            if(start_angle >= self.SERVO_MAX_ANGLE):
                servo_delta = -self.SERVO_STEP
            elif(start_angle <= self.SERVO_MIN_ANGLE):
                servo_delta = self.SERVO_STEP
            else:
                servo_delta = 0

            while ((start_angle == self.SERVO_MIN_ANGLE and next_angle <= self.SERVO_MAX_ANGLE) or
                  (start_angle == self.SERVO_MAX_ANGLE and next_angle >= self.SERVO_MIN_ANGLE)):
                distance = get_status_at(next_angle, servo_speed=self.servo_speed)
                #print('current angle', next_angle)
                isDetected = 1 if distance <= self.DETECTION_DISTANCE and not distance == -2 else 0
                self.scan_info.append({'distance': distance, 'angle': next_angle, 'detection': isDetected})      
                next_angle = get_servo_angle() + servo_delta
            #print('scan_info:',self.scan_info)
            time.sleep(self.DelayBtwnSweeps)
            
    def getScanInfo(self):
        return(self.scan_info)
            
    def setup(self):
        self.sweepScanThread = threading.Thread(target=self.perform_one_sweep, daemon=True)
        self.sweepScanThread.start()            

class ObjectDetection():
    def __init__(self, cameraSleepTime):
        self.cam_result = ""
        #Camera setting Camera Id = 0
        self.camera = cv.VideoCapture(0)
        self.camera.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        self.cameraScanDelay = cameraSleepTime
         
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
                    #print('camera_scan:Object list:',self.cam_result)
                else:
                    print('Camera read error1')                    
            else:
                print('Camera open error2')
            time.sleep(self.cameraScanDelay)

    def setup(self):
        self.cameraThread = threading.Thread(target=self.camera_scan, daemon=True)
        self.cameraThread.start()
        
    def getDetectedObject(self):
        return(self.cam_result)
    
def car_command(command, step=0):
    photointrrupter.resetDistance()
    while(command != 'stop'):
        #print('command:', command)
        if(command == 'forward'):
            while(photointrrupter.getDistance() <= step):
                picar_forward(FORWARD_SPEED)
            command = 'stop'
        elif(command == 'reverse'):
            while(photointrrupter.getDistance() <= step):
                picar_reverse(REVERSE_SPEED)
            command = 'stop'    
        elif(command == 'left'):
            picar_turn_left(TURN_SPEED)
            time.sleep(TURN_LEFT_TIME_FOR_90_DEGREE)
            command = 'stop'
        elif(command == 'right'):
            picar_turn_right(TURN_SPEED)
            time.sleep(TURN_RIGHT_TIME_FOR_90_DEGREE)
            command = 'stop'
        else:
            command = 'stop'
    picar_stop()

def compile_direction(car, next_point, orientation):
    next_point_x, next_point_y = next_point[0], next_point[1]
    car_x, car_y = car[0], car[1]
    dx = next_point_x - car_x
    dy = next_point_y - car_y
    if dx > 0:
        if orientation == 0:
            return 'right'
        elif orientation == 90:
            return 'forward'
        elif orientation == 180:
            return 'left'
        elif orientation == 270:
            return 'back'       #turn 180?
    elif dx < 0:
        if orientation == 0:
            return 'left'
        elif orientation == 90:
            return 'back'
        elif orientation == 180:
            return 'right'
        elif orientation == 270:
            return 'forward'
    elif dy > 0:
        if orientation == 0:
            return 'forward'
        elif orientation == 90:
            return 'left'
        elif orientation == 180:
            return 'back'
        elif orientation == 270:
            return 'right'
    elif dy < 0:
        if orientation == 0:
            return 'back'
        elif orientation == 90:
            return 'right'
        elif orientation == 180:
            return 'forward'
        elif orientation == 270:
            return 'left'

def update_orientation(curr, turn):
    degree = 0
    if turn == 'right':
        degree = 90
    elif turn == 'left':
        degree = -90
    curr = (curr + degree) % 360
    return curr 


def create_advanced_mapping_(map, sweep_info, orientation, car_start_map_x, car_start_map_y):
    for entry in sweep_info:
        if(entry['detection']):
            #(x,y) with respect to the car
            thetaInRadians = math.radians(entry['angle'])
            scanned_object_x = entry['distance']*math.sin(thetaInRadians)
            scanned_object_y = entry['distance']*math.cos(thetaInRadians)
              
            if orientation == 0:
                map_x = car_start_map_x + scanned_object_x
                map_y = car_start_map_y + scanned_object_y
            elif orientation == 90:
                map_x = car_start_map_x + scanned_object_y
                map_y = car_start_map_y - scanned_object_x
            elif orientation == 180:
                map_x = car_start_map_x - scanned_object_x
                map_y = car_start_map_y - scanned_object_y
            elif orientation == 270:
                map_x = car_start_map_x - scanned_object_y
                map_y = car_start_map_y + scanned_object_x

            map[int(map_x), int(map_y)] = 1
    return map

def runner(sweep_info, map, orientation, car, end):
    maze = create_advanced_mapping_(map, sweep_info, orientation, car[0], car[1])
    padded_maze = fix_maze(maze)
    #print(padded_maze)
    path = search(padded_maze, 1, car, end)
    directions = []
    curr_car = car
    curr_orientation = orientation
    for point in path:
        direction = compile_direction(curr_car, point, curr_orientation)
        directions.append(direction)
        curr_orientation = update_orientation(curr_orientation, direction)
        if direction == 'right' or direction == 'left':     #if our car is turning then it also needs to move to reach the next point so 2 moves
            d = compile_direction(curr_car, point, curr_orientation)
            directions.append(d)
        curr_car = point
    final_orientation, curr_position = follow_directions(directions, orientation, path)
    #print('curr_position:', curr_position)
    return final_orientation, curr_position

# maybe add the stuff below to new file later and import instead?

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

#This function return the path of the search
def return_path(current_node,maze):
    path = []
    no_rows, no_columns = np.shape(maze)
    # here we create the initialized result maze with -1 in every position
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    # Return reversed path as we need to show from start to end path
    path = path[::-1]
    return path

# add padding around obstacles
def fix_maze(maze, boundary_thickness=4): # coordinate system matches cartesian
    obstacles = np.nonzero(maze)
    #print(obstacles)
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

def follow_directions(directions, orientation, path, num_steps=30):
    count = 0
    curr_orientation = orientation
    for direction in directions:
        print('direction:',direction)
        car_command(direction, 1)
        if direction == 'forward':
            count += 1
        curr_orientation = update_orientation(curr_orientation, direction)
        #if count >= num_steps:
            #break
    print('path:',path[count-1])
    return curr_orientation, path[count-1]

if __name__ == '__main__':
    try:       
        photointrrupter = Photointrrupter(Pin('D6'))
        detector = ObjectDetection(cameraSleepTime=0.02)
        sweepScan = SweepScan(detectionDistance=30, delay_btw_scan_points=0.02, DelayBtwnSweeps=0.02)
              
        photointrrupter.setup()
        detector.setup()
        sweepScan.setup()
        
        start = (50,0)
        end = (0,100)
        orientation = 0
        map_ = np.zeros((200,200), dtype=np.uint8)

        #While ESC key is not pressed
        while(True):
            #print('Scan Info:',sweepScan.getScanInfo())
            orientation, start = runner(sweepScan.getScanInfo(), map_, orientation, start, end)

    finally:
        picar.stop()
        set_servo_angle(0)
