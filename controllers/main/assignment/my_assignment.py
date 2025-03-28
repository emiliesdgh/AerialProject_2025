import numpy as np
import time
import cv2

from exercises.ex3_motion_planner import MotionPlanner3D as MP
from lib.a_star_3D import AStar3D
from lib.mapping_and_planning_examples import path_planning as PP
from main import CrazyflieInDroneDome as Drone
# The available ground truth state measurements can be accessed by calling sensor_data[item]. 
# All values of "item" are provided as defined in main.py within the function read_sensors. 
# The "item" values that you may later retrieve for the hardware project are:

# "x_global": Global X position
# "y_global": Global Y position
# "z_global": Global Z position

# 'v_x": Global X velocity
# "v_y": Global Y velocity
# "v_z": Global Z velocity

# "ax_global": Global X acceleration
# "ay_global": Global Y acceleration
# "az_global": Global Z acceleration (With gravtiational acceleration subtracted)

# "roll": Roll angle (rad)
# "pitch": Pitch angle (rad)
# "yaw": Yaw angle (rad)

# "q_x": X Quaternion value
# "q_y": Y Quaternion value
# "q_z": Z Quaternion value
# "q_w": W Quaternion value

# A link to further information on how to access the sensor data on the Crazyflie hardware for the hardware practical can be found here: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#stateestimate
starting_position = True
second_position = False
pink_direction = None
position = "start"
initial = np.zeros(4)
second = np.zeros(4)
way_point1 = np.zeros(3)
way_point1W = np.zeros(3)
R_initial = np.zeros(3)

TPC1 = np.zeros(2)
TPC2 = np.zeros(2)

start_time = time.time()

def get_command(sensor_data, camera_data, dt):
    global starting_position, second_position, pink_direction, position, start_time, initial, second, way_point1, R_initial,TPC1, TPC2,way_point1W

    # NOTE: Displaying the camera image with cv2.imshow() will throw an error because GUI operations should be performed in the main thread.
    # If you want to display the camera image you can call it main.py.

    # Take off example
    if sensor_data['z_global'] < 0.49:
        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
        return control_command

    # ---- YOUR CODE HERE ----
    camera_data = np.array(camera_data)
    # print(camera_data.shape)
    # print(camera_data[0])

    # get_waypoints(sensor_data, camera_data, dt)

    # control_command = [0,0,0,0] # bring drone to origin
    # control_command = get_waypoints(sensor_data, camera_data, dt)
    # posInit = initial_position(sensor_data)

    # # pink_x, pink_y = detect_pink(camera_data)
    # # direction_to_pink(camera_data)

    # # if pink_direction == "left":
    # #     direction = [sensor_data['x_global'], sensor_data['y_global'] - 0.1, 1.0, sensor_data['yaw']]
    # # elif pink_direction == "right":
    # #     direction = [sensor_data['x_global'], sensor_data['y_global'] + 0.1, 1.0, sensor_data['yaw']]
    
    # # # if distance_to_pink(sensor_data, camera_data) > 0.1:
    # # while(pink_direction == "left") :
    # #     direction = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']+angle_to_pink(sensor_data, camera_data)]
    # # if pink_direction == None :
    # #     direction = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
    # if pink_direction == "above":
    #     control_command = [sensor_data['x_global'] - 0.1, sensor_data['y_global'], 1.0, sensor_data['yaw']]
    # elif pink_direction == "below":
    #     control_command = [sensor_data['x_global'] + 0.1, sensor_data['y_global'], 1.0, sensor_data['yaw']]
    # control_command = direction  


    # triangulate(sensor_data, camera_data, dt)   

    # if position == "start" :
    #     control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]   
    #     print("in first if")
    #     position = "second" 
    #     # x_current, y_current, z_current, yaw_current
    #     current_sensor_data = sensor_data
    #     # starting_position = False
    #     # second_position = True
    # print(sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw'])
    # print("current current points : ", current_sensor_data)
  
    # x_toGo, y_toGo, z_toGo, yaw_toGo = triangulate(current_sensor_data, camera_data, dt)
    # print(x_toGo, y_toGo, z_toGo, yaw_toGo)
    # print("HEEERREE 333333")
    # # second_position = False
    # position = None
    
    # control_command = [x_toGo, y_toGo, z_toGo, yaw_toGo]         
    # # control_command = [pink_x, pink_y, 1.0, sensor_data['yaw']]
    # print(position)




    #####
    # print("in get_command")
    # print(dt)
    deltaTime = time.time() - start_time 
    print(deltaTime)
    if deltaTime < 5:

        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
        x_initial = sensor_data['x_global']
        y_initial = sensor_data['y_global']
        z_initial = sensor_data['z_global']
        yaw_initial = sensor_data['yaw']
        R_initial = [sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']]
        initial = np.array([x_initial, y_initial, z_initial])

        pink_center_initial = detect_pink(camera_data)
        print("pink_center_initial : ", pink_center_initial)
        TPC1 = pink_center_initial

    elif deltaTime < 7 and deltaTime > 5:

        control_command = [sensor_data['x_global'], 3.5, 1.0, sensor_data['yaw']+0.01]
        # print("sensor data before moving")
        # print(sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw'])

    elif deltaTime < 8 and deltaTime > 7:

        # control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
        x_second = sensor_data['x_global']
        y_second = sensor_data['y_global']
        z_second = sensor_data['z_global']
        yaw_second = sensor_data['yaw']
        second = np.array([x_second, y_second, z_second])

        pink_center_second = detect_pink(camera_data)
        print("pink_center_second : ", pink_center_second)
        TPC2 = pink_center_second

        print("triangulate")

        ## not the right coordinates... hmmm 
        way_point1 = triangulate(sensor_data, camera_data, dt, initial, second, R_initial, TPC1, TPC2)

        way_point1W = C2W(R_initial, way_point1)
        # dy = way_point1[1] - y_second 
        # way_point1[1] = y_second - dy
        # way_point1 = np.array([x_second + way_point1[0], y_second - way_point1[1], 1.0])

    

        print("way_point1 : ", way_point1)
        print("way_point1W : ", way_point1W)
    if deltaTime > 8:
        print("over time 8")

        control_command = [way_point1W[0], way_point1W[1], 1.0, sensor_data['yaw']]
    # if position == "start" :
    #     control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
    #     print("in start")
        
    # print(camera_data.shape)
    # if position == "second" :
    #     control_command = [sensor_data['x_global'], sensor_data['y_global'] - 0.1, 1.0, sensor_data['yaw']]
    #     print("in second")
    #     position = "third"

    # control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]

    # position = "second"

    return control_command # Ordered as array with: [pos_x_cmd, pos_y_cmd, pos_z_cmd, yaw_cmd] in meters and radianss

def triangulate(sensor_data, camera_data, dt, P, Q, R, TPC1, TPC2) :
    f_pixel = 300/(2*np.tan(1.5/2))
    # alpha = 1
    # beta = 1
    # P = initial
    # Q = second
    # print(f_pixel)
    # print(P.shape)
    # print(Q.shape)
    # euler_angles = [sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']]
    euler_angles = [R[0], R[1], R[2]]

    R_roll = np.array([ [1, 0, 0], 
                        [0, np.cos(euler_angles[0]), -np.sin(euler_angles[0])],
                        [0, np.sin(euler_angles[0]), np.cos(euler_angles[0])]])
    
    R_pitch = np.array([[np.cos(euler_angles[1]), 0, np.sin(euler_angles[1])],
                        [0, 1, 0],
                        [-np.sin(euler_angles[1]), 0, np.cos(euler_angles[1])]])
    
    R_yaw = np.array([ [np.cos(euler_angles[2]), -np.sin(euler_angles[2]), 0],
                        [np.sin(euler_angles[2]), np.cos(euler_angles[2]), 0],
                        [0, 0, 1]])
    
    R = (R_yaw @ R_pitch @ R_roll).T
    # R = (R_yaw @ R_pitch @ R_roll)

    ## using sensor data
    # v = R @ np.array([P[0], P[1], 0])
    # v_prim = R @ np.array([Q[0], Q[1], 0])

    ## using pink center from camera data
    v = R @ np.array([TPC1[0], TPC1[1], 0])
    v_prim = R @ np.array([TPC2[0], TPC2[1], 0])

    r = R @ v
    s = R @ v_prim

    A = np.array([[np.dot(r,r) , -np.dot(s,r)],
                  [np.dot(r,s) , -np.dot(s,s)]])
    # print(A.shape)

    ## using sensor data
    # b = np.array([[np.dot((P - Q), r)], 
    #               [np.dot((P - Q), s)]])

    ## using pink center from camera data
    b = np.array([[np.dot((TPC1 - TPC2), r)], 
                  [np.dot((TPC1 - TPC2), s)]])
    
    alpha, beta = np.linalg.inv(A) @ b

    # ## Tuning alpha and beta
    # print("alpha : ", alpha, "beta : ", beta)

    # alpha = alpha + 0.1
    # beta = beta + 0.1
    # print("alpha : ", alpha, "beta : ", beta)

    ## using sensor data
    # F = P + alpha * r
    # G = Q + beta * s

    ## using pink center from camera data
    F = TPC1 + alpha * r
    G = TPC2 + beta * s

    H = (F + G)/2 

    # H = np.array([H[0], -H[1], 1.0])

    # print("H : ", H)
    # print("alpha : ", alpha, "beta : ", beta)
    return H

def C2W(R, point) :
    euler_angles = [R[0], R[1], R[2]]

    R_roll = np.array([ [1, 0, 0], 
                        [0, np.cos(euler_angles[0]), -np.sin(euler_angles[0])],
                        [0, np.sin(euler_angles[0]), np.cos(euler_angles[0])]])
    
    R_pitch = np.array([[np.cos(euler_angles[1]), 0, np.sin(euler_angles[1])],
                        [0, 1, 0],
                        [-np.sin(euler_angles[1]), 0, np.cos(euler_angles[1])]])
    
    R_yaw = np.array([ [np.cos(euler_angles[2]), -np.sin(euler_angles[2]), 0],
                        [np.sin(euler_angles[2]), np.cos(euler_angles[2]), 0],
                        [0, 0, 1]])
    
    R = (R_yaw @ R_pitch @ R_roll)

    point = point @ R

    return point


# def get_waypoints(sensor_data, camera_data, dt) :

#     print(camera_data)

# def initial_position(sensor_data):
#     global starting_position
#     starting_position = False

#     return sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], 1.0, sensor_data['yaw']

def detect_pink(camera_data):
    # Convert image to HSV (adjust parameters based on your actual pink color range)
    imgHSV = cv2.cvtColor(camera_data, cv2.COLOR_BGR2HSV)
    lower_pink = np.array([140, 100, 100])
    upper_pink = np.array([160, 255, 255])
    mask = cv2.inRange(imgHSV, lower_pink, upper_pink)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) > 100:  # Minimum area to avoid small noises
            x, y, w, h = cv2.boundingRect(contour)
            # print(x + w/2, y + h/2)
            center = np.array([x + w/2, y + h/2, 1.0])
            return center  # Return the center of the pink square

    return None

# ## function to determine where the pink square is to the drone
# def direction_to_pink(camera_data):
#     pink_x, pink_y = detect_pink(camera_data)
#     global pink_direction
#     if (pink_x < 160) :
#         print("Pink square is to the left")
#         pink_direction = "left"
        
#     elif (pink_x > 160) :
#         print("Pink square is to the right")
#         pink_direction = "right"
    
#     # if (pink_y < 120) :
#     #     print("Pink square is above")
#     #     pink_direction = "above"
    
#     # elif (pink_y > 120) :
#     #     print("Pink square is below")
#     #     pink_direction = "below"

# def distance_to_pink(sensor_data, camera_data):
#     pink_x, pink_y = detect_pink(camera_data)
#     return np.sqrt((sensor_data['x_global'] - pink_x)**2 + (sensor_data['y_global'] - pink_y)**2)

# def angle_to_pink(sensor_data, camera_data):
#     pink_x, pink_y = detect_pink(camera_data)
#     return np.arctan((pink_y - sensor_data['y_global'])/(pink_x - sensor_data['x_global']))