import numpy as np
import time
import cv2

import exercises.ex0_rotations as rot


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
second_position = True
pink_direction = None
position = "start"

initial_pos = np.zeros(4)
second_pos = np.zeros(4)

way_point1 = np.zeros(3)
way_point1W = np.zeros(3)
R_initial = np.eye(3)
R_second = np.eye(3)

X_Y_MAX = 8.0 # meters
res_pos = 0.035 # meters
grid_size = (int(X_Y_MAX/res_pos), int(X_Y_MAX/res_pos))

pink1 = np.zeros(3)
pink2 = np.zeros(3)

start_time = time.time()

def get_command(sensor_data, camera_data, dt):
    global starting_position, second_position, pink_direction, position, start_time, initial_pos, second_pos, way_point1, R_initial, R_second, pink1, pink2, way_point1W

    # NOTE: Displaying the camera image with cv2.imshow() will throw an error because GUI operations should be performed in the main thread.
    # If you want to display the camera image you can call it main.py.

    # Take off example
    if sensor_data['z_global'] < 0.49:
        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
        return control_command

    # ---- YOUR CODE HERE ----
    camera = camera_data.copy()

    deltaTime = time.time() - start_time 
    # print(deltaTime)
    if deltaTime < 5:

        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
        

    elif deltaTime < 7 and deltaTime > 5:
        if starting_position :
            R_initial = C2W([sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']])
            print("R_initial : ", R_initial.shape)
            initial_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']])
            print("initial_pos : ", initial_pos)
            
            pink1 = detect_pink(camera)
            # print("pink1 cam : ", pink1)
           
            ## convert pink1 to world frame, the 2 options are the same
            # pink1 = C2W(R_initial) @ pink1
            # pink1 = pink1 @ C2W(R_initial).T 
            print("pink1 world : ", pink1)
            # pink1 = rot.quaternion2rotmat([sensor_data['q_x'], sensor_data['q_y'], sensor_data['q_z'], sensor_data['q_w']]).T @ pink1
            # print("pink1 world : ", pink1)
            
            starting_position = False

        control_command = [0.75, 3.5, 1.0, sensor_data['yaw']+0.05]
        # print("sensor data before moving")
        # print(sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw'])

    elif deltaTime < 8.5 and deltaTime > 7:

        # control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
       
        if second_position :
            R_second = C2W([sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']])
            second_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']])
            print("second_pos : ", second_pos)

            pink2 = detect_pink(camera)
            # print("pink2 : ", pink2)
            ## convert pink1 to world frame
            # pink2 = C2W(R_second) @ pink2
            # pink2 = pink2 @ C2W(R_second).T
            # print("pink2 world : ", pink2)

            # pink2 = pink2 @ C2W(R_second)
            print("pink2 world : ", pink2)

            way_point1 = triangulate(R_initial, R_second, initial_pos, second_pos, pink1, pink2)
            print("way_point1 : ", way_point1)

            second_position = False

        # way_point1W = C2W(R_initial, way_point1)
        # dy = way_point1[1] - y_second 
        # way_point1[1] = y_second - dy
        # way_point1 = np.array([x_second + way_point1[0], y_second - way_point1[1], 1.0])


        # print("way_point1 : ", way_point1)
        # print("way_point1W : ", way_point1W)
    if deltaTime > 8.5:
        # print("over time 8")

        control_command = [way_point1[0], way_point1[1], way_point1[2], sensor_data['yaw']]
 
    
    return control_command # Ordered as array with: [pos_x_cmd, pos_y_cmd, pos_z_cmd, yaw_cmd] in meters and radianss



def triangulate(R1, R2, initial_pos, second_pos, pink1, pink2) :
    W = 300
    vz = W/(2*np.tan(1.5/2))

    v1 = np.array([pink1[0], pink1[1], vz])
    v2 = np.array([pink2[0], pink2[1], vz])
    # v1 = np.reshape(v1, (3,1))
    # v2 = np.reshape(v2, (3,1))
    v1 = v1/np.linalg.norm(v1)
    v2 = v2/np.linalg.norm(v2)

    C1 = np.array([initial_pos[0]+0.03, initial_pos[1], initial_pos[2]+0.01])
    C2 = np.array([second_pos[0]+0.03, second_pos[1], second_pos[2]+0.01])

    # d1 = v1/np.linalg.norm(v1)
    # d2 = v2/np.linalg.norm(v2)

    # A = np.vstack((d1, d2)).T
    # b = C2 - C1

    # lambdas = np.linalg.lstsq(A, b, rcond=None)[0]
    # lambda1, lambda2 = lambdas

    # p1 = C1 + lambda1 * d1
    # p2 = C2 + lambda2 * d2

    # print("R1 : ", R1.shape)
    # print("v1 : ", v1.shape)

    r = R1 @ v1
    s = R2 @ v2
    print("r : ", r)
    print("s : ", s)

    # P = np.array([initial_pos[0]+0.03, initial_pos[1]+0, initial_pos[2]+0.01])
    # Q = np.array([second_pos[0]+0.03, second_pos[1]+0, second_pos[2]+0.01])

    # P = np.array([initial_pos[0], initial_pos[1], initial_pos[2]])
    # Q = np.array([second_pos[0], second_pos[1], second_pos[2]])

    A = np.array([[np.dot(r,r) , -np.dot(s,r)],
                  [np.dot(r,s) , -np.dot(s,s)]])
    # print("A : ", A.shape)

    # b = np.array([[np.dot((pink1 - pink2), r)], 
    #               [np.dot((pink1 - pink2), s)]])
    b1 = np.dot((pink2 - pink1), r)
    b2 = np.dot((pink2 - pink1), s)

    # print("pink2-pink1 : ", pink2-pink1, " r : ", r)

    # b1 = np.dot((Q - P), r)
    # b2 = np.dot((Q - P), s)
    # print("b1 : ", b1, " b2 : ", b2)
    b = np.array([b1, b2])
    # b = np.array([[np.dot((C1 - C2), r)], 
    #               [np.dot((C1 - C2), s)]])
    
    print("A : ", A)
    print("b : ", b)
    # print("b : ", b.shape)

    # A = np.column_stack((r, -s))
    # b = C2 - C1
    
    # alpha, beta = np.linalg.inv(A) @ b
    alpha, beta = A @ b
    # gamma = np.linalg.inv(A) #@ b
    # print("gamma : ", gamma)
    alpha = alpha + 1.5
    beta = beta + 0.5
    # gamma = np.linalg.lstsq(A, b, rcond=None)[0]
    # print("gamma : ", gamma)
    print("alpha : ", alpha, " beta : ", beta)
    print("C1 : ", C1)
    print("C2 : ", C2)

    F = C1 + alpha * r
    # F = R1 @ F
    G = C2 + beta * s
    print("F1 : ", F)
    print("G1 : ", G)
    alpha = alpha - 1.5
    beta = beta + 0.5

    F = C1 + alpha * r
    # F = R1 @ F
    G = C2 + beta * s
    # G = R2 @ G
    print("F2 : ", F)
    print("G2 : ", G)

    H = (F + G)/2 
    # H = (p1 + p2)/2
    print("to go to for waypoint1 : ", H)

    return H

def C2W(rotationMat) :
    R = np.eye(3)

    # Convert angles to radians
    roll = np.radians(rotationMat[0])
    pitch = np.radians(rotationMat[1])
    yaw = np.radians(rotationMat[2])

    # Create rotation matrices
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])
    
    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
    
    # Calculate the rotation matrix
    R = R_yaw @ R_pitch @ R_roll
    # print("R : ", R)
    return R


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

    originX = 150
    originY = 150
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) > 100:  # Minimum area to avoid small noises
            x, y, w, h = cv2.boundingRect(contour)
            # print(x + w/2, y + h/2)
            center = np.array([x + w/2, y + h/2])

            new_center = np.array([center[0] - originX, center[1] - originY, 1.0])
            return new_center  # Return the center of the pink square

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

# def distance_to_pink(sensor_data, TPC):
#     # pink_x, pink_y = detect_pink(camera_data)
#     return np.sqrt((sensor_data['x_global'] - TPC[0])**2 + (sensor_data['y_global'] - TPC[1])**2)

# def angle_to_pink(sensor_data, camera_data):
#     pink_x, pink_y = detect_pink(camera_data)
#     return np.arctan((pink_y - sensor_data['y_global'])/(pink_x - sensor_data['x_global']))