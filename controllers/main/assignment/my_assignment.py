import numpy as np
import time
import cv2

# The available ground truth state measurements can be accessed by calling sensor_data[item]. All values of "item" are provided as defined in main.py within the function read_sensors.
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

start_time = time.time()

## camera info
WIDTH = 300
FOV = 1.5 #radians
F_PIXEL = WIDTH/(2*np.tan(FOV/2))

start_position = True
second_position = False
firstWaypoint = True
secondWaypoint = False

corners1 = None
corners2 = None
center1 = None
center2 = None
initial_pos = np.zeros(4)
second_pos = np.zeros(4)
R_initial = np.eye(3)
R_second = np.eye(3)

alpha = 0
dz = 0
deltaT = 7
control_command = np.zeros(4)
waypoint = np.zeros(3)
waypoint1 = np.zeros(3)
waypoint2 = np.zeros(3)
waypoint3 = np.zeros(3)
waypoint4 = np.zeros(3)
waypoint5 = np.zeros(3)


def get_command(sensor_data, camera_data, dt):
    global start_time, F_PIXEL, deltaT, control_command, firstWaypoint, secondWaypoint
    global start_position, second_position, corners1, corners2, center1, center2, initial_pos, second_pos, R_initial, R_second
    global alpha, dz, waypoint, waypoint1, waypoint2, waypoint3, waypoint4, waypoint5

    # NOTE: Displaying the camera image with cv2.imshow() will throw an error because GUI operations should be performed in the main thread.
    # If you want to display the camera image you can call it main.py.

    # Take off example
    if sensor_data['z_global'] < 0.49:
        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
        return control_command

    # ---- YOUR CODE HERE ----
    ## start time of the simulation
    deltaTime = time.time() - start_time
    # print("Delta Time:", deltaTime)

    ## get the camera data
    camera = camera_data.copy()


    if deltaTime < deltaT+8 and firstWaypoint :
        # print("Delta Time:", deltaTime)
        # print("deltaT:", deltaT)
        deltaTime = time.time() - start_time

        control_command, waypoint1 = get_waypoint(sensor_data, camera, deltaTime)
    
    if deltaTime < deltaT+8 and secondWaypoint :
        # print("in second waypoint")
        deltaTime = time.time() - start_time

        control_command, waypoint2 = get_waypoint(sensor_data, camera, deltaTime)
        # print("Waypoint 2:", waypoint2)
    
    if not firstWaypoint and not secondWaypoint:
        print("done first and second waypoint")


    return control_command # Ordered as array with: [pos_x_cmd, pos_y_cmd, pos_z_cmd, yaw_cmd] in meters and radians

def get_waypoint(sensor_data, camera, deltaTime):
    global start_time, F_PIXEL, deltaT, control_command, firstWaypoint, secondWaypoint
    global start_position, second_position, corners1, corners2, center1, center2, initial_pos, second_pos, R_initial, R_second
    global alpha, dz, waypoint, waypoint2

    ## let the drone take off
    if deltaTime <  deltaT:

        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]

    ## once drone stable enough, get starting position information ##
    elif deltaTime < deltaT+2.5 and deltaTime > deltaT :

        if start_position :
            ## detect the pink rectangle for image 1 ##
            center1, corners1 = detect_pink_rectangle(camera, False)

            # angle = np.arccos(FOV/center1[0])
            # print("angle : ", angle)

            ## get the initial position and orientation of the drone ##
            initial_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']])
            print("initial_pos : ", initial_pos)
            ## get the rotation matrix from camera frame to body frame to world frame R1 ##
            R_C2B1_roll, R_C2B1_pitch, R_C2B1_yaw = C2B([sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']])
            R_initial = B2W(R_C2B1_roll, R_C2B1_pitch, R_C2B1_yaw)

            start_position = False
            second_position = True
            # print("Corners1:", corners1)
            print("Center1:", center1)
        angle = np.arccos(FOV/center1[0])
        # print("angle : ", angle)
        ## move drone to the next position for 2nd image ##
        # control_command = [0.75, 3.5, 1.0, sensor_data['yaw']+0.1]
        if initial_pos[3] > 0:
            next_pos = np.array([initial_pos[0]+0.25, initial_pos[1]+0.5, 1.0, initial_pos[3]-0.1])
        else:
            next_pos = np.array([initial_pos[0]-0.25, initial_pos[1]-0.5, 1.0, initial_pos[3]+0.1])
        # print("next_pos : ", next_pos)
        # control_command = [next_pos[0], next_pos[1], next_pos[2], sensor_data['yaw']+0.1]
        control_command = [next_pos[0], next_pos[1], next_pos[2], next_pos[3]]

    elif deltaTime < deltaT+4 and deltaTime > deltaT+2.5:
        ## let drone move to 2nd position ##
        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]

    ## once drone is stable enough, get the 2nd position information ##
    if deltaTime > deltaT+4  and deltaTime < deltaT+6.5:

        if second_position :
            ## detect the pink rectangle for image 2 ##
            center2, corners2 = detect_pink_rectangle(camera, False)

            ## get the second position and orientation of the drone ##
            second_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']])
            print("second_pos : ", second_pos)
            ## get the rotation matrix from camera frame to body frame to world frame R2 ##
            R_C2B2_roll, R_C2B2_pitch, R_C2B2_yaw = C2B([sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']])
            R_second = B2W(R_C2B2_roll, R_C2B2_pitch, R_C2B2_yaw)

            second_position = False
            # print("Corners2:", corners2)
            print("Center2:", center2)

            ## triangulate the position of the pink rectangle in the world frame => get 1st waypoint ##
            waypoint, dz, alpha = triangulate(center1, center2, initial_pos, second_pos, R_initial, R_second)
            print("yaw angle : ", sensor_data['yaw'])
            print("alpha from triangulation : ", alpha)
            dz = dz + sensor_data['z_global']
            waypoint[2] = dz
            print("waypoint : ", waypoint)

        # control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
        ## move to the 1st waypoint ##
        
        control_command = [waypoint[0], waypoint[1], waypoint[2], alpha]

    if deltaTime > deltaT+6.5 and deltaTime < deltaT+8:
        initial_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']])
        control_command = [waypoint[0], waypoint[1], waypoint[2], alpha]
        
        start_position = True
        # print("deltaTime:", deltaTime)
    
    if deltaTime > deltaT + 7.5 and start_position:
        print("IN deltaTime = deltaT+8")
        print("deltaTime:", deltaTime)
        deltaT = deltaT + 10
        print("deltaT:", deltaT)
        firstWaypoint = False
        secondWaypoint = True

        if waypoint2.all() != 0:
            secondWaypoint = False
        # print("Waypoint:", waypoint)


        
        # return control_command, waypoint
    # print("Waypoint:", waypoint)

    return control_command, waypoint
    


# triangulate the position of the pink rectangle in the world frame
def triangulate(c1, c2, initial_pos, second_pos, R1, R2):
    '''function to triangulate the position of the pink rectangle in the world frame'''

    v1 = np.array([c1[0], c1[1], F_PIXEL])
    v2 = np.array([c2[0], c2[1], F_PIXEL])

    z = (c1[1] + c2[1])/2
    d_yaw = (c1[0] + c2[0])/2
    d_yaw = - d_yaw * FOV / WIDTH
    print("d_yaw : ", d_yaw)
    # print("z : ", z)

    theta = z * FOV / WIDTH

    dz = - np.tan(theta)
    # print("dz : ", dz)

    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    P = np.array([initial_pos[0]+0.03, initial_pos[1], initial_pos[2]])
    Q = np.array([second_pos[0]+0.03, second_pos[1], second_pos[2]])

    r = R1 @ v1
    s = R2 @ v2

    print("r : ", r)
    print("s : ", s)

    A = np.array([[np.dot(r,r) , -np.dot(s,r)],
                  [np.dot(r,s) , -np.dot(s,s)]])

    b2 = np.dot((Q - P), s)
    b1 = np.dot((Q - P), r)

    b = np.array([[b1], [b2]])

    alpha, beta = np.linalg.inv(A) @ b

    # alpha = alpha + 0.5
    # beta = beta + 0.6

    F = P + alpha * r
    G = Q + beta * s

    H = (F + G)/2

    return H, dz, d_yaw

# rotation from camera frame to body frame
def C2B(rotationMat) :
    '''function to convert the rotation matrix from camera frame to body frame'''

    roll = rotationMat[0]
    pitch = rotationMat[1]
    yaw = rotationMat[2]

    angles = np.array([[roll],
                       [pitch],
                       [yaw]])

    R = np.array([[0, 0, 1],
                  [-1, 0, 0,],
                  [0, -1, 0]])

    Rotation = R @ angles

    Rroll = Rotation[0][0]
    Rpitch = Rotation[1][0]
    Ryaw = Rotation[2][0]

    return Rroll, Rpitch, Ryaw

# rotation from body frame to world frame
def B2W(roll, pitch, yaw) :
    '''function to convert the rotation matrix from body frame to world frame'''

    R = np.eye(3)

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

    return R

def detect_rectangle_orientation(corners1, corners2):
    H, _ = cv2.findHomography(corners1, corners2)
    print("Homography Matrix:\n", H)



    # get the center of the rectangle
    center1 = np.mean(corners1, axis=0)
    center2 = np.mean(corners2, axis=0)

    # sort the corners
    corners1 = corners1[np.argsort(np.arctan2(corners1[:, 1] - center1[1], corners1[:, 0] - center1[0]))]
    corners2 = corners2[np.argsort(np.arctan2(corners2[:, 1] - center2[1], corners2[:, 0] - center2[0]))]

    return center1, center2, corners1, corners2

def angle_to_pink(corners, center):
#     pink_x, pink_y = detect_pink(camera_data)
#     return np.arctan((pink_y - sensor_data['y_global'])/(pink_x - sensor_data['x_global']))
# detect the pink rectangle in the camera frame
    # center, corners = detect_pink_rectangle(camera_data, True)
    if corners is not None:
        # get the center of the rectangle
        # center = np.mean(corners, axis=0)
        # sort the corners
        corners = corners[np.argsort(np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0]))]
        # get the angle of the rectangle
        angle = np.arctan2(center[1], center[0])
        return angle
    return None

def detect_pink_rectangle(camera, inMain):
    '''function to detect the pink rectangle in the camera frame'''

    HSV_img = cv2.cvtColor(camera, cv2.COLOR_BGR2HSV)
    lower_pink = np.array([140, 50, 50])
    upper_pink = np.array([170, 255, 255])
    mask = cv2.inRange(HSV_img, lower_pink, upper_pink)

    # if inMain :
    #     cv2.imshow('Mask', mask)
    #     cv2.waitKey(1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_aera = 50
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_aera]

    for cnt in contours:
        epsilon = 0.02 * cv2.arcLength(cnt, True)  # Approximation accuracy
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        if len(approx) == 4:  # Check if the detected shape has 4 corners
            corners = approx.reshape(4, 2)  # Convert to a (4,2) array

            centerDraw = np.mean(corners, axis=0)
            draw_center = tuple(np.round(centerDraw).astype(int))
            cv2.circle(mask, draw_center, 5, 0, -1)
            # change origin to the center of the image
            corners = corners - WIDTH/2
            # get center of the rectangle
            center = np.mean(corners, axis=0)
            # print("center : ", center)
            # sort the corners
            corners = corners[np.argsort(np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0]))]
            # angle = np.arctan2(center[1], center[0])
            # print("angle : ", angle)

    if inMain :
        cv2.imshow('Mask2', mask)
        cv2.waitKey(1)

    if not inMain :
        return center, corners if 'corners' in locals() else None
    return None
