import numpy as np
import time
import cv2

# === Imports === #
from lib.mapping_and_planning_examples import path_planning

# === Constants === #
WIDTH = 300
FOV = 1.5  # radians
F_PIXEL = WIDTH / (2 * np.tan(FOV / 2)) # = 161~

MAX_TARGETS = 5

# === Global Variables === #
waypoint = np.array([0.0, 0.0, 1.0])
control_command = [0.0, 0.0, 1.0, 0.0]

moved_to_second_position = False
waypoint_set = False
at_waypoint = False

mission_state = 0  # 0 = takeoff
target_index = 0   # which waypoint is targeted (0 to 4)

waypoint1 = np.zeros(4)
waypoint2 = np.zeros(4)
waypoint3 = np.zeros(4)
waypoint4 = np.zeros(4)
waypoint5 = np.zeros(4)

center1 = None
center2 = None
initial_pos = None
second_pos = None
new_pos = None
R_initial = None
R_second = None
alpha = 0
yaw = 0



##### compute position of center compared to center of image
## if on the left, move to the left and turn counterclockwise for second position
## if on the right, move to the right and turn clockwise for second position
def get_command(sensor_data, camera_data, dt):
    #  # NOTE: Displaying the camera image with cv2.imshow() will throw an error because GUI operations should be performed in the main thread.
    # # If you want to display the camera image you can call it main.py.
    # # # Take off example
    # # if sensor_data['z_global'] < 0.49 :
    # #     control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
    # #     return control_command

    # # ---- YOUR CODE HERE ----
    global waypoint, control_command, target_index

    # Get all Waypoints
    #### verifier si y'a du rose dans la camera sinon faut tourner jusqu'à ce qu'il y'en ai
    if target_index < MAX_TARGETS:
        control_command, waypoint = get_waypoint(sensor_data, camera_data)

    if target_index == MAX_TARGETS :
        path_planning(sensor_data, dt, [waypoint1, waypoint2, waypoint3, waypoint4, waypoint5], 0.1)

    return control_command

def get_waypoint(sensor_data, camera):
    '''function to get the waypoint to go to'''
    global waypoint, waypoint1, waypoint2, waypoint3, waypoint4, waypoint5
    global control_command, mission_state, target_index, moved_to_second_position, waypoint_set, at_waypoint
    global initial_pos, second_pos, new_pos, R_initial, R_second, center1, center2, alpha, yaw

    # DONE WITH ALL TARGETS
    if target_index >= MAX_TARGETS:
        return [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']], waypoint

    # TAKEOFF
    if mission_state == 0:
        if sensor_data['z_global'] < 0.95:
            return [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']], waypoint
        else:
            mission_state = 1
            print("Takeoff complete")

    # CAPTURE FIRST IMAGE + POSE
    if mission_state == 1:
        time.sleep(2)
        center1, _ = detect_pink_rectangle(camera, False)
        initial_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']])
        print("initial_pos : ", initial_pos)
        print("Center1:", center1)

        R_initial = B2W(sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw'])
        mission_state = 2
        print("Captured first position and pink rectangle")

        if target_index == 0:
            new_pos = [initial_pos[0]-0.5, initial_pos[1]-0.5, initial_pos[2], sensor_data['yaw']+0.1]
        elif target_index == 1:
            new_pos = [initial_pos[0]+0.5, initial_pos[1]-0.5, initial_pos[2], sensor_data['yaw']-0.1]
        elif target_index == 2:
            new_pos = [initial_pos[0]+0.5, initial_pos[1]-0.5, initial_pos[2], sensor_data['yaw']+0.1]
        elif target_index == 3:
            new_pos = [initial_pos[0]-0.5, initial_pos[1]+0.5, initial_pos[2], sensor_data['yaw']+0.15]
        elif target_index == 4:
            new_pos = [initial_pos[0]-0.5, initial_pos[1]+0.5, initial_pos[2], sensor_data['yaw']+0.15]

        control_command = new_pos

    # CONFIRM MOVED TO SECOND POSITION
    if mission_state == 2 and not moved_to_second_position:
        # Check if we're close to second position
        if np.sqrt((sensor_data['x_global']-new_pos[0])**2 + (sensor_data['y_global']-new_pos[1])**2) < 0.1:
            moved_to_second_position = True
            print("Moved to second position")

    # CAPTURE SECOND IMAGE + POSE
    if mission_state == 2 and moved_to_second_position:
        center2, _ = detect_pink_rectangle(camera, False)
        second_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']])
        print("second_pos : ", second_pos)
        print("Center2:", center2)

        R_second = B2W(sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw'])
        mission_state = 3
        print("Captured second position and pink rectangle")

        # TRIANGULATE
        waypoint, alpha = triangulate(center1, center2, initial_pos, second_pos, R_initial, R_second)

        if alpha < 0:
            alpha = sensor_data['yaw']

        waypoint_set = True
        print("Waypoint", target_index+1, "set :", waypoint)

        # SET WAYPOINT
        if target_index == 0:
            waypoint1[:3] = waypoint
            waypoint1[3] = sensor_data['yaw']
            yaw = 0
        elif target_index == 1:
            waypoint2[:3] = waypoint
            waypoint2[3] = sensor_data['yaw']
            yaw = 0
        elif target_index == 2:
            waypoint3[:3] = waypoint
            waypoint3[3] = sensor_data['yaw']
            yaw = np.deg2rad(90)
        elif target_index == 3:
            waypoint4[:3] = waypoint
            waypoint4[3] = sensor_data['yaw']
            yaw = np.deg2rad(150)
        elif target_index == 4:
            waypoint5[:3] = waypoint
            waypoint5[3] = sensor_data['yaw']
            yaw = np.deg2rad(180)


    # GO TO WAYPOINT
    if waypoint_set and not at_waypoint:
        dist = np.sqrt((sensor_data['x_global']-waypoint[0])**2 + (sensor_data['y_global']-waypoint[1])**2)

        if dist < 0.15:
            at_waypoint = True
            time.sleep(2)

            print("Reached waypoint ",target_index+1, " !")
            print("waypoints : ", waypoint1, waypoint2, waypoint3, waypoint4, waypoint5)

            # print("Current yaw : ", sensor_data['yaw'])

            mission_state = 1
            target_index += 1   # restart for next target
            moved_to_second_position = False
            waypoint_set = False
            at_waypoint = False
            print("target_index : ", target_index)


        # control_command = [waypoint[0], waypoint[1], waypoint[2], alpha]
        control_command = [waypoint[0], waypoint[1], waypoint[2], alpha + yaw] #sensor_data['yaw']+yaw]
       
    return control_command, waypoint


def triangulate(c1, c2, initial_pos, second_pos, R1, R2):
    '''function to triangulate the position of the pink rectangle in the world frame'''
    global target_index

    v1 = np.array([c1[0], c1[1], F_PIXEL])
    v2 = np.array([c2[0], c2[1], F_PIXEL])

    d_yaw = (c1[0] + c2[0])/2
    d_yaw = - d_yaw * FOV / WIDTH
    # print("d_yaw : ", target_index, d_yaw)

    P = np.array([initial_pos[0]+0.03, initial_pos[1], initial_pos[2]+0.01])
    Q = np.array([second_pos[0]+0.03, second_pos[1], second_pos[2]+0.01])

    r = R1 @ v1
    s = R2 @ v2

    A = np.column_stack((r, -s))
    b = Q - P

    alpha_beta = np.linalg.lstsq(A, b, rcond=None)[0]
    alpha, beta = alpha_beta

    F = P + alpha * r
    G = Q + beta * s

    H = (F + G)/2
    print("H : ", target_index, H)

    return H, d_yaw

def B2W(roll, pitch, yaw):
    '''function to convert the rotation matrix from body frame to world frame'''
    R_C2B = np.array([[0, 0, 1],
                  [-1, 0, 0,],
                  [0, -1, 0]])

    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw),  np.cos(yaw), 0],
                      [0, 0, 1]])

    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll),  np.cos(roll)]])

    R = R_yaw @ R_pitch @ R_roll @ R_C2B
    return R

def detect_pink_rectangle(camera, inMain):
    '''function to detect the pink rectangle in the camera frame'''

    HSV_img = cv2.cvtColor(camera, cv2.COLOR_BGR2HSV)
    lower_pink = np.array([140, 50, 50])
    upper_pink = np.array([170, 255, 255])
    mask = cv2.inRange(HSV_img, lower_pink, upper_pink)

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
            # sort the corners
            corners = corners[np.argsort(np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0]))]

    if inMain :
        cv2.imshow('Mask2', mask)
        cv2.waitKey(1)

    if not inMain :
        return center, corners if 'corners' in locals() else None
    return None