import numpy as np
import time
import cv2

# === Constants === #
WIDTH = 300
FOV = 1.5  # radians
F_PIXEL = WIDTH / (2 * np.tan(FOV / 2)) # = 161~

MAX_TARGETS = 5
MAX_LAPS = 3

# === Global Variables === #
waypoint = np.array([0.0, 0.0, 1.0])
control_command = [0.0, 0.0, 1.0, 0.0]

moved_to_second_position = False
waypoint_set = False
at_waypoint = False
nopink = True

mission_state = 0  # 0 = takeoff
target_index = 0   # which waypoint is targeted (0 to 4)
lap_count = 1#0

start_timed = False
timer = None
timer_done = None
index_current_setpoint = 0


setpoints = np.zeros((6, 4))
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


########### create function that analyses the center the detect pink rectangle returns 
########### and if there is several centers keep the closest one to the drone
########### so the pink function is just to detect if pink and if so return the return of this function
########### if more than 1 door in the image, take the closest one => calculate the distance

######## change the fact that the drone turns that much especially for the first waypoint
###### maybe doesnt even need to tale the cadre angle, juat turn when no pink
###### dron maybe doesn't need to turn at all if there's a condition on the fact that is doesn't see pink it turns

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
    global waypoint, control_command, target_index, start_timed, index_current_setpoint, timer
    global lap_count, setpoints, timer_done, nopink, mission_state

    # Get all Waypoints
    if nopink and mission_state != 0:
        R = np.zeros((3, 3))
        center1 = detect_pink_rectangle(sensor_data, camera_data, R, False)
        if center1 is not None:
            nopink = False
        else:
            return noPinkTurn(sensor_data)

    #### verifier si y'a du rose dans la camera sinon faut tourner jusqu'à ce qu'il y'en ai
    if target_index < MAX_TARGETS:
        control_command, waypoint = get_waypoint(sensor_data, camera_data)

        # When first lap is done, initialize setpoints
    if target_index == MAX_TARGETS and not start_timed:
        setpoints[0][:] = [1.0, 4.0, sensor_data['z_global'], 0.0]
        setpoints[1][:] = waypoint1
        setpoints[2][:] = waypoint2
        setpoints[3][:] = waypoint3
        setpoints[4][:] = waypoint4
        setpoints[5][:] = waypoint5

        start_timed = True
        index_current_setpoint = 0
        timer = None
        print("All waypoints acquired. Starting lap", lap_count)
        return [1.0, 4.0, sensor_data['z_global'], 0.0]

    # Laps 2 and beyond
    if start_timed and lap_count <= MAX_LAPS:

        control_command = trajectory_tracking(sensor_data, dt, setpoints, 0.1)#timepoints, setpoints_array, 0.15)

        # Check if lap is done
        if timer_done:
            lap_count += 1

            if lap_count != MAX_LAPS:
                print(f"Lap {lap_count} starting...")
                timer_done = None
                timer = None
                index_current_setpoint = 0
                control_command = setpoints[0]

    return control_command

def trajectory_tracking(sensor_data, dt, setpoints, tol):#, repeat=False):
    global index_current_setpoint, timer, lap_count

    if lap_count >= MAX_LAPS:
        return [setpoints[0][0], setpoints[0][1], setpoints[0][2], setpoints[0][3]]  # hover
        # return [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']]  # hover

    if index_current_setpoint == 0 and timer is None:
        timer = 0
        print(f"Starting Lap {lap_count + 1}")

    if timer is not None:
        timer += dt

    if index_current_setpoint < len(setpoints):
        current_setpoint = setpoints[index_current_setpoint]

        x, y, z = sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']
        distance = np.linalg.norm([x - current_setpoint[0], y - current_setpoint[1], z - current_setpoint[2]])

        if distance < tol:
            index_current_setpoint += 1
    else:
        # Lap complete
        lap_count += 1
        index_current_setpoint = 0
        timer = None
        print(f"Lap {lap_count} completed!")

    if index_current_setpoint < len(setpoints):
        # return pid_controller(sensor_data, setpoints[index_current_setpoint], dt)
        return [setpoints[index_current_setpoint][0], setpoints[index_current_setpoint][1], setpoints[index_current_setpoint][2], setpoints[index_current_setpoint][3]]
    else:
        # return [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']]
        return [setpoints[0][0], setpoints[0][1], setpoints[0][2], setpoints[0][3]]


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
        # time.sleep(2) # to diminue 
        initial_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']])
        R_initial = B2W(sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw'])
        center1 = detect_pink_rectangle(sensor_data, camera, R_initial, False)

        if center1 is None:
            return noPinkTurn(sensor_data), waypoint

        mission_state = 2

        if target_index == 0:
            new_pos = [initial_pos[0]-0.5, initial_pos[1]-0.5, initial_pos[2], sensor_data['yaw']+0.05]
        elif target_index == 1:
            new_pos = [initial_pos[0]+0.5, initial_pos[1]-0.5, initial_pos[2], sensor_data['yaw']-0.05]
        elif target_index == 2:
            new_pos = [initial_pos[0]+0.5, initial_pos[1]-0.5, initial_pos[2], sensor_data['yaw']+0.05]
        elif target_index == 3:
            new_pos = [initial_pos[0]+0.5, initial_pos[1]+0.75, initial_pos[2], sensor_data['yaw']+0.1]
        elif target_index == 4:
            new_pos = [initial_pos[0]-0.5, initial_pos[1]+0.75, initial_pos[2], sensor_data['yaw']+0.1]

        control_command = new_pos

    # CONFIRM MOVED TO SECOND POSITION
    if mission_state == 2 and not moved_to_second_position:
        # Check if we're close to second position
        if np.sqrt((sensor_data['x_global']-new_pos[0])**2 + (sensor_data['y_global']-new_pos[1])**2) < 0.1:
            moved_to_second_position = True

    # CAPTURE SECOND IMAGE + POSE
    if mission_state == 2 and moved_to_second_position:
        second_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']])
        R_second = B2W(sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw'])
        center2 = detect_pink_rectangle(sensor_data, camera, R_second, False)

        if center2 is None:
            return noPinkTurn(sensor_data), waypoint

        mission_state = 3

        # TRIANGULATE
        waypoint, alpha = triangulate(center1, center2, initial_pos, second_pos, R_initial, R_second)
        # waypoint[:4] = np.round(waypoint[:4], 2)
        
        # alpha = np.deg2rad(0)
        # yaw = np.deg2rad(0)

        waypoint_set = True

        # SET WAYPOINT
        # if target_index < 3:
        #     waypoint[0] = waypoint[0] + 0.15
        #     waypoint[1] = waypoint[1] + 0.15
        # else:
        #     waypoint[0] = waypoint[0] - 0.15
        #     waypoint[1] = waypoint[1] - 0.15

        if target_index == 0:
            waypoint1[:3] = waypoint
            waypoint1[3] = sensor_data['yaw']
            yaw = 0
        elif target_index == 1:
            waypoint2[:3] = waypoint
            waypoint2[3] = sensor_data['yaw']
            yaw = np.deg2rad(45)
            # yaw = 0
        elif target_index == 2:
            waypoint3[:3] = waypoint
            waypoint3[3] = sensor_data['yaw']
            yaw = np.deg2rad(90)
            # yaw = 0
        elif target_index == 3:
            waypoint4[:3] = waypoint
            waypoint4[3] = sensor_data['yaw']
            yaw = np.deg2rad(180)
            # yaw = 0
        elif target_index == 4:
            waypoint5[:3] = waypoint
            waypoint5[3] = sensor_data['yaw']
            # yaw = np.deg2rad(180)
            yaw = np.deg2rad(-90)
            # yaw = 0


    # GO TO WAYPOINT
    if waypoint_set and not at_waypoint:
        dist = np.sqrt((sensor_data['x_global']-waypoint[0])**2 + (sensor_data['y_global']-waypoint[1])**2)

        if dist < 0.1: # 0.15
            at_waypoint = True
            time.sleep(2)

            print("waypoints : ", waypoint1, waypoint2, waypoint3, waypoint4, waypoint5)

            mission_state = 1
            target_index += 1   # restart for next target
            moved_to_second_position = False
            waypoint_set = False
            at_waypoint = False

        control_command = [waypoint[0], waypoint[1], waypoint[2], alpha + yaw] #sensor_data['yaw']+yaw]

    return control_command, waypoint

def triangulate(c1, c2, initial_pos, second_pos, R1, R2):
    '''function to triangulate the position of the pink rectangle in the world frame'''
    global target_index

    v1 = np.array([c1[0], c1[1], F_PIXEL])
    v2 = np.array([c2[0], c2[1], F_PIXEL])

    d_x = c2[0]
    d_yaw = d_x * FOV / WIDTH
    print("d_yaw : ", d_yaw)

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

def detect_pink_rectangle(sensor_data, camera, R, inMain):

    global mission_state
 
    HSV_img = cv2.cvtColor(camera, cv2.COLOR_BGR2HSV)
    lower_pink = np.array([140, 50, 50])
    upper_pink = np.array([170, 255, 255])
    mask = cv2.inRange(HSV_img, lower_pink, upper_pink)
 
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_aera = 50
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_aera]

    centerXMin = -160
    numCircle = 0
    candidate_centers = []
    candidate_corners = []
    corners = None

    for cnt in contours:
        epsilon = 0.02 * cv2.arcLength(cnt, True)  # Approximation accuracy
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        if len(approx) == 4:  # Check if the detected shape has 4 corners
            corners = approx.reshape(4, 2)  # Convert to a (4,2) array
            centerDraw = np.mean(corners, axis=0)

            test_pos_corners = corners[:,0] < 7

            draw_center = tuple(np.round(centerDraw).astype(int))
            cv2.circle(mask, draw_center, 5, 0, -1)
            numCircle += 1

            # change origin to the center of the image
            corners = corners - WIDTH/2
            # get center of the rectangle
            center = np.mean(corners, axis=0)
            # Save for later distance comparison
            candidate_centers.append(center)
            
            # sort the corners
            corners = corners[np.argsort(np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0]))]
            candidate_corners.append(corners)

    if numCircle > 1:
        # find the closest center to the drone
        ### pour la distance, risque d'avoir besoin de la rotation pour avoir le centre dans le frame world ###
        distances = [np.sqrt((sensor_data['x_global']-center[0])**2 + (sensor_data['y_global']-center[1])**2) for center in candidate_centers]
        min_index = np.argmin(distances)

        center = candidate_centers[min_index]
        corners = candidate_corners[min_index]

        test_pos_corners = corners[:,0] < -140

        if test_pos_corners[0] or test_pos_corners[1] or test_pos_corners[2] or test_pos_corners[3]:
            center = None
    
    elif numCircle == 1:
        center = candidate_centers[0]
        if test_pos_corners[0] or test_pos_corners[1] or test_pos_corners[2] or test_pos_corners[3]:
            center = None

    else:
        center = None
    
    if inMain :
        cv2.imshow('Mask2', mask)
        cv2.waitKey(1)

    if not inMain :
        return center if 'center' in locals() else None
    return None


def noPinkTurn(sensor_data):
    '''function to turn until pink rectangle is detected'''
    global nopink

    return [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']+0.15]