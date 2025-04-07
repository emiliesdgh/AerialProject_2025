import numpy as np
import time
import cv2

# === Constants === #
WIDTH = 300
FOV = 1.5  # radians
F_PIXEL = WIDTH / (2 * np.tan(FOV / 2))

# === State Machine === #
STATE_TAKEOFF = 0
STATE_CAPTURE_IMAGE_1 = 1
STATE_MOVE_FOR_IMAGE_2 = 2
STATE_CAPTURE_IMAGE_2 = 3
STATE_TRIANGULATE = 4
STATE_MOVE_TO_WAYPOINT_1 = 5
STATE_WAIT_AT_WAYPOINT_1 = 6
STATE_DONE = 7

# === Global Variables === #
state = STATE_TAKEOFF
waypoint = np.array([0.0, 0.0, 1.0])
control_command = [0.0, 0.0, 1.0, 0.0]
has_taken_off = False  # declare this globally at the top
captured_first = False
moved_to_second_position = False
captured_second = False
waypoint_set = False
at_waypoint = False

start_position = True
second_position = False

center1 = None
center2 = None
initial_pos = None
second_pos = None
new_pos = None
R_initial = None
R_second = None
alpha = 0



##### compute position of center compared to center of image
## if on the left, move to the left and turn counterclockwise for second position
## if on the right, move to the right and turn clockwise for second position
def get_command(sensor_data, camera_data, dt):
    # global waypoint
    #  # NOTE: Displaying the camera image with cv2.imshow() will throw an error because GUI operations should be performed in the main thread.
    # # If you want to display the camera image you can call it main.py.

    # # # Take off example
    # # if sensor_data['z_global'] < 0.49 :
    # #     control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
    # #     return control_command

    # # ---- YOUR CODE HERE ----
    # print(f"[DEBUG] Current state: {state}, Position: ({sensor_data['x_global']:.2f}, {sensor_data['y_global']:.2f}, {sensor_data['z_global']:.2f}), Yaw: {sensor_data['yaw']:.2f}")

    # control_command, waypoint = get_waypoint(sensor_data, camera_data)

    # def get_command(sensor_data, camera_data, dt):
    global has_taken_off, waypoint

    # if not has_taken_off:
    #     if sensor_data['z_global'] < 0.95:
    #         # Keep taking off slowly
    #         return [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
    #     else:
    #         has_taken_off = True  # Reached desired height, move to next phase

    # Call waypoint logic after takeoff
    control_command, waypoint = get_waypoint(sensor_data, camera_data)
    return control_command

def get_waypoint(sensor_data, camera):
    global state, control_command, has_taken_off
    global captured_first, moved_to_second_position, captured_second, waypoint_set, at_waypoint
    global start_position, second_position
    global initial_pos, second_pos, new_pos, R_initial, R_second
    global center1, center2, waypoint, alpha

    # # if state == STATE_TAKEOFF:
    # #     if sensor_data['z_global'] < 0.49:
    # #         control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
    # #     else:
    # #         state = STATE_CAPTURE_IMAGE_1

    # TAKEOFF
    if not has_taken_off:
        if sensor_data['z_global'] < 0.95:
            return [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']], waypoint
        else:
            has_taken_off = True
            print("Takeoff complete")

    # CAPTURE FIRST IMAGE + POSE
    if has_taken_off and not captured_first:
        center1, _ = detect_pink_rectangle(camera, False)
        initial_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']])
        print("initial_pos : ", initial_pos)
        print("Center1:", center1)

        # R_initial = B2W(*C2B([sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']]))
        R_C2B1_roll, R_C2B1_pitch, R_C2B1_yaw = C2B([sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']])
        R_initial = B2W(R_C2B1_roll, R_C2B1_pitch, R_C2B1_yaw)
        captured_first = True
        print("Captured first position and pink rectangle")

        if center1[1] < 0 :
            print("here 1")
            new_pos = [initial_pos[0]-0.5, initial_pos[1]-0.5, initial_pos[2], sensor_data['yaw']+0.1]
        elif center1[1] > 0 :
            print("here 2")
            new_pos = [initial_pos[0]+0.5, initial_pos[1]+0.5, initial_pos[2], sensor_data['yaw']-0.1]
        
        control_command = new_pos
    
    # CONFIRM MOVED TO SECOND POSITION
    if captured_first and not moved_to_second_position:
        # Check if we're close to second position
        if np.sqrt((sensor_data['x_global']-new_pos[0])**2 + (sensor_data['y_global']-new_pos[1])**2) < 0.1:
            moved_to_second_position = True
            print("Moved to second position")

    # CAPTURE SECOND IMAGE + POSE
    if moved_to_second_position and not captured_second:
        center2, _ = detect_pink_rectangle(camera, False)
        second_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']])
        print("second_pos : ", second_pos)
        print("Center2:", center2)
        
        R_C2B2_roll, R_C2B2_pitch, R_C2B2_yaw = C2B([sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']])
        R_second = B2W(R_C2B2_roll, R_C2B2_pitch, R_C2B2_yaw)      
        captured_second = True
        print("Captured second position and pink rectangle")

        # TRIANGULATE
        waypoint, dz, alpha = triangulate(center1, center2, initial_pos, second_pos, R_initial, R_second)
        waypoint[2] = dz + sensor_data['z_global'] # Adjust height
        waypoint_set = True
        print("Waypoint set:", waypoint)

    # # TRIANGULATE
    # if captured_second and not waypoint_set:
    #     waypoint, dz, alpha = triangulate(center1, center2, initial_pos, second_pos, R_initial, R_second)
    #     waypoint[2] = dz + sensor_data['z_global'] # Adjust height
    #     waypoint_set = True
    #     print("Waypoint set:", waypoint)

    # GO TO WAYPOINT
    if waypoint_set and not at_waypoint:
        dist = np.sqrt((sensor_data['x_global']-waypoint[0])**2 + (sensor_data['y_global']-waypoint[1])**2)

        if dist < 0.15:
            at_waypoint = True
            print("Reached waypoint!")
        control_command = [waypoint[0], waypoint[1], waypoint[2], alpha]

    return control_command, waypoint


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
    print("H : ", H)

    return H, dz, d_yaw

def C2B(rotationMat):
    # roll, pitch, yaw = rotationMat
    # R = np.array([[0,  0, 1], 
    #               [-1, 0, 0], 
    #               [0, -1, 0]])
    # angles = R @ np.array([[roll], [pitch], [yaw]])
    # return angles[0][0], angles[1][0], angles[2][0]
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

def B2W(roll, pitch, yaw):
    '''function to convert the rotation matrix from body frame to world frame'''

    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw),  np.cos(yaw), 0],
                      [0, 0, 1]])

    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll),  np.cos(roll)]])

    R = R_yaw @ R_pitch @ R_roll
    return R

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
