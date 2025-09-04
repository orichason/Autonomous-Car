from lidar import Lidar
import time
import math

# Initialize the Lidar object and begin scanning in background threads
lidar = Lidar()
lidar.start_scanning()

# Constants for steering control
SERVO_CENTER = 1500 #was 1618
SERVO_LEFT = SERVO_CENTER - 300
SERVO_RIGHT = SERVO_CENTER + 300

# Constants for throttle control
THROTTLE_STOP = 1500    # No throttle (neutral)
THROTTLE_CRUISE = 1600  # Gentle forward
THROTTLE_SLOW = 1545    # Cautious during obstacle avoidance
MAX_THROTTLE = 1600     # Max throttle limit (safety cap)
MAX_DISTANCE = 1000

DISPARITY_THRESHOLD = 200

WIDTH_OF_CAR = 300 #mm
LIDAR_RESOLUTION = 0.717

# Distance threshold to consider an obstacle dangerous (in mm)
DANGER_THRESHOLD = 300   # 1 meter
obj_detected = False
target_angle = 0
target_distance = 0


previous_time = time.time()

def object_detected():
    global obj_detected
    return obj_detected

def get_open_angle():
    global target_angle
    return target_angle

def get_target_distance():
    global target_distance
    return target_distance

# Check if an angle is in the front-facing field of view
def in_front(angle): return angle <= 30 or angle >= 330

# Check if an angle is on the left side
def is_left(angle): return 240 <= angle <= 300

# Check if an angle is on the right side
def is_right(angle): return 60 <= angle <= 120

def get_disparities(filtered): # nOTE: SEE IF YOU CAN USE DISTANCES INSTEAD OF FILTER
    disparities = []

    n = len(filtered)

    if n < 2:
        return disparities
    
    for i in range(n - 1):
        d1 = filtered[i][1]
        d2 = filtered[i+1][1]
        if abs(d1 - d2) > DISPARITY_THRESHOLD:
            # Always extend toward the farther point (higher distance)
            print(f"d1 = {d1} , angle1 = {filtered[i][0]}")
            print(f"d2 = {d2} , angle2 = {filtered[i+1][0]}")
            if d2 >= d1:
                farther_idx = i + 1
                closer_distance = d1
            else:
                farther_idx = i
                closer_distance = d2

            disparities.append((farther_idx, closer_distance))
 
    return disparities

# Callback function called every time a full 360° LiDAR scan is complete
def on_full_scan(scan): #clean up function, split into little functions
    global obj_detected, target_angle, target_distance
    obj_detected = False
    target_distance = 0
    target_angle = 0

    filtered = [(p["angle"], p["distance"]) for p in scan if (p["angle"] <= 90 or p["angle"] >= 270) and p["confidence"] > 20]

    if not filtered:
        target_angle = SERVO_CENTER
        return
    
    distances = [d for _, d in filtered] # getting just the distances from filtered list
    
    disparities = get_disparities(filtered)

    for farther_idx, closer_distance in disparities:
        BUFFER = 0 #LEFT OFF HERE CHANGING BUFFER FROM 10 TO 30. TRYING TO DEBUGG MY DISPARITY SOMETHING IS WRONG WITH IT
        effective_width = WIDTH_OF_CAR + BUFFER
        theta = math.atan(effective_width / closer_distance)
        theta_deg = math.degrees(theta)
        samples_to_cover = int(theta_deg * LIDAR_RESOLUTION)
        print(f"samples to cover: {samples_to_cover}")
        i = farther_idx
        samples_filled = 0

        while samples_filled < samples_to_cover:
            if i >= len(distances):
                break
            if distances[i] > closer_distance: # Only overwrite if it's farther
                distances[i] = closer_distance
            samples_filled += 1
            i += 1
    
    temp_distance = -1
    temp_angle = SERVO_CENTER

    for i in range(len(distances)):
        if distances[i] > temp_distance:
            temp_distance = distances[i]
            temp_angle = filtered[i][0]

    target_distance = temp_distance
    target_angle = temp_angle

# Register the callback function on the LiDAR
lidar.set_callback(on_full_scan)

# Keep the main thread alive forever while background threads are running
# threading.Event().wait()