from lidar import Lidar
import time
lidar = Lidar()
lidar.start_scanning()

SERVO_CENTER = 1618
SERVO_LEFT = SERVO_CENTER - 300
SERVO_RIGHT = SERVO_CENTER + 300

# Throttle control (very slow forward only)
THROTTLE_STOP = 1500
THROTTLE_CRUISE = 1550  # Gentle forward
THROTTLE_SLOW = 1505    # Cautious during obstacle avoidance
MAX_THROTTLE = 1520     # Hard upper safety limit
MIN_THROTTLE = 1500     # No reverse

DANGER_THRESHOLD = 1000  # mm (1 meter)

previous_time = time.time()

def in_front(angle): return angle <= 30 or angle >= 330
def is_left(angle): return 240 <= angle <= 300
def is_right(angle): return 60 <= angle <= 120


def on_full_scan(scan):
    #global previous_time
    #print(f"Took {(time.time() - previous_time) * 1000:.2f} ms")
    #previous_time = time.time()
    obj_detected = False
    for p in scan:
        if in_front(p["angle"]) and p["distance"] <= DANGER_THRESHOLD and p["confidence"] >= 50:
            print(f"Object detected in front. Angle: {p['angle']}, Distance: {p['distance']}")
            obj_detected = True
            break
    if not obj_detected:
        return None
    

    left_points = [p["distance"] for p in scan if 240 <= p["angle"] <= 300]
    right_points = [p["distance"] for p in scan if 60 <= p["angle"] <= 120]

    left_avg = sum(left_points) / len(left_points) if left_points else 0
    right_avg = sum(right_points) / len(right_points) if right_points else 0
    #print(f"left average = {left_avg}")
    #print(f"right average = {right_avg}")
    
    return SERVO_LEFT if left_avg >= right_avg else SERVO_RIGHT

lidar.set_callback(on_full_scan)

while True:
    time.sleep(0.01)