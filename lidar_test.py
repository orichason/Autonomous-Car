from lidar import Lidar
import time

# Adjust this if your LiDAR shows up as a different device
PORT = "/dev/ttyS0"
BAUD = 230400


try:
    lidar = Lidar()
    prev_time = time.time()
    previous_angle = 0
    while True:
        packet = lidar.check_packet()
        points = lidar.parse_packet(packet)
        if not points:
            continue

        #for p in points:
            #print(f"angle = {p['angle']}")
            # if previous_angle is not None:
            #     if previous_angle > 300 and p["angle"] < 60:
            #         print(f"wrap around detected. new angle = {p['angle']} and old angle = {previous_angle}")
            #         print(f"took {time.time() - prev_time} second(s)")
            #         prev_time = time.time()

            # previous_angle = p["angle"]



except KeyboardInterrupt:
    print("Interrupted by keyboard")
finally:
    lidar.close()