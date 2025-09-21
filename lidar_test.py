from lidar import Lidar
import time
import serial


# Adjust this if your LiDAR shows up as a different device
PORT = "/dev/ttyS0"
BAUD = 230400


# lidar = serial.Serial(port="/dev/serial0", baudrate=230400)

# while True:
#     print("------------------------------")
#     packet = lidar.read(47)

#     # Split the raw bytes into chunks of ~10 for readability
#     hex_bytes = ' '.join(f"{b:02x}" for b in packet)

#     # Print 3–4 lines
#     print(hex_bytes[0:30])
#     print(hex_bytes[30:60])
#     print(hex_bytes[60:90])
#     print(hex_bytes[90:])

try:
    lidar = Lidar(port="/dev/serial0", baudrate=230400)
    prev_time = time.time()
    while True:
        packet = lidar.check_packet()
        points = lidar.parse_packet(packet)
        if points:
            for p in points:
                print(f"Angle = {p['angle']:.2f}°, Distance = {p['distance']}mm, Confidence = {p['confidence']}")

except KeyboardInterrupt:
    print("Interrupted by keyboard")
finally:
    lidar.close()