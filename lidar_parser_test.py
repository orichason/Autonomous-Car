import serial
import math
import time

lidar = serial.Serial(port="/dev/serial0", baudrate=230400)

POINTS_PER_PACKET = 12
prev_time = time.time()

def to_uint16(data, index):
    return data[index] | (data[index + 1] << 8)

def check_packet():
    first_byte = lidar.read(1)
    if first_byte and first_byte[0] == 0x54:
        rest = lidar.read(46)
        packet = first_byte + rest
        if len(rest) == 46 and packet[1] == 0x2C:
            return packet
    return None

def parse(packet):
    global prev_time
    if not packet:
        return None
    start_angle = to_uint16(packet, 4) / 100.0
    end_angle = to_uint16(packet, 42) / 100.0

    angle_diff = end_angle - start_angle
    if angle_diff < 0:
        angle_diff += 360

    angle_step = angle_diff / (POINTS_PER_PACKET - 1)

    points = []
    for i in range(POINTS_PER_PACKET):
        offset = 6 + i * 3
        distance = to_uint16(packet, offset)
        confidence = packet[offset + 2]
        angle = (start_angle + angle_step*i) % 360

        radians = math.radians(angle)
        x = distance * math.cos(radians)
        y = distance * math.sin(radians)

        points.append({
            "angle": angle,
            "distance": distance,
            "confidence":confidence,
            "x": x,
            "y": y
        })
    angles = [p["angle"] for p in points]
    print(f"[Packet] {len(points)} points → angles: {angles}")
    print(f"It took {time.time() - prev_time} second(s)")
    prev_time = time.time()
    return points

while True:
    packet = check_packet()
    parse(packet)
    #time.sleep(0.01)
