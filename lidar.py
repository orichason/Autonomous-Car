import serial
import math
import time
import threading

class Lidar:

    # Precomputed CRC8 lookup table for fast checksum verification
    CRC8_TABLE = [
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
        0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
        0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
        0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
        0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
        0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
        0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
        0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
        0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
        0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
        0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
        0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
        0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
        0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
        0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
        0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
        0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
        0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
        0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
        0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
        0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
        0x7f, 0x32, 0xe5, 0xa8
    ]

    # Calculate CRC8 checksum of a given packet
    def calc_crc8(self, packet: bytes) -> int:
        crc = 0
        for i in range(len(packet)):
            crc = self.CRC8_TABLE[(crc ^ packet[i]) & 0xff]
        return crc

    
    def __init__(self, port="/dev/serial0", baudrate=230400):
        self.ser = serial.Serial(port, baudrate) # Initialize serial connection to LiDAR
        self.POINTS_PER_PACKET = 12
        self.latest_full_scan = None
        self.point_buffer = [] # Stores all parsed LiDAR points
        self.lock = threading.Lock() # For safe access to shared buffer
        self.last_angle = 0
        self.last_scan_start_idx = 0

        self.full_scan_callback = None # User-defined function to call on full scan

        # Threads for reading data and scanning for wraparounds
        self.reader_thread = threading.Thread(target=self._serial_reader_thread, daemon=True)
        self.scanner_thread = threading.Thread(target=self._lidar_scanner_thread, daemon=True)

    # Converts two bytes from data into a uint16 integer
    def to_uint16(self, data, index):
        return data[index] | (data[index+1] << 8)
    
    # Reads one full 47-byte packet from serial and validates it
    def check_packet(self):
        first_byte = self.ser.read(1)
        if first_byte and first_byte[0] == 0x54:
            rest = self.ser.read(46)
            packet = first_byte + rest
            if len(rest) == 46 and packet[1] == 0x2C:
                expected_crc8 = packet[46]
                calculated_crc8 = self.calc_crc8(packet[:46])
                if expected_crc8 == calculated_crc8:
                    return packet
        return None

    # Parses a 47-byte packet into 12 angle-encoded distance points
    def parse_packet(self, packet):
        if not packet:
            return None
        start_angle = self.to_uint16(packet, 4) / 100.0
        end_angle = self.to_uint16(packet, 42) / 100.0

        angle_diff = end_angle - start_angle
        if angle_diff < 0:
            angle_diff += 360

        angle_step = angle_diff / (self.POINTS_PER_PACKET - 1)

        points = []
        for i in range(self.POINTS_PER_PACKET):
            offset = 6 + i * 3
            distance = self.to_uint16(packet, offset)
            confidence = packet[offset + 2]
            angle = (start_angle + angle_step*i) % 360

            points.append({
                "angle": angle,
                "distance": distance,
                "confidence":confidence,
            })

        return points

    # Continuously reads packets from serial and adds parsed points to shared buffer
    def _serial_reader_thread(self):
        while True:
            packet = self.check_packet()
            if packet:
                raw_points = self.parse_packet(packet)
                with self.lock:
                    self.point_buffer.extend(raw_points)
    
    # Continuously scans buffer for full 360° scan by detecting angle wraparound
    def _lidar_scanner_thread(self):
        while True:
            with self.lock:
                buffer_copy = self.point_buffer.copy()

            for i in range(len(buffer_copy)):
                angle = buffer_copy[i]["angle"]
                if i > 0 and self.last_angle > angle:
                    scan = buffer_copy[self.last_scan_start_idx:i]
                    # print(f"wraparound detected: took {(time.time() - self.prev_time) * 1000} milliseconds")
                    # self.prev_time = time.time()
                    #print(f"old ange = {self.last_angle} and new angle = {angle}")
                    if self.full_scan_callback:
                        self.full_scan_callback(scan)

                    self.last_scan_start_idx = i

                self.last_angle = angle

            with self.lock:
                if self.last_scan_start_idx > 0:
                    self.point_buffer = self.point_buffer[self.last_scan_start_idx:]
                    self.last_scan_start_idx = 0

            time.sleep(0.002)

    # Gracefully close serial port              
    def close(self):
        self.ser.close()

    # Register a callback to be invoked on every full 360° scan
    def set_callback(self, callback_func):
        self.full_scan_callback = callback_func

    # Start background threads for serial reading and scanning
    def start_scanning(self):
        self.reader_thread.start()
        self.scanner_thread.start()