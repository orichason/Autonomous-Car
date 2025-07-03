import serial
import math
import time
import threading

class Lidar:

    CRC8_TABLE = [
        0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
        0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
        0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
        0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
        0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
        0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
        0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
        0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
        0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
        0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
        0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
        0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
        0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
        0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
        0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
        0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8
    ]

    def calc_crc8(self, packet: bytes) -> int:
        crc = 0
        for b in packet:
            crc = self.CRC8_TABLE[(crc ^ b) & 0xFF]
        return crc

    
    def __init__(self, port="/dev/serial0", baud=230400, timeout=0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.POINTS_PER_PACKET = 12
        self.latest_full_scan = None
        self.point_buffer = []
        self.lock = threading.Lock()
        #self.last_angle = 0
        #self.last_scan_start_idx = 0
        self.buffer = bytearray()

        self.full_scan_callback = None

        self.reader_thread = threading.Thread(target=self._serial_reader_thread, daemon=True)
        self.scanner_thread = threading.Thread(target=self._lidar_scanner_thread, daemon=True)

    def to_uint16(self, data, index):
        return data[index] | (data[index+1] << 8)
    
    # def check_packet(self):
    #     print(f"Bytes waiting in buffer: {self.ser.in_waiting}")
    #     while self.ser.in_waiting >= 47:
    #         byte = self.ser.read(1)
    #         if byte and byte[0] == 0x54:
    #             rest = self.ser.read(46)
    #             packet = byte + rest
    #             if len(packet) == 47 and packet[1] == 0x2C:
    #                 checksum = packet[46]
    #                 if self.calc_crc8(packet) == checksum:
    #                     print("checksum good")
    #                     return packet
    #     return None
    
    def check_packet(self):
        self.buffer += self.ser.read(self.ser.in_waiting)

        while len(self.buffer) >= 47:
            if self.buffer[0] == 0x54 and self.buffer[1] == 0x2C:
                packet = self.buffer[:47]
                if self.calc_crc8(packet) == packet[46]:
                    self.buffer = self.buffer[47:]
                    return packet
                else:
                    self.buffer = self.buffer[1:]

            else:
                self.buffer = self.buffer[1:]
        return None
            
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
        return points

    def _serial_reader_thread(self):
        while True:
            packet = self.check_packet()
            if packet:
                raw_points = self.parse_packet(packet)
                points = [p for p in raw_points if p["confidence"] >= 30 and p["distance"] < 12000]
               
                with self.lock:
                    self.point_buffer.extend(points)
    

    def _lidar_scanner_thread(self):
        while True:
            with self.lock:
                buffer_copy = self.point_buffer.copy()

            scan_start_idx = 0
            last_angle = 0
            for i in range(len(buffer_copy)):
                angle = buffer_copy[i]["angle"]
                if i > 0 and last_angle > angle:
                    scan = buffer_copy[scan_start_idx:i]
                    #print(f"old ange = {last_angle} and new angle = {angle}")
                    if self.full_scan_callback:
                        self.full_scan_callback(scan)

                    scan_start_idx = i

                last_angle = angle

            with self.lock:
                if scan_start_idx > 0:
                    self.point_buffer = self.point_buffer[scan_start_idx:]

            time.sleep(0.01)
  
    def close(self):
        self.ser.close()

    def set_callback(self, callback_func):
        self.full_scan_callback = callback_func

    def start_scanning(self):
        self.reader_thread.start()
        self.scanner_thread.start()