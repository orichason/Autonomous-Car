import serial
import struct
import time
import pygame
import autonomous

#pi port: '/dev/ttyUSB0'
#pc port: 'COM4'
mcu_serial = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
time.sleep(2)
# lidar = Lidar()


# SERVO_CENTER = 1618
# SERVO_LEFT = SERVO_CENTER - 300
# SERVO_RIGHT = SERVO_CENTER + 300

# need to write autonomous logic 

def map_value(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def get_steering(left_stick_x):
    steer_raw = int(left_stick_x * 128) # Pygame returns -1.0 to 1.0

    if steer_raw < -15:
        return map_value(steer_raw, -128, 0, 2000, autonomous.SERVO_CENTER)
    elif steer_raw > 15:
        return map_value(steer_raw, 0, 127, autonomous.SERVO_CENTER, 1000)
    else:
        return autonomous.SERVO_CENTER

def get_throttle(r2_val, l2_val):
    r2 = int((r2_val + 1) * 127.5) # Scale to 0–255
    l2 = int((l2_val + 1.0) * 127.5)
    print(f"r2 = {r2}")
    print(f"l2 = {l2}")

    if r2 > 10 and l2 <= 10:
        return map_value(r2, 0, 255, 1500, 2000)
    elif l2 > 10 and r2 <= 10:
        return map_value(l2, 0, 255, 1500, 1000)
    else:
        return 1500

def wait_for_mcu(sleep, max_time):
    """
    Wait for a response from the MCU with a timeout.
    """
    start_time = time.time()
    while mcu_serial.in_waiting == 0:
        if time.time() - start_time > max_time:
            return False
        time.sleep(sleep)
    return True

def handshake_with_mcu():
    try:
        print("PC: opened port")
        mcu_serial.reset_input_buffer()
        mcu_serial.reset_output_buffer()

        # Send the handshake message
        mcu_serial.write(b'ready')
        print("PC: Sent I am ready to mcu")

        response = ""
        # Wait for response from Arduino
        if not wait_for_mcu(sleep=0.01, max_time=10):
            print(f"PC: No response received from mcu")
            return False

        # Read and decode the response
        response = mcu_serial.read(size=5).decode('ascii')
        if response == "ready":
            print("PC: Received 'I am ready' from MCU. Handshake successful")
            return True
        else:
            print(f"PC: Unexpected response from MCU: {response}")
            return False
    except Exception as e:
        print(f"PC: Error during handshake {e}")
        return False
    finally:
        print("Handshake complete")

def servo_call(microseconds):
    # example: 1500 = 00000101 11011100
    low_byte = microseconds & 0xFF
    high_byte = (microseconds >> 8) & 0xFF
    checksum = (low_byte + high_byte) & 0xFF
    message = struct.pack('BBB', low_byte, high_byte, checksum)
    mcu_serial.write(message)

def esc_call(microseconds):
    low_byte = microseconds & 0xFF
    high_byte = (microseconds >> 8) & 0xFF
    checksum = (low_byte + high_byte) & 0xFF
    message = struct.pack('BBB', low_byte, high_byte, checksum)
    mcu_serial.write(message)

def send_instruction(device, microseconds):
    if device == "servo":
        mcu_serial.write(b's') #calling s for servo
        print("PC: Sent 's' to MCU")

        if not wait_for_mcu(0.01, 10):
            print("PC: No acknowledgment from MCU for 's'")
            return

        # Read the acknowledgment message
        ack_message = mcu_serial.readline().decode('ascii').strip()
        print(f"MCU response: {ack_message}")

        # Send servo microseconds to arduino
        servo_call(microseconds)
        print("PC: Sent servo call to MCU")

        if not wait_for_mcu(0.01, 5):
            print("PC: No acknowledgment from MCU for servo call")
            return
        # Read the acknowledgment message
        servo_message = mcu_serial.readline().decode('ascii').strip()
        print(f"MCU response: {servo_message}")


    elif device == "esc":
        mcu_serial.write(b'e')

        print("PC: Sent 'e' to MCU")

        if not wait_for_mcu(0.01, 10):
            print("PC: No acknowledgment from MCU for 'e'")
            return

        # Read the acknowledgment message
        ack_message = mcu_serial.readline().decode('ascii').strip()
        print(f"MCU response: {ack_message}")

        # Send servo microseconds to arduino
        esc_call(microseconds)
        print("PC: Sent esc call to Arduino")

        if not wait_for_mcu(0.01, 5):
            print("PC: No acknowledgment from MCU for servo call")
            return
        # Read the acknowledgment message
        esc_message = mcu_serial.readline().decode('ascii').strip()
        print(f"MCU response: {esc_message}")

def arm_esc():
    print("ESC Arming Sequence Starting")

    send_instruction("esc", 1500)
    input("Press enter to continue")

    send_instruction("esc", 2000)
    input("Press enter to continue")

    send_instruction("esc", 1000)
    input("Press enter to continue")

    send_instruction("esc", 1500)
    input("Press enter to continue")
    
def percent_to_microseconds(percent):
    percent = max(-100, min(100, percent))
    return int(1500 + (percent / 100) * 500) 

def set_esc(speed_percent):
    microseconds = percent_to_microseconds(speed_percent)
    print(microseconds)
    send_instruction("esc", microseconds)

def set_steering(steering_percent):
    microseconds = percent_to_microseconds(steering_percent)
    print(microseconds)
    send_instruction("servo", microseconds)

def manual_driving():
    lx = joystick.get_axis(0)   # Left stick X axis
    r2 = joystick.get_axis(5)   # R2 (Right trigger)
    l2 = joystick.get_axis(2)   # L2 (Left trigger)

    steer_us = get_steering(lx)
    throttle_us = get_throttle(r2, l2)
        
    send_instruction("servo", steer_us)
    send_instruction("esc", throttle_us)

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()
manual_mode = True
prev_button_state = False

while True:
    autonomous.lidar.set_callback(autonomous.on_full_scan)
    #throttle = autonomous.THROTTLE_STOP
    # direction = autonomous.on_full_scan()
    # if direction == autonomous.SERVO_LEFT:
    #     print("left is more open")
    # elif direction == autonomous.SERVO_RIGHT:
    #     print("right is more open")

    # else:
    #     print("no direction")
    # if autonomous.object_detected():
    #     direction = autonomous.open_direction()
    #     if direction == autonomous.SERVO_LEFT:
    #         print("left is more open")
    #     else:
    #         print("right is more open")
    # else:
    #     throttle = autonomous.THROTTLE_CRUISE
    time.sleep(0.01)

if handshake_with_mcu():
    print("PC: Ready to communicate")
    try:
        input("Press enter to begin while true loop")
        autonomous.start_lidar_thread()
        last_scan_id = None
        throttle = autonomous.THROTTLE_STOP
        #send_instruction("esc", throttle)
        while True:
            pygame.event.pump()

            button_pressed = joystick.get_button(1) # checking if 'circle' is pressed on ps5 controller

            if button_pressed and not prev_button_state:
                    manual_mode = not manual_mode
                    print("Switched to", "MANUAL" if manual_mode else "AUTONOMOUS")
                    time.sleep(0.5)
            
            prev_button_state = button_pressed

            if manual_mode:
                manual_driving()
            
            else:
                scan = autonomous.latest_full_scan
                if not scan:
                    continue

                if id(scan) == last_scan_id:
                    continue

                last_scan_id = id(scan)

                throttle = autonomous.THROTTLE_STOP
                
                if autonomous.object_detected():
                    direction = autonomous.open_direction()
                    if direction == autonomous.SERVO_LEFT:
                        print("left is more open")
                    else:
                        print("right is more open")
                else:
                    throttle = autonomous.THROTTLE_CRUISE
                #send_instruction("servo", autonomous.SERVO_CENTER)
                #send_instruction("esc", throttle)
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nPC: Program interrupted by user. Exiting...")
    finally:
        # Perform any necessary cleanup here
        if mcu_serial.is_open:
            mcu_serial.close()
            print("PC: Serial port closed.")
else:
    print("PC: Handshake failed")



