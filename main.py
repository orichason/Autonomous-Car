import serial
import struct
import time
import pygame
import autonomous

#pi port: '/dev/ttyUSB0'
#pc port: 'COM4'
mcu_serial = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
time.sleep(2)

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
        #print("PC: Sent 's' to MCU")

        if not wait_for_mcu(0.01, 10):
            print("PC: No acknowledgment from MCU for 's'")
            return

        # Read the acknowledgment message
        ack_message = mcu_serial.readline().decode('ascii').strip()
        #print(f"MCU response: {ack_message}")

        # Send servo microseconds to arduino
        #print(microseconds)
        servo_call(microseconds)
        #print("PC: Sent servo call to MCU")

        if not wait_for_mcu(0.01, 5):
            print("PC: No acknowledgment from MCU for servo call")
            return
        # Read the acknowledgment message
        #servo_message = mcu_serial.readline().decode('ascii').strip()
        #print(f"MCU response: {servo_message}")


    elif device == "esc":
        mcu_serial.write(b'e')

        #print("PC: Sent 'e' to MCU")

        if not wait_for_mcu(0.01, 10):
            print("PC: No acknowledgment from MCU for 'e'")
            return

        # Read the acknowledgment message
        ack_message = mcu_serial.readline().decode('ascii').strip()
        #print(f"MCU response: {ack_message}")

        # Send servo microseconds to arduino
        esc_call(microseconds)
        #print("PC: Sent esc call to Arduino")

        if not wait_for_mcu(0.01, 5):
            print("PC: No acknowledgment from MCU for servo call")
            return
        # Read the acknowledgment message
        #esc_message = mcu_serial.readline().decode('ascii').strip()
        #print(f"MCU response: {esc_message}")

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

def stop_car():
    send_instruction("servo", autonomous.SERVO_CENTER)
    send_instruction("esc", autonomous.THROTTLE_STOP)

def map_angle_to_pwm(angle):
    if angle >= 270:
        delta = angle - 360
        return int(map_value(delta, -90, 0, 2000, autonomous.SERVO_CENTER))
    else:
        delta = angle
        return int(map_value(delta, 0, 90, autonomous.SERVO_CENTER, 1000))

def get_autonomous_throttle(angle, distance):
    print(f"Angle = {angle}")
    print(f"Distance = {distance}")
    if distance < 300:
        return autonomous.THROTTLE_STOP
    
    return autonomous.THROTTLE_SLOW
    
    distance_ratio = min(distance / autonomous.MAX_DISTANCE, 1.0)

    if angle >= 270:
        delta = angle - 360
    else:
        delta = angle

    turn_ratio = abs(delta) / 90.0

    throttle = autonomous.THROTTLE_SLOW + (autonomous.MAX_THROTTLE - autonomous.THROTTLE_SLOW) * distance_ratio * (1 - turn_ratio)

    return int(throttle)

def manual_driving():
    lx = joystick.get_axis(0)   # Left stick X axis
    r2 = joystick.get_axis(5)   # R2 (Right trigger)
    l2 = joystick.get_axis(2)   # L2 (Left trigger)

    steer_us = get_steering(lx)
    throttle_us = get_throttle(r2, l2)
    #print(steer_us)      
    send_instruction("servo", steer_us)
    send_instruction("esc", throttle_us)

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()
manual_mode = True
prev_button_state = False

# while True:
#     open_angle = autonomous.get_open_angle()
#     open_distance = autonomous.get_target_distance()
#     throttle = get_autonomous_throttle(open_angle, open_distance)
#     #print(f"Open angle = {open_angle}")
#     print(f"Throttle = {throttle}")

if handshake_with_mcu():
    print("PC: Ready to communicate")
    try:
        # while True:
        #     send_instruction("servo", 1500)
        #     time.sleep(0.6)
        #     send_instruction("servo", 2000)
        #     time.sleep(0.6)
        #     send_instruction("servo", 1000)
        #     time.sleep(0.6)
        input("Press enter to begin while true loop")
        last_scan_id = None
        throttle = autonomous.THROTTLE_STOP
        while True:
            pygame.event.pump()

            button_pressed = joystick.get_button(1) # checking if 'circle' is pressed on ps5 controller

            if button_pressed and not prev_button_state:
                    manual_mode = not manual_mode
                    send_instruction("esc", autonomous.THROTTLE_STOP)
                    send_instruction("servo", autonomous.SERVO_CENTER)
                    print("Switched to", "MANUAL" if manual_mode else "AUTONOMOUS")
                    time.sleep(0.5)
            
            prev_button_state = button_pressed

            if manual_mode:
                manual_driving()
            
            else:
                open_angle = autonomous.get_open_angle()
                open_distance = autonomous.get_target_distance()
                throttle = get_autonomous_throttle(open_angle, open_distance)
                #print(f"Open angle = {open_angle}")
                servo_us = map_angle_to_pwm(open_angle)
                #print(f"Steer = {servo_us}")
                print("-----------------------------")
                #send_instruction("servo", servo_us)
                #send_instruction("esc", throttle)
    except KeyboardInterrupt:
        print("\nPC: Program interrupted by user. Exiting...")
    finally:
        # Perform any necessary cleanup here
        if mcu_serial.is_open:
            mcu_serial.close()
            print("PC: Serial port closed.")
else:
    print("PC: Handshake failed")



