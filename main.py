import serial
import struct
import time

#pi port: '/dev/ttyUSB0'
#pc port: 'COM4'
arduino_serial = serial.Serial(port='COM3', baudrate=115200, timeout=1)
time.sleep(2)

def wait_for_arduino(sleep, max_time):
    """
    Wait for a response from the Arduino with a timeout.
    """
    start_time = time.time()
    while arduino_serial.in_waiting == 0:
        if time.time() - start_time > max_time:
            return False
        time.sleep(sleep)
    return True

def handshake_with_arduino():
    try:
        print("PC: opened port")
        arduino_serial.reset_input_buffer()
        arduino_serial.reset_output_buffer()

        # Send the handshake message
        arduino_serial.write(b'ready')
        print("PC: Sent I am ready to arduino")

        response = ""
        # Wait for response from Arduino
        if not wait_for_arduino(sleep=0.01, max_time=10):
            print(f"PC: No response received from arduino")
            return False

        # Read and decode the response
        response = arduino_serial.read(size=5).decode('ascii')
        if response == "ready":
            print("PC: Received 'I am ready' from Arduino. Handshake successful")
            return True
        else:
            print(f"PC: Unexpected response from Arduino: {response}")
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
    arduino_serial.write(message)

def esc_call(microseconds):
    low_byte = microseconds & 0xFF
    high_byte = (microseconds >> 8) & 0xFF
    checksum = (low_byte + high_byte) & 0xFF
    message = struct.pack('BBB', low_byte, high_byte, checksum)
    arduino_serial.write(message)

def send_instruction(device, microseconds):
    if device == "servo":
        arduino_serial.write(b's') #calling s for servo
        print("PC: Sent 's' to Arduino")

        if not wait_for_arduino(0.01, 10):
            print("PC: No acknowledgment from Arduino for 's'")
            return

        # Read the acknowledgment message
        ack_message = arduino_serial.readline().decode('ascii').strip()
        print(f"Arduino response: {ack_message}")

        # Send servo microseconds to arduino
        servo_call(microseconds)
        print("PC: Sent servo call to Arduino")

        if not wait_for_arduino(0.01, 5):
            print("PC: No acknowledgment from Arduino for servo call")
            return
        # Read the acknowledgment message
        servo_message = arduino_serial.readline().decode('ascii').strip()
        print(f"Arduino response: {servo_message}")


    elif device == "esc":
        arduino_serial.write(b'e')

        print("PC: Sent 'e' to Arduino")

        if not wait_for_arduino(0.01, 10):
            print("PC: No acknowledgment from Arduino for 'e'")
            return

        # Read the acknowledgment message
        ack_message = arduino_serial.readline().decode('ascii').strip()
        print(f"Arduino response: {ack_message}")

        # Send servo microseconds to arduino
        esc_call(microseconds)
        print("PC: Sent esc call to Arduino")

        if not wait_for_arduino(0.01, 5):
            print("PC: No acknowledgment from Arduino for servo call")
            return
        # Read the acknowledgment message
        esc_message = arduino_serial.readline().decode('ascii').strip()
        print(f"Arduino response: {esc_message}")

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


if handshake_with_arduino():
    print("PC: Ready to communicate")
    #arm_esc()
    try:
        input("Press enter to begin while true loop")
        # send_instruction("esc", 1000)
        # time.sleep(0.5)
        while True:
            set_steering(20) # Should stop the motor
            time.sleep(1)
            set_steering(0)
            print("in neutral")# Should start the motor at neutral throttle
            time.sleep(1)
            set_steering(-20)
    
            time.sleep(1)
            set_steering(0)
            time.sleep(1)

            # set_esc(20) # Should stop the motor
            # time.sleep(1)
            # set_esc(0)
            # print("in neutral")# Should start the motor at neutral throttle
            # time.sleep(1)
            # set_esc(-20)
            # time.sleep(1)
            # set_esc(0)
            # time.sleep(1)
            input("Press enter continue")
            # send_instruction("esc", 2000)
            # time.sleep(1)# Should move at full speed

        # while True:
        #     send_instruction("servo", 1000)
        #     time.sleep(2)
        #     send_instruction("servo", 1500)
        #     time.sleep(2)
        #     send_instruction("servo", 2000)
        #     time.sleep(2)
    except KeyboardInterrupt:
        #send_instruction("esc", 1000);
        print("\nPC: Program interrupted by user. Exiting...")
    finally:
        # Perform any necessary cleanup here
        if arduino_serial.is_open:
            arduino_serial.close()
            print("PC: Serial port closed.")
else:
    print("PC: Handshake failed")


