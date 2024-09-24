#!/usr/bin/env python
#
# *********   Follow Leader Arm   *********
#
# Reads the angles from the leader arm magnetic encoder controller and sets the follower arm servos to match.
#
# Available SCServo model on this example : All models using Protocol SCS
# Be sure that SCServo(STS/SMS/SCS) properties are already set as %% ID : 1 / Baudnum : 6 (Baudrate : 1000000)
#

import os

from scservo_sdk import *                    # Uses SCServo SDK library
import serial

# Control table address
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_SCS_GOAL_ACC          = 41
ADDR_SCS_GOAL_POSITION     = 42
ADDR_SCS_GOAL_SPEED        = 46
ADDR_SCS_PRESENT_POSITION  = 56

# Default setting
SCS_IDS                     = [1, 2, 3, 4, 5, 6] # SCServo IDs
BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
DEVICENAME                  = 'COM3'            # Servo Port: Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

ENCODER_PORT                = "COM4"            # Port of the magnetic encoder controller

REVERSE = [1, 1, -1, -1, -1, -1]          # Reverse direction of the servos

SCS_MAXIMUM_POSITION_VALUE  = 4095        # Servo maximum position value (STS3215: 4095, SCS225: 1023)
SCS_MOVING_STATUS_THRESHOLD = 20          # SCServo moving status threshold
SCS_MOVING_SPEED            = 0           # SCServo moving speed
SCS_MOVING_ACC              = 0           # SCServo moving acc
protocol_end                = 0           # SCServo byte order end(STS/SMS=0, SCS=1) 

servo_home = [878, 2071, 2046, 918, 1943, 1990]

def read_encoder_line(port):
    if port.in_waiting == 0:
        return None
    line = ""
    while True:
        chunk = port.readline()
        if chunk:
            line += chunk.decode('utf-8')
            if line.endswith('\n'):
                return line.strip('\n\r\t ')

def parse_encoder_line(line):
    # Ignore lines that do not start with a number.
    if not line or ((line[0] < '0' or line[0] > '9') and line[0] != '-'):
        return None
    parts = line.split(',')

    # Check correct number of values. There should be timestamp plus 6 pairs, so 13.
    if len(parts) != 13:
        print("Invalid line. There should be 13 numbers. Line: " + line)

    angles = []
    for i in range(0, 6):
        angles.append(float(parts[i*2+1]))
    return angles

def set_goal_position(scs_id, pos):
    # Write SCServo goal position
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, scs_id, ADDR_SCS_GOAL_POSITION, pos)
    if scs_comm_result != COMM_SUCCESS:
        print(f"Servo {scs_id} Set Goal {pos}: {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"Servo {scs_id} Set Goal {pos} error:{packetHandler.getRxPacketError(scs_error)}")


def get_current_position(scs_id):
    # Read SCServo present position
    scs_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, scs_id, ADDR_SCS_PRESENT_POSITION)
    if scs_comm_result != COMM_SUCCESS:
        print(f"Servo {scs_id} Get Current: {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"Servo {scs_id} Get Current error: {packetHandler.getRxPacketError(scs_error)}")

    scs_present_position = SCS_LOWORD(scs_present_position_speed)
    scs_present_speed = SCS_HIWORD(scs_present_position_speed)
    return scs_present_position, scs_present_speed        


def set_servo_home():
    global servo_home
    servo_home = []
    for srv in range(len(SCS_IDS)):
        scs_id = SCS_IDS[srv]
        pos, speed = get_current_position(scs_id)
        servo_home.append(pos)
    print(f"Servo Home: {servo_home}")


def show_servo_position():
    servo_pos = []
    for srv in range(len(SCS_IDS)):
        scs_id = SCS_IDS[srv]
        pos, speed = get_current_position(scs_id)
        servo_pos.append(pos)
    print(f"Current arm position: {servo_pos}")


# Open serial port of the magnetic encoder controller
encoder = serial.Serial(
            port=ENCODER_PORT,
            baudrate=115200,
            # parity = serial.PARITY_ODD,
            # stopbits = serial.STOPBITS_TWO,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )
if not encoder.isOpen():
    print("Failed to open encoder serial port " + ENCODER_PORT)
    quit()

print(F"Encoder serial port {ENCODER_PORT} is open: " + str(encoder.isOpen()))


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = PacketHandler(protocol_end)
    
# Open Servo port
if portHandler.openPort():
    print("Succeeded to open the servo port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the servo baudrate")
else:
    print("Failed to change the baudrate")
    quit()


for srv in range(len(SCS_IDS)):
    scs_id = SCS_IDS[srv]
    # Write SCServo acc
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, scs_id, ADDR_SCS_GOAL_ACC, SCS_MOVING_ACC)
    if scs_comm_result != COMM_SUCCESS:
        print(f"Servo {scs_id} Set Acc: {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"Servo {scs_id} Set Acc error: {packetHandler.getRxPacketError(scs_error)}")

    # Write SCServo speed
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, scs_id, ADDR_SCS_GOAL_SPEED, SCS_MOVING_SPEED)
    if scs_comm_result != COMM_SUCCESS:
        print(f"Servo {scs_id} Set Speed: {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"Servo {scs_id} Set Speed error: {packetHandler.getRxPacketError(scs_error)}")

print(f"Make sure Servo Home is set correctly: {servo_home}")
#set_servo_home()
show_servo_position()
print("Ready for the follower arm to follow the leader arm.")

while True:
    try:
        line = read_encoder_line(encoder)
        if line:
            print(line)
            angles = parse_encoder_line(line)
            if angles:
                for srv in range(len(SCS_IDS)):
                    scs_id = SCS_IDS[srv]
                    pos = int(servo_home[srv] + (angles[srv] / 360.0) * SCS_MAXIMUM_POSITION_VALUE * REVERSE[srv]) % SCS_MAXIMUM_POSITION_VALUE
                    print (f"Servo {scs_id} Angle: {angles[srv]} Home:{servo_home[srv]} Position: {pos}")
                    set_goal_position(scs_id, pos)

    except KeyboardInterrupt:
        break
    except Exception as e:
        print(e)
        break


for srv in range(len(SCS_IDS)):
    scs_id = SCS_IDS[srv]
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, scs_id, ADDR_SCS_TORQUE_ENABLE, 0)
    if scs_comm_result != COMM_SUCCESS:
        print(f"Servo {scs_id} Set Torque: {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"Servo {scs_id} Set Torque error: {packetHandler.getRxPacketError(scs_error)}")

# Close port
portHandler.closePort()
encoder.close()