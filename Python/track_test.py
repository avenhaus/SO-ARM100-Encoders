#!/usr/bin/env python
#
# *********    Track Test     *********
#

import os
from time import sleep
from scservo_sdk import *                    # Uses SCServo SDK library

# Control table address
SCS_MIN_POS_LIMIT     = [ 9, 2, "Min Position Limit"]
SCS_MAX_POS_LIMIT     = [11, 2, "Max Position Limit"]
SCS_POS_CORRECTION    = [31, 2, "Position Correction"]
SCS_WORK_MODE         = [33, 1, "Work Mode"]
SCS_TORQUE_ENABLE     = [40, 1, "Torque Enable"]
SCS_GOAL_ACC          = [41, 2, "Goal Acceleration"]
SCS_GOAL_POSITION     = [42, 2, "Goal Position"]
SCS_GOAL_SPEED        = [46, 2, "Goal Speed"]
SCS_TORQUE_LIMIT      = [48, 2, "Torque Limit"]
SCS_LOCK_FLAG         = [55, 1, "Lock Flag"]
SCS_PRESENT_POSITION  = [56, 2, "Present Position"]
SCS_PRESENT_VELOCITY  = [58, 2, "Present Velocity"]
SCS_PRESENT_LOAD      = [60, 2, "Present Load"]
SCS_PRESENT_CURRENT   = [69, 2, "Present Current"]
SCS_UNDOCUMENTED_MAGIC= [85, 1, "Undocumented MAGIC"]

# Default setting
SCS_ID                      = 1                 # SCServo ID : 1
BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
DEVICENAME                  = 'COM3'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
protocol_end                = 0           # SCServo bit end(STS/SMS=0, SCS=1)


rotations = 0
track_min = 0
track_max = 0
range

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = PacketHandler(protocol_end)

def init():
    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        quit()


def servo_set(ID, reg, value):
    # Write SCServo register
    if reg[1] == 1:
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, ID, reg[0], value)   
    else:
        scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, ID, reg[0], SCS_TOSCS(value,15))
    if scs_comm_result != COMM_SUCCESS:
        print(f"Servo {ID} Set {reg[2]} {value}: {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"Servo {ID} Set {reg[2]} {value} error:{packetHandler.getRxPacketError(scs_error)}")

def servo_get(ID, reg):
    # Read SCServo register
    if reg[1] == 1:
        value, scs_comm_result, scs_error = packetHandler.read1ByteTxRx(portHandler, ID, reg[0])
    else:
        value, scs_comm_result, scs_error = packetHandler.read2ByteTxRx(portHandler, ID, reg[0])
        value = SCS_TOHOST(value, 15)
    if scs_comm_result != COMM_SUCCESS:
        print(f"Servo {ID} Get {reg[2]}: {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"Servo {ID} Get {reg[2]} error: {packetHandler.getRxPacketError(scs_error)}")
    return value

def check_servo_config(ID):
    # This speeds up servo movements
    v = servo_get(ID, SCS_UNDOCUMENTED_MAGIC)
    print(f"{ID}: Magic Config = {v}")
    if v!=250:
        print(f"{ID}: Enabling Magic 250. Default is 50")
        servo_set(ID, SCS_LOCK_FLAG, 0)
        servo_set(ID, SCS_UNDOCUMENTED_MAGIC, 250)
        servo_set(ID, SCS_LOCK_FLAG, 1)


def get_current_position(ID):
    # Read SCServo present position
    scs_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, ID, SCS_PRESENT_POSITION[0])
    if scs_comm_result != COMM_SUCCESS:
        print(f"Servo {ID} Get Current: {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"Servo {ID} Get Current error: {packetHandler.getRxPacketError(scs_error)}")

    scs_present_position = SCS_LOWORD(scs_present_position_speed)
    scs_present_speed = SCS_HIWORD(scs_present_position_speed)
    return scs_present_position, scs_present_speed        

def find_end(ID, speed):
    global rotations
    print(f"Current Position: {servo_get(ID, SCS_PRESENT_POSITION)}")
    servo_set(ID, SCS_GOAL_SPEED, speed)
    print(f"Current Position: {servo_get(ID, SCS_PRESENT_POSITION)}")
    print(f"Goal Speed: {servo_get(ID, SCS_GOAL_SPEED)}")   
    servo_set(SCS_ID, SCS_TORQUE_ENABLE, 1) # Enable servo
    print(f"Current Position: {servo_get(ID, SCS_PRESENT_POSITION)}")

    current = 0
    old_pos = servo_get(ID, SCS_PRESENT_POSITION)
    pos, speed = get_current_position(ID)
    if old_pos < 1000 and pos > 3000: # Underflow
        rotations -= 1
    if old_pos > 3000 and pos < 1000: # Overflow
        rotations += 1
    track_pos = pos + 4096 * rotations
    print(f"ID:{ID} servo pos:{pos} rotations:{rotations} track pos:{track_pos} speed:{SCS_TOHOST(speed, 15)} old pos:{old_pos}")

    while abs(current) < 15:
        pos, speed = get_current_position(ID)
        if old_pos < 1000 and pos > 3000: # Underflow
            rotations -= 1
        if old_pos > 3000 and pos < 1000: # Overflow
            rotations += 1
        old_pos = pos
        load = servo_get(ID, SCS_PRESENT_LOAD)
        current = servo_get(ID, SCS_PRESENT_CURRENT)
        track_pos = pos + 4096 * rotations
        # print(f"ID:{ID} servo pos:{pos} rotations:{rotations} track pos:{track_pos} speed:{SCS_TOHOST(speed, 15)} load:{load}, current:{current}")
        if rotations < -7:
            print("Turns exceeded, Could not find home position")
            quit()
    servo_set(ID, SCS_GOAL_SPEED, 0)
    servo_set(SCS_ID, SCS_TORQUE_ENABLE, 0) # Disable servo
    sleep(0.1) # Wait for servo to stop

    pos, speed = get_current_position(ID)
    if old_pos < 1000 and pos > 3000: # Underflow
        rotations -= 1
    if old_pos > 3000 and pos < 1000: # Overflow
        rotations += 1
    track_pos = pos + 4096 * rotations
    print(f"ID:{ID} servo pos:{pos} rotations:{rotations} track pos:{track_pos} speed:{SCS_TOHOST(speed, 15)})")

    return track_pos


def move_to(ID, goal_pos):
    global rotations
    goal_pos = int(goal_pos)
    print(f"Goal Position: {goal_pos}")  

    servo_set(SCS_ID, SCS_GOAL_POSITION, goal_pos)
    servo_set(ID, SCS_WORK_MODE, 0) # Position Control Mode !! This can change the current servo position!!!
    servo_set(SCS_ID, SCS_TORQUE_ENABLE, 1) # Enable servo

    old_pos = servo_get(ID, SCS_PRESENT_POSITION)
    max_currewnt = 0
    while 1:
        pos, speed = get_current_position(ID)

        if old_pos < 1000 and pos > 3000: # Underflow
            rotations -= 1
        if old_pos > 3000 and pos < 1000: # Overflow
            rotations += 1
        old_pos = pos
        load = servo_get(ID, SCS_PRESENT_LOAD)
        current = servo_get(ID, SCS_PRESENT_CURRENT)
        track_pos = pos + 4096 * rotations

        #print(f"ID:{ID} servo pos:{pos} rotations:{rotations} track pos:{track_pos} goal pos:{goal_pos} speed:{SCS_TOHOST(speed, 15)}")
        if not (abs(goal_pos - track_pos) > 20):
           break
        current = servo_get(ID, SCS_PRESENT_CURRENT)
        if current > max_currewnt:
            max_currewnt = current
        if current > 80:
            servo_set(SCS_ID, SCS_TORQUE_ENABLE, 0) # Disable servo
            print(f"Current exceeded: {current} - Collisison detected!")
            quit()
    print(f"ID:{ID} servo pos:{pos} rotations:{rotations} track pos:{track_pos} goal pos:{goal_pos} speed:{SCS_TOHOST(speed, 15)} max current:{max_currewnt}")




def home_servo(ID, speed=400):
    global rotations,track_min, track_max, range
    print(f"Min: {servo_get(ID, SCS_MIN_POS_LIMIT)} Max: {servo_get(ID, SCS_MAX_POS_LIMIT)}")   

    print(f"Position before adjusting range: {servo_get(ID, SCS_PRESENT_POSITION)}")
    servo_set(ID, SCS_MIN_POS_LIMIT, 0)
    print(f"Position before after setting max: {servo_get(ID, SCS_PRESENT_POSITION)}")
    servo_set(ID, SCS_MAX_POS_LIMIT, 0)
    print(f"Min: {servo_get(ID, SCS_MIN_POS_LIMIT)} Max: {servo_get(ID, SCS_MAX_POS_LIMIT)}")   

    servo_set(ID, SCS_WORK_MODE, 1) # Speed Control Mode !! This can change the current servo position!!!
    servo_set(ID, SCS_GOAL_SPEED, -abs(speed))
    print(f"Goal Speed: {servo_get(ID, SCS_GOAL_SPEED)}")   
    servo_set(SCS_ID, SCS_TORQUE_ENABLE, 1) # Enable servo

    track_max = find_end(ID, abs(speed))
    print(f"Servo {ID} max at {track_max}")
    track_min = find_end(ID, -abs(speed))    
    range = track_max - track_min
    print(f"Servo {ID}: min at {track_min} max at {track_max} range {range} turns={round((range)/4096.0,2)}")

    success = False
    while not success:
        try:
            servo_set(SCS_ID, SCS_TORQUE_ENABLE, 128) # Reset Servo Position to 2048 to sync rotation counter
            success = True
        except Exception as e:
            print(f"Servo {ID} Reset Position: {e}")
            sleep(0.5)
    

    # Correct servo position offset to 0
    correction = servo_get(ID, SCS_POS_CORRECTION)
    correction = SCS_TOHOST(correction,11)
    print(f"Servo {ID}: Original Correction {correction} to 2048")
    correction = correction - 2048
    if correction < -2048:
        correction += 4096
    print(f"Servo {ID}: New Correction {correction} to 0")
    servo_set(ID, SCS_POS_CORRECTION, SCS_TOSCS(correction,11))

    # Set final servo range
    rotations = 0
    track_min = 0
    track_max = track_min + range
    servo_set(ID, SCS_MIN_POS_LIMIT, track_min)
    servo_set(ID, SCS_MAX_POS_LIMIT, track_max)

    print(f"Servo {ID}: adjusted min at {track_min} max at {track_max} range {range} turns={round((range)/4096.0,2)}")
    servo_set(ID, SCS_WORK_MODE, 0) # Position Control Mode 


init()
check_servo_config(SCS_ID)
home_servo(SCS_ID,800)

move_to(SCS_ID, range*0.9)
move_to(SCS_ID, range*0.1)
move_to(SCS_ID, range*0.9)
move_to(SCS_ID, range*0.1)

move_to(SCS_ID, range/2)

servo_set(SCS_ID, SCS_TORQUE_ENABLE, 0) # Disable servo
# Close port
portHandler.closePort()