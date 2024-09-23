#!/usr/bin/env python

#
# Read and parse the encoder controller data from the serial port.
#

import serial
import sys

ENCODER_PORT = "COM7"

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
        return
    parts = line.split(',')

    # Check correct number of values. There should be timestamp plus 6 pairs, so 13.
    if len(parts) != 13:
        print("Invalid line. There should be 13 numbers. Line: " + line)

    angles = []
    for i in range(0, 6):
        angles.append(float(parts[i*2+1]))
    print(angles)
    

encoder = serial.Serial(
            port=ENCODER_PORT,
            baudrate=115200,
            # parity = serial.PARITY_ODD,
            # stopbits = serial.STOPBITS_TWO,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )

print(F"Serial {ENCODER_PORT} is open: " + str(encoder.isOpen()))
print("Waiting for data. Press Ctrl-C to stop.")


while True:
    try:
        line = read_encoder_line(encoder)
        if line:
            print(line)
            parse_encoder_line(line)
    except KeyboardInterrupt:
        encoder.close()
        sys.exit()
    except Exception as e:
        print(e)
        encoder.close()
        sys.exit()
