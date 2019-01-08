#!/usr/bin/env python
import serial

PORT = '/dev/ttyACM2'
BAUD = 38400


current_reading = {}


def get_value(line):
    global current_reading

    if line.startswith('ACC'):
        values = [float(v.strip()) for v in line.split(' ')[1:]]
        current_reading['ACC'] = values
    elif line.startswith('GYR'):
        values = [float(v.strip()) for v in line.split(' ')[1:]]
        current_reading['GYR'] = values
    elif line.startswith('MAG'):
        values = [float(v.strip()) for v in line.split(' ')[1:]]
        current_reading['MAG'] = values
        val = current_reading.copy()
        current_reading = {}
        return val
    return None


def main():
    ser = serial.Serial(PORT, BAUD)
    while True:
        l = ser.readline().strip()
        data = get_value(l)
        if data:
            print ', '.join(
                str(v) for v in (data['ACC'] + data['GYR'] + data['MAG']))


if __name__ == '__main__':
    main()

