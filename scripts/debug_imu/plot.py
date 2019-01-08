#!/usr/bin/env python
import math
import serial
import Queue
import threading
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation


# PORT = '/dev/ttyACM1'
# BAUD = 38400

PORT = '/dev/ttyUSB0'
BAUD = 9600

q = Queue.Queue()
q_reading = None



def collect_serial():
    global q_reading
    ser = serial.Serial(PORT, BAUD)
    expected_headers = ['Error', 'Speed']
    with open('minicom.cap', 'r') as x:
        current_reading = {}
        while True:
            line = ser.readline().strip()
            try:
                for header in expected_headers:
                    if line.startswith(header):
                        values = [float(v.strip()) for v in line.split(' ')[1:]]
                        current_reading[header] = values
                        if header == expected_headers[-1]:
                            if set(current_reading.keys()) == set(
                                    expected_headers):
                                q_reading = current_reading.copy()
                                q.put(q_reading)
                                current_reading = {}
                        break
            except Exception:
                current_reading = {}
            time.sleep(0.01 / 3)


def generate_animation(lines, data_key):

    def animate(i, *args):
        x_len = 200
        # Get data
        data = None
        while True:
            if q_reading:
                data = q_reading
            if data:
                break
        # Get lines
        for idx, (arg, line) in enumerate(zip(args, lines)):
            val = data[data_key][idx]
            arg.append(val)
            arg = arg[-x_len:]
            line.set_ydata(arg)
        return lines

    return animate


def add_graph(fig, y_range, plt_location, x_len, line_labels, ylabel, data_key):
    ax = fig.add_subplot(*plt_location)
    ax.set_ylim(y_range)
    xs = list(range(0, x_len))

    line_datas = []
    lines = []
    for label in line_labels:
        line_datas.append([0] * x_len)
        line, = ax.plot(xs, line_datas[-1], label=label)
        lines.append(line)
    # Format
    plt.ylabel(ylabel)
    ax.legend()
    animate = generate_animation(lines, data_key)
    ani = animation.FuncAnimation(
        fig, animate, fargs=line_datas, interval=50, blit=True)
    return ani


def main():
    # Configure serial thread
    thread = threading.Thread(target=collect_serial)
    thread.daemon = True

    fig = plt.figure()
    plt.title('IMU Readings')
    # NOTE: For some reason ani needs to be present to work, maybe something to
    # do with python gc?
    animations = [
        # add_graph(
        #     fig=fig,
        #     y_range=[-1.5, 1.5],
        #     plt_location=[4, 1, 1],
        #     x_len=200,
        #     line_labels=['ACC X', 'ACC Y', 'ACC Z'],
        #     ylabel='Acceleration (g)',
        #     data_key='ACC',
        # ),
        # add_graph(
        #     fig=fig,
        #     y_range=[-180, 180],
        #     plt_location=[6, 1, 2],
        #     x_len=200,
        #     line_labels=['GYR X', 'GYR Y', 'GYR Z'],
        #     ylabel='degrees / s',
        #     data_key='GYR',
        # ),
        # add_graph(
        #     fig=fig,
        #     y_range=[-1000, 3000],
        #     plt_location=[6, 1, 3],
        #     x_len=200,
        #     line_labels=['MAG X', 'MAG Y', 'MAG Z'],
        #     ylabel='?',
        #     data_key='MAG',
        # ),
        # add_graph(
        #     fig=fig,
        #     y_range=[-180, 180],
        #     plt_location=[2, 1, 1],
        #     x_len=200,
        #     line_labels=['YAW', 'PITCH', 'ROLL'],
        #     ylabel='degrees',
        #     data_key='Orientation',
        # ),
        add_graph(
            fig=fig,
            y_range=[-90, 90],
            plt_location=[2, 1, 1],
            x_len=200,
            line_labels=['E'],
            ylabel='Degrees',
            data_key='Error',
        ),
        add_graph(
            fig=fig,
            y_range=[-30000, 30000],
            plt_location=[2, 1, 2],
            x_len=200,
            line_labels=['S'],
            ylabel='ticks',
            data_key='Speed',
        ),
        # add_graph(
        #     fig=fig,
        #     y_range=[-5000, 5000],
        #     plt_location=[4, 1, 4],
        #     x_len=200,
        #     line_labels=['P'],
        #     ylabel='factor',
        #     data_key='PID',
        # ),
    ]
    plt.tight_layout()

    thread.start()
    plt.show()


if __name__ == '__main__':
    main()

