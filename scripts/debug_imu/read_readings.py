#!/usr/bin/env python
import sys


def main():
    BASE_GROUP = {'ACC': [], 'GYR': [], 'MAG': [],}
    group = BASE_GROUP.copy()
    for line in sys.stdin.readlines():
        if line.startswith('ACC'):
            values = [float(v.strip()) for v in line.split(' ')[1:]]
            group['ACC'] = values
        elif line.startswith('GYR'):
            values = [float(v.strip()) for v in line.split(' ')[1:]]
            group['GYR'] = values
        elif line.startswith('MAG'):
            values = [float(v.strip()) for v in line.split(' ')[1:]]
            group['MAG'] = values
            print ', '.join(
                str(v) for v in (group['ACC'] + group['GYR'] + group['MAG']))
            group = BASE_GROUP.copy()


if __name__ == '__main__':
    main()

