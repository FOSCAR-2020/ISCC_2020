#!/usr/bin/env python

import sys
import numpy as np

file_name = ''
path = ''

def parse_txt(path):
    f = open(path, "r")
    data = f.read()
    f.close()

    data = data.rstrip().split('\n')
    data = [x.split(' ') for x in data]
    print(data)
    data = np.array(data, np.float64)

    return data

if __name__ == "__main__":
    file_name = sys.argv[1]

    if len(file_name) == 0:
        print('The file is not exist')
        exit(-1)

    path = parse_txt(file_name)
    print(path)
    path = path[::-1]
    print(path)

    f = open(file_name.split('.')[0] + '.rvs.txt', 'w')
    out_str = ''
    for point in path:
        out_str += str(point[0]) + ' ' + str(point[1]) + '\n'
    f.write(out_str[:-1])
    f.close()
