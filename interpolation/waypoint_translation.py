#!/usr/bin/env python

import sys
import numpy as np

a = 0
b = 0
a_trig = False
b_trig = False
f_trig = False
file_name = ''
path = ''

def parse_txt(path):
    f = open(path, "r")
    data = f.read()
    f.close()

    data = data.split('\n')
    data = [x.split(' ') for x in data]
    print(data)
    data = np.array(data, np.float64)

    return data


if __name__ == "__main__":
    for i in sys.argv:
        if i == '-a':
            a_trig = True
        elif i == '-b':
            b_trig = True
        elif i == '-p':
            f_trig = True
        elif a_trig == True:
            a = float(i)
            a_trig = False
        elif b_trig == True:
            b = float(i)
            b_trig = False
        elif f_trig == True:
            file_name = i
            f_trig = False

    if len(file_name) == 0:
        print('The file is not exist')
        exit(-1)

    path = parse_txt(file_name)

    t = np.array([a,b])
    path = path - t

    print(len(path))
    f = open(file_name.split('.')[0]+'.trs.txt','w')
    out_str = ''
    for point in path:
        print(point)
        out_str += str(point[0]) + ' ' + str(point[1]) + '\n'
    f.write(out_str[:-1])
    f.close()
