#!/usr/bin/env python

import sys
import numpy as np

mode = 0
range_from = 0
range_to = 0
m_trig = False
f_trig = False
range_trig = False
file_name = ''
path = ''

def parse_txt(path):
    f = open(path, "r")
    data = f.read()
    f.close()

    data = data.split('\n')
    n_data = []
    for s in data:
        vec = s.split(' ')
        if len(vec) == 2:
            vec += [0]
        n_data.append(vec)
    data = n_data
    print(data)
    data = np.array(data, np.float64)

    return data


if __name__ == "__main__":
    for i in sys.argv:
        if i == '-m':
            m_trig = True
        elif i == '-p':
            f_trig = True
        elif i == '-r':
            range_trig = True
        elif m_trig == True:
            mode = int(i)
            m_trig = False
        elif f_trig == True:
            file_name = i
            f_trig = False
        elif range_trig == True:
            rng = i.split(':')
            range_from = int(rng[0])
            range_to = int(rng[1])
            range_trig = False

    if len(file_name) == 0:
        print('The file is not exist')
        exit(-1)

    path = parse_txt(file_name)
    if (range_from == 0 and range_to == 0):
        path[:,2]=mode
    else:
        path[range_from-1:range_to,2]=mode

    print(len(path))
    f = open(file_name.split('.')[0]+'.md.txt','w')
    out_str = ''
    for point in path:
        print(point)
        out_str += str(point[0]) + ' ' + str(point[1]) + ' ' + str(int(point[2])) + '\n'
    f.write(out_str[:-1])
    f.close()
