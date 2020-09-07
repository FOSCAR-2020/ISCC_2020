#!/usr/bin/env python

import sys
import numpy as np

mode = 0
m_trig = False
f_trig = False
file_name = ''
path = ''

def parse_txt(path):
    f = open(path, "r")
    data = f.read()
    f.close()

    data = data.rstrip().split('\n')
    data = [x.split(' ')[:2]+[0] for x in data]
    print(data)
    data = np.array(data, np.float64)

    return data


if __name__ == "__main__":
    for i in sys.argv:
        if i == '-m':
            m_trig = True
        elif i == '-p':
            f_trig = True
        elif m_trig == True:
            mode = int(i)
            m_trig = False
        elif f_trig == True:
            file_name = i
            f_trig = False

    if len(file_name) == 0:
        print('The file is not exist')
        exit(-1)

    path = parse_txt(file_name)
    path = path + np.array([0,0,mode])

    print(len(path))
    f = open(file_name.split('.')[0]+'.md.txt','w')
    out_str = ''
    for point in path:
        print(point)
        out_str += str(point[0]) + ' ' + str(point[1]) + ' ' + str(int(point[2])) + '\n'
    f.write(out_str[:-1])
    f.close()
