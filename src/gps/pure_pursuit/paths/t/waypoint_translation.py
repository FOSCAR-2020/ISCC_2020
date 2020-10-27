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

    data = data.rstrip().split('\n')
    data = [x.split(' ') for x in data]
    #print(data)
    data = np.array(data, np.float64)

    return data


if __name__ == "__main__":
    file_name = sys.argv[1]

    if len(file_name) == 0:
        print('The file is not exist')
        exit(-1)

    path = parse_txt(file_name)

    k_city_start_position = path[0]
    mode = k_city_start_position[2]

    # 955568.875223, 1956919.86782 <- center
    # 955576.342688 1956929.82429 <- side
    school_start_position = np.array([955568.875223, 1956919.86782, mode],np.float64)
    print("k_city_start_position : {}".format(k_city_start_position))
    print("school_start_position : {}".format(school_start_position))

    # offset = np.array([19923.241491,  40945.30306, 0])
    offset = school_start_position - k_city_start_position

    print("offset : {}".format(offset))
    
    #### for rotation ####
    theta = 260 # degree
    theta = theta * np.pi / 180
    rotation_matrix = np.array([[np.cos(theta), np.sin(theta), 0],[-np.sin(theta), np.cos(theta), 0], [0, 0, 1]], np.float64)
    
    rotation_path = path - k_city_start_position
    #rotation_path = path - offset

    rotation_path = np.dot(rotation_path, rotation_matrix)
    
    rotation_path += k_city_start_position
    #rotation_path += offset
    ######################
    
    
    #### for parallel translation ####
    new_path = rotation_path + offset    
    
    ##################################
    
    f = open(file_name.split('.')[0]+'.trs.txt','w')
    out_str = ''
    for point in new_path:
        out_str += str(point[0]) + ' ' + str(point[1]) + ' ' + str(int(point[2])) + '\n'
    f.write(out_str)
    f.close()
