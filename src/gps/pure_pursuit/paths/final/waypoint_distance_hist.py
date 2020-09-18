#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt

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
    #print(path)
    distances = [np.linalg.norm(path[i+1]-path[i]) for i in range(0,len(path)-1)]
    x = [i for i in range(0,len(path) - 1)]

    for idx, dist in enumerate(distances):
        if dist > 1:
            print(idx)
        

    #print(distances)
    #plt.bar(np.linspace(0,len(path)-2,len(path)-1),distances,width=1)
    #plt.show()
    plt.plot(x,distances, 'or')
    plt.show()

