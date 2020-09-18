#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt
import math
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
    distances2=[]

    for i in range(0,len(path)-1):
        x = path[i+1][0]-path[i][0]
        y = path[i+1][1]-path[i][1]
        distances2.append(math.sqrt(x*x) +(y*y))


    # distances = [np.linalg.norm(path[i+1]-path[i]) for i in range(0,len(path)-1)]
    for i in range(len(distances2)):
        if distances2[i] >= 0.5:
            print("{} :".format(i),distances2[i])

    plt.plot(np.linspace(0,len(path)-2,len(path)-1),distances2,marker='o')
    plt.show()
