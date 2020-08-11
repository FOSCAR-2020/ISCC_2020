import sys
import numpy as np

#from sklearn.linear_model import RANSACRegressor
from scipy.interpolate import interp1d
from scipy.interpolate import CubicSpline

import matplotlib.pyplot as plt

mod = '-l'
g_trig = False
d_trig = False
f_trig = False
o_trig = False
file_name = ''
path = ''

def parse_txt(path):
    f = open(path, "r")
    data = f.read()
    f.close()

    data = data.split('\n')
    data = [x.split(' ') for x in data]
    data = np.array(data, np.float64)

    if d_trig:
        print(data)

    return data

def graph_path(x,y,xr,yr):
    plt.figure(figsize=(12,8))
    '''
    plt.subplot(1,3,1)
    plt.plot(x,y,'r')
    plt.subplot(1,3,2)
    plt.plot(xr,yr,'b')
    plt.subplot(1,3,3)
    '''
    plt.scatter(xr, yr, s=0.1, c='blue')
    plt.scatter(x,y,s=1,c='red')

    plt.show()



if __name__ == "__main__":
    for i in sys.argv:
        if i == '-l' or i == '-c' or i == '-cs':
           mod = i
        elif i == '-g':
            g_trig = True
        elif i == '-d':
            d_trig = True
        elif i == '-p':
            f_trig = True
        elif i == '-o':
            o_trig = True
        elif f_trig == True:
            file_name = i
            f_trig = False

    if len(file_name) == 0:
        print('The file is not exist')
        exit(-1)

    path = parse_txt(file_name)

    x, y = path[:, 0], path[:, 1]
    t = np.arange(0,len(path),1)
    tr = np.linspace(0,len(path)-1,500)
    print(t)
    print(tr)
    xr = np.linspace(np.min(x), np.max(x), 500)
    if mod == '-l':
        model = interp1d(t,x,'linear')
        xr = model(tr)
        model = interp1d(t, y, 'linear')
        yr = model(tr)
    elif mod == '-cs':
        model = CubicSpline(t, x)
        xr = model(tr)
        model = CubicSpline(t, y)
        yr = model(tr)
    elif mod == '-c':
        model = interp1d(t, x, 'cubic')
        xr = model(tr)
        model = interp1d(t, y,'cubic')
        yr = model(tr)


    path = np.transpose((xr,yr))
    if d_trig == True:
        print(path)

    print(len(path))
    if o_trig == True:
        print('out trigger')
        f = open(file_name.split('.')[0]+'.itp.txt','w')
        out_str = ''
        for point in path:
            print(point)
            out_str += str(point[0]) + ' ' + str(point[1]) + '\n'
        f.write(out_str[:-1])
        f.close()

    if g_trig == True:
        graph_path(x,y,xr,yr)

