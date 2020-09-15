
f = open('/home/foscar/ISCC_2020/src/gps/pure_pursuit/paths/final/test.txt','r')

x=[]
y=[]
mode = []
while True:
    line = f.readline()
    if not line: break
    x.append(float(line.split()[0]))
    y.append(float(line.split()[1]))
    mode.append(int(line.split()[2])+1)
    
   
for i in range(len(x)):
    print(x[i]),
    print(y[i]),
    print(mode[i])
