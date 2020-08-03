import serial

gps = serial.Serial("/dev/ttyACM0", baudrate=38400)

while True:
	line = gps.readline()
	data = line.split(",")

	if data[0] == "$GNVTG":
		print('degree : %s' % (data[1]))