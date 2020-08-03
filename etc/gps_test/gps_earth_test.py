import serial

gps = serial.Serial("/dev/ttyUSB1", baudrate=38400)

lat_lons = []

while True:
	line = gps.readline()
	data = line.split(",")

	if data[0] == "$GPRMC":
		if data[2] == "A":

			latgps = float(data[3])
			if data[4] == "S":
				latgps = -latgps

			latdeg = int(latgps/100)
			latmin = latgps - latdeg*100
			lat = latdeg+(latmin/60)

			longps = float(data[5])
			if data[6] == "W":
				longps = -longps

			londeg = int(longps/100)
			lonmin = longps - londeg*100
			lon = londeg+(lonmin/60)

			lat_lons.append([lat,lon])

			print "lat:%s" % lat_lons[-1][0]
			print "lon:%s" % lat_lons[-1][1]


			with open("position.kml", "w") as pos:
				pos.write("""<kml xmlns="http://www.opengis.net/kml/2.2"
xmlns:gx="http://www.google.com/kml/ext/2.2"><Placemark>
  <name>live_gps_from_python</name>
  <description>asdf</description>
  <LineStyle>
   <color>7f00ffff</color>
   <width>4</width>
  </LineStyle>
  <LineString>
  <extrude>1</extrude>
  <tessellate>1</tessellate>
  <altitudeMode>absolute</altitudeMode>
  <coordinates>""")
				for lat_lon in lat_lons:
					pos.write("%s,%s,0\n" % (lat_lon[1], lat_lon[0]))
				pos.write("""
  </coordinates>
  </LineString>
</Placemark></kml>""")

