# Integration of code for new sensors, made by SB on Decmeber 21,2018
# GPS Prediction Removed
# New waypoints made
#Garmin GPS and UM7 IMU

#Import Libraries
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import serial
import math
import Adafruit_BBIO.UART as UART
import time
from time import sleep

#Initialization of UART
UART.setup("UART1")  #Initialize UART1
UART.setup("UART4")  #Initialize UART4
ser=serial.Serial('/dev/ttyO1',19200) #Initialize Serial Port at 19200 for Garmin
ser1=serial.Serial('/dev/ttyO4',115200) #Initialize Serial Port at 115200 for UM7
print 'UART initialized'
#Assign DI, DO and PWM
start_button="P8_8"
okled_pin="P8_10" #red
runled_pin="P8_12" #yellow
esc_pin = "P9_21"
ser_pin = "P8_13"
GPIO.setup(okled_pin, GPIO.OUT)
GPIO.setup(runled_pin, GPIO.OUT)
GPIO.setup(start_button, GPIO.IN)
GPIO.output(okled_pin, GPIO.LOW)
GPIO.output(runled_pin, GPIO.LOW)

print 'GPIO initialized'
#Reset PWM to default conditions
dc_fbeep = 13.93
dc_stop=11
ser_dc = 26.2 
esc_f=90.9
ser_f=181.2
PWM.start(esc_pin, dc_fbeep, 90.9) #starting frequency and duty cycle for esc_pin
time.sleep(3)
PWM.start(ser_pin, ser_dc, 181.2) #starting frequency and duty cycle for ser_pin
time.sleep(0.1)
PWM.set_duty_cycle(esc_pin,float(dc_fbeep))
PWM.set_duty_cycle(ser_pin,float(ser_dc))
PWM.stop(esc_pin)
PWM.stop(ser_pin)
print 'PWM Reset'
#Initialize ESC and servo
PWM.start(esc_pin, dc_fbeep, 90.9)
time.sleep(3)
PWM.start(ser_pin, ser_dc, 181.2) #starting duty cycle for ser_pin
throttle=14.25 # Initial DC for starting speed
ser_dc=26.2 # Initial steering position

#Open file for write
f=open("demo.txt","a")
f.write("SampleTime Waypoint_No CurrLat CurrLong  TargetLat TargetLong Current_Heading Target_Heading Heading_Error D2T Yaw_Rate Ax Ay Az Magx Magy Magz Ser_dc Total_Gain\n")

# Way point/map parameters
# Way point/map parameters
WAYPOINT_DIST_TOLERANCE = 5
HEADING_TOLERANCE = 10

TarLat = [47.169502,47.169640,47.169795,47.169917,47.169934]
TarLong = [-88.507541,-88.507583,-88.507640,-88.507768,-88.508037]

x0 = 47.169502    # Vehicle start point  
y0 = -88.507711  #just near to the back door of the APSRC
n=4 #number of waypoints, zero position being the first waypoint

i=0
t1=0
t2=0

#Empty the serial buffers for serial input
ser.flushInput()
ser1.flushInput()
print 'Serial Buffers flushed'

#General Parameters
gpscount=3
count=1
d0=0    #starting point distance
d_cal=0
delta=0
starttime=0
looptime=0
lat = [0,0]
long = [0,0]
z=1
cc=1
dist=0
speed_mps=0.05
integral=0

#Function for GPS
class GPS:
	def read(self):
		ser.flushInput()
		ser.flushInput()
		while ser.inWaiting()==0:
			pass
		self.NMEA1=ser.readline()
		while ser.inWaiting()==0:
			pass
		self.NMEA2=ser.readline()
		NMEA1_array=self.NMEA1.split(',')
		NMEA2_array=self.NMEA2.split(',')
		#print 'NMEA1:',NMEA1_array
		#print 'NMEA2:',NMEA2_array

		if NMEA1_array[0]=='$GPGGA':
			self.latDeg=NMEA1_array[2][:-8]
			self.latMin=NMEA1_array[2][-8:]
			self.latHem=NMEA1_array[3]
			self.lonDeg=NMEA1_array[4][:-8]
			self.lonMin=NMEA1_array[4][-8:]
			self.lonHem=NMEA1_array[5]
			if NMEA1_array[7]==' ' or NMEA1_array[7]==0:
				self.sat=0
			else:
				self.sat=NMEA1_array[7]
		
		if NMEA2_array[0]=='$GPRMC':		
			self.latDeg=NMEA2_array[3][:-8]
			self.latMin=NMEA2_array[3][-8:]
			self.latHem=NMEA2_array[4]
			self.lonDeg=NMEA2_array[5][:-8]
			self.lonMin=NMEA2_array[5][-8:]
			self.lonHem=NMEA2_array[6]
			if NMEA2_array[7]==' ' or NMEA2_array[7]==0:
				self.speed=0
			else:
				self.speed=NMEA2_array[7]
			
		if NMEA2_array[0]=='$GPGGA':
			self.latDeg=NMEA2_array[2][:-8]
			self.latMin=NMEA2_array[2][-8:]
			self.latHem=NMEA2_array[3]
			self.lonDeg=NMEA2_array[4][:-8]
			self.lonMin=NMEA2_array[4][-8:]
			self.lonHem=NMEA2_array[5]
			if NMEA2_array[7]==' ' or NMEA2_array[7]==0:
				self.sat=0
			else:
				self.sat=NMEA2_array[7]
		
		if NMEA1_array[0]=='$GPRMC':		
			self.latDeg=NMEA1_array[3][:-8]
			self.latMin=NMEA1_array[3][-8:]
			self.latHem=NMEA1_array[4]
			self.lonDeg=NMEA1_array[5][:-8]
			self.lonMin=NMEA1_array[5][-8:]
			self.lonHem=NMEA1_array[6]
			if NMEA1_array[7]==' ' or NMEA1_array[7]==0:
				self.speed=0
			else:
				self.speed=NMEA1_array[7]

#Function for IMU
class UM7():
	def read(self):
		ser1.flushInput()
		ser1.flushInput()
		time.sleep(0.1)
		while ser1.inWaiting()==0:
				pass
		self.NMEA3=ser1.readline()      #Read NMEA1 
		NMEA3_array=self.NMEA3.split(',')
		# ser1.flushInput()
		# ser1.flushOutput()
		# time.sleep(0.01)
		
		while ser1.inWaiting()==0:
				pass
		self.NMEA4=ser1.readline()      #Read NMEA2
		NMEA4_array=self.NMEA4.split(',')
		# ser1.flushInput()
		# ser1.flushOutput()
		# time.sleep(0.01)	
		
		while ser1.inWaiting()==0:
				pass
		self.NMEA5=ser1.readline()      #Read NMEA3
		NMEA5_array=self.NMEA5.split(',')
		# ser1.flushInput()
		# ser1.flushOutput()
		# time.sleep(0.01)		
		
		while ser1.inWaiting()==0:
				pass
		self.NMEA6=ser1.readline()      #Read NMEA4
		NMEA6_array=self.NMEA6.split(',')
		# ser1.flushInput()
		# ser1.flushOutput()
		# time.sleep(0.01)		
		#print 'NMEA3: ',self.NMEA3
		#print 'NMEA4: ',self.NMEA4
		#print 'NMEA5: ',self.NMEA5
		#print 'NMEA6: ',self.NMEA6

		if NMEA3_array[0]=='$PCHRP':
			# print 'NMEA1: ',self.NMEA1
			# print 'NMEA2: ',self.NMEA2
			# print 'NMEA3: ',self.NMEA3
			# print 'NMEA4: ',self.NMEA4
			if NMEA3_array[0]=='$PCHRP':
				self.yaw=NMEA3_array[7] #Yaw or current heading
			
			if NMEA4_array[0]=='$PCHRS':
				self.yaw_rate=NMEA4_array[5] #Yaw Rate
					
			if NMEA5_array[0]=='$PCHRS':
				self.ax=NMEA5_array[3] #Acceleration in X Direction
				self.ay=NMEA5_array[4] #Acceleration in Y Direction
				self.az=NMEA5_array[5] #Acceleration in Z Direction
					
			if NMEA6_array[0]=='$PCHRS':
				self.magx=NMEA6_array[3] #Mag Sensor value in X Direction
				self.magy=NMEA6_array[4] #Mag Sensor value in Y Direction
				self.magz=NMEA6_array[5] #Mag Sensor value in Z Direction
			
# Self routine for GPS and IMU
myGPS=GPS()
imu=UM7()
time.sleep(1)
lat=0
sat=0
flag=0
total_gain=0
distanceToTarget=100

for x in range(0, 150):
	GPIO.output(okled_pin, GPIO.LOW)
	imu.read()
	curr_hdng_deg=float(imu.yaw)
	if curr_hdng_deg<0:
		curr_hdng_deg=curr_hdng_deg+360
	print 'IMU Settling'


# while distanceToTarget > 16:
	# myGPS.read()
	# myGPS.latMin=float(myGPS.latMin)
	# myGPS.latDeg=float(myGPS.latDeg)
	# myGPS.latMin = myGPS.latMin * 0.01666667     #Convert Minutes to Degrees for latitude
	# CurrLat = myGPS.latDeg + myGPS.latMin
	# myGPS.lonDeg=float(myGPS.lonDeg)
	# myGPS.lonMin=float(myGPS.lonMin)
	# myGPS.lonMin = myGPS.lonMin * 0.01666667     #Convert Minutes to Degrees for longitude
	# CurrLong = myGPS.lonDeg + myGPS.lonMin
	# if myGPS.latHem=='S':	#Convert latitude to -ve if in southern hemisphere
		# CurrLat = CurrLat * -1
	# else:
		# CurrLat = CurrLat * 1
	# if myGPS.lonHem=='W':	#Convert longitude to -ve if in western hemisphere
		# CurrLong = CurrLong * -1
	# else:	
		# CurrLong = CurrLong * 1
		
	# Now calculations for Distance to Target
	# TarLat1 = math.radians(TarLat[i])
	# TarLong1 = math.radians(TarLong[i])
	# CurrLat1 = math.radians(CurrLat)
	# CurrLong1 = math.radians(CurrLong)
	# delta = CurrLong1 - TarLong1
	# sdlong = math.sin(delta)
	# cdlong = math.cos(delta)
	# slat1 = math.sin(CurrLat1)
	# clat1 = math.cos(CurrLat1)
	# slat2 = math.sin(TarLat1)
	# clat2 = math.cos(TarLat1)
	# delta1 = (clat1 * slat2) - (slat1 * clat2 * cdlong)
	# delta1 = math.pow(delta1,2) 
	# temp = clat2 * sdlong
	# delta1 = delta1 + math.pow(temp,2)
	# delta1 = math.sqrt(delta1)
	# denom = (slat1 * slat2) + (clat1 * clat2 * cdlong)
	# delta2 = math.atan2(delta1, denom)
	# distanceToTarget = delta2 * 6372795
	
	# GPIO.output(okled_pin, GPIO.LOW)
	# status=0
	# print 'GPS Not Stable.............Distance to Target = ',distanceToTarget

if distanceToTarget < 16:
	GPIO.output(okled_pin, GPIO.HIGH)
	print 'Ready............'

while(status==0):
	status=GPIO.input(start_button)
	GPIO.output(okled_pin, GPIO.HIGH)
	old_status=status
	time.sleep(0.1)

#Main loop
while(i<=n and status==1):
	#print 'Main loop'
	GPIO.output(okled_pin, GPIO.LOW)
	GPIO.output(runled_pin, GPIO.HIGH)
	PWM.set_duty_cycle(esc_pin,float(throttle))
	t1=time.time()
	imu.read()
	curr_hdng_deg=float(imu.yaw)
	if curr_hdng_deg<0:
		curr_hdng_deg=curr_hdng_deg+360
	yaw_rate=float(imu.yaw_rate)
	ax=float(imu.ax)*(-1)
	ay=float(imu.ay)*(-1)
	az=float(imu.az)
	magx=float(imu.magx)
	magy=float(imu.magy)
	magz=float(imu.magz)
	print 'Current Heading(Degrees): ',curr_hdng_deg, 'Yaw_Rate (Degress/Second): ',yaw_rate, 'Ax: ',ax, 'Ay: ',ay, 'Az: ',az, 'MagX: ',magx, 'MagY: ',magy, 'MagZ: ',magz
	if z==1:
		x = x0   # Vehicle start point #center point of the APSRC road
		y = y0  
		d = d0
		z=z+1
	else:
		# print 'Entered in GPS Loop '
		myGPS.read()
		myGPS.latMin=float(myGPS.latMin)
		myGPS.lonMin=float(myGPS.lonMin)
		myGPS.latDeg=float(myGPS.latDeg)
		myGPS.lonDeg=float(myGPS.lonDeg)
		#speed=round(((float(myGPS.speed))*0.514444),2)
		speed=0
		#sat=float(myGPS.sat)
		sat=0
		myGPS.latMin = myGPS.latMin * 0.01666667     #Convert Minutes to Degrees for latitude
		myGPS.lonMin = myGPS.lonMin * 0.01666667     #Convert Minutes to Degrees for longitude
		CurrLat = myGPS.latDeg + myGPS.latMin
		CurrLong = myGPS.lonDeg + myGPS.lonMin
		if myGPS.latHem=='S':	#Convert latitude to -ve if in southern hemisphere
			CurrLat = CurrLat * -1
		if myGPS.lonHem=='W':	#Convert longitude to -ve if in western hemisphere
			CurrLong = CurrLong * -1
		x=CurrLat
		y=CurrLong
	#Now calculations for Distance to Target
	TarLat1 = math.radians(TarLat[i])
	TarLong1 = math.radians(TarLong[i])
	CurrLat1 = math.radians(x)
	CurrLong1 = math.radians(y)
	delta = CurrLong1 - TarLong1
	sdlong = math.sin(delta)
	cdlong = math.cos(delta)
	slat1 = math.sin(CurrLat1)
	clat1 = math.cos(CurrLat1)
	slat2 = math.sin(TarLat1)
	clat2 = math.cos(TarLat1)
	delta1 = (clat1 * slat2) - (slat1 * clat2 * cdlong)
	delta1 = math.pow(delta1,2) 
	temp = clat2 * sdlong
	delta1 = delta1 + math.pow(temp,2)
	delta1 = math.sqrt(delta1)
	denom = (slat1 * slat2) + (clat1 * clat2 * cdlong)
	delta2 = math.atan2(delta1, denom)
	distanceToTarget = delta2 * 6372795
					
	#Now calculations for Target Heading
	dlon = TarLong1-CurrLong1
	a1 = math.sin(dlon) * math.cos(TarLat1)
	a2 = math.sin(CurrLat1) * math.cos(TarLat1) * math.cos(dlon)
	a2 = math.cos(CurrLat1) * math.sin(TarLat1) - a2
	a2 = math.atan2(a1, a2)
	if a2 < 0.0:
		a2 = a2 + (2*math.pi)
	targetHeading = math.degrees(a2)
	print 'Current Lattitude (Degrees): ',x, 'Current Longitude (Degrees): ',y, 'Distance to target (Mtrs.): ',distanceToTarget, 'Target Heading (Degrees): ',targetHeading	
	
	headingerror = targetHeading - curr_hdng_deg
	# adjust for compass wrap
	if headingerror < -180:     
		headingerror = headingerror+360
	if headingerror > 180:
		headingerror = headingerror-360
	# If heading error is negative turn servo to left and vice versa
	# Steering system PID controller
	p_gain = (headingerror*0.04)
	integral = integral + headingerror
	#i_gain = 0.0001*integral
	i_gain=0
	total_gain=p_gain+i_gain
	if distanceToTarget > WAYPOINT_DIST_TOLERANCE:
		if abs(headingerror) <= HEADING_TOLERANCE:
			ser_dc=26.2
			time.sleep(0.05)
		else:
			ser_dc = 26.2 + ((total_gain)) #21 being extreme left and 31 being extreme right
		if ser_dc<=21:
			ser_dc = 21		
		if ser_dc>=31:
			ser_dc = 31
		PWM.set_duty_cycle(ser_pin,float(ser_dc))
		time.sleep(0.1)
	elif distanceToTarget <= WAYPOINT_DIST_TOLERANCE:
		PWM.set_duty_cycle(esc_pin,float(dc_stop))
		time.sleep(3)
		i=i+1			
	
	t2=time.time()
	looptime=t2-t1
	print 'Loop Time: ',looptime
	print "\n"
	f.write("%0.2f %0.1f %0.8f %0.8f %0.8f %0.8f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f\n" %(looptime,i,x,y,TarLat[i],TarLong[i],curr_hdng_deg,targetHeading,headingerror,distanceToTarget,yaw_rate,ax,ay,az,magx,magy,magz,ser_dc,total_gain))
	status=GPIO.input(start_button)
	if status==0:
		GPIO.output(okled_pin, GPIO.HIGH)
		GPIO.output(runled_pin, GPIO.LOW)
		PWM.set_duty_cycle(esc_pin,float(dc_stop))
		time.sleep(0.1)
		break
while True:
	PWM.stop(esc_pin)
	PWM.stop(ser_pin)
	PWM.cleanup()
	f.close()


