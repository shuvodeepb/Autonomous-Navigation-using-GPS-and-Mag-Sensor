# Updated with adafruit libraries
#Code developed for beagle bone black RevC 
import serial
import math
import Adafruit_BBIO.UART as UART
from time import sleep
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import smtplib
import time
import smbus  # for IMU sensor
ADC.setup()

# PWM reset
esc_pin = "P9_21"
ser_pin = "P8_13"
dc_fbeep = 32
PWM.start(esc_pin, dc_fbeep, 200)
time.sleep(3)
ser_dc = 7.5
PWM.start(ser_pin, ser_dc, 50) #starting duty cycle for ser_pin
time.sleep(0.1)
PWM.set_duty_cycle(esc_pin,float(dc_fbeep))
PWM.stop(esc_pin)
PWM.stop(ser_pin)

# Mag/IMU sensor parameters
bus = smbus.SMBus(2) # Get I2C bus
bus.write_byte_data(0x1E, 0x01, 0x60) #(HMC5883 address, Register A address, Measurement Mode)
bus.write_byte_data(0x1E, 0x02, 0x00) #(HMC5883 address, Mode Register A , Contd. Measurement Mode)
declination = -0.0721		    #radian				     #Houghton declination -4 Degrees and 8 Minutes

# ESC unit parameters
esc_pin = "P9_21"
PWM.start(esc_pin, 32, 200)
time.sleep(3)
throttle = 33.8

# Servo unit parameters
ser_pin = "P8_13"
ser_dc = 7.5                 # straight position of servo duty cycle
PWM.start(ser_pin, ser_dc, 50) #starting duty cycle for ser_pin
led_pin = "P8_8"   # for checking the Gps fix

# Speed sensor parameters
spd_pin = "P8_7"
t1 = 0
t2 = 0
dc_stop=15
dc_slow=33.4
dc_med=33.6
dc_cruise=33.8
dc_fast=33.8
pin = [0,0]
t_vel = [0,0]
speed_mps = [0,0]
diameter = 0.0889
sp = 0
speed_mps = 0.05

#PWM.set_duty_cycle(esc_pin,float(33.8))
GPIO.setup(spd_pin, GPIO.IN) # Speed sensor pin setup
GPIO.setup(led_pin, GPIO.OUT) # Speed sensor pin setup
GPIO.output("P8_8", GPIO.LOW)

# Gps parameters
UART.setup("UART1")
#ser=serial.Serial('/dev/ttyO1',57600) # for Tiny gps unit
ser=serial.Serial('/dev/ttyO1',9600)   # for Adafruit Gps unit


# general parameters
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

# Way point/map parameters
n=4 #number of waypoints, zero position being first waypoint
i=0
WAYPOINT_DIST_TOLERANCE = 3
HEADING_TOLERANCE = 10
TarLat = [47.169502,47.169640,47.169795,47.169917,47.169934]
TarLong = [-88.507541,-88.507583,-88.507640,-88.507768,-88.508037]
x0 = 47.169502    # Vehicle start point  
y0 = -88.507711  #just near to the back door of the APSRC

# postprocessing parameters
f=open("final.txt","a")
f.write("SampleTime Count Waypoint CurrLat CurrLong TargetLat TargetLong CurrHeading TarHeading HeadingError Dist2Tar(m) Servo dist(m) speed(m/s)\n")

def collision_detect():
		value=ADC.read("P9_40")
		voltage=value*1.8     
		distance=(voltage/0.00457)*0.7 # change this number for calibrating the sensor
		#print"distance: %f" %distance
		if distance>200:
			PWM.set_duty_cycle(esc_pin,float(dc_fast))
		elif distance>150 and distance<=200:
			PWM.set_duty_cycle(esc_pin,float(dc_cruise))
			time.sleep(0.1)
		elif distance>100 and distance<=150:
			PWM.set_duty_cycle(esc_pin,float(dc_med))
			time.sleep(0.1)
		else:
			PWM.set_duty_cycle(esc_pin,float(dc_stop))	
			time.sleep(0.1)
		return distance
def IMU():
		
		#Calibration Values for X
		xa=130
		xb=153
		xc=2.6144

		#Calibration Values for Y
		ya=312
		yb=156
		yc=2.5641
		
		# Read data back from 0x03(03), 6 bytes
		# X-Axis MSB, X-Axis LSB, Z-Axis MSB, Z-Axis LSB, Y-Axis MSB, Y-Axis LSB
		data = bus.read_i2c_block_data(0x1E, 0x03, 6)
		# Convert the data
		xMag = data[0] * 256 + data[1]
		if xMag > 32767:
			xMag = float(xMag - 65536)
		zMag = data[2] * 256 + data[3]
		if zMag > 32767:
			zMag = float(zMag - 65536)
		yMag = data[4] * 256 + data[5]
		if yMag > 32767:
			yMag = float(yMag - 65536) 
		bx=xMag
		by=yMag
		print 'before x: ',bx
		print 'before y: ',by
		xMag=((xMag+xa)-xb)*xc
		yMag=((yMag+ya)-yb)*yc
		print 'a x Value:%f ',xMag
		print 'a y Value:%f ',yMag
		# Output data to screen
			#print "X-Axis : %d " %xMag
			#print "Magnetic field in Y-Axis : %d" %yMag
		heading = math.atan2(yMag,xMag)
		heading=heading+declination
		if heading<0:
			heading=heading+(2*math.pi)
		elif heading>2*math.pi:
			heading=heading-(2*math.pi)
		headingdegress = math.degrees(heading) #Radians to Degrees

		return headingdegress


class GPS:				      #Create GPS class
		def __init__(self):     #This init will run when you create a GPS object.
				#This sets up variables for useful commands.
				#This set is used to set the rate the GPS reports
				UPDATE_10_sec=  "$PMTK220,10000*2F\r\n" #Update Every 10 Seconds
				UPDATE_5_sec=  "$PMTK220,5000*1B\r\n"   #Update Every 5 Seconds  
				UPDATE_1_sec=  "$PMTK220,1000*1F\r\n"   #Update Every One Second
				UPDATE_200_msec=  "$PMTK220,200*2C\r\n" #Update Every 200 Milliseconds
				#This set is used to set the rate the GPS takes measurements
				MEAS_10_sec = "$PMTK300,10000,0,0,0,0*2C\r\n" #Measure every 10 seconds
				MEAS_5_sec = "$PMTK300,5000,0,0,0,0*18\r\n"   #Measure every 5 seconds
				MEAS_1_sec = "$PMTK300,1000,0,0,0,0*1C\r\n"   #Measure once a second
				MEAS_200_msec= "$PMTK300,200,0,0,0,0*2F\r\n"  #Meaure 5 times a second
				#Set the Baud Rate of GPS
				BAUD_57600 = "$PMTK251,57600*2C\r\n"		  #Set Baud Rate at 57600
				BAUD_9600 ="$PMTK251,9600*17\r\n"		     #Set 9600 Baud Rate
				#Commands for which NMEA Sentences are sent

				GPGGA_ONLY="$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"#Send GPGGA Sentences
				# SEND_ALL ="$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" #Send All Sentences
				# SEND_NOTHING="$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" #Send Nothing
				ser.write(BAUD_9600)   #Set Baud Rate to 57600
				sleep(1)				#Paulse
				# ser.baudrate=57600      #IMPORTANT Since change ser baudrate to match GPS
				ser.baudrate=9600      #IMPORTANT Since change ser baudrate to match GPS
				ser.write(UPDATE_1_sec) #Set update rate
				sleep(1)
				ser.write(MEAS_200_msec)  #Set measurement rate
				sleep(1)
				ser.write(GPGGA_ONLY)    #Ask for only GPRMC and GPGGA Sentences
				sleep(1)
				ser.flushInput()		  #clear buffers
				ser.flushOutput()
				# print "GPS is Initialized" #Print message
		def read(self):
				# print 'Entered in read'
				ser.flushInput()
				ser.flushInput()
				while ser.inWaiting()==0:
					pass
				self.NMEA1=ser.readline()

				NMEA1_array=self.NMEA1.split(',')
				# print 'before NMEA if'
				if len(NMEA1_array)<15 or NMEA1_array[0]!='$GPGGA' or NMEA1_array[6]==' ' or NMEA1_array[6]=='0':
					# print 'C1'
					self.x_gps = 0  # if NMEA sentance is corrputed use prediction values
					self.y_gps = 0
					# self.d_gps = 0
					

				else: 
					# print 'C3'
					self.latDeg=NMEA1_array[2][:-7]
					self.latMin=NMEA1_array[2][-7:]
					self.latHem=NMEA1_array[3]
					self.lonDeg=NMEA1_array[4][:-7]
					self.lonMin=NMEA1_array[4][-7:]
					self.lonHem=NMEA1_array[5]
					self.fix=1
					
					self.x_new_Deg = float(self.latDeg)
					self.x_new_Min = float(self.latMin)
					self.y_new_Deg = float(self.lonDeg)
					self.y_new_Min = float(self.lonMin)
					self.x_new_Min = self.x_new_Min * 0.01666667
					self.y_new_Min = self.y_new_Min * 0.01666667
					self.x_new = self.x_new_Deg + self.x_new_Min     # lat is defined as x_new_rad
					self.y_new = self.y_new_Deg + self.y_new_Min
					# print 'Lat_gps worked'
					if self.latHem=='S':						#Convert lattitude to -ve if in southern hemisphere
						self.x_new = self.x_new * -1
					if self.lonHem=='W':						#Convert longitude to -ve if in western hemisphere
						self.y_new = self.y_new * -1
					if abs(self.x_new - x)>0.0001 or abs(self.y_new - y)>0.0001:
						self.x_gps = 0  # if NMEA sentance is corrputed use prediction values
						self.y_gps = 0
						# self.d_gps = 0
					else:
						self.x_gps = self.x_new
						self.y_gps = self.y_new
						# self.d_gps = 0
						# print 'GPS else worked'
myGPS=GPS()
PWM.set_duty_cycle(esc_pin,float(throttle))
dist=collision_detect()
while(i<=n):
	
	t1=time.time()
	if i==2 or i==3:
		dc_fast = 33.9
	else:
		dc_fast = 33.8
	curr_hdng_deg=IMU()
	# sp=0
	# while(sp<=500):
		# pin[0] = pin[1]
		# pin[1] = GPIO.input(spd_pin)
		# sp=sp+1
		# if (pin[0]==1 and pin[1]==0):
			# print 'Enter If'
			# total_time = 0
			# print '1 rotation completed'
			# t_vel[0] = t_vel[1]
			# t_vel[1]= time.time()
			# total_time = t_vel[1]-t_vel[0]
			# speed_mps[0] = speed_mps[1]
			# speed_mps[1] = (3.14*diameter)/total_time
		# print'sp:',sp
	# if speed_mps[0]>100:
		# speed_mps[0]=0
	if z==1:
		# print 'Loop 1st if Entered'
		x= x0   # Vehicle start point
		y = y0  #center point of the APSRC road
		d= d0
		z=z+1
	else:
		# print 'Entered in main cal loop'
		if count==1:
			flag=1
			
		elif count<gpscount and count>1:
			flag=0

		if flag==1:
			# print 'Entered in GPS Loop '
			dist=collision_detect()	
			myGPS.read()
			if myGPS.x_gps>0:
				x = myGPS.x_gps
				y = myGPS.y_gps
				d = 0
			else:
				x = x + myGPS.x_gps
				y = y + myGPS.y_gps
				d = 0
			count=count+1
			
		elif flag==0:
			# print 'Entered in prediction Loop '
			dist=dist-10  #It is approximated that it moves 10 cm per loop. Should be calibrated based on speed
			if dist>200:
				PWM.set_duty_cycle(esc_pin,float(dc_fast))
			elif dist>150 and dist<=200:
				PWM.set_duty_cycle(esc_pin,float(dc_cruise))
				time.sleep(0.1)
			elif dist>100 and dist<=150:
				PWM.set_duty_cycle(esc_pin,float(dc_med))
				time.sleep(0.1)
			else:
				PWM.set_duty_cycle(esc_pin,float(dc_stop))	
				time.sleep(0.1)		
			c_heading_rad=math.radians(curr_hdng_deg)
			d_x=speed_mps*(math.cos(c_heading_rad))*looptime
			d_y=speed_mps*(math.sin(c_heading_rad))*looptime
			d_cal=math.sqrt((d_x*d_x) + (d_y*d_y))
			d=d+d_cal
			x_rad=math.radians(x)
			y_rad=math.radians(y)
			delta__predict=d/6372795
			x_new_rad = math.asin((math.sin(x_rad)*math.cos(delta__predict))+(math.cos(x_rad)*math.sin(delta__predict)*math.cos(c_heading_rad)))
			y_new_rad = y_rad+ math.atan2((math.sin(c_heading_rad)*math.sin(delta__predict)*math.cos(x_rad)),(math.cos(delta__predict)-math.sin(x_rad)*math.sin(x_new_rad)))
			x_new=math.degrees(x_new_rad) #Current Lattitude in radians
			y_new=math.degrees(y_new_rad) #Current Longitude in radians
			x=x_new #Current Lat
			y=y_new #Current Long
			d = 0
			count=count+1
			if count==gpscount:
				count=1
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
	#print 'Distance to target (Mtrs.): ',distanceToTarget
	#print 'Target Heading (Degrees): ',targetHeading	
	
	headingerror = targetHeading - curr_hdng_deg
	# adjust for compass wrap
	if headingerror < -180:     
		headingerror = headingerror+360
	if headingerror > 180:
		headingerror = headingerror-360
# if heading error is negitive turn servo to left and vice versa
	p_gain = ((headingerror*2)/1000)
	#integral = integral + headingerror
	#i_gain = 0.0001*integral
	i_gain=0
	if distanceToTarget > WAYPOINT_DIST_TOLERANCE:
		if abs(headingerror) <= HEADING_TOLERANCE:
			PWM.set_duty_cycle(ser_pin,float(7.5))
			#integral=0
			time.sleep(0.1)
		# ser_dc = (7.0 - (9.7222*(p_gain + i_gain)))
		ser_dc = (0.0089*(p_gain + i_gain)) + 7.4333
		if ser_dc<=4:
			ser_dc = 4		
		if ser_dc>=11:
			ser_dc = 11
		PWM.set_duty_cycle(ser_pin,float(ser_dc))
		time.sleep(0.1)
	elif distanceToTarget <= WAYPOINT_DIST_TOLERANCE:
		PWM.set_duty_cycle(esc_pin,float(dc_stop))
		time.sleep(3)
		i=i+1			
	
	t2=time.time()
	looptime=t2-t1
	f.write("%0.2f %0.2f %0.2f %0.8f %0.8f %0.8f %0.8f %0.8f %0.8f %0.8f %0.8f %0.2f %0.2f %0.8f\n" %(looptime,count,i,x,y,TarLat[i],TarLong[i],curr_hdng_deg,targetHeading,headingerror,distanceToTarget,ser_dc,d,speed_mps))
	#print 'speed (m/s): ',speed_mps[0]
	print 'currentheading: ',curr_hdng_deg
	print 'targetheading: ',targetHeading
	print 'targetheading: ',headingerror
	print 'Dist 2 Tar (m): ',distanceToTarget
	print 'Servo angle: ',ser_dc
	print 'Loop Time: ',looptime
while True:
	PWM.set_duty_cycle(esc_pin,float(dc_stop))
	PWM.stop(esc_pin)
	PWM.stop(ser_pin)
	PWM.cleanup()
	f.close()

