from time   import sleep
from ev3dev.auto import *
"""
	To run: send project to robot, run:
	python project3c.py
"""

#Initializing motors, sensor and button
# Connect two large motors on output ports B and C:
motors = [LargeMotor(address) for address in (OUTPUT_A, OUTPUT_B)]
#check that the device has actually been connected.
assert all([m.connected for m in motors]), \
    "Two large motors should be connected to ports A and B"
cs = ColorSensor();    assert cs.connected
btn = Button()

#creating global variables
done, rightspeed, leftspeed = 0,0,0
# set constants for P.I.D.
kp = 1
ki = 0.01
kd = 0.1
speed = -50		#set speed
sp = 38			#set sensor (the goal)
i = 0
preError = 0

#starts motor at set speed to get them rolling
def start():
	for m in motors:
		m.run_direct()
		m.duty_cycle_sp = speed

#Takes right and left values and updates motor speed using duty_cycle_sp
def updateSpeed(right, left):
	global motors
	if right < -100:
		right = -100
	elif right > 100:
		right = 100
	if left < -100:
		left = -100
	elif left > 100:
		left = 100
	motors[0].duty_cycle_sp = right
	motors[1].duty_cycle_sp = left

#calculates the speed of right and left motors (PID controller)
def getRightAndLeft(spi, spo, actual):
	global leftspeed, rightspeed
	global ki, i, kp, kd, preError
	motorMin = -100
	motorMax = 100
	sensorMin = 5
	sensorMax = 75
	E = spi - actual
	p = actual - spo				#calculate p
	if(abs((i*0.8+E)*ki) < 1000):	#calculate i with bounds
		i = i * 0.8
		i = E + i
	d = preError - E								#calculate d
	lowRise = spo - motorMin
	lowRun = actual - sensorMin
	highRise = motorMax - spo
	highRun = sensorMax - actual
	if(E > 0):
		leftspeed  = spo + lowRise/lowRun*p*kp   + i*ki + d*kd
		rightspeed = spo + highRise/lowRun*p*kp  + i*ki + d*kd
	else:
		leftspeed =  spo + highRise/highRun*p*kp + i*ki + d*kd
		rightspeed = spo + lowRise/highRun*p*kp  + i*ki + d*kd
	
	preError = E
	print "R: %d L: %d P: %d I: %d D: %d" % (rightspeed, leftspeed, E, i*ki, kd*d)

# Check for button press to exit
def buttonCheck():
	global done
	if btn.backspace:
		done = 1
		
# Wait for enter to start
while not btn.enter:
	sleep(0.01)

sleep(0.5)
start()
prevalue = cs.value()
while not done == 1:
	value = cs.value()
	#getPowerLeft(sp, speed, value)
	#getPowerRight(sp, speed, value)
	getRightAndLeft(sp, speed, value)
	updateSpeed(rightspeed, leftspeed)
	buttonCheck()
	prevalue = value
	sleep(.00001)

# Stop the motors before exiting.
for m in motors:
	m.stop()
