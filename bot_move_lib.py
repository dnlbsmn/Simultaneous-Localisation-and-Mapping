import slamBotHD as sb
import math
from time import sleep
from time import time
from constants import *

prev_flag = False
	
def relative_rotate(angle):
	error = 2
	desired_angle = float(sb.imuYaw) + angle
	P = ROTATE_P
	I = ROTATE_I
	dt = 0
	prev_time = 0
	integral = 0
	error_count = 0
	r_speed_prev = 0
	
	while(True):
		dt = time() - prev_time
		prev_time = time() 
		error = angle_difference(desired_angle, sb.imuYaw)
		
		integral += error/dt
		
		if (abs(error) > 5):
			integral = 0
		
		rotate_speed = P*error + I*integral 
		print(error)
		
		if (rotate_speed > 1.3):
			rotate_speed = 1.3
		if (rotate_speed < -1.3):
			rotate_speed = -1.3
			
		if ((rotate_speed - r_speed_prev) > 0.2):
			rotate_speed = r_speed_prev + 0.2
		if ((rotate_speed - r_speed_prev) < -0.2):
			rotate_speed = r_speed_prev - 0.2
		
		r_speed_prev = rotate_speed	
		
		sb.moveBot(0, rotate_speed)
		
		if (abs(error) > 1): 
			error_count = 0
		else:
			error_count += 1
		
		
		if (error_count > 20):
			print("break")
			break
			
		sleep(0.1)

def angle_difference(angle1, angle2):
	if (angle1 - angle2 > 180):
		angle1 -= 360
		
	if (angle1 - angle2 < -180):
		angle1 += 360
		
	return angle1 - angle2
		
def read_encoders(l_init, r_init):
	global l_over, r_over, r_prev, l_prev
	
	l = sb.encLeft
	r = sb.encRight
	
	left =  (7.07*math.pi*((l / 2578) + (l_over / 2578) - (l_init / 2578)))
	right = (7.07*math.pi*((r / 2578) + (r_over / 2578) - (r_init / 2578)))
	
	print("RL: ", r, l)
	# forward overflow catch
	if (l_prev - l > 50000):
		l_over  += 65535
		print("overflow")
	
	if (r_prev - r > 50000):
		r_over  += 65535
		print("overflow2")
	
	# reverse overflow catch
	if (l_prev - l < -50000):
		l_over  -= 65535
		print("overflow3")
		
	if (r_prev - r < -50000):
		r_over  -= 65535
		print("overflow4")
		
	l_prev = l
	r_prev = r
			
	return [left, right]

def relative_move(dist):
	global l_prev, r_prev, l_over, r_over
	
	l_init = sb.encLeft
	r_init = sb.encRight
	l_prev = l_init
	r_prev = r_init
	l_over = 0
	r_over = 0
	
	heading = sb.imuYaw
	
	P = LINEAR_P
	I = LINEAR_I
	integral = 0
	prev_time = 0
	error_counter = 0
	forward_prev = 0
	
	
	while(True):
		dt = time() - prev_time
		prev_time = time() 
		
		left, right = read_encoders(l_init, r_init)
		d = (left + right)/2
		
		error = dist - d
		integral += error/dt
		
		skew = sb.imuYaw - heading
		skew_speed = skew*SKEW_P
		if (skew_speed > 0.5): skew_speed = 0.5
		
		if (abs(error) > 5):  integral = 0
		
		forward_speed = P*error + I*integral
		
		if (abs(forward_speed) > LINEAR_VEL_CAP):
			forward_speed = LINEAR_VEL_CAP
			
		if ((forward_speed - forward_prev) > LINEAR_ACC_CAP):
			forward_speed = forward_prev + LINEAR_ACC_CAP
		
		forward_prev = forward_speed	
		
		sb.moveBot(forward_speed, -skew_speed)
		
		if (abs(error) > 1): 
			error_count = 0
		else:
			error_count += 1
		
		if (error_count > 10):
			print("break")
			break
			
		print("INIT: ", l_init, r_init)
		print("CM: ", left, right)
		print("OVER: ", l_over, r_over)
		#print(forward_speed)
		#print(sb.encLeft, sb.encRight)
		#print("error: ", error, "forward: ", forward_speed, "integral: ", I*integral)
		sleep(0.1)
	
	print(d)
	sb.moveBot(0, 0)
	
'''
sb.startUp()
sb.readCoreData()

print(sb.botBatt)

sleep(6)
a = sb.imuYaw

relative_rotate(0)
sleep(2)
relative_move(60)
sleep(2)
relative_rotate(-126.87)
sleep(2)
relative_move(100)
sleep(2)
relative_rotate(-143.13)
sleep(2)
relative_move(80)
sleep(2)
relative_rotate(-90)

relative_rotate(0)
relative_move(5)
relative_rotate(-180)
relative_move(5)
relative_rotate(-180)
relative_move(3)
relative_rotate(-180)
relative_move(3)
relative_rotate(-180)
relative_move(15)
relative_rotate(-180)
relative_move(15)
relative_rotate(-180)
relative_move(20)
relative_rotate(-180)
relative_move(20)
relative_rotate(-180)
sleep(2)

print(a)
print(sb.imuYaw)

#sb.moveBot(0, 0)

sleep(1)
rotate_set_angle(90)
sleep(0.5)
rotate_set_angle(90)
sleep(0.5)
rotate_set_angle(90)
sleep(0.5)
rotate_set_angle(90)
print("Mooooooooooooooooooove")

sb.shutDown()
'''
