import slamBotHD as sb
import math
from time import sleep
from time import time

prev_flag = False
	
def relative_rotate(angle):
	error = 2
	desired_angle = sb.imuYaw + angle
	P = 0.02
	I = 0.00003
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
		
		if (error > 5):
			integral = 0
		
		rotate_speed = P*error + I*integral 
		print(error)
		
		if (abs(rotate_speed) > 1.3):
			rotate_speed = 1.3
			
		if ((rotate_speed - r_speed_prev) > 0.2):
			rotate_speed = r_speed_prev + 0.2
		
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
	if (abs(angle1 - angle2) > 180):
		angle1 -= 360
		
	return angle1 - angle2
		
def read_encoders(l_init, r_init):
	global l_error, l_over, l_prev, r_error, r_over, r_prev
	
	l = int(sb.encLeft)
	r = int(sb.encRight)
	
	if ((abs(l_prev - l) > 50000) and (prev_flag)):
		l_error += (65535 % 2578)
		l_over += 65535
		print("overflow")
		
	if ((abs(r_prev - r) > 50000) and (prev_flag)):
		r_error += (65535 % 2578)
		r_over += 65535
		print("overflow")
		
	
	left =  (7.07*math.pi*(l + l_over - l_init)) / 2578 # + l_error
	right = (7.07*math.pi*(r + r_over - r_init)) / 2578
	
	l_prev = l
	r_prev = r
	prev_flag = True
	
	return [left, right]
	
	# [398.35492629945696, 412.2948021722265]
	# Output after going about 105 cm


def relative_move(dist):
	l_init = sb.encLeft
	r_init = sb.encRight
	prev_flag = False
	
	P = 0.003
	I = 0.00002
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
		
		if (error > 20):
			integral = 0
		
		forward_speed = P*error + I*integral
		
		if (abs(forward_speed) > 0.4):
			forward_speed = 0.4
			
		if ((forward_speed - forward_prev) > 0.1):
			forward_speed = forward_prev + 0.1
		
		forward_prev = forward_speed	
		
		sb.moveBot(forward_speed, 0)
		
		if (abs(error) > 2): 
			error_count = 0
		else:
			error_count += 1
		
		if (error_count > 10):
			print("break")
			break
		print(left, right)
		print(forward_speed)
		#print(sb.encLeft, sb.encRight)
		#print("error: ", error, "forward: ", forward_speed, "integral: ", I*integral)
		sleep(0.1)
	
	print(d)
	sb.moveBot(0, 0)
	

sb.startUp()
sb.readCoreData()

print(sb.botBatt)

l_error = 0
l_over = 0
l_prev = 0
r_error = 0
r_over = 0
r_prev = 0

a = sb.imuYaw

relative_rotate(180)
sleep(0.2)
relative_move(50)
sleep(2)
print(a)
print(sb.imuYaw)

#sb.moveBot(0, 0)
"""
sleep(1)
rotate_set_angle(90)
sleep(0.5)
rotate_set_angle(90)
sleep(0.5)
rotate_set_angle(90)
sleep(0.5)
rotate_set_angle(90)
print("Mooooooooooooooooooove")
"""
sb.shutDown()
