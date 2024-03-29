#!/usr/bin/env python
import math
import numpy as np
from itertools import count
import rospy 
import tf 
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist
from switch_frame_config import*
from Parameters import parameters

# ---------- Global Variables ---------------
Command_Vel = [0]*3
Command_Vel[0] = 0.5	# Cmd along X
Command_Vel[1] = 0		# Cmd along Y
Command_Vel[2] = 0		# Cmd along Z

Error_Vel = [0]*3		#Error velocity 

angle_wrt_startup = [0]*3 
init_a0 = True

depth_wrt_startup = 0
depth_p0 = 0
init_p0 = True

Vmax_mot = 1900
Vmin_mot = 1100

arming = False
set_mode = [0]*3
set_mode[0] = True				# Mode manual
set_mode[1] = False				# Mode automatic without correction
set_mode[2] = False				# Mode with correction
enable_depth = False 			# Don't Publish the depth data until asked
								# Error Accumulation
custom_8_motors = False 
custom_PID = False
counter = 0
def joyCallback(data):
	global arming
	global set_mode
	global init_a0
	global init_p0
	global custom_8_motors
	btn_arm = 		data.buttons[7] # Start button
	btn_disarm = 		data.buttons[6] # Back button
	btn_manual_mode = 	data.buttons[3] # Y button
	btn_automatic_mode = 	data.buttons[2] # X button
	btn_corrected_mode =	data.buttons[0] # A button
	

	# Disarming when Back button is pressed
	if (btn_disarm == 1 and arming == True):
		arming = False
		armDisarm(arming)
	# Arming when Start button is pressed
	if (btn_arm == 1 and arming == False):
		arming = True
		armDisarm(arming)

	# Switch manual and auto mode
	if (btn_manual_mode and not set_mode[0]):
		set_mode[0] = True
		set_mode[1] = False
		set_mode[2] = False		
		rospy.loginfo("Mode manual")
	if (btn_automatic_mode and not set_mode[1]):
		set_mode[0] = False
		set_mode[1] = True
		set_mode[2] = False	
		rospy.loginfo("Mode automatic")
	if (btn_corrected_mode and not set_mode[2]):
		init_a0 = True
		init_p0 = True
		# set sum errors to 0 here, ex: Sum_Errors_Vel = [0]*3
		set_mode[0] = False
		set_mode[1] = False
		set_mode[2] = True	
		rospy.loginfo("Mode correction")

def armDisarm(armed):
	# This functions sends a long command service with 400 code to arm or disarm motors
	if (armed):
		rospy.wait_for_service('mavros/cmd/command')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
			armService(0, 400, 0, 1, 0, 0, 0, 0, 0, 0)
			rospy.loginfo("Arming Succeeded")
		except (rospy.ServiceException, e):
			rospy.loginfo("Except arming")
	else:
		rospy.wait_for_service('mavros/cmd/command')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
			armService(0, 400, 0, 0, 0, 0, 0, 0, 0, 0)
			rospy.loginfo("Disarming Succeeded")
		except (rospy.ServiceException, e):
			rospy.loginfo("Except disarming")	



def velCallback(cmd_vel):
	global set_mode
	# Only continue if manual_mode is enabled
	if (set_mode[1] or set_mode[2]):
		return

	# Extract cmd_vel message
	# roll_left_right 	= mapValueScalSat(cmd_vel.angular.x)
	# yaw_left_right 		= mapValueScalSat(-cmd_vel.angular.z)
	# ascend_descend 		= mapValueScalSat(cmd_vel.linear.z)
	# forward_reverse 	= mapValueScalSat(cmd_vel.linear.x)
	# lateral_left_right 	= mapValueScalSat(-cmd_vel.linear.y)
	# pitch_left_right 	= mapValueScalSat(cmd_vel.angular.y)
	# setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right)
	

	# roll_left_right 	= mapValueScalSat(cmd_vel.angular.x)
	# yaw_left_right 		= mapValueScalSat(-cmd_vel.angular.z)
	# ascend_descend 		= mapValueScalSat(cmd_vel.linear.z)
	# forward_reverse 	= mapValueScalSat(cmd_vel.linear.x)
	# lateral_left_right 	= mapValueScalSat(-cmd_vel.linear.y)
	# pitch_left_right 	= mapValueScalSat(cmd_vel.angular.y)

	surge_f = mapValueScalSat(cmd_vel.linear.x)
	surge	= force_Cmd( surge_f)
	sway 	= force_Cmd(mapValueScalSat(-cmd_vel.linear.y))
	heave 	= force_Cmd(mapValueScalSat(-cmd_vel.linear.z))
	roll 	= force_Cmd(mapValueScalSat(cmd_vel.angular.x))
	pitch 	= force_Cmd(mapValueScalSat(-cmd_vel.angular.y))
	yaw 	= force_Cmd(mapValueScalSat(-cmd_vel.angular.z))
	print("surge_foce", surge_f)
	print("surge", surge)
	print("sway", sway)
	print("heave", heave)
	print("roll", roll)
	print("pitch", pitch)
	print("yaw", yaw)

	setOverrideRCIN_6ddl(surge, sway, heave, roll, pitch, yaw)
	

def OdoCallback(data):
	global angle_roll_a0
	global angle_pitch_a0
	global angle_yaw_a0
	global angle_wrt_startup
	global init_a0

	orientation = data.orientation
	# extraction of yaw angle
	q = [orientation.x, orientation.y, orientation.z, orientation.w]
	euler = tf.transformations.euler_from_quaternion(q)
	angle_roll = euler[0]
	angle_pitch = euler[1]
	angle_yaw = euler[2]

	if (init_a0):
		# at 1st execution, init
		angle_roll_a0 = angle_roll
		angle_pitch_a0 = angle_pitch
		angle_yaw_a0 = angle_yaw
		init_a0 = False

	angle_wrt_startup[0] = ((angle_roll - angle_roll_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	angle_wrt_startup[1] = ((angle_pitch - angle_pitch_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	angle_wrt_startup[2] = ((angle_yaw - angle_yaw_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
		
	angle = Twist()
	angle.angular.x = angle_wrt_startup[0]
	angle.angular.y = angle_wrt_startup[1]
	angle.angular.z = angle_wrt_startup[2]

	pub_angle_degre.publish(angle)


def PressureCallback(data):
	global depth_p0
	global depth_wrt_startup
	global init_p0
	global enable_depth
	global custom_PID
	global counter

	rho = 1000.0 # 1025.0 for sea water
	g = 9.80665

	if(enable_depth ):
		pressure = data.fluid_pressure
		if (init_p0):
			# 1st execution, init
			depth_p0 += (pressure - 101300)/(rho*g)
			counter += 1
			if(counter == 100):
				depth_p0 /= 100
				init_p0 = False
    
		else:
			depth_wrt_startup = (pressure - 101300)/(rho*g) - depth_p0
			print(depth_wrt_startup)

			if(custom_PID):
				#ControlDepth(0.5, depth_wrt_startup)
			# else:
				msg = Float64()
				msg.data = depth_wrt_startup
				pub_depth.publish(msg)


	# Only continue if manual_mode is disabled
	if (set_mode[0]):
		return
	elif (set_mode[1]):
		# Only continue if automatic_mode is enabled
		# Define an arbitrary velocity command and observe robot's velocity
		#setOverrideRCIN(1550, 1500, 1500, 1500, 1500, 1500)
		return




def mapValueScalSat(value):
	global Vmax_mot
	global Vmin_mot
	# Correction_Vel and joy between -1 et 1
	# scaling for publishing with setOverrideRCIN values between 1100 and 1900
	# neutral point is 1500
	pulse_width = value * 400 + 1500

	# On limite la commande en vitesse
	if pulse_width > Vmax_mot:
		pulse_width = Vmax_mot
	if pulse_width < Vmin_mot:
		pulse_width = Vmin_mot
	
	return pulse_width


def setOverrideRCIN(channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
	# This function replaces setservo for motor commands.
	# It overrides Rc channels inputs and simulates motor controls.
	# In this case, each channel manages a group of motors not individually as servo set

	msg_override = OverrideRCIn()

	msg_override.channels[0] = np.uint(channel_pitch)		#pulseCmd[4]  # pitch		Tangage
	msg_override.channels[1] = np.uint(channel_roll)			#pulseCmd[3]  # roll 		Roulis
	msg_override.channels[2] = np.uint(channel_throttle)		#pulseCmd[2]  # up/down		Montee/descente
	msg_override.channels[3] = np.uint(channel_yaw)			#pulseCmd[5]  # yaw		Lace
	msg_override.channels[4] = np.uint(channel_forward)		#pulseCmd[0]  # forward		Devant/derriere
	msg_override.channels[5] = np.uint(channel_lateral)		#pulseCmd[1]  # lateral		Gauche/droite
	msg_override.channels[6] = 1500
	msg_override.channels[7] = 1500
	# print("<3=====D ",msg_override)
	pub_msg_override.publish(msg_override)


def setOverrideRCIN_8_motors(motor_1, motor_2, motor_3,  motor_4, motor_5, motor_6, motor_7, motor_8):
	# This function allow us to control  motors individually
	# It overrides Rc channels inputs and simulates motor controls.
	# In this case, each channel represent a motor signal 

	msg_override = OverrideRCIn()
	msg_override.channels[0] = np.uint(motor_1)		
	msg_override.channels[1] = np.uint(motor_2)			
	msg_override.channels[2] = np.uint(motor_3)		
	msg_override.channels[3] = np.uint(motor_4)			
	msg_override.channels[4] = np.uint(motor_5)		
	msg_override.channels[5] = np.uint(motor_6)		
	msg_override.channels[6] = np.uint(motor_7)	
	msg_override.channels[7] = np.uint(motor_8)	
	
	# print("<3=====D ",msg_override)
	pub_msg_override.publish(msg_override)

para = parameters()

def setOverrideRCIN_6ddl(surge, sway, heave, roll, pitch, yaw):
	msg_override = OverrideRCIn()
	genelized_forces_moment = np.array([surge, sway, heave, roll, pitch, yaw])
	motor_force = para.T_inv.dot(genelized_forces_moment)
	print("motorforce", motor_force)
	msg_override.channels[0] = np.uint(PWM_Cmd(motor_force[0]))		
	msg_override.channels[1] = np.uint(PWM_Cmd(motor_force[1]))			
	msg_override.channels[2] = np.uint(PWM_Cmd(motor_force[2]))		
	msg_override.channels[3] = np.uint(PWM_Cmd(motor_force[3]))			
	msg_override.channels[4] = np.uint(PWM_Cmd(motor_force[4]))		
	msg_override.channels[5] = np.uint(PWM_Cmd(motor_force[5]))		
	msg_override.channels[6] = np.uint(PWM_Cmd(motor_force[6]))	
	msg_override.channels[7] = np.uint(PWM_Cmd(motor_force[7]))	
	print("msg_override.channels[0]", msg_override.channels[0])
	print("msg_override.channels[1]", msg_override.channels[1])
	print("msg_override.channels[2]", msg_override.channels[2])
	print("msg_override.channels[3]", msg_override.channels[3])
	pub_msg_override.publish(msg_override)


def DoThing(msg):
	
	setOverrideRCIN(1500, 1500, msg.data, 1500, 1500, 1500)

def PI_Controller_With_Comp(x_desired, x_real, K_P, K_I, step, I0,g):
    
    e = x_desired - x_real  # Error between the real and desired value
    P = K_P * e                          #Proportional controller 
    I = I0 + K_I * e * step              #Integral controller
    # Tau = P + g
    Tau = P + I + g                      #Output of the PID controller 
    I0 = I                               #Update the initial value of integral controller 
    
    return -Tau, I0

def PIDControlCallback(pid_effort, floatability = 0):
	thrust_req = floatability + pid_effort.data
	m = 76
	c = 1532
	pwm = int(m*thrust_req/4) + c
	setOverrideRCIN(1500, 1500, pwm, 1500, 1500, 1500)

def PWM_Cmd(thrust_req):
    if (thrust_req > 0 and thrust_req > abs(10**(-2))):
        m = 86.93393326839376   # Slope of the positive force linear function
        b = 1536
    elif (thrust_req < 0 and thrust_req < abs(10**(-2))):
        m = 110.918185437553874  # Slope of the negtaive force linear function
        b = 1464
    else:
        m = 0
        b = 1500
    PWM = int(m * thrust_req) + b
    if PWM > Vmax_mot:
        PWM = Vmax_mot
    if PWM < Vmin_mot:
        PWM = Vmin_mot
    return PWM

def force_Cmd(pwm):
    if (pwm > 1500) :
        m = 0.011502988101466315  # Slope of the positive force linear function
        b = -17.66858972385226
    elif (pwm < 1500 ):
        m = 0.009015654160363025 # Slope of the negtaive force linear function
        b = -13.198917690771468
    else:
        m = 0
        b = 0
    force = int(m * pwm) + b
    if force >= 50*4:
        force = 50*4 
    if force <= -40*4:
        force = 0
    return force

def EnableDepthCallback(msg):
	global counter 
	global enable_depth
	global init_p0
	counter = 0
	enable_depth = True
	init_p0 = True

def subscriber():
	rospy.Subscriber("joy", Joy, joyCallback)
	rospy.Subscriber("cmd_vel", Twist, velCallback)
	rospy.Subscriber("mavros/imu/data", Imu, OdoCallback)
	rospy.Subscriber("mavros/imu/water_pressure", FluidPressure, PressureCallback)
	rospy.Subscriber("pid/depth/control_effort", Float64, PIDControlCallback)
	rospy.Subscriber("enable_depth", Empty, EnableDepthCallback)
	rospy.Subscriber("do/thing", Int16, DoThing)
	rospy.spin() # Execute subscriber in loop


if __name__ == '__main__':
	armDisarm(False) # Not automatically disarmed at startup
	rospy.init_node('autonomous_MIR', anonymous=False)
	# config = get_frame_config()
	# # set_frame_config_custom_8_motors()
	# set_frame_config_normal_6DOF()	
	pub_msg_override = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=10, tcp_nodelay=True)
	pub_angle_degre = rospy.Publisher('angle_degree', Twist, queue_size=10, tcp_nodelay=True)
	pub_depth = rospy.Publisher('pid/depth/state', Float64, queue_size=10, tcp_nodelay=True)
	subscriber()







