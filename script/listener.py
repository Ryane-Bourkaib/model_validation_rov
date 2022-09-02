#!/usr/bin/env python

import rospy
import tf
import math
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String, Float64
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist


# ---------- Global Variables ---------------
Command_Vel = [0]*3
Command_Vel[0] = 0.5	# Cmd along X
Command_Vel[1] = 0	# Cmd along Y
Command_Vel[2] = 0	# Cmd along Z

Error_Vel = [0]*3

angle_wrt_startup = [0]*3
init_a0 = True

depth_wrt_startup = 0
init_p0 = True

Vmax_mot = 1900
Vmin_mot = 1100

arming = False
set_mode = [0]*3
set_mode[0] = True	# Mode manual
set_mode[1] = False	# Mode automatic without correction
set_mode[2] = False	# Mode with correction

init_t0 = True

liste_temps_consignes = [4,6,10]
consignes_yaw_forward = [[1500, 1600], [1550, 1500], [1500, 1400]]
indice_temps = 0

choix_asserv_profondeur = True
choix_ligne_droite = False


def joyCallback(data):
	global arming
	global set_mode
	global init_a0
	global init_p0
	global Sum_Errors_Vel
	global Sum_Errors_angle_yaw
	global Sum_Errors_depth
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
    		except rospy.ServiceException, e:
        		rospy.loginfo("Except arming")
	else:
		rospy.wait_for_service('mavros/cmd/command')
    		try:
        		armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
			armService(0, 400, 0, 0, 0, 0, 0, 0, 0, 0)
			rospy.loginfo("Disarming Succeeded")
    		except rospy.ServiceException, e:
			rospy.loginfo("Except disarming")	



def velCallback(cmd_vel):
	global set_mode
	# Only continue if manual_mode is enabled
	if (set_mode[1] or set_mode[2]):
		return

	# Extract cmd_vel message
	roll_left_right 	= mapValueScalSat(cmd_vel.angular.x)
	yaw_left_right 		= mapValueScalSat(-cmd_vel.angular.z)
	ascend_descend 		= mapValueScalSat(cmd_vel.linear.z)
	forward_reverse 	= mapValueScalSat(cmd_vel.linear.x)
	lateral_left_right 	= mapValueScalSat(-cmd_vel.linear.y)
	pitch_left_right 	= mapValueScalSat(cmd_vel.angular.y)

	setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right)


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



def Asservissement_profondeur(data):
	global depth_p0
	global depth_wrt_startup
	global init_p0
	global integral
	global init_t0 
	global t0
	rho = 1000.0 # 1025.0 for sea water
	g = 9.80665


	pressure = data.fluid_pressure

	if (init_p0):
		# 1st execution, init
		depth_p0 = (pressure - 101300)/(rho*g)
		init_p0 = False
		integral = 0
    
    	if (init_t0):
        	init_t0 = False
        	t0 = rospy.Time.now().to_sec()

    	t_now = rospy.Time.now().to_sec()
	dt = t_now - t0
	print('dt', dt)

    	val_temps = Float64(0.0)
    	val_temps.data = t_now
    	pub_temps.publish(val_temps)
    
    

	depth_wrt_startup = (pressure - 101300)/(rho*g) - depth_p0
	
	val_profondeur = Float64(0.0)
	val_profondeur.data = depth_wrt_startup
	pub_profondeur.publish(val_profondeur)
	
	
	depth_ref = 0.5
	Kp = 30
	Ki = 0.05
	
	#setup depth servo control here
	error = depth_ref-depth_wrt_startup # z~ or epsilon
	integral += error * dt

	correction_T = Kp*(error) + 2.5 + Ki * integral # gravity/flotability compensation
	print("depth ref", depth_ref)
	print("depth_wrt_startup",depth_wrt_startup)
	print("depth_ref-depth_wrt_startup", depth_ref-depth_wrt_startup)
	
	if correction_T > 0:
	 	Correction_depth = 7.07*correction_T + 1528
	else:
		Correction_depth = 9.063*correction_T + 1472
	print("tau", correction_T)
	print("error depth:", Correction_depth)

	

	#Correction_depth = 1500	
	

	# Send PWM commands to motors
	Correction_depth = int(Correction_depth)
	setOverrideRCIN(1500, 1500, Correction_depth, 1500, 1500, 1500)
	t0 = t_now
	print(dt)

def Ligne_droite():
	global t0
	global init_t0
	global set_mode

	global liste_temps_consignes
	global indice_temps
	global consignes_yaw_forward
	
	t_now = rospy.Time.now().to_sec()-t0
	
	if (t_now>liste_temps_consignes[indice_temps]):
		indice_temps += 1

	if indice_temps<len(liste_temps_consignes):
		cons_yaw = consignes_yaw_forward[indice_temps][0] 
		cons_fw = consignes_yaw_forward[indice_temps][1]
		setOverrideRCIN(1500, 1500, 1500, cons_yaw, cons_fw, 1500)
	
	else:		
		indice_temps = 0
		init_t0 = True
		set_mode[0] = True
		set_mode[1] = False
		set_mode[2] = False		
		Dont_move()
		



def Dont_move():
	setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)


def PressureCallback(data):
	global init_t0
	global t0

	# Only continue if manual_mode is disabled
	if arming:
		if (set_mode[0]):
			#print("Je suis ici")
			return

		elif (set_mode[1]):
			# Only continue if automatic_mode is enabled
			# Define an arbitrary velocity command and observe robot's velocity
			setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
			return

		else:
			
			#print("Je repasse")
			if (init_t0):
        			init_t0 = False
        			t0 = rospy.Time.now().to_sec()

			if choix_asserv_profondeur:
				Asservissement_profondeur(data)

			elif choix_ligne_droite:
				Ligne_droite()

	    		else:
				Dont_move()
	
	else:
		init_t0 = True
		set_mode[0] = True
		set_mode[1] = False
		set_mode[2] = False




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
	msg_override.channels[0] = channel_pitch		#pulseCmd[4]  # pitch		Tangage
	msg_override.channels[1] = channel_roll			#pulseCmd[3]  # roll 		Roulis
	msg_override.channels[2] = channel_throttle		#pulseCmd[2]  # up/down		Montee/descente
	msg_override.channels[3] = channel_yaw			#pulseCmd[5]  # yaw		Lace
	msg_override.channels[4] = channel_forward		#pulseCmd[0]  # forward		Devant/derriere
	msg_override.channels[5] = channel_lateral		#pulseCmd[1]  # lateral		Gauche/droite
	msg_override.channels[6] = 1500
	msg_override.channels[7] = 1500

	pub_msg_override.publish(msg_override)


def subscriber():
	rospy.Subscriber("joy", Joy, joyCallback)
	rospy.Subscriber("cmd_vel", Twist, velCallback)
	rospy.Subscriber("mavros/imu/data", Imu, OdoCallback)
	rospy.Subscriber("mavros/imu/water_pressure", FluidPressure, PressureCallback)
	rospy.spin() # Execute subscriber in loop


if __name__ == '__main__':
	armDisarm(False) # Not automatically disarmed at startup
	rospy.init_node('autonomous_MIR', anonymous=False)
	t0 = rospy.Time.now().to_sec()
	pub_msg_override = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=10, tcp_nodelay=True)
	pub_angle_degre = rospy.Publisher('angle_degree', Twist, queue_size=10, tcp_nodelay=True)
	pub_temps = rospy.Publisher('temps', Float64, queue_size=10, tcp_nodelay=True)
	pub_profondeur = rospy.Publisher('profondeur', Float64, queue_size=10, tcp_nodelay=True)
	subscriber()







