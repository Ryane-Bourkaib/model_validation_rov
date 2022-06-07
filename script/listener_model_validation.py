#!/usr/bin/env python
from __future__ import division 
import glob
import math
from os import kill
import string
import numpy as np
from yaml import FlowEntryToken
import rospy
import tf
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from waterlinked_a50_ros_driver.msg import DVL
from waterlinked_a50_ros_driver.msg import DVLBeam
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist
from autonomous_rov.msg import pwm
from PI_Controller import*
from Parameters import parameters
from torque_rov import torque_rov
from generate_trajectory import quintic_Trajectory
from alpha_beta_gamma_filter import alpha_beta_gamma_filter
from SM_Controller import SM_Controller
from brping import Ping1D
import time
import sys
import argparse
import stopp

# ---------- Global Variables ---------------
global enable_depth
global init_p0
global depth_p0
global depth_wrt_startup
global test_Model
set_mode = [0]*3
angle_wrt_startup = [0]*3
depth_wrt_startup = 0
depth_p0 = 0
pinger_confidence = 0
pinger_distance = 0
Vmax_mot = 1900
Vmin_mot = 1100

# ---------- Conditions --------------------------
init_a0 = True      
init_p0 = True
test_Model = True
enable_depth = True  # don't Publish the depth data until asked
enable_ping = True
arming = False
set_mode[0] = True   # manual mode
set_mode[1] = False  # automatic mode (without correction)
set_mode[2] = False  # correction mode

# ---------- Linear/Angular velocity ------------
u = 0                # linear surge velocity
v = 0                # linear sway velocity
w = 0                # linear heave velocity
p = 0                # angular  velocity along x
q = 0                # angular velocity along y
r = 0                # angular velocity along z

# ---------- joystick callback ---------------
def joyCallback(data):  
    global initial_time_bool 
    global test_Model
    global depth_p0
    global arming
    global set_mode
    global init_a0
    global init_p0
    global mode
  
    # Joystick buttons
    btn_arm = data.buttons[7]               # start button
    btn_disarm = data.buttons[6]            # back button
    btn_manual_mode = data.buttons[3]       # Y button
    btn_automatic_mode = data.buttons[2]    # X button
    btn_corrected_mode = data.buttons[0]    # A button

    # Disarming when Back button is pressed
    if (btn_disarm == 1 and arming == True):
        arming = False
        armDisarm(arming)

    # Arming when Start button is pressed
    if (btn_arm == 1 and arming == False):
        arming = True
        armDisarm(arming)

    # Switch manual/auto and correction mode
    if (btn_manual_mode and not set_mode[0]):
        set_mode[0] = True
        set_mode[1] = False
        set_mode[2] = False
        test_Model = False
        rospy.loginfo("Mode manual")
    if (btn_automatic_mode and not set_mode[1]):
        set_mode[0] = False
        set_mode[1] = True
        set_mode[2] = False
        test_Model = False
        rospy.loginfo("Mode automatic")
    if (btn_corrected_mode and not set_mode[2]):
        init_a0 = True
        init_p0 = True
        initial_time_bool = True
        set_mode[0] = False
        set_mode[1] = False
        set_mode[2] = True
        test_Model = True
        rospy.loginfo("Mode correction")

# ---------- Arm/disarm motors ---------------
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


# ---------- Velocity callback ---------------
def velCallback(cmd_vel):  
    global set_mode
    # Only continue if manual_mode is enabled
    if (set_mode[1] or set_mode[2]):
        return
    # Extract cmd_vel message
    roll_left_right = mapValueScalSat(cmd_vel.angular.x)
    yaw_left_right = mapValueScalSat(-cmd_vel.angular.z)
    ascend_descend = mapValueScalSat(cmd_vel.linear.z)
    forward_reverse = mapValueScalSat(cmd_vel.linear.x)
    lateral_left_right = mapValueScalSat(-cmd_vel.linear.y)
    pitch_left_right = mapValueScalSat(cmd_vel.angular.y)

    setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend,
                    yaw_left_right, forward_reverse, lateral_left_right)


# ---------- Pinger callback ---------------
def pingerCallback(data):
    print("pressure")

    global pinger_confidence
    global pinger_distance
    
    # Extract the pinger data 
    if enable_ping == True:
       pinger_distance = data.data[0] * 10**(-3) # distance (m)
       pinger_confidence = data.data[1]
      
    # # publish the distance(m)
    #    distance = Float64()
    #    distance.data = pinger_distance
    #    pub_distance.publish(distance)

# ---------- IMU callback ---------------      
def OdoCallback(data):
    global angle_roll_a0
    global angle_pitch_a0
    global angle_yaw_a0
    global angle_wrt_startup
    global init_a0
    global p
    global q
    global r

    # Extract the IMU data : orientation and anguler velocity
    orientation = data.orientation
    angular_velocity = data.angular_velocity

    # Extract the euler angles
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

    # Convert to degree
    angle_wrt_startup[0] = (angle_roll - angle_roll_a0) * 180/math.pi 
    angle_wrt_startup[1] = (angle_pitch - angle_pitch_a0 ) * 180/math.pi
    angle_wrt_startup[2] = (angle_yaw - angle_yaw_a0) * 180/math.pi
    
    # publish the angle values (degree)
    angle = Twist()
    angle.angular.x = angle_wrt_startup[0]
    angle.angular.y = angle_wrt_startup[1]
    angle.angular.z = angle_wrt_startup[2]
    pub_angle_degre.publish(angle)

    # Extract the angular velocities (degree/s)
    p = angular_velocity.x * 180/math.pi
    q = angular_velocity.y * 180/math.pi
    r = angular_velocity.z * 180/math.pi
    
    # publish the angular velocities (degree/s)
    ang_vel = Twist()
    ang_vel.angular.x = p
    ang_vel.angular.y = q
    ang_vel.angular.z = r
    pub_angular_velocity.publish(ang_vel)




# ---------- Pressure callback ---------------     
# To validate the dynamic model we test diffrent motion (surge, sway and yaw motion) in BO   
# Initialisation 
rho = 1000.0           # water density
gravity = 9.80665      # gravity
I0_psi= 0              # initial integral term for yaw controller 
e0_psi = 0             # initial yaw error 
yaw_d = 0              # desired yaw angle 
e0_z = 0               # initial integral term for depth controller 
I0_z = 0               # initial error for depth 
depth_d = 0.3          # desired depth
w_d = 0                # desired depth velocity 
flotability = 0.19  
initial_time_bool = True 
current_time = 0       # current time (s) 
initial_time = 0
dt = 0.03
v_e0 = 0
x_e0 = 0


def PressureCallback(data):
    
    global depth_wrt_startup
    global initial_time_bool
    global enable_depth
    global initial_time
    global current_time
    global flotability
    global test_Model
    global depth_p0
    global init_p0
    global w_d 
    global dt
    global yaw_d
    global depth_d
    global I0_psi
    global e0_psi
    global e0_z
    global I0_z
    global v_e0 
    global x_e0 
    global u
    global v
    global w
    global r
    
    # Desired trajectory for surge motion
    xd_init = 0             # initial position (m) 
    xd_final = 1            # final position (m) 
    t_final_surge = 6.4
    # Desired trajectory for surge motion
    yd_init = 0            # final position (m) 
    yd_final = 1           # final position (m) 
    t_final_sway = 6
    # Desired trajectory for surge motion
    psid_init = 0          # initial position (m) 
    psid_final = 1.5708    # final position (90 degree)
    t_final_yaw = 3.5  

    if(enable_depth):  
        pressure = data.fluid_pressure
        if (init_p0):
            # 1st execution, init
            depth_p0 = (pressure - 101300)/(rho* gravity)
            init_p0 = False

        depth_wrt_startup = (pressure - 101300)/(rho * gravity) - depth_p0
        
        if(test_Model):
            if initial_time_bool == True:
                # 1st execution, init : calculate the initial time (s)
                initial_time = rospy.Time.now().to_sec()       
                initial_time_bool = False
            else:
                # calculate the current time(s)
                current_time = rospy.Time.now().to_sec() - initial_time 

            
            if motion == "heave" : 
                # Depth control : with pid or SM 
                #heave_force = PID_Controller(depth_d, depth_wrt_startup, Kp_z, Ki_z, Kd_z, e0_z, I0_z, dt, flotability, r = None)
                
                # calculate the real speed with alpha beta gamma filter 
                real_Speed, x_e = alpha_beta_gamma_filter(x_e0, v_e0, 0, depth_wrt_startup, 0.45, 0.1, dt)
                #print("real_Speed", real_Speed)
                v_e0 = real_Speed
                x_e0 = x_e
                heave_force = - SM_Controller(depth_d, depth_wrt_startup, w_d, real_Speed, rhoo, K, phi)
                # Send PWM commands to motors
                heave_pwm = PWM_Cmd(heave_force)
                setOverrideRCIN(1500, 1500, heave_pwm, 1500, 1500, 1500)

            if motion == "surge" : # "moving forward" 
                
                # Control the heading to ensure that the robot move in a straight line (P controller)
                yaw_torque = yaw_Controller(yaw_d, angle_wrt_startup[2], r, Kp_psi, Ki_psi, Kd_psi, I0_psi, dt)
                # Send PWM commands to motors
                yaw_pwm = mapValueScalSat(yaw_torque)
            
                # Desired position/velocity and acceleration 
                xd, ud, ud_d = quintic_Trajectory(xd_init , xd_final, current_time, t_final_surge) 
                
                # Calculate the necessary torque to move forward (using dynamic model)
                Pos = np.array([xd, 0, 0, 0, 0, 0])         # position vector ([surge_pos, sway_pos, heave_pos, roll_pos, pitch_pos, yaw_pos ])
                Vel_B = np.array([ud, 0, 0, 0, 0, 0])       # velocity vector ([surge_vel, sway_vel, heave_vel, roll_vel, pitch_vel, yaw_vel ])
                Acc_B = np.array([ud_d, 0, 0, 0, 0, 0])     # acceleration vector ([surge_acc, sway_acc, heave_acc, roll_acc, pitch_acc, yaw_acc ])
                torque = torque_rov(Acc_B, Pos, Vel_B)
                # Send PWM commands to motors
                surge_pwm = PWM_Cmd(torque[0])
                setOverrideRCIN(1500, 1500, 1500, yaw_pwm ,surge_pwm, 1500)
                
                # Publish surge force
                force = Float64()
                force.data = torque[0]
                pub_force.publish(force)

            if motion == "sway": #"sliding motion"
                # Control the heading to ensure that the robot move in a straight line (P controller)
                yaw_torque = yaw_Controller(yaw_d, angle_wrt_startup[2], r, Kp_psi, Ki_psi, Kd_psi, I0_psi, dt)
                # Send PWM commands to motors
                yaw_pwm = mapValueScalSat(yaw_torque)
            
                # Desired position/velocity and acceleration :
                yd, vd, vd_d = quintic_Trajectory(yd_init, yd_final, current_time, t_final_sway) 
                
                # Calculate the necessary torque to slide (using dynamic model)
                Pos = np.array([0, yd, 0, 0, 0, 0])
                Vel_B = np.array([0, vd, 0, 0, 0, 0])
                Acc_B = np.array([0,vd_d, 0, 0, 0, 0])
                torque = torque_rov(Acc_B, Pos, Vel_B)
                sway_pwm = PWM_Cmd(torque[1])
                setOverrideRCIN(1500, 1500, 1500, yaw_pwm , 1500, sway_pwm)
                
                # publish sway force
                force = Float64()
                force.data = torque[1]
                pub_force.publish(force)
            
            if motion == "yaw": #"heading motion"
                # Desired position/velocity and acceleration :
                psid, rd, rd_d = quintic_Trajectory(psid_init, psid_final, current_time, t_final_yaw) 
                
                # Calculate the necessary moment to turn around z (using dynamic model)
                Pos = np.array([0, 0, 0, 0, 0, psid])
                Vel_B = np.array([0, 0, 0, 0, 0, rd])
                Acc_B = np.array([0, 0, 0, 0, 0, rd_d])
              
                # Send PWM commands to motors
                torque = torque_rov(Acc_B, Pos, Vel_B)
                yaw_pwm = PWM_Cmd(torque[5])
                setOverrideRCIN(1500, 1500, 1500, yaw_pwm , 1500, 1500)
    
                # Publish yaw force
                force = Float64()
                force.data = torque[5]
                pub_force.publish(force)
            
            
          
            # publish depth_wrt_startup data (m)
            msg = Float64()
            msg.data = depth_wrt_startup
            pub_depth.publish(msg)

            # publish distance data (m)
            distance = Float64()
            distance.data = pinger_distance
            pub_distance.publish(distance)

           

    # Only continue if manual_mode is disabled
    if (set_mode[0]):
        return
    elif (set_mode[1]):
        # Only continue if automatic_mode is enabled
        # Define an arbitrary velocity command and observe robot's velocity
        # setOverrideRCIN ( Pitch , Roll , Heave , Yaw ,Surge, Sway)
        setOverrideRCIN(1500, 1500, 1700, 1500, 1500, 1500)
        return


# ---------- DVL callback ---------------     
def DvlCallback(data):
    global set_mode
    global angle_wrt_startup 
    global u
    global v
    global w

    u = data.velocity.x  # Linear velocity along x (surge)
    v = data.velocity.y  # Linear velocity along y (sway)
    w = data.velocity.z  # Linear velocity along z (heave)
    
    # # Extract linear velocity 
    # linear_vel = Twist()
    # linear_vel.linear.x = u
    # linear_vel.linear.y = v
    # linear_vel.linear.z = w
    # pub_linear_vel.publish(linear_vel)


    # Only continue if manual_mode is disabled
    if (set_mode[0]):
        return
    elif (set_mode[1]):
        # Only continue if automatic_mode is enabled
        # Define an arbitrary velocity command and observe robot's velocity
        # setOverrideRCIN ( Pitch , Roll , Heave , Yaw ,Surge, Sway)
        setOverrideRCIN(1500, 1500, 1700, 1500, 1500, 1500)
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

    msg_override.channels[0] = np.uint(channel_pitch)       # pulseCmd[4]--> pitch	
    msg_override.channels[1] = np.uint(channel_roll)        # pulseCmd[3]--> roll 	
    msg_override.channels[2] = np.uint(channel_throttle)    # pulseCmd[2]--> heave 
    msg_override.channels[3] = np.uint( channel_yaw)        # pulseCmd[5]--> yaw		    
  
    msg_override.channels[4] = np.uint(channel_forward)     # pulseCmd[0]--> surge		
    
    msg_override.channels[5] = np.uint(channel_lateral)     # pulseCmd[1]--> sway
    msg_override.channels[6] = 1500
    msg_override.channels[7] = 1500
    
    # print("<3=====D ",msg_override)
    pub_msg_override.publish(msg_override)



# ----------Functions ----------------
# Function used to calculate the necessary PWM of each motor
def PWM_Cmd(thrust_req):
    if (thrust_req > 0 and thrust_req > abs(10**(-2))):
        m = 86.93393326839376   # Slope of the positive force linear function
        b = 1536
    elif (thrust_req < 0 and thrust_req < abs(10**(-2))):
        m = 110.918185437553874 # Slope of the negtaive force linear function
        b = 1464
    else : 
        m = 0
        b = 1500
    PWM = int(m * thrust_req/4) + b 
    if PWM > Vmax_mot:
        PWM = Vmax_mot
    if PWM < Vmin_mot:
        PWM = Vmin_mot
    return PWM

# Function used to enble the depth calback
def EnableDepthCallback(msg):
    global enable_depth
    global init_p0
    global counter
    counter = 0
    enable_depth = True
    init_p0 = True


def subscriber():
    rospy.Subscriber("joy", Joy, joyCallback)
    rospy.Subscriber("cmd_vel", Twist, velCallback)
    rospy.Subscriber("mavros/imu/data", Imu, OdoCallback)
    rospy.Subscriber("mavros/imu/water_pressure",FluidPressure, PressureCallback)
    rospy.Subscriber("/dvl/data", DVL, DvlCallback)
    rospy.Subscriber("distance_sonar", Float64MultiArray, pingerCallback)
    rospy.Subscriber("enable_depth", Empty, EnableDepthCallback)
    rospy.spin()  # Execute subscriber in loop


if __name__ == '__main__':
    #armDisarm(False)  # Not automatically disarmed at startup
    rospy.init_node('autonomous_MIR', anonymous=False)
    pub_msg_override = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=10, tcp_nodelay=True)
    pub_angle_degre = rospy.Publisher( 'angle_degree', Twist, queue_size=10, tcp_nodelay=True)
    pub_depth = rospy.Publisher( 'pid/depth/state', Float64, queue_size=10, tcp_nodelay=True)
    pub_force = rospy.Publisher('force', Float64, queue_size=10, tcp_nodelay=True)
    pub_distance = rospy.Publisher('distance', Float64, queue_size=10, tcp_nodelay=True)
    pub_angular_velocity = rospy.Publisher( 'angular_velocity', Twist, queue_size=10, tcp_nodelay=True)
    pub_linear_vel = rospy.Publisher('linear_velocity', Twist, queue_size=10, tcp_nodelay=True)

    parser = argparse.ArgumentParser(description='model_validation')
    parser.add_argument('--motion', default = "surge")
    parser.add_argument('--Kp_psi', type = float, default = 0)
    parser.add_argument('--Ki_psi', type = float, default = 0)
    parser.add_argument('--Kd_psi', type = float, default = 0)
    
    parser.add_argument('--Kp_z', type = float, default = 0)
    parser.add_argument('--Ki_z', type = float, default = 0)
    parser.add_argument('--Kd_z', type = float, default = 0)

    parser.add_argument('--rhoo0', type = float, default = 0)
    parser.add_argument('--rhoo1', type = float, default = 0)
    parser.add_argument('--K', type = float, default = 0)
    parser.add_argument('--phi', type = float, default = 0)
    
    sys.argv = rospy.myargv(argv=sys.argv)
    args = parser.parse_args()
    
    motion = args.motion
    Kp_psi, Ki_psi, Kd_psi = args.Kp_psi, args.Ki_psi, args.Kd_psi  
    Kp_z, Ki_z, Kd_z = args.Kp_z, args.Ki_z, args.Kd_z  
    rhoo0 = args.rhoo0
    rhoo1 = args.rhoo1
    rhoo = np.array([rhoo0, rhoo1])
    K = args.K
    phi = args.phi  #06   #0

    subscriber()


