# Importing all the necessary libraries
import pybullet as p
import time
import pybullet_data
import numpy as np
import os

curr_pwd = os.getcwd()

# Update the robot wheel velocities as per the fraction of rays colliding with objects
def update_wheel_vel(fraction):
	if fraction < 0.4:
		fraction = 0

	left_vel = 4 + fraction
	right_vel = 4 - fraction
	return left_vel, right_vel

def get_key_pressed():
	pressed_keys = []
	events = p.getKeyboardEvents()
	key_codes = events.keys()
	for key in key_codes:
		pressed_keys.append(key)

	return pressed_keys

# Connecting with the GUI interface of pybullet to visualize all the simulations
physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version

# Getting the data path of pybullet inbuilt 
data_path = pybullet_data.getDataPath()
p.setGravity(0,0,-9.81)			# Setting the gravity of the environment

# Setting the starting position and orientation of robot (turtlebot3 in our case)
startPos = [0,0,0.1]
startOrientation = p.getQuaternionFromEuler([0,0,0])

# Loading the URDF of the robot
robot_id = p.loadURDF(curr_pwd + "/packages/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro", startPos, startOrientation)

planeId = p.loadURDF(data_path + "/plane.urdf")
soccerball1 = p.loadURDF(data_path + "/soccerball.urdf", [0.9, 0.1, 0.2], globalScaling=0.35)
soccerball2 = p.loadURDF(data_path + "/soccerball.urdf", [3.4, -1.6, 0.2], globalScaling=0.35)
cube1 = p.loadURDF(data_path + "/cube.urdf", [2.5, -0.8, 0.3], globalScaling=0.5)

# p.getQuaternionFromEuler([0,0,0])
cube_angle = np.pi/3
cube2 = p.loadURDF(data_path + "/cube.urdf", [3.55, -3.1, 0.3], p.getQuaternionFromEuler([0,0,cube_angle]), globalScaling=0.5)
cube3 = p.loadURDF(data_path + "/cube.urdf", [3.3, -3.71, 0.3], p.getQuaternionFromEuler([0,0,cube_angle]), globalScaling=0.5)
cube4 = p.loadURDF(data_path + "/cube.urdf", [3.1, -4.22, 0.3], p.getQuaternionFromEuler([0,0,cube_angle]), globalScaling=0.5)
duck1 = p.loadURDF(data_path + "/duck_vhacd.urdf", [3.45, -2.2, 0.2], globalScaling=5)


#set the center of mass frame (loadURDF sets base link frame) startPos/Orn
p.resetBasePositionAndOrientation(robot_id, startPos, startOrientation)

num_joints = p.getNumJoints(robot_id)

left_wheel_joint_id = 1
right_wheel_joint_id = 2
    
left_wheel_vel = 4
right_wheel_vel = 4
p.setJointMotorControl2(robot_id, left_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity = left_wheel_vel, force = 5)
p.setJointMotorControl2(robot_id, right_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity = right_wheel_vel, force = 5)

left_vel = 0
right_vel = 0

for i in range (20000):
	p.stepSimulation()
	keys_pressed_list = get_key_pressed()
	# print(len(keys_pressed_list))
	while len(keys_pressed_list) > 0:
		key = keys_pressed_list.pop()
		if key == 65297:	# Up
			left_vel = 4
			right_vel = 4

		elif key == 65298:	# Down
			left_vel = 0
			right_vel = 0

		elif key == 65296:	# Right
			left_vel = 4
			right_vel = 2

		elif key == 65295:	# Left
			left_vel = 2
			right_vel = 4

	# left_wheel_vel, right_wheel_vel = update_wheel_vel(fraction)
	p.setJointMotorControl2(robot_id, left_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity = left_vel, force = 5)
	p.setJointMotorControl2(robot_id, right_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity = right_vel, force = 5)
	
	time.sleep(1./240.)

p.disconnect()
