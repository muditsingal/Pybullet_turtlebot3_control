# Importing all the necessary libraries
import pybullet as p
import time
import pybullet_data
import numpy as np
import os

curr_pwd = os.getcwd()

# Set up the lidar parameters
lidar_range = 3.5 							# 3.5 meters of range
lidar_resolution = 360						# number of rays
lidar_angles = np.linspace(-np.pi/2, np.pi/2, lidar_resolution)		# Angles of lidar going from -90 degrees to +90 degrees

# Lidar ray starting and ending points as required by pybullet
lidar_ray_from = np.zeros((lidar_resolution, 3))
lidar_ray_to = np.zeros((lidar_resolution, 3))

# Function to update the lidar properties - starting and ending points of lidar rays as per the current position of robot's base
def update_lidar_props(base_id):
	# Using global parameters to avoid unnecessary variable repetition
	global lidar_link_pos, lidar_link_orn, lidar_range, lidar_angles
	global lidar_ray_from, lidar_ray_to
	global ray_color

	# Get the current position and orientation of object with id - base_id
	lidar_link_pos, lidar_link_orn = p.getBasePositionAndOrientation(base_id)

	# Update the lidar ray starting and ending points
	for i, angle in enumerate(lidar_angles):
	    lidar_ray_from[i, :] = lidar_link_pos + np.array([0,0,0.1])
	    lidar_ray_to[i, :] = lidar_link_pos + lidar_range * np.array([np.cos(angle), np.sin(angle), 0.1])

	return lidar_ray_from, lidar_ray_to

# Update the robot wheel velocities as per the fraction of rays colliding with objects
def update_wheel_vel(fraction):
	if fraction < 0.4:
		fraction = 0

	left_vel = 4 + fraction
	right_vel = 4 - fraction
	return left_vel, right_vel


# Connecting with the GUI interface of pybullet to visualize all the simulations
physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
# physicsClient = p.connect(p.DIRECT)


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
# sam = p.loadURDF(data_path + "/samurai.urdf", [1.9, 0.2, 0.2])



#set the center of mass frame (loadURDF sets base link frame) startPos/Orn
p.resetBasePositionAndOrientation(robot_id, startPos, startOrientation)

num_joints = p.getNumJoints(robot_id)
print("\nNumber of joints in robot are: ", num_joints)


# Print joint information if required for debugging purposes
for joint_idx in range(num_joints):
	joint_info = p.getJointInfo(robot_id, joint_idx)
	print("For joint id {}, joint info is {}".format(joint_idx, joint_info))

left_wheel_joint_id = 1
right_wheel_joint_id = 2
lidar_joint_id = 5
lidar_link_id = 5
ray_color = [0,0,1]


# Print link information if required for debugging purposes
for link_idx in range(num_joints):
	print("For link id: {} link info is: {}".format(link_idx, p.getLinkState(robot_id, link_idx)))

    

lidar_ray_from, lidar_ray_to = update_lidar_props(robot_id)
lidar_data = p.rayTestBatch(lidar_ray_from, lidar_ray_to)

left_wheel_vel = 4
right_wheel_vel = 4
p.setJointMotorControl2(robot_id, left_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity = left_wheel_vel, force = 5)
p.setJointMotorControl2(robot_id, right_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity = right_wheel_vel, force = 5)


for i in range (20000):
	p.stepSimulation()
	fraction = 0
	detected_poses = np.zeros(3)
	n_rays = 0
	# lidar_ray_from, lidar_ray_to = update_lidar_props(robot_id)

	for i, obj_id in enumerate(lidar_data):
		if(obj_id[0] != -1 and i > 160 and i < 200):
			fraction += obj_id[2]
			n_rays += 1

	if n_rays != 0:
		fraction = 1 - (fraction / n_rays)
		print("Obstacle detected with rays ", n_rays)

	left_wheel_vel, right_wheel_vel = update_wheel_vel(fraction)
	p.setJointMotorControl2(robot_id, left_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity = left_wheel_vel, force = 5)
	p.setJointMotorControl2(robot_id, right_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity = right_wheel_vel, force = 5)

	lidar_data = p.rayTestBatch(lidar_ray_from, lidar_ray_to, parentObjectUniqueId=robot_id)

	
	time.sleep(1./240.)

	

p.disconnect()

# objectUniqueId, linkIndex, hitFraction, hitPosition, hitNormal
# (-1, -1, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))