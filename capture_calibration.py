import pybullet as p
import time
import numpy as np
import math
from datetime import datetime
import pybullet_data
import rospy
import cv2
from sensor_msgs.msg import JointState
import struct
from scipy.io import savemat
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import threading
joint_states = [0.000 ,-0.785 ,0.000 ,-2.356 ,0.000 ,1.571 ,1.585];
rp =  [0.000 ,-0.785 ,0.000 ,-2.356 ,0.000 ,1.571 ,1.585];
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1];
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
ll = [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973]
ul = [2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973]

near = 0.01
far = 1000
def callback(data):
	global joint_states
	joint_states = data.position
	
	#print(data)
	print(joint_states)

def convert_depth_frame_to_pointcloud(depth_image):
	camera_intrinsics ={"fx":554.2563,"ppx": 320,"fy":415.6922,"ppy":240}
	[height, width] = depth_image.shape
	nx = np.linspace(0, width-1, width)
	ny = np.linspace(0, height-1, height)
	u, v = np.meshgrid(nx, ny)
	x = (u.flatten() - camera_intrinsics["ppx"])/camera_intrinsics["fx"]
	y = (v.flatten() - camera_intrinsics["ppy"])/camera_intrinsics["fy"]

	z = depth_image.flatten() / 1000;
	x = np.multiply(x,z)
	y = np.multiply(y,z)

	x = x[np.nonzero(z)]
	y = y[np.nonzero(z)]
	z = z[np.nonzero(z)]
	return x, y, z

 
def getCameraImage(cam_pos,cam_orn):
	fov = 60
	aspect = 640/480
	angle = 0.0;
	q = p.getQuaternionFromEuler(cam_orn)
	cam_orn = np.reshape(p.getMatrixFromQuaternion(q ),(3,3));
	view_pos = np.matmul(cam_orn,np.array([-0.001,0,0.0]).T)
	view_pos = np.array(view_pos+cam_pos)
	view_matrix = p.computeViewMatrix([cam_pos[0],cam_pos[1],cam_pos[2]], view_pos, [0,0,1])
	projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	images = p.getCameraImage(640,
					480,
					view_matrix,
					projection_matrix,
					shadow=False,
					renderer=p.ER_BULLET_HARDWARE_OPENGL)
	return images
if __name__ == "__main__":
	clid = p.connect(p.SHARED_MEMORY)
	if (clid < 0):
		p.connect(p.GUI)
		#p.connect(p.SHARED_MEMORY_GUI)

	p.setAdditionalSearchPath(pybullet_data.getDataPath())

	#p.loadURDF("plane.urdf", [0, 0, -1.0])
	tableId=p.loadURDF("shelfandtable/shelfandtable.urdf", [0, 0, 0.0])
	
	d435Id = p.loadURDF("d435/d435.urdf", [0, 0, 0.0])
	p.resetBasePositionAndOrientation(d435Id, [0.7, 0.5, 0.5],p.getQuaternionFromEuler([0,-math.pi+math.pi/4,-math.pi/4]))

	pandaId = p.loadURDF("Panda/panda.urdf", [0, 0, 0])
	p.resetBasePositionAndOrientation(pandaId, [0, 0, 0], [0, 0, 0, 1])
	cid = p.createConstraint(tableId, -1, pandaId, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0., 0., 0],[0, 0, 0, 1])
	x_Id = p.addUserDebugParameter("x", 0, 1, 0.4)
	y_Id = p.addUserDebugParameter("y", -1, 1, 0.0)
	z_Id = p.addUserDebugParameter("z", 0, 2, 0.589)
	roll_Id = p.addUserDebugParameter("roll", -math.pi*2, math.pi*2, 0.0)
	pitch_Id = p.addUserDebugParameter("pitch", -math.pi*2, math.pi*2, -math.pi)
	yaw_Id = p.addUserDebugParameter("yaw", -math.pi*2, math.pi*2, math.pi/2)

	pandaEndEffectorIndex = 6
	numJoints = 7	
	t = 0
	capture_count = 0;
	save_count = 0;
	for i in range(numJoints):
			p.resetJointState(pandaId, i, joint_states[i])
	images_list = np.array([]);
	poses_list = np.array([]);
	jointPoses_list = np.array([]);
	
	while 1:
		events = p.getKeyboardEvents()	
		key=0;
		for key in events:
			key=key;
			if key==32:
				capture_count =capture_count+1;

		x = p.readUserDebugParameter(x_Id)
		y = p.readUserDebugParameter(y_Id)
		z = p.readUserDebugParameter(z_Id)
		roll = p.readUserDebugParameter(roll_Id)
		pitch = p.readUserDebugParameter(pitch_Id)
		yaw = p.readUserDebugParameter(yaw_Id)
		
		
		t = t + 0.001
		pos = [x, y,z]
		orn = p.getQuaternionFromEuler([roll,pitch,yaw])
		jointPoses = p.calculateInverseKinematics(pandaId, pandaEndEffectorIndex, pos, orn, ll, ul,jr, rp)


		d435pos, d435orn = p.getBasePositionAndOrientation(d435Id)
		d435quat = d435orn
		d435orn =  p.getEulerFromQuaternion(d435orn)
		for i in range(numJoints):
			p.resetJointState(pandaId, i, jointPoses[i])
		image = getCameraImage(d435pos,d435orn)

		depth_img = np.array(image[3],dtype=np.float)
		depth_img = far * near / (far - (far - near) * depth_img)
		
		color_img = image[2]
		eef_pose = np.array([x,y,z,roll,pitch,yaw])
		jointPoses = np.array(jointPoses)
		if capture_count==3:
			print(str(save_count)+"th image saved.")
			if save_count==0:
				images_list=np.array(color_img,dtype=np.uint8);
				s = color_img.shape
				images_list = np.reshape(images_list,(s[0],s[1],s[2],1))

				s2 = eef_pose.shape
				poses_list = np.reshape(eef_pose,(s2[0],1))

				s3 = jointPoses.shape
				jointPoses_list = np.reshape(jointPoses,(s3[0],1))

				save_count = save_count+1
				capture_count=0;

				continue
			s = color_img.shape
			color_img = np.reshape(color_img,(s[0],s[1],s[2],1))
			s2 = eef_pose.shape
			eef_pose = np.reshape(eef_pose,(s2[0],1))
			s3 = jointPoses.shape
			jointPoses = np.reshape(jointPoses,(s3[0],1))

			images_list=np.append(images_list,color_img,axis=3);
			poses_list=np.append(poses_list,eef_pose,axis=1);
			jointPoses_list=np.append(jointPoses_list,jointPoses,axis=1);


			print(images_list.shape)
			print(poses_list.shape)
			print(jointPoses_list.shape)
			
			save_count = save_count+1

			#savedic = {"image": color_img, "pose": [x,y,z,roll,pitch,yaw],"jointPoses":jointPoses}
			#savemat(str(save_count)+".mat", savedic)
			#save_count = save_count+1;
			capture_count=0;
		if key==65309:
			savedic = {"image": images_list, "pose": poses_list,"jointPoses":jointPoses_list}
			savemat("./matlab_files/data.mat", savedic)
			break;
			
			
		p.stepSimulation()
	print("\n\n\n\n\n\n")
	print("===========================DATA WAS SAVED=================================");
	print("\n\n\n\n\n\n")
	p.disconnect()
