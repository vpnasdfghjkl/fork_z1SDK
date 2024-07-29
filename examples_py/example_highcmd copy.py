import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np
import cv2
print("Press ctrl+\ to quit process.")


import pandas as pd
import threading
from datetime import datetime
import os

np.set_printoptions(precision=3, suppress=True)
arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
arm.loopOn()

# Create a VideoCapture object to access the camera
cam_fixed = cv2.VideoCapture(1)
cam_followed = cv2.VideoCapture(0)

# Check if the camera is opened successfully
if not cam_fixed.isOpened() and not cam_followed.isOpened():
    print("Unable to access the camera")
    exit()
    
os.makedirs("dataset/cam/cam_fixed", exist_ok=True)
os.makedirs("dataset/cam/cam_followed", exist_ok=True)
os.makedirs("dataset/joint", exist_ok=True)
'''
def save_joint_positions():
    with open('dataset/joint/joint_positions.txt', 'w') as f:
        while True:
            joint_positions = arm.getJointState()  # 获取机械臂当前关节状态
            f.write(f"{datetime.now()}: {joint_positions}\n")
            f.flush()  # 刷新缓冲区，确保实时写入文件
            time.sleep(0.1)  # 设置记录间隔
def save_camera_images():
    while True:
        ret, frame = cam_fixed.read()
        ret1, frame1 = cam_followed.read()
        if ret:
            cv2.imwrite(f"dataset/cam/cam1/{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.png", frame)
        if ret1:
            cv2.imwrite(f"dataset/cam/cam2/{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.png", frame1)
        time.sleep(0.1)  # 设置记录间隔

# # 启动记录机械臂位置信息的线程
# joint_thread = threading.Thread(target=save_joint_positions)
# joint_thread.daemon = True
# joint_thread.start()
# # 启动记录相机图像的线程
# camera_thread = threading.Thread(target=save_camera_images)
# camera_thread.daemon = True
# camera_thread.start()  
# 启动记录线程

'''
def record_data():
    while True:
        ret, frame = cam_fixed.read()
        ret1, frame1 = cam_followed.read()
        joint_positions = arm.getCurrentState()  #TODO
        if ret:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            cv2.imwrite(f"dataset/cam/cam_fixed/{timestamp}.png", frame)
        if ret1:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            cv2.imwrite(f"dataset/cam/cam_followed/{timestamp}.png", frame1)
        with open('dataset/joint/joint_positions.txt', 'a') as f:
            f.write(f"{datetime.now()}: {joint_positions}\n")
        
        time.sleep(0.05)  # 设置记录间隔，保持与相机的记录频率一致    
         

record_thread = threading.Thread(target=record_data)
record_thread.daemon = True
record_thread.start()

# 从 CSV 文件中读取位姿信息并执行机械臂运动
try:
    arm.labelRun("forward")
    gripper_pos = 0.0
    jnt_speed = 2.0
    
    # 读取 CSV 文件中的位姿信息
    df = pd.read_csv('octo.csv')
    
    for index, row in df.iterrows():
        target_joint_positions = np.array(row.values)
        arm.MoveJ(target_joint_positions, gripper_pos, jnt_speed)
        # time.sleep(2)  # 假设每次移动后等待 2 秒
    
except KeyboardInterrupt:
    print("Process interrupted by user.")
finally:
    # 释放资源
    cam_fixed.release()
    cam_followed.release()
    cv2.destroyAllWindows()
    
# while 1:
#     # Read the csv from teaching trajectory csv file name octo.csv
#     df = pd.read_csv('octo.csv')
#     arm.labelRun("forward")
#     gripper_pos = 0.0
#     jnt_speed = 2.0
#     arm.MoveJ(df.values, gripper_pos, jnt_speed)
    











# 1. highcmd_basic : armCtrlInJointCtrl
arm.labelRun("forward")
arm.startTrack(armState.JOINTCTRL)
jnt_speed = 1.0
for i in range(0, 1000):
    arm.jointCtrlCmd(np.array([0,0,0,-1,0,0,-1]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)

# 2. highcmd_basic : armCtrlByFSM
arm.labelRun("forward")
gripper_pos = 0.0
jnt_speed = 2.0
arm.MoveJ(np.array([0.5,0.1,0.1,0.5,-0.2,0.5]), gripper_pos, jnt_speed)
gripper_pos = -1.0
cartesian_speed = 0.5
arm.MoveL(np.array([0,0,0,0.45,-0.2,0.2]), gripper_pos, cartesian_speed)
gripper_pos = 0.0
arm.MoveC(np.array([0,0,0,0.45,0,0.4]), np.array([0,0,0,0.45,0.2,0.2]), gripper_pos, cartesian_speed)

# 3. highcmd_basic : armCtrlInCartesian
arm.labelRun("forward")
arm.startTrack(armState.CARTESIAN)
angular_vel = 0.3
linear_vel = 0.3
for i in range(0, 1000):
    arm.cartesianCtrlCmd(np.array([0,0,0,0,0,-1,-1]), angular_vel, linear_vel)
    time.sleep(arm._ctrlComp.dt)

arm.backToStart()
arm.loopOff()

