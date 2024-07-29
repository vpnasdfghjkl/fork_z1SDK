import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np

import cv2
import pandas as pd
import threading
from datetime import datetime
import os
import numpy as np
import pyrealsense2 as rs
from matplotlib import pyplot as plt
import argparse

print("Press ctrl+\ to quit process.")

np.set_printoptions(precision=3, suppress=True)
arm = unitree_arm_interface.ArmInterface(hasGripper=True)
armModel = arm._ctrlComp.armModel
arm.setFsmLowcmd()


duration = 1000
lastPos = arm.lowstate.getQ()
targetPos = np.array([0.0, 1.5, -1.0, -0.54, 0.0, 0.0]) #forward
# gripper_q=arm.gripperQd
# print(gripper_q)
init_pose = np.array([0.00369,  0.00357, -0.00592, -0.06833,0.00251,  0.00145]) #forward




def record_data(cam01_rgb_dir, cam01_depth_dir, cam02_rgb_dir,cam02_depth_dir,joint_dir):
    print("Recording data...")
    # 确定图像的输入分辨率与帧率
    resolution_width = 640  # pixels
    resolution_height = 480  # pixels
    frame_rate = 60  # fps

    # 注册数据流，并对其图像
    align = rs.align(rs.stream.color)
    rs_config = rs.config()
    rs_config.enable_stream(rs.stream.depth, resolution_width, resolution_height, rs.format.z16, frame_rate)
    rs_config.enable_stream(rs.stream.color, resolution_width, resolution_height, rs.format.bgr8, frame_rate)
    ### d435i
    #
    rs_config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, frame_rate)
    rs_config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, frame_rate)
    # check相机是不是进来了
    connect_device = []
    for d in rs.context().devices:
        print('Found device: ',
              d.get_info(rs.camera_info.name), ' ',
              d.get_info(rs.camera_info.serial_number))
        if d.get_info(rs.camera_info.name).lower() != 'platform camera':
            connect_device.append(d.get_info(rs.camera_info.serial_number))


    if len(connect_device) < 2:
        print('Registrition needs two camera connected.But got one.')
        exit()

    # 确认相机并获取相机的内部参数
    pipeline1 = rs.pipeline()
    rs_config.enable_device(connect_device[0])
    # pipeline_profile1 = pipeline1.start(rs_config)
    pipeline1.start(rs_config)

    pipeline2 = rs.pipeline()
    rs_config.enable_device(connect_device[1])
    # pipeline_profile2 = pipeline2.start(rs_config)
    pipeline2.start(rs_config)
    
    try:
        while not stop_thread.is_set():
            frames1 = pipeline1.wait_for_frames()
            frames2 = pipeline2.wait_for_frames()
            
            t_s1=frames1.get_timestamp()
            t_s2=frames2.get_timestamp()
            dt1 = datetime.fromtimestamp(t_s1 / 1e3)  
            dt2 = datetime.fromtimestamp(t_s2 / 1e3) 
             
            aligned_frames1 = align.process(frames1)
            aligned_frames2 = align.process(frames2)
            
            color_frame1 = aligned_frames1.get_color_frame()
            depth_frame1 = aligned_frames1.get_depth_frame()
            color_frame2 = aligned_frames2.get_color_frame()
            depth_frame2 = aligned_frames2.get_depth_frame()
            
            depth_frame1 = frames1.get_depth_frame()
            color_frame1 = frames1.get_color_frame()
            depth_frame2 = frames2.get_depth_frame()
            color_frame2 = frames2.get_color_frame()
            
            
            
            # ir_frame_left1 = frames1.get_infrared_frame(1)
            # ir_frame_right1 = frames1.get_infrared_frame(2)
            if not depth_frame1 or not color_frame1:
                continue
            # ir_frame_left2 = frames2.get_infrared_frame(1)
            # ir_frame_right2 = frames2.get_infrared_frame(2)
            if not depth_frame2 or not color_frame2:
                continue
            
            color_image1 = np.asanyarray(color_frame1.get_data())
            depth_image1 = np.asanyarray(depth_frame1.get_data())
            # ir_left_image1 = np.asanyarray(ir_frame_left1.get_data())
            # ir_right_image1 = np.asanyarray(ir_frame_right1.get_data())

            color_image2 = np.asanyarray(color_frame2.get_data())
            depth_image2 = np.asanyarray(depth_frame2.get_data())
            # ir_left_image2 = np.asanyarray(ir_frame_left2.get_data())
            # ir_right_image2 = np.asanyarray(ir_frame_right2.get_data())
            
            depth_colormap1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image1, alpha=0.03), cv2.COLORMAP_JET)
            depth_colormap2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image2, alpha=0.03), cv2.COLORMAP_JET)

            np.save(f"{cam01_rgb_dir}/color_{dt1}.npy", color_image1)
            cv2.imwrite(f"{cam01_rgb_dir}/color_{dt1}.png", color_image1)
            np.save(f"{cam01_depth_dir}/depth_{dt1}.npy", depth_image1)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image1, alpha=0.03), cv2.COLORMAP_JET)
            cv2.imwrite(f"{cam01_depth_dir}/depth_{dt1}.png", depth_colormap1)
            
            np.save(f"{cam02_rgb_dir}/color_{dt2}.npy", color_image2)
            cv2.imwrite(f"{cam02_rgb_dir}/color_{dt2}.png", color_image2)
            np.save(f"{cam02_depth_dir}/depth_{dt2}.npy", depth_image2)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image2, alpha=0.03), cv2.COLORMAP_JET)
            cv2.imwrite(f"{cam02_depth_dir}/depth_{dt2}.png", depth_colormap2)
            
            
            images1 = np.hstack((color_image1, depth_colormap1))

            images2 = np.hstack((color_image2, depth_colormap2))
            cv2.imshow('RealSense1', images1)
            cv2.imshow('RealSense2', images2)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break   
    except KeyboardInterrupt:
        print("Process interrupted by user.")
    finally:
        pipeline1.stop()
        pipeline2.stop()

# main
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='dataset name.')
    parser.add_argument('--dataset', type=str, required=True, help='The name of the dataset.')
    
    current_dir=os.getcwd()
    print("current_dir:",current_dir)
    
    args = parser.parse_args()
    if not os.path.exists(os.path.join(current_dir,"dataset")):
        os.makedirs(os.path.join(current_dir,"dataset"))
    
    cam01_rgb_dir = os.path.join(current_dir, f"dataset/dataset_{args.dataset}/cam01/rgb")
    cam01_depth_dir = os.path.join(current_dir, f"dataset/dataset_{args.dataset}/cam01/depth")
    cam02_rgb_dir = os.path.join(current_dir, f"dataset/dataset_{args.dataset}/cam02/rgb")
    cam02_depth_dir = os.path.join(current_dir, f"dataset/dataset_{args.dataset}/cam02/depth")
    joint_dir = os.path.join(current_dir, f"dataset/dataset_{args.dataset}/joint")
    
    os.makedirs(cam01_rgb_dir, exist_ok=True)
    os.makedirs(cam01_depth_dir, exist_ok=True)
    os.makedirs(cam02_rgb_dir, exist_ok=True)
    os.makedirs(cam02_depth_dir, exist_ok=True)
    os.makedirs(joint_dir, exist_ok=True)
    
    stop_thread = threading.Event()
    record_thread = threading.Thread(target=record_data, args=(cam01_rgb_dir, cam01_depth_dir, cam02_rgb_dir,cam02_depth_dir,joint_dir))
    record_thread.start()
    # time.sleep(20)

    df = pd.read_csv(f'/home/zlz/z1_ws/z1_controller/config/Traj_{args.dataset}.csv')
    for index, row in df.iterrows():
        target_joint_positions = np.array(row.values)
        arm.q= target_joint_positions[1:7]
        arm.qd= target_joint_positions[8:14]
        arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque
        arm.gripperQ = target_joint_positions[7]
        arm.sendRecv()# udp connection
        # print(arm._ctrlComp.dt)
        # timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        joint_positions = [j for j in arm.lowstate.getQ()]  
        joint_positions.append(arm.gripperQ)  
        print(joint_positions)
        with open(f'{joint_dir}/joint_positions.txt', 'a') as f:
            joint_positions_str = str(joint_positions)
            f.write(f"{datetime.now()}: {joint_positions_str}\n")
        time.sleep(arm._ctrlComp.dt+0.002)
        
    stop_thread.set()

    record_thread.join()

    arm.loopOn()
    arm.backToStart()
    arm.loopOff()
