#!/usr/bin/env python3
import argparse
import os
import time
import numpy as np
from datetime import datetime
from tqdm import tqdm
import unrealcv 

#from unrealengine 

def RotMatX(deg):
    rad = np.deg2rad(deg)
    s, c = np.sin(rad), np.cos(rad)

    Rx = np.eye(3)
    Rx[1, 1] = c
    Rx[2, 2] = c
    Rx[1, 2] = -s
    Rx[2, 1] = s
    return Rx

def RotMatY(deg):
    rad = np.deg2rad(deg)
    s, c = np.sin(rad), np.cos(rad)

    Ry = np.eye(3)
    Ry[0, 0] = c
    Ry[2, 2] = c
    Ry[2, 0] = -s
    Ry[0, 2] = s
    return Ry

def RotMatZ(deg):
    rad = np.deg2rad(deg)
    s, c = np.sin(rad), np.cos(rad)

    Rz = np.eye(3)
    Rz[0, 0] = c
    Rz[1, 1] = c
    Rz[0, 1] = -s
    Rz[1, 0] = s

    return Rz

def eulerToRotmatUE(roll_deg, pitch_deg, yaw_deg):
    R_roll = RotMatX(-roll_deg)
    R_pitch = RotMatY(-pitch_deg)
    R_yaw = RotMatZ(yaw_deg)

    return np.linalg.multi_dot([R_yaw, R_pitch, R_roll])

def rotmat2qvec(R):
    Rxx, Ryx, Rzx, Rxy, Ryy, Rzy, Rxz, Ryz, Rzz = R.flat
    K = np.array([
        [Rxx - Ryy - Rzz, 0, 0, 0],
        [Ryx + Rxy, Ryy - Rxx - Rzz, 0, 0],
        [Rzx + Rxz, Rzy + Ryz, Rzz - Rxx - Ryy, 0],
        [Ryz - Rzy, Rzx - Rxz, Rxy - Ryx, Rxx + Ryy + Rzz]]) / 3.0
    eigvals, eigvecs = np.linalg.eigh(K)
    qvec = eigvecs[[3, 0, 1, 2], np.argmax(eigvals)]
    if qvec[0] < 0:
        qvec *= -1
    return qvec


def TwcToColmapQT(Twc):
    qvec = rotmat2qvec(Twc[0:3, 0:3].transpose())
    tvec = -np.dot(Twc[0:3, 0:3].transpose(), Twc[0:3, 3])
    return qvec.tolist() + tvec.tolist()


def read_unreal_poses(pose_file):
    """Read UE-formatted poses (timestamp x y z pitch yaw roll)"""
    poses = []
    with open(pose_file, 'r') as f:
        for line in f:
            if line.strip() and not line.startswith('#'):
                parts = line.strip().split()
                xyz = list(map(float, parts[1:4]))  # x,y,z in meters
                pyr = list(map(float, parts[4:7]))  # pitch,yaw,roll in degrees
                poses.append((xyz, pyr))
                # print("each pose",  poses)
    return poses

def getT_wue_w():
    T_wue_w = np.eye(4)
    T_wue_w[1, 1] = -1
    return T_wue_w


def getT_w_wue():
    return np.linalg.inv(getT_wue_w())

def getT_c_cue():
    T_cue_c = np.zeros((4, 4))
    T_cue_c[0, 1] = 1
    T_cue_c[1, 2] = -1
    T_cue_c[2, 0] = 1
    T_cue_c[3, 3] = 1

    return T_cue_c


def getT_cue_c():
    return np.linalg.inv(getT_c_cue())


def ueTwcToStandard(Twc_ue):
    return np.linalg.multi_dot([getT_w_wue(), Twc_ue, getT_cue_c()])

def calculate_focal_length(width, h_fov_deg):
    return (width / 2) / np.tan(np.radians(h_fov_deg) / 2)

def ue_to_colmap_pose(xyz, pyr):
    Twc_ue = np.eye(4)
    Twc_ue[0:3, 0:3] = eulerToRotmatUE(pyr[2], pyr[0], pyr[1])
    # print("twc_ue",Twc_ue)
    Twc_ue[0:3, 3] = np.array(xyz)

    Twc = ueTwcToStandard(Twc_ue)
    qtvec_i = TwcToColmapQT(Twc)
    return qtvec_i

def main():
    parser = argparse.ArgumentParser(description="Render images from UE poses for COLMAP")
    parser.add_argument('ue_pose_txt', help="Input pose file (timestamp x y z pitch yaw roll)")
    parser.add_argument('--width', type=int, default=640, help="Image width")
    parser.add_argument('--height', type=int, default=480, help="Image height")
    parser.add_argument('--fov', type=float, default=90.0, help="Horizontal FOV in degrees")
    parser.add_argument('--save_dir', help="Output directory")
    parser.add_argument('--sleep', type=float, default=0.3, help="Delay after pose change")
    args = parser.parse_args()

    # Setup output directory
    output_dir = args.save_dir if args.save_dir else f"colmap_input_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(output_dir, exist_ok=True)
    img_dir = os.path.join(output_dir, 'images')
    os.makedirs(img_dir, exist_ok=True)

    # Calculate camera intrinsics
    focal = calculate_focal_length(args.width, args.fov)
    cx, cy = args.width/2, args.height/2

    # Connect to UnrealCV
    # client = unrealcv.Client(('localhost', 9000), timeout=15) 
    client = unrealcv.Client(('localhost', 9000))    
   
    client.connect()
    if not client.isconnected():
        raise ConnectionError("Failed to connect to UnrealCV")
    print("Connected to UnrealCV")

    # Read and process poses
    poses = read_unreal_poses(args.ue_pose_txt)
    print(f"Processing {len(poses)} poses...")

    # Create COLMAP files
    with open(os.path.join(output_dir, 'img_nm_to_colmap_cam.txt'), 'w') as cam_file, \
         open(os.path.join(output_dir, 'img_name_to_colmap_Tcw.txt'), 'w') as pose_file:
        
        # Write camera intrinsics for EACH image
        for idx, (xyz, pyr) in enumerate(tqdm(poses)):
            img_name = f"{idx:05d}.png"
            print("img_name", img_name, "iteration", pyr)
            # Write camera parameters (one line per image)
            cam_file.write(
                f"{img_name} PINHOLE {args.width} {args.height} {focal} {focal} {cx} {cy}\n"
            )

            # Rest of your pose capture logic...
            client.request(f'vset /camera/0/location {xyz[0]*100} {xyz[1]*100} {xyz[2]*100}')
            
            pyr = [round(v, 2) for v in pyr]
            client.request(f'vset /camera/0/rotation {pyr[0]} {pyr[1]} {pyr[2]}')
            print(xyz, "  , rot:  ",pyr)
            time.sleep(args.sleep)

            img_data = client.request('vget /camera/0/lit png')
            with open(os.path.join(img_dir, img_name), 'wb') as f:
                f.write(img_data)
   
            qtvec_i = ue_to_colmap_pose(xyz, pyr)
            poses_colmap = qtvec_i  # single pose for this frame

            # Write quaternion + translation in COLMAP format
            pose_file.write(f"{img_name} {' '.join(map(str, poses_colmap))}\n")
    
    client.disconnect()



if __name__ == "__main__":
    main()