#!/usr/bin/env python3
import argparse
import os
import time
import numpy as np
from datetime import datetime
from tqdm import tqdm
import unrealcv 

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
    return poses

def calculate_focal_length(width, h_fov_deg):
    """Calculate focal length in pixels from horizontal FOV"""
    return (width / 2) / np.tan(np.radians(h_fov_deg) / 2)

def robust_ue_to_colmap_pose(xyz_ue, pyr_ue):
    """Convert UE pose to COLMAP format with proper coordinate transformation"""
    # Position: UE (meters, Z-up) → COLMAP (meters, Y-up)
    x, y, z = xyz_ue
    position_colmap = [x, z, -y]  # Transform to Y-up
    
    # Convert UE rotations (degrees) to radians
    pitch, yaw, roll = np.radians(pyr_ue)
    
    # Calculate quaternion components (UE uses Y-Z-X rotation order)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    # Transform quaternion from UE (Z-up) to COLMAP (Y-up)
    qw_colmap = 0.70710678118 * (qw + qx)  # 1/sqrt(2) = 0.707...
    qx_colmap = 0.70710678118 * (qx - qw)
    qy_colmap = 0.70710678118 * (qz + qy)
    qz_colmap = 0.70710678118 * (qy - qz)
    
    # Normalize quaternion
    norm = np.sqrt(qw_colmap**2 + qx_colmap**2 + qy_colmap**2 + qz_colmap**2)
    quat_colmap = [qw_colmap/norm, qx_colmap/norm, qy_colmap/norm, qz_colmap/norm]
    
    return position_colmap, quat_colmap

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
            
            # Write camera parameters (one line per image)
            cam_file.write(
                f"{img_name} PINHOLE {args.width} {args.height} {focal} {focal} {cx} {cy}\n"
            )

            # Rest of your pose capture logic...
            client.request(f'vset /camera/0/location {xyz[0]*100} {xyz[1]*100} {xyz[2]*100}')
            client.request(f'vset /camera/0/rotation {pyr[0]} {pyr[1]} {pyr[2]}')
            time.sleep(args.sleep)

            img_data = client.request('vget /camera/0/lit png')
            with open(os.path.join(img_dir, img_name), 'wb') as f:
                f.write(img_data)

            pos_colmap, quat_colmap = robust_ue_to_colmap_pose(xyz, pyr)
            pose_file.write(
                f"{img_name} {' '.join(map(str, quat_colmap))} {' '.join(map(str, pos_colmap))}\n"
            )



    print(f"\nRendering complete. Results saved to: {output_dir}")
    print("COLMAP input structure:")
    print(f"  {output_dir}/img_name_to_colmap_Tcw.txt")
    print(f"  {output_dir}/img_nm_to_colmap_cam.txt")
    print(f"  {output_dir}/images/*.png")

if __name__ == "__main__":
    main()