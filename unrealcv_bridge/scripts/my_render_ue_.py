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
                if len(parts) < 7:
                    continue
                xyz = list(map(float, parts[1:4]))  # x,y,z in meters
                pyr = list(map(float, parts[4:7]))  # pitch,yaw,roll in degrees
                poses.append((xyz, pyr))
    return poses

def calculate_focal_length(width, h_fov_deg):
    """Calculate focal length in pixels from horizontal FOV"""
    return (width / 2) / np.tan(np.radians(h_fov_deg) / 2)

def euler_to_rotmat_ue(roll_deg, pitch_deg, yaw_deg):
    """
    Convert UE Euler angles to rotation matrix
    Following original repo: uc.eulerToRotmatUE(pyr[2], pyr[0], pyr[1])
    Which means: eulerToRotmatUE(roll, pitch, yaw)
    UE rotation order: Yaw (Z) -> Pitch (Y) -> Roll (X)
    """
    # Convert to radians
    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_deg) 
    yaw = np.radians(yaw_deg)
    
    # UE rotation order: Yaw (Z) -> Pitch (Y) -> Roll (X)
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cr = np.cos(roll)
    sr = np.sin(roll)
    
    # Rotation matrix: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])
    
    return R

def ue_twc_to_standard(Twc_ue):
    """Convert UE world-to-camera to standard coordinate system"""
    # UE: X=Forward, Y=Right, Z=Up
    # Standard: X=Right, Y=Down, Z=Forward
    T_ue_to_std = np.array([
        [0, 1, 0, 0],
        [0, 0, 1, 0], 
        [1, 0, 0, 0],
        [0, 0, 0, 1]
    ])
    
    Twc_std = T_ue_to_std @ Twc_ue @ np.linalg.inv(T_ue_to_std)
    return Twc_std

def twc_to_colmap_qt(Twc):
    """Convert world-to-camera transform to COLMAP quaternion + translation"""
    # COLMAP wants camera-to-world, so we need to invert
    Tcw = np.linalg.inv(Twc)
    
    # Extract rotation matrix and convert to quaternion
    R = Tcw[:3, :3]
    
    # Robust rotation matrix to quaternion conversion
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s  
        z = (R[1, 0] - R[0, 1]) * s
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    
    quat = [w, x, y, z]
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    quat = [q/norm for q in quat]
    
    # Translation is just the position part of Tcw
    tvec = Tcw[:3, 3]
    
    return quat + tvec.tolist()

def main():
    parser = argparse.ArgumentParser(description="Render images from UE poses for COLMAP")
    parser.add_argument('ue_pose_txt', help="Input pose file (timestamp x y z pitch yaw roll)")
    
    # Add missing parameters from original repo
    parser.add_argument('--unrealcv_ini', type=str, required=True, 
                       help="UnrealCV config file (required like original repo)")
    
    parser.add_argument('--width', type=int, default=640, help="Image width")
    parser.add_argument('--height', type=int, default=480, help="Image height")
    parser.add_argument('--fov', type=float, default=90.0, help="Horizontal FOV in degrees")
    
    # Use same parameter name as original repo
    parser.add_argument('--save_sleep_sec', type=float, default=0.3, 
                       help="Delay after pose change (same as original)")
    
    # Add directory parameters like original repo
    parser.add_argument('--top_save_dir', type=str, default=None,
                       help="top dir under which a stamped folder will be created")
    parser.add_argument('--save_dir', type=str, default=None,
                       help="directly specify save dir")

    args = parser.parse_args()

    # Setup output directory - match original repo structure
    if args.save_dir:
        save_dir = args.save_dir
    else:
        if args.top_save_dir:
            assert os.path.exists(args.top_save_dir)
            save_dir = os.path.join(args.top_save_dir, '{}_rec_img_pose'.format(
                datetime.now().strftime("%Y%m%d%H%M%S")))
        else:
            save_dir = f"colmap_input_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    
    if os.path.exists(save_dir):
        import shutil
        shutil.rmtree(save_dir)
    os.makedirs(save_dir)
    
    # Create same file structure as original repo
    colmap_pose_fn = os.path.join(save_dir, 'img_name_to_colmap_Tcw.txt')
    ue_pose_fn = os.path.join(save_dir, 'ue_xyzpyr.txt')
    img_dir = os.path.join(save_dir, 'images')
    os.makedirs(img_dir)
    cam_fn = os.path.join(save_dir, 'img_nm_to_colmap_cam.txt')
    
    # Copy original pose file like original repo
    import shutil
    shutil.copy2(args.ue_pose_txt, ue_pose_fn)

    print(f"Going to render from {args.ue_pose_txt} and save in {save_dir}:")
    print(f"- colmap poses: {colmap_pose_fn}")
    print(f"- ue poses: {ue_pose_fn}")
    print(f"- images: {img_dir}")
    print(f"- intrinsics: {cam_fn}")

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

    # Create COLMAP files - match original repo format
    with open(cam_fn, 'w') as cam_file, \
         open(colmap_pose_fn, 'w') as pose_file:
        
        for idx, (xyz, pyr) in enumerate(tqdm(poses)):
            img_name = f"{idx:05d}.png"
            
            # Write camera parameters (one line per image like original)
            intri_str = f"PINHOLE {args.width} {args.height} {focal:.6f} {focal:.6f} {cx:.6f} {cy:.6f}"
            cam_file.write(f"{img_name} {intri_str}\n")

            # Set pose in Unreal Engine - matches original repo logic
            # Convert meters to centimeters for UnrealCV
            xyz_ue_scale = [v * 100 for v in xyz]
            client.request(f'vset /camera/0/location {xyz_ue_scale[0]} {xyz_ue_scale[1]} {xyz_ue_scale[2]}')
            client.request(f'vset /camera/0/rotation {pyr[0]} {pyr[1]} {pyr[2]}')  # pitch, yaw, roll
            time.sleep(args.save_sleep_sec)

            # Capture image
            img_data = client.request('vget /camera/0/lit png')
            with open(os.path.join(img_dir, img_name), 'wb') as f:
                f.write(img_data)

            # Convert pose following original repo logic exactly
            # pyr contains [pitch, yaw, roll] = [pyr[0], pyr[1], pyr[2]]
            pitch, yaw, roll = pyr
            
            # Create UE world-to-camera matrix (matches original repo)
            Twc_ue = np.eye(4)
            # This matches: uc.eulerToRotmatUE(pyr[2], pyr[0], pyr[1]) = (roll, pitch, yaw)
            Twc_ue[0:3, 0:3] = euler_to_rotmat_ue(roll, pitch, yaw)
            Twc_ue[0:3, 3] = np.array(xyz)
            
            # Convert to standard coordinate system
            Twc_std = ue_twc_to_standard(Twc_ue)
            
            # Convert to COLMAP format
            qtvec = twc_to_colmap_qt(Twc_std)
            
            # Write pose in same format as original repo
            pose_file.write(f"{img_name} {' '.join(f'{q:.10f}' for q in qtvec)}\n")

    print(f"\nRendering complete. Results saved to: {save_dir}")
    print("COLMAP input structure (matches original repo):")
    print(f"  {colmap_pose_fn}")
    print(f"  {cam_fn}")
    print(f"  {img_dir}/*.png")
    print(f"  {ue_pose_fn}")

if __name__ == "__main__":
    main()