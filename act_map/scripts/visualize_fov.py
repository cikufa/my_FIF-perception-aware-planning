
import os
import sys
import subprocess
import argparse
from shutil import copyfile

import rospkg
import numpy as np
from colorama import init, Fore
import matplotlib.pyplot as plt
from matplotlib import rc
from mpl_toolkits.mplot3d import Axes3D

import exp_utils as eu

import matplotlib

def fov_plot(points, opt_quivers, BF_quivers, quiversforonepoint):

    scale_factorg = 0.2
    scale_factor = 0.2
    scale_factorbf = 0.2
    arrow = 0.3
    # Create the 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the 3D map points
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='blue', alpha=0.5, label='Map Points')
    ax.scatter(mean[:,0], mean[:,1], mean[:,2], c='blue', s= 20, label='Mean PC')
    ax.scatter(mean[:,3], mean[:,4], mean[:,5], c='orange', s= 20, label='Mean norm')

    # points_a = opt_quivers[:, :3] 
    # points_b = opt_quivers[:, 3:] 
    # for a, b in zip(points_a, points_b):
    #     ax.quiver(a[0], a[1], a[2],  b[0]*scale_factorg, b[1]*scale_factorg, b[2]*scale_factorg,
    #             color='orange', arrow_length_ratio=arrow)  
        
    # p_a = BF_quivers[:, :3] 
    # p_b = BF_quivers[:, 3:] 
    # for a, b in zip(p_a, p_b):
    #     ax.quiver(a[0], a[1], a[2],  b[0]*scale_factorbf, b[1]*scale_factorbf, b[2]*scale_factorbf,
    #             color='green', arrow_length_ratio=arrow)  


    pnts_a = quiversforonepoint[:, :3]  
    pnts_b = quiversforonepoint[:, 3:] 
    for a , b in zip(pnts_a, pnts_b):
        # print(b)
        ax.quiver(a[0], a[1], a[2],  b[0] * scale_factor, b[1]* scale_factor, b[2]* scale_factor,
                   alpha=0.7, color='red', arrow_length_ratio=arrow)  # Increase arrow head size
        
    

    # # Add labels and title
    # ax.set_title('FOV quivers')
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')

    # plt.legend()
    # plt.show()


    def plot_quivers_with_black_heads(ax, points_a, points_b, shaft_color, scale_factor):
        for a, b in zip(points_a, points_b):
            # Shaft (without arrowhead)
            ax.quiver(a[0], a[1], a[2],  
                      b[0]*scale_factor, b[1]*scale_factor, b[2]*scale_factor,
                      color=shaft_color, arrow_length_ratio=0)  # No arrowhead
            
            # Arrowhead (small vector at the end)
            head_length = arrow * 0.2  # Adjust head size
            head_x = a[0] + b[0]*scale_factor
            head_y = a[1] + b[1]*scale_factor
            head_z = a[2] + b[2]*scale_factor
            
            ax.quiver(head_x, head_y, head_z,  
                      b[0]*head_length, b[1]*head_length, b[2]*head_length,
                      color='black', arrow_length_ratio=1)  # Only arrowhead

    # Plot quivers with black heads
    plot_quivers_with_black_heads(ax, opt_quivers[:, :3], opt_quivers[:, 3:], 'orange', scale_factorg)
    plot_quivers_with_black_heads(ax, BF_quivers[:, :3], BF_quivers[:, 3:], 'green', scale_factorbf)

    ax.set_title('FOV quivers')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.legend()
    plt.show()


points = np.loadtxt("/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map/maps/two_walls_points_w.txt", delimiter=' ')
fov_quivers = np.loadtxt("/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map/FOVData/myquivers.csv", delimiter=',')
quiversforonepoint = np.loadtxt("/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map/FOVData/quiversforonepoint.csv", delimiter=',')
mean = np.loadtxt("/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map/FOVData/mean.csv", delimiter=',')
BF_quivers = np.loadtxt("/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map/FOVData/single_run_brute_force_rotated_quivers.csv", delimiter=',')

print(quiversforonepoint)
fov_plot(points, fov_quivers,BF_quivers, quiversforonepoint)



