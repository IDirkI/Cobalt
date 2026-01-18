#!/usr/bin/env python3

import csv
import sys
import pathlib
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# -------------------------------
# Quaternion utilities
# -------------------------------
def quat_to_rot(q):
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)]
    ])

# -------------------------------
# Drawing helpers
# -------------------------------
def draw_line(ax, p0, p1, style='-', color='k', lw=2, alpha=1.0):
    ax.plot(
        [p0[0], p1[0]],
        [p0[1], p1[1]],
        [p0[2], p1[2]],
        linestyle=style,
        color=color,
        linewidth=lw,
        alpha=alpha
    )

def draw_frame(ax, pos, quat, scale=0.1):
    R = quat_to_rot(quat)
    axes = np.eye(3)
    colors = ['r', 'b', 'g']  # X=red, Y=blue, Z=green

    for i in range(3):
        axis = R @ axes[:, i]
        ax.plot(
            [pos[0], pos[0] + scale*axis[0]],
            [pos[1], pos[1] + scale*axis[1]],
            [pos[2], pos[2] + scale*axis[2]],
            color=colors[i],
            linewidth=2
        )

def draw_sphere(ax, pos, radius=0.025, color='black', alpha=0.9, resolution=20):
    """Draw a sphere at the given position"""
    u = np.linspace(0, 2 * np.pi, resolution)
    v = np.linspace(0, np.pi, resolution)
    x = radius * np.outer(np.cos(u), np.sin(v)) + pos[0]
    y = radius * np.outer(np.sin(u), np.sin(v)) + pos[1]
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + pos[2]
    ax.plot_surface(x, y, z, color=color, alpha=alpha, shade=True)

def draw_cylinder(ax, pos, axis, radius=0.03, length=0.12, color='orange', resolution=20):
    """Draw a cylinder aligned with the given axis"""
    if np.linalg.norm(axis) < 1e-8:
        return
    
    axis = axis / np.linalg.norm(axis)
    
    # Find perpendicular vectors
    if abs(axis[2]) < 0.9:
        perp1 = np.cross(axis, np.array([0, 0, 1]))
    else:
        perp1 = np.cross(axis, np.array([1, 0, 0]))
    perp1 = perp1 / np.linalg.norm(perp1)
    perp2 = np.cross(axis, perp1)
    
    # Create cylinder mesh
    theta = np.linspace(0, 2*np.pi, resolution)
    z = np.array([-length/2, length/2])
    theta_grid, z_grid = np.meshgrid(theta, z)
    
    # Circular cross-sections
    x_circle = radius * np.cos(theta_grid)
    y_circle = radius * np.sin(theta_grid)
    
    # Convert to 3D points and rotate
    points = np.zeros((len(z), len(theta), 3))
    for i, zi in enumerate(z):
        for j, th in enumerate(theta):
            local_pt = x_circle[i,j] * perp1 + y_circle[i,j] * perp2 + zi * axis
            points[i, j] = pos + local_pt
    
    # Plot surface
    ax.plot_surface(points[:,:,0], points[:,:,1], points[:,:,2], 
                    color=color, alpha=0.7, shade=True)
    
    # Add end caps
    for i in [0, 1]:
        cap_center = pos + z[i] * axis
        cap_points = np.zeros((len(theta), 3))
        for j, th in enumerate(theta):
            local_pt = radius * (np.cos(th) * perp1 + np.sin(th) * perp2)
            cap_points[j] = cap_center + local_pt
        
        # Create triangular fan for cap
        verts = [cap_points.tolist()]
        ax.add_collection3d(Poly3DCollection(verts, alpha=0.7, facecolor=color, edgecolor='none'))

def draw_box(ax, pos, axis, width=0.05, height=0.05, length=0.10, color='cyan', alpha=0.7):
    """Draw a box aligned with the given axis"""
    if np.linalg.norm(axis) < 1e-8:
        return
    
    axis = axis / np.linalg.norm(axis)
    
    # Find perpendicular vectors
    if abs(axis[2]) < 0.9:
        perp1 = np.cross(axis, np.array([0, 0, 1]))
    else:
        perp1 = np.cross(axis, np.array([1, 0, 0]))
    perp1 = perp1 / np.linalg.norm(perp1)
    perp2 = np.cross(axis, perp1)
    
    # Create 8 vertices of the box
    half_len = length / 2
    vertices = []
    for z_sign in [-1, 1]:
        for y_sign in [-1, 1]:
            for x_sign in [-1, 1]:
                v = pos + axis * (z_sign * half_len) + \
                    perp1 * (x_sign * width / 2) + \
                    perp2 * (y_sign * height / 2)
                vertices.append(v)
    
    vertices = np.array(vertices)
    
    # Define the 6 faces of the box (indices into vertices array)
    faces = [
        [0, 1, 3, 2],  # bottom
        [4, 5, 7, 6],  # top
        [0, 1, 5, 4],  # front
        [2, 3, 7, 6],  # back
        [0, 2, 6, 4],  # left
        [1, 3, 7, 5]   # right
    ]
    
    # Create face collection
    face_collection = [[vertices[i] for i in face] for face in faces]
    poly = Poly3DCollection(face_collection, alpha=alpha, facecolor=color, edgecolor='darkgray', linewidths=1)
    ax.add_collection3d(poly)

# -------------------------------
# CSV loader
# -------------------------------
def read_robot_csv(csv_file):
    links = {}
    joints = []
    frames = {}

    with open(csv_file, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rtype = row['type']

            def safe_float(v):
                return float(v) if v != '' else 0.0

            if rtype == 'link':
                links[row['name']] = {
                    'pos': np.array([
                        safe_float(row['x']),
                        safe_float(row['y']),
                        safe_float(row['z'])
                    ]),
                    'quat': np.array([
                        safe_float(row['qw']),
                        safe_float(row['qx']),
                        safe_float(row['qy']),
                        safe_float(row['qz'])
                    ])
                }

            elif rtype == 'joint':
                joints.append({
                    'parent': row['parent'],
                    'child': row['child'],
                    'joint_type': int(float(row['joint_type'])) if row['joint_type'] else 0,
                    'pos': np.array([
                        safe_float(row['x']),
                        safe_float(row['y']),
                        safe_float(row['z'])
                    ]),
                    'axis': np.array([
                        safe_float(row['axis_x']),
                        safe_float(row['axis_y']),
                        safe_float(row['axis_z'])
                    ]),
                    'value': safe_float(row['value'])
                })

            elif rtype == 'frame':
                frames[row['name']] = {
                    'parent': row['parent'],
                    'pos': np.array([
                        safe_float(row['x']),
                        safe_float(row['y']),
                        safe_float(row['z'])
                    ]),
                    'quat': np.array([
                        safe_float(row['qw']),
                        safe_float(row['qx']),
                        safe_float(row['qy']),
                        safe_float(row['qz'])
                    ])
                }

    return links, joints, frames

# -------------------------------
# Visualization
# -------------------------------
def visualize_robot(csv_file):
    links, joints, frames = read_robot_csv(csv_file)

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # --- Link COMs ---
    for name, link in links.items():
        p = link['pos']
        # Just show the name, no large marker
        ax.text(p[0], p[1], p[2] + 0.02, name, fontsize=8, weight='bold')

    # --- Joints and Connections ---
    for j in joints:
        jtype = j['joint_type']
        p_parent = links[j['parent']]['pos']
        p_child = links[j['child']]['pos']
        p_joint = j['pos']
        axis_world = j['axis']  # Axis is already in world frame from C++
        value = j['value']
        
        # Get parent link orientation to rotate joint visuals
        parent_quat = links[j['parent']]['quat']
        R_parent = quat_to_rot(parent_quat)
        
        # 1. Parent COM to Joint (dimgrey)
        draw_line(ax, p_parent, p_joint, '-', 'dimgrey', lw=2)
        
        # 2. Joint visualization based on type
        if jtype == 0:  # Fixed - small black sphere
            draw_sphere(ax, p_joint, radius=0.025, color='black', alpha=0.9)
            draw_line(ax, p_joint, p_child, '-', 'dimgrey', lw=2)
            
        elif jtype == 1:  # Revolute - cylinder with red axis
            if np.linalg.norm(axis_world) > 1e-8:
                # Draw cylinder aligned with world-frame axis
                draw_cylinder(ax, p_joint, axis_world, radius=0.035, length=0.12, color='orange')
                
                # Draw axis line through cylinder (thin red)
                axis_norm = axis_world / np.linalg.norm(axis_world)
                axis_length = 0.20
                p_axis_start = p_joint - axis_norm * axis_length / 2
                p_axis_end = p_joint + axis_norm * axis_length / 2
                draw_line(ax, p_axis_start, p_axis_end, '-', 'red', lw=1)
            
            draw_line(ax, p_joint, p_child, '-', 'dimgrey', lw=2)
            
        elif jtype == 2:  # Prismatic - box with blue axis and extension
            if np.linalg.norm(axis_world) > 1e-8:
                # Draw box aligned with world-frame axis
                draw_box(ax, p_joint, axis_world, width=0.05, height=0.05, length=0.10, color='darkturquoise')
                
                # Draw axis line through box (thin blue)
                axis_norm = axis_world / np.linalg.norm(axis_world)
                axis_length = 0.18
                p_axis_start = p_joint - axis_norm * axis_length / 2
                p_axis_end = p_joint + axis_norm * axis_length / 2
                draw_line(ax, p_axis_start, p_axis_end, '-', 'blue', lw=1)
                
                # Draw extension if joint is displaced
                if abs(value) > 1e-6:
                    p_extended = p_joint + axis_norm * value
                    # Dashed darkturquoise line showing extension
                    draw_line(ax, p_joint, p_extended, '--', 'darkturquoise', lw=2, alpha=0.8)
                    # Connect extended position to child COM
                    draw_line(ax, p_extended, p_child, '-', 'dimgrey', lw=2)
                else:
                    # No extension - connect directly
                    draw_line(ax, p_joint, p_child, '-', 'dimgrey', lw=2)
            else:
                draw_line(ax, p_joint, p_child, '-', 'dimgrey', lw=2)
        
        else:  # Unknown type
            ax.scatter(p_joint[0], p_joint[1], p_joint[2], 
                      color='purple', s=80, marker='D', edgecolors='indigo', linewidths=1.5)
            draw_line(ax, p_joint, p_child, '-', 'dimgrey', lw=2)

    # --- Frames ---
    for fname, frame in frames.items():
        p_frame = frame['pos']
        p_link = links[frame['parent']]['pos']

        # Frame to parent link COM (darker purple dotted line)
        draw_line(ax, p_link, p_frame, '-', 'dimgrey', lw=2, alpha=1)
        draw_line(ax, p_link, p_frame, ':', 'rebeccapurple', lw=2, alpha=0.7)
        draw_frame(ax, p_frame, frame['quat'], scale=0.12)
        ax.text(p_frame[0], p_frame[1], p_frame[2] + 0.05, fname, fontsize=9, 
                color='rebeccapurple', weight='bold')

    # --- Formatting ---
    ax.set_xlabel("X", fontsize=11)
    ax.set_ylabel("Y", fontsize=11)
    ax.set_zlabel("Z", fontsize=11)
    ax.set_title("Robot Kinematic State", fontsize=14, weight='bold')

    # Set equal aspect ratio
    all_points = np.array([link['pos'] for link in links.values()])
    max_range = np.array([all_points[:, 0].max() - all_points[:, 0].min(),
                          all_points[:, 1].max() - all_points[:, 1].min(),
                          all_points[:, 2].max() - all_points[:, 2].min()]).max() / 2.0
    mid_x = (all_points[:, 0].max() + all_points[:, 0].min()) * 0.5
    mid_y = (all_points[:, 1].max() + all_points[:, 1].min()) * 0.5
    mid_z = (all_points[:, 2].max() + all_points[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.tight_layout()
    plt.show()

# -------------------------------
# Entry point
# -------------------------------
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python robot_plotter.py <csv_name_without_extension>")
        sys.exit(1)

    proj_root = pathlib.Path(__file__).resolve().parents[2]
    csv_path = proj_root / "results" / "kinematics" / (sys.argv[1] + ".csv")

    if not csv_path.exists():
        print(f"CSV not found: {csv_path}")
        sys.exit(1)

    visualize_robot(csv_path)