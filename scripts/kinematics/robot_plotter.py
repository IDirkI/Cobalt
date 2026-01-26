#!/usr/bin/env python3

import csv
import sys
import pathlib
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R

# -------------------------------
# Colors
# -------------------------------
## Links
COLOR_LINK = 'grey'

## Joints
# Fixed
COLOR_FIXED = 'black'

# Revolute
COLOR_REVOLUTE = 'orange'
COLOR_REVOLUTE_AXIS = 'orangered'

# Prismatic
COLOR_PRISMATIC = 'darkturquoise'
COLOR_PRISMATIC_AXIS = 'navy'
COLOR_PRISMATIC_EXTENSION = 'cadetblue'

# Universal
COLOR_UNIVERSAL = 'forestgreen'
COLOR_UNIVERSAL_AXIS = 'darkgreen'

# Spherical
COLOR_SPHERICAL = 'mediumpurple'
COLOR_SPHERICAL_AXIS = 'darkorchid'

# Cylinderical 
COLOR_CYLINDERICAL = 'indianred'
COLOR_CYLINDERICAL_AXIS = 'maroon'

# Helical 
COLOR_HELICAL = 'hotpink'
COLOR_HELICAL_AXIS = 'mediumvioletred'

# Planar 
COLOR_PLANAR = 'gold'
COLOR_PLANAR_AXIS = 'goldenrod'
COLOR_PLANAR_EXTENSION = 'darkkhaki'

# Unknown/Unrecognizes/Missing/Error
COLOR_UNKNOWN = 'mistyrose'
COLOR_UNKNOWN_EDGE = 'tomato'

## Frames
COLOR_FRAME_LINE = 'rebeccapurple'
COLOR_X = 'r'
COLOR_Y = 'b'
COLOR_Z = 'g'

# -------------------------------
# Parameters
# -------------------------------
## Thickness
LINK_THICKNESS = 3
FRAME_LINE_THICKNESS = 2
JOINT_AXIS_THICKNESS = 2

## Alpha
JOINT_FIXED_ALPHA = 0.6
JOINT_REVOLUTE_ALPHA = 0.7
JOINT_PRISMATIC_ALPHA = 0.7
JOINT_UNIVERSAL_ALPHA = 0.7
JOINT_SPHERICAL_ALPHA = 0.85
JOINT_CYLINDERICAL_ALPHA = 0.7
JOINT_PLANAR_ALPHA = 0.6

## Axis-len
JOINT_REVOLUTE_AXIS_LENGTH = 0.20
JOINT_PRISMATIC_AXIS_LENGTH = 0.18
JOINT_UNIVERSAL_AXIS_LENGTH = 0.22
JOINT_SPHERICAL_AXIS_LENGTH = 0.16
JOINT_CYLINDERICAL_AXIS_LENGTH = 0.1
JOINT_PLANAR_LONGAXIS_LENGTH = 0.15
JOINT_PLANAR_SHORTAXIS_LENGTH = 0.06

## Dimention
JOINT_FIXED_SIDE = 0.03
JOINT_PLANAR_LONG_SIZE = 0.1
JOINT_PLANAR_SHORT_SIZE = 0.005
JOINT_PLANAR_EXTENSION_SIZE = 90
JOINT_UNKNOWN_SIZE = 180

# -------------------------------
# Rotation utilities
# -------------------------------
def quat_to_rot(q):
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)]
    ])

def axisangle_to_rot(axis, angle_radians):
  rot_vector = np.array(axis) * angle_radians
  r = R.from_rotvec(rot_vector)
  
  rotation_matrix = r.as_matrix()
  
  return rotation_matrix

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
    colors = [COLOR_X, COLOR_Y, COLOR_Z]  # X=red, Y=blue, Z=green

    for i in range(3):
        axis = R @ axes[:, i]
        ax.plot(
            [pos[0], pos[0] + scale*axis[0]],
            [pos[1], pos[1] + scale*axis[1]],
            [pos[2], pos[2] + scale*axis[2]],
            color=colors[i],
            linewidth=2
        )

def draw_sphere(ax, pos, radius=0.025, color=COLOR_SPHERICAL, alpha=0.9, resolution=20):
    u = np.linspace(0, 2 * np.pi, resolution)
    v = np.linspace(0, np.pi, resolution)
    x = radius * np.outer(np.cos(u), np.sin(v)) + pos[0]
    y = radius * np.outer(np.sin(u), np.sin(v)) + pos[1]
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + pos[2]
    ax.plot_surface(x, y, z, color=color, alpha=alpha, shade=True)

def draw_cylinder(ax, pos, axis, radius=0.03, length=0.12, color=COLOR_REVOLUTE, resolution=20, alpha=0.7):
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
                    color=color, alpha=alpha, shade=True)
    
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

def draw_box(ax, pos, axis, width=0.05, height=0.05, length=0.10, color=COLOR_PRISMATIC, alpha=0.7):
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
            
            def safe_int(v):
                return int(float(v)) if v != '' else 0

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
                    ]),
                    'virtual': safe_int(row.get('virtual', '0'))
                }

            elif rtype == 'joint':
                joints.append({
                    'parent': row['parent'],
                    'child': row['child'],
                    'joint_type': safe_int(row['joint_type']),
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
                    'value': safe_float(row['value']),
                    'comp_type': safe_int(row.get('comp_type')),
                    'comp_index': safe_int(row.get('comp_index', '-1'))
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
# Compound joint detection
# -------------------------------
def group_compound_joints(joints):
    compound_groups = []
    current_group = []
    
    for i, j in enumerate(joints):
        if j['comp_type'] != 0:
            if not current_group or j['comp_index'] == 0:
                # Start new group
                if current_group:
                    compound_groups.append(current_group)
                current_group = [i]
            else:
                # Continue current group
                current_group.append(i)
        else:
            if current_group:
                compound_groups.append(current_group)
                current_group = []
    
    if current_group:
        compound_groups.append(current_group)
    
    return compound_groups

# -------------------------------
# Visualization
# -------------------------------
def visualize_robot(csv_file):
    links, joints, frames = read_robot_csv(csv_file)

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Group compound joints
    compound_groups = group_compound_joints(joints)
    compound_joint_indices = set()
    for group in compound_groups:
        compound_joint_indices.update(group)

    # --- Link COMs ---
    for name, link in links.items():
        p = link['pos']
        # Only show name if link is not virtual
        if link['virtual'] == 0:
            ax.text(p[0], p[1], p[2] + 0.02, name, fontsize=8, weight='bold')

    # --- Compound Joints (Spherical and Universal) ---
    for group in compound_groups:
        compound_type = joints[group[0]]['comp_type']
        if len(group) == 3:  # Spherical joint
            if compound_type == 1:  # Spherical Joint, type: 1
                # Use the first joint's position and parent orientation
                j0 = joints[group[0]]
                j1 = joints[group[1]]
                j2 = joints[group[2]]
                p_joint = j0['pos']
                p_parent = links[j0['parent']]['pos']
                p_child = links[j2['child']]['pos']

                axis0 = j0['axis']
                axis1 = j1['axis']
                axis2 = j2['axis']

                # Draw connection from parent to joint
                draw_line(ax, p_parent, p_joint, '-', COLOR_LINK, lw=LINK_THICKNESS)
                
                # Draw sphere for spherical joint
                draw_sphere(ax, p_joint, radius=0.055, color=COLOR_SPHERICAL, alpha=JOINT_SPHERICAL_ALPHA, resolution=25)
                
                # Draw spherical axis lines
                for axis in [axis0, axis1, axis2]:
                    axis_norm = axis / np.linalg.norm(axis)
                    axis_length = JOINT_SPHERICAL_AXIS_LENGTH
                    p_axis_start = p_joint - axis_norm * axis_length / 2
                    p_axis_end = p_joint + axis_norm * axis_length / 2
                    draw_line(ax, p_axis_start, p_axis_end, '-', COLOR_SPHERICAL_AXIS, lw=JOINT_AXIS_THICKNESS)
                
                # Draw connection from joint to child
                draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)
            elif compound_type == 3: # Planar Joint, type: 3
                # Use the first joint's position and parent orientation
                j0 = joints[group[0]]
                j1 = joints[group[1]]
                j2 = joints[group[2]]
                p_joint = j0['pos']
                p_parent = links[j0['parent']]['pos']
                p_child = links[j2['child']]['pos']

                axis0 = j0['axis']
                axis1 = j1['axis']
                axis2 = j2['axis']

                axis0_norm = axis0 / np.linalg.norm(axis0)
                axis1_norm = axis1 / np.linalg.norm(axis1)
                value0 = j0['value']
                value1 = j1['value']
                value2 = j2['value']

                p_extended0 = p_joint + (axis0_norm * value0)
                p_extended = p_extended0 + (axis1_norm * value1)
                
                # Draw connection from parent to joint
                draw_line(ax, p_parent, p_extended0, '--', COLOR_PLANAR_EXTENSION, lw=JOINT_AXIS_THICKNESS)
                draw_line(ax, p_extended0, p_extended, '--', COLOR_PLANAR_EXTENSION, lw=JOINT_AXIS_THICKNESS)
                ax.scatter(p_joint[0], p_joint[1], p_joint[2], color=COLOR_PLANAR_EXTENSION, s=JOINT_PLANAR_EXTENSION_SIZE, marker='s', edgecolors=COLOR_PLANAR_EXTENSION, linewidths=JOINT_AXIS_THICKNESS)

                Rot = axisangle_to_rot(axis2, value2)
                axes = Rot @ (np.array([
                        [axis0[0], axis1[0], axis2[0]], 
                        [axis0[1], axis1[1], axis2[1]], 
                        [axis0[2], axis1[2], axis2[2]], 
                    ]))

                # Draw flat rectangle plane for planar joint
                draw_box(ax, p_extended, axes[0], width=JOINT_PLANAR_LONG_SIZE, height=JOINT_PLANAR_SHORT_SIZE, length=JOINT_PLANAR_LONG_SIZE, color=COLOR_PLANAR, alpha=JOINT_PLANAR_ALPHA)
                # Draw planar axis lines
                for axis in [axes[0], axes[1]]:
                    axis_norm = axis / np.linalg.norm(axis)
                    axis_length = JOINT_PLANAR_LONGAXIS_LENGTH 
                    p_axis_start = p_extended - axis_norm * axis_length / 2
                    p_axis_end = p_extended + axis_norm * axis_length / 2
                    draw_line(ax, p_axis_start, p_axis_end, '-', COLOR_PLANAR_AXIS, lw=JOINT_AXIS_THICKNESS)
                # Draw short planar axis line
                axis_norm = axes[2] / np.linalg.norm(axes[2])
                axis_length = JOINT_PLANAR_SHORTAXIS_LENGTH 
                p_axis_start = p_extended - axis_norm * axis_length / 2
                p_axis_end = p_extended + axis_norm * axis_length / 2
                draw_line(ax, p_axis_start, p_axis_end, '-', COLOR_PLANAR_AXIS, lw=JOINT_AXIS_THICKNESS)
                
                # Draw connection from joint to child
                draw_line(ax, p_extended, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)
            
        elif len(group) == 2: 
            if compound_type == 2: # Universal Joint, type: 2
                # Use the first joint's position
                j0 = joints[group[0]]
                j1 = joints[group[1]]
                p_joint = j0['pos']
                p_parent = links[j0['parent']]['pos']
                p_child = links[j1['child']]['pos']
                
                axis0 = j0['axis']
                axis1 = j1['axis']
                
                # Draw connection from parent to joint
                draw_line(ax, p_parent, p_joint, '-', COLOR_LINK, lw=LINK_THICKNESS)
                
                # Draw two cylinders for universal joint
                if np.linalg.norm(axis0) > 1e-8:
                    draw_cylinder(ax, p_joint, axis0, radius=0.04, length=0.14, color=COLOR_UNIVERSAL, alpha=JOINT_UNIVERSAL_ALPHA)
                    # Draw axis line
                    axis_norm = axis0 / np.linalg.norm(axis0)
                    axis_length = JOINT_UNIVERSAL_AXIS_LENGTH
                    p_axis_start = p_joint - axis_norm * axis_length / 2
                    p_axis_end = p_joint + axis_norm * axis_length / 2
                    draw_line(ax, p_axis_start, p_axis_end, '-', COLOR_UNIVERSAL_AXIS, lw=JOINT_AXIS_THICKNESS)
                
                if np.linalg.norm(axis1) > 1e-8:
                    draw_cylinder(ax, p_joint, axis1, radius=0.04, length=0.14, color=COLOR_UNIVERSAL, alpha=JOINT_UNIVERSAL_ALPHA)
                    # Draw axis line
                    axis_norm = axis1 / np.linalg.norm(axis1)
                    axis_length = JOINT_UNIVERSAL_AXIS_LENGTH
                    p_axis_start = p_joint - axis_norm * axis_length / 2
                    p_axis_end = p_joint + axis_norm * axis_length / 2
                    draw_line(ax, p_axis_start, p_axis_end, '-', COLOR_UNIVERSAL_AXIS, lw=JOINT_AXIS_THICKNESS)
                
                # Draw connection from joint to child
                draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)
            elif compound_type == 3:    # Cylinderical Joint, type: 3
                # Use the first joint's position
                j0 = joints[group[0]]
                j1 = joints[group[1]]
                p_joint1 = j0['pos']
                p_parent = links[j0['parent']]['pos']
                p_child = links[j1['child']]['pos']
                axis = j0['axis']

                axis_norm = axis / np.linalg.norm(axis)
                value = joints[group[1]]['value']
                p_extended = p_joint1 + axis_norm * value
                
                # Draw connection from parent to joint
                draw_line(ax, p_parent, p_joint1, '-', COLOR_LINK, lw=LINK_THICKNESS)
                
                # Draw short cylinders for cylinderical joint
                if np.linalg.norm(axis) > 1e-8:
                    draw_cylinder(ax, p_joint1, axis, radius=0.04, length=0.05, color=COLOR_CYLINDERICAL, alpha=JOINT_CYLINDERICAL_ALPHA)
                    # Draw axis line
                    
                    axis_length = JOINT_CYLINDERICAL_AXIS_LENGTH
                    p_axis_start = p_joint1 - axis_norm * axis_length / 2
                    p_axis_end = p_joint1 + axis_norm * axis_length / 2
                    draw_line(ax, p_axis_start, p_axis_end, '-', COLOR_CYLINDERICAL_AXIS, lw=JOINT_AXIS_THICKNESS)

                # Draw extension if joint is displaced
                if abs(value) > 1e-6:
                    # Dashed line showing extension
                    draw_line(ax, p_joint1, p_extended, '--', COLOR_CYLINDERICAL_AXIS, lw=JOINT_AXIS_THICKNESS, alpha=0.8)
                    # Connect extended position to child COM
                    draw_line(ax, p_extended, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)
                else:
                    # No extension - connect directly
                    draw_line(ax, p_extended, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)
                
                if np.linalg.norm(axis) > 1e-8:
                    draw_cylinder(ax, p_extended, axis, radius=0.04, length=0.05, color=COLOR_CYLINDERICAL, alpha=JOINT_CYLINDERICAL_ALPHA)
                    # Draw axis line
                    axis_length = JOINT_CYLINDERICAL_AXIS_LENGTH
                    p_axis_start = p_extended - axis_norm * axis_length / 2
                    p_axis_end = p_extended + axis_norm * axis_length / 2
                    draw_line(ax, p_axis_start, p_axis_end, '-', COLOR_CYLINDERICAL_AXIS, lw=JOINT_AXIS_THICKNESS)


    # --- Regular Joints ---
    for i, j in enumerate(joints):
        # Skip if this joint is part of a compound joint
        if i in compound_joint_indices:
            continue
        
        jtype = j['joint_type']
        p_parent = links[j['parent']]['pos']
        p_child = links[j['child']]['pos']
        p_joint = j['pos']
        axis_world = j['axis']
        value = j['value']
        
        draw_line(ax, p_parent, p_joint, '-', COLOR_LINK, lw=LINK_THICKNESS)
        
        if jtype == 0:  # Fixed
            draw_box(ax, p_joint, [1, 0, 0], width=JOINT_FIXED_SIDE, height=JOINT_FIXED_SIDE, length=JOINT_FIXED_SIDE, color=COLOR_FIXED, alpha=JOINT_FIXED_ALPHA)
            draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)
            
        elif jtype == 1:  # Revolute
            if np.linalg.norm(axis_world) > 1e-8:
                # Draw cylinder aligned with world-frame axis
                draw_cylinder(ax, p_joint, axis_world, radius=0.035, length=0.12, color=COLOR_REVOLUTE, alpha=JOINT_REVOLUTE_ALPHA)
                
                # Draw axis line through cylinder (thin red)
                axis_norm = axis_world / np.linalg.norm(axis_world)
                axis_length = JOINT_REVOLUTE_AXIS_LENGTH
                p_axis_start = p_joint - axis_norm * axis_length / 2
                p_axis_end = p_joint + axis_norm * axis_length / 2
                draw_line(ax, p_axis_start, p_axis_end, '-', COLOR_REVOLUTE_AXIS, lw=JOINT_AXIS_THICKNESS)
            
            draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)
            
        elif jtype == 2:  # Prismatic
            if np.linalg.norm(axis_world) > 1e-8:
                # Draw box aligned with world-frame axis
                draw_box(ax, p_joint, axis_world, width=0.05, height=0.05, length=0.10, color=COLOR_PRISMATIC, alpha=JOINT_PRISMATIC_ALPHA)
                
                # Draw axis line through box (thin blue)
                axis_norm = axis_world / np.linalg.norm(axis_world)
                axis_length = JOINT_PRISMATIC_AXIS_LENGTH
                p_axis_start = p_joint - axis_norm * axis_length / 2
                p_axis_end = p_joint + axis_norm * axis_length / 2
                draw_line(ax, p_axis_start, p_axis_end, '-', COLOR_PRISMATIC_AXIS, lw=JOINT_AXIS_THICKNESS)
                
                # Draw extension if joint is displaced
                if abs(value) > 1e-6:
                    p_extended = p_joint + axis_norm * value
                    # Dashed line showing extension
                    draw_line(ax, p_joint, p_extended, '--', COLOR_PRISMATIC_EXTENSION, lw=JOINT_AXIS_THICKNESS, alpha=0.8)
                    # Connect extended position to child COM
                    draw_line(ax, p_extended, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)
                else:
                    # No extension - connect directly
                    draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)
            else:
                draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)
        
        else:  # Unknown type
            ax.scatter(p_joint[0], p_joint[1], p_joint[2], 
                      color=COLOR_UNKNOWN, s=JOINT_UNKNOWN_SIZE, marker='D', edgecolors=COLOR_UNKNOWN_EDGE, linewidths=1.5)
            draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)

    # --- Frames ---
    for fname, frame in frames.items():
        p_frame = frame['pos']
        p_link = links[frame['parent']]['pos']

        # Frame to parent link COM (darker purple dotted line)
        draw_line(ax, p_link, p_frame, '-', COLOR_LINK, lw=LINK_THICKNESS, alpha=1)
        draw_line(ax, p_link, p_frame, ':', COLOR_FRAME_LINE, lw=FRAME_LINE_THICKNESS, alpha=0.7)
        draw_frame(ax, p_frame, frame['quat'], scale=0.12)
        ax.text(p_frame[0], p_frame[1], p_frame[2] + 0.05, fname, fontsize=9, 
                color=COLOR_FRAME_LINE, weight='bold')

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