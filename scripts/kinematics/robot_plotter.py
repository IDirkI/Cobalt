#!/usr/bin/env python3

import sys
import json
import struct
import pathlib
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as widgets
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R

# -------------------------------
# Colors
# -------------------------------
## Links
COLOR_LINK = 'grey'

## Joints
COLOR_FIXED           = 'black'
COLOR_REVOLUTE        = 'orange'
COLOR_REVOLUTE_AXIS   = 'orangered'
COLOR_PRISMATIC       = 'darkturquoise'
COLOR_PRISMATIC_AXIS  = 'navy'
COLOR_PRISMATIC_EXTENSION = 'cadetblue'
COLOR_UNIVERSAL       = 'forestgreen'
COLOR_UNIVERSAL_AXIS  = 'darkgreen'
COLOR_SPHERICAL       = 'mediumpurple'
COLOR_SPHERICAL_AXIS  = 'darkorchid'
COLOR_CYLINDERICAL    = 'indianred'
COLOR_CYLINDERICAL_AXIS = 'maroon'
COLOR_HELICAL         = 'hotpink'
COLOR_HELICAL_AXIS    = 'mediumvioletred'
COLOR_PLANAR          = 'gold'
COLOR_PLANAR_AXIS     = 'goldenrod'
COLOR_PLANAR_EXTENSION = 'darkkhaki'
COLOR_UNKNOWN         = 'mistyrose'
COLOR_UNKNOWN_EDGE    = 'tomato'

## Frames
COLOR_FRAME_LINE = 'rebeccapurple'
COLOR_X = 'r'
COLOR_Y = 'b'
COLOR_Z = 'g'

## End-effector trace (animation only)
COLOR_EE_TRACE = 'red'
ALPHA_EE_TRACE = 0.7

# -------------------------------
# Parameters
# -------------------------------
LINK_THICKNESS       = 3
FRAME_LINE_THICKNESS = 2
JOINT_AXIS_THICKNESS = 2

JOINT_FIXED_ALPHA       = 0.6
JOINT_REVOLUTE_ALPHA    = 0.7
JOINT_PRISMATIC_ALPHA   = 0.7
JOINT_UNIVERSAL_ALPHA   = 0.7
JOINT_SPHERICAL_ALPHA   = 0.85
JOINT_CYLINDERICAL_ALPHA = 0.7
JOINT_PLANAR_ALPHA      = 0.6

JOINT_REVOLUTE_AXIS_LENGTH    = 0.20
JOINT_PRISMATIC_AXIS_LENGTH   = 0.18
JOINT_UNIVERSAL_AXIS_LENGTH   = 0.22
JOINT_SPHERICAL_AXIS_LENGTH   = 0.16
JOINT_CYLINDERICAL_AXIS_LENGTH = 0.1
JOINT_PLANAR_LONGAXIS_LENGTH  = 0.15
JOINT_PLANAR_SHORTAXIS_LENGTH = 0.06

JOINT_FIXED_SIDE          = 0.03
JOINT_PLANAR_LONG_SIZE    = 0.1
JOINT_PLANAR_SHORT_SIZE   = 0.005
JOINT_PLANAR_EXTENSION_SIZE = 90
JOINT_UNKNOWN_SIZE        = 180

# -------------------------------
# Binary format constants
# -------------------------------
CLOG_MAGIC        = b'COBL'
CLOG_FRAME_SYNC   = 0xCBCB

FILE_HEADER_FMT  = '<4sBBBBI'   # magic[4], version, nLinks, nJoints, nFrames, jsonLen
FILE_HEADER_SIZE = struct.calcsize(FILE_HEADER_FMT)   # 12 bytes

FRAME_HEADER_FMT  = '<HxxIQ'    # sync(u16), 2 pad bytes, frameIndex(u32), timestamp_us(u64)
FRAME_HEADER_SIZE = struct.calcsize(FRAME_HEADER_FMT)  # 16 bytes

TRANSFORM_FMT  = '<7f'          # pos[3] + quat[4]
TRANSFORM_SIZE = struct.calcsize(TRANSFORM_FMT)   # 28 bytes

JOINT_REC_FMT  = '<8f'          # pos[3] + quat[4] + q
JOINT_REC_SIZE = struct.calcsize(JOINT_REC_FMT)   # 32 bytes

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
    return r.as_matrix()

# -------------------------------
# .clog binary reader
# -------------------------------
def _is_virtual_link(name):
    return '_s_link' in name

def _rotate_axis(local_axis, quat_wxyz):
    return quat_to_rot(quat_wxyz) @ np.array(local_axis)

def _tag_compound_joints(joints_list):
    for j in joints_list:
        j['_compound'] = _is_virtual_link(j['parent']) or _is_virtual_link(j['child'])

    i = 0
    while i < len(joints_list):
        if joints_list[i]['_compound']:
            group_start = i
            while i < len(joints_list) and joints_list[i]['_compound']:
                i += 1
            group_end  = i
            group_size = group_end - group_start
            comp_type  = 1 if group_size == 3 else 2
            for k in range(group_start, group_end):
                joints_list[k]['comp_type']  = comp_type
                joints_list[k]['comp_index'] = k - group_start
        else:
            joints_list[i]['comp_type']  = 0
            joints_list[i]['comp_index'] = -1
            i += 1

    for j in joints_list:
        del j['_compound']

    return joints_list

def _reconstruct_frame(header, link_recs, joint_recs, frame_recs):
    link_names  = header['links']
    joint_infos = header['joints']
    frame_infos = header['frames']

    links = {}
    for i, name in enumerate(link_names):
        px, py, pz, qw, qx, qy, qz = link_recs[i]
        links[name] = {
            'pos':     np.array([px, py, pz]),
            'quat':    np.array([qw, qx, qy, qz]),
            'virtual': 1 if _is_virtual_link(name) else 0
        }

    joints_list = []
    for i, jinfo in enumerate(joint_infos):
        px, py, pz, qw, qx, qy, qz, q = joint_recs[i]
        world_quat = np.array([qw, qx, qy, qz])
        joints_list.append({
            'parent':     jinfo['parent'],
            'child':      jinfo['child'],
            'joint_type': jinfo['type'],
            'pos':        np.array([px, py, pz]),
            'axis':       _rotate_axis(jinfo['axis'], world_quat),
            'value':      q,
        })

    _tag_compound_joints(joints_list)

    frames = {}
    for i, finfo in enumerate(frame_infos):
        px, py, pz, qw, qx, qy, qz = frame_recs[i]
        frames[finfo['name']] = {
            'parent': finfo['parent'],
            'pos':    np.array([px, py, pz]),
            'quat':   np.array([qw, qx, qy, qz])
        }

    return links, joints_list, frames

def read_robot_clog(clog_path):
    with open(clog_path, 'rb') as f:

        raw_header = f.read(FILE_HEADER_SIZE)
        if len(raw_header) < FILE_HEADER_SIZE:
            raise ValueError(f"Truncated .clog file: {clog_path}")

        magic, version, nL, nJ, nE, json_len = struct.unpack(FILE_HEADER_FMT, raw_header)

        if magic != CLOG_MAGIC:
            raise ValueError(
                f"Not a .clog file (bad signature: {magic!r}, expected {CLOG_MAGIC!r}): {clog_path}"
            )

        json_bytes = f.read(json_len)
        if len(json_bytes) < json_len:
            raise ValueError("Truncated JSON header in .clog file")

        header = json.loads(json_bytes.decode('utf-8'))

        all_frames = []

        while True:
            raw_fh = f.read(FRAME_HEADER_SIZE)
            if len(raw_fh) == 0:
                break
            if len(raw_fh) < FRAME_HEADER_SIZE:
                raise ValueError("Truncated frame header in .clog file")

            sync, frame_idx, timestamp_us = struct.unpack(FRAME_HEADER_FMT, raw_fh)

            if sync != CLOG_FRAME_SYNC:
                raise ValueError(
                    f"Bad frame sync word 0x{sync:04X} at frame {frame_idx} "
                    f"(expected 0x{CLOG_FRAME_SYNC:04X})"
                )

            link_recs = []
            for _ in range(nL):
                link_recs.append(struct.unpack(TRANSFORM_FMT, f.read(TRANSFORM_SIZE)))

            joint_recs = []
            for _ in range(nJ):
                joint_recs.append(struct.unpack(JOINT_REC_FMT, f.read(JOINT_REC_SIZE)))

            frame_recs = []
            for _ in range(nE):
                frame_recs.append(struct.unpack(TRANSFORM_FMT, f.read(TRANSFORM_SIZE)))

            links, joints_list, frames_dict = _reconstruct_frame(
                header, link_recs, joint_recs, frame_recs
            )
            all_frames.append((timestamp_us, links, joints_list, frames_dict))

    return header, all_frames

# -------------------------------
# Drawing helpers
# -------------------------------
def draw_line(ax, p0, p1, style='-', color='k', lw=2, alpha=1.0):
    ax.plot(
        [p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]],
        linestyle=style, color=color, linewidth=lw, alpha=alpha
    )

def draw_frame(ax, pos, quat, scale=0.1):
    R_mat  = quat_to_rot(quat)
    colors = [COLOR_X, COLOR_Y, COLOR_Z]
    for i in range(3):
        axis = R_mat @ np.eye(3)[:, i]
        ax.plot(
            [pos[0], pos[0] + scale*axis[0]],
            [pos[1], pos[1] + scale*axis[1]],
            [pos[2], pos[2] + scale*axis[2]],
            color=colors[i], linewidth=2
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
    axis  = axis / np.linalg.norm(axis)
    perp1 = np.cross(axis, [0,0,1] if abs(axis[2]) < 0.9 else [1,0,0])
    perp1 = perp1 / np.linalg.norm(perp1)
    perp2 = np.cross(axis, perp1)

    theta          = np.linspace(0, 2*np.pi, resolution)
    z_ends         = np.array([-length/2, length/2])
    theta_g, _     = np.meshgrid(theta, z_ends)
    x_c            = radius * np.cos(theta_g)
    y_c            = radius * np.sin(theta_g)

    points = np.zeros((2, len(theta), 3))
    for i, zi in enumerate(z_ends):
        for j in range(len(theta)):
            points[i, j] = pos + x_c[i,j]*perp1 + y_c[i,j]*perp2 + zi*axis

    ax.plot_surface(points[:,:,0], points[:,:,1], points[:,:,2],
                    color=color, alpha=alpha, shade=True)

    for i in [0, 1]:
        cap_center = pos + z_ends[i] * axis
        cap_pts    = np.array([
            cap_center + radius*(np.cos(t)*perp1 + np.sin(t)*perp2)
            for t in theta
        ])
        ax.add_collection3d(Poly3DCollection([cap_pts.tolist()],
                            alpha=0.7, facecolor=color, edgecolor='none'))

def draw_box(ax, pos, axis, width=0.05, height=0.05, length=0.10, color=COLOR_PRISMATIC, alpha=0.7):
    if np.linalg.norm(axis) < 1e-8:
        return
    axis  = axis / np.linalg.norm(axis)
    perp1 = np.cross(axis, [0,0,1] if abs(axis[2]) < 0.9 else [1,0,0])
    perp1 = perp1 / np.linalg.norm(perp1)
    perp2 = np.cross(axis, perp1)
    hl    = length / 2

    verts = np.array([
        pos + z*hl*axis + x*(width/2)*perp1 + y*(height/2)*perp2
        for z in [-1,1] for y in [-1,1] for x in [-1,1]
    ])
    faces = [[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]]
    ax.add_collection3d(Poly3DCollection(
        [[verts[i] for i in f] for f in faces],
        alpha=alpha, facecolor=color, edgecolor='darkgray', linewidths=1
    ))

# -------------------------------
# Compound joint detection
# -------------------------------
def group_compound_joints(joints):
    compound_groups = []
    current_group   = []
    for i, j in enumerate(joints):
        if j['comp_type'] != 0:
            if not current_group or j['comp_index'] == 0:
                if current_group:
                    compound_groups.append(current_group)
                current_group = [i]
            else:
                current_group.append(i)
        else:
            if current_group:
                compound_groups.append(current_group)
                current_group = []
    if current_group:
        compound_groups.append(current_group)
    return compound_groups

# -------------------------------
# Core draw routine
# -------------------------------
def _draw_robot(ax, links, joints, frames):
    compound_groups        = group_compound_joints(joints)
    compound_joint_indices = set()
    for group in compound_groups:
        compound_joint_indices.update(group)

    # --- Link labels ---
    for name, link in links.items():
        p = link['pos']
        if link['virtual'] == 0:
            ax.text(p[0], p[1], p[2] + 0.02, name, fontsize=8, weight='bold')

    # --- Compound joints ---
    for group in compound_groups:
        compound_type = joints[group[0]]['comp_type']

        if len(group) == 3:
            if compound_type == 1:  # Spherical
                j0, j1, j2   = joints[group[0]], joints[group[1]], joints[group[2]]
                p_joint       = j0['pos']
                p_parent      = links[j0['parent']]['pos']
                p_child       = links[j2['child']]['pos']

                draw_line(ax, p_parent, p_joint, '-', COLOR_LINK, lw=LINK_THICKNESS)
                draw_sphere(ax, p_joint, radius=0.055, color=COLOR_SPHERICAL,
                            alpha=JOINT_SPHERICAL_ALPHA, resolution=25)

                for jn in [j0, j1, j2]:
                    if np.linalg.norm(jn['axis']) > 1e-8:
                        an  = jn['axis'] / np.linalg.norm(jn['axis'])
                        p_s = p_joint - an * JOINT_SPHERICAL_AXIS_LENGTH / 2
                        p_e = p_joint + an * JOINT_SPHERICAL_AXIS_LENGTH / 2
                        draw_line(ax, p_s, p_e, '-', COLOR_SPHERICAL_AXIS, lw=JOINT_AXIS_THICKNESS)

                draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)

            elif compound_type == 3:  # Planar
                j0, j1, j2   = joints[group[0]], joints[group[1]], joints[group[2]]
                p_joint       = j0['pos']
                p_parent      = links[j0['parent']]['pos']
                p_child       = links[j2['child']]['pos']

                an0 = j0['axis'] / np.linalg.norm(j0['axis']) if np.linalg.norm(j0['axis']) > 1e-8 else j0['axis']
                an1 = j1['axis'] / np.linalg.norm(j1['axis']) if np.linalg.norm(j1['axis']) > 1e-8 else j1['axis']

                p_ext0 = p_joint + an0 * j0['value']
                p_ext  = p_ext0  + an1 * j1['value']

                draw_line(ax, p_parent, p_ext0, '--', COLOR_PLANAR_EXTENSION, lw=JOINT_AXIS_THICKNESS)
                draw_line(ax, p_ext0,   p_ext,  '--', COLOR_PLANAR_EXTENSION, lw=JOINT_AXIS_THICKNESS)
                ax.scatter(*p_joint, color=COLOR_PLANAR_EXTENSION, s=JOINT_PLANAR_EXTENSION_SIZE,
                           marker='s', edgecolors=COLOR_PLANAR_EXTENSION, linewidths=JOINT_AXIS_THICKNESS)

                Rot  = axisangle_to_rot(j2['axis'], j2['value'])
                axes = Rot @ np.column_stack([j0['axis'], j1['axis'], j2['axis']])

                draw_box(ax, p_ext, axes[:,0],
                         width=JOINT_PLANAR_LONG_SIZE, height=JOINT_PLANAR_SHORT_SIZE,
                         length=JOINT_PLANAR_LONG_SIZE, color=COLOR_PLANAR, alpha=JOINT_PLANAR_ALPHA)

                for col in [0, 1]:
                    an  = axes[:,col] / np.linalg.norm(axes[:,col])
                    p_s = p_ext - an * JOINT_PLANAR_LONGAXIS_LENGTH / 2
                    p_e = p_ext + an * JOINT_PLANAR_LONGAXIS_LENGTH / 2
                    draw_line(ax, p_s, p_e, '-', COLOR_PLANAR_AXIS, lw=JOINT_AXIS_THICKNESS)

                an  = axes[:,2] / np.linalg.norm(axes[:,2])
                p_s = p_ext - an * JOINT_PLANAR_SHORTAXIS_LENGTH / 2
                p_e = p_ext + an * JOINT_PLANAR_SHORTAXIS_LENGTH / 2
                draw_line(ax, p_s, p_e, '-', COLOR_PLANAR_AXIS, lw=JOINT_AXIS_THICKNESS)
                draw_line(ax, p_ext, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)

        elif len(group) == 2:
            if compound_type == 2:  # Universal
                j0, j1   = joints[group[0]], joints[group[1]]
                p_joint  = j0['pos']
                p_parent = links[j0['parent']]['pos']
                p_child  = links[j1['child']]['pos']

                draw_line(ax, p_parent, p_joint, '-', COLOR_LINK, lw=LINK_THICKNESS)
                for jn in [j0, j1]:
                    if np.linalg.norm(jn['axis']) > 1e-8:
                        draw_cylinder(ax, p_joint, jn['axis'], radius=0.04, length=0.14,
                                      color=COLOR_UNIVERSAL, alpha=JOINT_UNIVERSAL_ALPHA)
                        an  = jn['axis'] / np.linalg.norm(jn['axis'])
                        p_s = p_joint - an * JOINT_UNIVERSAL_AXIS_LENGTH / 2
                        p_e = p_joint + an * JOINT_UNIVERSAL_AXIS_LENGTH / 2
                        draw_line(ax, p_s, p_e, '-', COLOR_UNIVERSAL_AXIS, lw=JOINT_AXIS_THICKNESS)
                draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)

            elif compound_type == 3:  # Cylindrical
                j0, j1   = joints[group[0]], joints[group[1]]
                p_joint  = j0['pos']
                p_parent = links[j0['parent']]['pos']
                p_child  = links[j1['child']]['pos']
                axis     = j0['axis']

                draw_line(ax, p_parent, p_joint, '-', COLOR_LINK, lw=LINK_THICKNESS)

                if np.linalg.norm(axis) > 1e-8:
                    an    = axis / np.linalg.norm(axis)
                    value = j1['value']
                    p_ext = p_joint + an * value

                    draw_cylinder(ax, p_joint, axis, radius=0.04, length=0.05,
                                  color=COLOR_CYLINDERICAL, alpha=JOINT_CYLINDERICAL_ALPHA)
                    p_s = p_joint - an * JOINT_CYLINDERICAL_AXIS_LENGTH / 2
                    p_e = p_joint + an * JOINT_CYLINDERICAL_AXIS_LENGTH / 2
                    draw_line(ax, p_s, p_e, '-', COLOR_CYLINDERICAL_AXIS, lw=JOINT_AXIS_THICKNESS)

                    if abs(value) > 1e-6:
                        draw_line(ax, p_joint, p_ext, '--', COLOR_CYLINDERICAL_AXIS,
                                  lw=JOINT_AXIS_THICKNESS, alpha=0.8)
                        draw_line(ax, p_ext, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)
                    else:
                        draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)

                    draw_cylinder(ax, p_ext, axis, radius=0.04, length=0.05,
                                  color=COLOR_CYLINDERICAL, alpha=JOINT_CYLINDERICAL_ALPHA)
                    p_s = p_ext - an * JOINT_CYLINDERICAL_AXIS_LENGTH / 2
                    p_e = p_ext + an * JOINT_CYLINDERICAL_AXIS_LENGTH / 2
                    draw_line(ax, p_s, p_e, '-', COLOR_CYLINDERICAL_AXIS, lw=JOINT_AXIS_THICKNESS)

    # --- Simple joints ---
    for i, joint in enumerate(joints):
        if i in compound_joint_indices:
            continue

        p_joint  = joint['pos']
        p_parent = links[joint['parent']]['pos']
        p_child  = links[joint['child']]['pos']
        jtype    = joint['joint_type']
        axis     = joint['axis']
        value    = joint['value']

        draw_line(ax, p_parent, p_joint, '-', COLOR_LINK, lw=LINK_THICKNESS)

        if jtype == 0:  # Fixed
            draw_box(ax, p_joint, [1,0,0],
                     width=JOINT_FIXED_SIDE, height=JOINT_FIXED_SIDE, length=JOINT_FIXED_SIDE,
                     color=COLOR_FIXED, alpha=JOINT_FIXED_ALPHA)
            draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)

        elif jtype == 1:  # Revolute
            if np.linalg.norm(axis) > 1e-8:
                draw_cylinder(ax, p_joint, axis, radius=0.035, length=0.12,
                              color=COLOR_REVOLUTE, alpha=JOINT_REVOLUTE_ALPHA)
                an  = axis / np.linalg.norm(axis)
                p_s = p_joint - an * JOINT_REVOLUTE_AXIS_LENGTH / 2
                p_e = p_joint + an * JOINT_REVOLUTE_AXIS_LENGTH / 2
                draw_line(ax, p_s, p_e, '-', COLOR_REVOLUTE_AXIS, lw=JOINT_AXIS_THICKNESS)
            draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)

        elif jtype == 2:  # Prismatic
            if np.linalg.norm(axis) > 1e-8:
                draw_box(ax, p_joint, axis, width=0.05, height=0.05, length=0.10,
                         color=COLOR_PRISMATIC, alpha=JOINT_PRISMATIC_ALPHA)
                an  = axis / np.linalg.norm(axis)
                p_s = p_joint - an * JOINT_PRISMATIC_AXIS_LENGTH / 2
                p_e = p_joint + an * JOINT_PRISMATIC_AXIS_LENGTH / 2
                draw_line(ax, p_s, p_e, '-', COLOR_PRISMATIC_AXIS, lw=JOINT_AXIS_THICKNESS)
                if abs(value) > 1e-6:
                    p_ext = p_joint + an * value
                    draw_line(ax, p_joint, p_ext,   '--', COLOR_PRISMATIC_EXTENSION,
                              lw=JOINT_AXIS_THICKNESS, alpha=0.8)
                    draw_line(ax, p_ext,   p_child, '-',  COLOR_LINK, lw=LINK_THICKNESS)
                else:
                    draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)

        elif jtype == 4:  # Cylindrical
            if np.linalg.norm(axis) > 1e-8:
                draw_cylinder(ax, p_joint, axis, radius=0.04, length=0.14,
                              color=COLOR_CYLINDERICAL, alpha=JOINT_CYLINDERICAL_ALPHA)
                an  = axis / np.linalg.norm(axis)
                p_s = p_joint - an * JOINT_CYLINDERICAL_AXIS_LENGTH / 2
                p_e = p_joint + an * JOINT_CYLINDERICAL_AXIS_LENGTH / 2
                draw_line(ax, p_s, p_e, '-', COLOR_CYLINDERICAL_AXIS, lw=JOINT_AXIS_THICKNESS)
            draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)

        else:  # Unknown
            ax.scatter(*p_joint, color=COLOR_UNKNOWN, s=JOINT_UNKNOWN_SIZE, marker='D',
                       edgecolors=COLOR_UNKNOWN_EDGE, linewidths=1.5)
            draw_line(ax, p_joint, p_child, '-', COLOR_LINK, lw=LINK_THICKNESS)

    # --- End-effector frames ---
    for fname, frame in frames.items():
        p_frame = frame['pos']
        p_link  = links[frame['parent']]['pos']
        draw_line(ax, p_link, p_frame, '-',  COLOR_LINK,       lw=LINK_THICKNESS,       alpha=1.0)
        draw_line(ax, p_link, p_frame, ':',  COLOR_FRAME_LINE, lw=FRAME_LINE_THICKNESS, alpha=0.7)
        draw_frame(ax, p_frame, frame['quat'], scale=0.12)
        ax.text(p_frame[0], p_frame[1], p_frame[2] + 0.05, fname,
                fontsize=9, color=COLOR_FRAME_LINE, weight='bold')

# -------------------------------
# Axis limits
# -------------------------------
def _compute_axis_limits(all_frames):
    all_points = np.array([
        link['pos']
        for _, links, _, _ in all_frames
        for link in links.values()
    ])

    max_range = np.array([
        all_points[:, 0].max() - all_points[:, 0].min(),
        all_points[:, 1].max() - all_points[:, 1].min(),
        all_points[:, 2].max() - all_points[:, 2].min()
    ]).max() / 2.0 * 1.2
    
    mid = np.array([
        (all_points[:, 0].max() + all_points[:, 0].min()) * 0.5,
        (all_points[:, 1].max() + all_points[:, 1].min()) * 0.5,
        (all_points[:, 2].max() + all_points[:, 2].min()) * 0.5,
    ])

    return (mid[0] - max_range, mid[0] + max_range,
            mid[1] - max_range, mid[1] + max_range,
            mid[2] - max_range, mid[2] + max_range)

def _set_axes(ax, links, fixed_limits=None):
    ax.set_xlabel("X", fontsize=11)
    ax.set_ylabel("Y", fontsize=11)
    ax.set_zlabel("Z", fontsize=11)

    if fixed_limits is not None:
        xl, xh, yl, yh, zl, zh = fixed_limits
    else:
        pts = np.array([l['pos'] for l in links.values()])
        max_range = np.array([
            pts[:, 0].max() - pts[:, 0].min(),
            pts[:, 1].max() - pts[:, 1].min(),
            pts[:, 2].max() - pts[:, 2].min()
        ]).max() / 2.0
        mid_x = (pts[:, 0].max() + pts[:, 0].min()) * 0.5
        mid_y = (pts[:, 1].max() + pts[:, 1].min()) * 0.5
        mid_z = (pts[:, 2].max() + pts[:, 2].min()) * 0.5
        xl, xh = mid_x - max_range, mid_x + max_range
        yl, yh = mid_y - max_range, mid_y + max_range
        zl, zh = mid_z - max_range, mid_z + max_range

    ax.set_xlim(xl, xh)
    ax.set_ylim(yl, yh)
    ax.set_zlim(zl, zh)

# -------------------------------
# Visualization
# -------------------------------
def visualize_robot(clog_path):
    header, all_frames = read_robot_clog(clog_path)
    robot_name         = header.get('robot', clog_path.stem)

    if len(all_frames) == 1:
        # ---- Single frame - static plot ----
        _, links, joints, frames = all_frames[0]

        fig = plt.figure(figsize=(14, 10))
        ax  = fig.add_subplot(111, projection='3d')
        ax.set_title(f"Robot Kinematic State - {robot_name}", fontsize=14, weight='bold')

        _draw_robot(ax, links, joints, frames)
        _set_axes(ax, links)

        plt.tight_layout()
        plt.show()

    else:
        # ---- Multi-frame - static plot + frame slider + end-effector trace ----
        fig = plt.figure(figsize=(14, 11))
        ax  = fig.add_subplot(111, projection='3d')
        plt.subplots_adjust(bottom=0.12)

        # Fixed axis limits - computed once, never change during animation
        fixed_limits = _compute_axis_limits(all_frames)

        # End-effector trace positions - one per frame, collected once
        trace_points = np.array([
            next(iter(frames.values()))['pos'].copy()
            for _, _, _, frames in all_frames
        ])  # shape (N, 3)

        def draw_frame_idx(idx):
            ax.cla()
            timestamp_us, links, joints, frames = all_frames[idx]
            timestamp_s = timestamp_us / 1e6

            ax.set_title(
                f"Robot Kinematic State - {robot_name} "
                f"[frame {idx + 1}/{len(all_frames)},  t = {timestamp_s:.4f} s]",
                fontsize=13, weight='bold'
            )

            _draw_robot(ax, links, joints, frames)

            # Trace - draw all positions up to and including the current frame
            if idx > 0:
                pts = trace_points[:idx + 1]
                ax.plot(
                    pts[:, 0], pts[:, 1], pts[:, 2],
                    color=COLOR_EE_TRACE,
                    linewidth=2,
                    alpha=ALPHA_EE_TRACE,
                    linestyle='-'
                )

            _set_axes(ax, links, fixed_limits=fixed_limits)
            fig.canvas.draw_idle()

        draw_frame_idx(0)

        # Frame slider
        ax_slider = plt.axes([0.15, 0.04, 0.65, 0.03])
        slider    = widgets.Slider(
            ax_slider, 'Frame',
            0, len(all_frames) - 1,
            valinit=0, valstep=1
        )
        slider.on_changed(lambda val: draw_frame_idx(int(val)))

        # Play button
        ax_play = plt.axes([0.82, 0.03, 0.08, 0.05])
        btn     = widgets.Button(ax_play, 'Play')

        def play(_event):
            for i in range(len(all_frames)):
                slider.set_val(i)
                plt.pause(0.08)

        btn.on_clicked(play)

        plt.show()

# -------------------------------
# Entry point
# -------------------------------
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python robot_plotter.py <name_without_extension>")
        sys.exit(1)

    proj_root = pathlib.Path(__file__).resolve().parents[2]
    clog_path = proj_root / "results" / "kinematics" / (sys.argv[1] + ".clog")

    if not clog_path.exists():
        print(f"File not found: {clog_path}")
        sys.exit(1)

    visualize_robot(clog_path)