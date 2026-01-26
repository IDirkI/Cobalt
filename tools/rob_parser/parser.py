import sys
import os
import re
import ast
import math
import operator as op

from defaults import *

from dataclasses import dataclass
from typing import List
from math import sqrt

_ALLOWED_OPERATORS = {
    ast.Add: op.add,
    ast.Sub: op.sub,
    ast.Mult: op.mul,
    ast.Div: op.truediv,
    ast.Pow: op.pow,
    ast.USub: op.neg
}

## ========== DATA ==========
@dataclass
class Link:
    id: int                     # Unique identifier for the link
    name: str                   # Name of the link
    mass: float                 # Mass of the link
    inertia: List[List[float]]  # 3x3 inertia matrix
    com_xyz: List[float]        # COM in x-y-z coordinates
    com_rpy: List[float]        # COM in roll-pitch-yaw angles
    virtual: bool = False       # Virtualness of a link

@dataclass
class Joint:
    id: int                      # Unique identifier for the joint
    parent_id: int               # ID of the parent link
    child_id: int                # ID of the child link
    type: str                    # Type of joint (e.g., revolute, prismatic)
    axis: List[float]            # Axis of rotation or translation
    limits: List[float]          # Joint limits (min, max)
    init: float                  # Initial joint value
    home: float                  # Home(reference) position of the joint
    origin_xyz: List[float]      # Origin in x-y-z coordinates
    origin_rpy: List[float]      # Origin in roll-pitch-yaw angles
    comp_type: str               # True if the joint is a compound collection of R & P
    comp_index: int = -1         # Index within the compound joint

@dataclass
class Frame:
    id: int                     # ID of the frame
    link_id: int                # ID of the link the frame is attached to
    name: str                   # Name of the frame
    origin_xyz: List[float]     # Origin in x-y-z coordinates
    origin_rpy: List[float]     # Origin in roll-pitch-yaw angles

@dataclass
class PartBlock:
    def __init__(self, type: str, subtype: str):
        self.type = type
        self.subtype = subtype
        self.fields = {}
        self.subblocks = {}

## ========== MATH ==========
def safe_eval(expr: str) -> float:
    def _eval(node):
        if isinstance(node, ast.Constant):
            value = node.value
            if isinstance(value, (int, float)):
                return value
            else:
                raise ValueError("[ ERROR ] | Invalid expresison")
        elif isinstance(node, ast.UnaryOp) and (type(node.op) in _ALLOWED_OPERATORS):
            return float(_ALLOWED_OPERATORS[type(node.op)](_eval(node.operand)))
        elif isinstance(node, ast.BinOp) and (type(node.op) in _ALLOWED_OPERATORS):
            return float(_ALLOWED_OPERATORS[type(node.op)](_eval(node.left), _eval(node.right)))
        elif isinstance(node, ast.Name):
            if node.id == "pi":
                return pi
            elif node.id == "e":
                return math.e
            else:
                raise ValueError("[ ERROR ] | Invalid expresison")
        else:
            raise ValueError("[ ERROR ] | Invalid expresison")
        
    node = ast.parse(expr, mode='eval').body
    return _eval(node)

## ========== HELPER FUNCTIONS/PARSERS ==========
def collect_blocks(lines):
    blocks = []
    current = None
    brace_depth = 0
    subblock_stack = []

    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue

        lower = line.lower()

        # -------- Block start --------
        if lower.startswith("\\"):
            parts = lower.lstrip("\\").split()
            if len(parts) < 2:
                raise ValueError("[ ERROR ] | Invalid block declaration")

            block_type, subtype = parts[0], parts[1]

            if lower.endswith("{"):
                current = PartBlock(block_type, subtype)
                brace_depth = 1
                subblock_stack.clear()
            else:
                blocks.append(PartBlock(block_type, subtype))
            continue

        if current is None:
            continue

        # -------- Inside block --------
        # Start of named subblock
        if "=" in line and line.endswith("{"):
            name = line.split("=", 1)[0].strip().lstrip(".")
            current.subblocks[name] = []
            subblock_stack.append(name)
            brace_depth += 1
            continue

        # Closing brace
        if line == "}":
            brace_depth -= 1
            if subblock_stack:
                subblock_stack.pop()
            else:
                blocks.append(current)
                current = None
            continue

        # Regular field or subblock content
        if subblock_stack:
            current.subblocks[subblock_stack[-1]].append(lower)
        else:
            k, v = line.split("=", 1)
            current.fields[k.strip().lstrip(".")] = v.strip()

    return blocks

def parse_bracket(line: str) -> List[float]:
    items = re.findall(r"[-+*\/\w().]+", line)
    value_list = []

    for i in items:
        try:
            value_list.append(safe_eval(i))
        except:
            raise ValueError("[ ERROR ] | An attribute must be a number")

    return value_list

def parse_inertia(value: str):
    mat = ast.literal_eval(value)

    if len(mat) != 3 or any(len(row) != 3 for row in mat):
        raise ValueError("[ ERROR ] Inertia must be a 3x3 matrix")

    # symmetry check
    eps = 1e-6
    for i in range(3):
        for j in range(3):
            if abs(mat[i][j] - mat[j][i]) > eps:
                raise ValueError("[ ERROR ] | Inertia matrix must be symmetric")

    # diagonal must be non-negative
    for i in range(3):
        if mat[i][i] < 0:
            raise ValueError("[ ERROR ] | Inertia diagonal must be non-negative")

    return mat

def extract_named_block(lines: List[str], name: str):
    collected = []
    inside = False

    for line in lines:
        if line.startswith(f".{name}") and "{" in line:
            inside = True
            continue

        if inside:
            if line.startswith("}"):
                break
            collected.append(line)

    return collected

def parse_transform_block(lines: List[str]):
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]

    for line in lines:
        if line.startswith(".xyz"):
            xyz = parse_bracket(line.split("=", 1)[1])
        elif line.startswith(".rpy"):
            rpy = parse_bracket(line.split("=", 1)[1])

    return xyz, rpy

def parse_parent_child(line: str) -> List[str]:
    items = re.findall(r"[-+]?\d*\.?\d+|[a-zA-Z\/\\\d_]+", line.lower())
    value_list = []

    for i in items:
        value_list.append(i)

    return value_list

def parse_joint_type(joint: str) -> str:
    key = joint.strip().lower()

    if key not in JOINT_TYPE_ALIASES:
        raise ValueError(
            f"[ ERROR ] | Invalid joint type '{joint}'. "
            f">>> Allowed joint types: fixed/f, revolute/r, prismatic/p, universal/u, spherical/s"
        )

    return JOINT_TYPE_ALIASES[key]

def parse_nested_list(value: str) -> List:
    # Remove outer brackets and whitespace
    value = value.strip()
    if not (value.startswith('[') and value.endswith(']')):
        raise ValueError("[ ERROR ] | Value must be a list")
    
    # Remove outer brackets
    inner = value[1:-1].strip()
    
    # Split by '], [' to get individual sublists
    # Handle the case where there are nested lists
    result = []
    depth = 0
    current = ""
    
    for char in inner:
        if char == '[':
            depth += 1
            current += char
        elif char == ']':
            depth -= 1
            current += char
            if depth == 0 and current.strip():
                # Parse this sublist
                result.append(parse_sublist(current.strip()))
                current = ""
        elif char == ',' and depth == 0:
            if current.strip() and not current.strip().startswith('['):
                # Single value
                result.append(safe_eval(current.strip()))
                current = ""
        else:
            current += char
    
    # Don't forget the last item
    if current.strip():
        if current.strip().startswith('['):
            result.append(parse_sublist(current.strip()))
        else:
            result.append(safe_eval(current.strip()))
    
    return result

def parse_sublist(value: str) -> List[float]:

    value = value.strip()
    if not (value.startswith('[') and value.endswith(']')):
        raise ValueError("[ ERROR ] | Sublist must be enclosed in brackets")
    
    inner = value[1:-1].strip()
    elements = [elem.strip() for elem in inner.split(',')]
    
    return [safe_eval(elem) for elem in elements if elem]

def parse_multi_limits(value) -> List[List[float]]:
    # First check if it's already a list (from defaults)
    if isinstance(value, list):
        return [[safe_eval(str(v[0])) if not isinstance(v[0], (int, float)) else float(v[0]), 
                 safe_eval(str(v[1])) if not isinstance(v[1], (int, float)) else float(v[1])] 
                for v in value]
    
    # Parse the string
    try:
        mat = parse_nested_list(value)
    except:
        raise ValueError("[ ERROR ] | Failed to parse multi-DOF limits")
    
    # Check if it's a list of lists
    if not isinstance(mat, list):
        raise ValueError("[ ERROR ] | Multi-DOF limits must be a list of limit pairs")
    
    result = []
    for limit_pair in mat:
        if not isinstance(limit_pair, list) or len(limit_pair) != 2:
            raise ValueError("[ ERROR ] | Each limit must be [min, max]")
        
        # Use safe_eval to handle any remaining expressions
        min_val = limit_pair[0] if isinstance(limit_pair[0], (int, float)) else safe_eval(str(limit_pair[0]))
        max_val = limit_pair[1] if isinstance(limit_pair[1], (int, float)) else safe_eval(str(limit_pair[1]))
        result.append([min_val, max_val])
    
    return result

def parse_multi_axes(value) -> List[List[float]]:
    # Handle if it's already a list (from defaults)
    if isinstance(value, list):
        mat = value
    else:
        try:
            mat = parse_nested_list(value)
        except:
            raise ValueError("[ ERROR ] | Failed to parse multi-axis")
    
    if not isinstance(mat, list):
        raise ValueError("[ ERROR ] | Multi-axis must be a list of axes")
    
    result = []
    for axis in mat:
        if not isinstance(axis, list) or len(axis) != 3:
            raise ValueError("[ ERROR ] | Each axis must have 3 components")
        
        # Handle each component
        parsed_axis = []
        for i in range(3):
            if isinstance(axis[i], (int, float)):
                parsed_axis.append(float(axis[i]))
            else:
                parsed_axis.append(safe_eval(str(axis[i])))
        
        # Normalize check
        norm = sqrt(sum(a**2 for a in parsed_axis))
        if abs(norm - 1.0) > JOINT_AXIS_THRESHOLD:
            raise ValueError(f"[ ERROR ] | Each axis must be normalized (got norm={norm})")
        
        result.append(parsed_axis)
    
    return result

def parse_multi_values(value) -> List[float]:
    # Handle if it's already a list (from defaults)
    if isinstance(value, list):
        return [safe_eval(str(v)) if not isinstance(v, (int, float)) else float(v) for v in value]
    
    try:
        vals = parse_nested_list(value)
        # Flatten if it's a list of lists (shouldn't be for init/home)
        if vals and isinstance(vals[0], list):
            raise ValueError("[ ERROR ] | init/home values must be a flat list")
    except:
        raise ValueError("[ ERROR ] | Failed to parse multi-DOF values")
    
    if not isinstance(vals, list):
        raise ValueError("[ ERROR ] | Multi-DOF values must be a list")
    
    result = []
    for v in vals:
        if isinstance(v, (int, float)):
            result.append(float(v))
        else:
            result.append(safe_eval(str(v)))
    
    return result

def pascal_case(str : str) -> str:
    s = str.replace("_", " ").replace("-", " ");
    s = s.title();
    s = s.replace(" ", "");
    return s


## ========== TOP-LEVEL PARSER ==========
def parse_rob_file(file_path: str):
    # --- Params ---
    robot_name = None
    links: List[Link] = []
    joints: List[Joint] = []
    frames: List[Frame] = []

    name_to_id = {}

    # ===== Line Extraction =====
    with open(file_path, "r") as f:
        lines = [l.strip() for l in f if l.strip() and not l.startswith("#")]

    # ===== Robot Name =====
    for line in lines:
        if line.startswith(">"):
            if robot_name is not None:
                raise ValueError("[ ERROR ] | Robot named more than once")
            robot_name = line[1:].strip()
            break

    if robot_name is None:
        raise ValueError("[ ERROR ] | Robot must have a name defined using '>'")
    
    # ===== Blocks =====
    blocks = collect_blocks(lines)

    # ===== Parsing =====
    # === Links === 
    for block in blocks:
        if block.type != "link":
            continue

        name = block.subtype
        
        mass = safe_eval(block.fields.get("mass", LINK_DEFAULT_MASS))
        inertia = parse_inertia(block.fields.get("inertia", LINK_DEFAULT_INERTIA))

        if "com" in block.subblocks:
            com_xyz, com_rpy = parse_transform_block(block.subblocks["com"])
        else:
            com_xyz = LINK_DEFAULT_COM_XYZ
            com_rpy = LINK_DEFAULT_COM_RPY

        link_id = len(links)
        name_to_id[name] = link_id

        # Attribute checks
        for l in links:                                 # Unique link names
            if l.name == name:
                raise ValueError("[ ERROR ] | Link names must be unique")

        links.append(
            Link(
                id=link_id,
                name=name,
                mass=mass,
                inertia=inertia,
                com_xyz=com_xyz,
                com_rpy=com_rpy,
                virtual=False
            )
        )
    
    # === Joints === 
    for block in blocks:
        if block.type != "joint":
            continue

        joint_type = parse_joint_type(block.subtype)
        
        parent_name, child_name = parse_parent_child(block.fields["links"])
        parent_id = name_to_id[parent_name]
        child_id = name_to_id[child_name]
        
        if "origin" in block.subblocks:
            origin_xyz, origin_rpy = parse_transform_block(block.subblocks["origin"])
        else:
            origin_xyz = JOINT_DEFAULT_ORIGIN_XYZ
            origin_rpy = JOINT_DEFAULT_ORIGIN_RPY

        # Handle different joint types
        if joint_type == "universal":
            # Parse universal joint parameters
            axes_str = block.fields.get("axis", None)
            if axes_str is None:
                axes = UNIVERSAL_DEFAULT_AXES
            else:
                axes = parse_multi_axes(axes_str)
            
            limits_str = block.fields.get("limits", None)
            if limits_str is None:
                limits = UNIVERSAL_DEFAULT_LIMITS
            else:
                limits = parse_multi_limits(limits_str)
            
            init_str = block.fields.get("init", None)
            if init_str is None:
                init_vals = UNIVERSAL_DEFAULT_INIT
            else:
                init_vals = parse_multi_values(init_str)
            
            home_str = block.fields.get("home", None)
            if home_str is None:
                home_vals = UNIVERSAL_DEFAULT_HOME
            else:
                home_vals = parse_multi_values(home_str)
            
            # Validation
            if len(axes) != 2:
                raise ValueError("[ ERROR ] | Universal joint must have exactly 2 axes")
            if len(limits) != 2 or len(init_vals) != 2 or len(home_vals) != 2:
                raise ValueError("[ ERROR ] | Universal joint must have 2 DOF parameters")
            
            # Axis orthogonality validation
            axis0 = axes[0]
            axis1 = axes[1]
            dot_prod = axis0[0]*axis1[0] + axis0[1]*axis1[1] + axis0[2]*axis1[2]
            if abs(dot_prod) > JOINT_AXIS_THRESHOLD:
                raise ValueError("[ ERROR ] | Universal joint axes must be orthogonal")
            
            # Create 2 revolute joints + 1 invisible link
            base_joint_id = len(joints)
            invisible_link_id = len(links)
            
            # Create invisible intermediate link
            links.append(Link(
                id=invisible_link_id,
                name=f"_u_link_{base_joint_id}",
                mass=0.0,
                inertia=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                com_xyz=[0.0, 0.0, 0.0],
                com_rpy=[0.0, 0.0, 0.0],
                virtual=True
            ))
            name_to_id[f"_u_link_{base_joint_id}"] = invisible_link_id
            
            # Create revolute joints
            for i, (axis, limit, init_val, home_val) in enumerate(zip(axes, limits, init_vals, home_vals)):
                if limits[i][0] > limits[i][1]:
                    raise ValueError(f"[ ERROR ] | Universal joint DOF {i}: max limit < min limit")
                if not ((limits[i][0] <= init_vals[i]) and (init_vals[i] <= limits[i][1])):
                    raise ValueError(f"[ ERROR ] | Universal joint DOF {i}: initial value must be valid")
                
                current_parent = parent_id if i == 0 else invisible_link_id
                current_child = invisible_link_id if i == 0 else child_id
                current_origin_xyz = origin_xyz if i == 0 else [0.0, 0.0, 0.0]
                current_origin_rpy = origin_rpy if i == 0 else [0.0, 0.0, 0.0]
                
                joints.append(Joint(
                    id=len(joints),
                    parent_id=current_parent,
                    child_id=current_child,
                    type="revolute",
                    axis=axis,
                    limits=limit,
                    init=init_val,
                    home=home_val,
                    origin_xyz=current_origin_xyz,
                    origin_rpy=current_origin_rpy,
                    comp_type=joint_type,
                    comp_index=i
                ))
        elif joint_type == "spherical":
            # Parse spherical joint parameters
            limits_str = block.fields.get("limits", None)
            if limits_str is None:
                limits = SPHERICAL_DEFAULT_LIMITS
            else:
                limits = parse_multi_limits(limits_str)
            
            init_str = block.fields.get("init", None)
            if init_str is None:
                init_vals = SPHERICAL_DEFAULT_INIT
            else:
                init_vals = parse_multi_values(init_str)
            
            home_str = block.fields.get("home", None)
            if home_str is None:
                home_vals = SPHERICAL_DEFAULT_HOME
            else:
                home_vals = parse_multi_values(home_str)
            
            # Spherical always uses X, Y, Z axes in parent frame
            axes = SPHERICAL_DEFAULT_AXES
            
            # Validation
            if len(limits) != 3 or len(init_vals) != 3 or len(home_vals) != 3:
                raise ValueError("[ ERROR ] | Spherical joint must have 3 DOF parameters")
            
            # Create 3 revolute joints + 2 invisible links
            base_joint_id = len(joints)
            invisible_link_1_id = len(links)
            invisible_link_2_id = len(links) + 1
            
            # Create first invisible link
            links.append(Link(
                id=invisible_link_1_id,
                name=f"_s_link1_{base_joint_id}",
                mass=0.0,
                inertia=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                com_xyz=[0.0, 0.0, 0.0],
                com_rpy=[0.0, 0.0, 0.0],
                virtual=True
            ))
            name_to_id[f"_s_link1_{base_joint_id}"] = invisible_link_1_id
            
            # Create second invisible link
            links.append(Link(
                id=invisible_link_2_id,
                name=f"_s_link2_{base_joint_id}",
                mass=0.0,
                inertia=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                com_xyz=[0.0, 0.0, 0.0],
                com_rpy=[0.0, 0.0, 0.0],
                virtual=True
            ))
            name_to_id[f"_s_link2_{base_joint_id}"] = invisible_link_2_id
            
            # Create 3 revolute joints
            for i, (axis, limit, init_val, home_val) in enumerate(zip(axes, limits, init_vals, home_vals)):
                if limits[i][0] > limits[i][1]:
                    raise ValueError(f"[ ERROR ] | Spherical joint DOF {i}: max limit < min limit")
                if not ((limits[i][0] <= init_vals[i]) and (init_vals[i] <= limits[i][1])):
                    raise ValueError(f"[ ERROR ] | Spherical joint DOF {i}: initial value must be valid")
                
                if i == 0:
                    current_parent = parent_id
                    current_child = invisible_link_1_id
                    current_origin_xyz = origin_xyz
                    current_origin_rpy = origin_rpy
                elif i == 1:
                    current_parent = invisible_link_1_id
                    current_child = invisible_link_2_id
                    current_origin_xyz = [0.0, 0.0, 0.0]
                    current_origin_rpy = [0.0, 0.0, 0.0]
                else:  # i == 2
                    current_parent = invisible_link_2_id
                    current_child = child_id
                    current_origin_xyz = [0.0, 0.0, 0.0]
                    current_origin_rpy = [0.0, 0.0, 0.0]
                
                joints.append(Joint(
                    id=len(joints),
                    parent_id=current_parent,
                    child_id=current_child,
                    type="revolute",
                    axis=axis,
                    limits=limit,
                    init=init_val,
                    home=home_val,
                    origin_xyz=current_origin_xyz,
                    origin_rpy=current_origin_rpy,
                    comp_type=joint_type,
                    comp_index=i
                ))
        elif joint_type == "cylinderical":
            # Parse cylinderical joint parameters
            axis = parse_bracket(block.fields.get("axis", JOINT_DEFAULT_AXIS))
            
            limits_str = block.fields.get("limits", None)
            if limits_str is None:
                limits = CYLINDERICAL_DEFAULT_LIMITS
            else:
                limits = parse_multi_limits(limits_str)
            
            init_str = block.fields.get("init", None)
            if init_str is None:
                init_vals = CYLINDERICAL_DEFAULT_INIT
            else:
                init_vals = parse_multi_values(init_str)
            
            home_str = block.fields.get("home", None)
            if home_str is None:
                home_vals = CYLINDERICAL_DEFAULT_HOME
            else:
                home_vals = parse_multi_values(home_str)
            
            # Validation
            if len(limits) != 2 or len(init_vals) != 2 or len(home_vals) != 2:
                raise ValueError("[ ERROR ] | Cylinderical joint must have 2 DOF parameters")
            
            # Create 1 revolute joints + 1 prismatic joint + 1 invisible link
            base_joint_id = len(joints)
            invisible_link_id = len(links)
            
            # Create invisible intermediate link
            links.append(Link(
                id=invisible_link_id,
                name=f"_c_link_{base_joint_id}",
                mass=0.0,
                inertia=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                com_xyz=[0.0, 0.0, 0.0],
                com_rpy=[0.0, 0.0, 0.0],
                virtual=True
            ))
            name_to_id[f"_c_link_{base_joint_id}"] = invisible_link_id
            
            # Create revolute joint
            for i, (limit, init_val, home_val) in enumerate(zip(limits, init_vals, home_vals)):
                if limits[i][0] > limits[i][1]:
                    raise ValueError(f"[ ERROR ] | Cylinderical joint DOF {i}: max limit < min limit")
                if not ((limits[i][0] <= init_vals[i]) and (init_vals[i] <= limits[i][1])):
                    raise ValueError(f"[ ERROR ] | Cylinderical joint DOF {i}: initial value must be valid")
                
                current_parent = parent_id if i == 0 else invisible_link_id
                current_child = invisible_link_id if i == 0 else child_id
                current_origin_xyz = origin_xyz if i == 0 else [0.0, 0.0, 0.0]
                current_origin_rpy = origin_rpy if i == 0 else [0.0, 0.0, 0.0]
                
                joints.append(Joint(
                    id=len(joints),
                    parent_id=current_parent,
                    child_id=current_child,
                    type= "revolute" if i == 0 else "prismatic",
                    axis=axis,
                    limits=limit,
                    init=init_val,
                    home=home_val,
                    origin_xyz=current_origin_xyz,
                    origin_rpy=current_origin_rpy,
                    comp_type=joint_type,
                    comp_index=i
                ))
        elif joint_type == "planar":
            # Parse planar joint parameters
            axes_str = block.fields.get("axis", None)
            if axes_str is None:
                axes = PLANAR_DEFAULT_AXES
            else:
                axes = parse_multi_axes(axes_str)
            
            limits_str = block.fields.get("limits", None)
            if limits_str is None:
                limits = PLANAR_DEFAULT_LIMITS
            else:
                limits = parse_multi_limits(limits_str)
            
            init_str = block.fields.get("init", None)
            if init_str is None:
                init_vals = PLANAR_DEFAULT_INIT
            else:
                init_vals = parse_multi_values(init_str)
            
            home_str = block.fields.get("home", None)
            if home_str is None:
                home_vals = PLANAR_DEFAULT_HOME
            else:
                home_vals = parse_multi_values(home_str)
            
            # Validation
            if len(axes) != 3:
                raise ValueError("[ ERROR ] | Planar joint must have exactly 3 axes")
            if len(limits) != 3 or len(init_vals) != 3 or len(home_vals) != 3:
                raise ValueError("[ ERROR ] | Planar joint must have 3 DOF parameters")
            
            # Axis orthogonality validation
            axis0 = axes[0]
            axis1 = axes[1]
            axis2 = axes[2]
            dot_prod1 = axis0[0]*axis1[0] + axis0[1]*axis1[1] + axis0[2]*axis1[2]
            dot_prod2 = axis0[0]*axis2[0] + axis0[1]*axis2[1] + axis0[2]*axis2[2]
            dot_prod3 = axis1[0]*axis2[0] + axis1[1]*axis2[1] + axis1[2]*axis2[2]
            if abs(dot_prod1) > JOINT_AXIS_THRESHOLD or abs(dot_prod2) > JOINT_AXIS_THRESHOLD or abs(dot_prod3) > JOINT_AXIS_THRESHOLD:
                raise ValueError("[ ERROR ] | Planar joint axes must be orthogonal")
            
            # Create 2 prismatic joints + 1 revolute joint + 2 invisible link
            base_joint_id = len(joints)
            invisible_link_1_id = len(links)
            invisible_link_2_id = len(links) + 1
            
            # Create invisible intermediate links
            links.append(Link(
                id=invisible_link_1_id,
                name=f"_e_link1_{base_joint_id}",
                mass=0.0,
                inertia=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                com_xyz=[0.0, 0.0, 0.0],
                com_rpy=[0.0, 0.0, 0.0],
                virtual=True
            ))
            name_to_id[f"_e_link1_{base_joint_id}"] = invisible_link_1_id

            links.append(Link(
                id=invisible_link_1_id,
                name=f"_e_link2_{base_joint_id}",
                mass=0.0,
                inertia=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                com_xyz=[0.0, 0.0, 0.0],
                com_rpy=[0.0, 0.0, 0.0],
                virtual=True
            ))
            name_to_id[f"_e_link2_{base_joint_id}"] = invisible_link_1_id
            
            # Create prismatic and revolute joints
            for i, (axis, limit, init_val, home_val) in enumerate(zip(axes, limits, init_vals, home_vals)):
                if limits[i][0] > limits[i][1]:
                    raise ValueError(f"[ ERROR ] | Planar joint DOF {i}: max limit < min limit")
                if not ((limits[i][0] <= init_vals[i]) and (init_vals[i] <= limits[i][1])):
                    raise ValueError(f"[ ERROR ] | Planar joint DOF {i}: initial value must be valid")
                
                if i == 0:
                    current_parent = parent_id
                    current_child = invisible_link_1_id
                    current_origin_xyz = origin_xyz
                    current_origin_rpy = origin_rpy
                    currnt_type = "prismatic"
                elif i == 1:
                    current_parent = invisible_link_1_id
                    current_child = invisible_link_2_id
                    current_origin_xyz = [0.0, 0.0, 0.0]
                    current_origin_rpy = [0.0, 0.0, 0.0]
                    currnt_type = "prismatic"
                else:   # i == 2
                    current_parent = invisible_link_2_id
                    current_child = child_id
                    current_origin_xyz = [0.0, 0.0, 0.0]
                    current_origin_rpy = [0.0, 0.0, 0.0]
                    currnt_type = "revolute"
                
                joints.append(Joint(
                    id=len(joints),
                    parent_id=current_parent,
                    child_id=current_child,
                    type=currnt_type,
                    axis=axis,
                    limits=limit,
                    init=init_val,
                    home=home_val,
                    origin_xyz=current_origin_xyz,
                    origin_rpy=current_origin_rpy,
                    comp_type=joint_type,
                    comp_index=i
                ))
        else:  # Regular joint (fixed, revolute, prismatic)
            axis = parse_bracket(block.fields.get("axis", JOINT_DEFAULT_AXIS))
            limits = parse_bracket(block.fields.get("limits", JOINT_DEFAULT_LIMITS))
            home = safe_eval(block.fields.get("home", JOINT_DEFAULT_HOME))
            initial = safe_eval(block.fields.get("init", JOINT_DEFAULT_INIT))
            
            # Attribute checks
            if (parent_id == -1) or (child_id == -1):
                raise ValueError("[ ERROR ] | Joint must have a parent and child")
            if (parent_id == child_id):
                raise ValueError("[ ERROR ] | Joint parent and child link cannot be the same link")
            if limits[0] > limits[1]:
                raise ValueError("[ ERROR ] | max limit < min limit")
            if abs(1 - sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)) > JOINT_AXIS_THRESHOLD:
                raise ValueError("[ ERROR ] | Joint axis must be normalized")
            if not ((limits[0] <= initial - home) and (initial - home <= limits[1])):
                raise ValueError("[ ERROR ] | Joint initial value must be valid")
            
            joints.append(Joint(
                id=len(joints),
                parent_id=parent_id,
                child_id=child_id,
                type=joint_type,
                axis=axis,
                limits=limits,
                init=initial,
                home=home,
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                comp_type="none",
                comp_index=-1
            ))
    
    # === Frame === 
    for block in blocks:
        if block.type != "frame":
            continue
    
        frame_id = len(frames)
        frame_name = block.subtype
        frame_link_name = block.fields["link"]
        frame_link_id = name_to_id[frame_link_name]
        
        if "origin" in block.subblocks:
            frame_xyz, frame_rpy = parse_transform_block(block.subblocks["origin"])
        else:
            frame_xyz = FRAME_DEFAULT_ORIGIN_XYZ
            frame_rpy = FRAME_DEFAULT_ORIGIN_RPY

        frames.append(
            Frame(
                id=frame_id,
                link_id=frame_link_id,
                name=frame_name,
                origin_xyz=frame_xyz,
                origin_rpy=frame_rpy,
            )
        )

    return robot_name, links, joints, frames

# ========== HEADER GENERATION ==========
def generate_code(name : str, links : List[Link], joints : List[Joint], frames : List[Frame]) -> str:
    L = len(links)
    J = len(joints)
    F = len(frames)
    
    desc_sep = "//" + "-" * (len(name)*3 - 3)


    # Header Guard
    code = "#pragma once\n\n"

    # Includes
    code += "#include \"cobalt/math/linear_algebra/vector/vector.hpp\"\n"
    code += "#include \"cobalt/math/linear_algebra/matrix/matrix.hpp\"\n"
    code += "#include \"cobalt/math/geometry/transform/transform.hpp\"\n\n"

    code += "#include \"cobalt/kinematics/config.hpp\"\n"
    code += "#include \"cobalt/kinematics/core/joint.hpp\"\n"
    code += "#include \"cobalt/kinematics/core/link.hpp\"\n"
    code += "#include \"cobalt/kinematics/core/frame_attachment.hpp\"\n"
    code += "#include \"cobalt/kinematics/model/robot_model.hpp\"\n"
    code += "#include \"cobalt/kinematics/state/robot_state.hpp\"\n"
    code += "#include \"cobalt/kinematics/robot.hpp\"\n\n"
    
    code += f"namespace cobalt::kinematics::robot {{\n\n"

    # Description
    code += desc_sep + "\n"
    code += f"// {name.center(len(name)*3-1 - 3)}\n"
    code += desc_sep + "\n"
    code += f"/** \n"
    code += f" * Generated Robot '{name}' from its .rob file.\n"
    code += f" */\n\n"


    code += f"namespace {name}_internals {{\n"
    # ========== Links ==========
    code += f"// ===== Links =====\n"
    code += f"inline const std::array<Link, {L}> &getLinks() {{\n"
    code += f"  static const std::array<Link, {L}> {name}_links = {{\n"
    for link in links:
        code += f"    Link({link.id}, \"{link.name}\", {link.mass},\n"
        code += f"      cobalt::math::linear_algebra::Matrix<3,3>({{{{ {link.inertia[0][0]}, {link.inertia[1][0]}, {link.inertia[2][0]} }},\n"
        code += f"                                                 {{ {link.inertia[0][1]}, {link.inertia[1][1]}, {link.inertia[2][1]} }},\n"
        code += f"                                                 {{ {link.inertia[0][2]}, {link.inertia[1][2]}, {link.inertia[2][2]} }}}}),\n"
        code += f"      cobalt::math::geometry::Transform<>::eye().rotateZ({link.com_rpy[2]}).rotateY({link.com_rpy[1]}).rotateX({link.com_rpy[0]})\n"
        code += f"                                                .translate(cobalt::math::linear_algebra::Vector<3>({link.com_xyz[0]}, {link.com_xyz[1]}, {link.com_xyz[2]})),\n"
        code += f"      {str(link.virtual).lower()}),\n"
    code += f"  }};\n\n"
    code += f"  return {name}_links;\n"
    code += f"}}\n"

    # ========== Joints ==========
    code += f"// ===== Joints =====\n"
    code += f"inline const std::array<Joint, {J}> &getJoints() {{\n"
    code += f"  static const std::array<Joint, {J}> {name}_joints = {{\n"
    for joint in joints:
        limitsEnabled = (joint.type != "fixed")

        code += f"          Joint({joint.id}, {joint.parent_id}, {joint.child_id}, JointType::{joint.type.capitalize()},\n"
        code += f"              cobalt::math::geometry::Transform<>::eye().translate(cobalt::math::linear_algebra::Vector<3>({joint.origin_xyz[0]}, {joint.origin_xyz[1]}, {joint.origin_xyz[2]}))\n"
        code += f"                                                        .rotateZ({joint.origin_rpy[2]}).rotateY({joint.origin_rpy[1]}).rotateX({joint.origin_rpy[0]}),\n"
        code += f"               cobalt::math::linear_algebra::Vector<3>({float(joint.axis[0])}, {float(joint.axis[1])}, {float(joint.axis[2])}),\n"
        code += f"              JointLimits{{ {float(joint.limits[0])}, {float(joint.limits[1])}, {str(limitsEnabled).lower()} }},\n"
        code += f"               {float(joint.home)},\n"
        code += f"               CompoundJointType::{joint.comp_type.capitalize()},\n"
        code += f"               (cidx_t){int(joint.comp_index)}),\n"
    code += f"  }};\n\n"
    code += f"  return {name}_joints;\n"
    code += f"}}\n\n"

    # ========== Frames ==========
    code += f"// ===== Frames =====\n"
    code += f"inline const std::array<FrameAttachment, {F}> &getFrames() {{\n"
    code += f"  static const std::array<FrameAttachment, {F}> {name}_frames = {{\n"
    for frame in frames:
        code += f"      FrameAttachment({frame.id}, {frame.link_id}, \"{frame.name}\",\n"
        code += f"                      cobalt::math::geometry::Transform<>::eye().rotateZ({frame.origin_rpy[2]}).rotateY({frame.origin_rpy[1]}).rotateX({frame.origin_rpy[0]})\n"
        code += f"                                                                .translate(cobalt::math::linear_algebra::Vector<3>({frame.origin_xyz[0]}, {frame.origin_xyz[1]}, {frame.origin_xyz[2]}))),\n"
    code += f"  }};\n\n"
    code += f"  return {name}_frames;\n"
    code += f"}}\n\n"

    # ========== MAKE MODEL ==========
    code += f"// ===== RobotModel =====\n"
    code += f"inline const RobotModel<{L}, {J}, {F}> &getModel() {{\n"
    code += f"  static const RobotModel<{L}, {J}, {F}> {name}_model(\"{name}\", getLinks(), getJoints(), getFrames());\n"
    code += f"  return {name}_model;\n"
    code += f"}}\n\n"

    code += f"}} // {name}_internals\n\n"

    code += f"inline Robot<{L}, {J}, {F}> make{pascal_case(name)}() {{\n"
    code += f"  "

    # ========== MAKE STATE ==========
    code += f"// ===== RobotState =====\n"
    initial_vals = ", ".join(f"{float(joint.init)}" for joint in joints)
    initial_velocities = ", ".join(["0.0"]*J)

    code += f"  RobotState<{L}, {J}, {F}> {name}_state{{\n"
    code += f"      cobalt::math::linear_algebra::Vector<{J}>({initial_vals}),  // q \n"
    code += f"      cobalt::math::linear_algebra::Vector<{J}>({initial_velocities}),  // dq\n"
    code += f"      {{}},    // J\n"
    code += f"      {{}},    // linkTransforms\n"
    code += f"      {{}},    // frameTransforms\n"
    code += f"      false,  // validJ\n"
    code += f"      false,  // validLinks\n"
    code += f"      false   // validFrames\n"
    code += f"  }};\n\n"

    # ========== MAKE ROBOT ==========
    code += f"  // ===== Robot =====\n"
    code += f"      Robot<{L}, {J}, {F}> {name}({name}_internals::getModel(), {name}_state);\n"
    code += f"      return {name};\n"
    code += f"  }}\n\n"

    code += f"}}; // cobalt::kinematics::robot\n"

    return code

def generate_header(name : str, code : str, out_dir : str):
    script_dir = os.path.dirname(os.path.abspath(file))
    realtive_out_dir = "../../include/cobalt/kinematics/robots"

    if out_dir == "":
        out_path = os.path.join(script_dir, realtive_out_dir, f"{name}.hpp")
    else:
        out_path = os.path.join(out_dir, f"{name}.hpp")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    with open(out_path, "w") as f:
            f.write(code)

    print(f"[ SUCCESS ] | Generated {name}.hpp Robot header: {out_path}")

# ========== MAIN ==========
if __name__ == "__main__":
    files = []

    if len(sys.argv) > 1:   # Called with input argument
        in_file = sys.argv[1]
        files.append(in_file)

        if len(sys.argv) == 2: # Called with only input argument
            out_file = ""
        else:                  # Called with both input & output argument
            out_file = sys.argv[2]

    else:                   # Called with no arguments
        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_dir = "robots/"
        files_dir = os.path.join(os.path.join(script_dir,"../../"), default_dir)
        out_file = "generated/"
        
        for entry in os.listdir(files_dir):
            full_path = os.path.join(files_dir, entry)
            if os.path.isfile(full_path):
                files.append(full_path)


    for file in files:
        [name, links, joints, frames] = parse_rob_file(file)

        code = generate_code(name, links, joints, frames)
        generate_header(name, code, out_file)