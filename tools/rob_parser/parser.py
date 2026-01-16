import sys
import os
import re
import ast
import math
import operator as op

from defaults import *

from dataclasses import dataclass
from typing import List, Tuple
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
            f">>> Allowed joint types: fixed/f, revolute/r, prismatic/p"
        )

    return JOINT_TYPE_ALIASES[key]


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
        joint_id = len(joints)

        axis = parse_bracket(block.fields.get("axis", JOINT_DEFAULT_AXIS))
        limits = parse_bracket(block.fields.get("limits", JOINT_DEFAULT_LIMITS))
        home = safe_eval(block.fields.get("home", JOINT_DEFAULT_HOME))
        initial = safe_eval(block.fields.get("init", JOINT_DEFAULT_INIT))
        
        if "origin" in block.subblocks:
            origin_xyz, origin_rpy = parse_transform_block(block.subblocks["origin"])
        else:
            origin_xyz = JOINT_DEFAULT_ORIGIN_XYZ
            origin_rpy = JOINT_DEFAULT_ORIGIN_RPY

         # Attribute checks
        if (parent_id == -1) or (child_id == -1):                                           # Parent/child links index must exist
            raise ValueError("[ ERROR ] | Joint must have a parent and child")
        if (parent_id == child_id):                                                         # Parent =/= child link
            raise ValueError("[ ERROR ] | Joint parent and child link cannot be the same link")
        if limits[0] > limits[1]:                                                           # Min limit < Max limit
            raise ValueError("[ ERROR ] | max limit < min limit")
        if abs(1 - sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)) > JOINT_AXIS_THRESHOLD:      # Normalized axis
            raise ValueError("[ ERROR ] | Joint axis must be normalized")
        if not ((limits[0] <= initial - home) and (initial - home <= limits[1])):           # Initial value is valid
            raise ValueError("[ ERROR ] | Joint initial value must be valid")

        joints.append(
            Joint(
                id=joint_id,
                parent_id=parent_id,
                child_id=child_id,
                type=joint_type,
                axis=axis,
                limits=limits,
                init=initial,
                home=home,
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
            )
        )
    
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

    # ========== Links ==========
    if(L > 0):
        code += f"// ===== Links =====\n"
        code += f"  const std::array<Link, {L}> {name}_links = {{\n"
        for link in links:
            code += f"    Link({link.id}, \"{link.name}\", {link.mass},\n"
            code += f"      cobalt::math::linear_algebra::Matrix<3,3>({{{{ {link.inertia[0][0]}, {link.inertia[1][0]}, {link.inertia[2][0]} }},\n"
            code += f"                                                 {{ {link.inertia[0][1]}, {link.inertia[1][1]}, {link.inertia[2][1]} }},\n"
            code += f"                                                 {{ {link.inertia[0][2]}, {link.inertia[1][2]}, {link.inertia[2][2]} }}}}),\n"
            code += f"      cobalt::math::geometry::Transform<>::eye().rotateX({link.com_rpy[0]}).rotateY({link.com_rpy[1]}).rotateZ({link.com_rpy[2]}).translate(cobalt::math::linear_algebra::Vector<3>({link.com_xyz[0]}, {link.com_xyz[1]}, {link.com_xyz[2]}))),\n"
        code += f"  }};\n\n"

    # ========== Joints ==========
    if(J > 0):
        code += f"// ===== Joints =====\n"
        code += f"  const std::array<Joint, {J}> {name}_joints = {{\n"
        for joint in joints:
            limitsEnabled = (joint.type != "fixed")

            code += f"      Joint({joint.id}, {joint.parent_id}, {joint.child_id}, JointType::{joint.type.capitalize()},\n"
            code += f"            cobalt::math::geometry::Transform<>::eye().rotateX({joint.origin_rpy[0]}).rotateY({joint.origin_rpy[1]}).rotateZ({joint.origin_rpy[2]}).translate(cobalt::math::linear_algebra::Vector<3>({joint.origin_xyz[0]}, {joint.origin_xyz[1]}, {joint.origin_xyz[2]})),\n"
            code += f"            cobalt::math::linear_algebra::Vector<3>({float(joint.axis[0])}, {float(joint.axis[1])}, {float(joint.axis[2])}),\n"
            code += f"            JointLimits{{ {float(joint.limits[0])}, {float(joint.limits[1])}, {str(limitsEnabled).lower()} }},\n"
            code += f"            {float(joint.home)}),\n"
        code += f"  }};\n\n"

    # ========== Frames ==========
    if(F > 0):
        code += f"// ===== Frames =====\n"
        code += f"  const std::array<FrameAttachment, {F}> {name}_frames = {{\n"
        for frame in frames:
            code += f"      FrameAttachment({frame.id}, {frame.link_id}, \"{frame.name}\",\n"
            code += f"            cobalt::math::geometry::Transform<>::eye().rotateX({frame.origin_rpy[0]}).rotateY({frame.origin_rpy[1]}).rotateZ({frame.origin_rpy[2]}).translate(cobalt::math::linear_algebra::Vector<3>({frame.origin_xyz[0]}, {frame.origin_xyz[1]}, {frame.origin_xyz[2]}))),\n"
        code += f"  }};\n\n"

    # ========== MAKE MODEL ==========
    code += f"// ===== RobotModel =====\n"
    code += f"  inline RobotModel<{L}, {J}, {F}> {name}_model(\"{name}\", {name}_links, {name}_joints, {name}_frames);\n\n"

    # ========== MAKE STATE ==========
    code += f"// ===== RobotState =====\n"
    initial_vals = ", ".join(f"{float(joint.init)}" for joint in joints)
    initial_velocities = ", ".join(["0.0"]*J)

    code += f"  inline RobotState<{L}, {J}, {F}> {name}_state{{\n"
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
    code += f"// ===== Robot =====\n"
    code += f"  inline Robot<{L}, {J}, {F}> {name}({name}_model, {name}_state);\n\n"

    code += f"}}; // cobalt::kinematics::robot\n"

    return code


def generate_header(name : str, code : str, out_dir : str):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    realtive_out_dir = "..\\..\\include\\cobalt\\kinematics\\robots"

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