import sys
import ntpath
import os
import re
import ast
import math
import operator as op

from math import pi, sqrt
from dataclasses import dataclass

_ALLOWED_OPERATORS = {
    ast.Add: op.add,
    ast.Sub: op.sub,
    ast.Mult: op.mul,
    ast.Div: op.truediv,
    ast.Pow: op.pow,
    ast.USub: op.neg
}

@dataclass
class Link:
    id: int
    name: str
    child: int
    length: float
    mass: float

@dataclass
class Joint:
    id: int
    type: str
    parent: str
    child: str
    parentId: int
    childId: int
    limits: list
    axis: list
    home: float
    init: float

@dataclass
class PartBlock:
    type: str
    subtype: str
    content: list[str]

## ========== Helper Functions ==========
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


def parse_bracket(line: str) -> list[float]:
    items = re.findall(r"[-+*\/\w().]+", line)
    value_list = []

    for i in items:
        try:
            value_list.append(safe_eval(i))
        except:
            raise ValueError("[ ERROR ] | An attribute must be a number")

    return value_list

def parse_parent_child(line: str) -> list[str]:
    items = re.findall(r"[-+]?\d*\.?\d+|[a-z\/\\\d_]+", line.lower())
    value_list = []

    for i in items:
        value_list.append(i)

    return value_list

def collect_block(lines : list[str]) -> list[PartBlock]:
    blocks = []
    working_block = None
    in_block = False

    for line in lines:
        line = line.strip().lower()

        if line.startswith("\\"):   # Block Start
            parts = line.strip().lstrip("\\").split()

            type = parts[0]
            subtype = parts[1]

            if not line.endswith("{"):  # Skip for the 'base' link case
                blocks.append(PartBlock(type, subtype, []))
                continue

            working_block = PartBlock(type, subtype, [])
            in_block = True
            continue
            
        if in_block:
            if line.startswith("}"): # Block End
                blocks.append(working_block)
                working_block = None
                in_block = False
            else:
                if working_block is not None:   # Should always pass
                    if line.count("{") != 0:
                        raise ValueError("[ ERROR ] | Unclosed curly brace '{'")
                    working_block.content.append(line)

    return blocks
## ======================================

def parse_file(filename : str) -> tuple[str, list[Link], list[Joint]]:
    rob_name = ""
    links = []
    joints = []

    # ===== File Extension Check =====
    file = ntpath.basename(filename)
    if not file.endswith(".rob"):
        raise ValueError("[ ERROR ] | Invalid file type. Use '.rob' config files")
    
    abs_in_path = os.path.abspath(filename)
    if not os.path.isfile(abs_in_path):
        raise FileNotFoundError(f" [ ERROR ] | Could not find the file at: {abs_in_path}")

    # ===== Initial Parsing =====
    isNamed = False

    with open(abs_in_path) as f:
        for line in f:
            # Empty line
            if not line:    
                continue

            startChar = line[0]

            match startChar:
                case ('#'): # Comments line
                    continue

                case ('>'): # Robot Name
                    if isNamed:
                        raise ValueError("[ ERROR ] | The robot can only be named once.")
                    
                    rob_name = line.lstrip(">").strip().lower()
                    isNamed = True

                case ('\\'): # Link/Joint
                    if not isNamed:
                        raise ValueError("[ ERROR ] | .rob file should start with the robot name.")
                    

    # ===== Block Parsing =====
    with open(abs_in_path) as f:
        # Collect curly brace block
        blocks = collect_block(f.readlines())

    valid_types = {
        "revolute",
        "prismatic"
    }

    link_index = 0
    joint_index = 0

    for block in blocks:
        fields = {}
        for block_line in block.content:
            if not block_line:  # Empty line
                continue
            key, value = block_line.split("=", 1)
            key = key.lstrip(".").strip()
            value = value.strip()
            fields[key] = value

        match block.type:
            case ("link"):      # Parse Link
                link_name = block.subtype
                link_length = parse_bracket(fields.get("length", "-1"))[0]
                link_mass = parse_bracket(fields.get("mass", "0"))[0]

                # Attribute checks
                if (link_length < 0) and (link_name != "base"): # Non-negative length
                    raise ValueError("[ ERROR ] | Non-base link must have a valid length")
                for l in links:                                 # Unique link names
                    if l.name == link_name:
                        raise ValueError("[ ERROR ] | Link names must be unique")
                
                links.append(Link(id=link_index, name=link_name, child=-1, length=link_length, mass=link_mass))
                
                link_index = link_index + 1
                continue
 
            case ("joint"):     # Parse Joint
                joint_type = block.subtype
                joint_links = parse_parent_child(fields.get("links", []))
                joint_parent = -1
                joint_child = -1
                joint_limits = parse_bracket(fields.get("limits", [-pi, pi]))
                joint_axis = parse_bracket(fields.get("axis", [0, 0, 1]))
                joint_home = parse_bracket(fields.get("home", "0"))[0]
                joint_init = parse_bracket(fields.get("init", "0"))[0]
                for link in links:
                    if link.name == joint_links[0]:
                        joint_parent = link.id
                    if link.name == joint_links[1]:
                        joint_child = link.id


                # Attribute checks
                if not valid_types.__contains__(joint_type):                                        # Valid joint type
                    raise ValueError("[ ERROR ] | Invlid joint type")
                if joint_links is []:                                                               # Parent/child links must exist
                    raise ValueError("[ ERROR ] | Joint must have a parent and child")
                if (joint_parent == -1) or (joint_child == -1):                                     # Parent/child links index must exist
                    raise ValueError("[ ERROR ] | Joint must have a parent and child")
                if joint_limits[0] > joint_limits[1]:                                               # Min limit < Max limit
                    raise ValueError("[ ERROR ] | Joint min/max limits must be valid")
                if abs(1 - sqrt(joint_axis[0]**2 + joint_axis[1]**2 + joint_axis[2]**2)) > 0.001:   # Normalized axis
                    raise ValueError("[ ERROR ] | Joint axis must be normalized")
                if not ((joint_limits[0] <= joint_init) and (joint_init <= joint_limits[1])):       # Initial value is valid
                    raise ValueError("[ ERROR ] | Joint initial value must be valid")

                joints.append(Joint(id=joint_index, type=joint_type, parent=joint_links[0], parentId=joint_parent, childId=joint_child, child=joint_links[1], limits=joint_limits, axis=joint_axis, home=joint_home, init=joint_init))

                # Set associated link's child joint id
                for l in links:
                    if l.name == joint_links[0]:
                        if l.child != -1:
                            raise ValueError("[ ERROR ] | A link can only have 1 child joint")
                        
                        l.child = joint_index
                        break

                joint_index = joint_index + 1
                continue

            case _:             # Unknown case
                raise ValueError("[ ERROR ] | Invlid part type")
    return rob_name, links, joints

def generate_code(name : str, links : list[Link], joints : list[Joint]) -> str:
    L = len(links)
    J = len(joints)
    
    desc_sep = "//" + "-" * (len(name)*3 - 3)


    # Header Guard
    code = "#pragma once\n\n"

    # Includes
    code += "#include \"cobalt/kinematics/joint.hpp\"\n"
    code += "#include \"cobalt/kinematics/link.hpp\"\n"
    code += "#include \"cobalt/kinematics/robot_chain.hpp\"\n\n"

    # Description
    code += desc_sep + "\n"
    code += f"// {name.center(len(name)*3-1 - 3)}\n"
    code += desc_sep + "\n"
    code += f"/** \n"
    code += f" * This is an automatically generated '{name}' RobotChain from its .rob file.\n"
    code += f" * Include '{name}.hpp' in your project upon calling it to access and use the created '{name}' chain.\n"
    code += f" * Do not touch this file for any edits and use the .rob file for any edits.\n"
    code += f" */ \n"
    
    code += f"namespace cobalt::kinematics::robot {{\n\n"

    # Make Links
    code += f"  const std::array<Link, {L}> {name}_links = {{\n"
    for link in links:
        code += f"      Link(\"{link.name}\", {float(link.length)}f, {link.id}, {link.child}, {float(link.mass)}f),\n"
    code += f"  }};\n\n"

    # Make Joints
    code += f"  const std::array<Joint, {J}> {name}_joints = {{\n"
    for joint in joints:
        code += f"""      Joint(JointType::{joint.type.capitalize()}, {joint.id}, {joint.parentId}, {joint.childId},
            cobalt::math::linear_algebra::Vector<3>({float(joint.axis[0])}f, {float(joint.axis[1])}f, {float(joint.axis[2])}f),
            {float(joint.limits[0])}f, {float(joint.limits[1])}f, {float(joint.init)}f, {float(joint.home)}f),\n"""
    code += f"  }};\n\n"

    # Make Robot Chain
    code += f"  inline RobotChain<{L}, {J}> {name}({name}_links, {name}_joints);\n\n"

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
    
    print(f"[ SUCCESS ] | Generated {name}.hpp RobotChain header: {out_path}")

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
        files_dir = os.path.join(script_dir, default_dir)
        out_file = ""
        
        for entry in os.listdir(files_dir):
            full_path = os.path.join(files_dir, entry)
            if os.path.isfile(full_path):
                files.append(full_path)


    for file in files:
        [name, links, joints] = parse_file(file)

        code = generate_code(name, links, joints)
        generate_header(name, code, out_file)
    
