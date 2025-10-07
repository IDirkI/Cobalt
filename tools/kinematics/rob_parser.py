import sys
import ntpath
import re

from math import pi, sqrt
from dataclasses import dataclass

@dataclass
class Link:
    name: str
    length: float
    child: int

@dataclass
class Joint:
    id: int
    type: str
    parent: str
    child: str
    limits: list
    axis: list
    home: float
    init: float

@dataclass
class PartBlock:
    type: str
    subtype: str
    content: list[str]

def parse_bracket(line: str) -> list[float]:
    items = re.findall(r"[-+]?\d*\.?\d+|[a-z\/\\\d_]+", line.lower())
    value_list = []

    for i in items:
        try:
            value_list.append(float(i))
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


def parse_file(filename : str) -> tuple[list[Link], list[Joint]]:
    rob_name = ""
    links = []
    joints = []

    # ===== File Extension Check =====
    file = ntpath.basename(filename)
    if not file.endswith(".rob"):
        raise ValueError("[ ERROR ] | Invalid file type. Use '.rob' config files")


    # ===== Initial Parsing =====
    isNamed = False

    with open(filename) as f:
        # Collect curly brace block
        blocks = collect_block(f.readlines())

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
                    
                    rob_name = line[1:].strip()
                    isNamed = True

                case ('\\'): # Link/Joint
                    if not isNamed:
                        raise ValueError("[ ERROR ] | .rob file should start with the robot name.")
                    

    # ===== Block Parsing =====
    valid_types = {
        "revolute",
        "prismatic"
    }
    joint_index = 0

    for block in blocks:
        fields = {}
        for block_line in block.content:
            key, value = block_line.split("=", 1)
            key = key.strip().lstrip(".")
            value = value.strip()
            fields[key] = value

        match block.type:
            case ("link"):      # Parse Link
                link_name = block.subtype
                link_length = float(fields.get("length", -1))

                # Attribute checks
                if (link_length < 0) and (link_name != "base"): # Non-negative length
                    raise ValueError("[ ERROR ] | Non-base link must have a valid length")
                
                links.append(Link(name=link_name, length=link_length, child=-1))
                continue
 
            case ("joint"):     # Parse Joint
                joint_type = block.subtype
                joint_links = parse_parent_child(fields.get("links", []))
                joint_limits = parse_bracket(fields.get("limits", [-pi, pi]))
                joint_axis = parse_bracket(fields.get("axis", [0, 0, 1]))
                joint_home = parse_bracket(fields.get("home", 0))[0]
                joint_init = parse_bracket(fields.get("init", 0))[0]

                # Attribute checks
                if not valid_types.__contains__(joint_type):                                         # Valid joint type
                    raise ValueError("[ ERROR ] | Invlid joint type")
                if joint_links is []:                                                               # Parent/child links must exist
                    raise ValueError("[ ERROR ] | Joint must have a parent and child")
                if joint_limits[0] > joint_limits[1]:                                               # Min limit < Max limit
                    raise ValueError("[ ERROR ] | Joint min/max limits must be valid")
                if abs(1 - sqrt(joint_axis[0]**2 + joint_axis[1]**2 + joint_axis[2]**2)) > 0.001:   # Normalized axis
                    raise ValueError("[ ERROR ] | Joint axis must be normalized")
                if not ((joint_limits[0] <= joint_home) and (joint_home <= joint_limits[1])):             # Home value is valid
                    raise ValueError("[ ERROR ] | Joint home value must be valid")
                if not ((joint_limits[0] <= joint_init) and (joint_init <= joint_limits[1])):             # Initial value is valid
                    raise ValueError("[ ERROR ] | Joint initial value must be valid")

                joints.append(Joint(id=joint_index, type=joint_type, parent=joint_links[0], child=joint_links[1], limits=joint_limits, axis=joint_axis, home=joint_home, init=joint_init))

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
    return links, joints


if __name__ == "__main__":
    in_file = sys.argv[1]
    links, joints = parse_file(in_file)
    

    print("### Links ###")
    for link in links:
        print("-----------------------------")
        print("- Name: %s" % (link.name))
        print("- Child: %d" % (link.child))   
        print("- Lenght: %.3f" % (link.length))
    print("-----------------------------\n")

    print("### Joints ###")
    for joint in joints:
        print("-----------------------------")
        print("- Type: %s" % (joint.type))
        print("- Parent: %s" % (joint.parent))   
        print("- Child: %s" % (joint.child))   
        print("- Limits: [%.3f, %.3f]" % (joint.limits[0], joint.limits[1]))
        print("- Inital: %.3f" % (joint.init))
        print("- Home: %.3f" % (joint.home))
        print("- Axis: [%.3f, %.3f, %.3f]" % (joint.axis[0], joint.axis[1], joint.axis[2]))
    print("-----------------------------")

