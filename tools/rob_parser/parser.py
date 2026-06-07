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
from rich.console import Console

# ========== GLOBALS ==========
console = Console()
WARNING = True
VERBOSE = False
DEBUG = False
LINE_NO = 0
COMPLETE_COUNT = 0
FILE_COUNT = 0

# ========== MATH CONST ==========
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
    id: int
    name: str
    mass: float
    inertia: List[List[float]]
    com_xyz: List[float]
    com_rpy: List[float]
    virtual: bool = False

@dataclass
class Joint:
    id: int
    parent_id: int
    child_id: int
    type: str
    axis: List[float]
    limits: List[float]
    vel_limit: float
    init: float
    home: float
    origin_xyz: List[float]
    origin_rpy: List[float]
    comp_type: str
    comp_index: int = -1

@dataclass
class Frame:
    id: int
    link_id: int
    name: str
    origin_xyz: List[float]
    origin_rpy: List[float]

@dataclass
class PartBlock:
    def __init__(self, type: str, subtype: str, line_no: int = 0):
        self.type = type
        self.subtype = subtype
        self.line_no = line_no 
        self.fields = {}
        self.subblocks = {}
        self.field_lines = {}
        self.subblock_lines_with_nums = {}

## ========== ERROR HANDLING ==========
def error(msg: str):
    console.print(f" ↳ 🗵  {msg} | line-({LINE_NO})", style="bold bright_red")
    sys.exit(1)

def param_error(msg: str):
    console.print(f" ↳ ﹗ {msg} | line-({LINE_NO})", style="bold deep_pink2")
    sys.exit(1)

def warning(msg: str):
    if WARNING:
        console.print(f" ↳ ⚠  {msg} | line-({LINE_NO})", style="bold orange3")

def success(msg: str):
    console.print(f" ↳ ☑  {msg} ", style="bold green")

def log(msg: str):
    if VERBOSE:
        console.print(f" ↳  ℹ  {msg} ", style="bold purple3", markup=False)

def debug(msg: str):
    if DEBUG:
        console.print(f" ↳ 🛠  {msg} ", style="bold turquoise4", markup=False)

## ========== MATH ==========
def safe_eval(expr: str) -> float:
    try:
        def _eval(node):
            if isinstance(node, ast.Constant):
                value = node.value
                if isinstance(value, (int, float)):
                    return value
                else:
                    error(f"Expression '{expr}' contains non-numeric constant")
            elif isinstance(node, ast.UnaryOp) and (type(node.op) in _ALLOWED_OPERATORS):
                return float(_ALLOWED_OPERATORS[type(node.op)](_eval(node.operand)))
            elif isinstance(node, ast.BinOp) and (type(node.op) in _ALLOWED_OPERATORS):
                return float(_ALLOWED_OPERATORS[type(node.op)](_eval(node.left), _eval(node.right)))
            elif isinstance(node, ast.Name):
                if node.id == "pi":
                    return math.pi
                elif node.id == "e":
                    return math.e
                else:
                    error(f"Unknown constant '{node.id}' in expression (only 'pi' and 'e' are allowed)")
            else:
                error(f"Unsupported operation in expression '{expr}'")
        
        node = ast.parse(expr, mode='eval').body
        return _eval(node)
    except SyntaxError:
        error(f"Invalid syntax in math expression '{expr}'")
    except Exception as e:
        error(f"Failed to evaluate expression '{expr}': {str(e)}")

## ========== HELPER FUNCTIONS/PARSERS ==========
def collect_blocks(lines):
    global LINE_NO
    blocks = [] 
    current = None
    brace_stack = []  # Each entry: ("type", line_no, description)
    current_subblock = None
    subblock_lines = []
    subblock_line_nums = []  # Track line numbers for subblock lines

    # Track which attributes typically have subblocks
    SUBBLOCK_ATTRIBUTES = {'com', 'origin', 'inertia'}

    line_num = 0
    for raw in lines:
        line_num += 1
        LINE_NO = line_num
        line = raw.split("#", 1)[0].strip()
        
        if not line:
            continue
        if line.startswith(">") and current is None:
            continue

        # New top-level block
        if line.startswith("\\"):
            parts = line[1:].split()
            if len(parts) < 2:
                error("Block declaration incomplete - expected format: '\\type name {'")
            block_type, block_name = parts[0], parts[1]
            
            # Check if this is a standalone declaration (no opening brace)
            if not line.endswith("{"):
                blocks.append(PartBlock(block_type, block_name, LINE_NO))
                continue
            
            # Block with body - save the line number
            current = PartBlock(block_type, block_name, LINE_NO)
            brace_stack.append(("block", LINE_NO, f"\\{block_type} {block_name}"))
            continue

        if current is None:
            error(f"Content found outside of any block - all attributes must be inside a block")

        # Closing brace
        if line == "}":
            if not brace_stack:
                error("Unexpected closing brace '}' - no matching opening brace")
            
            block_type, start_line, description = brace_stack.pop()
            
            if block_type == "subblock":
                # Store the collected subblock lines with their line numbers
                if current_subblock:
                    current.subblocks[current_subblock] = subblock_lines
                    current.subblock_lines_with_nums[current_subblock] = subblock_line_nums
                    current_subblock = None
                    subblock_lines = []
                    subblock_line_nums = []
            elif block_type == "block":
                # End of main block
                blocks.append(current)
                current = None
            continue

        # Check for subblock start (e.g., .com = {)
        if "=" in line and line.endswith("{"):
            k = line.split("=", 1)[0].strip().lstrip(".")
            current_subblock = k
            subblock_lines = []
            subblock_line_nums = []
            brace_stack.append(("subblock", LINE_NO, f".{k}"))
            continue

        # Inside a subblock
        if current_subblock is not None:
            subblock_lines.append(line)
            subblock_line_nums.append(LINE_NO)
            continue

        # Regular field assignment
        if "=" in line:
            k, v = line.split("=", 1)
            k = k.strip().lstrip(".")
            v = v.strip()
            
            # Check if this looks like a subblock attribute that's missing its brace
            if k in SUBBLOCK_ATTRIBUTES and not v:
                error(f"Attribute '.{k}' is missing opening brace '{{' - use format: '.{k} = {{'")
            
            # Check if we're seeing a nested attribute without being in a subblock
            if k in {'xyz', 'rpy'} and current_subblock is None:
                error(f"Attribute '.{k}' must be inside a subblock (like .com or .origin) - did you forget the opening brace '{{'?")
            
            # Validate brackets are properly nested
            if "[" in v or "]" in v:
                if not validate_brackets(v):
                    error(f"Attribute '.{k}' has mismatched or improperly nested brackets")
            
            # Validate braces
            if "{" in v or "}" in v:
                if v.count("{") != v.count("}"):
                    error(f"Attribute '.{k}' has mismatched braces '{{' and '}}'e")
            
            current.fields[k] = v
            current.field_lines[k] = LINE_NO  # Track line number
        else:
            error(f"Invalid syntax - expected attribute assignment '.attribute = value' or closing brace '}}'")

    # Final check for unclosed blocks
    if brace_stack:
        block_type, start_line, description = brace_stack[-1]
        LINE_NO = start_line
        error(f"Unclosed {block_type} '{description}' - missing closing brace '}}'")

    return blocks

def validate_brackets(value: str) -> bool:
    stack = []
    depth = 0
    
    for i, char in enumerate(value):
        if char == '[':
            stack.append(i)
            depth += 1
        elif char == ']':
            if not stack:
                return False
            stack.pop()
            depth -= 1
            
            
            if depth > 0:  
                rest = value[i+1:].lstrip()
                if rest and rest[0] not in ',]':
                    return False
    
    return len(stack) == 0

def parse_bracket(line: str) -> List[float]:
    global LINE_NO
    line = line.strip()
    if not line.startswith("[") or not line.endswith("]"):
        error("List value must be enclosed in square brackets '[...]'")
    try:
        inner = line[1:-1].strip()
        if not inner:
            error("Empty list '[]' is not allowed")
        items = re.split(r"\s*,\s*", inner)
        return [safe_eval(i) for i in items if i]
    except Exception as e:
        error(f"Invalid list value - {str(e)}")

def parse_inertia(value: str):
    global LINE_NO
    val = value.strip()
    if val.count("[") != 4 or val.count("]") != 4:
        error("Inertia matrix must be a 3x3 matrix in row major form: '[[...], [...], [...]]'")
    if not (val.startswith("[") and val.endswith("]")):
        error("Inertia matrix must be enclosed in outer brackets '[...]'")
    try:
        mat = ast.literal_eval(value)
    except Exception as e:
        error(f"Invalid inertia matrix syntax: {str(e)}")

    if len(mat) != 3 or any(len(row) != 3 for row in mat):
        error("Inertia matrix must be 3x3")

    eps = 1e-6
    for i in range(3):
        for j in range(3):
            if abs(mat[i][j] - mat[j][i]) > eps:
                param_error(f"Inertia matrix must be symmetric (element [{i}][{j}] = {mat[i][j]} != [{j}][{i}] = {mat[j][i]})")
    for i in range(3):
        if mat[i][i] < 0:
            param_error(f"Inertia diagonal elements must be non-negative (element [{i}][{i}] = {mat[i][i]} < 0)")
    return mat

def parse_transform_block(lines: List[str], line_nums: List[int] = None):
    global LINE_NO
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]

    for i, line in enumerate(lines):
        if line_nums:
            LINE_NO = line_nums[i]  # Set correct line number
            
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith(".xyz"):
            if "=" not in line:
                error("Transform .xyz attribute requires assignment: '.xyz = [x, y, z]'")
            try:
                xyz = parse_bracket(line.split("=", 1)[1])
            except Exception as e:
                error(f"Invalid .xyz value: {str(e)}")
        elif line.startswith(".rpy"):
            if "=" not in line:
                error("Transform .rpy attribute requires assignment: '.rpy = [roll, pitch, yaw]'")
            try:
                rpy = parse_bracket(line.split("=", 1)[1])
            except Exception as e:
                error(f"Invalid .rpy value: {str(e)}")
        else:
            error(f"Unknown attribute in transform block - only '.xyz' and '.rpy' are allowed")

    if len(xyz) != 3:
        error(f"Transform .xyz must have exactly 3 values [x, y, z], found {len(xyz)} values")
    if len(rpy) != 3:
        error(f"Transform .rpy must have exactly 3 values [roll, pitch, yaw], found {len(rpy)} values")
    return xyz, rpy

def parse_parent_child(line: str) -> List[str]:
    result = re.findall(r"[a-zA-Z_][a-zA-Z0-9_]*", line)
    if len(result) != 2:
        error(f"Joint .links attribute must specify exactly 2 link names as [parent, child], found {len(result)}")
    return result

def parse_joint_type(joint: str) -> str:
    key = joint.strip().lower()
    if key not in JOINT_TYPE_ALIASES:
        valid_types = ", ".join(sorted(set(JOINT_TYPE_ALIASES.values())))
        param_error(f"Unknown joint type '{joint}' - valid types are: {valid_types}")
    return JOINT_TYPE_ALIASES[key]

def parse_nested_list(value: str) -> List:
    global LINE_NO
    value = value.strip()
    if not (value.startswith('[') and value.endswith(']')):
        error("Nested list must be enclosed in square brackets '[...]'")
    
    inner = value[1:-1].strip()
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
                result.append(parse_sublist(current.strip()))
                current = ""
        elif char == ',' and depth == 0:
            if current.strip() and not current.strip().startswith('['):
                result.append(safe_eval(current.strip()))
                current = ""
        else:
            current += char
    
    if current.strip():
        if current.strip().startswith('['):
            result.append(parse_sublist(current.strip()))
        else:
            result.append(safe_eval(current.strip()))
    return result

def parse_sublist(value: str) -> List[float]:
    global LINE_NO
    value = value.strip()
    if not (value.startswith('[') and value.endswith(']')):
        error("Sublist must be enclosed in square brackets '[...]'")
    inner = value[1:-1].strip()
    if not inner:
        error("Empty sublist '[]' is not allowed")
    elements = [elem.strip() for elem in inner.split(',')]
    return [safe_eval(elem) for elem in elements if elem]

def parse_multi_limits(value) -> List[List[float]]:
    global LINE_NO
    if isinstance(value, list):
        return [[safe_eval(str(v[0])) if not isinstance(v[0], (int, float)) else float(v[0]), 
                 safe_eval(str(v[1])) if not isinstance(v[1], (int, float)) else float(v[1])] 
                for v in value]
    
    try:
        mat = parse_nested_list(value)
    except Exception as e:
        error(f"Failed to parse multi-DOF limits: {str(e)}")
    
    if not isinstance(mat, list):
        error("Multi-DOF limits must be a list of [min, max] pairs, e.g. [[min1, max1], [min2, max2], ...]")
    
    result = []
    for i, limit_pair in enumerate(mat):
        if not isinstance(limit_pair, list) or len(limit_pair) != 2:
            param_error(f"DOF {i} limits must be a pair [min, max], found {limit_pair}")
        
        min_val = limit_pair[0] if isinstance(limit_pair[0], (int, float)) else safe_eval(str(limit_pair[0]))
        max_val = limit_pair[1] if isinstance(limit_pair[1], (int, float)) else safe_eval(str(limit_pair[1]))
        result.append([min_val, max_val])
    
    return result

def parse_multi_axes(value) -> List[List[float]]:
    global LINE_NO
    if isinstance(value, list):
        mat = value
    else:
        try:
            mat = parse_nested_list(value)
        except Exception as e:
            error(f"Failed to parse multi-axis: {str(e)}")
    
    if not isinstance(mat, list):
        error("Multi-axis must be a list of 3D vectors, e.g. [[x1,y1,z1], [x2,y2,z2]]")
    
    result = []
    for i, axis in enumerate(mat):
        if not isinstance(axis, list) or len(axis) != 3:
            param_error(f"Axis {i} must have exactly 3 components [x, y, z], found {len(axis) if isinstance(axis, list) else 'non-list'}")
        
        parsed_axis = []
        for j in range(3):
            if isinstance(axis[j], (int, float)):
                parsed_axis.append(float(axis[j]))
            else:
                parsed_axis.append(safe_eval(str(axis[j])))
        
        norm = sqrt(sum(a**2 for a in parsed_axis))
        if abs(norm - 1.0) > JOINT_AXIS_THRESHOLD:
            param_error(f"Axis {i} must be normalized (unit length), current length = {norm:.6f}")
        
        result.append(parsed_axis)
    
    return result

def parse_multi_values(value) -> List[float]:
    global LINE_NO
    if isinstance(value, list):
        return [safe_eval(str(v)) if not isinstance(v, (int, float)) else float(v) for v in value]
    
    try:
        vals = parse_nested_list(value)
        if vals and isinstance(vals[0], list):
            param_error("Init/home values must be a flat list '[...]', not nested lists")
    except Exception as e:
        error(f"Failed to parse multi-DOF values: {str(e)}")
    
    if not isinstance(vals, list):
        error("Multi-DOF values must be a list")
    
    result = []
    for v in vals:
        if isinstance(v, (int, float)):
            result.append(float(v))
        else:
            result.append(safe_eval(str(v)))
    
    return result

def pascal_case(s: str) -> str:
    s = s.replace("_", " ").replace("-", " ")
    s = s.title()
    s = s.replace(" ", "")
    return s

## ========== TOP-LEVEL PARSER ==========
def parse_rob_file(file_path: str):
    global LINE_NO
    
    robot_name = None
    links: List[Link] = []
    joints: List[Joint] = []
    frames: List[Frame] = []
    name_to_id = {}

    # ===== Line Extraction =====
    try:
        with open(file_path, "r") as f:
            lines = [l.rstrip() for l in f]
    except FileNotFoundError:
        console.print(f"File not found: '{file_path}'", style="bold bright_red")
        sys.exit(1)
    except Exception as e:
        console.print(f"Failed to read file '{file_path}': {str(e)}", style="bold bright_red")
        sys.exit(1)

    # ===== Robot Name =====
    LINE_NO = 0
    for i, line in enumerate(lines, 1):
        LINE_NO = i
        line = line.split("#", 1)[0].strip()
        if line.startswith(">"):
            if robot_name is not None:
                param_error("Robot name defined multiple times - only one '> name' declaration allowed")
            robot_name = line[1:].strip()
            if not robot_name:
                param_error("Robot must be named - use format '> robot_name'")
    
    LINE_NO = 0
    if robot_name is None:
        param_error("Robot name not defined - add '> robot_name' at the top of the file")
    if len(robot_name) > 31:
        param_error(f"Robot name '{robot_name}' exceeds maximum length of 31 characters")
    if not re.match(r"[A-Za-z_][A-Za-z0-9_]*$", robot_name):
        param_error(f"Robot name '{robot_name}' must be a valid C identifier (letters, numbers, underscore; cannot start with number)")
    
    # ===== Blocks =====
    blocks = collect_blocks(lines)

    # ===== Parsing =====
    # === Links === 
    for block in blocks:
        if block.type != "link":
            continue

        LINE_NO = block.line_no
        name = block.subtype
        if name == "" or name == "{":
            param_error("Link declaration missing name - use format '\\link name {'")
        # --- Name Attribute checks ---
        if len(name) > 31:
            param_error(f"Link name '{name}' exceeds maximum length of 31 characters")
        if not re.match(r"[A-Za-z_][A-Za-z0-9_]*$", name):
            param_error(f"Link name '{name}' must be a valid C identifier (letters, numbers, underscore; cannot start with number)")
        for l in links:
            if l.name == name:
                param_error(f"Duplicate link name '{name}' - all link names must be unique")

        LINE_NO += 1;
        try:
            mass = safe_eval(block.fields.get("mass", LINK_DEFAULT_MASS))
        except Exception as e:
            error(f"Failed to parse .mass for link '{name}': {str(e)}")
        # --- Mass Attribute checks ---
        if mass < 0:
            param_error(f"Link '{name}' has negative mass ({mass}) - mass must be non-negative")
        if mass == 0 and name.lower() not in LINK_ALLOWED_BASES:
            warning(f"Link '{name}' has zero mass - may be unassigned")

        LINE_NO += 1;
        try:
            inertia = parse_inertia(block.fields.get("inertia", LINK_DEFAULT_INERTIA))
        except Exception as e:
            error(f"Failed to parse .inertia for link '{name}': {str(e)}")

        LINE_NO += 1;
        if "com" in block.subblocks:
            try:
                line_nums = block.subblock_lines_with_nums.get("com", [])
                com_xyz, com_rpy = parse_transform_block(block.subblocks["com"], line_nums)
            except Exception as e:
                error(f"Failed to parse .com block for link '{name}': {str(e)}")
        else:
            if name.lower() not in LINK_ALLOWED_BASES:
                warning(f"Link '{name}' has no .com block defined, using default values")
            com_xyz = LINK_DEFAULT_COM_XYZ
            com_rpy = LINK_DEFAULT_COM_RPY

        link_id = len(links)
        name_to_id[name] = link_id

        # --- Debug Prints ---
        debug(f"> === Link-{link_id} | {robot_name} ===")
        debug(f"    —— name: {name}")
        debug(f"    —— mass: {mass}")
        debug(f"          ┌                      ┐")
        debug(f"          │ {inertia[0][0]:2.4f} {inertia[0][1]:2.4f} {inertia[0][1]:2.4f} │")
        debug(f"    —— I: │ {inertia[1][0]:2.4f} {inertia[1][1]:2.4f} {inertia[1][1]:2.4f} │")
        debug(f"          │ {inertia[2][0]:2.4f} {inertia[2][1]:2.4f} {inertia[2][1]:2.4f} │")
        debug(f"          └                      ┘")
        debug("    —— COM: {")
        debug(f"    \txyz: <{com_xyz[0]:2.4f}, {com_xyz[1]:2.4f}, {com_xyz[2]:2.4f}>")
        debug(f"    \trpy: <{com_rpy[0]:2.4f}, {com_rpy[1]:2.4f}, {com_rpy[2]:2.4f}>")
        debug("      }")
        debug(f"    —— virtual: false")
        debug(f"")

        links.append(Link(
            id=link_id,
            name=name,
            mass=mass,
            inertia=inertia,
            com_xyz=com_xyz,
            com_rpy=com_rpy,
            virtual=False
        ))

    # --- Base/ground/reference link check (BEFORE joint parsing) ---
    base_candidates = [l for l in links if l.name.lower() in LINK_ALLOWED_BASES]
    if len(base_candidates) == 0:
        LINE_NO = 0
        param_error("No base link found - robot must have exactly one link named 'base', 'ground', or 'reference'")
    elif len(base_candidates) > 1:
        LINE_NO = 0
        base_names = ", ".join([f"'{l.name}'" for l in base_candidates])
        param_error(f"Multiple base links found: {base_names} - robot must have exactly one link named 'base', 'ground', or 'reference'")

    # === Joints ===
    for block in blocks:
        if block.type != "joint":
            continue
        
        LINE_NO = block.line_no
        joint_type = parse_joint_type(block.subtype)
        
        if "links" not in block.fields:
            error("Joint missing required attribute .links = [parent, child]")
        
        try:
            parent_name, child_name = parse_parent_child(block.fields["links"])
        except Exception as e:
            error(f"Failed to parse joint .links attribute: {str(e)}")
        
        if parent_name not in name_to_id:
            error(f"Joint references unknown parent link '{parent_name}'")
        if child_name not in name_to_id:
            error(f"Joint references unknown child link '{child_name}'")
            
        parent_id = name_to_id[parent_name]
        child_id = name_to_id[child_name]
        
        if "origin" in block.subblocks:
            try:
                line_nums = block.subblock_lines_with_nums.get("origin", [])
                origin_xyz, origin_rpy = parse_transform_block(block.subblocks["origin"], line_nums)
            except Exception as e:
                error(f"Failed to parse joint .origin block: {str(e)}")
        else:
            warning(f"Joint {joint_type} [{parent_name} -> {child_name}] has no .origin block defined, using default values")
            origin_xyz = JOINT_DEFAULT_ORIGIN_XYZ
            origin_rpy = JOINT_DEFAULT_ORIGIN_RPY

        # Handle compound joint types
        if joint_type == "universal":
            # Parse universal joint parameters
            axes_str = block.fields.get("axis", None)
            if axes_str is None:
                warning(f"Universal joint has no .axis defined, using defaults")
                axes = UNIVERSAL_DEFAULT_AXES
            else:
                axes = parse_multi_axes(axes_str)
            
            limits_str = block.fields.get("limits", None)
            if limits_str is None:
                warning(f"Universal joint has no .limits defined, using defaults")
                limits = UNIVERSAL_DEFAULT_LIMITS
            else:
                limits = parse_multi_limits(limits_str)

            vel_limits_str = block.fields.get("vel_limit", None)
            if vel_limits_str is None:
                warning(f"Universal joint has no .vel_limit defined, using defaults")
                vel_limits = UNIVERSAL_DEFAULT_VELOCITY_LIMIT
            else:
                vel_limits = parse_multi_values(vel_limits_str)
            
            init_str = block.fields.get("init", None)
            if init_str is None:
                warning(f"Universal joint has no .init defined, using defaults")
                init_vals = UNIVERSAL_DEFAULT_INIT
            else:
                init_vals = parse_multi_values(init_str)
            
            home_str = block.fields.get("home", None)
            if home_str is None:
                warning(f"Universal joint has no .home defined, using defaults")
                home_vals = UNIVERSAL_DEFAULT_HOME
            else:
                home_vals = parse_multi_values(home_str)
            
            # Validation
            if len(axes) != 2:
                param_error(f"Universal joint must have exactly 2 axes, found {len(axes)}")
            if len(limits) != 2:
                param_error(f"Universal joint must have exactly 2 DOF limit pairs, found {len(limits)}")
            if len(vel_limits) != 2:
                param_error(f"Universal joint must have exactly 2 DOF velocity limits, found {len(vel_limits)}")
            if len(init_vals) != 2:
                param_error(f"Universal joint must have exactly 2 initial values, found {len(init_vals)}")
            if len(home_vals) != 2:
                param_error(f"Universal joint must have exactly 2 home values, found {len(home_vals)}")
            
            # Axis orthogonality validation
            axis0 = axes[0]
            axis1 = axes[1]
            dot_prod = axis0[0]*axis1[0] + axis0[1]*axis1[1] + axis0[2]*axis1[2]
            if abs(dot_prod) > JOINT_AXIS_THRESHOLD:
                param_error(f"Universal joint axes must be orthogonal")
            
            # Create 2 revolute joints + 1 invisible link
            base_joint_id = len(joints)
            invisible_link_id = len(links)

            # --- Debug Prints ---
            debug(f"> === Link-{invisible_link_id} | {robot_name} ===")
            debug(f"    —— name: _u_link_{base_joint_id}")
            debug(f"    —— virtual: true")
            debug(f"")
            
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
            for i, (axis, limit, vel_limit, init_val, home_val) in enumerate(zip(axes, limits, vel_limits, init_vals, home_vals)):
                if limit[0] > limit[1]:
                    param_error(f"Universal joint DOF {i}: min limit ({limit[0]}) > max limit ({limit[1]})")
                if vel_limit < 0:
                    param_error(f"Universal joint DOF {i}: veloicty limit ({vel_limit}) must be non-negative")
                if not ((limit[0] <= init_val) and (init_val <= limit[1])):
                    param_error(f"Universal joint DOF {i}: initial value ({init_val}) is outside limits [{limit[0]}, {limit[1]}]")
                
                current_parent = parent_id if i == 0 else invisible_link_id
                current_child = invisible_link_id if i == 0 else child_id
                current_origin_xyz = origin_xyz if i == 0 else [0.0, 0.0, 0.0]
                current_origin_rpy = origin_rpy if i == 0 else [0.0, 0.0, 0.0]

                # --- Debug Prints ---
                parent = f"{parent_name if i == 0 else f"_u_link_{base_joint_id}"}"
                child = f"{f"_u_link_{base_joint_id}" if i == 0 else child_name}"
                debug(f"> === Joint-{len(joints)} | {robot_name} ===")
                debug(f"    —— links: [{parent}({current_parent}), {child}({current_child})]")
                debug(f"    —— type: revolute")
                debug(f"    —— init: {init_val:2.4f}")
                debug(f"    —— home: {home_val:2.4f}")
                debug(f"    —— limits: [{limit[0]:2.4f}, {limit[1]:2.4f}]")
                debug(f"    —— vel_limit: {vel_limit:2.4f}")
                debug(f"    —— axis: <{axis[0]:2.4f}, {axis[1]:2.4f}, {axis[2]:2.4f}>")
                debug("     —— origin: {")
                debug(f"    \txyz: <{current_origin_xyz[0]:2.4f}, {current_origin_xyz[1]:2.4f}, {current_origin_xyz[2]:2.4f}>")
                debug(f"    \trpy: <{current_origin_rpy[0]:2.4f}, {current_origin_rpy[1]:2.4f}, {current_origin_rpy[2]:2.4f}>")
                debug("      }")
                debug(f"    —— comp_type: {joint_type}")
                debug(f"    —— comp_index: {i}")
                debug(f"")
                
                joints.append(Joint(
                    id=len(joints),
                    parent_id=current_parent,
                    child_id=current_child,
                    type="revolute",
                    axis=axis,
                    limits=limit,
                    vel_limit=vel_limit,
                    init=init_val,
                    home=home_val,
                    origin_xyz=current_origin_xyz,
                    origin_rpy=current_origin_rpy,
                    comp_type=joint_type,
                    comp_index=i
                ))
        elif joint_type == "spherical":
            axes_str = block.fields.get("axis", None)
            if axes_str is None:
                warning(f"Spherical joint has no .axis defined, using defaults")
                axes = SPHERICAL_DEFAULT_AXES
            else:
                axes = parse_multi_axes(axes_str)

            limits_str = block.fields.get("limits", None)
            if limits_str is None:
                warning(f"Spherical joint has no .limits defined, using defaults")
                limits = SPHERICAL_DEFAULT_LIMITS
            else:
                limits = parse_multi_limits(limits_str)

            vel_limits_str = block.fields.get("vel_limit", None)
            if vel_limits_str is None:
                warning(f"Spherical joint has no .vel_limit defined, using defaults")
                vel_limits = SPHERICAL_DEFAULT_VELOCITY_LIMIT
            else:
                vel_limits = parse_multi_values(vel_limits_str)
            
            init_str = block.fields.get("init", None)
            if init_str is None:
                warning(f"Spherical joint has no .init defined, using defaults")
                init_vals = SPHERICAL_DEFAULT_INIT
            else:
                init_vals = parse_multi_values(init_str)
            
            home_str = block.fields.get("home", None)
            if home_str is None:
                warning(f"Spherical joint has no .home defined, using defaults")
                home_vals = SPHERICAL_DEFAULT_HOME
            else:
                home_vals = parse_multi_values(home_str)
            
            # Validation
            if len(axes) != 3:
                param_error(f"Spherical joint must have exactly 3 axes, found {len(axes)}")
            if len(limits) != 3:
                param_error(f"Spherical joint must have exactly 3 DOF limit pairs, found {len(limits)}")
            if len(vel_limits) != 3:
                param_error(f"Spherical joint must have exactly 3 DOF velocity limits, found {len(vel_limits)}")
            if len(init_vals) != 3:
                param_error(f"Spherical joint must have exactly 3 initial values, found {len(init_vals)}")
            if len(home_vals) != 3:
                param_error(f"Spherical joint must have exactly 3 home values, found {len(home_vals)}")

            # Axis orthogonality validation
            axis0 = axes[0]
            axis1 = axes[1]
            axis2 = axes[2]
            dot_prod1 = axis0[0]*axis1[0] + axis0[1]*axis1[1] + axis0[2]*axis1[2]
            dot_prod2 = axis1[0]*axis2[0] + axis1[1]*axis2[1] + axis1[2]*axis2[2]
            dot_prod3 = axis0[0]*axis2[0] + axis0[1]*axis2[1] + axis0[2]*axis2[2]
            if abs(dot_prod1) > JOINT_AXIS_THRESHOLD and abs(dot_prod2) > JOINT_AXIS_THRESHOLD and abs(dot_prod3) > JOINT_AXIS_THRESHOLD:
                param_error(f"Spherical joint axes must not all be aligned")
            
            base_joint_id = len(joints)
            invisible_link_1_id = len(links)
            invisible_link_2_id = len(links) + 1

            # --- Debug Prints ---
            debug(f"> === Link-{invisible_link_1_id} | {robot_name} ===")
            debug(f"    —— name: _s_link1_{base_joint_id}")
            debug(f"    —— virtual: true")
            debug(f"")

            debug(f"> === Link-{invisible_link_2_id} | {robot_name} ===")
            debug(f"    —— name: _s_link2_{base_joint_id}")
            debug(f"    —— virtual: true")
            debug(f"")
            
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
            
            for i, (axis, limit, vel_limit, init_val, home_val) in enumerate(zip(axes, limits, vel_limits, init_vals, home_vals)):
                if limit[0] > limit[1]:
                    param_error(f"Spherical joint DOF {i}: min limit ({limit[0]}) > max limit ({limit[1]})")
                if vel_limit < 0:
                    param_error(f"Spherical joint DOF {i}: veloicty limit ({vel_limit}) must be non-negative")
                if not ((limit[0] <= init_val) and (init_val <= limit[1])):
                    param_error(f"Spherical joint DOF {i}: initial value ({init_val}) is outside limits [{limit[0]}, {limit[1]}]")
                
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
                else:
                    current_parent = invisible_link_2_id
                    current_child = child_id
                    current_origin_xyz = [0.0, 0.0, 0.0]
                    current_origin_rpy = [0.0, 0.0, 0.0]

                # --- Debug Prints ---
                parent = f"{parent_name if i == 0 else (f"_s_link1_{base_joint_id}" if i == 1 else f"_s_link2_{base_joint_id}")}"
                child = f"{f"_s_link1_{base_joint_id}" if i == 0 else (f"_s_link2_{base_joint_id}" if i == 1 else child_name)}"
                debug(f"> === Joint-{len(joints)} | {robot_name} ===")
                debug(f"    —— links: [{parent}({current_parent}), {child}({current_child})]")
                debug(f"    —— type: revolute")
                debug(f"    —— init: {init_val:2.4f}")
                debug(f"    —— home: {home_val:2.4f}")
                debug(f"    —— limits: [{limit[0]:2.4f}, {limit[1]:2.4f}]")
                debug(f"    —— vel_limit: {vel_limit:2.4f}")
                debug(f"    —— axis: <{axis[0]:2.4f}, {axis[1]:2.4f}, {axis[2]:2.4f}>")
                debug("     —— origin: {")
                debug(f"    \txyz: <{current_origin_xyz[0]:2.4f}, {current_origin_xyz[1]:2.4f}, {current_origin_xyz[2]:2.4f}>")
                debug(f"    \trpy: <{current_origin_rpy[0]:2.4f}, {current_origin_rpy[1]:2.4f}, {current_origin_rpy[2]:2.4f}>")
                debug("      }")
                debug(f"    —— comp_type: {joint_type}")
                debug(f"    —— comp_index: {i}")
                debug(f"")
                
                joints.append(Joint(
                    id=len(joints),
                    parent_id=current_parent,
                    child_id=current_child,
                    type="revolute",
                    axis=axis,
                    limits=limit,
                    vel_limit=vel_limit,
                    init=init_val,
                    home=home_val,
                    origin_xyz=current_origin_xyz,
                    origin_rpy=current_origin_rpy,
                    comp_type=joint_type,
                    comp_index=i
                ))
        elif joint_type == "cylinderical":
            # Parse cylinderical joint parameters
            axis_str = block.fields.get("axis", None)
            if axis_str is None:
                warning(f"Cylinderical joint has no .axis defined, using default")
                axis = CYLINDERICAL_DEFAULT_AXIS
            else:
                axis = parse_bracket(axis_str)
            
            limits_str = block.fields.get("limits", None)
            if limits_str is None:
                warning(f"Cylinderical joint has no .limit defined, using defaults")
                limits = CYLINDERICAL_DEFAULT_LIMITS
            else:
                limits = parse_multi_limits(limits_str)
            
            vel_limits_str = block.fields.get("vel_limit", None)
            if vel_limits_str is None:
                warning(f"Cylinderical joint has no .vel_limit defined, using defaults")
                vel_limits = CYLINDERICAL_DEFAULT_VELOCITY_LIMIT
            else:
                vel_limits = parse_multi_values(vel_limits_str)
            
            init_str = block.fields.get("init", None)
            if init_str is None:
                warning(f"Cylinderical joint has no .init defined, using default")
                init_vals = CYLINDERICAL_DEFAULT_INIT
            else:
                init_vals = parse_multi_values(init_str)
            
            home_str = block.fields.get("home", None)
            if home_str is None:
                warning(f"Cylinderical joint has no .home defined, using default")
                home_vals = CYLINDERICAL_DEFAULT_HOME
            else:
                home_vals = parse_multi_values(home_str)
            
            # Validation
            if len(limits) != 2:
                param_error(f"Cylinderical joint must have exactly 2 DOF limit pairs, found {len(limits)}")
            if len(vel_limits) != 2:
                param_error(f"Cylinderical joint must have exactly 2 DOF velocity limits, found {len(vel_limits)}")
            if len(init_vals) != 2:
                param_error(f"Cylinderical joint must have exactly 2 initial values, found {len(init_vals)}")
            if len(home_vals) != 2:
                param_error(f"Cylinderical joint must have exactly 2 home values, found {len(home_vals)}")
            
            # Create 1 revolute joints + 1 prismatic joint + 1 invisible link
            base_joint_id = len(joints)
            invisible_link_id = len(links)

            # --- Debug Prints ---
            debug(f"> === Link-{invisible_link_id} | {robot_name} ===")
            debug(f"    —— name: _c_link_{base_joint_id}")
            debug(f"    —— virtual: true")
            debug(f"")
            
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
            for i, (limit, vel_limit, init_val, home_val) in enumerate(zip(limits, vel_limits, init_vals, home_vals)):
                if limit[0] > limit[1]:
                    param_error(f"Cylinderical joint DOF {i}: min limit ({limit[0]}) > max limit ({limit[1]})")
                if vel_limit < 0:
                    param_error(f"Cylinderical joint DOF {i}: veloicty limit ({vel_limit}) must be non-negative")
                if not ((limit[0] <= init_val) and (init_val <= limit[1])):
                    param_error(f"Cylinderical joint DOF {i}: initial value ({init_val}) is outside limits [{limit[0]}, {limit[1]}]")
                
                current_parent = parent_id if i == 0 else invisible_link_id
                current_child = invisible_link_id if i == 0 else child_id
                current_origin_xyz = origin_xyz if i == 0 else [0.0, 0.0, 0.0]
                current_origin_rpy = origin_rpy if i == 0 else [0.0, 0.0, 0.0]

                # --- Debug Prints ---
                parent = f"{parent_name if i == 0 else f"_c_link_{base_joint_id}"}"
                child = f"{f"_c_link_{base_joint_id}" if i == 0 else child_name}"
                debug(f"> === Joint-{len(joints)} | {robot_name} ===")
                debug(f"    —— links: [{parent}({current_parent}), {child}({current_child})]")
                debug(f"    —— type: {"revolute" if i == 0 else "prismatic"}")
                debug(f"    —— init: {init_val:2.4f}")
                debug(f"    —— home: {home_val:2.4f}")
                debug(f"    —— limits: [{limit[0]:2.4f}, {limit[1]:2.4f}]")
                debug(f"    —— vel_limit: {vel_limit:2.4f}")
                debug(f"    —— axis: <{axis[0]:2.4f}, {axis[1]:2.4f}, {axis[2]:2.4f}>")
                debug("     —— origin: {")
                debug(f"    \txyz: <{current_origin_xyz[0]:2.4f}, {current_origin_xyz[1]:2.4f}, {current_origin_xyz[2]:2.4f}>")
                debug(f"    \trpy: <{current_origin_rpy[0]:2.4f}, {current_origin_rpy[1]:2.4f}, {current_origin_rpy[2]:2.4f}>")
                debug("      }")
                debug(f"    —— comp_type: {joint_type}")
                debug(f"    —— comp_index: {i}")
                debug(f"")
                
                joints.append(Joint(
                    id=len(joints),
                    parent_id=current_parent,
                    child_id=current_child,
                    type= "revolute" if i == 0 else "prismatic",
                    axis=axis,
                    limits=limit,
                    vel_limit=vel_limit,
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
                warning(f"Planar joint has no .axis defined, using default")
                axes = PLANAR_DEFAULT_AXES
            else:
                axes = parse_multi_axes(axes_str)
            
            limits_str = block.fields.get("limits", None)
            if limits_str is None:
                warning(f"Planar joint has no .limit defined, using default")
                limits = PLANAR_DEFAULT_LIMITS
            else:
                limits = parse_multi_limits(limits_str)

            vel_limits_str = block.fields.get("vel_limit", None)
            if vel_limits_str is None:
                warning(f"Planar joint has no .vel_limit defined, using defaults")
                vel_limits = PLANAR_DEFAULT_VELOCITY_LIMIT
            else:
                vel_limits = parse_multi_values(vel_limits_str)
            
            init_str = block.fields.get("init", None)
            if init_str is None:
                warning(f"Planar joint has no .init defined, using default")
                init_vals = PLANAR_DEFAULT_INIT
            else:
                init_vals = parse_multi_values(init_str)
            
            home_str = block.fields.get("home", None)
            if home_str is None:
                warning(f"Planar joint has no .home defined, using default")
                home_vals = PLANAR_DEFAULT_HOME
            else:
                home_vals = parse_multi_values(home_str)
            
            # Validation
            if len(axes) != 3:
                param_error(f"Planar joint must have exactly 3 axes, found {len(axes)}")
            if len(limits) != 3:
                param_error(f"Planar joint must have exactly 3 DOF limit pairs, found {len(limits)}")
            if len(vel_limits) != 3:
                param_error(f"Planar joint must have exactly 3 DOF velocity limits, found {len(vel_limits)}")
            if len(init_vals) != 3:
                param_error(f"Planar joint must have exactly 3 initial values, found {len(init_vals)}")
            if len(home_vals) != 3:
                param_error(f"Planar joint must have exactly 3 home values, found {len(home_vals)}")

            # Axis orthogonality validation
            axis0 = axes[0]
            axis1 = axes[1]
            dot_prod = axis0[0]*axis1[0] + axis0[1]*axis1[1] + axis0[2]*axis1[2]
            if abs(dot_prod) > JOINT_AXIS_THRESHOLD:
                param_error(f"Planar joint's prismatic axes must be orthogonal")
            
            # Create 2 prismatic joints + 1 revolute joint + 2 invisible link
            base_joint_id = len(joints)
            invisible_link_1_id = len(links)
            invisible_link_2_id = len(links) + 1

            # --- Debug Prints ---
            debug(f"> === Link-{invisible_link_1_id} | {robot_name} ===")
            debug(f"    —— name: _e_link1_{base_joint_id}")
            debug(f"    —— virtual: true")
            debug(f"")

            debug(f"> === Link-{invisible_link_2_id} | {robot_name} ===")
            debug(f"    —— name: _e_link2_{base_joint_id}")
            debug(f"    —— virtual: true")
            debug(f"")
            
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
                id=invisible_link_2_id,
                name=f"_e_link2_{base_joint_id}",
                mass=0.0,
                inertia=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                com_xyz=[0.0, 0.0, 0.0],
                com_rpy=[0.0, 0.0, 0.0],
                virtual=True
            ))
            name_to_id[f"_e_link2_{base_joint_id}"] = invisible_link_2_id
            
            # Create prismatic and revolute joints
            for i, (axis, limit, vel_limit, init_val, home_val) in enumerate(zip(axes, limits, vel_limits, init_vals, home_vals)):
                if limit[0] > limit[1]:
                    param_error(f"Planar joint DOF {i}: min limit ({limit[0]}) > max limit ({limit[1]})")
                if vel_limit < 0:
                    param_error(f"Planar joint DOF {i}: veloicty limit ({vel_limit}) must be non-negative")
                if not ((limit[0] <= init_val) and (init_val <= limit[1])):
                    param_error(f"Planar joint DOF {i}: initial value ({init_val}) is outside limits [{limit[0]}, {limit[1]}]")
                
                if i == 0:
                    current_parent = parent_id
                    current_child = invisible_link_1_id
                    current_origin_xyz = origin_xyz
                    current_origin_rpy = origin_rpy
                    current_type = "prismatic"
                elif i == 1:
                    current_parent = invisible_link_1_id
                    current_child = invisible_link_2_id
                    current_origin_xyz = [0.0, 0.0, 0.0]
                    current_origin_rpy = [0.0, 0.0, 0.0]
                    current_type = "prismatic"
                else:   # i == 2
                    current_parent = invisible_link_2_id
                    current_child = child_id
                    current_origin_xyz = [0.0, 0.0, 0.0]
                    current_origin_rpy = [0.0, 0.0, 0.0]
                    current_type = "revolute"

                # --- Debug Prints ---
                parent = f"{parent_name if i == 0 else (f"_e_link1_{base_joint_id}" if i == 1 else f"_e_link2_{base_joint_id}")}"
                child = f"{f"_e_link1_{base_joint_id}" if i == 0 else (f"_e_link2_{base_joint_id}" if i == 1 else child_name)}"
                debug(f"> === Joint-{len(joints)} | {robot_name} ===")
                debug(f"    —— links: [{parent}({parent_id}), {child}({child_id})]")
                debug(f"    —— type: {current_type}")
                debug(f"    —— init: {init_val:2.4f}")
                debug(f"    —— home: {home_val:2.4f}")
                debug(f"    —— limits: [{limits[0]:2.4f}, {limits[1]:2.4f}]")
                debug(f"    —— vel_limit: {vel_limit:2.4f}")
                debug(f"    —— axis: <{axis[0]:2.4f}, {axis[1]:2.4f}, {axis[2]:2.4f}>")
                debug("     —— origin: {")
                debug(f"    \txyz: <{current_origin_xyz[0]:2.4f}, {current_origin_xyz[1]:2.4f}, {current_origin_xyz[2]:2.4f}>")
                debug(f"    \trpy: <{current_origin_rpy[0]:2.4f}, {current_origin_rpy[1]:2.4f}, {current_origin_rpy[2]:2.4f}>")
                debug("      }")
                debug(f"    —— comp_type: {joint_type}")
                debug(f"    —— comp_index: {i}")
                debug(f"")
                
                joints.append(Joint(
                    id=len(joints),
                    parent_id=current_parent,
                    child_id=current_child,
                    type=current_type,
                    axis=axis,
                    limits=limit,
                    vel_limit=vel_limit,
                    init=init_val,
                    home=home_val,
                    origin_xyz=current_origin_xyz,
                    origin_rpy=current_origin_rpy,
                    comp_type=joint_type,
                    comp_index=i
                ))

        else:  # Regular joint (fixed, revolute, prismatic)
            axis_str = block.fields.get("axis", None)
            if axis_str is None:
                if joint_type != "fixed":
                    warning(f"Joint {joint_type} has no .axis defined, using default [0, 0, 1]")
                axis = parse_bracket(JOINT_DEFAULT_AXIS)
            else:
                axis = parse_bracket(axis_str)

            limits_str = block.fields.get("limits", None)
            if limits_str is None:
                if joint_type != "fixed":
                    warning(f"Joint {joint_type} has no .limits defined, using defaults")
                limits = parse_bracket(JOINT_DEFAULT_LIMITS)
            else:
                limits = parse_bracket(limits_str)

            vel_limit_str = block.fields.get("vel_limit", None)
            if vel_limit_str is None:
                warning(f"Joint has no .vel_limit defined, using defaults")
                vel_limit = safe_eval(JOINT_DEFAULT_VELOCITY_LIMIT)
            else:
                vel_limit = safe_eval(vel_limit_str)

            home_str = block.fields.get("home", None)
            if home_str is None:
                if joint_type != "fixed":
                    warning(f"Joint {joint_type} has no .home defined, using default 0.0")
                home = safe_eval(JOINT_DEFAULT_HOME)
            else:
                home = safe_eval(home_str)

            init_str = block.fields.get("init", None)
            if init_str is None:
                if joint_type != "fixed":
                    warning(f"Joint {joint_type} has no .init defined, using default 0.0")
                initial = safe_eval(JOINT_DEFAULT_INIT)
            else:
                initial = safe_eval(init_str)
            
            if parent_id == child_id:
                param_error(f"Joint has same parent and child link '{parent_name}' - joints must connect different links")
            if len(limits) != 2:
                param_error(f"Joint .limits must have 2 values '[min, max]', found {len(limits)}")
            if limits[0] > limits[1]:
                param_error(f"Joint min limit ({limits[0]}) > max limit ({limits[1]})")
            if len(axis) != 3:
                param_error(f"Joint .axis must have 3 values [x, y, z], found {len(axis)}")
            if abs(1 - sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)) > JOINT_AXIS_THRESHOLD:
                axis_norm = sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
                param_error(f"Joint .axis must be normalized (unit length), current length = {axis_norm:.6f}")
            if not ((limits[0] <= initial - home) and (initial - home <= limits[1])):
                param_error(f"Joint initial value ({initial}) relative to home ({home}) = {initial - home} is outside limits [{limits[0]}, {limits[1]}]")
            
            # --- Debug Prints ---
            debug(f"> === Joint-{len(joints)} | {robot_name} ===")
            debug(f"    —— links: [{parent_name}({parent_id}), {child_name}({child_id})]")
            debug(f"    —— type: {joint_type}")
            debug(f"    —— init: {initial:2.4f}")
            debug(f"    —— home: {home:2.4f}")
            debug(f"    —— limits: [{limits[0]:2.4f}, {limits[1]:2.4f}]")
            debug(f"    —— vel_limit: {vel_limit:2.4f}")
            debug(f"    —— axis: <{axis[0]:2.4f}, {axis[1]:2.4f}, {axis[2]:2.4f}>")
            debug("     —— origin: {")
            debug(f"    \txyz: <{origin_xyz[0]:2.4f}, {origin_xyz[1]:2.4f}, {origin_xyz[2]:2.4f}>")
            debug(f"    \trpy: <{origin_rpy[0]:2.4f}, {origin_rpy[1]:2.4f}, {origin_rpy[2]:2.4f}>")
            debug("      }")
            debug(f"    —— comp_type: none")
            debug(f"    —— comp_index: -1")
            debug(f"")

            joints.append(Joint(
                id=len(joints),
                parent_id=parent_id,
                child_id=child_id,
                type=joint_type,
                axis=axis,
                limits=limits,
                vel_limit=vel_limit,
                init=initial,
                home=home,
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                comp_type="none",
                comp_index=-1
            ))
    
    # === Frames === 
    for block in blocks:
        if block.type != "frame":
            continue
        
        LINE_NO = block.line_no
        frame_name = block.subtype
        if frame_name == "" or frame_name == "{":
            param_error("Frame declaration missing name - use format '\\frame name {'")

        frame_link_name_str = block.fields.get("link", None)
        if frame_link_name_str is None:
            warning(f"Frame '{frame_name}' has no .link defined, attaching to last link '{links[-1].name}'")
            frame_link_name = links[-1].name
        else:
            frame_link_name = frame_link_name_str
        
        if frame_link_name not in name_to_id:
            param_error(f"Frame '{frame_name}' references unknown link '{frame_link_name}'")

        frame_id = len(frames)
        frame_link_id = name_to_id[frame_link_name]
        
        if "origin" in block.subblocks:
            try:
                line_nums = block.subblock_lines_with_nums.get("origin", [])
                frame_xyz, frame_rpy = parse_transform_block(block.subblocks["origin"], line_nums)
            except Exception as e:
                error(f"Failed to parse .origin block for frame '{frame_name}': {str(e)}")
        else:
            warning(f"Frame '{frame_name}' has no .origin block defined, using default values")
            frame_xyz = FRAME_DEFAULT_ORIGIN_XYZ
            frame_rpy = FRAME_DEFAULT_ORIGIN_RPY

        if len(frame_name) > 31:
            param_error(f"Frame name '{frame_name}' exceeds maximum length of 31 characters")
        if not re.match(r"[A-Za-z_][A-Za-z0-9_]*$", frame_name):
            param_error(f"Frame name '{frame_name}' must be a valid C identifier (letters, numbers, underscore; cannot start with number)")
        for f in frames:
            if f.name == frame_name:
                param_error(f"Duplicate frame name '{frame_name}' - all frame names must be unique")

        frames.append(Frame(
            id=frame_id,
            link_id=frame_link_id,
            name=frame_name,
            origin_xyz=frame_xyz,
            origin_rpy=frame_rpy,
        ))

    return robot_name, links, joints, frames

# ... [rest of generate_code, generate_header, and main remain the same] ...

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
    code += f"inline const std::array<Link, {L}> {name}_links = {{\n"
    for link in links:
        code += f"  Link({link.id}, \"{link.name}\", {link.mass},\n"
        code += f"    cobalt::math::linear_algebra::Matrix<3,3>({{{{ {link.inertia[0][0]}, {link.inertia[1][0]}, {link.inertia[2][0]} }},\n"
        code += f"                                                 {{ {link.inertia[0][1]}, {link.inertia[1][1]}, {link.inertia[2][1]} }},\n"
        code += f"                                                 {{ {link.inertia[0][2]}, {link.inertia[1][2]}, {link.inertia[2][2]} }}}}),\n"
        code += f"    cobalt::math::geometry::Transform<>::eye().rotateZ({link.com_rpy[2]}).rotateY({link.com_rpy[1]}).rotateX({link.com_rpy[0]})\n"
        code += f"                                              .translate(cobalt::math::linear_algebra::Vector<3>({link.com_xyz[0]}, {link.com_xyz[1]}, {link.com_xyz[2]})),\n"
        code += f"    {str(link.virtual).lower()}),\n"
    code += f"}};\n\n"

    # ========== Joints ==========
    code += f"// ===== Joints =====\n"
    code += f"inline const std::array<Joint, {J}> {name}_joints = {{\n"
    for joint in joints:
        limitsEnabled = (joint.type != "fixed")

        code += f"        Joint({joint.id}, {joint.parent_id}, {joint.child_id}, JointType::{joint.type.capitalize()},\n"
        code += f"            cobalt::math::geometry::Transform<>::eye().translate(cobalt::math::linear_algebra::Vector<3>({joint.origin_xyz[0]}, {joint.origin_xyz[1]}, {joint.origin_xyz[2]}))\n"
        code += f"                                                      .rotateZ({joint.origin_rpy[2]}).rotateY({joint.origin_rpy[1]}).rotateX({joint.origin_rpy[0]}),\n"
        code += f"             cobalt::math::linear_algebra::Vector<3>({float(joint.axis[0])}, {float(joint.axis[1])}, {float(joint.axis[2])}),\n"
        code += f"            JointLimits{{ {float(joint.limits[0])}, {float(joint.limits[1])}, {str(limitsEnabled).lower()} }},\n"
        code += f"            JointVelocityLimit{{ {float(joint.vel_limit)}, {str(limitsEnabled).lower()} }},\n"
        code += f"             {float(joint.home)},\n"
        code += f"             CompoundJointType::{joint.comp_type.capitalize()},\n"
        code += f"             (cidx_t){int(joint.comp_index)}),\n"
    code += f"}};\n\n"

    # ========== Frames ==========
    code += f"// ===== Frames =====\n"
    code += f"inline const std::array<FrameAttachment, {F}> {name}_frames = {{\n"
    for frame in frames:
        code += f"    FrameAttachment({frame.id}, {frame.link_id}, \"{frame.name}\",\n"
        code += f"                    cobalt::math::geometry::Transform<>::eye().rotateZ({frame.origin_rpy[2]}).rotateY({frame.origin_rpy[1]}).rotateX({frame.origin_rpy[0]})\n"
        code += f"                                                              .translate(cobalt::math::linear_algebra::Vector<3>({frame.origin_xyz[0]}, {frame.origin_xyz[1]}, {frame.origin_xyz[2]}))),\n"
    code += f"}};\n\n"

    # ========== MAKE MODEL ==========
    code += f"// ===== RobotModel =====\n"
    code += f"inline const RobotModel<{L}, {J}, {F}> {name}_model(\"{name}\", {name}_links, {name}_joints, {name}_frames);\n"

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
    code += f"      cobalt::math::linear_algebra::Matrix<6,{J}>::zero(),    // J\n"
    code += f"      {{}},    // linkTransforms\n"
    code += f"      {{}},    // jointTransforms\n"
    code += f"      {{}},    // frameTransforms\n"
    code += f"      false,  // validJ\n"
    code += f"      false,  // validLinks\n"
    code += f"      false,  // validJoints\n"
    code += f"      false   // validFrames\n"
    code += f"  }};\n\n"

    # ========== MAKE ROBOT ==========
    code += f"  // ===== Robot =====\n"
    code += f"      Robot<{L}, {J}, {F}> {name}({name}_internals::{name}_model, {name}_state);\n"
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

    log(f" Parsed[{COMPLETE_COUNT+1}]:  {len(links)}-{"link" if len(links) == 1 else "links"}, {len(joints)}-{"joint" if len(joints) == 1 else "joints"}, {len(frames)}-{"frame" if len(frames) == 1 else "frames"}")
    success(f"[{COMPLETE_COUNT+1}/{FILE_COUNT}] Generated {name}.hpp robot header: [{out_path.replace("\\", "/")}]")

# ========== MAIN ==========
if __name__ == "__main__":
    files = []

    VERBOSE = "-v" in sys.argv or "--verbose" in sys.argv
    DEBUG = "-d" in sys.argv or "--debug" in sys.argv
    WARNING = "-w" in sys.argv or "--warning" in sys.argv
    sys.argv = [arg for arg in sys.argv if arg not in ("", "-v", "--verbose", "-d", "--debug", "-w", "--warning")]

    if len(sys.argv) > 1:   # Called with arguments
        in_file = sys.argv[1]
        files.append(in_file)

        if all(arg.endswith(".rob") for arg in sys.argv[1:-1]): # Called with both multiple inputs & output argument
            files = sys.argv[1:-1]
            out_dir = sys.argv[-1]

            startLog = f"ROB-Dir: {os.path.dirname(files[0])}"
            log(f"{"=" * (len(startLog) + 6)}");
            log("   " + startLog + "   ")
            log(f"{"=" * (len(startLog) + 6)}");
        else:

            if len(sys.argv) == 2:      # Called with only input argument
                out_dir = ""
            elif len(sys.argv) == 3:    # Called with both input & output argument
                out_dir = sys.argv[2]                   

    else:                   # Called with no arguments
        project_root = os.environ.get("PROJECT_DIR")

        if project_root is None:
            # Fallback
            script_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.abspath(os.path.join(script_dir, "../../.."))

        robots_dir = os.path.join(project_root, "robots")
        out_dir = os.path.join(project_root, "generated")
        
        for entry in os.listdir(robots_dir):
            full_path = os.path.join(robots_dir, entry)
            if os.path.isfile(full_path):
                files.append(full_path)

        startLog = f"ROB-Dir: {robots_dir}"
        log(f"{"=" * (len(startLog) + 6)}");
        log("   " + startLog + "   ")
        log(f"{"=" * (len(startLog) + 6)}");

    FILE_COUNT = len(files)
    COMPLETE_COUNT = 0
    for file in files:
        log(f"Parsing[{COMPLETE_COUNT+1}] - {os.path.basename(file)}")
        [name, links, joints, frames] = parse_rob_file(file)

        code = generate_code(name, links, joints, frames)
        generate_header(name, code, out_dir)
        COMPLETE_COUNT += 1

    endLog = f"Parsing complete for all '.rob' files. Generated files are under {out_dir}"
    log(f"{"=" * (len(endLog) + 6)}");
    log("   " + endLog + "   ")
    log(f"{"=" * (len(endLog) + 6)}");