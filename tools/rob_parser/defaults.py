from math import pi

# === LINK ===
LINK_DEFAULT_MASS = "0.0"
LINK_DEFAULT_INERTIA = """[
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
]"""

LINK_DEFAULT_COM_XYZ = [0.0, 0.0, 0.0]
LINK_DEFAULT_COM_RPY = [0.0, 0.0, 0.0]

# === JOINT ===
JOINT_DEFAULT_AXIS = [0.0, 0.0, 1.0]
JOINT_DEFAULT_LIMITS = [-pi, pi]
JOINT_DEFAULT_HOME = 0.0
JOINT_DEFAULT_INIT = 0.0

JOINT_DEFAULT_ORIGIN_XYZ = [0.0, 0.0, 0.0]
JOINT_DEFAULT_ORIGIN_RPY = [0.0, 0.0, 0.0]

JOINT_TYPE_ALIASES = {
    "fixed":     "fixed",
    "f":         "fixed",

    "revolute":  "revolute",
    "r":         "revolute",

    "prismatic": "prismatic",
    "p":         "prismatic",
}

JOINT_AXIS_THRESHOLD = 0.0001

# === FRAME ===
FRAME_DEFAULT_ORIGIN_XYZ = [0.0, 0.0, 0.0]
FRAME_DEFAULT_ORIGIN_RPY = [0.0, 0.0, 0.0]