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
JOINT_DEFAULT_AXIS = "[0.0, 0.0, 1.0]"
JOINT_DEFAULT_LIMITS = "[-pi, pi]"
JOINT_DEFAULT_HOME = "0.0"
JOINT_DEFAULT_INIT = "0.0"

JOINT_DEFAULT_ORIGIN_XYZ = [0.0, 0.0, 0.0]
JOINT_DEFAULT_ORIGIN_RPY = [0.0, 0.0, 0.0]

JOINT_TYPE_ALIASES = {
    "fixed":     "fixed",
    "f":         "fixed",

    "revolute":  "revolute",
    "r":         "revolute",

    "prismatic": "prismatic",
    "p":         "prismatic",

    "universal": "universal",
    "u":         "universal",

    "spherical": "spherical",
    "s":         "spherical",

    "cylinderical": "cylinderical",
    "c":            "cylinderical",

    "planar":       "planar",
    "e":            "planar"
}

JOINT_AXIS_THRESHOLD = 0.0001

## Universal Joint
UNIVERSAL_DEFAULT_AXES = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
UNIVERSAL_DEFAULT_LIMITS = [[-pi, pi], [-pi, pi]]
UNIVERSAL_DEFAULT_INIT = [0.0, 0.0]
UNIVERSAL_DEFAULT_HOME= [0.0, 0.0]

## Spherical Joint
SPHERICAL_DEFAULT_AXES = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]
SPHERICAL_DEFAULT_LIMITS = [[-pi, pi], [-pi, pi], [-pi, pi]]
SPHERICAL_DEFAULT_INIT = [0.0, 0.0, 0.0]
SPHERICAL_DEFAULT_HOME= [0.0, 0.0, 0.0]

## Cylinderical Joint
CYLINDERICAL_DEFAULT_AXIS = [1.0, 0.0, 0.0]
CYLINDERICAL_DEFAULT_LIMITS = [[-pi, pi], [-1, 1]]
CYLINDERICAL_DEFAULT_INIT = [0.0, 0.0]
CYLINDERICAL_DEFAULT_HOME= [0.0, 0.0]

## Planar Joint
PLANAR_DEFAULT_AXES = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
PLANAR_DEFAULT_LIMITS = [[-1, 1], [-1, 1], [-pi, pi]]
PLANAR_DEFAULT_INIT = [0.0, 0.0, 0.0]
PLANAR_DEFAULT_HOME= [0.0, 0.0, 0.0]

# === FRAME ===
FRAME_DEFAULT_ORIGIN_XYZ = [0.0, 0.0, 0.0]
FRAME_DEFAULT_ORIGIN_RPY = [0.0, 0.0, 0.0]