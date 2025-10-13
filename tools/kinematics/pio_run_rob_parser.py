import os
import sys
import subprocess

from pathlib import Path

Import("env")

# ===== Project Root =====
project_dir = Path(env["PROJECT_DIR"]).resolve()
cobalt_root = os.path.join(project_dir, "lib", "Cobalt")
kinematics_path = os.path.join(cobalt_root, "tools", "kinematics")
parser_path = os.path.join(kinematics_path, "rob_parser.py")

sys.path.insert(0, kinematics_path)

if not os.path.exists(parser_path):
    print(f"[ ERROR ] | Couldn't find parser at {parser_path}")
    env.Exit(1)
else:
    print(f"Running robot parser from {parser_path}")

result = subprocess.run(
    [sys.executable, parser_path],
    cwd=kinematics_path
)

if result.returncode != 0:
    print("[ ERROR ] | rob_parser Filed")
    env.Exit(result.returncode)
else:
    print("[ SUCCESS ] | Robot file headers generated successfully")