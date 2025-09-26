import sys
import math

def parse_file(filename):
    # File type check for '.rob'
    file_parts = filename.strip().split(".")
    print(file_parts[1])
    if(file_parts[1] != "rob"):
        raise ValueError("[ ERROR ] | Invalid file type. Use '.rob' config files")

    rob_name = ""
    link = []
    joint = []

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()

            # Comments
            if not line or line.startswith("#"):
                continue

            # Robot Name
            if line.startswith(">"):
                line = line[1:].strip()
                rob_name = line

            # Robot Link & Joints
            if line.startswith("."):
                parts = line[1:].strip().split()

                if(parts[0] == "link"): # Handle Links
                    name = parts[1]

                    # Name duplicate check
                    for l in link:
                        if(l[0] == name):
                            raise ValueError("[ ERROR ] | Duplicate Link Names")
                    
                    if(name == "base"):
                        length = float(0)
                        mass = float(0)
                    else:
                        length = float(parts[2])
                        if(len(parts) == 4):
                            mass = float(parts[3])
                        else:
                            mass = float(0)

                    if(len(parts) > 4):
                        raise ValueError("[ ERROR ] | Too many arguments")
                    
                    link.append((name, length, mass))
                
                if(parts[0] == "joint"): # Handle Joints
                    type = parts[1]
                    parent, child = parts[2], parts[3]

                    if(len(parts) == 5):
                        raise ValueError("[ ERROR ] | Not enough parameters")

                    if(len(parts) >= 6):
                        min_val, max_val = float(parts[4]), float(parts[5])
                    else:
                        min_val, max_val = -math.pi, math.pi

                    if(len(parts) == 9):
                        axis = [float(s) for s in parts[6:9]]
                    else:
                        axis = [float(0), float(0), float(1)]
                    
                    if(len(parts) > 9):
                        raise ValueError("[ ERROR ] | Too many arguments")
                    
                    joint.append((type, parent, child, min_val, max_val, axis))

    print(rob_name)
    print(link)
    print(joint)

if __name__ == "__main__":
    in_file = sys.argv[1]
    parse_file(in_file)
